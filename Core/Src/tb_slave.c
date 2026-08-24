#include "tb_slave.h"

#include <string.h>

#include "main.h"

/*
 * DIRECTION IS BACKWARDS FROM WHAT THE NAME SUGGESTS. In the F4 HAL,
 * HAL_I2C_AddrCallback's TransferDirection describes the MASTER, not us:
 * I2C_DIRECTION_TRANSMIT means the master is transmitting, so WE RECEIVE.
 * (stm32f4xx_hal_i2c.c: I2C_Slave_ADDR sets TRANSMIT when the TRA flag is
 * *reset*.) Getting this inverted gives a slave that answers reads with
 * nothing and is a genuinely nasty afternoon, so the checks below name the
 * intent rather than the constant.
 */
#define TB_MASTER_IS_WRITING(dir) ((dir) == I2C_DIRECTION_TRANSMIT)

volatile uint32_t mon_i2c_reads = 0;
volatile uint32_t mon_i2c_writes = 0;
volatile uint32_t mon_i2c_errors = 0;
volatile uint32_t mon_i2c_recoveries = 0;

/* How long the slave may look wedged before tb_slave_service() re-inits it.
 * Longer than the ESP32's 50 ms poll and than any single transaction (the
 * 132-byte PPG read is ~13 ms at 100 kHz), short enough that a jam clears
 * before the ESP32 finishes booting and gives up on the bus. */
#define TB_SLAVE_STUCK_MS 1000U

/* Last tick at which the slave was armed and listening. */
static uint32_t s_healthy_ms = 0;

/*
 * Double buffer. The superloop fills s_stage; the ISR only ever touches
 * s_live. tb_slave_publish() copies stage->live with the I2C interrupt masked,
 * which is the only critical section in the file and is ~48 bytes long.
 *
 * A single buffer with a "busy" flag would be smaller but would let a publish
 * land between the master's two transactions (pointer write, then data read)
 * and hand back half of one sample set and half of the next.
 */
static tb_snapshot_t s_stage;
static volatile tb_snapshot_t s_live;

/* Latched at the start of each master read so a multi-byte read is coherent
 * even if the superloop publishes mid-transaction. */
static uint8_t s_tx[sizeof(tb_snapshot_t)];

/*
 * PPG waveform ring, written by tb_slave_ppg_push() and latched into s_ppg_tx
 * the same way s_live is. It needs the latch for the same reason: a 132-byte
 * read takes ~12ms at 100kHz, which is long enough for a new sample to land on
 * the slot the master is part-way through reading.
 *
 * volatile is doing real work here, not decoration -- see the store order in
 * tb_slave_ppg_push().
 */
static volatile tb_ppg_block_t s_ppg;
static uint8_t s_ppg_tx[sizeof(tb_ppg_block_t)];

static volatile uint8_t s_ptr;         /* register pointer, auto-increments */
static volatile bool s_ptr_valid;      /* false until the master writes one */
static volatile uint8_t s_rx;          /* one-byte landing pad for writes */

static volatile uint8_t s_cmd = TB_CMD_NONE;
static volatile uint8_t s_priority;
static volatile uint8_t s_confidence;
static volatile bool s_result_fresh = false;

static void arm_receive(void)
{
    /* One byte at a time. It costs an interrupt per byte, but each one is a
     * couple of stores, and it avoids reconstructing "how many bytes actually
     * arrived" from XferSize/XferCount when the master stops early -- which is
     * exactly the case that happens on every pointer write. */
    (void)HAL_I2C_Slave_Seq_Receive_IT(&hi2c2, (uint8_t *)&s_rx, 1U,
                                      I2C_NEXT_FRAME);
}

void tb_slave_init(void)
{
    memset(&s_stage, 0, sizeof(s_stage));
    s_stage.proto_ver = TB_PROTO_VER;
    s_stage.battery = 0xFFU; /* not measured */

    memcpy((void *)&s_live, &s_stage, sizeof(s_stage));
    memcpy(s_tx, &s_stage, sizeof(s_stage));

    memset((void *)&s_ppg, 0, sizeof(s_ppg));
    memset(s_ppg_tx, 0, sizeof(s_ppg_tx));

    s_ptr = 0U;
    s_ptr_valid = false;

    /* CubeMX leaves OwnAddress1 at 0, so set it and re-init. HAL_I2C_Init
     * writes OAR1 from Init, so this is the whole change needed. */
    HAL_I2C_DeInit(&hi2c2);
    hi2c2.Init.OwnAddress1 = TB_I2C_SLAVE_ADDR_HAL;
    hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    /*
     * Clock stretching MUST stay enabled (NoStretch DISABLE). This slave is
     * behind a priority-0 ADC ISR at 497.5 Hz, so an address match can wait a
     * few microseconds before we service it; with stretching off that becomes
     * corrupt data instead of a brief delay.
     */
    hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c2) != HAL_OK) {
        Error_Handler();
    }

    /*
     * NVIC needs no code here: I2C2's global interrupt is ticked in the .ioc, so
     * HAL_I2C_MspInit enables I2C2_EV/ER_IRQn -- and because HAL_I2C_Init calls
     * MspInit, the DeInit/Init pair above leaves them enabled. (MspDeInit does
     * disable them in between; that is fine, nothing is listening yet.)
     *
     * Both sit at preempt priority 0, like every other interrupt in this
     * firmware, so this cannot preempt the 497.5 Hz ADC ISR and can be delayed
     * by it by a few microseconds. That is exactly why the transmit buffer is
     * pre-staged: the delay costs a little clock stretching, never wrong data.
     */

    if (HAL_I2C_EnableListen_IT(&hi2c2) != HAL_OK) {
        Error_Handler();
    }

    s_healthy_ms = HAL_GetTick();
}

/*
 * Watchdog for a wedged slave. Call from the superloop.
 *
 * Healthy means one exact thing: HAL state LISTEN -- armed, between
 * transactions. Anything else that PERSISTS for TB_SLAVE_STUCK_MS is a wedge:
 * stuck mid-transfer (BUSY_TX_LISTEN / BUSY_RX_LISTEN) while holding SCL low, or
 * listening lost entirely (READY) because a re-arm failed on an error path.
 * Recovery is DeInit/Init/EnableListen; MspDeInit puts PB10/PB3 back to analog,
 * which releases both lines. The published snapshot is untouched.
 *
 * This matters because I2C2's pins are the ESP32's ONLY I2C bus, shared with the
 * GT911 touch controller, the TCA9554 display expander and the SW6106 PMIC. A
 * slave holding SCL takes all three down, and it shows up as a white panel with
 * no UI -- nothing on the ESP32 side can clear it.
 *
 * WHAT THIS CANNOT SEE, so never read mon_i2c_recoveries == 0 as "the bus is
 * fine": a transmit under-run, where the master clocks past the armed length.
 * I2C_SlaveTransmit_TXE() sets State back to LISTEN as soon as the last armed
 * byte reaches DR, so the wedge looks armed and idle and this function refreshes
 * s_healthy_ms every pass while SCL is on the floor. There is no state to test
 * for. The only defence is to keep the master's read length equal to the block
 * length; see HAL_I2C_AddrCallback.
 *
 * SR2's BUSY flag is deliberately NOT part of the test: it tracks the bus rather
 * than us, so it sets when the ESP32 talks to the touch controller, and testing
 * it would re-init a perfectly healthy slave because of someone else's traffic.
 *
 * ponytail: only fixes the case where WE hold the bus. If the ESP32 side holds a
 * line low the STM32 cannot clear it -- that needs the nine-clock master recovery
 * pulse with PB10 driven as GPIO. Add it if mon_i2c_recoveries climbs but the
 * panel stays white.
 */
void tb_slave_service(void)
{
    uint32_t now = HAL_GetTick();

    if (HAL_I2C_GetState(&hi2c2) == HAL_I2C_STATE_LISTEN) {
        s_healthy_ms = now;
        return;
    }
    if ((now - s_healthy_ms) < TB_SLAVE_STUCK_MS) {
        return; /* mid-transaction; let it finish normally */
    }

    ++mon_i2c_recoveries;
    s_healthy_ms = now;

    HAL_I2C_DeInit(&hi2c2);
    s_ptr = 0U;
    s_ptr_valid = false;
    if (HAL_I2C_Init(&hi2c2) != HAL_OK) {
        /* Do NOT call Error_Handler() here: it never returns, and an I2C2 left
         * enabled with no ISR stretches SCL forever on the next address match --
         * which is the whole fault this function exists to clear. A dead link is
         * bad; a dead panel is worse. */
        __HAL_I2C_DISABLE(&hi2c2);
        return;
    }
    if (HAL_I2C_EnableListen_IT(&hi2c2) != HAL_OK) {
        __HAL_I2C_DISABLE(&hi2c2);
    }
}

void tb_slave_publish(uint8_t flags, uint8_t buttons, uint16_t hr,
                      uint16_t spo2, uint16_t rr_x10, uint16_t bp_sys,
                      uint16_t bp_dia, uint8_t battery, uint8_t sensor_ok,
                      const char *rfid, uint8_t rfid_len)
{
    s_stage.proto_ver = TB_PROTO_VER;
    ++s_stage.seq; /* wraps; the ESP32 only tests for inequality */
    s_stage.flags = flags;
    s_stage.buttons = buttons;
    s_stage.hr = hr;
    s_stage.spo2 = spo2;
    s_stage.rr_x10 = rr_x10;
    s_stage.bp_sys = bp_sys;
    s_stage.bp_dia = bp_dia;
    s_stage.battery = battery;
    s_stage.sensor_ok = sensor_ok;

    if (rfid_len > TB_RFID_MAX) {
        rfid_len = TB_RFID_MAX;
    }
    s_stage.rfid_len = (rfid == NULL) ? 0U : rfid_len;
    memset(s_stage.rfid, 0, sizeof(s_stage.rfid));
    if ((rfid != NULL) && (rfid_len > 0U)) {
        memcpy(s_stage.rfid, rfid, rfid_len);
    }

    /* Mask only I2C2, not all interrupts: the ADC/DMA ISR must keep running or
     * the ECG window drops samples. */
    HAL_NVIC_DisableIRQ(I2C2_EV_IRQn);
    memcpy((void *)&s_live, &s_stage, sizeof(s_stage));
    HAL_NVIC_EnableIRQ(I2C2_EV_IRQn);
}

void tb_slave_ppg_push(float ir, float red)
{
    /* All of it is in tb_regs.h, next to the layout it has to agree with, and
     * host-tested there -- see tb_ppg_push() for why total is stored last. */
    tb_ppg_push(&s_ppg, ir, red);
}

uint8_t tb_slave_take_cmd(void)
{
    uint8_t cmd = s_cmd;

    if (cmd != TB_CMD_NONE) {
        s_cmd = TB_CMD_NONE;
    }
    return cmd;
}

bool tb_slave_take_result(uint8_t *priority, uint8_t *confidence)
{
    if (!s_result_fresh) {
        return false;
    }
    if (priority != NULL) {
        *priority = s_priority;
    }
    if (confidence != NULL) {
        *confidence = s_confidence;
    }
    s_result_fresh = false;
    return true;
}

/* ---- ISR half: copies bytes, nothing else ------------------------------- */

void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection,
                          uint16_t AddrMatchCode)
{
    (void)AddrMatchCode;

    if (hi2c->Instance != I2C2) {
        return; /* I2C1 (MAX30102) and I2C3 are masters; not ours. */
    }

    if (TB_MASTER_IS_WRITING(TransferDirection)) {
        /* Next byte is the register pointer. */
        s_ptr_valid = false;
        arm_receive();
    } else {
        uint8_t start = s_ptr_valid ? s_ptr : 0U;
        uint8_t *src;
        uint16_t len;

        /* Latch whichever block was addressed. This is the instant that makes a
         * multi-byte read coherent, and only the block actually being read is
         * copied -- the waveform block is 132 bytes and there is no reason to
         * pay for it on a vitals poll. */
        if (start < sizeof(s_tx)) {
            memcpy(s_tx, (const void *)&s_live, sizeof(s_tx));
            src = &s_tx[start];
            len = (uint16_t)(sizeof(s_tx) - start);
        } else if ((start >= TB_REG_PPG_BASE) && (start < TB_REG_PPG_END)) {
            memcpy(s_ppg_tx, (const void *)&s_ppg, sizeof(s_ppg_tx));
            src = &s_ppg_tx[start - TB_REG_PPG_BASE];
            len = (uint16_t)(TB_REG_PPG_END - start);
        } else {
            /* Out-of-range pointer, or the write block, which is not readable.
             * Feed 0xFF rather than leaking adjacent memory -- 0xFF is also
             * what an idle bus looks like, so it reads as "nothing here" on the
             * master's console. */
            static uint8_t pad = 0xFFU;
            (void)HAL_I2C_Slave_Seq_Transmit_IT(hi2c, &pad, 1U, I2C_LAST_FRAME);
            return;
        }

        (void)HAL_I2C_Slave_Seq_Transmit_IT(hi2c, src, len, I2C_LAST_FRAME);
    }
}

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    uint8_t byte;

    if (hi2c->Instance != I2C2) {
        return;
    }

    byte = s_rx;

    if (!s_ptr_valid) {
        s_ptr = byte;
        s_ptr_valid = true;
    } else {
        /* A data byte written to the current pointer. Only the command block
         * is writable; everything else is read-only by design, so a stray
         * write cannot corrupt sensor readings. */
        switch (s_ptr) {
        case TB_REG_CMD:
            s_cmd = byte;
            break;
        case TB_REG_PRIORITY:
            s_priority = byte;
            break;
        case TB_REG_CONFIDENCE:
            s_confidence = byte;
            /* Confidence is written last, so the pair is complete here. */
            s_result_fresh = true;
            break;
        default:
            break;
        }
        if (s_ptr < 0xFFU) {
            ++s_ptr; /* auto-increment, same as the read side */
        }
        ++mon_i2c_writes;
    }

    arm_receive(); /* stay ready; STOP arrives as ListenCplt */
}

void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C2) {
        ++mon_i2c_reads;
    }
}

void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance != I2C2) {
        return;
    }
    /* STOP seen. Drop the pointer so a bare read (no pointer write) starts at
     * register 0 instead of wherever the last transaction happened to end. */
    s_ptr_valid = false;
    (void)HAL_I2C_EnableListen_IT(hi2c);
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    uint32_t err;

    if (hi2c->Instance != I2C2) {
        return;
    }
    err = HAL_I2C_GetError(hi2c);

    /*
     * AF (acknowledge failure) on a slave transmit is NORMAL, not an error:
     * it is how the master says "I have read enough" -- it NACKs the last byte
     * it wants. The F4 HAL surfaces that here anyway. Counting it would make
     * mon_i2c_errors climb once per read and hide real faults.
     */
    if ((err & ~(uint32_t)HAL_I2C_ERROR_AF) != 0U) {
        ++mon_i2c_errors;
    }

    /* Listening always has to be re-armed after any error, AF included, or the
     * slave silently stops answering after the first read. */
    (void)HAL_I2C_EnableListen_IT(hi2c);
}
