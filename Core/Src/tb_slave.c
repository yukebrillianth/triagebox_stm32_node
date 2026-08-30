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
 * before the ESP32 finishes booting and gives up on the bus.
 *
 * 200 ms, not the 1000 ms this used to be, because 1000 ms loses the race it
 * exists to win. The ESP32 reaches its GT911 read -- the first thing that dies
 * on a held bus -- 1333 ms after its own reset, measured. A wedge that starts
 * when the ESP32 resets mid-transfer therefore had 333 ms of margin, and the
 * superloop can be delayed by the 497.5 Hz ADC ISR and by a LoRa transmit, so
 * that margin was routinely gone. At 200 ms the bus is free ~1.1 s before the
 * ESP32 looks at it. The floor is a whole transaction: 13 ms. */
#define TB_SLAVE_STUCK_MS 200U

/* Last tick at which the slave was armed and listening. */
static uint32_t s_healthy_ms = 0;

/* mon_i2c_reads + mon_i2c_writes as of the previous tb_slave_service() call.
 * A completed transfer is the one piece of evidence no register can fake, and it
 * is what keeps the watchdog from firing on sampling luck -- see the top of
 * tb_slave_service(). */
static uint32_t s_served_prev = 0;

/*
 * Are we holding a line low? Read straight from the pads -- IDR reflects the pin
 * even while it is in alternate-function mode, so this costs two loads and
 * disturbs nothing.
 *
 * BOTH lines are tested, and they mean different wedges:
 *
 *   SCL (PB10) low -- we are clock-stretching, stuck mid-transaction.
 *   SDA (PB3)  low -- we are mid-byte with an ACK or a data bit on the wire. The
 *                     slave drives SDA low during an ACK, and if the master is
 *                     reset in that window it can come back with the bus looking
 *                     busy and the slave parked exactly there. This case is not
 *                     hypothetical: with the ESP32 side alive and its console
 *                     silent, IDR read 0xF4D5 (SDA low, SCL high) and holding the
 *                     STM32 in reset released it to 0x74D9.
 *
 * The earlier version of this checked SCL only, on the theory that SDA is the
 * master's problem to clock out. The master cannot clock it out: the ESP32's
 * own bus-recovery bails the moment it sees SCL low, so a wedged slave that
 * stretches while ALSO sitting on SDA is never recovered by either side.
 *
 * Deliberately no port bit-bang here: in AF mode the pad is an input to the I2C
 * peripheral's open-drain driver, so reading it cannot disturb the bus.
 */
static bool tb_slave_bus_released(void)
{
    return HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10) == GPIO_PIN_SET
        && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3) == GPIO_PIN_SET;
}

/*
 * Is the peripheral actually able to answer, as opposed to merely believing it
 * is? PE must be set and the event interrupt must be enabled, because a slave
 * with ACK on and ITEVTEN off is the worst of both: the hardware ACKs its
 * address, then stretches SCL waiting for an ISR that can never run.
 *
 * This is not defensive programming, it is a measured failure. After ~50 s of
 * healthy traffic the link stopped for good, and the register dump was:
 *
 *   CR1=0x0401  PE=1, ACK=1          -- enabled
 *   OAR1=0x4084 address 0x42          -- correct
 *   CR2=0x0032  ITEVTEN=0, ITERREN=0  -- DEAF
 *   SR1=0x0100  BERR                  -- an error got us here
 *
 * while HAL_I2C_GetState() still said LISTEN and mon_i2c_recoveries stayed
 * frozen. The HAL's I2C_ITError() disables EVT/BUF/ERR interrupts, and the
 * re-arm in HAL_I2C_ErrorCallback can fail without changing State, so State is
 * not evidence. CR2 is.
 */
static bool tb_slave_armed_in_hw(void)
{
    return (hi2c2.Instance->CR1 & I2C_CR1_PE) != 0U
        && (hi2c2.Instance->CR2 & I2C_CR2_ITEVTEN) != 0U;
}

/*
 * Is SR2's BUSY flag latched on? THIS IS THE ONE THAT ACTUALLY KILLS THE LINK,
 * and every test above is blind to it.
 *
 * Measured with the ESP32 polling at 20 Hz: 21 reads/s for 22 s, then reads stop
 * dead and never resume, while the slave looks perfect --
 *
 *   CR1=0x0401  PE=1, ACK=1             enabled
 *   CR2=0x0332  ITEVTEN=1               armed
 *   SR1=0x0000  no error at all
 *   IDR         both lines high         bus released
 *   SR2=0x0002  BUSY                    <-- the only bit that moved
 *
 * BUSY set means the peripheral saw a START and never the matching STOP, so it
 * will not recognise the next one. It stops answering silently: no error flag, no
 * state change, nothing to find it by. HAL_I2C_GetState() reads LISTEN the whole
 * time, which is exactly why the state test refreshed s_healthy_ms forever and
 * the watchdog never fired -- `stuck` stayed under 10 ms for the 27 s the link
 * was dead. The cure is confirmed as well: forcing PE=0 made the recovery below
 * run, and BUSY went 0x0002 -> 0x0000.
 *
 * BUSY is also set legitimately on every transaction, ours and every other
 * master's on this shared bus, which is why it can only count as a fault when it
 * lasts TB_SLAVE_STUCK_MS with not one completed transfer in that whole window.
 */
static bool tb_slave_busy_latched(void)
{
    return (hi2c2.Instance->SR2 & I2C_SR2_BUSY) != 0U;
}

/*
 * Both lines on the floor at once is NOT our wedge, and recovering on it is
 * actively harmful.
 *
 * A stretching slave holds SCL and leaves SDA to the pull-up; a slave caught
 * mid-ACK holds SDA while the master drives SCL. One line, never both. Both low
 * means nothing is pulling the bus up at all -- the ESP32 board is unpowered, and
 * its 3V3 rail is where the pull-ups live -- and re-initialising I2C2 cannot
 * conjure a pull-up.
 *
 * Measured: with the ESP32 board off, IDR read both pins low and this test's
 * absence cost 2662 recoveries in 600 s, 4.4 every single second. Each one is a
 * DeInit/Init that drops PB10/PB3 to analog on the way through, so if the master
 * IS alive and mid-transfer, the slave vanishes off the bus in the middle of a
 * byte. That is a fault this function invents rather than fixes.
 */
static bool tb_slave_bus_dead(void)
{
    return HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10) == GPIO_PIN_RESET
        && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3) == GPIO_PIN_RESET;
}

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

/* The ESP32's fuel-gauge reading, which this board cannot take itself. 0xFF
 * until it writes one, and 0xFF is "no reading" rather than 0% -- see
 * TB_REG_HOST_BATTERY. Not a "take": this is state like the button bitmask, so a
 * read that misses an update costs nothing and there is nothing to drain. */
static volatile uint8_t s_host_battery = 0xFFU;

static void arm_receive(void)
{
    /* One byte at a time. It costs an interrupt per byte, but each one is a
     * couple of stores, and it avoids reconstructing "how many bytes actually
     * arrived" from XferSize/XferCount when the master stops early -- which is
     * exactly the case that happens on every pointer write. */
    (void)HAL_I2C_Slave_Seq_Receive_IT(&hi2c2, (uint8_t *)&s_rx, 1U,
                                      I2C_NEXT_FRAME);
}

/*
 * Swap the analog noise filter for the digital one. THIS IS THE FIX FOR THE
 * LATCHED BUSY FLAG, not a tuning knob -- call it after every HAL_I2C_Init().
 *
 * F411 erratum 2.8.7, "I2C analog filter may provide wrong value, locking BUSY
 * flag": a glitch on SDA or SCL can make the analog filter hand the state machine
 * a wrong level, and SR2.BUSY then latches set with no way to clear it from
 * software. ST's own workaround is a software reset of the peripheral; the erratum
 * also notes the analog filter is what produces the wrong value.
 *
 * Which matches what this bus does, measured: BUSY set with SR1 clean, both pads
 * released, the slave silently refusing to answer, and it survived 394
 * DeInit/Init recoveries and a CR1.SWRST -- 75 s with the link down and not one
 * transfer completing. ANOFF removes the mechanism instead of cleaning up after
 * it, and DNF=1 keeps a filter: one I2C clock period of digital debounce, which
 * on this 100 kHz link is 20 ns of spike rejection and is not subject to the
 * erratum.
 *
 * FLTR is writable only with PE=0 (RM0383 18.6.9), and HAL_I2C_Init() leaves PE
 * set, so the enable has to be bracketed here rather than folded into MspInit.
 */
static void tb_slave_use_digital_filter(void)
{
    __HAL_I2C_DISABLE(&hi2c2);
    hi2c2.Instance->FLTR = I2C_FLTR_ANOFF | 1U;
    __HAL_I2C_ENABLE(&hi2c2);
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
    tb_slave_use_digital_filter();

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
 * Healthy means all four together: HAL state LISTEN -- armed, between
 * transactions -- AND the event interrupt actually enabled AND both bus lines
 * released AND SR2's BUSY flag clear. Anything else that PERSISTS for
 * TB_SLAVE_STUCK_MS with no transfer completing is a wedge: stuck mid-transfer
 * (BUSY_TX_LISTEN / BUSY_RX_LISTEN), listening lost entirely (READY) because a
 * re-arm failed on an error path, armed-but-still-sitting-on-a-line, or armed and
 * idle and silently deaf with BUSY latched (below). Recovery resets the
 * peripheral and re-arms; MspDeInit puts PB10/PB3 back to analog, which releases
 * both lines. The published snapshot is untouched.
 *
 * This matters because I2C2's pins are the ESP32's ONLY I2C bus, shared with the
 * GT911 touch controller, the TCA9554 display expander and the SW6106 PMIC. One
 * wedged slave takes all three down. On SCL it shows up as a white panel that
 * never finishes booting; on SDA the ESP32 boots fine and then dies a few seconds
 * later in a storm of "I2C bus is still busy but software timeout detected",
 * because a START needs SDA to fall and it is already on the floor.
 *
 * WHY THE PADS ARE TESTED AND NOT JUST THE STATE. On a transmit under-run -- the
 * master clocks past the armed length, or vanishes in the window just after the
 * last armed byte reaches DR -- I2C_SlaveTransmit_TXE() has already set State
 * back to LISTEN. The wedge then looks armed and idle while a line is held down,
 * so testing the state alone refreshes s_healthy_ms on every pass and recovers
 * nothing. Measured: mon_i2c_recoveries sat at 6 while only 14% of the ESP32's
 * polls were completing, s_healthy_ms tracked uwTick the whole time, and IDR read
 * SDA low / SCL high for as long as it was sampled. Holding the STM32 in reset
 * released SDA, which is what proves the line was ours.
 *
 * SR2's BUSY flag IS part of the test, and getting there took two wrong turns.
 * It tracks the bus rather than us, so it sets whenever the ESP32 talks to the
 * touch controller -- testing it naively re-inits a healthy slave because of
 * someone else's traffic. But BUSY latched with no STOP is also the ONE failure
 * that actually took the link down in practice, and nothing else in the register
 * file shows it. Both are true, so the resolution is not to drop the test but to
 * require it to LAST: see the completed-transfer gate at the top of the function.
 * The pads have the same exposure and the same answer.
 *
 * ponytail: only fixes the case where WE hold the bus. If the ESP32 side holds a
 * line low the STM32 cannot clear it -- that needs the nine-clock master recovery
 * pulse with PB10 driven as GPIO. Add it if mon_i2c_recoveries climbs but the
 * panel stays white.
 */
void tb_slave_service(void)
{
    uint32_t now = HAL_GetTick();
    uint32_t served = mon_i2c_reads + mon_i2c_writes;

    /*
     * A transfer completed since the last call, so whatever the registers say
     * right now, this slave is answering the master. This gate is what lets the
     * tests above be strict without being trigger-happy: every one of them --
     * state, ITEVTEN, the pads, BUSY -- reads "faulty" for a few microseconds
     * during any normal transaction, and the poll task runs 20 of those a second.
     * Sampling luck used to be the difference between a healthy board and 4.4
     * destructive re-inits per second.
     */
    if (served != s_served_prev) {
        s_served_prev = served;
        s_healthy_ms = now;
        return;
    }

    /* Nobody is pulling the bus up: not our fault and not ours to fix. Keep the
     * clock running so a wedge is not measured across the ESP32's downtime. */
    if (tb_slave_bus_dead()) {
        s_healthy_ms = now;
        return;
    }

    if ((HAL_I2C_GetState(&hi2c2) == HAL_I2C_STATE_LISTEN)
        && tb_slave_armed_in_hw()
        && tb_slave_bus_released()
        && !tb_slave_busy_latched()) {
        s_healthy_ms = now;
        return;
    }
    if ((now - s_healthy_ms) < TB_SLAVE_STUCK_MS) {
        return; /* mid-transaction; let it finish normally */
    }

    ++mon_i2c_recoveries;
    s_healthy_ms = now;

    /*
     * SWRST first, and DeInit alone is NOT enough -- this is the difference
     * between a recovery that logs and a recovery that works.
     *
     * Measured: with only the DeInit/Init pair below, a latched SR2.BUSY survived
     * 394 consecutive recoveries over 75 s and the link never came back. The
     * reason is timing, not thoroughness: the ESP32 retries every ~150 ms, so
     * HAL_I2C_Init() re-enables a fresh peripheral straight into somebody else's
     * in-flight transaction, and it latches BUSY again on the START it was never
     * present for. DeInit clears PE, which releases the pads, but PE alone does
     * not reset the state machine that owns BUSY.
     *
     * SWRST does: RM0383 says the peripheral is held under reset while the bit is
     * set, which clears BUSY and releases SCL and SDA. It is also the only way out
     * of the stuck-BUSY condition ST documents. Two writes, no delay loop needed --
     * the bus is APB1 and the write has landed before the next instruction reads it.
     */
    hi2c2.Instance->CR1 |= I2C_CR1_SWRST;
    hi2c2.Instance->CR1 &= ~I2C_CR1_SWRST;

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
    tb_slave_use_digital_filter();
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

/*
 * See tb_slave.h for why this is a setter and not another publish parameter.
 * No critical section: a single byte store into the staging copy the superloop
 * owns, and the ISR only ever reads s_live.
 */
void tb_slave_set_rssi(int8_t dbm)
{
    s_stage.lora_rssi = dbm;
}

void tb_slave_ppg_push(float ir, float red)
{    /* All of it is in tb_regs.h, next to the layout it has to agree with, and
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

uint8_t tb_slave_host_battery(void)
{
    return s_host_battery;
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
        case TB_REG_HOST_BATTERY:
            /* Taken as-is, 0xFF included: that value is the ESP32 saying it
             * could not read the gauge, and passing it through unchanged is what
             * makes the station omit the key instead of reporting a flat pack. */
            s_host_battery = byte;
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

    /*
     * Re-arm, and the state has to be forced first or the re-arm is a no-op.
     * THIS IS THE BUG THAT KILLED THE LINK EVERY TIME, and it is two lines of
     * HAL contract nobody reads:
     *
     *   I2C_ITError()            -- hi2c->State = HAL_I2C_STATE_LISTEN  (a slave
     *                               that was listening is put back in LISTEN)
     *   HAL_I2C_EnableListen_IT() -- returns HAL_BUSY unless State is READY, and
     *                               touches nothing on the way out
     *
     * So after any error that goes through I2C_ITError -- a BERR, an overrun, a
     * misplaced START from noise -- EVT and ERR interrupts are off, State says
     * LISTEN, and every re-arm from here silently fails. The slave then ACKs its
     * address in hardware and stretches SCL forever waiting for an ISR that is
     * disabled. On the shared display bus that takes the GT911 down with it, and
     * the panel freezes.
     *
     * Measured, before this: the link ran 6-28 s from boot, then stopped for good
     * with CR2=0x0032 (ITEVTEN=0, ITERREN=0) while HAL_I2C_GetState() still
     * reported LISTEN. AF is exempt because the HAL routes acknowledge-failure
     * through I2C_Slave_AF() and ListenCpltCallback instead, which is why reads
     * kept working right up to the first real error.
     */
    if (hi2c->State == HAL_I2C_STATE_LISTEN) {
        hi2c->State = HAL_I2C_STATE_READY;
    }
    (void)HAL_I2C_EnableListen_IT(hi2c);
}
