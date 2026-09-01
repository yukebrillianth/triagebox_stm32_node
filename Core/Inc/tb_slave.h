#ifndef TB_SLAVE_H
#define TB_SLAVE_H

#include <stdbool.h>
#include <stdint.h>

#include "i2c.h"
#include "tb_regs.h"

/*
 * I2C2 register-map slave for the ESP32-S3 master. Wire contract: tb_regs.h.
 *
 * WHY EVERYTHING IS PRE-STAGED: every peripheral interrupt in this firmware
 * sits at preempt priority 0, so the I2C slave ISR cannot preempt the 497.5 Hz
 * ADC/DMA ISR -- it waits behind it. If the ISR had to go and read sensor
 * state, it would hold SCL low (clock-stretching) for however long that took,
 * and on the ESP32 side that same bus also drives the touch controller and the
 * display expander. A stretched clock there shows up as a frozen touchscreen.
 *
 * So the ISR does exactly one thing: hand out bytes from a buffer that was
 * filled earlier by tb_slave_publish() from the superloop. No sensor reads, no
 * float maths, no HAL calls beyond the I2C ones.
 */

/*
 * Bring up the slave on I2C2 and arm it for the first transaction.
 * Call AFTER MX_I2C2_Init(). Re-initialises the peripheral because the CubeMX
 * init leaves OwnAddress1 at 0.
 */
void tb_slave_init(void);

/*
 * Superloop tick. Re-inits I2C2 if the slave has been wedged -- stuck
 * mid-transfer, or no longer listening -- for a second or more.
 *
 * Not optional politeness: I2C2's pins are the ESP32's *only* I2C bus, shared
 * with the GT911 touch controller, the TCA9554 display expander and the SW6106
 * PMIC. A slave that holds SCL low takes all three with it, and on the ESP32 it
 * shows as a white screen with no UI -- bsp_display_start() fails on the GT911
 * probe before the first LVGL screen is ever loaded. Nothing on the ESP32 side
 * can clear that; only this can.
 */
void tb_slave_service(void);

/*
 * Publish a consistent snapshot for the master to read. Call from the superloop
 * with the latest values; cheap (a struct fill plus a memcpy of ~48 bytes).
 *
 * bp_sys/bp_dia are the ESP32's own ML prediction, taken back off this link with
 * tb_slave_take_bp() and echoed here so every consumer of the read block sees one
 * BP field -- see tb_regs.h. Pass 0/0 with TB_FLAG_BP_VALID clear when no
 * validated pair has arrived.
 */
void tb_slave_publish(uint8_t flags, uint8_t buttons, uint16_t hr,
                      uint16_t spo2, uint16_t rr_x10, uint16_t bp_sys,
                      uint16_t bp_dia, uint8_t battery, uint8_t sensor_ok,
                      const char *rfid, uint8_t rfid_len);

/*
 * Downlink RSSI in dBm: how strongly this node heard the station's last poll.
 * The ESP32 shows it in its status bar, which is what someone walking the box
 * away from the station reads to find the range.
 *
 * A SETTER rather than another tb_slave_publish() parameter, because the two run
 * on different clocks: a poll arrives once per LORA_POLL_PERIOD_MS (15 s) while
 * publish runs every superloop pass. As a parameter, every caller would have to
 * remember the last value and pass it back in, and the one that forgot would
 * publish 0 -- which reads as "no poll heard yet" and would blank the number for
 * most of every cycle.
 *
 * Lands in the staging copy, so the next tb_slave_publish() carries it across.
 * Cheap enough to call from the LoRa path directly: one byte store.
 *
 * CLAMP BEFORE CALLING. LoRa_getRSSI() returns -164 + RegPktRssiValue, so its
 * range is -164..+91, and -164 narrowed to int8_t wraps to +92 -- a value that
 * passes every plausibility test and displays as a very strong signal.
 */
void tb_slave_set_rssi(int8_t dbm);

/*
 * Append one waveform sample -- smoothed PPG in raw MAX30102 counts plus the
 * latest ECG value in raw ADC counts -- to the ring the ESP32 reads at
 * TB_REG_PPG_BASE. Call once per sample at TB_PPG_FS_HZ, from wherever the PPG
 * sample is produced: three packs and three stores, no HAL, no critical section,
 * so it is safe from an ISR as well as the superloop.
 *
 * The three channels must describe the SAME instant as closely as the two
 * sources allow -- the ESP32's BP model measures the delay between the ECG R wave
 * and the finger pulse, so a pairing error is a pressure error. See the push site
 * in main.c for the ~5ms of jitter that costs.
 *
 * Never blocks and never drops on this side: if the ESP32 stops reading, the
 * ring keeps turning and the ESP32 sees the gap in the sample counter. See the
 * ring contract in tb_regs.h.
 */
void tb_slave_wave_push(float ir, float red, uint16_t ecg);

/*
 * Pending command from the master, or TB_CMD_NONE. Reading it clears it, so
 * each command is acted on exactly once. Poll from the superloop.
 */
uint8_t tb_slave_take_cmd(void);

/*
 * Latest triage result written by the ESP32. Returns false if none has arrived
 * since the last call. `priority` is in LoRa order (0=BLACK 1=RED 2=YELLOW
 * 3=GREEN) -- that is what the LoRa packet wants, so pass it through unchanged.
 */
bool tb_slave_take_result(uint8_t *priority, uint8_t *confidence);

/*
 * Latest blood pressure the ESP32's model wrote, in mmHg. Returns false if no
 * NEW validated pair has arrived since the last call, leaving @p sys and @p dia
 * untouched -- so the caller keeps its own last-known copy rather than blanking
 * a standing reading. A pair that failed tb_bp_pair_valid() never gets here; it
 * is counted in mon_bp_writes_rejected instead.
 *
 * A take rather than state, unlike tb_slave_host_battery(), because the caller
 * has to know a value is NEW: TB_FLAG_BP_VALID and the LoRa packet both carry
 * the pair onwards, and "the model has spoken once" is a different fact from
 * "the model spoke this second".
 */
bool tb_slave_take_bp(uint16_t *sys, uint16_t *dia);

/*
 * The ESP32's fuel-gauge percentage, or 0xFF if it has not sent one or could not
 * read the PMIC. NOT a take -- this is state, so call it as often as you like.
 *
 * Substitute it for the STM32's own battery field when publishing: this board has
 * no gauge of its own, and the gauge that exists sits on the ESP32's side of the
 * bus. Pass 0xFF through rather than mapping it to 0; the station omits the JSON
 * key for 0xFF and reports a flat pack for 0.
 */
uint8_t tb_slave_host_battery(void);

/* Diagnostics for CubeMonitor / a status register later. */
extern volatile uint32_t mon_i2c_reads;   /**< completed master reads */
extern volatile uint32_t mon_i2c_writes;  /**< completed master writes */
extern volatile uint32_t mon_i2c_errors;  /**< bus errors, incl. recovered AF */
extern volatile uint32_t mon_i2c_recoveries; /**< wedged-bus re-inits; >0 means
                                                  the bus jammed at least once.
                                                  0 is NOT proof it did not --
                                                  see tb_slave_service(). */
/* BP pairs thrown away by tb_bp_pair_valid(). Climbing means the ESP32's model
 * is predicting values a cuff could not produce, which is a model problem, not a
 * link problem -- the writes themselves are landing, or this would not move. */
extern volatile uint32_t mon_bp_writes_rejected;

#endif /* TB_SLAVE_H */
