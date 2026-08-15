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
 * Publish a consistent snapshot for the master to read. Call from the superloop
 * with the latest values; cheap (a struct fill plus a memcpy of ~48 bytes).
 *
 * bp_sys/bp_dia are accepted but must be 0 with TB_FLAG_BP_VALID clear until a
 * BP method exists -- see tb_regs.h.
 */
void tb_slave_publish(uint8_t flags, uint8_t buttons, uint16_t hr,
                      uint16_t spo2, uint16_t rr_x10, uint16_t bp_sys,
                      uint16_t bp_dia, uint8_t battery, uint8_t sensor_ok,
                      const char *rfid, uint8_t rfid_len);

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

/* Diagnostics for CubeMonitor / a status register later. */
extern volatile uint32_t mon_i2c_reads;   /**< completed master reads */
extern volatile uint32_t mon_i2c_writes;  /**< completed master writes */
extern volatile uint32_t mon_i2c_errors;  /**< bus errors, incl. recovered AF */

#endif /* TB_SLAVE_H */
