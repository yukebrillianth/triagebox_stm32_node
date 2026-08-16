/*
 * Just enough fake HAL to compile rfid_pn532.c on the host.
 *
 * The PN532 frame parser is the one part of that file worth testing off-target:
 * checksum arithmetic, a start-code search over a padded buffer, and length
 * bounds derived from bytes a card supplies. All of it is pure byte-shuffling
 * that the compiler cannot check and hardware checks only one card-tap at a
 * time. The I2C calls are stubbed to fail, which is fine -- the tests drive the
 * parser directly rather than through a transfer.
 *
 * Used only by tools/run_selftests.sh. Included explicitly by the test BEFORE
 * the driver, and it claims main.h's own include guard, so the real
 * Core/Inc/main.h expands to nothing when rfid_pn532.h asks for it. An -I could
 * not do this alone: `#include "main.h"` searches the including file's own
 * directory first, so Core/Inc/main.h always wins on path order.
 */
#ifndef STUB_MAIN_H
#define STUB_MAIN_H

/* Core/Inc/main.h's guard. Defining it here neutralises that header, which
 * would otherwise pull in the whole STM32 HAL. */
#define __MAIN_H

#include <stdint.h>

typedef enum { HAL_OK = 0, HAL_ERROR = 1, HAL_BUSY = 2, HAL_TIMEOUT = 3 }
HAL_StatusTypeDef;

typedef struct { int dummy; } I2C_HandleTypeDef;

static inline uint32_t HAL_GetTick(void) { return 0U; }
static inline void HAL_Delay(uint32_t ms) { (void)ms; }

static inline HAL_StatusTypeDef HAL_I2C_Master_Transmit(I2C_HandleTypeDef *h,
        uint16_t a, uint8_t *d, uint16_t n, uint32_t t)
{ (void)h; (void)a; (void)d; (void)n; (void)t; return HAL_ERROR; }

static inline HAL_StatusTypeDef HAL_I2C_Master_Receive(I2C_HandleTypeDef *h,
        uint16_t a, uint8_t *d, uint16_t n, uint32_t t)
{ (void)h; (void)a; (void)d; (void)n; (void)t; return HAL_ERROR; }

static inline HAL_StatusTypeDef HAL_I2C_IsDeviceReady(I2C_HandleTypeDef *h,
        uint16_t a, uint32_t tries, uint32_t t)
{ (void)h; (void)a; (void)tries; (void)t; return HAL_ERROR; }

#endif /* STUB_MAIN_H */
