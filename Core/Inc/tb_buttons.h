#ifndef TB_BUTTONS_H
#define TB_BUTTONS_H

#include <stddef.h>
#include <stdint.h>

/*
 * Debounce for the 4 front-panel buttons on PB12..PB15.
 *
 * Deliberately free of HAL and of any GPIO knowledge: the caller reads the
 * pins and passes the levels in, this returns the debounced state byte that
 * goes on the I2C wire. That split is what makes the debounce logic testable
 * on a host with no fakes at all (tb_buttons_selftest.c).
 */

/*
 * Consecutive agreeing polls needed before a change is accepted. The superloop
 * polls about every 10 ms (HAL_Delay(10) plus whatever the DSP took), so 3 is
 * roughly 30-40 ms -- past the few ms of contact bounce on a tactile switch,
 * still far below the ~100 ms where a deliberate tap starts to feel laggy.
 */
#ifndef TB_BTN_DEBOUNCE_POLLS
#define TB_BTN_DEBOUNCE_POLLS 3U
#endif

typedef struct {
    uint8_t stable;               /* last accepted mask, 1 = pressed */
    uint8_t candidate;            /* mask being counted towards acceptance */
    uint8_t agree[4];             /* per-button consecutive agreeing polls */
} tb_buttons_t;

/* Zero-initialize, or call this. Starts with all buttons released. */
void tb_buttons_init(tb_buttons_t *b);

/*
 * Feed one poll. `pin_levels` bit i is the RAW logic level of BUTTON_(i+1)'s
 * pin: bit0 = PB12 ... bit3 = PB15. Bits above 3 are ignored.
 *
 * The pins are input + pull-up with the switches to ground, so a pressed
 * button reads LOW. This function does that inversion, and it is the ONLY
 * place it happens -- everything downstream, including the I2C register, uses
 * 1 = pressed. If the hardware ever changes to pull-down, this is the one line
 * to edit and the ESP32 needs no change at all.
 *
 * Returns the debounced mask (TB_BTN_* from tb_regs.h).
 */
uint8_t tb_buttons_poll(tb_buttons_t *b, uint8_t pin_levels);

/* Last debounced mask without feeding a new sample. */
uint8_t tb_buttons_state(const tb_buttons_t *b);

#endif /* TB_BUTTONS_H */
