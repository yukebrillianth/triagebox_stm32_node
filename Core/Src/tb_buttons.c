#include "tb_buttons.h"

#include "tb_regs.h"

/* Only the low 4 bits of a pin-level mask mean anything. */
#define TB_BTN_ALL (TB_BTN_1 | TB_BTN_2 | TB_BTN_3 | TB_BTN_4)

void tb_buttons_init(tb_buttons_t *b)
{
    if (b == NULL) {
        return;
    }
    b->stable = 0U;
    b->candidate = 0U;
    for (uint32_t i = 0; i < 4U; ++i) {
        b->agree[i] = 0U;
    }
}

uint8_t tb_buttons_poll(tb_buttons_t *b, uint8_t pin_levels)
{
    uint8_t raw;

    if (b == NULL) {
        return 0U;
    }

    /* Active-low pins -> active-high mask. The one inversion in the system. */
    raw = (uint8_t)(~pin_levels) & TB_BTN_ALL;

    /*
     * Per-button counters rather than one counter for the whole byte: two
     * buttons pressed a few milliseconds apart are independent events, and a
     * shared counter would restart on the second press and delay the first.
     */
    for (uint32_t i = 0; i < 4U; ++i) {
        const uint8_t mask = (uint8_t)(1U << i);
        const uint8_t now = raw & mask;

        if (now == (b->stable & mask)) {
            b->agree[i] = 0U; /* already settled here */
            continue;
        }
        if (b->agree[i] < 0xFFU) {
            ++b->agree[i];
        }
        if (b->agree[i] >= TB_BTN_DEBOUNCE_POLLS) {
            b->stable = (uint8_t)((b->stable & ~mask) | now);
            b->agree[i] = 0U;
        }
    }

    b->candidate = raw;
    return b->stable;
}

uint8_t tb_buttons_state(const tb_buttons_t *b)
{
    return (b != NULL) ? b->stable : 0U;
}
