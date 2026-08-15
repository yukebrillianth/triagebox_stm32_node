/*
 * Host selftest for the parts of the ESP32 link that are pure logic: the
 * register-map layout and the button debounce. Both are things the compiler
 * cannot check and hardware checks only slowly.
 *
 * Not part of the firmware build -- Core/Src is a CubeIDE source folder, so a
 * file here would be cross-compiled for the STM32. It lives in tools/ and is
 * built by tools/run_selftests.sh.
 *
 * Asserts, no framework, per the project's existing host-test style.
 */

#include <assert.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

#include "tb_buttons.h"
#include "tb_regs.h"

/* ---- The register map is a wire contract, so pin every offset ------------ */

static void test_layout(void)
{
    /*
     * These offsets are what the ESP32 will index with. If someone reorders a
     * field in tb_snapshot_t, the struct still compiles, the firmware still
     * runs, and the ESP32 silently reads heart rate out of the SpO2 slot. That
     * is the bug this test exists to make impossible.
     */
    assert(offsetof(tb_snapshot_t, proto_ver) == TB_REG_PROTO_VER);
    assert(offsetof(tb_snapshot_t, seq)       == TB_REG_SEQ);
    assert(offsetof(tb_snapshot_t, flags)     == TB_REG_FLAGS);
    assert(offsetof(tb_snapshot_t, buttons)   == TB_REG_BUTTONS);
    assert(offsetof(tb_snapshot_t, hr)        == TB_REG_HR);
    assert(offsetof(tb_snapshot_t, spo2)      == TB_REG_SPO2);
    assert(offsetof(tb_snapshot_t, rr_x10)    == TB_REG_RR_X10);
    assert(offsetof(tb_snapshot_t, bp_sys)    == TB_REG_BP_SYS);
    assert(offsetof(tb_snapshot_t, bp_dia)    == TB_REG_BP_DIA);
    assert(offsetof(tb_snapshot_t, battery)   == TB_REG_BATTERY);
    assert(offsetof(tb_snapshot_t, sensor_ok) == TB_REG_SENSOR_OK);
    assert(offsetof(tb_snapshot_t, rfid_len)  == TB_REG_RFID_LEN);
    assert(offsetof(tb_snapshot_t, rfid)      == TB_REG_RFID);

    /* No tail padding: the slave transmits sizeof(tb_snapshot_t) bytes, so any
     * padding would be junk on the wire and would shift TB_REG_READ_END. */
    assert(sizeof(tb_snapshot_t) == TB_REG_READ_END);
    assert(sizeof(tb_snapshot_t) == 0x30U);

    /* Read and write blocks must not overlap; the slave enforces read-only on
     * everything outside the write block by switching on the pointer. */
    assert(TB_REG_READ_END <= TB_REG_CMD);

    /* The HAL wants the address shifted; ESP-IDF does not. Both sides derive
     * from the same constant, so check the shift itself. */
    assert(TB_I2C_SLAVE_ADDR_HAL == 0x84U);
    assert(TB_I2C_SLAVE_ADDR == 0x42U);

    /* Must not collide with anything already on the shared bus (V3.0 scan). */
    assert(TB_I2C_SLAVE_ADDR != 0x20U); /* TCA9554 expander */
    assert(TB_I2C_SLAVE_ADDR != 0x26U); /* partial-decode phantom */
    assert(TB_I2C_SLAVE_ADDR != 0x3cU); /* SW6106 PMIC */
    assert(TB_I2C_SLAVE_ADDR != 0x51U); /* PCF85063A RTC */
    assert(TB_I2C_SLAVE_ADDR != 0x5dU); /* GT911 touch */
    /* And must be a legal 7-bit address, outside the reserved ranges. */
    assert(TB_I2C_SLAVE_ADDR >= 0x08U && TB_I2C_SLAVE_ADDR <= 0x77U);

    /* Little-endian u16s: byte 0 of hr must be the low byte. This is what lets
     * both sides memcpy instead of shifting. */
    {
        tb_snapshot_t s;
        const unsigned char *raw = (const unsigned char *)&s;

        memset(&s, 0, sizeof(s));
        s.hr = 0x0102U;
        assert(raw[TB_REG_HR] == 0x02U);
        assert(raw[TB_REG_HR + 1] == 0x01U);
    }
}

/* ---- Command numbering must match the ESP32's tb_cmd_t ------------------ */

static void test_cmd_numbering(void)
{
    /* Copied from components/triagebox_link/include/tb_frame.h. Two enums in
     * two repos that must agree; nothing but a test will catch a drift. */
    assert(TB_CMD_START_SCAN    == 0x01U);
    assert(TB_CMD_START_MEASURE == 0x02U);
    assert(TB_CMD_ABORT         == 0x03U);
    assert(TB_CMD_POWER_OFF     == 0x04U);
    /* NONE must be distinguishable from every real command, since
     * tb_slave_take_cmd() uses it as "nothing pending". */
    assert(TB_CMD_NONE == 0x00U);
}

/* ---- Buttons: polarity first, because it is the easiest thing to invert -- */

static void test_button_polarity(void)
{
    tb_buttons_t b;

    tb_buttons_init(&b);
    assert(tb_buttons_state(&b) == 0U);

    /* All pins HIGH = all released (pull-ups, switches to ground). Feed enough
     * polls that debounce could not be hiding a change. */
    for (unsigned i = 0; i < TB_BTN_DEBOUNCE_POLLS + 2U; ++i) {
        assert(tb_buttons_poll(&b, 0x0fU) == 0U);
    }

    /* BUTTON_1 pressed pulls PB12 LOW -> bit0 set. This is the assertion the
     * whole wire contract rests on: 1 means pressed. */
    for (unsigned i = 0; i < TB_BTN_DEBOUNCE_POLLS; ++i) {
        (void)tb_buttons_poll(&b, 0x0eU);
    }
    assert(tb_buttons_state(&b) == TB_BTN_1);

    /* Bit order: BUTTON_4 (PB15) is bit3, not bit0. */
    tb_buttons_init(&b);
    for (unsigned i = 0; i < TB_BTN_DEBOUNCE_POLLS; ++i) {
        (void)tb_buttons_poll(&b, 0x07U);
    }
    assert(tb_buttons_state(&b) == TB_BTN_4);
}

static void test_button_debounce(void)
{
    tb_buttons_t b;
    unsigned i;

    tb_buttons_init(&b);
    for (i = 0; i < TB_BTN_DEBOUNCE_POLLS + 2U; ++i) {
        (void)tb_buttons_poll(&b, 0x0fU);
    }

    /* A press must NOT register before the threshold. */
    for (i = 0; i < TB_BTN_DEBOUNCE_POLLS - 1U; ++i) {
        assert(tb_buttons_poll(&b, 0x0eU) == 0U);
    }
    /* ...and must register exactly on it. */
    assert(tb_buttons_poll(&b, 0x0eU) == TB_BTN_1);

    /* Contact bounce: a single stray released-poll mid-press must not produce
     * a release, and must not leave the counter primed to fire spuriously. */
    (void)tb_buttons_poll(&b, 0x0fU); /* one bounce */
    assert(tb_buttons_state(&b) == TB_BTN_1);
    (void)tb_buttons_poll(&b, 0x0eU); /* back to pressed */
    assert(tb_buttons_state(&b) == TB_BTN_1);
    for (i = 0; i < TB_BTN_DEBOUNCE_POLLS + 2U; ++i) {
        assert(tb_buttons_poll(&b, 0x0eU) == TB_BTN_1);
    }

    /* Release needs the same debounce as press. */
    for (i = 0; i < TB_BTN_DEBOUNCE_POLLS - 1U; ++i) {
        assert(tb_buttons_poll(&b, 0x0fU) == TB_BTN_1);
    }
    assert(tb_buttons_poll(&b, 0x0fU) == 0U);
}

static void test_buttons_independent(void)
{
    tb_buttons_t b;
    unsigned i;

    /*
     * Two buttons pressed one poll apart. With a single shared counter the
     * second press would restart the count and delay the first -- the reason
     * the debounce is per-button. This is also the case a single-slot event
     * queue on the ESP32 would drop, which is why the wire carries state.
     */
    tb_buttons_init(&b);
    for (i = 0; i < TB_BTN_DEBOUNCE_POLLS + 2U; ++i) {
        (void)tb_buttons_poll(&b, 0x0fU);
    }

    (void)tb_buttons_poll(&b, 0x0eU); /* btn1 down */
    for (i = 0; i < TB_BTN_DEBOUNCE_POLLS; ++i) {
        (void)tb_buttons_poll(&b, 0x0cU); /* btn1 + btn2 down */
    }
    assert(tb_buttons_state(&b) == (TB_BTN_1 | TB_BTN_2));

    /* Releasing one must not disturb the other. Pin HIGH = released, so
     * button 2 released with button 1 still held is 0x0e, not 0x0d. */
    for (i = 0; i < TB_BTN_DEBOUNCE_POLLS; ++i) {
        (void)tb_buttons_poll(&b, 0x0eU); /* btn2 released, btn1 held */
    }
    assert(tb_buttons_state(&b) == TB_BTN_1);
}

static void test_buttons_ignore_high_bits(void)
{
    tb_buttons_t b;
    unsigned i;

    /* Only 4 pins exist. Garbage in the upper nibble (a caller ORing in
     * something unrelated) must never appear in the published byte, or the
     * ESP32 would see phantom buttons. */
    tb_buttons_init(&b);
    for (i = 0; i < TB_BTN_DEBOUNCE_POLLS + 2U; ++i) {
        assert((tb_buttons_poll(&b, 0xffU) & 0xf0U) == 0U);
    }
    for (i = 0; i < TB_BTN_DEBOUNCE_POLLS + 2U; ++i) {
        assert((tb_buttons_poll(&b, 0x00U) & 0xf0U) == 0U);
    }
    assert(tb_buttons_state(&b) == (TB_BTN_1 | TB_BTN_2 | TB_BTN_3 | TB_BTN_4));
}

static void test_buttons_null_safe(void)
{
    /* The publish path runs every 10 ms; a null here would be a hard fault
     * rather than a visible error, so degrade quietly instead. */
    assert(tb_buttons_poll(NULL, 0x0fU) == 0U);
    assert(tb_buttons_state(NULL) == 0U);
    tb_buttons_init(NULL);
}

int main(void)
{
    test_layout();
    test_cmd_numbering();
    test_button_polarity();
    test_button_debounce();
    test_buttons_independent();
    test_buttons_ignore_high_bits();
    test_buttons_null_safe();

    printf("tb_link_selftest: all checks passed\n");
    return 0;
}
