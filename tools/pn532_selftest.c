/*
 * Host selftest for the PN532 frame parser.
 *
 * Includes the driver source directly so the static parsing helpers can be
 * called: they are where all the arithmetic lives, and testing them through
 * Pn532_Service would require faking the whole I2C peripheral -- which mostly
 * tests the fake. The HAL calls come from tools/stub/main.h and always fail;
 * these tests never make a transfer.
 *
 * Asserts, no framework, per the project's existing host-test style.
 */

#include <assert.h>
#include <stdio.h>
#include <string.h>

/* Must precede the driver: it claims Core/Inc/main.h's include guard, so the
 * real header (and the STM32 HAL behind it) expands to nothing. */
#include "stub/main.h"

#include "../Core/Src/rfid_pn532.c"

/* ---- start-code search --------------------------------------------------- */

static void test_find_start(void)
{
    /* Canonical frame: 00 00 FF, so the 0xFF is at index 2. */
    {
        const uint8_t f[] = { 0x00u, 0x00u, 0xFFu, 0x02u, 0xFEu, 0xD5u };
        assert(FindStart(f, sizeof(f)) == 2U);
    }

    /*
     * The regression this exists for. A module that emits one padding zero
     * ahead of the preamble shifts everything by a byte; the old parser tested
     * f[0..2] literally and reported the module dead. Any amount of leading
     * padding must be tolerated.
     */
    {
        const uint8_t f[] = { 0x00u, 0x00u, 0x00u, 0xFFu, 0x02u, 0xFEu };
        assert(FindStart(f, sizeof(f)) == 3U);
    }
    {
        const uint8_t f[] = { 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0xFFu };
        assert(FindStart(f, sizeof(f)) == 5U);
    }

    /* No start code at all -> 0xFF sentinel, never a valid index. */
    {
        const uint8_t f[] = { 0x00u, 0x00u, 0x00u, 0x00u };
        assert(FindStart(f, sizeof(f)) == 0xFFu);
    }
    /* An idle/open bus reads all ones. Must not be mistaken for a frame. */
    {
        const uint8_t f[] = { 0xFFu, 0xFFu, 0xFFu, 0xFFu };
        assert(FindStart(f, sizeof(f)) == 0xFFu);
    }
    /* 0xFF with no 0x00 before it is not a start code. */
    {
        const uint8_t f[] = { 0xD5u, 0xFFu, 0x02u };
        assert(FindStart(f, sizeof(f)) == 0xFFu);
    }
    /* Degenerate lengths must not read out of bounds (ASan proves it). */
    {
        const uint8_t f[] = { 0x00u };
        assert(FindStart(f, 1U) == 0xFFu);
        assert(FindStart(f, 0U) == 0xFFu);
    }
}

/* ---- response header ----------------------------------------------------- */

/*
 * Builds 00 00 FF LEN LCS D5 <payload> DCS 00 with @p pad extra leading zeros.
 *
 * The leading 0x00 is the preamble and 00 FF is the start code, so a frame with
 * no padding puts LEN at index 3 -- the indices the reject tests poke at.
 */
static uint8_t build(uint8_t *buf, uint8_t pad, const uint8_t *payload,
        uint8_t n)
{
    uint8_t i = 0;
    uint8_t k;

    for (k = 0; k < pad; ++k) {
        buf[i++] = 0x00u;
    }
    buf[i++] = 0x00u; /* preamble */
    buf[i++] = 0x00u; /* start code, byte 1 */
    buf[i++] = 0xFFu; /* start code, byte 2 */

    const uint8_t len = (uint8_t)(n + 1U); /* payload + TFI */
    buf[i++] = len;
    buf[i++] = (uint8_t)(~len + 1U); /* LCS */
    buf[i++] = PN532_PN532_TO_HOST;

    uint8_t sum = PN532_PN532_TO_HOST;
    for (k = 0; k < n; ++k) {
        buf[i++] = payload[k];
        sum = (uint8_t)(sum + payload[k]);
    }
    buf[i++] = (uint8_t)(~sum + 1U); /* DCS */
    buf[i++] = 0x00u;
    return i;
}

static void test_parse_header_basic(void)
{
    /* GetFirmwareVersion reply payload: 03 IC Ver Rev Support. */
    const uint8_t payload[] = { 0x03u, 0x32u, 0x01u, 0x06u, 0x07u };
    uint8_t f[32];
    uint8_t at = 0;

    const uint8_t n = build(f, 0U, payload, sizeof(payload));
    assert(ParseHeader(f, n, &at) == sizeof(payload));
    /* The offset must point at the first payload byte, not at TFI: getting this
     * off by one is how a reply "arrives" but the command echo never matches. */
    assert(f[at] == 0x03u);
    assert(memcmp(&f[at], payload, sizeof(payload)) == 0);
}

static void test_parse_header_tolerates_padding(void)
{
    const uint8_t payload[] = { 0x15u }; /* SAMConfiguration reply */
    uint8_t at_none = 0;
    uint8_t at_pad = 0;
    uint8_t a[32];
    uint8_t b[32];

    const uint8_t na = build(a, 0U, payload, sizeof(payload));
    const uint8_t nb = build(b, 3U, payload, sizeof(payload));

    /* Same frame, three bytes of padding: same payload length, and the offset
     * moves by exactly the padding. */
    assert(ParseHeader(a, na, &at_none) == 1U);
    assert(ParseHeader(b, nb, &at_pad) == 1U);
    assert(at_pad == (uint8_t)(at_none + 3U));
    assert(a[at_none] == 0x15u);
    assert(b[at_pad] == 0x15u);
}

static void test_parse_header_rejects(void)
{
    /* With no padding the frame is 00 00 FF LEN LCS TFI ... so LEN is at 3. */
    enum { LEN_AT = 3, LCS_AT = 4, TFI_AT = 5 };
    const uint8_t payload[] = { 0x4Bu, 0x01u };
    uint8_t f[32];
    uint8_t at = 0;
    uint8_t n;

    /* Corrupt LCS: LEN + LCS no longer wraps to zero. */
    n = build(f, 0U, payload, sizeof(payload));
    f[LCS_AT] = (uint8_t)(f[LCS_AT] + 1U);
    assert(ParseHeader(f, n, &at) == 0U);

    /* Wrong direction byte: 0xD4 is host-to-PN532, so a reply carrying it is
     * our own frame echoed back, not a response. */
    n = build(f, 0U, payload, sizeof(payload));
    f[TFI_AT] = PN532_HOST_TO_PN532;
    assert(ParseHeader(f, n, &at) == 0U);

    /* LEN = 0 means not even a TFI, so there is no payload to point at. */
    n = build(f, 0U, payload, sizeof(payload));
    f[LEN_AT] = 0x00u;
    f[LCS_AT] = 0x00u; /* keeps LEN + LCS == 0, so only the LEN < 1 test fires */
    assert(ParseHeader(f, n, &at) == 0U);

    /*
     * A frame claiming more payload than was actually read must be refused
     * rather than parsed: this is the bounds check that stops a truncated read
     * from being interpreted over uninitialised buffer bytes.
     */
    n = build(f, 0U, payload, sizeof(payload));
    assert(ParseHeader(f, n, &at) == sizeof(payload));
    assert(ParseHeader(f, (uint8_t)(n - 3U), &at) == 0U);

    /* No frame at all. */
    memset(f, 0xFFu, sizeof(f));
    assert(ParseHeader(f, sizeof(f), &at) == 0U);
}

/* ---- buffer sizing ------------------------------------------------------- */

static void test_scan_read_fits(void)
{
    /*
     * ReadFrame reads want+1 bytes to consume the leading status byte, so asking
     * for the full buffer overflows its own bounds check and the read silently
     * never happens. That returned NO_CARD, which is indistinguishable from an
     * empty field -- the reason the bug survived a bench session.
     */
    assert((uint32_t)PN532_SCAN_READ + 1U <= PN532_RX_MAX);

    /* The longest reply the scan must hold: status + 00 00 FF + LEN + LCS + TFI
     * + 4B + NbTg + Tg + SENS_RES(2) + SEL_RES + UIDLen + 10 UID + DCS + 00. */
    assert(PN532_SCAN_READ >= 23U);

    /* The raw capture must not overrun mon_rfid_raw[] on the main.c side. */
    assert(PN532_RAW_MAX <= PN532_RX_MAX);
}

/* ---- UID hashing --------------------------------------------------------- */

static void test_hash_uid(void)
{
    const uint8_t a[4] = { 0xDEu, 0xADu, 0xBEu, 0xEFu };
    const uint8_t b[7] = { 0xDEu, 0xADu, 0xBEu, 0xEFu, 0x01u, 0x02u, 0x03u };

    /* 0 is reserved for "no tag", so a real UID must never hash to it. */
    assert(Pn532_HashUid(a, sizeof(a)) != 0U);
    assert(Pn532_HashUid(NULL, 4U) == 0U);
    assert(Pn532_HashUid(a, 0U) == 0U);

    /* Deterministic, and length-sensitive: a 4-byte UID must not collide with a
     * 7-byte one sharing its first four bytes. */
    assert(Pn532_HashUid(a, sizeof(a)) == Pn532_HashUid(a, sizeof(a)));
    assert(Pn532_HashUid(a, sizeof(a)) != Pn532_HashUid(b, sizeof(b)));
    /* Same bytes, different claimed length. */
    assert(Pn532_HashUid(b, 4U) != Pn532_HashUid(b, 5U));
}

int main(void)
{
    test_find_start();
    test_parse_header_basic();
    test_parse_header_tolerates_padding();
    test_parse_header_rejects();
    test_scan_read_fits();
    test_hash_uid();

    printf("pn532_selftest: all checks passed\n");
    return 0;
}
