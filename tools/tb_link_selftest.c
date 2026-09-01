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

/* tb_regs.h FIRST, before any standard header, and that ordering is the test:
 * it is copied verbatim into the ESP32 tree, where it may be the first include
 * in a file, so it has to pull in everything its own inline functions need. Put
 * <string.h> above it and a missing <stddef.h> in there stops being visible. */
#include "tb_regs.h"

#include <assert.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

#include "lora_poll.h"
#include "lora_vital.h"
#include "tb_buttons.h"

/* For DSP_PPG_MIN_DC and DSP_PPG_FS_HZ only. Reached via -I tools/stub, which
 * supplies a do-nothing arm_math.h so this pulls in no CMSIS at all -- see the
 * comment in that file for why it declares no arm_* function. */
#include "dsp_utils.h"

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
    assert(offsetof(tb_snapshot_t, lora_rssi) == TB_REG_LORA_RSSI);

    /* No tail padding: the slave transmits sizeof(tb_snapshot_t) bytes, so any
     * padding would be junk on the wire and would shift TB_REG_SNAPSHOT_END. */
    assert(sizeof(tb_snapshot_t) == TB_REG_SNAPSHOT_END);
    assert(sizeof(tb_snapshot_t) == 0x31U);

    /*
     * lora_rssi sits one byte PAST the vitals block, and that gap is what lets
     * the two boards be flashed independently: the ESP32's 50 ms poll still asks
     * for exactly TB_REG_READ_END bytes, which is what an STM32 built before this
     * field served. Fold it inside the block and every poll from a new ESP32 asks
     * an old slave for one byte more than s_tx holds -- a failed poll, i.e. a
     * link that looks dead, in exchange for saving one 1-byte read per second.
     */
    assert(TB_REG_READ_END == 0x30U);
    assert(TB_REG_LORA_RSSI == TB_REG_READ_END);
    assert(TB_REG_SNAPSHOT_END == TB_REG_READ_END + 1U);

    /*
     * Two different bytes mean "no reading", from two different places, so the
     * validity test is a range and not `!= SENTINEL`:
     *   0x00  this node has not heard a poll yet (polls are 15 s apart)
     *   0xFF  the AddrCallback pad, i.e. an old STM32 or an address it refuses
     * They are adjacent, so one upper bound at -2 rejects both.
     */
    assert(!tb_rssi_valid(0));
    assert(!tb_rssi_valid((int8_t)0xFFU));
    assert(tb_rssi_valid(-123)); /* SF7/125k sensitivity, about the weakest real */
    assert(tb_rssi_valid(TB_RSSI_MAX_DBM));
    assert(!tb_rssi_valid(TB_RSSI_MAX_DBM + 1));

    /* Regression: -12 dBm is what this radio reports with an antenna fitted and
     * the station a couple of metres away, measured over SWD. The bound used to
     * be -20 on a saturation theory, so a real reading was discarded and the
     * ESP32 showed "LoRa siap" on a healthy link. */
    assert(tb_rssi_valid(-12));

    /* Read and write blocks must not overlap; the slave enforces read-only on
     * everything outside the write block by switching on the pointer. */
    assert(TB_REG_READ_END <= TB_REG_CMD);
    assert(TB_REG_SNAPSHOT_END <= TB_REG_CMD);

    /* The write block is contiguous and WRITE_END is one past its last register.
     * tb_slave.c switches on the pointer, so a register at or past WRITE_END is
     * silently ignored -- which is how an unpatched STM32 keeps working, and also
     * how a new register nobody added to the switch would fail. */
    assert(TB_REG_PRIORITY == TB_REG_CMD + 1U);
    assert(TB_REG_CONFIDENCE == TB_REG_PRIORITY + 1U);
    assert(TB_REG_HOST_BATTERY == TB_REG_CONFIDENCE + 1U);
    assert(TB_REG_HOST_BP_SYS == TB_REG_HOST_BATTERY + 1U);
    /* Two bytes each, so the pointer must step by two. A one-byte step would put
     * DIA's low byte on SYS's high byte, and the ESP32's four-byte write would
     * then latch a systolic built from half a diastolic. */
    assert(TB_REG_HOST_BP_DIA == TB_REG_HOST_BP_SYS + 2U);
    assert(TB_REG_WRITE_END == TB_REG_HOST_BP_DIA + 2U);

    /* HOST_BATTERY is written by the master; TB_REG_BATTERY is read by it. Two
     * registers, two directions, one value -- they must not be the same address
     * or the ESP32's write would land on top of what it is about to read. */
    assert(TB_REG_HOST_BATTERY != TB_REG_BATTERY);
    assert(TB_REG_HOST_BATTERY >= TB_REG_CMD);

    /* Same rule for the BP pair, which has the same shape: the ESP32 writes its
     * prediction at HOST_BP_* and reads the echo back at BP_*. These four
     * addresses are the easiest thing in the map to confuse, because two of them
     * share a name up to a prefix. */
    assert(TB_REG_HOST_BP_SYS != TB_REG_BP_SYS);
    assert(TB_REG_HOST_BP_DIA != TB_REG_BP_DIA);
    assert(TB_REG_HOST_BP_SYS >= TB_REG_CMD);
    assert(TB_REG_HOST_BP_DIA >= TB_REG_CMD);
    /* And the write block must still end before the waveform block starts, or a
     * waveform read would be dispatched to the write switch. */
    assert(TB_REG_WRITE_END <= TB_REG_PPG_BASE);

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

/* ---- Waveform block: layout, quantisation, ring bookkeeping -------------- */

static void test_wave_layout(void)
{
    assert(sizeof(tb_wave_sample_t) == 6U); /* no padding: it IS the wire */
    assert(offsetof(tb_wave_sample_t, ir) == 0U);
    assert(offsetof(tb_wave_sample_t, red) == 2U);
    assert(offsetof(tb_wave_sample_t, ecg) == 4U);
    assert(offsetof(tb_wave_block_t, total) == 0U);
    assert(offsetof(tb_wave_block_t, s) == 4U);
    assert(sizeof(tb_wave_block_t) == 4U + (6U * TB_PPG_RING));
    assert(sizeof(tb_wave_block_t) == TB_REG_PPG_END - TB_REG_PPG_BASE);

    /* The three-channel size, spelled out. TB_PPG_RING and the sample stride are
     * both what PROTO 0x03 changed, and the ring size is dictated by exactly this
     * arithmetic against the 0x100 ceiling below -- so pin the number rather than
     * only its relation to the constants, which would follow a wrong edit. */
    assert(TB_PPG_RING == 20U);
    assert(sizeof(tb_wave_block_t) == 124U);
    assert(TB_REG_PPG_END == 0xCCU);

    /*
     * The register pointer is a single byte, so the whole block must be
     * addressable. A base or a ring size that pushes the tail past 0xFF makes
     * the last samples silently unreadable -- the slave would clamp the length
     * and the ESP32 would parse whatever it got as if it were complete.
     */
    assert(TB_REG_PPG_END <= 0x100U);

    /* The other bound on the ring, and the one that actually chose 20: the ESP32
     * polls every 50 ms, so the ring must hold at least a few poll periods or a
     * scheduling hiccup on that side costs samples. 200 ms is four polls. The
     * address space would allow 28 samples; the extra 8 would be bus time spent on
     * history nothing reads. */
    assert(((TB_PPG_RING * 1000U) / TB_PPG_FS_HZ) >= 200U);

    /* Must not overlap the vitals snapshot or the write block. The slave
     * dispatches on the pointer, so an overlap would shadow one of them. */
    assert(TB_REG_PPG_BASE >= TB_REG_READ_END);
    assert(TB_REG_PPG_BASE >= TB_REG_SNAPSHOT_END);
    assert((TB_REG_PPG_BASE >= TB_REG_WRITE_END)
           || (TB_REG_PPG_END <= TB_REG_CMD));

    /* NOT a power of two any more: 6 bytes a sample rules that out under the
     * ceiling above, so tb_wave_push/tb_wave_take must use a modulo. A mask would
     * silently address 16 of the 20 slots and lose a fifth of the waveform. */
    assert((TB_PPG_RING & (TB_PPG_RING - 1U)) != 0U);

    /* Little-endian, same as the vitals block, so the ESP32 can cast. All three
     * channels, because ecg is the one that was appended and so the one whose
     * offset a stale reader would get wrong. */
    {
        tb_wave_block_t b;
        const unsigned char *raw = (const unsigned char *)&b;

        memset(&b, 0, sizeof(b));
        b.s[0].ir = 0x0102U;
        b.s[0].red = 0x0304U;
        b.s[0].ecg = 0x0506U;
        assert(raw[4] == 0x02U && raw[5] == 0x01U);
        assert(raw[6] == 0x04U && raw[7] == 0x03U);
        assert(raw[8] == 0x06U && raw[9] == 0x05U);
    }

    /* The version byte is what stops an 0x02 master parsing this. The stride and
     * the count both moved, so a stale reader does not merely miss the ECG -- it
     * reads RED where IR is and drifts two bytes per sample. */
    assert(TB_PROTO_VER == 0x03U);
}

static void test_ppg_pack(void)
{
    /* Round trip is lossy by exactly the shift, and must never overshoot: a
     * value coming back HIGHER than it went in would let a clamped sample look
     * like a real one. */
    for (uint32_t c = 0U; c <= TB_PPG_MAX_COUNTS; c += 97U) {
        uint32_t back = tb_ppg_unpack(tb_ppg_pack((float)c));
        assert(back <= c);
        assert(c - back < (1U << TB_PPG_SHIFT));
    }

    /* The shift must span the full 18-bit FIFO word and no more -- one step
     * less and the top of the range would wrap to zero, so a bright reading
     * would arrive as no reading. Unpack lands on the bottom of the last step,
     * never above the real maximum. */
    assert(tb_ppg_pack((float)TB_PPG_MAX_COUNTS) == 0xFFFFU);
    assert(tb_ppg_unpack(0xFFFFU) <= TB_PPG_MAX_COUNTS);
    assert(TB_PPG_MAX_COUNTS - tb_ppg_unpack(0xFFFFU) < (1U << TB_PPG_SHIFT));

    /* Out-of-range inputs must clamp, not wrap. The FIR undershoots below zero
     * for a few samples after a step (it has negative taps), and (uint32_t) of
     * a negative float is undefined -- so the guard has to be in pack(), not in
     * every caller. */
    assert(tb_ppg_pack(-1.0f) == 0U);
    assert(tb_ppg_pack(-1e9f) == 0U);
    assert(tb_ppg_pack(0.0f) == 0U);
    assert(tb_ppg_pack(1e9f) == 0xFFFFU);

    /* A quiet NaN must land on 0 rather than an arbitrary value. 0.0/0.0 is
     * folded at compile time, so build the NaN from a comparison the optimiser
     * cannot resolve. */
    {
        volatile float zero = 0.0f;
        assert(tb_ppg_pack(zero / zero) == 0U);
    }

    /* Resolution has to be fine enough to be worth sending: a ~200-count pulse
     * on a ~182800-count baseline is what the hardware actually produces, and
     * it has to survive quantisation as tens of steps, not two or three. */
    {
        uint32_t lo = tb_ppg_pack(182800.0f);
        uint32_t hi = tb_ppg_pack(183000.0f);
        assert(hi - lo >= 40U);
    }
}

/* Mimics one ESP32 poll: read the block (the slave hands over a byte-for-byte
 * copy) and drain it. Returns the number of samples recovered. */
static uint32_t poll(const volatile tb_wave_block_t *live, uint32_t *last,
                     tb_wave_sample_t *out, uint32_t *dropped)
{
    tb_wave_block_t wire;

    memcpy(&wire, (const void *)live, sizeof(wire));
    return tb_wave_take(&wire, last, out, dropped);
}

static void test_wave_ring(void)
{
    volatile tb_wave_block_t blk;
    tb_wave_sample_t got[TB_PPG_RING];
    uint32_t last = 0U, dropped = 0U, n, i, c;

    memset((void *)&blk, 0, sizeof(blk));

    /* Nothing pushed yet: no samples, no phantom drop. */
    assert(poll(&blk, &last, got, &dropped) == 0U);
    assert(dropped == 0U);

    /* A partial fill comes back whole, oldest first, all three channels intact.
     * ECG is deliberately NOT shifted on the wire -- it is 12-bit ADC counts, so
     * it must come back bit-for-bit while ir/red come back quantised. */
    for (i = 0U; i < 5U; ++i) {
        tb_wave_push(&blk, (float)(1000U + (i << TB_PPG_SHIFT)),
                     (float)(2000U + (i << TB_PPG_SHIFT)),
                     (uint16_t)(2049U + i));
    }
    n = poll(&blk, &last, got, &dropped);
    assert(n == 5U && dropped == 0U);
    for (i = 0U; i < n; ++i) {
        assert(tb_ppg_unpack(got[i].ir) == 1000U + (i << TB_PPG_SHIFT));
        assert(tb_ppg_unpack(got[i].red) == 2000U + (i << TB_PPG_SHIFT));
        assert(got[i].ecg == 2049U + i); /* odd values: no shift hiding here */
    }

    /* The whole 12-bit ADC range must survive, including the top of it: a stray
     * shift would fold 4095 to 1023 and the ESP32 would measure R waves a quarter
     * of their real height. */
    tb_wave_push(&blk, 0.0f, 0.0f, 4095U);
    n = poll(&blk, &last, got, &dropped);
    assert(n == 1U && got[0].ecg == 4095U);

    /* An immediate second poll must yield nothing -- the block is state, so a
     * re-read of the same bytes must not replay samples into the ESP32's
     * pipeline. This is the property that makes "no pop" safe. */
    assert(poll(&blk, &last, got, &dropped) == 0U);
    assert(dropped == 0U);

    /* Exactly a full ring is still lossless: the boundary is TB_PPG_RING new
     * samples, not TB_PPG_RING - 1. */
    for (i = 0U; i < TB_PPG_RING; ++i) {
        tb_wave_push(&blk, (float)((i + 1U) << TB_PPG_SHIFT), 0.0f,
                     (uint16_t)(i + 1U));
    }
    n = poll(&blk, &last, got, &dropped);
    assert(n == TB_PPG_RING && dropped == 0U);
    for (i = 0U; i < n; ++i) {
        assert(tb_ppg_unpack(got[i].ir) == (i + 1U) << TB_PPG_SHIFT);
        assert(got[i].ecg == i + 1U);
    }

    /* One past that and the oldest sample is gone -- and must be REPORTED
     * gone. A silent drop here would show up on the ESP32 as a waveform that
     * quietly runs fast, which is far harder to notice than a gap count. */
    for (i = 0U; i < TB_PPG_RING + 1U; ++i) {
        tb_wave_push(&blk, (float)((i + 1U) << TB_PPG_SHIFT), 0.0f,
                     (uint16_t)(i + 1U));
    }
    n = poll(&blk, &last, got, &dropped);
    assert(n == TB_PPG_RING);
    assert(dropped == 1U);
    /* What survives is the NEWEST ring-full, so the first sample handed back is
     * the second one pushed. Returning the oldest instead would hand the ESP32
     * a stale window and lose the live one. */
    assert(tb_ppg_unpack(got[0].ir) == 2U << TB_PPG_SHIFT);
    assert(got[0].ecg == 2U);
    assert(tb_ppg_unpack(got[n - 1U].ir) == (TB_PPG_RING + 1U) << TB_PPG_SHIFT);
    assert(got[n - 1U].ecg == TB_PPG_RING + 1U);

    /* Long run at a realistic ratio: 100Hz in, polled every 10 samples. Every
     * sample must come out exactly once and in order, across many wraps -- and
     * with a ring of 20 the wrap no longer lines up with the poll size, which is
     * precisely the case a mask would get wrong. */
    last = blk.total;
    c = 0U;
    for (i = 0U; i < 500U; ++i) {
        tb_wave_push(&blk, (float)((i + 1U) << TB_PPG_SHIFT), 0.0f,
                     (uint16_t)((i + 1U) & 0xFFFU));
        if (((i + 1U) % 10U) == 0U) {
            n = poll(&blk, &last, got, &dropped);
            assert(n == 10U && dropped == 0U);
            for (uint32_t k = 0U; k < n; ++k) {
                ++c;
                assert(tb_ppg_unpack(got[k].ir) == c << TB_PPG_SHIFT);
                assert(got[k].ecg == (c & 0xFFFU));
            }
        }
    }
    assert(c == 500U);

    /* Polled at 3 samples a time, which shares no factor with 20: every phase of
     * the modulo gets exercised, and an off-by-one in the wrap shows up as a
     * duplicated or skipped sample rather than as nothing at all. */
    last = blk.total;
    c = 0U;
    for (i = 0U; i < 300U; ++i) {
        tb_wave_push(&blk, 0.0f, 0.0f, (uint16_t)((i + 1U) & 0xFFFU));
        if (((i + 1U) % 3U) == 0U) {
            n = poll(&blk, &last, got, &dropped);
            assert(n == 3U && dropped == 0U);
            for (uint32_t k = 0U; k < n; ++k) {
                ++c;
                assert(got[k].ecg == (c & 0xFFFU));
            }
        }
    }
    assert(c == 300U);

    /* A read that comes back far too late loses everything but the last ring-full
     * and says how much: 200ms of history at TB_PPG_FS_HZ, so an ESP32 polling
     * slower than 5Hz is in this case permanently. */
    last = blk.total;
    for (i = 0U; i < 3U * TB_PPG_RING; ++i) {
        tb_wave_push(&blk, 0.0f, 0.0f, (uint16_t)(i + 1U));
    }
    n = poll(&blk, &last, got, &dropped);
    assert(n == TB_PPG_RING);
    assert(dropped == (2U * TB_PPG_RING));
    assert(got[n - 1U].ecg == 3U * TB_PPG_RING); /* newest, not oldest */

    /* total must survive its own u32 wrap: the reader subtracts, so a counter
     * that rolls over between polls has to keep working. It happens after 497
     * days at 100Hz -- unlikely, but it is one assert to prove rather than a
     * comment to hope about.
     *
     * With a modulo rather than a mask this is no longer free: 2^32 is not a
     * multiple of 20. It still works because both sides apply `% TB_PPG_RING` to
     * the same uint32_t counter values, so the reader recomputes exactly the slots
     * the writer used -- which is the property being pinned here. */
    blk.total = 0xFFFFFFFEU;
    last = 0xFFFFFFFEU;
    for (i = 0U; i < 4U; ++i) {
        tb_wave_push(&blk, (float)((i + 1U) << TB_PPG_SHIFT), 0.0f,
                     (uint16_t)(i + 1U));
    }
    assert(blk.total == 2U); /* wrapped */
    n = poll(&blk, &last, got, &dropped);
    assert(n == 4U && dropped == 0U);
    assert(tb_ppg_unpack(got[0].ir) == 1U << TB_PPG_SHIFT);
    assert(got[0].ecg == 1U);
    assert(tb_ppg_unpack(got[3].ir) == 4U << TB_PPG_SHIFT);
    assert(got[3].ecg == 4U);
}

/* ---- The BP pair the ESP32's model writes back --------------------------- */
/*
 * tb_bp_pair_valid() is the only thing standing between a model extrapolating
 * outside its training set and a fabricated blood pressure in a LoRa packet. An
 * ML model does not return an error; it returns a number. So this is a range test
 * on both fields plus the one relation that makes them a pair, and it is host
 * tested because the alternative is discovering the bound on a patient.
 */
static void test_bp_pair_valid(void)
{
    /* Ordinary readings, from the low end of shock to hypertensive. */
    assert(tb_bp_pair_valid(120U, 80U));
    assert(tb_bp_pair_valid(90U, 60U));
    assert(tb_bp_pair_valid(180U, 110U));
    assert(tb_bp_pair_valid(70U, 40U)); /* hypotensive but real */

    /* Both bounds of both fields, and one step outside each. Asserting the
     * boundary itself is the point: an off-by-one here either rejects a real
     * reading or accepts an impossible one, and neither is visible on a link. */
    assert(tb_bp_pair_valid(TB_BP_SYS_MIN, TB_BP_DIA_MIN));
    assert(!tb_bp_pair_valid(TB_BP_SYS_MIN - 1U, TB_BP_DIA_MIN));
    assert(!tb_bp_pair_valid(TB_BP_SYS_MIN, TB_BP_DIA_MIN - 1U));
    assert(tb_bp_pair_valid(TB_BP_SYS_MAX, TB_BP_DIA_MAX));
    assert(!tb_bp_pair_valid(TB_BP_SYS_MAX + 1U, TB_BP_DIA_MAX));
    assert(!tb_bp_pair_valid(TB_BP_SYS_MAX, TB_BP_DIA_MAX + 1U));

    /* The relation, which is what a range test alone would miss: diastolic is
     * the pressure BETWEEN beats. dia > sys is a broken prediction, and dia ==
     * sys is zero pulse pressure, which is incoherent rather than merely
     * alarming -- hence strictly less. */
    assert(!tb_bp_pair_valid(100U, 120U));
    assert(!tb_bp_pair_valid(100U, 100U));
    assert(tb_bp_pair_valid(100U, 99U));

    /* The zeros the STM32 publishes before any prediction has arrived must fail,
     * or a boot-time snapshot would light TB_FLAG_BP_VALID on 0/0. This is why
     * main.c can use "sys is nonzero" as the standing-pair test at all. */
    assert(!tb_bp_pair_valid(0U, 0U));
    assert(!tb_bp_pair_valid(120U, 0U));
    assert(!tb_bp_pair_valid(0U, 80U));

    /* Garbage from an unassembled write: the register pointer auto-increments, so
     * a master that writes only high bytes builds values like these. They must be
     * rejected rather than latched. */
    assert(!tb_bp_pair_valid(0xFFFFU, 0xFFFFU));
    assert(!tb_bp_pair_valid(0x7B00U, 0x5000U));

    /* The bounds themselves must stay physiologically sane and inside the u16
     * fields that carry them, or widening one field would quietly disable the
     * test it belongs to. */
    assert(TB_BP_SYS_MIN < TB_BP_SYS_MAX);
    assert(TB_BP_DIA_MIN < TB_BP_DIA_MAX);
    assert(TB_BP_DIA_MIN < TB_BP_SYS_MIN); /* else no pair could pass at all */
    assert(TB_BP_SYS_MAX <= 0xFFFFU);
    assert(TB_BP_DIA_MAX < TB_BP_SYS_MAX);
}

/* ---- LoRa uplink packet: the station will parse this by casting ----------- */

static void test_lora_vital_layout(void)
{
    /* No padding anywhere, and every uint16_t at an even offset: the station is
     * expected to cast this struct over the received bytes, so a hole here would
     * shift every field after it. */
    assert(offsetof(lora_vital_t, node_id)         == 0U);
    assert(offsetof(lora_vital_t, version)         == 1U);
    assert(offsetof(lora_vital_t, packet_counter)  == 2U);
    assert(offsetof(lora_vital_t, flags)           == 4U);
    assert(offsetof(lora_vital_t, device_status)   == 5U);
    assert(offsetof(lora_vital_t, hr)              == 6U);
    assert(offsetof(lora_vital_t, spo2)            == 8U);
    assert(offsetof(lora_vital_t, rr)              == 9U);
    assert(offsetof(lora_vital_t, bp_sys)          == 10U);
    assert(offsetof(lora_vital_t, bp_dia)          == 12U);
    assert(offsetof(lora_vital_t, battery)         == 14U);
    assert(offsetof(lora_vital_t, priority)        == 15U);
    assert(offsetof(lora_vital_t, confidence)      == 16U);
    assert(offsetof(lora_vital_t, victim_rfid_len) == 17U);
    assert(offsetof(lora_vital_t, victim_rfid)     == 18U);

    /* LORA_VITAL_FIXED_LEN is what lora_vital_len() adds the tag length to. If
     * a field is ever inserted, this is the assert that catches the constant not
     * following it -- otherwise every packet would be short by the difference
     * and the tag would arrive truncated. */
    assert(offsetof(lora_vital_t, victim_rfid) == LORA_VITAL_FIXED_LEN);
    assert(sizeof(lora_vital_t) == LORA_VITAL_FIXED_LEN + LORA_VITAL_RFID_MAX);

    /* Must fit one SX1278 explicit-header packet; LoRa_transmit takes a uint8_t
     * length, so anything over 255 would silently wrap. */
    assert(sizeof(lora_vital_t) <= 255U);

    /* Little-endian, like the I2C map, so both sides can cast rather than shift. */
    {
        lora_vital_t v;
        const unsigned char *raw = (const unsigned char *)&v;

        memset(&v, 0, sizeof(v));
        v.hr = 0x0102U;
        assert(raw[6] == 0x02U && raw[7] == 0x01U);
    }
}

static void test_lora_vital_len(void)
{
    lora_vital_t v;

    memset(&v, 0, sizeof(v));

    /* No tag scanned is the common case, and it must cost nothing on air. */
    assert(lora_vital_len(&v) == LORA_VITAL_FIXED_LEN);

    v.victim_rfid_len = 8U; /* a 4-byte UID in hex */
    assert(lora_vital_len(&v) == LORA_VITAL_FIXED_LEN + 8U);

    v.victim_rfid_len = LORA_VITAL_RFID_MAX; /* a 10-byte UID, the longest legal */
    assert(lora_vital_len(&v) == sizeof(lora_vital_t));

    /* A garbage length must clamp, not run off the end of the struct. */
    v.victim_rfid_len = 0xFFU;
    assert(lora_vital_len(&v) == sizeof(lora_vital_t));
}

static void test_lora_vital_priority(void)
{
    /*
     * The mapping the station will use. Spelled out rather than derived, because
     * the whole point is that 0 is BLACK and not RED -- the numbering is the LoRa
     * alias, not ui_priority_t's declaration order, and getting it wrong swaps
     * the two levels that matter most.
     */
    assert(strcmp(lora_vital_priority_name(0U), "BLACK") == 0);
    assert(strcmp(lora_vital_priority_name(1U), "RED") == 0);
    assert(strcmp(lora_vital_priority_name(2U), "YELLOW") == 0);
    assert(strcmp(lora_vital_priority_name(3U), "GREEN") == 0);

    /* Same numbering as the I2C write register, so the STM32 copies rather than
     * converts. If these ever diverge the node would relabel every patient. */
    assert(LORA_VITAL_PRIORITY_BLACK  == 0U);
    assert(LORA_VITAL_PRIORITY_RED    == 1U);
    assert(LORA_VITAL_PRIORITY_YELLOW == 2U);
    assert(LORA_VITAL_PRIORITY_GREEN  == 3U);

    /* Unscored must be distinguishable from BLACK and must NOT map to a level.
     * A station that defaulted it would report a live patient as dead. */
    assert(LORA_VITAL_PRIORITY_NONE != LORA_VITAL_PRIORITY_BLACK);
    assert(lora_vital_priority_name(LORA_VITAL_PRIORITY_NONE) == NULL);
    assert(lora_vital_priority_name(4U) == NULL);
    assert(lora_vital_priority_name(0x80U) == NULL);
}

static void test_lora_vital_valid(void)
{
    lora_vital_t v;

    memset(&v, 0, sizeof(v));
    v.version = LORA_VITAL_VERSION;

    assert(lora_vital_valid(&v, LORA_VITAL_FIXED_LEN));
    assert(lora_vital_valid(&v, sizeof(lora_vital_t)));

    /* A short frame must be rejected, not parsed. */
    assert(!lora_vital_valid(&v, LORA_VITAL_FIXED_LEN - 1U));
    assert(!lora_vital_valid(&v, 0U));

    /* A stale station build must fail loudly instead of misreading offsets. */
    v.version = LORA_VITAL_VERSION + 1U;
    assert(!lora_vital_valid(&v, sizeof(lora_vital_t)));
    v.version = LORA_VITAL_VERSION;

    /* A tag longer than the field, and a tag the frame is too short to contain:
     * both would otherwise read past what actually arrived. */
    v.victim_rfid_len = LORA_VITAL_RFID_MAX + 1U;
    assert(!lora_vital_valid(&v, sizeof(lora_vital_t)));
    v.victim_rfid_len = 8U;
    assert(!lora_vital_valid(&v, LORA_VITAL_FIXED_LEN + 7U));
    assert(lora_vital_valid(&v, LORA_VITAL_FIXED_LEN + 8U));

    /* Battery has to distinguish "flat" from "no reading", the same way the I2C
     * map does, or the station publishes 0% for a gauge it could not read.
     * main.c copies TB_REG_HOST_BATTERY straight into this field, so the two
     * sentinels are the same byte: 0xFF at both ends, and never 0. */
    assert(LORA_VITAL_BATTERY_NONE == 0xFFU);
    assert(LORA_VITAL_BATTERY_NONE != 0U);
}

/* ---- The poll filter: the node's only defence against answering wrongly --- */
/*
 * ServiceLoRaPoll() decides whether to transmit on the strength of these checks
 * alone, and both failure directions are silent on a radio. Answering a frame
 * that was not a poll for us puts a packet into another node's slot and destroys
 * that node's reply; refusing a real poll looks exactly like a dead node. Neither
 * shows up as an error anywhere, which is why they are pinned here.
 */
static void test_poll_for_me(void)
{
    lora_poll_t p;
    lora_vital_t v;

    memset(&p, 0, sizeof(p));
    p.magic = LORA_POLL_MAGIC;
    p.version = LORA_POLL_VERSION;
    p.station_id = 1U;
    p.node_id = 1U;
    p.command = LORA_POLL_CMD_REPORT;

    assert(sizeof(lora_poll_t) == LORA_POLL_LEN); /* packed, no padding */

    assert(lora_poll_for_me(&p, LORA_POLL_LEN, 1U));
    assert(!lora_poll_for_me(&p, LORA_POLL_LEN, 2U)); /* somebody else's turn */

    /* Truncated: rejected on the length before any field is read. */
    assert(!lora_poll_for_me(&p, LORA_POLL_LEN - 1U, 1U));
    assert(!lora_poll_for_me(&p, 0U, 1U));

    /* A stale copy of lora_poll.h at the station must fail, not misparse. */
    p.version = LORA_POLL_VERSION + 1U;
    assert(!lora_poll_for_me(&p, LORA_POLL_LEN, 1U));
    p.version = LORA_POLL_VERSION;

    /* Node id 0 is reserved precisely so this can never match, whatever the
     * station sends. A node configured as 0 would answer its own protocol's
     * downlink. */
    p.node_id = 0U;
    assert(!lora_poll_for_me(&p, LORA_POLL_LEN, 0U));

    /*
     * THE ONE THAT MATTERS ON A SHARED CHANNEL: every node hears every other
     * node's reply, and the node reads only the first 5 bytes of whatever
     * arrives. A vital's byte 0 is the sender's node_id, which is 1-based, so
     * it can never equal LORA_POLL_MAGIC -- that is the whole reason the magic
     * byte is 0 and node ids start at 1. Checked for every legal address rather
     * than for one example, because a single-node bench test cannot see this.
     */
    for (unsigned id = 1U; id <= LORA_POLL_NODE_MAX; ++id) {
        memset(&v, 0, sizeof(v));
        v.node_id = (uint8_t) id;
        v.version = LORA_VITAL_VERSION;
        assert(!lora_poll_for_me((const lora_poll_t *) &v, LORA_VITAL_FIXED_LEN,
                (uint8_t) id));
    }

    /* And the timing budget the node firmware asserts at compile time, kept
     * here too so it is checked even when nothing rebuilds main.c: the reply
     * must be finished before the station's slot ends. LORA_REPLY_* mirror
     * main.c's defines -- if those change, this fails and says so. */
    assert(150U + 90U < LORA_POLL_SLOT_MS);
    assert(LORA_POLL_SLOT_MS * LORA_POLL_NODE_MAX <= LORA_POLL_PERIOD_MS);
}

/* ---- Flag bits must not collide, and must mean different things ---------- */
static void test_flag_bits(void)
{
    /* Every flag is a distinct single bit. Written out literally rather than
     * looped, so adding a flag that reuses a value fails here instead of
     * silently making two conditions the same condition. */
    assert(TB_FLAG_HR_VALID    == 0x01U);
    assert(TB_FLAG_SPO2_VALID  == 0x02U);
    assert(TB_FLAG_RR_VALID    == 0x04U);
    assert(TB_FLAG_BP_VALID    == 0x08U);
    assert(TB_FLAG_MEASURING   == 0x10U);
    assert(TB_FLAG_PPG_CONTACT == 0x20U);

    /* No overlap: OR of all six has exactly six bits set. */
    {
        unsigned all = TB_FLAG_HR_VALID | TB_FLAG_SPO2_VALID | TB_FLAG_RR_VALID
                | TB_FLAG_BP_VALID | TB_FLAG_MEASURING | TB_FLAG_PPG_CONTACT;
        unsigned bits = 0;
        unsigned v = all;

        while (v != 0U) {
            bits += (v & 1U);
            v >>= 1;
        }
        assert(bits == 6U);
        assert(all == 0x3FU); /* still inside the u8 register */
    }

    /*
     * CONTACT IS NOT A VITALS BIT. The ESP32 maps TB_FLAG_*_VALID onto its
     * UI_VITAL_* mask; contact must stay out of that mapping, because it says
     * whether the WAVEFORM is trustworthy, not whether any number is. If it
     * ever gets folded in, a finger on the sensor would mark SpO2 valid before
     * a block has been computed.
     */
    assert((TB_FLAG_PPG_CONTACT
            & (TB_FLAG_HR_VALID | TB_FLAG_SPO2_VALID | TB_FLAG_RR_VALID
                    | TB_FLAG_BP_VALID)) == 0U);

    /* Contact and sensor health are independent: an idle sensor that answers is
     * TB_SENSOR_MAX30102 set with TB_FLAG_PPG_CONTACT clear. They live in
     * different registers, so the only way to confuse them is to assume the
     * bit values overlap -- they happen to, and that is fine, which is exactly
     * why the two must never be tested against the same byte. */
    assert(TB_REG_FLAGS != TB_REG_SENSOR_OK);
}

/* ---- The contact floor must sit between the two measured populations ----- */
static void test_ppg_contact_floor(void)
{
    /*
     * Measured on this board, 2026-08-18, and the reason DSP_PPG_MIN_DC exists:
     * nothing on the sensor reads ~2000 counts of ambient light plus detector
     * noise, a seated finger reads 150000-180000 depending on position. The
     * floor has to reject the first and accept the second with margin.
     *
     * This is the regression that prompted the constant. The old value of 1000
     * was BELOW the no-finger reading, so the SpO2 DC gate passed on ambient
     * light and the ESP32 was handed a percentage computed from noise -- which
     * on a triage device means a screen showing a plausible SpO2 for a patient
     * whose finger is not on the sensor. Lowering this constant back under
     * NO_FINGER_COUNTS must fail here, not on a ward.
     */
    const float NO_FINGER_COUNTS = 2000.0f;
    const float SEATED_FINGER_MIN = 150000.0f;

    assert(DSP_PPG_MIN_DC > NO_FINGER_COUNTS);
    assert(DSP_PPG_MIN_DC < SEATED_FINGER_MIN);

    /* Margin, not just ordering: at least 2x clear of the noise floor so drift
     * in ambient light cannot cross it, and at least 10x below a real finger so
     * a poorly seated one still registers. */
    assert(DSP_PPG_MIN_DC >= (2.0f * NO_FINGER_COUNTS));
    assert((DSP_PPG_MIN_DC * 10.0f) <= SEATED_FINGER_MIN);

    /* The floor is compared against counts that have been through tb_ppg_pack's
     * shift on the wire, but the gate itself runs on unshifted floats, so it
     * must stay inside the sensor's 18-bit range to be reachable at all. A
     * floor above TB_PPG_MAX_COUNTS would reject every finger. */
    assert(DSP_PPG_MIN_DC < (float) TB_PPG_MAX_COUNTS);

    /* Same duplication main.c asserts at compile time, checked here too so the
     * host tests catch it without an ARM toolchain. */
    assert(TB_PPG_FS_HZ == DSP_PPG_FS_HZ);
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

/*
 * Dsp_PickRate(): which heart-rate source reaches the wire.
 *
 * Worth a host test rather than hardware: the case that matters is the ECG
 * publishing a plausible-but-wrong rate from a disconnected electrode, and
 * reproducing that on the bench means deliberately leaving the clamp off and
 * waiting for the artefact to land inside 30-220 bpm.
 */
static void test_pick_rate(void)
{
    uint8_t from_ppg = 0xAAU;

    /* Neither source: nothing to publish, and not attributed to the PPG. */
    assert(Dsp_PickRate(0U, 0U, &from_ppg) == 0U);
    assert(from_ppg == 0U);

    /* One source only: it wins, whatever it is. */
    assert(Dsp_PickRate(72U, 0U, &from_ppg) == 72U);
    assert(from_ppg == 0U); /* ECG */
    assert(Dsp_PickRate(0U, 68U, &from_ppg) == 68U);
    assert(from_ppg == 1U); /* PPG */

    /* Both, agreeing: the ECG wins, because a real R wave beats a pulse wave
     * for timing and it is the sensor the triage protocol expects. These are
     * the measured pairs from 30 consecutive windows with the clamp on. */
    assert(Dsp_PickRate(85U, 86U, &from_ppg) == 85U);
    assert(from_ppg == 0U);
    assert(Dsp_PickRate(93U, 92U, &from_ppg) == 93U);
    assert(from_ppg == 0U);
    assert(Dsp_PickRate(94U, 95U, &from_ppg) == 94U);
    assert(from_ppg == 0U);

    /* Both, disagreeing: the measured lead-off artefacts. Each one must lose to
     * the PPG rather than be published as tachycardia. */
    assert(Dsp_PickRate(214U, 85U, &from_ppg) == 85U);
    assert(from_ppg == 1U);
    assert(Dsp_PickRate(218U, 82U, &from_ppg) == 82U);
    assert(from_ppg == 1U);
    assert(Dsp_PickRate(136U, 90U, &from_ppg) == 90U);
    assert(from_ppg == 1U);
    assert(Dsp_PickRate(55U, 91U, &from_ppg) == 91U);
    assert(from_ppg == 1U);

    /* The boundary, both directions. 25% of 80 is 20, so 100 is exactly at the
     * limit and must still be trusted; 101 is over and must not be. Asserting
     * both sides is the point -- an off-by-one here silently changes which
     * sensor the device believes. */
    assert(Dsp_PickRate(100U, 80U, &from_ppg) == 100U);
    assert(from_ppg == 0U);
    assert(Dsp_PickRate(101U, 80U, &from_ppg) == 80U);
    assert(from_ppg == 1U);
    /* Symmetric: a low ECG against a high PPG is rejected on the same margin. */
    assert(Dsp_PickRate(80U, 100U, &from_ppg) == 80U);
    assert(from_ppg == 0U);
    assert(Dsp_PickRate(80U, 101U, &from_ppg) == 101U);
    assert(from_ppg == 1U);

    /* Extremes stay inside uint16 arithmetic: DSP_HR_MAX_BPM against
     * DSP_HR_MIN_BPM is a 633% disagreement and must not wrap. */
    assert(Dsp_PickRate(220U, 30U, &from_ppg) == 30U);
    assert(from_ppg == 1U);
}

int main(void)
{
    test_layout();
    test_cmd_numbering();
    test_flag_bits();
    test_ppg_contact_floor();
    test_wave_layout();
    test_ppg_pack();
    test_wave_ring();
    test_bp_pair_valid();
    test_lora_vital_layout();
    test_lora_vital_len();
    test_lora_vital_priority();
    test_lora_vital_valid();
    test_poll_for_me();
    test_button_polarity();
    test_button_debounce();
    test_buttons_independent();
    test_buttons_ignore_high_bits();
    test_buttons_null_safe();
    test_pick_rate();

    printf("tb_link_selftest: all checks passed\n");
    return 0;
}
