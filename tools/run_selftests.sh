#!/bin/sh
# Compile and run the host-testable parts of this firmware.
#
# Mostly pure-logic code: the register-map layout (tb_regs.h), the button
# debounce (tb_buttons.c), the PN532 frame parser, the PPG FIR, and the LoRa
# poll-response path lifted out of main.c. tb_slave.c is excluded on purpose --
# it is almost entirely HAL callbacks, and faking enough of the F4 I2C
# peripheral to test it would mostly test the fake.
#
# Mirrors the ESP32 repo's tools/run_selftests.sh: assert-based, no framework,
# warnings as errors, ASan + UBSan on.
set -eu

cd "$(dirname "$0")/.."

CC=${CC:-gcc}
OUT=$(mktemp -d)
trap 'rm -rf "$OUT"' EXIT

# -std=c99 matches CubeIDE's default for this project. UBSan catches the
# unsigned-shift and packed-struct-alignment mistakes that are easy to make in
# a wire-format header and invisible on a working board.
#
# The sanitizers are separable because ASan does not work under MSYS2/Git Bash:
# its interceptors cannot hook memcpy on Windows, so every binary dies with
# "CHECK failed: ... (real_memcpy) != (0)" before reaching main. The assertions
# are the actual test and they run either way, so the platform without ASan
# still gets the checks -- it just loses the memory-error net. Override with
# SAN= to force them off, or SAN=-fsanitize=undefined for UBSan alone.
case "$(uname -s)" in
MINGW* | MSYS* | CYGWIN*) : "${SAN=}" ;;
*)                        : "${SAN=-fsanitize=address,undefined}" ;;
esac
CFLAGS="-std=c99 -Wall -Wextra -Werror -g $SAN"
[ -n "$SAN" ] || echo "note: sanitizers off, assertions still checked"

echo "==> tb_link_selftest"
# -I tools/stub supplies a do-nothing arm_math.h so this can include
# dsp_utils.h for DSP_PPG_MIN_DC and DSP_PPG_FS_HZ without an ARM toolchain.
# shellcheck disable=SC2086
$CC $CFLAGS -I Core/Inc -I tools/stub \
    tools/tb_link_selftest.c Core/Src/tb_buttons.c \
    -o "$OUT/tb_link_selftest"
"$OUT/tb_link_selftest"

# The PN532 parser needs a fake HAL: tools/pn532_selftest.c includes
# tools/stub/main.h first, which claims main.h's include guard so the real one
# (and the whole STM32 HAL behind it) expands to nothing. The test also
# #includes rfid_pn532.c to reach the static parsing helpers.
echo "==> pn532_selftest"
# shellcheck disable=SC2086
$CC $CFLAGS -I tools -I Core/Inc \
    tools/pn532_selftest.c \
    -o "$OUT/pn532_selftest"
"$OUT/pn532_selftest"

echo "==> ppg_fir_selftest"
# The PPG smoothing FIR is pure arithmetic, so it needs no stubs at all - it
# checks the coefficient set (gain, symmetry, magnitude response) and what the
# filter does to a synthetic PPG carrying hardware-level noise.
# shellcheck disable=SC2086
$CC $CFLAGS tools/ppg_fir_selftest.c -lm -o "$OUT/ppg_fir_selftest"
"$OUT/ppg_fir_selftest"

# The pulse-rate detector is the other half of dsp_utils.c that touches no
# arm_* function, so like ServiceLoRaPoll it is CUT OUT rather than copied and
# the firmware stays the only copy. Three ranges: MedianF32 and RateIsRegular,
# which it shares with the ECG path, and everything from PrAt to end of file.
# The last range assumes the PR section stays LAST in dsp_utils.c - if something
# is appended after it, the grep below fails and this says so.
echo "==> ppg_pr_selftest"
sed -n '/^static float32_t MedianF32/,/^}[[:space:]]*$/p' \
    Core/Src/dsp_utils.c > "$OUT/pr_body.inc"
sed -n '/^static uint8_t RateIsRegular/,/^}[[:space:]]*$/p' \
    Core/Src/dsp_utils.c >> "$OUT/pr_body.inc"
sed -n '/^static float32_t PrAt/,$p' Core/Src/dsp_utils.c >> "$OUT/pr_body.inc"
grep -q 'MedianF32' "$OUT/pr_body.inc" \
    || { echo "cannot find MedianF32() in dsp_utils.c"; exit 1; }
# RateIsRegular is the shared verdict both rate paths must go through. If the
# extraction stops finding it, the ECG path has silently lost its agreement
# test as far as this test is concerned - fail rather than test half of it.
grep -q 'RateIsRegular(const float32_t' "$OUT/pr_body.inc" \
    || { echo "cannot find RateIsRegular() in dsp_utils.c"; exit 1; }
grep -q 'out->bpm = (uint16_t) (bpm + 0.5f);' "$OUT/pr_body.inc" \
    || { echo "cannot find the end of Dsp_PrCompute() in dsp_utils.c"; exit 1; }
grep -q 'arm_' "$OUT/pr_body.inc" \
    && { echo "extracted PR code calls CMSIS; it cannot be host-tested"; exit 1; }
# shellcheck disable=SC2086
$CC $CFLAGS -I Core/Inc -I tools/stub -I "$OUT" \
    tools/ppg_pr_selftest.c -lm \
    -o "$OUT/ppg_pr_selftest"
"$OUT/ppg_pr_selftest"

# ServiceLoRaPoll() is the one piece of main.c worth testing on the host: it is
# pure control flow over a radio, and there is no ARM toolchain here, so without
# this it would reach hardware never having executed once.
#
# The function is CUT OUT OF main.c rather than copied, so main.c stays the only
# copy and an edit there cannot silently stop being tested. sed is fragile by
# nature, hence the two greps: if the markers move, this fails loudly instead of
# testing an empty file.
echo "==> lora_poll_selftest"
sed -n '/^#define NODE_ID /,/^#define LORA_IRQ_CRC_ERROR /p' Core/Src/main.c \
    | grep '^#define' > "$OUT/svc_defs.inc"
# The [[:space:]]* on the closing brace is not decoration: main.c has CRLF line
# endings, so a bare /^}$/ matches nothing and the range runs to end of file.
sed -n '/^_Static_assert(LORA_REPLY_DEADLINE_MS/,/^}[[:space:]]*$/p' \
    Core/Src/main.c > "$OUT/svc_body.inc"
grep -q '^#define LORA_REPLY_DEADLINE_MS ' "$OUT/svc_defs.inc" \
    || { echo "cannot find the LoRa defines in main.c"; exit 1; }
grep -q 'mon_lora_reply_ms' "$OUT/svc_body.inc" \
    || { echo "cannot find ServiceLoRaPoll() in main.c"; exit 1; }
# -std=c11 overrides the c99 above (last -std wins) because the extracted block
# opens with main.c's two _Static_asserts, which are the compile-time proof that
# deadline + airtime fits inside the slot. Under -std=c99 -pedantic gcc rejects
# _Static_assert outright, and dropping it here would mean the budget is proved
# only by whatever CubeIDE happens to accept. -pedantic is on for this test and
# no other: the generated include is the only place a stray GNU extension can
# enter, and the whole point is to catch that before the ARM compiler does.
# shellcheck disable=SC2086
$CC $CFLAGS -std=c11 -pedantic -I Core/Inc -I "$OUT" \
    tools/lora_poll_selftest.c \
    -o "$OUT/lora_poll_selftest"
"$OUT/lora_poll_selftest"

echo "All host selftests passed."
