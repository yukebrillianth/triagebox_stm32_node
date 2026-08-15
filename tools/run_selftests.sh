#!/bin/sh
# Compile and run the host-testable parts of the ESP32 I2C link.
#
# Only pure-logic code is testable this way: the register-map layout
# (tb_regs.h) and the button debounce (tb_buttons.c). tb_slave.c is excluded on
# purpose -- it is almost entirely HAL callbacks, and faking enough of the F4
# I2C peripheral to test it would mostly test the fake.
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
CFLAGS="-std=c99 -Wall -Wextra -Werror -g -fsanitize=address,undefined"

echo "==> tb_link_selftest"
# shellcheck disable=SC2086
$CC $CFLAGS -I Core/Inc \
    tools/tb_link_selftest.c Core/Src/tb_buttons.c \
    -o "$OUT/tb_link_selftest"
"$OUT/tb_link_selftest"

echo "All host selftests passed."
