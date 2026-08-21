#ifndef ARM_MATH_H
#define ARM_MATH_H

/*
 * Host stub for CMSIS-DSP, so a selftest can #include "dsp_utils.h" and read
 * its CONSTANTS -- DSP_PPG_MIN_DC, DSP_PPG_FS_HZ and friends -- without an ARM
 * toolchain on the path. Same trick as tools/stub/main.h: claim the real
 * header's include guard so the real one expands to nothing.
 *
 * This is NOT a CMSIS implementation and must never become one. It declares no
 * arm_* function, so any test that tries to actually filter something fails to
 * link rather than quietly testing a fake DSP library. dsp_utils.h itself only
 * needs arm_math.h for the __has_include check at its top and for float32_t in
 * the .c file; every declaration in the header uses plain `float`.
 *
 * Reach it with -I tools/stub, and only for tests that want the constants.
 */

typedef float float32_t;
typedef double float64_t;

#endif /* ARM_MATH_H */
