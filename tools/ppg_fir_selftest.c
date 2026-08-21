/**
 * Host check for the PPG smoothing FIR in Core/Src/dsp_utils.c.
 *
 * The taps below are a copy of ppg_lp_coeffs. They are duplicated rather than
 * included because dsp_utils.c cannot be compiled on the host - it needs the
 * whole of CMSIS-DSP - and the point of this test is the coefficient set, not
 * arm_fir_f32. If you change one, change the other; assert_gain() would catch a
 * paste that is not a unity-gain symmetric lowpass at all, but not a paste of
 * some other perfectly valid filter.
 *
 * Regenerate the taps with:
 *
 *   python3 - <<'EOF'
 *   import math
 *   fs, fc, N = 100.0, 10.0, 31
 *   M = (N - 1) // 2
 *   h = [(2*fc/fs if n-M == 0 else math.sin(2*math.pi*fc/fs*(n-M))/(math.pi*(n-M)))
 *        * (0.54 - 0.46*math.cos(2*math.pi*n/(N-1))) for n in range(N)]
 *   g = sum(h)
 *   print(', '.join('%+.10ff' % (x/g) for x in h))
 *   EOF
 */
#include <assert.h>
#include <math.h>
#include <stdio.h>

#define TAPS 31
#define FS 100.0
/* -std=c99 hides PI (it is a POSIX extension, not ISO C). */
#define PI 3.14159265358979323846
/* (TAPS-1)/2: a symmetric FIR delays every frequency by exactly this much. */
#define GROUP_DELAY 15

static const double h[TAPS] = {
	+0.0000000000, +0.0012014264, +0.0027843278, +0.0042273161,
	+0.0039427634, +0.0000000000, -0.0082567773, -0.0185832101,
	-0.0253898204, -0.0212353087, +0.0000000000, +0.0395881126,
	+0.0918888770, +0.1450990810, +0.1849028786, +0.1996606673,
	+0.1849028786, +0.1450990810, +0.0918888770, +0.0395881126,
	+0.0000000000, -0.0212353087, -0.0253898204, -0.0185832101,
	-0.0082567773, +0.0000000000, +0.0039427634, +0.0042273161,
	+0.0027843278, +0.0012014264, +0.0000000000 };

/** |H(f)|. */
static double mag(double f)
{
	double re = 0.0, im = 0.0;
	for (int n = 0; n < TAPS; ++n) {
		re += h[n] * cos(-2.0 * PI * f / FS * n);
		im += h[n] * sin(-2.0 * PI * f / FS * n);
	}
	return sqrt(re * re + im * im);
}

/** Direct convolution, the reference arm_fir_f32 is expected to match. */
static void filter(const double *x, double *y, int len)
{
	for (int i = 0; i < len; ++i) {
		double acc = 0.0;
		for (int k = 0; k < TAPS; ++k) {
			if (i - k >= 0) {
				acc += h[k] * x[i - k];
			}
		}
		y[i] = acc;
	}
}

static void test_shape(void)
{
	double dc = 0.0;
	for (int n = 0; n < TAPS; ++n) {
		dc += h[n];
	}
	/* Unity DC gain: the smoother must not shift the perfusion baseline, which
	 * the SpO2 ratio divides by. */
	assert(fabs(dc - 1.0) < 1e-6);

	/* Symmetry is what makes the phase linear, and linear phase is the whole
	 * reason this is an FIR: blood-pressure features are read from the timing
	 * and shape of the pulse, which a frequency-dependent delay deforms. */
	for (int n = 0; n < TAPS / 2; ++n) {
		assert(h[n] == h[TAPS - 1 - n]);
	}
	printf("  shape: DC gain %.9f, symmetric\n", dc);
}

static void test_response(void)
{
	/* Passband: flat through the 8th harmonic of a 75bpm pulse (1.25Hz). */
	const double pass[] = { 0.5, 1.25, 2.5, 5.0 };
	for (unsigned i = 0; i < sizeof pass / sizeof pass[0]; ++i) {
		double m = mag(pass[i]);
		printf("  %5.2f Hz -> %+7.2f dB\n", pass[i], 20.0 * log10(m));
		assert(m > 0.98);
	}
	/* Stopband: the paper this follows asks for noise suppressed above 15Hz. */
	const double stop[] = { 15.0, 20.0, 25.0, 30.0, 40.0, 50.0 };
	for (unsigned i = 0; i < sizeof stop / sizeof stop[0]; ++i) {
		double m = mag(stop[i]);
		printf("  %5.1f Hz -> %+7.2f dB\n", stop[i], 20.0 * log10(m));
		assert(m < 0.01); /* -40dB */
	}
}

/** Deterministic uniform noise in [-1,1); a fixed sequence keeps the assertions
 *  below reproducible, which rand() across libcs would not. */
static double hash_noise(unsigned *s)
{
	*s = (*s * 1103515245u) + 12345u;
	return ((double) ((*s >> 8) & 0xFFFFu) / 32768.0) - 1.0;
}

static void test_signal(void)
{
	enum { N = 1000 }; /* 10s at 100Hz */
	static double clean[N], noisy[N], out[N];
	unsigned seed = 1u;

	/* A PPG-shaped pulse: 1.25Hz fundamental plus the harmonics that make the
	 * systolic rise and the dicrotic notch, on the ~182800-count baseline the
	 * MAX30102 actually reads, with 25 counts of hash on top - the amplitudes
	 * measured on hardware. */
	for (int i = 0; i < N; ++i) {
		double t = (double) i / FS;
		double p = 100.0 * sin(2.0 * PI * 1.25 * t)
				+ 30.0 * sin(2.0 * PI * 2.5 * t + 0.7)
				+ 12.0 * sin(2.0 * PI * 3.75 * t + 1.4);
		clean[i] = 182800.0 + p;
		noisy[i] = clean[i] + 25.0 * hash_noise(&seed);
	}

	filter(noisy, out, N);

	/* Compare against the clean signal delayed by the group delay, over the
	 * region past the filter's settling. */
	double err_in = 0.0, err_out = 0.0;
	int lo = TAPS, count = 0;
	for (int i = lo; i < N; ++i) {
		double ref = clean[i - GROUP_DELAY];
		err_in += (noisy[i] - clean[i]) * (noisy[i] - clean[i]);
		err_out += (out[i] - ref) * (out[i] - ref);
		++count;
	}
	err_in = sqrt(err_in / count);
	err_out = sqrt(err_out / count);
	printf("  noise RMS %.2f counts -> %.2f counts (%.1fx cleaner)\n", err_in,
			err_out, err_in / err_out);
	assert(err_out < (err_in / 2.0)); /* white noise comes out at 0.42x */

	/* And the pulse itself has to survive. Measured on the noise-free signal:
	 * peak-to-peak of the filtered noisy trace would be inflated by whatever
	 * residual hash happens to land on a peak, which is a property of the
	 * noise, not of the filter's fidelity. */
	static double ideal[N];
	filter(clean, ideal, N);
	double cmin = 1e30, cmax = -1e30, omin = 1e30, omax = -1e30;
	for (int i = lo; i < N; ++i) {
		double ref = clean[i - GROUP_DELAY];
		cmin = (ref < cmin) ? ref : cmin;
		cmax = (ref > cmax) ? ref : cmax;
		omin = (ideal[i] < omin) ? ideal[i] : omin;
		omax = (ideal[i] > omax) ? ideal[i] : omax;
	}
	double ratio = (omax - omin) / (cmax - cmin);
	printf("  pulse peak-to-peak %.1f -> %.1f counts (%.3fx)\n", cmax - cmin,
			omax - omin, ratio);
	assert(ratio > 0.98 && ratio < 1.02);

	/* The delay must really be GROUP_DELAY: a wrong alignment here would mean
	 * pulse-timing features are read at the wrong instant. Correlating the
	 * output against the clean signal at +/-3 samples must peak at 0 offset. */
	double best_err = 1e30;
	int best_shift = 99;
	for (int s = GROUP_DELAY - 3; s <= GROUP_DELAY + 3; ++s) {
		double e = 0.0;
		for (int i = lo; i < N; ++i) {
			double d = out[i] - clean[i - s];
			e += d * d;
		}
		if (e < best_err) {
			best_err = e;
			best_shift = s;
		}
	}
	printf("  best alignment at %d samples (expected %d)\n", best_shift,
			GROUP_DELAY);
	assert(best_shift == GROUP_DELAY);
}

int main(void)
{
	printf("ppg_fir_selftest: %d taps, %.0fHz sample rate\n", TAPS, FS);
	test_shape();
	test_response();
	test_signal();
	printf("ppg_fir_selftest: OK\n");
	return 0;
}
