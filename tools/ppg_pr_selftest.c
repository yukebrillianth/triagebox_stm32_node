/**
 * Host check for the PPG pulse-rate detector in Core/Src/dsp_utils.c.
 *
 * Dsp_PrCompute and its three helpers are CUT OUT of dsp_utils.c by
 * tools/run_selftests.sh and #included here as generated fragments, the same
 * trick tools/lora_poll_selftest.c uses on main.c. Copying them instead would
 * mean this test slowly stops describing the firmware; extracting them means an
 * edit there is an edit here. It is only possible at all because the whole
 * detector is plain arithmetic over one ring buffer -- it calls no arm_*
 * function, so it needs no CMSIS and no ARM toolchain.
 *
 * What is being checked is the thing that cannot be checked on hardware: a
 * finger gives you one rate at a time and no ground truth, whereas here the
 * true rate is an input. So the cases are the ones that matter and are hard to
 * stage deliberately -- a dicrotic notch that must NOT be counted, an irregular
 * rhythm that must be refused rather than averaged, and rates at both
 * plausibility limits.
 */
#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "dsp_utils.h"

/* -std=c99 hides M_PI (it is a POSIX extension, not ISO C). */
#define PR_PI 3.14159265358979323846f

/* The extracted code's file-statics, non-static here so the test can stage the
 * ring directly. Same names, same types as dsp_utils.c. */
static float32_t pr_window[DSP_PR_WINDOW];
static uint16_t pr_head;
static uint16_t pr_filled;

#include "pr_body.inc"

/* ---- staging ------------------------------------------------------------- */

/** Fills the ring with a full window, oldest sample first. */
static void stage(const float32_t *v, uint16_t n, uint16_t head)
{
	assert(n == DSP_PR_WINDOW);
	pr_head = head;
	pr_filled = DSP_PR_WINDOW;
	for (uint16_t i = 0; i < n; ++i) {
		pr_window[(head + i) % DSP_PR_WINDOW] = v[i];
	}
}

static float32_t buf[DSP_PR_WINDOW];

/**
 * A PPG pulse train at @p bpm, in the units Dsp_PrCompute actually sees:
 * norm_pct, peak-to-peak about 1.5%, centred near zero by the DC filter.
 *
 * The dicrotic notch is the point of the shape. It is a real secondary peak at
 * @p notch_frac of the systolic height, 280ms later -- inside any plausible
 * beat interval, so the refractory alone cannot reject it. If the amplitude
 * threshold is ever loosened, this test reports double the true rate.
 */
static void synth(float bpm, float notch_frac, float jitter_frac)
{
	float period = 60.0f * (float) DSP_PPG_FS_HZ / bpm; /* samples */
	float notch_delay = 0.28f * (float) DSP_PPG_FS_HZ;
	for (uint16_t i = 0; i < DSP_PR_WINDOW; ++i) {
		buf[i] = -0.2f;
	}
	/* Jitter alternates in sign so the MEDIAN interval stays at `period` and
	 * only the spread changes -- otherwise a jitter test would also be a rate
	 * test and a failure would not say which broke. */
	int beat = 0;
	for (float t = 5.0f; t < (float) DSP_PR_WINDOW; t += period) {
		float shift = ((beat & 1) ? +1.0f : -1.0f) * jitter_frac * period;
		float centre = t + shift;
		++beat;
		for (uint16_t i = 0; i < DSP_PR_WINDOW; ++i) {
			float d = (float) i - centre;
			/* Systolic: 100ms half-width raised cosine. */
			if (d > -10.0f && d < 10.0f) {
				buf[i] += 1.3f * 0.5f * (1.0f + cosf(PR_PI * d / 10.0f));
			}
			float dn = d - notch_delay;
			if (dn > -8.0f && dn < 8.0f) {
				buf[i] += 1.3f * notch_frac * 0.5f
						* (1.0f + cosf(PR_PI * dn / 8.0f));
			}
		}
	}
}

static Dsp_PrResult run(float bpm, float notch_frac, float jitter_frac,
		uint16_t head)
{
	Dsp_PrResult r;
	synth(bpm, notch_frac, jitter_frac);
	stage(buf, DSP_PR_WINDOW, head);
	Dsp_PrCompute(&r);
	return r;
}

int main(void)
{
	/* ---- the rate itself, across the triage range ------------------------ */
	/* 4s of window at 40bpm is under 3 intervals, which is the floor, so the
	 * low end of what this can report is set by DSP_PR_WINDOW and not by
	 * DSP_HR_MIN_BPM. Worth knowing before someone wonders why a bradycardic
	 * patient reads 0. */
	const float rates[] = { 50.0f, 60.0f, 75.0f, 100.0f, 150.0f, 190.0f };
	for (unsigned i = 0; i < sizeof(rates) / sizeof(rates[0]); ++i) {
		Dsp_PrResult r = run(rates[i], 0.0f, 0.0f, 0);
		printf("  %6.1f bpm -> %3u bpm, %u pulses, spread %.3f, regular %u\n",
				rates[i], r.bpm, r.pulses, (double) r.spread, r.regular);
		assert(r.regular == 1);
		/* +-7 bpm: the interval is an integer number of 100Hz samples, so at
		 * 190bpm one sample of quantisation is already 6.2bpm. */
		assert(fabsf((float) r.bpm - rates[i]) <= 7.0f);
	}

	/* ---- above DSP_HR_MAX_BPM the rate is refused, not reported ---------- */
	/* 200bpm used to be the top of the sweep above and passed. It is now over
	 * the ceiling, and the ceiling moved for a measured reason: above ~208bpm
	 * the +-DSP_PR_SPREAD_FRAC window's lower edge falls inside the detector's
	 * own refractory, so no interval can fail low and RateIsRegular() loses all
	 * discriminating power. A lead-off ECG published 214 and 218bpm through
	 * exactly that hole. Asserting the refusal here is what stops someone
	 * "fixing" the sweep by raising the ceiling back.
	 *
	 * pulses stays populated: this is a real measurement of an implausible
	 * rate, which the caller must be able to tell from no signal at all. */
	Dsp_PrResult fast = run(210.0f, 0.0f, 0.0f, 0);
	printf("  210.0 bpm -> %3u bpm, %u pulses, regular %u (over the ceiling)\n",
			fast.bpm, fast.pulses, fast.regular);
	assert(fast.bpm == 0);
	assert(fast.pulses > 0);

	/* ---- the dicrotic notch must not be counted ------------------------- */
	/* 50% is the top of the physiological range for the notch. If this reports
	 * ~150 instead of 75, DSP_PR_THRESH_FRAC has been lowered past it. */
	for (float nf = 0.0f; nf <= 0.55f; nf += 0.1f) {
		Dsp_PrResult r = run(75.0f, nf, 0.0f, 0);
		printf("  notch %.0f%% -> %3u bpm (%u pulses)\n", (double) (nf * 100.0f),
				r.bpm, r.pulses);
		assert(r.bpm >= 73 && r.bpm <= 77);
	}

	/* ---- irregular intervals are refused, not averaged ------------------ */
	Dsp_PrResult reg = run(75.0f, 0.3f, 0.05f, 0);
	printf("  jitter 5%%  -> %3u bpm, spread %.3f, regular %u\n", reg.bpm,
			(double) reg.spread, reg.regular);
	assert(reg.regular == 1);
	assert(reg.bpm >= 70 && reg.bpm <= 80);

	Dsp_PrResult irr = run(75.0f, 0.3f, 0.30f, 0);
	printf("  jitter 30%% -> %3u bpm, spread %.3f, regular %u, pulses %u\n",
			irr.bpm, (double) irr.spread, irr.regular, irr.pulses);
	assert(irr.spread > DSP_PR_SPREAD_FRAC);
	assert(irr.regular == 0);
	/* The whole point: no number reaches the caller, but the evidence that
	 * something WAS there does. Reporting 0 pulses here would be a bug of a
	 * different kind -- indistinguishable from no finger. */
	assert(irr.bpm == 0);
	assert(irr.pulses >= 3);

	/* ---- ONE knock must not discard the window --------------------------- */
	/* This is the case that decides whether the detector works on a sensor that
	 * is taped rather than clipped, and it is why the amplitude reference is a
	 * median over quarters and the regularity test is a majority vote. Both
	 * halves fail without both fixes: a spike 8x the pulse height sets a
	 * window-maximum threshold above every real beat (so nothing is detected at
	 * all), and the spike's own two intervals fail a unanimous agreement test
	 * (so the correct median is thrown away).
	 *
	 * 8x, mid-window, one sample wide -- a finger bumped once in 4 seconds. The
	 * true rate is still 75, and 75 is what must come out. */
	Dsp_PrResult knock;
	synth(75.0f, 0.3f, 0.0f);
	buf[201] = 8.0f * buf[5]; /* buf[5] is a systolic peak; see synth() */
	stage(buf, DSP_PR_WINDOW, 0);
	Dsp_PrCompute(&knock);
	printf("  one knock  -> %3u bpm, %u pulses, %u agree, spread %.3f, regular %u\n",
			knock.bpm, knock.pulses, knock.agree, (double) knock.spread,
			knock.regular);
	assert(knock.bpm >= 70 && knock.bpm <= 80);
	assert(knock.regular == 1);
	/* The evidence that it happened must survive: `spread` still reports the
	 * knock even though the gate forgave it. A test that only checked bpm would
	 * pass just as well with the diagnostic thrown away. */
	assert(knock.spread > DSP_PR_SPREAD_FRAC);
	assert(knock.agree >= DSP_PR_MIN_INTERVALS);
	assert(knock.agree < knock.pulses); /* the knock's own intervals dissented */

	/* ---- the shared verdict, which the ECG path also goes through --------- */
	/* RateIsRegular() is tested directly because the case that matters cannot be
	 * staged through the PPG ring at all: it is a LEAD-OFF ECG, where isolated
	 * cable transients produce one or two intervals that used to be published as
	 * a rate. The ECG detector itself is not host-testable (it is all arm_*),
	 * so this is where its gate gets checked.
	 *
	 * The rule for both paths: min_agree intervals within DSP_PR_SPREAD_FRAC of
	 * the median, and a strict majority. */
	uint32_t na = 99;
	/* Two transients 2s apart at 500Hz: the single interval that read as exactly
	 * DSP_HR_MIN_BPM. DSP_ECG_MIN_INTERVALS rejects it before this is reached,
	 * but if that floor is ever lowered the majority test must still refuse it. */
	float32_t one[1] = { 1000.0f };
	assert(RateIsRegular(one, 1, 1000.0f, DSP_ECG_MIN_INTERVALS, &na) == 0);
	assert(na == 1); /* it agrees with itself; agreement was never the question */
	/* Three transients, wildly unequal spacing: a median exists, nothing backs
	 * it. This is the shape a dangling electrode actually produces. */
	float32_t junk[3] = { 120.0f, 1000.0f, 340.0f };
	assert(RateIsRegular(junk, 3, 340.0f, DSP_ECG_MIN_INTERVALS, &na) == 0);
	assert(na == 1);
	/* A real 75bpm rhythm at 500Hz, one beat missed in the middle (the doubled
	 * interval): 3 of 4 agree, which is both the floor and a majority. */
	float32_t sinus[4] = { 400.0f, 800.0f, 396.0f, 404.0f };
	assert(RateIsRegular(sinus, 4, 402.0f, DSP_ECG_MIN_INTERVALS, &na) == 1);
	assert(na == 3);
	/* Exactly at the floor with no majority: 3 agree, 3 do not. Refused, because
	 * min_agree alone would have passed this. */
	float32_t split[6] = { 400.0f, 401.0f, 402.0f, 900.0f, 150.0f, 1300.0f };
	assert(RateIsRegular(split, 6, 401.0f, DSP_ECG_MIN_INTERVALS, &na) == 0);
	assert(na == 3);
	/* A zero or negative median cannot be agreed with by anything. */
	assert(RateIsRegular(sinus, 4, 0.0f, DSP_ECG_MIN_INTERVALS, &na) == 0);
	assert(na == 0);
	printf("  RateIsRegular: lead-off shapes refused, sinus with a dropped beat accepted\n");

	/* ---- nothing on the sensor ------------------------------------------ */
	Dsp_PrResult flat;
	for (uint16_t i = 0; i < DSP_PR_WINDOW; ++i) {
		buf[i] = 0.0f;
	}
	stage(buf, DSP_PR_WINDOW, 0);
	Dsp_PrCompute(&flat);
	assert(flat.bpm == 0 && flat.pulses == 0);

	/* Contact floor gates norm_pct to exactly 0, so a no-finger window is all
	 * zeros and `peak <= 0` short-circuits. A negative-only window (baseline
	 * above signal, which the settling DC filter produces) must also not
	 * produce a rate rather than finding a "peak" among negatives. */
	for (uint16_t i = 0; i < DSP_PR_WINDOW; ++i) {
		buf[i] = -0.5f - 0.01f * (float) (i % 7);
	}
	stage(buf, DSP_PR_WINDOW, 0);
	Dsp_PrCompute(&flat);
	assert(flat.bpm == 0 && flat.pulses == 0);

	/* ---- a partial window is not a rate --------------------------------- */
	synth(75.0f, 0.3f, 0.0f);
	stage(buf, DSP_PR_WINDOW, 0);
	pr_filled = DSP_PR_WINDOW - 1U;
	Dsp_PrCompute(&flat);
	assert(flat.bpm == 0 && flat.pulses == 0);

	/* ---- the ring wraps ------------------------------------------------- */
	/* PrAt() is the only place the ring is unrolled, and a head of 0 exercises
	 * none of it. The rate must not depend on where the window happens to be
	 * split, so the same signal at several heads must give the same answer. */
	uint16_t heads[] = { 0, 1, 137, DSP_PR_WINDOW / 2, DSP_PR_WINDOW - 1 };
	uint16_t first = 0;
	for (unsigned i = 0; i < sizeof(heads) / sizeof(heads[0]); ++i) {
		Dsp_PrResult r = run(75.0f, 0.3f, 0.0f, heads[i]);
		printf("  head %3u -> %3u bpm\n", heads[i], r.bpm);
		if (i == 0) {
			first = r.bpm;
			assert(first != 0);
		} else {
			/* Not exact equality: rotating the ring moves which beat falls off
			 * the edge, so one interval can enter or leave the median. One bpm
			 * of drift is that, not a wraparound bug. */
			assert(abs((int) r.bpm - (int) first) <= 1);
		}
	}

	/* ---- NULL ----------------------------------------------------------- */
	Dsp_PrCompute(NULL); /* must not fault */

	printf("ppg_pr_selftest: all assertions passed\n");
	return 0;
}
