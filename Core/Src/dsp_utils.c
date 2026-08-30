/**
 ******************************************************************************
 * @file    dsp_utils.c
 * @brief   Hardware-accelerated vital-sign DSP for STM32F411 (Cortex-M4F).
 *
 * All filtering uses CMSIS-DSP direct-form-1 biquad cascades, which compile to
 * the M4's single-cycle VFPv4 multiply-accumulate. Coefficients are RBJ
 * cookbook designs (Butterworth Q = 1/sqrt(2)) laid out in the CMSIS order
 * {b0, b1, b2, -a1, -a2} per stage. Every design was checked for pole radius
 * < 1 and verified against its magnitude response before being pasted here.
 ******************************************************************************
 */

#include "dsp_utils.h"
#include <math.h>
#include <string.h>

/* ---- Filter coefficients ------------------------------------------------ */

/* ECG bandpass, 5-15Hz at 500Hz. Rejects baseline wander (-28dB at 1Hz) and
 * mains hum (-21dB at 50Hz) while passing the QRS complex (-1dB at 10Hz). */
static const float32_t ecg_bp_coeffs[10] = { +0.9565432256f, -1.9130864511f,
		+0.9565432256f, +1.9111970674f, -0.9149758348f, +0.0078202080f,
		+0.0156404161f, +0.0078202080f, +1.7347257688f, -0.7660066009f };

/* Mic highpass, 20Hz at 500Hz. Strips the DC bias of the electret bias
 * network and any handling thump below the breath-sound band. */
static const float32_t mic_hp_coeffs[5] = { +0.8370891906f, -1.6741783811f,
		+0.8370891906f, +1.6474599811f, -0.7008967812f };

/* Envelope lowpass, 2Hz x2 at 500Hz. Follows the breath-sound amplitude and
 * doubles as the anti-alias filter for the 50x decimation to 10Hz: -32dB at
 * 5Hz, the decimated Nyquist. */
static const float32_t mic_env_coeffs[10] = { +0.0001551484f, +0.0003102968f,
		+0.0001551484f, +1.9644605802f, -0.9650811739f, +0.0001551484f,
		+0.0003102968f, +0.0001551484f, +1.9644605802f, -0.9650811739f };

/* Envelope highpass, 0.06Hz at the DECIMATED 10Hz rate. 0.06Hz = 3.6 brpm, so
 * it sits below DSP_RR_MIN_BRPM and does not touch any rate this device will
 * report: measured -0.53dB at 6 brpm, -0.07dB at 10, -0.02dB at 14.8, flat
 * above. What it removes is slow drift in overall loudness - a patient
 * shifting, a rescuer walking past, ambient level wandering across the 25.6s
 * window. On real hardware that drift produced a 687-magnitude peak at
 * 5.33 brpm which beat the genuine 144-magnitude breath peak, and the reading
 * was lost. Attenuating it here is better than rejecting it after the FFT,
 * because it also stops it inflating the in-band mean that the peak-to-noise
 * gate is measured against. Pole radius 0.9737. */
static const float32_t mic_drift_hp_coeffs[5] = { +0.9736948117f, -1.9473896234f,
		+0.9736948117f, +1.9466975408f, -0.9480817061f };

/* PPG smoothing FIR, 31-tap Hamming lowpass, 10Hz at 100Hz. Windowed sinc,
 * normalised to unity DC gain (so it cannot shift the perfusion baseline).
 * Measured response: -0.01dB at 1.2Hz, -0.10dB at 5Hz, -6.05dB at 10Hz,
 * -40.6dB at 15Hz, below -63dB everywhere above 20Hz. White noise comes out at
 * 0.42x amplitude. Symmetric, so the delay is a constant 15 samples.
 * Regenerate with the design script in tools/ppg_fir_selftest.c's header
 * comment if the corner ever needs to move. */
static const float32_t ppg_lp_coeffs[DSP_PPG_LP_TAPS] = {
		+0.0000000000f, +0.0012014264f, +0.0027843278f, +0.0042273161f,
		+0.0039427634f, +0.0000000000f, -0.0082567773f, -0.0185832101f,
		-0.0253898204f, -0.0212353087f, +0.0000000000f, +0.0395881126f,
		+0.0918888770f, +0.1450990810f, +0.1849028786f, +0.1996606673f,
		+0.1849028786f, +0.1450990810f, +0.0918888770f, +0.0395881126f,
		+0.0000000000f, -0.0212353087f, -0.0253898204f, -0.0185832101f,
		-0.0082567773f, +0.0000000000f, +0.0039427634f, +0.0042273161f,
		+0.0027843278f, +0.0012014264f, +0.0000000000f };

/* PPG DC lowpass, 0.5Hz at 100Hz: the perfusion baseline. */
static const float32_t ppg_dc_coeffs[5] = { +0.0002413590f, +0.0004827181f,
		+0.0002413590f, +1.9555782403f, -0.9565436765f };

/* PPG AC bandpass, 0.5-5Hz at 100Hz: the pulsatile component, 30-300bpm. */
static const float32_t ppg_ac_coeffs[10] = { +0.9780304792f, -1.9560609584f,
		+0.9780304792f, +1.9555782403f, -0.9565436765f, +0.0200833656f,
		+0.0401667311f, +0.0200833656f, +1.5610180758f, -0.6413515381f };

/* ---- ECG state ---------------------------------------------------------- */
static arm_biquad_casd_df1_inst_f32 ecg_bp;
static float32_t ecg_bp_state[8]; /* 4 per stage */
/* Not static, and named mon_ so CubeMonitor lists it: this is both the working
 * buffer and the only place the filtered ECG waveform exists. Declared in
 * dsp_utils.h -- read the contract there before plotting it. */
float32_t mon_ecg_filtered[DSP_ECG_WINDOW];
static float32_t ecg_energy[DSP_ECG_WINDOW];

/* ---- Mic / respiratory state -------------------------------------------- */
static arm_biquad_casd_df1_inst_f32 mic_hp;
static float32_t mic_hp_state[4];
static arm_biquad_casd_df1_inst_f32 mic_env;
static float32_t mic_env_state[8];
/* Runs at 10Hz, after decimation, not at 500Hz. */
static arm_biquad_casd_df1_inst_f32 mic_drift_hp;
static float32_t mic_drift_hp_state[4];

static float32_t resp_window[DSP_RESP_FFT_LEN]; /* circular */
static float32_t resp_hann[DSP_RESP_FFT_LEN];
static float32_t resp_fft_in[DSP_RESP_FFT_LEN];
static float32_t resp_fft_out[DSP_RESP_FFT_LEN];
static float32_t resp_mag[DSP_RESP_FFT_LEN / 2];
static arm_rfft_fast_instance_f32 resp_fft;
static uint16_t resp_head; /* next write index */
static uint16_t resp_filled; /* saturates at DSP_RESP_FFT_LEN */
static uint16_t resp_decim_count;
static uint16_t resp_hop_count;
static volatile uint8_t resp_ready;

/* ---- SpO2 state --------------------------------------------------------- */
static arm_fir_instance_f32 ir_lp_f, red_lp_f;
/* numTaps + blockSize - 1, with blockSize 1. */
static float32_t ir_lp_state[DSP_PPG_LP_TAPS], red_lp_state[DSP_PPG_LP_TAPS];
static arm_biquad_casd_df1_inst_f32 ir_dc_f, red_dc_f, ir_ac_f, red_ac_f;
static float32_t ir_dc_state[4], red_dc_state[4];
static float32_t ir_ac_state[8], red_ac_state[8];
static float32_t ir_ac_buf[DSP_PPG_BLOCK], red_ac_buf[DSP_PPG_BLOCK];
static float32_t ir_dc_acc, red_dc_acc;
static uint16_t ppg_count;
static uint16_t ppg_warmup; /* counts down from DSP_PPG_WARMUP */
static volatile uint8_t ppg_ready;

/* ---- Pulse rate state --------------------------------------------------- */
/* Ring of the normalised pulse (norm_pct), newest at pr_head. Separate from
 * ir_ac_buf even though both hold a pulse, because they need different lengths
 * for different reasons: the SpO2 block is 128 samples so the ratio-of-ratios
 * updates often, and 128 samples is only 1-2 beats - too few for a median of
 * intervals. Sharing one buffer would force one of the two to be wrong. */
static float32_t pr_window[DSP_PR_WINDOW];
static uint16_t pr_head;
static uint16_t pr_filled; /* saturates at DSP_PR_WINDOW */

void Dsp_Init(void)
{
	memset(ecg_bp_state, 0, sizeof(ecg_bp_state));
	arm_biquad_cascade_df1_init_f32(&ecg_bp, 2, ecg_bp_coeffs, ecg_bp_state);

	memset(mic_hp_state, 0, sizeof(mic_hp_state));
	memset(mic_env_state, 0, sizeof(mic_env_state));
	memset(mic_drift_hp_state, 0, sizeof(mic_drift_hp_state));
	arm_biquad_cascade_df1_init_f32(&mic_hp, 1, mic_hp_coeffs, mic_hp_state);
	arm_biquad_cascade_df1_init_f32(&mic_env, 2, mic_env_coeffs, mic_env_state);
	arm_biquad_cascade_df1_init_f32(&mic_drift_hp, 1, mic_drift_hp_coeffs,
			mic_drift_hp_state);

	memset(ir_dc_state, 0, sizeof(ir_dc_state));
	memset(red_dc_state, 0, sizeof(red_dc_state));
	memset(ir_ac_state, 0, sizeof(ir_ac_state));
	memset(red_ac_state, 0, sizeof(red_ac_state));
	/* arm_fir_init_f32 zeroes the state buffer itself. */
	arm_fir_init_f32(&ir_lp_f, DSP_PPG_LP_TAPS, ppg_lp_coeffs, ir_lp_state, 1);
	arm_fir_init_f32(&red_lp_f, DSP_PPG_LP_TAPS, ppg_lp_coeffs, red_lp_state, 1);
	arm_biquad_cascade_df1_init_f32(&ir_dc_f, 1, ppg_dc_coeffs, ir_dc_state);
	arm_biquad_cascade_df1_init_f32(&red_dc_f, 1, ppg_dc_coeffs, red_dc_state);
	arm_biquad_cascade_df1_init_f32(&ir_ac_f, 2, ppg_ac_coeffs, ir_ac_state);
	arm_biquad_cascade_df1_init_f32(&red_ac_f, 2, ppg_ac_coeffs, red_ac_state);

	memset(resp_window, 0, sizeof(resp_window));
	resp_head = 0;
	resp_filled = 0;
	resp_decim_count = 0;
	resp_hop_count = 0;
	resp_ready = 0;

	ir_dc_acc = 0.0f;
	red_dc_acc = 0.0f;
	ppg_count = 0;
	ppg_warmup = DSP_PPG_WARMUP;
	ppg_ready = 0;

	memset(pr_window, 0, sizeof(pr_window));
	pr_head = 0;
	pr_filled = 0;

	/* Periodic Hann window, w[n] = 0.5 - 0.5*cos(2*pi*n/N). arm_cos_f32 takes
	 * radians (it scales by 1/2pi internally). */
	for (uint32_t n = 0; n < DSP_RESP_FFT_LEN; ++n) {
		resp_hann[n] = 0.5f
				- 0.5f * arm_cos_f32(
						(2.0f * PI * (float32_t) n) / (float32_t) DSP_RESP_FFT_LEN);
	}

	arm_rfft_fast_init_f32(&resp_fft, DSP_RESP_FFT_LEN);
}

/* ------------------------------------------------------------------------- */
/*  ECG                                                                      */
/* ------------------------------------------------------------------------- */

/** Median by insertion sort. n is <= ~10 here, so this beats anything clever. */
static float32_t MedianF32(float32_t *v, uint32_t n)
{
	for (uint32_t i = 1; i < n; ++i) {
		float32_t key = v[i];
		uint32_t j = i;
		while (j > 0U && v[j - 1U] > key) {
			v[j] = v[j - 1U];
			--j;
		}
		v[j] = key;
	}
	return (n & 1U) ? v[n / 2U] : 0.5f * (v[n / 2U - 1U] + v[n / 2U]);
}

/**
 * Shared verdict for both rate detectors: a median is only a rate if the
 * intervals it came from agree with it.
 *
 * One rule in one place on purpose. The ECG and PR paths drifted apart once
 * already -- PR demanded three agreeing intervals while the ECG published a
 * rate from a single interval with no agreement test at all, and on a lead-off
 * AD8232 that turned two dangling-cable transients 2s apart into a confident
 * "30 bpm" on the triage screen. Whatever this rule should be, both paths have
 * to be wrong together or right together.
 *
 * @p min_agree is the only difference left between the callers, because their
 * windows hold different numbers of beats. @p agree_out may be NULL.
 * @return 1 when at least @p min_agree intervals are within
 *         DSP_PR_SPREAD_FRAC of @p median AND they are a strict majority.
 */
static uint8_t RateIsRegular(const float32_t *iv, uint32_t n, float32_t median,
		uint32_t min_agree, uint32_t *agree_out)
{
	uint32_t agree = 0;

	if (median > 0.0f) {
		for (uint32_t i = 0; i < n; ++i) {
			float32_t d = iv[i] - median;
			if (d < 0.0f) {
				d = -d;
			}
			if (d <= (DSP_PR_SPREAD_FRAC * median)) {
				++agree;
			}
		}
	}
	if (agree_out != NULL) {
		*agree_out = agree;
	}
	/* Majority as well as a floor: min_agree alone would let 3 agreeing
	 * intervals carry a window that also held 9 dissenting ones. */
	return (agree >= min_agree && (agree * 2U) > n) ? 1U : 0U;
}

void Dsp_EcgProcessWindow(const uint16_t *raw, uint32_t len, Dsp_EcgResult *out)
{
	if (out == NULL) {
		return;
	}
	out->bpm = 0;
	out->mean_raw = 0;
	out->rms_filtered = 0.0f;
	out->beats = 0;

	if (raw == NULL || len < (DSP_ADC_FS_HZ / 2U)) {
		return; /* under 0.5s cannot hold an R-R interval */
	}
	if (len > DSP_ECG_WINDOW) {
		len = DSP_ECG_WINDOW;
	}

	for (uint32_t i = 0; i < len; ++i) {
		mon_ecg_filtered[i] = (float32_t) raw[i];
	}

	float32_t mean_raw = 0.0f;
	arm_mean_f32(mon_ecg_filtered, len, &mean_raw);
	out->mean_raw = (uint16_t) mean_raw;

	/* Centre the block before filtering. Without this the highpass sees the
	 * ~2048 ADC offset as a step and its transient dwarfs every QRS complex. */
	arm_offset_f32(mon_ecg_filtered, -mean_raw, mon_ecg_filtered, len);

	/* Bandpass in place, then square to get an energy envelope in which the
	 * QRS complex dominates far more sharply than in the raw signal. */
	arm_biquad_cascade_df1_f32(&ecg_bp, mon_ecg_filtered, mon_ecg_filtered, len);
	arm_rms_f32(mon_ecg_filtered, len, &out->rms_filtered);
	arm_mult_f32(mon_ecg_filtered, mon_ecg_filtered, ecg_energy, len);

	/* Absolute amplitude gate. The peak-to-mean ratio test below is scale
	 * invariant, so pure noise can pass it whenever a random sample happens
	 * to sit well above the average; this rejects that case outright. */
	if (out->rms_filtered < DSP_ECG_MIN_RMS) {
		return;
	}

	/* Skip the filter settling region: 2 stages at 5Hz need ~100ms to ring
	 * out, and residual transient there would set a bogus threshold. */
	const uint32_t settle = DSP_ADC_FS_HZ / 10U;
	if (len <= (settle + 2U)) {
		return;
	}
	const float32_t *energy = &ecg_energy[settle];
	const uint32_t elen = len - settle;

	float32_t peak_energy = 0.0f;
	uint32_t peak_index = 0;
	arm_max_f32(energy, elen, &peak_energy, &peak_index);

	float32_t mean_energy = 0.0f;
	arm_mean_f32(energy, elen, &mean_energy);

	/* A flat or lead-off trace has no peak standing clear of the mean. */
	if (peak_energy <= 0.0f || peak_energy < (8.0f * mean_energy)) {
		return;
	}

	const float32_t threshold = 0.35f * peak_energy;
	/* 200ms refractory caps detection at 300bpm and stops the R and T waves
	 * of one beat being counted separately. */
	const uint32_t refractory = (DSP_ADC_FS_HZ * 200U) / 1000U;
	const uint32_t max_intervals = 16U;
	float32_t intervals[16];
	uint32_t interval_count = 0;
	uint32_t beats = 0;
	int32_t last_peak = -(int32_t) refractory - 1;

	for (uint32_t i = 1; i + 1U < elen; ++i) {
		if (energy[i] > threshold && energy[i] >= energy[i - 1U]
				&& energy[i] >= energy[i + 1U]
				&& ((int32_t) i - last_peak) > (int32_t) refractory) {
			if (last_peak >= 0 && interval_count < max_intervals) {
				intervals[interval_count++] = (float32_t) ((int32_t) i - last_peak);
			}
			last_peak = (int32_t) i;
			++beats;
		}
	}

	out->beats = (uint8_t) ((beats > 255U) ? 255U : beats);
	if (interval_count < DSP_ECG_MIN_INTERVALS) {
		return;
	}

	/* Median, not mean: one missed or doubled beat skews a mean badly but
	 * leaves the median of 3+ intervals intact. */
	float32_t median_samples = MedianF32(intervals, interval_count);
	if (median_samples <= 0.0f) {
		return;
	}

	float32_t bpm = (60.0f * DSP_ADC_FS_TRUE_HZ) / median_samples;
	if (bpm < DSP_HR_MIN_BPM || bpm > DSP_HR_MAX_BPM) {
		return;
	}
	/* Same agreement test the PR path applies, and for the same reason: a
	 * plausible number from intervals that do not agree with each other is not
	 * a measurement. On a lead-off AD8232 this is the gate that matters --
	 * two cable transients seconds apart clear the energy test and produce an
	 * interval in range, and without this they are published as bradycardia. */
	if (!RateIsRegular(intervals, interval_count, median_samples,
			DSP_ECG_MIN_INTERVALS, NULL)) {
		return;
	}
	out->bpm = (uint16_t) (bpm + 0.5f);
}

/* ------------------------------------------------------------------------- */
/*  Respiratory rate from breathing sound                                    */
/* ------------------------------------------------------------------------- */

void Dsp_MicPushSample(uint16_t raw)
{
	float32_t x = (float32_t) raw;
	float32_t hp = 0.0f;

	/* Highpass to centre the signal, then full-wave rectify: the rectified
	 * magnitude is what carries the breath-sound envelope. */
	arm_biquad_cascade_df1_f32(&mic_hp, &x, &hp, 1);
	float32_t rectified = (hp < 0.0f) ? -hp : hp;

	float32_t envelope = 0.0f;
	arm_biquad_cascade_df1_f32(&mic_env, &rectified, &envelope, 1);

	/* Decimate 500Hz -> 10Hz. The 2Hz cascade above is the anti-alias. */
	if (++resp_decim_count < DSP_RESP_DECIM) {
		return;
	}
	resp_decim_count = 0;

	/* Strip slow drift at the decimated rate, where a 0.06Hz corner is a
	 * well-conditioned biquad; at 500Hz the same corner would need a pole
	 * radius of 0.9995 and would lose precision in float32. */
	float32_t detrended = 0.0f;
	arm_biquad_cascade_df1_f32(&mic_drift_hp, &envelope, &detrended, 1);

	resp_window[resp_head] = detrended;
	resp_head = (uint16_t) ((resp_head + 1U) % DSP_RESP_FFT_LEN);
	if (resp_filled < DSP_RESP_FFT_LEN) {
		++resp_filled;
	}

	if (++resp_hop_count >= DSP_RESP_HOP) {
		resp_hop_count = 0;
		if (resp_filled >= DSP_RESP_FFT_LEN) {
			resp_ready = 1;
		}
	}
}

uint8_t Dsp_RespReady(void)
{
	return resp_ready;
}

/** Parabolic interpolation across a magnitude peak, returning the sub-bin
 *  offset in (-0.5, +0.5). The true frequency rarely sits on a bin centre. */
static float32_t PeakOffset(const float32_t *mag, uint32_t k)
{
	float32_t y0 = mag[k - 1U];
	float32_t y1 = mag[k];
	float32_t y2 = mag[k + 1U];
	float32_t denom = y0 - (2.0f * y1) + y2;
	if (denom == 0.0f) {
		return 0.0f;
	}
	float32_t d = 0.5f * (y0 - y2) / denom;
	return (d > 0.5f || d < -0.5f) ? 0.0f : d;
}

/** Fills dbg->top_brpm/top_mag with the DSP_RESP_TOP_N strongest local maxima
 *  in [lo,hi], descending, and @p bins with the bin index of each. Local maxima
 *  only, so one broad peak does not fill every slot with its own shoulders. */
static void CollectTopPeaks(Dsp_RespDebug *dbg, uint32_t *bins, uint32_t lo,
		uint32_t hi, float32_t bin_hz)
{
	for (uint32_t k = lo; k <= hi; ++k) {
		/* Skip anything that is not a local maximum. */
		if (k > 0U && resp_mag[k] < resp_mag[k - 1U]) {
			continue;
		}
		if (resp_mag[k] < resp_mag[k + 1U]) {
			continue;
		}
		/* Insertion sort into the descending top-N list. */
		for (uint32_t s = 0; s < DSP_RESP_TOP_N; ++s) {
			if (resp_mag[k] > dbg->top_mag[s]) {
				for (uint32_t m = DSP_RESP_TOP_N - 1U; m > s; --m) {
					dbg->top_mag[m] = dbg->top_mag[m - 1U];
					dbg->top_brpm[m] = dbg->top_brpm[m - 1U];
					bins[m] = bins[m - 1U];
				}
				dbg->top_mag[s] = resp_mag[k];
				dbg->top_brpm[s] = ((float32_t) k + PeakOffset(resp_mag, k))
						* bin_hz * 60.0f;
				bins[s] = k;
				break;
			}
		}
	}
}

/**
 * Turns one spectral peak into a breathing rate, resolving the octave ambiguity.
 *
 * A peak at frequency f could be a breathing rate of f (only one phase audible)
 * or of f/2 (both phases audible, the usual case). Look for energy at f/2:
 * asymmetric breathing, where one phase is louder, leaves a real bump at the
 * true fundamental. If it is there, that is the rate. If it is absent and the
 * peak is above the plausible maximum, the two phases were near-symmetric and
 * cancelled the fundamental, so the peak must still be the doubled rate.
 *
 * @p halved and @p sub_found receive the decision taken, for diagnostics.
 */
static float32_t ResolveOctave(uint32_t peak_bin, uint32_t min_bin,
		uint32_t max_bin, float32_t bin_hz, uint8_t *halved, uint8_t *sub_found)
{
	const float32_t peak_brpm = ((float32_t) peak_bin
			+ PeakOffset(resp_mag, peak_bin)) * bin_hz * 60.0f;
	*halved = 0;
	*sub_found = 0;

	const uint32_t half_bin = peak_bin / 2U;
	if (half_bin >= min_bin && half_bin >= 1U) {
		/* Take the strongest of the three bins around f/2: the subharmonic can
		 * straddle a boundary when peak_bin is odd. */
		float32_t sub_mag = resp_mag[half_bin];
		uint32_t sub_bin = half_bin;
		for (uint32_t k = (half_bin > 1U) ? (half_bin - 1U) : 1U;
				k <= half_bin + 1U && k <= max_bin; ++k) {
			if (resp_mag[k] > sub_mag) {
				sub_mag = resp_mag[k];
				sub_bin = k;
			}
		}
		float32_t sub_brpm = ((float32_t) sub_bin
				+ PeakOffset(resp_mag, sub_bin)) * bin_hz * 60.0f;

		/* The candidate must actually be at half the peak frequency. The +/-1
		 * bin window is generous at low bin numbers, where one bin is a large
		 * fraction of the frequency: measured on the host, a true 10 brpm
		 * peaked at 9.87 (bin 4) and the window reached bin 3 (7.03 brpm),
		 * which is nowhere near 4.93 and was being accepted, halving a
		 * correct reading into a wrong one. Compare in frequency, not bins. */
		float32_t want = 0.5f * peak_brpm;
		float32_t err = sub_brpm - want;
		if (err < 0.0f) {
			err = -err;
		}
		const uint8_t at_half = (want > 0.0f) && ((err / want) < 0.15f);

		if (at_half && sub_brpm >= DSP_RR_MIN_BRPM
				&& sub_mag >= (DSP_RR_SUBHARM_RATIO * resp_mag[peak_bin])) {
			*halved = 1;
			*sub_found = 1;
			return sub_brpm;
		}
	}

	/* No usable subharmonic. If the peak itself is a plausible rate, take it;
	 * otherwise symmetric inhale/exhale cancelled the fundamental, so fold. */
	if (peak_brpm > DSP_RR_MAX_BRPM) {
		*halved = 1;
		return 0.5f * peak_brpm;
	}
	return peak_brpm;
}

float Dsp_RespRateBrpm(Dsp_RespDebug *dbg)
{
	if (dbg != NULL) {
		memset(dbg, 0, sizeof(*dbg));
	}
	if (!resp_ready) {
		return 0.0f;
	}
	resp_ready = 0;

	/* Unwrap the circular buffer oldest-first. */
	for (uint32_t i = 0; i < DSP_RESP_FFT_LEN; ++i) {
		resp_fft_in[i] = resp_window[(resp_head + i) % DSP_RESP_FFT_LEN];
	}

	/* Remove the mean before windowing, or the DC term leaks into the low
	 * bins we care about and swamps the breathing peak. */
	float32_t mean = 0.0f;
	arm_mean_f32(resp_fft_in, DSP_RESP_FFT_LEN, &mean);
	arm_offset_f32(resp_fft_in, -mean, resp_fft_in, DSP_RESP_FFT_LEN);
	arm_mult_f32(resp_fft_in, resp_hann, resp_fft_in, DSP_RESP_FFT_LEN);

	arm_rfft_fast_f32(&resp_fft, resp_fft_in, resp_fft_out, 0);

	/* CMSIS packs DC in [0] and Nyquist in [1], so complex bin k sits at
	 * resp_fft_out[2k], [2k+1] for k >= 1. Magnitude index k is therefore
	 * bin k directly; index 0 is the bogus DC/Nyquist pair, and unused. */
	arm_cmplx_mag_f32(resp_fft_out, resp_mag, DSP_RESP_FFT_LEN / 2);

	const float32_t bin_hz = (float32_t) DSP_RESP_FS_HZ
			/ (float32_t) DSP_RESP_FFT_LEN;

	/* Search to 2x the max rate: the envelope fundamental is usually at twice
	 * the breathing rate because both inhale and exhale are audible.
	 *
	 * Round the band start UP. Truncating put the floor below the stated
	 * minimum: (uint32_t)((6/60)/0.0390625) is int(2.56) = 2, and bin 2 is
	 * 4.69 brpm, so the search admitted drift bins the gates were meant to
	 * exclude. +0.999f rather than a ceil() call keeps <math.h> out of this
	 * translation unit. */
	uint32_t min_bin = (uint32_t) (((DSP_RR_MIN_BRPM / 60.0f) / bin_hz)
			+ 0.999f);
	uint32_t max_bin = (uint32_t) ((DSP_RR_SEARCH_MAX_BRPM / 60.0f) / bin_hz)
			+ 1U;
	if (min_bin < 1U) {
		min_bin = 1U;
	}
	if (max_bin > (DSP_RESP_FFT_LEN / 2U) - 2U) {
		max_bin = (DSP_RESP_FFT_LEN / 2U) - 2U;
	}
	if (min_bin >= max_bin) {
		return 0.0f;
	}

	uint32_t best = min_bin;
	for (uint32_t k = min_bin + 1U; k <= max_bin; ++k) {
		if (resp_mag[k] > resp_mag[best]) {
			best = k;
		}
	}

	/* Require the peak to stand clear of the in-band average, otherwise this
	 * is just noise and a confident-looking number would be a lie. */
	float32_t band_sum = 0.0f;
	for (uint32_t k = min_bin; k <= max_bin; ++k) {
		band_sum += resp_mag[k];
	}
	float32_t band_mean = band_sum / (float32_t) (max_bin - min_bin + 1U);
	float32_t peak_ratio = (band_mean > 0.0f) ? (resp_mag[best] / band_mean)
			: 0.0f;

	/* The candidate list is needed whether or not the caller wants diagnostics,
	 * so collect into a local when dbg is NULL and copy out at the end. */
	Dsp_RespDebug local;
	Dsp_RespDebug *d = (dbg != NULL) ? dbg : &local;
	if (d == &local) {
		memset(d, 0, sizeof(*d));
	}
	uint32_t top_bin[DSP_RESP_TOP_N] = { 0 };

	d->peak_brpm = ((float32_t) best + PeakOffset(resp_mag, best))
			* bin_hz * 60.0f;
	d->peak_ratio = peak_ratio;
	CollectTopPeaks(d, top_bin, min_bin, max_bin, bin_hz);

	if (band_mean <= 0.0f || peak_ratio < 3.0f) {
		return 0.0f;
	}

	/* Try each candidate peak strongest-first and take the first whose resolved
	 * rate is plausible. Stopping at the single strongest peak is what lost the
	 * reading on real hardware: slow baseline drift is broad and can be several
	 * times stronger than the breath itself, so the strongest peak was drift at
	 * 5.33 brpm and the genuine breath at 29.61 (-> 14.8) was never looked at.
	 * The 0.06Hz envelope highpass now attenuates that drift, but a rescuer
	 * moving the patient mid-window can still put a stronger artefact in band,
	 * and giving up on the first miss throws away a good reading either way. */
	uint8_t halved = 0;
	uint8_t sub_found = 0;
	float32_t brpm = 0.0f;

	for (uint32_t c = 0; c < DSP_RESP_TOP_N; ++c) {
		if (d->top_mag[c] <= 0.0f) {
			break; /* list is descending: no further candidates exist */
		}
		/* A candidate has to stand clear of the noise floor in its own right,
		 * or a weak fourth-strongest bump could supply a confident number the
		 * strongest peak was denied. */
		if ((d->top_mag[c] / band_mean) < 3.0f) {
			break;
		}
		float32_t cand = ResolveOctave(top_bin[c], min_bin, max_bin, bin_hz,
				&halved, &sub_found);
		if (cand >= DSP_RR_MIN_BRPM && cand <= DSP_RR_MAX_BRPM) {
			brpm = cand;
			break;
		}
	}

	d->halved = halved;
	d->subharmonic_found = sub_found;

	return brpm;
}

/* ------------------------------------------------------------------------- */
/*  SpO2                                                                     */
/* ------------------------------------------------------------------------- */

void Dsp_Spo2PushSample(uint32_t ir, uint32_t red, Dsp_PpgSample *out)
{
	float32_t ir_f = (float32_t) ir;
	float32_t red_f = (float32_t) red;
	float32_t ir_s = 0.0f, red_s = 0.0f;

	/* Smooth first, so every stage below works on a waveform without the
	 * out-of-band hash: on hardware the raw stream carries ~25 counts of
	 * sample-to-sample noise on a ~200-count pulse, and noise above 5Hz still
	 * leaks through the AC bandpass and inflates its RMS, which biases the
	 * ratio-of-ratios. The FIR runs ahead of the block-full gate below so its
	 * input never has a gap and the exported waveform stays continuous. */
	arm_fir_f32(&ir_lp_f, &ir_f, &ir_s, 1);
	arm_fir_f32(&red_lp_f, &red_f, &red_s, 1);

	float32_t ir_dc = 0.0f, red_dc = 0.0f;
	arm_biquad_cascade_df1_f32(&ir_dc_f, &ir_s, &ir_dc, 1);
	arm_biquad_cascade_df1_f32(&red_dc_f, &red_s, &red_dc, 1);

	/* Normalised by the SAME baseline the pulse was taken from, not by a
	 * separately smoothed one, so the ratio is exact at every sample rather
	 * than beating between two filters with different settling. Gated on
	 * the shared contact floor: below it the "baseline" is ambient light,
	 * and pulse/tiny_baseline is a large fabricated percentage. */
	float32_t pulse = ir_s - ir_dc;
	float32_t norm_pct = (ir_dc >= DSP_PPG_MIN_DC)
			? (100.0f * pulse / ir_dc) : 0.0f;

	if (out != NULL) {
		out->ir = ir_s;
		out->red = red_s;
		out->pulse = pulse;
		out->norm_pct = norm_pct;
	}

	/* Feed the rate ring here, ahead of the block-full return below, so its
	 * timebase has no gaps: Dsp_PrCompute converts sample counts to bpm, so a
	 * dropped sample does not merely lose data, it shortens an interval and
	 * biases the rate upward. Warm-up samples are excluded because the settling
	 * 0.5Hz baseline makes `pulse` a large slow ramp that would read as one
	 * enormous beat. */
	if (ppg_warmup == 0U) {
		pr_window[pr_head] = norm_pct;
		pr_head = (uint16_t) ((pr_head + 1U) % DSP_PR_WINDOW);
		if (pr_filled < DSP_PR_WINDOW) {
			++pr_filled;
		}
	}

	if (ppg_count >= DSP_PPG_BLOCK) {
		return; /* block full, waiting on Dsp_Spo2Compute */
	}

	arm_biquad_cascade_df1_f32(&ir_ac_f, &ir_s, &ir_ac_buf[ppg_count], 1);
	arm_biquad_cascade_df1_f32(&red_ac_f, &red_s, &red_ac_buf[ppg_count], 1);

	ir_dc_acc += ir_dc;
	red_dc_acc += red_dc;

	/* Burn the warm-up samples through the filters but never let them form a
	 * block: the step from zero state to a live PPG level rings the AC
	 * bandpass hard, which would otherwise read as a huge false pulse. */
	if (ppg_warmup > 0U) {
		--ppg_warmup;
		if (++ppg_count >= DSP_PPG_BLOCK) {
			ppg_count = 0; /* discard, do not signal ready */
			ir_dc_acc = 0.0f;
			red_dc_acc = 0.0f;
		}
		return;
	}

	if (++ppg_count >= DSP_PPG_BLOCK) {
		ppg_ready = 1;
	}
}

uint8_t Dsp_Spo2Ready(void)
{
	return ppg_ready;
}

void Dsp_Spo2Compute(Dsp_Spo2Result *out)
{
	if (out == NULL) {
		return;
	}
	memset(out, 0, sizeof(*out));

	if (!ppg_ready) {
		out->reject = (ppg_warmup > 0U) ? DSP_SPO2_REJ_WARMUP
				: DSP_SPO2_REJ_NO_BLOCK;
		return;
	}

	const float32_t n = (float32_t) DSP_PPG_BLOCK;
	float32_t ir_dc = ir_dc_acc / n;
	float32_t red_dc = red_dc_acc / n;
	float32_t ir_ac = 0.0f, red_ac = 0.0f;
	arm_rms_f32(ir_ac_buf, DSP_PPG_BLOCK, &ir_ac);
	arm_rms_f32(red_ac_buf, DSP_PPG_BLOCK, &red_ac);

	/* Reset for the next block before any early return. */
	ir_dc_acc = 0.0f;
	red_dc_acc = 0.0f;
	ppg_count = 0;
	ppg_ready = 0;

	/* Publish the raw components unconditionally: when a gate rejects the
	 * block these are the only way to tell which one and why. */
	out->ir_dc = ir_dc;
	out->red_dc = red_dc;
	out->ir_ac = ir_ac;
	out->red_ac = red_ac;
	out->perfusion = (ir_dc > 0.0f) ? (ir_ac / ir_dc) : 0.0f;

	/* No finger, or the LED is not reaching the detector. RED is tested first
	 * because it is the smaller channel and so the one that trips. */
	if (red_dc < DSP_PPG_MIN_DC || ir_dc < DSP_PPG_MIN_DC) {
		out->reject = DSP_SPO2_REJ_DC_LOW;
		return;
	}
	/* Perfusion index below ~0.05% is noise, not a pulse. */
	if (out->perfusion < DSP_SPO2_MIN_PI) {
		out->reject = DSP_SPO2_REJ_PERFUSION;
		return;
	}

	float32_t r = (red_ac / red_dc) / out->perfusion;
	out->r_value = r; /* keep it visible even when out of range */
	if (r <= 0.0f || r > 3.4f) {
		out->reject = DSP_SPO2_REJ_R_RANGE;
		return;
	}

	/* Empirical Beer-Lambert approximation used by most MAX3010x reference
	 * code. Uncalibrated: treat as a trend, not a clinical reading. */
	float32_t spo2 = 110.0f - (25.0f * r);

	/*
	 * Say when the answer came from the ends of the fit rather than its middle,
	 * because on this hardware it usually does and the number looks no different.
	 *
	 * The clamp below bites at exactly r = 0.4000: r 0.40 -> 0.44 maps to
	 * 100.0 -> 99.0. Measured over 75s on a finger in a 3D-printed housing, r
	 * stayed inside 0.40-0.44 the whole time and the reported SpO2 never left
	 * 99.1-100.0 -- so the entire recording lived in that sliver with its top
	 * pinned by the clamp, which is where the sd of 0.27pp comes from. Reading 97%
	 * under this fit would need r = 0.520.
	 *
	 * r is not wrong for being 0.42; this curve was derived for 660/940nm
	 * transmission probes and this is a 660/880nm reflectance part, whose DC ratio
	 * enters r multiplicatively. Retuning 110-25r to force a nicer number without a
	 * reference oximeter would move the reading, not measure it -- a curve fitted
	 * at one point on one finger reports that value for everyone. So the honest
	 * change is to flag the saturation, not to hide it.
	 *
	 * 1.6 at the other end is where the fit reaches its 70% floor.
	 */
	out->extrapolated = (r < 0.40f || r > 1.6f) ? 1U : 0U;

	if (spo2 > 100.0f) {
		spo2 = 100.0f;
	}
	if (spo2 < 70.0f) {
		out->spo2_pct = spo2; /* visible, but not flagged valid */
		out->reject = DSP_SPO2_REJ_SPO2_LOW;
		return; /* below this the linear fit is meaningless */
	}

	out->spo2_pct = spo2;
	out->valid = 1;
	out->reject = DSP_SPO2_OK;
}

/* ------------------------------------------------------------------------- */
/*  Pulse rate from the PPG                                                  */
/* ------------------------------------------------------------------------- */

/** Oldest-first access to the ring. Once full, pr_head is the oldest sample. */
static float32_t PrAt(uint16_t i)
{
	uint16_t idx = (uint16_t) ((pr_head + i) % DSP_PR_WINDOW);
	return pr_window[idx];
}

void Dsp_PrCompute(Dsp_PrResult *out)
{
	if (out == NULL) {
		return;
	}
	memset(out, 0, sizeof(*out));

	/* Partial window only ever gives a worse estimate of the same number, and a
	 * rate that appears 2s after boot and then changes is harder to trust than
	 * one that appears late. */
	if (pr_filled < DSP_PR_WINDOW) {
		return;
	}

	/* Amplitude reference: the MEDIAN of four per-quarter maxima, NOT the maximum
	 * of the whole window. The difference is what makes this work on a sensor
	 * that is not mounted. One knock produces a spike of 10% or more where a
	 * pulse is 1%, and with a window maximum the threshold becomes 0.6 of the
	 * ARTEFACT: every real beat then falls below it, no intervals are found, and
	 * the whole 4s is thrown away because of one bad sample. Taking a median over
	 * quarters discards that outlier -- an artefact inflates one quarter's
	 * maximum and the median ignores it.
	 *
	 * Four quarters is the most that still guarantees a beat in each: a quarter
	 * is 1s, so at DSP_HR_MIN_BPM a quarter can be empty, and an empty quarter
	 * only ever lowers the reference, which costs sensitivity and not
	 * correctness -- the refractory and the agreement test below still have to
	 * pass. If DSP_PR_WINDOW stops being a multiple of 4 the remainder is simply
	 * not scanned for the reference, which is harmless. */
	const uint16_t qlen = (uint16_t) (DSP_PR_WINDOW / 4U);
	float32_t qmax[4] = { 0.0f, 0.0f, 0.0f, 0.0f };
	for (uint16_t q = 0; q < 4U; ++q) {
		for (uint16_t i = 0; i < qlen; ++i) {
			float32_t v = PrAt((uint16_t) ((q * qlen) + i));
			if (v > qmax[q]) {
				qmax[q] = v;
			}
		}
	}
	float32_t ref = MedianF32(qmax, 4U);
	if (ref <= 0.0f) {
		return; /* flat, or gated to zero by the contact floor */
	}

	const float32_t threshold = DSP_PR_THRESH_FRAC * ref;
	/* 200ms, as in the ECG path: caps detection at 300bpm, above the 220 the
	 * plausibility gate allows, so an interval that survives this and still
	 * fails DSP_HR_MAX_BPM is a real measurement of an implausible rate rather
	 * than a double-count. */
	const uint16_t refractory = (uint16_t) ((DSP_PPG_FS_HZ * 200U) / 1000U);
	const uint32_t max_intervals = 16U;
	float32_t intervals[16];
	uint32_t interval_count = 0;
	uint32_t pulses = 0;
	int32_t last_peak = -(int32_t) refractory - 1;

	for (uint16_t i = 1; (uint16_t) (i + 1U) < DSP_PR_WINDOW; ++i) {
		float32_t v = PrAt(i);
		if (v > threshold && v >= PrAt((uint16_t) (i - 1U))
				&& v >= PrAt((uint16_t) (i + 1U))
				&& ((int32_t) i - last_peak) > (int32_t) refractory) {
			if (last_peak >= 0 && interval_count < max_intervals) {
				intervals[interval_count++] = (float32_t) ((int32_t) i - last_peak);
			}
			last_peak = (int32_t) i;
			++pulses;
		}
	}

	out->pulses = (uint8_t) ((pulses > 255U) ? 255U : pulses);
	if (interval_count < DSP_PR_MIN_INTERVALS) {
		return;
	}

	float32_t median = MedianF32(intervals, interval_count);
	if (median <= 0.0f) {
		return;
	}

	/* Two separate questions, deliberately not conflated:
	 *
	 *   `spread` (reported, diagnostic) is the WIDEST deviation from the median,
	 *   because one ectopic beat among five regular ones barely moves an RMS
	 *   spread but is the whole finding. Taken against the median so a single
	 *   outlier cannot inflate the reference it is compared to.
	 *
	 *   `regular` (the gate) is a MAJORITY vote, not that worst case, and lives
	 *   in RateIsRegular() so the ECG path cannot apply a weaker one. Requiring
	 *   every interval to agree means one artefact vetoes an otherwise clean 4s
	 *   window, which on an unmounted sensor is most windows -- and the median it
	 *   would have vetoed is already correct, since a median over 4-5 intervals
	 *   is exactly what survives one bad one.
	 *
	 * This does not open the door to publishing a rate for atrial fibrillation:
	 * there no cluster holds a majority within 30%, so `regular` still comes out
	 * 0. What it stops is throwing away a regular rhythm because the finger was
	 * bumped once. */
	float32_t worst = 0.0f;
	uint32_t agree = 0;
	for (uint32_t i = 0; i < interval_count; ++i) {
		float32_t d = intervals[i] - median;
		if (d < 0.0f) {
			d = -d;
		}
		if (d > worst) {
			worst = d;
		}
	}
	out->spread = worst / median;
	out->regular = RateIsRegular(intervals, interval_count, median,
			DSP_PR_MIN_INTERVALS, &agree);
	out->agree = (uint8_t) ((agree > 255U) ? 255U : agree);

	float32_t bpm = (60.0f * (float32_t) DSP_PPG_FS_HZ) / median;
	if (bpm < DSP_HR_MIN_BPM || bpm > DSP_HR_MAX_BPM) {
		return;
	}
	/* Irregular intervals leave bpm at 0 on purpose. The median would still
	 * produce a plausible-looking number, and a plausible number is exactly
	 * what must not reach a triage screen when the input was motion artefact.
	 * out->pulses and out->spread stay populated so the caller can tell this
	 * apart from no signal at all. */
	if (!out->regular) {
		return;
	}
	out->bpm = (uint16_t) (bpm + 0.5f);
}
