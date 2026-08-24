#ifndef DSP_UTILS_H
#define DSP_UTILS_H

#include <stdint.h>

#if defined(__has_include)
#if !__has_include("arm_math.h")
#error "CMSIS-DSP not on the include path. Add Middlewares/ST/ARM/DSP/Inc and link libarm_cortexM4lf_math.a."
#endif
#endif

/* CMSIS-DSP 1.7.0 selects the M4 DSP/FPU paths from __ARM_FEATURE_DSP and
 * __FPU_USED, both set by -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard.
 * No ARM_MATH_CMx define is needed (or read) by this version. */
#include "arm_math.h"

/* ---- Sample rates (Hz) -------------------------------------------------- */
/* TIM2: 100MHz timer clock / (999+1) / (200+1) = 497.5Hz, both ADC ranks. */
#define DSP_ADC_FS_HZ 500u
/* MAX30102 at sr_800 with smp_ave_8 averages down to ~100Hz output. */
#define DSP_PPG_FS_HZ 100u

/* ---- Window sizes ------------------------------------------------------- */
/* 4s of ECG: 4-6 beats, so 3-5 R-R intervals to take a median over. */
#define DSP_ECG_WINDOW 2000u
/* Mic envelope is decimated to 10Hz; 256 pts = 25.6s, bin = 0.039Hz. */
#define DSP_RESP_FFT_LEN 256u
#define DSP_RESP_DECIM 50u
#define DSP_RESP_FS_HZ (DSP_ADC_FS_HZ / DSP_RESP_DECIM)
/* Recompute RR every 25 new envelope samples (2.5s) over the 25.6s window. */
#define DSP_RESP_HOP 25u
/* 1.28s of PPG per SpO2 estimate. */
#define DSP_PPG_BLOCK 128u
/* PPG smoothing FIR: 31-tap Hamming lowpass, 10Hz corner at 100Hz.
 * Linear phase is the reason this is an FIR and not another biquad: every
 * frequency is delayed by exactly (31-1)/2 = 15 samples (150ms), so the shape
 * and the relative timing of the systolic peak and dicrotic notch survive
 * intact. A biquad shifts each frequency differently and quietly deforms
 * exactly those features. Keep that property: the smoothed stream is what any
 * downstream waveform analysis works from, here or on the ESP32.
 * 10Hz corner: -40dB at 15Hz for only -0.10dB at 5Hz, i.e. flat through the
 * 8th harmonic of a 75bpm pulse, so the hash goes and the pulse does not. A
 * 15Hz corner is not available at this sample rate - 15Hz is 0.15 of 100Hz and
 * a 31-tap design there has no stopband left below Nyquist.
 * Verified in tools/ppg_fir_selftest.c. */
#define DSP_PPG_LP_TAPS 31u
/* Samples to discard after Dsp_Init before any block is trusted. The 0.5Hz DC
 * lowpass has a ~0.3s time constant, so from a cold start (state 0, input
 * jumping to tens of thousands of counts) it needs several seconds to settle,
 * and until it does the AC bandpass is still ringing on that step and reports
 * a large bogus AC amplitude. Measured on the host: at 300 samples the residual
 * ring still gives a perfusion index of 0.0019, inside the plausible-pulse
 * range, so it would pass the gate as a false reading. 640 = 5 whole blocks
 * (6.4s), by which point the ring is 3 orders of magnitude down. The smoothing
 * FIR adds DSP_PPG_LP_TAPS-1 = 30 samples of its own settling, still far
 * inside this. */
#define DSP_PPG_WARMUP 640u

/* ---- Pulse rate from the PPG -------------------------------------------- */
/* 4s of PPG, matching DSP_ECG_WINDOW's 4s so PR and HR are averages over the
 * same span and can be compared beat-for-beat when both are alive. At 100Hz
 * that is 3-5 pulse intervals to take a median over. */
#define DSP_PR_WINDOW 400u
/* Two peaks (one interval) is not a rate, it is a coincidence: a single motion
 * bump plus one real systolic peak produces exactly that. Three intervals is
 * the minimum that can outvote one bad one through the median. */
#define DSP_PR_MIN_INTERVALS 3u
/* The same floor for the ECG window, which is also 4s and so also holds 3-5
 * intervals. It used to be 1 -- any single interval in the plausible range
 * became a published bpm -- and on a lead-off AD8232 that is exactly what
 * happens: the rails saturate, two cable transients seconds apart clear the
 * 8x peak-to-mean energy test, and their one interval lands in range. The
 * result was a confident bradycardia on the triage screen from an unplugged
 * electrode. Three intervals plus the agreement test cost the low end of the
 * range -- 3 intervals do not fit in 4s below ~45bpm, so DSP_HR_MIN_BPM's 30 is
 * unreachable from either path -- and that is the right trade, because the cost
 * is a BLANK. A blank is a visible failure that an operator retries; a
 * plausible wrong number is not. (Contrast DSP_RR_MAX_BRPM, where the failure
 * was halving a high rate into a reassuring normal one.) */
#define DSP_ECG_MIN_INTERVALS 3u
/* Intervals within this fraction of the median count as agreeing. Motion
 * artefact scatters them far wider; a resting pulse holds inside a few percent,
 * and even marked sinus arrhythmia stays inside 30%.
 *
 * This gate has a deliberate cost: a genuinely irregular rhythm (AF, frequent
 * ectopics) also fails it, so PR reports 0 rather than a number. That is the
 * right trade for a PPG, which cannot tell an ectopic beat from a knock on the
 * sensor - both are one out-of-place pulse. Dsp_PrResult.regular carries the
 * distinction outward so the caller can say "irregular" instead of "no
 * reading". Diagnosing the rhythm itself needs the ECG; see Dsp_EcgResult. */
#define DSP_PR_SPREAD_FRAC 0.30f
/* Fraction of the systolic peak used as the detection threshold. The dicrotic
 * notch is the thing being excluded: it follows the systolic peak by 200-400ms
 * (inside a plausible interval, so the refractory alone will not reject it) and
 * reaches 20-50% of its height. 0.6 clears it with margin. Compare the ECG
 * path's 0.35 of a SQUARED envelope, which is 0.59 in amplitude terms - the
 * same place, reached differently. */
#define DSP_PR_THRESH_FRAC 0.6f

/* ---- Plausibility limits ------------------------------------------------ */
#define DSP_HR_MIN_BPM 30.0f
#define DSP_HR_MAX_BPM 220.0f
#define DSP_RR_MIN_BRPM 6.0f
/* Ceiling is set by triage need, not by adult resting range. This device is for
 * earthquake triage where patients may be adults or children: infants normally
 * breathe 30-60, children 20-30, adults 12-20, and JumpSTART flags paediatric
 * patients outside 15-45 as immediate. A 30 brpm cap (used until 2026-08-14)
 * caused severe tachypnoea to be HALVED into a reassuring normal number -
 * under-triage, the worst failure this device can have. 70 covers infants plus
 * severe adult tachypnoea with margin.
 * Measured limit of the signal chain: the 2Hz envelope lowpass gives -6dB at a
 * 2Hz envelope (60 brpm with both phases audible) and -15.6dB at 3Hz, so rates
 * beyond ~70 brpm are attenuated too far to trust. */
#define DSP_RR_MAX_BRPM 70.0f
/* A breath makes sound twice per cycle (inhale and exhale), so the envelope's
 * strongest component usually sits at 2x the breathing rate. The FFT search
 * therefore has to run to 2 * DSP_RR_MAX_BRPM to see that peak at all; the
 * octave ambiguity is resolved afterwards. Searching only to 30 brpm is why
 * respiratory rate always read 0 before 2026-08-14. */
#define DSP_RR_SEARCH_MAX_BRPM (2.0f * DSP_RR_MAX_BRPM)
/* Energy at half the peak frequency must exceed this fraction of the peak for
 * the subharmonic to count as the true fundamental. Breathing is normally
 * asymmetric (one phase louder), which leaves a real bump there; symmetric
 * breathing cancels it and the peak is then the doubled rate. */
#define DSP_RR_SUBHARM_RATIO 0.30f
/* How many spectral peaks Dsp_RespRateBrpm reports for debugging, and how many
 * it will evaluate as rate candidates. Slow baseline drift in the mic envelope
 * (a patient shifting, ambient level wandering) is broad and often STRONGER
 * than the breath itself, so taking only the single strongest peak is not
 * enough - on real hardware a 687-magnitude drift peak at 5.33 brpm beat the
 * genuine 144-magnitude breath peak at 29.61 brpm, and the reading was lost. */
#define DSP_RESP_TOP_N 5u
/* Minimum RMS (ADC LSB) in the 5-15Hz band for a window to count as ECG
 * rather than a lead-off or flat trace. An AD8232 QRS is >100 LSB peak, so
 * 5 LSB is a generous floor that still rejects a bare noise floor. */
#define DSP_ECG_MIN_RMS 5.0f
/* Skin-contact floor, in raw counts. Below it there is no finger and NOTHING
 * downstream may treat the waveform as a measurement: it gates the SpO2 DC
 * check (DSP_SPO2_REJ_DC_LOW) and TB_FLAG_PPG_CONTACT, which is why it is one
 * constant and not one per consumer -- two thresholds for the same physical
 * question would eventually disagree, and the failure would be a plausible
 * number on the screen with no finger on the sensor.
 *
 * 5000 comes from measurement on this board, not from the datasheet. With
 * nothing on the sensor RED reads ~2000 counts (ambient plus detector noise);
 * a seated finger reads 150000-180000, varying with position. So the two
 * populations are 75x apart and any floor in between works -- 5000 sits 2.5x
 * above the noise and 30x below the signal. The previous value of 1000 was
 * BELOW the no-finger reading, so the DC gate passed on ambient light and the
 * ESP32 was shown an SpO2 percentage derived from noise.
 *
 * Checked against RED rather than IR because tissue attenuates red far more,
 * so RED is always the smaller of the two and therefore the binding channel. */
#define DSP_PPG_MIN_DC 5000.0f
/* Perfusion floor rejects a finger resting without pressure: a real pulse is
 * 0.5-2% AC/DC, so 0.05% is well below any true reading. */
#define DSP_SPO2_MIN_PI 0.0005f

/* ---- SpO2 calibration --------------------------------------------------- */
/* Number of points in the R -> SpO2 calibration table in dsp_utils.c. Edit the
 * table there, then set this to match. See the comment above the table for how
 * to collect the points. */
#define DSP_SPO2_CAL_POINTS 4u

typedef struct Dsp_EcgResult {
	/** 0 when no plausible rate was found. Requires DSP_ECG_MIN_INTERVALS
	 *  intervals that agree with their own median (the same test the PR path
	 *  uses), so a lead-off trace reads 0 rather than the interval between two
	 *  cable transients. Compare @p beats: beats > 0 with bpm 0 means peaks were
	 *  found but nothing consistent, which is the lead-off signature. */
	uint16_t bpm;
	uint16_t mean_raw; /**< Mean of the unfiltered window, for telemetry. */
	float rms_filtered; /**< RMS of the 5-15Hz band, a signal-quality hint. */
	uint8_t beats; /**< R-peaks detected in this window, before any gating. */
} Dsp_EcgResult;

/** Why a block was rejected. Mirrored to mon_spo2_reject for CubeMonitor. */
typedef enum Dsp_Spo2Reject {
	DSP_SPO2_OK = 0,
	DSP_SPO2_REJ_NO_BLOCK = 1, /**< called before a block finished */
	DSP_SPO2_REJ_DC_LOW = 2, /**< RED or IR DC under DSP_PPG_MIN_DC: no finger */
	DSP_SPO2_REJ_PERFUSION = 3, /**< AC/DC under DSP_SPO2_MIN_PI: no pulse */
	DSP_SPO2_REJ_R_RANGE = 4, /**< ratio-of-ratios outside 0..3.4 */
	DSP_SPO2_REJ_SPO2_LOW = 5, /**< result below the calibrated range */
	DSP_SPO2_REJ_WARMUP = 6 /**< filters still settling after Dsp_Init */
} Dsp_Spo2Reject;

typedef struct Dsp_Spo2Result {
	float r_value; /**< (ACred/DCred) / (ACir/DCir). */
	float spo2_pct; /**< From the calibration table; 0 when not valid. */
	uint8_t valid; /**< 0 when perfusion was too low to trust. */
	uint8_t reject; /**< Dsp_Spo2Reject: why valid is 0. */
	/* 1 when r_value fell outside the calibrated R range and spo2_pct is an
	 * extrapolation rather than an interpolation. Report it as "approximate"
	 * or refuse to display it; do not present it as a measurement. */
	uint8_t extrapolated;
	/* Raw components, always filled even on rejection, so the gate that
	 * fired can be diagnosed from the actual numbers. */
	float ir_dc;
	float red_dc;
	float ir_ac; /**< RMS of the 0.5-5Hz band. */
	float red_ac;
	float perfusion; /**< ir_ac / ir_dc. Typical finger: 0.005-0.02. */
} Dsp_Spo2Result;

/** Resets every filter state and buffer. Call once before sampling starts. */
void Dsp_Init(void);

/* ---- ECG ---------------------------------------------------------------- */
/**
 * The 5-15Hz filtered ECG window, mean removed, in ADC counts. This is the
 * waveform to plot: the raw mon_ecg_raw stream carries baseline wander and
 * mains hum that bury the QRS complex.
 *
 * READ THE UPDATE PATTERN BEFORE TRUSTING A PLOT. Unlike the PPG monitors,
 * which are per-sample, this whole array is rewritten in one burst every 4s
 * when Dsp_EcgProcessWindow() runs -- 2000 samples appear at once, then nothing
 * changes for 4s. CubeMonitor sampling it as a scalar therefore shows a
 * staircase, not a trace; plot it as an array, or index one element to watch
 * that phase of the beat.
 *
 * Only the first @p len entries of the last call are meaningful. The tail holds
 * whatever the previous, longer window left there. main.c always passes the
 * full ECG_BUFFER_SIZE, so in this firmware all 2000 are live.
 *
 * Also the function's working buffer, hence "not ISR-safe" below: it is
 * overwritten in place with the raw copy, the mean-removed copy, and then the
 * filter output, so reading it while Dsp_EcgProcessWindow() runs gives a mix of
 * the three. Harmless for a debug plot, wrong for anything that computes on it.
 */
extern float32_t mon_ecg_filtered[DSP_ECG_WINDOW];

/**
 * Bandpasses a raw ECG window to 5-15Hz and measures the heart rate from the
 * median R-R interval. Leaves the filtered window in mon_ecg_filtered.
 * Not ISR-safe (uses a shared working buffer) and not
 * reentrant; call from the main loop only. @p len must be <= DSP_ECG_WINDOW.
 */
void Dsp_EcgProcessWindow(const uint16_t *raw, uint32_t len,
		Dsp_EcgResult *out);

/* ---- Respiratory rate from breathing sound ------------------------------ */
/**
 * Diagnostics from one respiratory FFT. Populated by Dsp_RespRateBrpm on every
 * call, whether or not it found a plausible rate, so a wrong or absent reading
 * can be traced to the stage that caused it.
 */
typedef struct Dsp_RespDebug {
	/** Strongest in-band bin, in brpm, BEFORE any halving. Compare against
	 *  brpm: if peak_brpm is 2x brpm then the harmonic was correctly folded. */
	float peak_brpm;
	/** 1 when peak_brpm was halved to get the reported rate. */
	uint8_t halved;
	/** 1 when a subharmonic bump justified the halving (asymmetric breathing);
	 *  0 when it was halved purely because the peak exceeded DSP_RR_MAX_BRPM. */
	uint8_t subharmonic_found;
	/** Peak magnitude over the mean in-band magnitude. Under 3.0 is rejected
	 *  as noise; a strong breath gives 4 or more. */
	float peak_ratio;
	/** The DSP_RESP_TOP_N strongest in-band peaks, descending by magnitude.
	 *  brpm[i] is 0 when fewer than i+1 distinct peaks were found. */
	float top_brpm[DSP_RESP_TOP_N];
	float top_mag[DSP_RESP_TOP_N];
} Dsp_RespDebug;

/**
 * Feeds one raw mic sample through highpass -> rectify -> lowpass envelope,
 * decimating into the FFT window. ISR-safe; call at DSP_ADC_FS_HZ.
 */
void Dsp_MicPushSample(uint16_t raw);

/** Non-zero once a fresh FFT window is available. */
uint8_t Dsp_RespReady(void);

/**
 * Runs a 256-point real FFT over the mic envelope and returns the dominant
 * breathing frequency in breaths/min, or 0 if none is plausible. Clears the
 * ready flag. Not ISR-safe; call from the main loop.
 *
 * @p dbg may be NULL. When non-NULL it receives the peak, the octave decision,
 * and the top DSP_RESP_TOP_N spectral peaks — see Dsp_RespDebug.
 */
float Dsp_RespRateBrpm(Dsp_RespDebug *dbg);

/* ---- SpO2 --------------------------------------------------------------- */
/**
 * One PPG sample after the smoothing FIR. Filled by Dsp_Spo2PushSample so the
 * cleaned waveform is available for plotting and for whatever downstream tier
 * wants a usable waveform, which the raw MAX30102 stream is too hashy to be:
 * ~25 counts of sample-to-sample noise on a ~200-count pulse.
 */
typedef struct Dsp_PpgSample {
	float ir; /**< smoothed IR, raw counts, baseline included */
	float red; /**< smoothed RED, raw counts */
	/** Smoothed IR minus its 0.5Hz baseline: the pulse waveform, centred on
	 *  zero. Preferred over the 0.5-5Hz AC output when the SHAPE matters - the
	 *  AC biquad's phase response varies across the pulse's own harmonics,
	 *  whereas subtracting a slow baseline leaves the FIR's linear phase
	 *  intact above ~1Hz. */
	float pulse;
	/** The same pulse as a percentage of its own baseline (100 * pulse / DC),
	 *  i.e. the instantaneous perfusion index. This is the only PPG trace here
	 *  whose AMPLITUDE means anything on its own: dividing by DC cancels LED
	 *  current, skin tone, sensor distance and contact pressure, so two
	 *  recordings taken hours apart with the finger held differently are
	 *  directly comparable, which raw counts are not. A seated finger gives
	 *  0.5-2% peak to peak.
	 *
	 *  0 whenever the IR baseline is below DSP_PPG_MIN_DC. That gate is not
	 *  about divide-by-zero -- with no finger the baseline is ambient light, and
	 *  dividing a noise pulse by a small noise baseline manufactures a large,
	 *  entirely fictional percentage.
	 *
	 *  Like @p pulse, it shows the 0.5Hz DC filter settling for the first
	 *  seconds after Dsp_Init (see DSP_PPG_WARMUP): the baseline is still
	 *  climbing towards the signal, so the difference between them is far too
	 *  large. Real on a plot, meaningless as a measurement. */
	float norm_pct;
} Dsp_PpgSample;

/**
 * Smooths one IR/RED pair with a linear-phase FIR (DSP_PPG_LP_TAPS), then
 * splits it into DC (<0.5Hz) and AC (0.5-5Hz) components for SpO2.
 * ISR-safe; call once per MAX30102 sample at DSP_PPG_FS_HZ.
 *
 * @p out may be NULL; when given it receives the smoothed sample.
 */
void Dsp_Spo2PushSample(uint32_t ir, uint32_t red, Dsp_PpgSample *out);

/** Non-zero once DSP_PPG_BLOCK samples have accumulated. */
uint8_t Dsp_Spo2Ready(void);

/**
 * Computes the ratio-of-ratios from the buffered block. Clears the ready
 * flag. Not ISR-safe; call from the main loop.
 */
void Dsp_Spo2Compute(Dsp_Spo2Result *out);

/* ---- Pulse rate from the PPG -------------------------------------------- */
/**
 * A rate counted from the PPG instead of the ECG. NOT a heart rate, and the
 * difference is not pedantry: this counts mechanical pulses arriving at the
 * fingertip, whereas HR counts electrical depolarisations at the chest. A beat
 * too weak to open the aortic valve — an early ectopic, or any beat during a
 * pulse deficit — is invisible here and present in the ECG. So PR <= HR always,
 * and where they diverge the difference is itself the finding.
 *
 * Use it as a stand-in for HR when the ECG is unusable, which for triage
 * rate-counting it is: on a perfusing patient the two agree within a beat or
 * two. Do not use it to judge rhythm.
 */
typedef struct Dsp_PrResult {
	uint16_t bpm; /**< 0 when no plausible rate was found. */
	uint8_t pulses; /**< Peaks detected in the window, before any gating. */
	/** 1 when a MAJORITY of intervals agreed with the median to within
	 *  DSP_PR_SPREAD_FRAC, and at least DSP_PR_MIN_INTERVALS of them did. 0 with
	 *  a non-zero @p pulses count means peaks were found but no consistent
	 *  spacing dominated: either motion artefact or a genuinely irregular
	 *  rhythm, and a PPG cannot distinguish those. Surface it as "irregular",
	 *  never as a diagnosis, and never as a rate.
	 *
	 *  A majority rather than unanimity on purpose: an unmounted sensor gets
	 *  bumped, and one bad interval among five should not discard a window whose
	 *  median is already correct. Compare @p spread, which does report that
	 *  worst case. */
	uint8_t regular;
	/** Interval scatter actually measured, as a fraction of the median, taken
	 *  from the WORST interval. Its value when @p regular is 0 says which
	 *  failure it was: 0.3-0.5 is a plausible arrhythmia, several hundred
	 *  percent is the sensor being knocked. Note a large @p spread alongside
	 *  @p regular 1 is the normal, healthy outcome of one artefact in an
	 *  otherwise clean window. 0 when fewer than DSP_PR_MIN_INTERVALS intervals
	 *  were found. */
	float spread;
	/** How many intervals landed within DSP_PR_SPREAD_FRAC of the median. Read
	 *  it against @p pulses: agree == pulses-1 is a clean window, and anything
	 *  less is the count of artefacts the median absorbed. */
	uint8_t agree;
} Dsp_PrResult;

/**
 * Counts the pulse rate from the last DSP_PR_WINDOW samples of the normalised
 * PPG. Reads a ring that Dsp_Spo2PushSample fills, so it needs no buffer from
 * the caller and can be called at any cadence; call it no faster than once per
 * DSP_PR_WINDOW/4 samples or consecutive results will share most of their
 * peaks. Not ISR-safe; call from the main loop.
 *
 * Works on norm_pct, not raw counts, so the threshold is a fixed fraction of a
 * quantity already independent of LED current, distance and skin tone. Returns
 * bpm 0 (and leaves @p out zeroed) until the ring has filled.
 */
void Dsp_PrCompute(Dsp_PrResult *out);

#endif /* DSP_UTILS_H */
