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
/* Samples to discard after Dsp_Init before any block is trusted. The 0.5Hz DC
 * lowpass has a ~0.3s time constant, so from a cold start (state 0, input
 * jumping to tens of thousands of counts) it needs several seconds to settle,
 * and until it does the AC bandpass is still ringing on that step and reports
 * a large bogus AC amplitude. Measured on the host: at 300 samples the residual
 * ring still gives a perfusion index of 0.0019, inside the plausible-pulse
 * range, so it would pass the gate as a false reading. 640 = 5 whole blocks
 * (6.4s), by which point the ring is 3 orders of magnitude down. */
#define DSP_PPG_WARMUP 640u

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
/* SpO2 gates. DC floor rejects "no finger" (the MAX30102 reads a few hundred
 * counts of ambient with nothing on it, and 6.2mA through tissue gives tens of
 * thousands). Perfusion floor rejects a finger resting without pressure: a
 * real pulse is 0.5-2% AC/DC, so 0.05% is well below any true reading. */
#define DSP_SPO2_MIN_DC 1000.0f
#define DSP_SPO2_MIN_PI 0.0005f

/* ---- SpO2 calibration --------------------------------------------------- */
/* Number of points in the R -> SpO2 calibration table in dsp_utils.c. Edit the
 * table there, then set this to match. See the comment above the table for how
 * to collect the points. */
#define DSP_SPO2_CAL_POINTS 4u

typedef struct Dsp_EcgResult {
	uint16_t bpm; /**< 0 when no plausible rate was found. */
	uint16_t mean_raw; /**< Mean of the unfiltered window, for telemetry. */
	float rms_filtered; /**< RMS of the 5-15Hz band, a signal-quality hint. */
	uint8_t beats; /**< R-peaks detected in this window. */
} Dsp_EcgResult;

/** Why a block was rejected. Mirrored to mon_spo2_reject for CubeMonitor. */
typedef enum Dsp_Spo2Reject {
	DSP_SPO2_OK = 0,
	DSP_SPO2_REJ_NO_BLOCK = 1, /**< called before a block finished */
	DSP_SPO2_REJ_DC_LOW = 2, /**< IR or RED DC under DSP_SPO2_MIN_DC */
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
 * Bandpasses a raw ECG window to 5-15Hz and measures the heart rate from the
 * median R-R interval. Not ISR-safe (uses a shared working buffer) and not
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
 * Splits one IR/RED pair into DC (<0.5Hz) and AC (0.5-5Hz) components.
 * ISR-safe; call once per MAX30102 sample at DSP_PPG_FS_HZ.
 */
void Dsp_Spo2PushSample(uint32_t ir, uint32_t red);

/** Non-zero once DSP_PPG_BLOCK samples have accumulated. */
uint8_t Dsp_Spo2Ready(void);

/**
 * Computes the ratio-of-ratios from the buffered block. Clears the ready
 * flag. Not ISR-safe; call from the main loop.
 */
void Dsp_Spo2Compute(Dsp_Spo2Result *out);

#endif /* DSP_UTILS_H */
