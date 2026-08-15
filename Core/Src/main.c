/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include "stdint.h"
#include "LoRa.h"
#include "dsp_utils.h"
#include "max30102_for_stm32_hal.h"
#include "rfid_pn532.h"
#include "tb_buttons.h"
#include "tb_regs.h"
#include "tb_slave.h"
//#include "filter_1.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum Gender_t {
	GENDER_UNSET = 0, GENDER_MALE = 1, GENDER_FEMALE = 2
} Gender_t;

#pragma pack(push, 1)
typedef struct LoRa_SendData {
	uint8_t node_id;
	uint8_t heart_rate; /**< bpm, 0 = no reading */
	uint8_t spo2_pct; /**< %, 0 = no reading */
	uint16_t ecg_mean; /**< raw ADC mean, signal-quality hint */
	uint16_t respiratory_rate_x10; /**< breaths/min * 10, 0 = no reading */
	uint8_t age;
	uint8_t gender; /**< Gender_t, held as uint8_t so the wire size is fixed */
	uint32_t rfid_uid;
} LoRa_SendData;
#pragma pack(pop)
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// ADC DMA buffer indices (0-based positions in the scan sequence, NOT the
// HAL "Rank" numbers, which are 1-based). Rank 1 = ADC1_IN1 (PA1, AD8232 ECG),
// Rank 2 = ADC1_IN2 (PA2, breathing-sound microphone).
//
// ECG moved from PA0 to PA1 on 2026-08-14. PA0 is the onboard USER KEY on
// WeAct Black Pill boards, so that button circuit shares the net with the
// analogue input. The symptom was mon_ecg_raw pinned at ~4025 counts (3.24V of
// 3.3V) with only a few LSB of hash on it, identical across two different
// AD8232 modules and unaffected by electrode placement - i.e. not a signal at
// all. PA1 has no onboard function. These indices are unchanged by the move:
// the ranks kept their order, only Rank 1's channel changed.
#define ECG_ADC_INDEX 0u
#define MIC_ADC_INDEX 1u
#define ANALOG_CHANNEL_COUNT 2u
// ECG window: DSP_ECG_WINDOW samples at DSP_ADC_FS_HZ = 4s, enough for 4-6
// beats and so 3-5 R-R intervals to take a median over.
#define ECG_BUFFER_SIZE DSP_ECG_WINDOW
// Lora Config
#define NODE_ID 0x01u
#define LORA_FREQUENCY 433000000u
#define NODE_REPORT_INTERVAL 750u
#define LORA_TX_TIMEOUT 100u

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// LoRa
LoRa hlora;

char send_data[sizeof(LoRa_SendData)];
LoRa_SendData lora_send_data_struct;
uint16_t lora_status;
uint8_t volatile lora_send_ready = 0;

// SPO2
max30102_t hmax30102;

// Analog Sensor
static volatile uint16_t analog_data[ANALOG_CHANNEL_COUNT];
// ECG double buffer: the ADC ISR fills one half while the main loop processes
// the other, so a 4s window can be analysed without blocking acquisition.
static uint16_t ecg_buffer[2][ECG_BUFFER_SIZE];
static volatile uint8_t ecg_fill_bank = 0;
static volatile uint16_t ecg_buffer_index = 0;
static volatile uint8_t ecg_ready_bank = 0xFF; /* 0xFF = none pending */

/* ---- Monitored outputs -------------------------------------------------- */
/* Every value the firmware produces, in one place, for STM32CubeMonitor.
 * CubeMonitor reads these out of RAM over SWD using the DWARF info in the
 * .elf, so they need two properties: external linkage (no `static`), so the
 * names are unambiguous in the variable picker and survive any inlining, and
 * `volatile`, so the compiler cannot discard a store that nothing inside the
 * firmware ever reads back. The mon_ prefix means typing "mon_" in the
 * variable filter lists exactly these and nothing else.
 *
 * Reads are not atomic: a 32-bit float sampled mid-update can momentarily
 * show a torn value. Harmless for monitoring, not safe to log as truth. */

/* Raw inputs, updated at the acquisition rate. */
volatile uint16_t mon_ecg_raw = 0; /**< 500Hz raw ECG sample, ADC LSB */
volatile uint16_t mon_mic_raw = 0; /**< 500Hz raw mic sample, ADC LSB */
volatile uint32_t mon_ir_raw = 0; /**< latest MAX30102 IR count */
volatile uint32_t mon_red_raw = 0; /**< latest MAX30102 RED count */

/* ECG / heart rate, updated once per 4s window. */
volatile uint16_t mon_hr_bpm = 0; /**< 0 = no plausible rate found */
volatile uint8_t mon_ecg_beats = 0; /**< R-peaks in the last window */
volatile uint16_t mon_ecg_mean = 0; /**< raw ADC mean, electrode-contact hint */
volatile float mon_ecg_rms = 0.0f; /**< 5-15Hz RMS, signal-quality hint */

/* Respiratory rate, updated every 2.5s once the 25.6s window has filled. */
volatile float mon_resp_brpm = 0.0f; /**< breaths/min, 0 = no reading */
/* Respiratory diagnostics. A breath is audible twice per cycle (inhale and
 * exhale), so the envelope's strongest component is usually at 2x the true
 * rate. mon_resp_peak_brpm is that raw spectral peak before correction, and
 * mon_resp_halved says whether it was folded in half to get mon_resp_brpm.
 * If mon_resp_brpm looks like double or half the real rate, these two show
 * which way the decision went. mon_resp_peak_ratio under 3.0 means the peak
 * did not stand clear of the noise and the window was rejected. */
volatile float mon_resp_peak_brpm = 0.0f;
volatile uint8_t mon_resp_halved = 0;
volatile uint8_t mon_resp_subharmonic = 0;
volatile float mon_resp_peak_ratio = 0.0f;
/* The DSP_RESP_TOP_N strongest spectral peaks, descending by magnitude. Plot
 * these as a group in CubeMonitor to see the whole envelope spectrum: a real
 * breath gives one dominant peak plus its harmonic at 2x, while room noise
 * gives several peaks of similar magnitude at unrelated frequencies. */
volatile float mon_resp_top_brpm[DSP_RESP_TOP_N];
volatile float mon_resp_top_mag[DSP_RESP_TOP_N];

/* SpO2, updated once per 128-sample (1.28s) PPG block. */
volatile float mon_spo2_pct = 0.0f;
volatile float mon_spo2_r = 0.0f; /**< ratio-of-ratios behind the % above */
volatile uint8_t mon_spo2_valid = 0;
/* Diagnostics: why a block was rejected, and the numbers behind the decision.
 * mon_spo2_reject is a Dsp_Spo2Reject: 0=OK, 2=DC too low (no finger),
 * 3=perfusion too low (finger present but no pulse detected), 4=R out of
 * range, 5=result under 70%. mon_spo2_perfusion should be 0.005-0.02 on a
 * properly seated finger. These are filled on every block, valid or not. */
volatile uint8_t mon_spo2_reject = 0;
volatile float mon_spo2_ir_dc = 0.0f;
volatile float mon_spo2_red_dc = 0.0f;
volatile float mon_spo2_ir_ac = 0.0f;
volatile float mon_spo2_red_ac = 0.0f;
volatile float mon_spo2_perfusion = 0.0f;
/* Counts how many consecutive blocks have been rejected. Climbing steadily
 * means the sensor is being read but never producing a usable reading. */
volatile uint32_t mon_spo2_rejects = 0;

/* Liveness counters. If mon_adc_samples is not climbing the DMA has stalled;
 * if mon_ecg_windows is not climbing the main loop is stuck or starved. */
volatile uint32_t mon_adc_samples = 0;
volatile uint32_t mon_ecg_windows = 0;
volatile uint32_t mon_lora_frames = 0;

/* ---- ESP32 I2C link ----------------------------------------------------- */
/* Debounce state for the 4 front-panel buttons. The published byte is
 * mon_buttons (1 = pressed, bit0 = BUTTON_1); watch it in CubeMonitor to check
 * wiring without needing the ESP32 attached. */
static tb_buttons_t btn_state;
volatile uint8_t mon_buttons = 0;
/* Last command the ESP32 asked for, latched for visibility. Acting on these is
 * the next step; right now they are only counted so the link can be proven in
 * both directions before any behaviour depends on it. */
volatile uint8_t mon_last_cmd = 0;
volatile uint32_t mon_cmds = 0;
/* Triage result written back by the ESP32 (LoRa order: 0=BLACK 1=RED 2=YELLOW
 * 3=GREEN). Not yet placed in the LoRa packet -- the 14-byte struct has no
 * field for it. */
volatile uint8_t mon_priority = 0;
volatile uint8_t mon_confidence = 0;

/* ---- RFID (PN532 on I2C3) ------------------------------------------------ */
/* The tag as ASCII hex, which is what goes on the wire and onto the ESP32's
 * screen. Uppercase, no separators: "04A2B3C4". Not NUL-terminated on the wire,
 * so rfid_ascii_len is authoritative. */
static char rfid_ascii[TB_RFID_MAX];
static uint8_t rfid_ascii_len;
/* Set by TB_CMD_START_SCAN, cleared when a tag is found, when the ESP32 aborts,
 * or when RFID_SCAN_WINDOW_MS expires.
 *
 * Why a retry window and not a single poll: the ESP32 asks to scan once, but the
 * operator presents the card seconds later. One PN532 poll would look at an
 * empty field, report NO_CARD and give up -- exactly what "nothing happened when
 * I tapped the card" looks like.
 *
 * Why the window is bounded rather than waiting for TB_CMD_ABORT: the ESP32 does
 * not send ABORT today. Without a deadline, a scan nobody completes would retry
 * forever, and each attempt blocks the superloop ~120 ms, so the ECG and SpO2
 * paths would stay permanently starved after one unanswered scan. The window is
 * generous enough for a person to find their card and short enough that a
 * forgotten scan heals itself. */
#define RFID_SCAN_WINDOW_MS 30000u
static uint8_t rfid_scanning;
static uint32_t rfid_scan_started_ms;
/* CubeMonitor view of the last scan. mon_rfid_status is a Pn532_Status:
 * 0=IDLE 1=FOUND 2=NO_CARD 3=ERR_I2C 4=ERR_ABSENT 5=ERR_PROTO. A steady 4 means
 * the module is not answering at all -- wrong bus mode strap, wiring or power. */
volatile uint8_t mon_rfid_status = 0;
volatile uint8_t mon_rfid_uid_len = 0;
volatile uint8_t mon_rfid_uid[PN532_UID_MAX] = { 0 };
volatile uint32_t mon_rfid_scans = 0;
volatile uint32_t mon_rfid_found = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void PackTelemetry(void);
static void PublishToEsp32(void);
static void ServiceRfid(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void max30102_plot(uint32_t ir_sample, uint32_t red_sample);
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_SPI1_Init();
  MX_TIM10_Init();
  MX_I2C3_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
	// DSP filter states must be reset before any sample can arrive.
	Dsp_Init();

	// LoRa Init
	memset(send_data, 0, sizeof(send_data));
	memset(&lora_send_data_struct, 0, sizeof(lora_send_data_struct));
	lora_send_data_struct.node_id = NODE_ID;
	// age / gender / rfid_uid stay 0 until the RFID reader is wired up; the
	// memset above already zeroed them, this just names the intent.
	lora_send_data_struct.gender = (uint8_t) GENDER_UNSET;

	hlora = newLoRa();

	hlora.CS_port = SX1278_NSS_GPIO_Port;
	hlora.CS_pin = SX1278_NSS_Pin;
	hlora.reset_port = SPI1_RST_GPIO_Port;
	hlora.reset_pin = SPI1_RST_Pin;
	hlora.DIO0_port = SX1278_DIO0_GPIO_Port;
	hlora.DIO0_pin = SX1278_DIO0_Pin;
	hlora.hSPIx = &hspi1;

	HAL_Delay(100);

	lora_status = LoRa_init(&hlora);

	// SPO2 Init
	HAL_I2C_Init(&hi2c1);
	max30102_init(&hmax30102, &hi2c1);
	max30102_reset(&hmax30102);
	max30102_clear_fifo(&hmax30102);

	// FIFO configurations
	max30102_set_fifo_config(&hmax30102, max30102_smp_ave_8, 1, 7);
	// LED and SPO2 configurations
	max30102_set_led_pulse_width(&hmax30102, max30102_pw_16_bit);
	max30102_set_adc_resolution(&hmax30102, max30102_adc_2048);
	max30102_set_sampling_rate(&hmax30102, max30102_sr_800);
	max30102_set_led_current_1(&hmax30102, 6.2f);
	max30102_set_led_current_2(&hmax30102, 6.2f);
	// Enter SpO2 mode
	max30102_set_mode(&hmax30102, max30102_spo2);
	// Enable FIFO_A_FULL interrupt
	max30102_set_a_full(&hmax30102, 1);
	max30102_set_ppg_rdy(&hmax30102, 1);
	// Enable die temperature measurement
	max30102_set_die_temp_en(&hmax30102, 1);
	// Enable DIE_TEMP_RDY interrupt
	max30102_set_die_temp_rdy(&hmax30102, 1);

	HAL_TIM_Base_Start_IT(&htim10);

	// Init ECG and mic ADC DMA
	// Element-wise, not memset: analog_data is volatile and memset would want
	// the qualifier cast away.
	for (uint32_t i = 0; i < ANALOG_CHANNEL_COUNT; ++i) {
		analog_data[i] = 0;
	}
	memset(ecg_buffer, 0, sizeof(ecg_buffer));
	ecg_buffer_index = 0;
	ecg_fill_bank = 0;
	ecg_ready_bank = 0xFF;
	// Start ADC DMA and timing TIM2
	// HAL_ADC_Start_DMA takes uint32_t* for legacy reasons; the DMA itself is
	// configured HALFWORD/HALFWORD, so the uint16_t buffer is what the hardware
	// actually writes. Cast through void* to drop volatile without a warning.
	HAL_ADC_Start_DMA(&hadc1, (uint32_t *) (void *) analog_data,
			ANALOG_CHANNEL_COUNT);
	HAL_TIM_Base_Start(&htim2);

	// RFID: PN532 on I2C3 (PA8 SCL / PB4 SDA). Bind only -- the module is
	// probed lazily on the first scan, so a missing or mis-strapped reader costs
	// nothing at boot and cannot stall startup.
	Pn532_Bind(&hi2c3);

	// ESP32 link: I2C2 slave at TB_I2C_SLAVE_ADDR. Last, so a master read that
	// arrives immediately finds the sensors already streaming.
	tb_buttons_init(&btn_state);
	tb_slave_init();
	PublishToEsp32();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		/* ---- ECG: heart rate from the bank the ISR just finished ---- */
		uint8_t bank = ecg_ready_bank;
		if (bank != 0xFF) {
			Dsp_EcgResult ecg;
			Dsp_EcgProcessWindow(ecg_buffer[bank], ECG_BUFFER_SIZE, &ecg);
			ecg_ready_bank = 0xFF; /* release the bank back to the ISR */

			lora_send_data_struct.heart_rate = (uint8_t) ((ecg.bpm > 255U) ?
					255U : ecg.bpm);
			lora_send_data_struct.ecg_mean = ecg.mean_raw;

			mon_hr_bpm = ecg.bpm;
			mon_ecg_beats = ecg.beats;
			mon_ecg_mean = ecg.mean_raw;
			mon_ecg_rms = ecg.rms_filtered;
			++mon_ecg_windows;
		}

		/* ---- Respiratory rate from the breathing-sound envelope ---- */
		if (Dsp_RespReady()) {
			Dsp_RespDebug rdbg;
			float brpm = Dsp_RespRateBrpm(&rdbg);
			mon_resp_brpm = brpm;
			mon_resp_peak_brpm = rdbg.peak_brpm;
			mon_resp_halved = rdbg.halved;
			mon_resp_subharmonic = rdbg.subharmonic_found;
			mon_resp_peak_ratio = rdbg.peak_ratio;
			for (uint32_t i = 0; i < DSP_RESP_TOP_N; ++i) {
				mon_resp_top_brpm[i] = rdbg.top_brpm[i];
				mon_resp_top_mag[i] = rdbg.top_mag[i];
			}
			lora_send_data_struct.respiratory_rate_x10 =
					(uint16_t) ((brpm * 10.0f) + 0.5f);
		}

		/* ---- SpO2 ---- */
		if (Dsp_Spo2Ready()) {
			Dsp_Spo2Result s;
			Dsp_Spo2Compute(&s);

			/* Always mirror the diagnostics, valid or not. */
			mon_spo2_valid = s.valid;
			mon_spo2_reject = s.reject;
			mon_spo2_ir_dc = s.ir_dc;
			mon_spo2_red_dc = s.red_dc;
			mon_spo2_ir_ac = s.ir_ac;
			mon_spo2_red_ac = s.red_ac;
			mon_spo2_perfusion = s.perfusion;
			mon_spo2_r = s.r_value;

			if (s.valid) {
				mon_spo2_pct = s.spo2_pct;
				mon_spo2_rejects = 0;
				lora_send_data_struct.spo2_pct = (uint8_t) (s.spo2_pct + 0.5f);
			} else {
				++mon_spo2_rejects;
				/* Hold the last good percentage on screen rather than blanking
				 * it: a single noisy 1.28s block among good ones would
				 * otherwise look like the reading collapsing to zero. Only
				 * declare no-reading after ~4s of consecutive rejects, which
				 * is a genuine loss of signal. mon_spo2_valid still goes 0
				 * immediately, so the staleness is always visible. */
				if (mon_spo2_rejects >= 3U) {
					mon_spo2_pct = 0.0f;
					lora_send_data_struct.spo2_pct = 0;
				}
			}
		}

		if ((lora_status == LORA_OK) && lora_send_ready) {
			PackTelemetry();
			LoRa_transmit(&hlora, (uint8_t*) send_data, sizeof(send_data),
					LORA_TX_TIMEOUT);
			lora_send_ready = 0;
			++mon_lora_frames;
		}

		// Prefer the interrupt path, but fall back to polling if the edge never arrives
		if (max30102_has_interrupt(&hmax30102)) {
			max30102_interrupt_handler(&hmax30102);
		} else {
			max30102_read_fifo(&hmax30102);
		}

		/* ---- ESP32 link ----
		 *
		 * Order matters: take the command first, so a START_SCAN arriving this
		 * pass is serviced this pass, and publish last, so the tag it found is
		 * in the very next snapshot the ESP32 reads. The other way round costs
		 * two full loops of latency on every scan. */
		{
			uint8_t cmd = tb_slave_take_cmd();
			if (cmd != TB_CMD_NONE) {
				/* START_SCAN is acted on; the rest are still latched only.
				 * Gating the measure window on START_MEASURE and parking sensors
				 * on POWER_OFF is the next step -- a half-wired command that
				 * silently does nothing is worse than one that visibly does. */
				if (cmd == TB_CMD_START_SCAN) {
					/* Clear the published tag before scanning. Without this the
					 * previous patient's card is still in the snapshot when the
					 * ESP32 opens its scanning screen, and the scan "succeeds"
					 * instantly on a stale identifier -- the worst possible
					 * failure mode here, since it attaches one patient's
					 * telemetry to another's ID. */
					rfid_ascii_len = 0U;
					rfid_scanning = 1U;
					rfid_scan_started_ms = HAL_GetTick();
				} else if (cmd == TB_CMD_ABORT) {
					rfid_scanning = 0U;
				}
				mon_last_cmd = cmd;
				++mon_cmds;
			}
		}

		ServiceRfid();

		PublishToEsp32();

		{
			uint8_t prio;
			uint8_t conf;
			if (tb_slave_take_result(&prio, &conf)) {
				/* Nowhere to put these yet: LoRa_SendData is 14 packed bytes
				 * with no priority field, and widening it is a station-side
				 * change too. Held here so the round trip is observable in
				 * CubeMonitor. */
				mon_priority = prio;
				mon_confidence = conf;
			}
		}

		HAL_Delay(10);
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 12;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 5;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM10) {
		lora_send_ready = 1;
	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
	// if (hadc->Instance != ADC1) return; Commented, no other ADC
	(void) hadc;

	uint16_t ecg_sample = analog_data[ECG_ADC_INDEX];
	uint16_t mic_sample = analog_data[MIC_ADC_INDEX];
	mon_ecg_raw = ecg_sample;
	mon_mic_raw = mic_sample;
	++mon_adc_samples;

	/* ---- ECG: fill the current bank, hand it over when full ---- */
	uint8_t bank = ecg_fill_bank;
	ecg_buffer[bank][ecg_buffer_index++] = ecg_sample;

	if (ecg_buffer_index >= ECG_BUFFER_SIZE) {
		ecg_buffer_index = 0;
		/* Only hand over if the main loop has released the previous window;
		 * otherwise keep overwriting this bank rather than corrupting the one
		 * being processed. */
		if (ecg_ready_bank == 0xFF) {
			ecg_ready_bank = bank;
			ecg_fill_bank = (uint8_t) (bank ^ 1U);
		}
	}

	/* ---- Respiratory: mic sample into the envelope detector ---- */
	Dsp_MicPushSample(mic_sample);
}

/* HAL calls HAL_GPIO_EXTI_Callback (no line number in the name) from
 * HAL_GPIO_EXTI_IRQHandler. The old HAL_GPIO_EXTI9_5_Callback was never
 * called by anything, which is why the MAX30102 needed a polling fallback.
 *
 * Only the MAX30102 is handled here. SX1278_DIO0 (PB1) is configured as an
 * EXTI line in gpio.c but nothing services it: EXTI1_IRQn is not enabled in
 * the NVIC, and this node only transmits. LoRa_transmit() polls RegIrqFlags
 * for TxDone and clears them itself, so the DIO0 edge has no work to do. */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == MAX30102_INT_Pin) {
		max30102_on_interrupt(&hmax30102);
	}
}

/* Called by the MAX30102 driver for every FIFO sample it reads.
 *
 * The driver's parameter names are not trustworthy. Per the MAX30102 register
 * map, 0x0C is LED1_PA = RED and 0x0D is LED2_PA = IR, and in SpO2 mode the
 * FIFO word order is LED1 (RED) first, then LED2 (IR). This driver has those
 * labels the other way round: it #defines MAX30102_LED_IR_PA1 as 0x0c and
 * unpacks FIFO bytes [0..2] as "ir". So what arrives here as ir_sample is
 * physically the RED channel and vice versa.
 *
 * Left uncorrected this inverts the ratio-of-ratios: instead of R ~ 0.5 for a
 * healthy finger you get R ~ 1.9, which fails the 70% floor and reports
 * DSP_SPO2_REJ_SPO2_LOW (reject code 5). Swapped here, at the one and only
 * place the driver's labels cross into application code.
 *
 * MON_SPO2_SWAP_CHANNELS exists because this rests on the datasheet register
 * map rather than a measurement on your board. Set it to 0 to pass the driver's
 * labels through unchanged; check mon_spo2_ir_dc > mon_spo2_red_dc, which is
 * true for a real finger since tissue attenuates red far more than infrared. */
#define MON_SPO2_SWAP_CHANNELS 1

void max30102_plot(uint32_t ir_sample, uint32_t red_sample) {
#if MON_SPO2_SWAP_CHANNELS
	uint32_t ir = red_sample;
	uint32_t red = ir_sample;
#else
	uint32_t ir = ir_sample;
	uint32_t red = red_sample;
#endif
	mon_ir_raw = ir;
	mon_red_raw = red;
	Dsp_Spo2PushSample(ir, red);
}

/* Serialises the telemetry struct into the LoRa payload. Kept separate from
 * the field updates so a transmit always sends one self-consistent snapshot
 * rather than a mix of old and new readings. */
static void PackTelemetry(void) {
	memcpy(send_data, &lora_send_data_struct, sizeof(lora_send_data_struct));
}

/* Keeps a requested RFID scan running and converts a found UID to the ASCII hex
 * the wire carries. Returns immediately when no scan is pending, so the cost in
 * the common case is one flag test.
 *
 * Retries every pass while a scan is pending, up to RFID_SCAN_WINDOW_MS, because
 * the ESP32 asks once but the operator taps the card seconds later -- one poll of
 * an empty field would just report NO_CARD and give up. Stops on the first tag
 * found, so a card left on the reader is not read repeatedly.
 *
 * Each attempt blocks for up to ~120 ms inside Pn532_Service, which is safe here
 * for the reason its header documents: ADC and PPG samples arrive by interrupt,
 * so a stalled superloop delays processing rather than losing data, and the
 * margin is the 4 s ECG window. Note that the ESP32's snapshot poll also pauses
 * for that long -- its 2 s frozen-seq warning has plenty of headroom, but this
 * is why scanning is request-scoped and not a background free-run.
 *
 * Why ASCII rather than the raw bytes: TB_REG_RFID is a char array the ESP32
 * puts straight on the screen and into its session record, and a 4-byte binary
 * UID containing 0x00 would truncate there. Hex is twice as long and always
 * printable. 10-byte UIDs give 20 characters, well inside TB_RFID_MAX (31).
 *
 * The tag is then STICKY: it keeps being published until the next START_SCAN
 * clears it. The ESP32 consumes it as a one-shot (ui_mock_rfid_ready clears its
 * own copy), so the card does not have to stay in the field while the operator
 * works through the following screens. */
static void ServiceRfid(void) {
	Pn532_Tag tag;
	uint8_t status;

	if (!rfid_scanning) {
		return; /* idle; leave the previously published tag in place */
	}

	/* Subtraction, not (start + window) > now: HAL_GetTick wraps at 2^32 ms and
	 * unsigned subtraction stays correct across the wrap. */
	if ((HAL_GetTick() - rfid_scan_started_ms) >= RFID_SCAN_WINDOW_MS) {
		rfid_scanning = 0U;
		return;
	}

	Pn532_RequestScan();
	status = Pn532_Service(&tag);

	++mon_rfid_scans;
	mon_rfid_status = status;

	if (status != PN532_FOUND) {
		/* Keep retrying, and deliberately keep rfid_ascii: a failed poll must
		 * not blank an identifier the operator is already looking at.
		 * mon_rfid_status is where the failure is visible -- a steady 4
		 * (ERR_ABSENT) means the module is not answering at all. */
		return;
	}

	rfid_scanning = 0U;
	++mon_rfid_found;
	mon_rfid_uid_len = tag.uid_len;
	for (uint8_t i = 0; i < PN532_UID_MAX; ++i) {
		mon_rfid_uid[i] = (i < tag.uid_len) ? tag.uid[i] : 0U;
	}
	lora_send_data_struct.rfid_uid = tag.uid_hash;

	static const char k_hex[] = "0123456789ABCDEF";
	/* Two hex characters per UID byte always fits: the longest UID ISO14443-3
	 * allows is 10 bytes = 20 characters, and TB_REG_RFID holds 31. Asserted at
	 * compile time rather than clamped at run time, so if either constant ever
	 * moves the build fails instead of silently truncating an identifier. */
	_Static_assert((PN532_UID_MAX * 2U) <= TB_RFID_MAX,
			"UID hex does not fit TB_REG_RFID");
	for (uint8_t i = 0; i < tag.uid_len; ++i) {
		rfid_ascii[i * 2U] = k_hex[(tag.uid[i] >> 4) & 0x0FU];
		rfid_ascii[(i * 2U) + 1U] = k_hex[tag.uid[i] & 0x0FU];
	}
	rfid_ascii_len = (uint8_t) (tag.uid_len * 2U);
}

/* Polls the buttons and hands the ESP32 a fresh snapshot. Called every superloop
 * pass (~10ms), which sets both the button response time and the rate at which
 * the sequence counter advances.
 *
 * Reads the mon_* globals rather than taking arguments: they are already the
 * one place every produced value lands, so a second parameter list would be a
 * second thing to keep in sync. They are volatile and not read atomically, so a
 * float could in principle tear; only the integers below go on the wire, and a
 * 16-bit load on Cortex-M4 is single-instruction. */
static void PublishToEsp32(void) {
	uint8_t pins = 0;
	uint8_t flags = 0;
	uint8_t sensors = 0;

	/* Raw pin levels, LSB = BUTTON_1. Pressed reads LOW (input + pull-up);
	 * tb_buttons_poll owns that inversion, so nothing here knows about it. */
	if (HAL_GPIO_ReadPin(BUTTON_1_GPIO_Port, BUTTON_1_Pin) == GPIO_PIN_SET) {
		pins |= 0x01U;
	}
	if (HAL_GPIO_ReadPin(BUTTON_2_GPIO_Port, BUTTON_2_Pin) == GPIO_PIN_SET) {
		pins |= 0x02U;
	}
	if (HAL_GPIO_ReadPin(BUTTON_3_GPIO_Port, BUTTON_3_Pin) == GPIO_PIN_SET) {
		pins |= 0x04U;
	}
	if (HAL_GPIO_ReadPin(BUTTON_4_GPIO_Port, BUTTON_4_Pin) == GPIO_PIN_SET) {
		pins |= 0x08U;
	}
	mon_buttons = tb_buttons_poll(&btn_state, pins);

	/* Per-vital validity. A zero reading means "no reading" in every one of
	 * these, which is why each gets its own bit: a finger off the MAX30102
	 * must not invalidate a perfectly good ECG heart rate. */
	if (mon_hr_bpm > 0U) {
		flags |= TB_FLAG_HR_VALID;
		sensors |= TB_SENSOR_ECG;
	}
	if (mon_spo2_valid != 0U) {
		flags |= TB_FLAG_SPO2_VALID;
	}
	if (mon_spo2_pct > 0.0f) {
		sensors |= TB_SENSOR_MAX30102;
	}
	if (mon_resp_brpm > 0.0f) {
		flags |= TB_FLAG_RR_VALID;
		sensors |= TB_SENSOR_MIC;
	}
	/* TB_FLAG_BP_VALID stays clear and BP stays 0: nothing measures pressure.
	 * PA2 is the breathing microphone now, so the MPX5010 path is gone. The
	 * ESP32's SVM uses BP as 2 of its 5 features and must see the bit clear
	 * rather than believe a fabricated 0/0. */
	if (lora_status == LORA_OK) {
		sensors |= TB_SENSOR_LORA;
	}
	/* RFID health is "the module answered", not "a card is present": an empty
	 * field is the normal state and must not show as a dead sensor on the
	 * ESP32's status dots. PN532_ERR_* and the never-scanned IDLE both leave it
	 * clear, so the dot only lights once the reader has actually been reached. */
	if ((mon_rfid_status == PN532_FOUND) || (mon_rfid_status == PN532_NO_CARD)) {
		sensors |= TB_SENSOR_RFID;
	}

	tb_slave_publish(flags, mon_buttons, mon_hr_bpm,
			(uint16_t) (mon_spo2_pct + 0.5f),
			(uint16_t) ((mon_resp_brpm * 10.0f) + 0.5f),
			0U, /* bp_sys: no source yet */
			0U, /* bp_dia: no source yet */
			0xFFU, /* battery: not measured on this board */
			sensors, rfid_ascii, rfid_ascii_len);
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
