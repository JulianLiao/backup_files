/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "config.h"
#include "can.h"
#include "DCMotor.h"
#include "encoder.h"
#include "PID.h"
#include "timer.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

CAN_HandleTypeDef hcan;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

USART_HandleTypeDef husart3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART3_Init(void);
static void MX_CAN_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
CAN_BrakeStatusFrame status_frame;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define LOG_PERIOD 2000  // unit: 0.5ms， 2000 * 0.5ms = 1000ms
#define SYS_STABLE_TIME 1000  // unit: ms

// 如果duty连续10次都在[-0.03, 0.03]区间内，则认为duty达标
#define PID_duty_TOLERANCE 0.1
#define PID_duty_continue_CNT 50
#define PWM_DUTY_REFERENCE 16000
// 如果cur_pose与tgt_pose连续40都只相差[-10, 10]，则认为pose_comp达标
#define POSE_COMP_TOLERANCE 10
#define POSE_COMP_continue_CNT 40

uint8_t hasBackToZERO = 0;
uint8_t stop_ctrl;
uint8_t hasSetNewBrake;
uint16_t brake_value;
int32_t gRefCnt;

// PositionPID's output is taken as the input of SpeedPID.
PID_t gPositionPID;
PID_t gSpeedPID;
volatile uint8_t gPosition_PIDflag;
volatile uint8_t gSpeed_PIDflag;
float gPositionTarget;
uint16_t gPWMNum = 0;

uint32_t gStartCtrlTicks;

typedef enum {
	SR_UNKNOWN = 0x00000000,
	SR_CTRL_TIMEOUT = 0x00000001,
	SR_OVER_CURRENT = 0x00000010,
	SR_PWM_OUT_CCNT_REACHED = 0x00000100,
	SR_PWM_DUTY_CCNT_REACHED = 0x00001000,
	SR_POSE_COMP_CCNT_REACHED = 0x00010000,
	SR_ZERO_COMPENSATE_TIMEOUT = 0x00100000,
	SR_ZERO_COMPENSATE_REACHED = 0x01000000
} StopReason;

StopReason mReason;

// if #ADC_BUF_LEN is too small, the adc callback #HAL_ADC_ConvCpltCallback will be triggered continuously, which will block main while(1) loop.
#define ADC_BUF_LEN 6144  // (4096 + 8192) / 2 = 6144
#define ADC_Newest_Num 20
#define Motor_Reverse_Over_Vol 2.0  // unit: v
#define Motor_Forward_Over_Vol 0.24  // unit: v
#define Over_Vol_continue_CNT 10
volatile uint8_t is_adc_complt = 0;
uint32_t cur_adc_val = 0;
float cur_cvt_vol = 0.0;
uint16_t ADC1_ConvertedVal[ADC_BUF_LEN];

int __io_putchar(int ch) {
	HAL_USART_Transmit(&husart3, (uint8_t*) &ch, 1, HAL_MAX_DELAY);
	return ch;
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
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
	MX_TIM3_Init();
	MX_USART3_Init();
	MX_CAN_Init();
	MX_TIM1_Init();
	MX_TIM2_Init();
	MX_ADC1_Init();
	MX_TIM4_Init();
	/* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start_IT(&htim2);
	HAL_TIM_Base_Start_IT(&htim4);
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
	__HAL_TIM_CLEAR_IT(&htim3, TIM_IT_UPDATE|TIM_FLAG_CC1);
	HAL_TIM_IC_CaptureCallback(&htim3);
	HAL_TIM_Encoder_Start_IT(&htim3, TIM_CHANNEL_ALL);
	CAN_User_Init(&hcan);
	// HAL_ADC_Start_IT(&hadc1);
	Start_Motor();
	Set_Motor_Break();
	// uint16_t cur_enc;
	// char tmp_msg[60];

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	uint8_t init_last = 0;  // includes init lastAbs and last_500us_ticks
	int32_t curAbs = 0;
	int32_t lastAbs = 0;
	uint32_t cur_500us_ticks = 0;
	uint32_t last_500us_ticks = 0;
	float cur_speed = 0.0;
	int32_t cur_pos = 0;
	HAL_Delay(SYS_STABLE_TIME);  // wait 1sec until the system gets stable
	uint8_t photoelec = 100;

	// HAL_ADCEx_Calibration_Start(&hadc1);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) ADC1_ConvertedVal, ADC_BUF_LEN);

	mReason = SR_UNKNOWN;

	// For INIT_BackToZERO Begin
	uint8_t recordFirstBackToZERO = 0;
	uint32_t backToZeroStartTicks = 0;
	// For INIT_BackToZERO End

	// For ZERO-COMPENSATE Begin
	uint8_t recordFirstZeroComp = 0;
	uint32_t zeroCompStartTicks = 0;
	// For ZERO-COMPENSATE End

	// kp = 0.7, ki = 0.2, kd = 0
	// serr: [-16000 / 2, 16000 / 2]
	// out: [-16000 / 2, 16000 / 2]
	PID_init(&gPositionPID, 1.0, 0.0, 0.0, (PWM_DUTY_REFERENCE / 2.0),
			(PWM_DUTY_REFERENCE / 2.0));
	PID_init(&gSpeedPID, 3.0, 0.0, 0, 300.0, 1.0);

	while (1) {
		photoelec = Get_Photoeletric_Status();
		curAbs = Encoder_GetAbs();
		cur_500us_ticks = Get_Cur_500usTicks();
		if (init_last) {
			int32_t deltaAbs = curAbs - lastAbs;
			uint32_t delta500usTicks = cur_500us_ticks - last_500us_ticks;
			cur_speed = deltaAbs * OneEn_LEN_meter
					/ (delta500usTicks * 0.0005); // unit: m/s
		} else {
			printf("init, no cur_speed yet!\r\n");
		}

		static uint8_t reverse_over_vol_ccnt = 0;
		static uint8_t forward_over_vol_ccnt = 0;
		if (is_adc_complt) {
			is_adc_complt = 0;
			cur_adc_val = 0;
			for (int k = (ADC_BUF_LEN - 1); k >= (ADC_BUF_LEN - ADC_Newest_Num);
					k--) {
				cur_adc_val += (ADC1_ConvertedVal[k] & 0xFFF);
			}
			cur_adc_val /= ADC_Newest_Num;
			cur_cvt_vol = cur_adc_val * 3.3 / 4096.0;
			if (cur_cvt_vol >= Motor_Reverse_Over_Vol) {
				reverse_over_vol_ccnt++;
			} else {
				reverse_over_vol_ccnt = 0;
			}
			if (cur_cvt_vol <= Motor_Forward_Over_Vol) {
				forward_over_vol_ccnt++;
			} else {
				forward_over_vol_ccnt = 0;
			}
			if (0 == cur_500us_ticks % LOG_PERIOD) {
				printf("cur_adc_val: %ld, cur_cvt_vol: %f\r\n", cur_adc_val,
						cur_cvt_vol);
			}
		}

		if (1 == hasBackToZERO) {
			if (0 == photoelec) {
				gRefCnt = Encoder_GetAbs();
				hasBackToZERO = 1;
				Set_Motor_Break();
				if (0 == cur_500us_ticks % LOG_PERIOD) {
					printf(
							"main | photoelec(0) -> Motor_Break, gRefCnt: %ld\r\n",
							gRefCnt);
				}
			}
			if (1 == hasSetNewBrake) {
				float move_dist = TOTAL_LEN_meter * brake_value / 100.0;
				gPositionTarget = -(move_dist / Motor_OneRound_LEN_meter)
						* Motor_OneRound;
				cur_pos = curAbs - gRefCnt;

				static uint8_t out_ccnt = 0;
				float tgt_speed = 0.0;
				if (0 == stop_ctrl) {
					if (forward_over_vol_ccnt >= Over_Vol_continue_CNT
							|| reverse_over_vol_ccnt >= Over_Vol_continue_CNT) {
						stop_ctrl = 1;
						mReason |= SR_OVER_CURRENT;
					} else if (gStartCtrlTicks
							> 0&& Get_Cur_500usTicks() - gStartCtrlTicks>= MAX_CTRL_500us_Ticks) {
						stop_ctrl = 1;
						mReason |= SR_CTRL_TIMEOUT;
					} else {
						if (gPosition_PIDflag) {
							gPosition_PIDflag = 0;
							PID_calc(&gPositionPID, gPositionTarget, cur_pos,
							NULL, POSITION_PID_dt);
						}

						if (gSpeed_PIDflag) {
							gSpeed_PIDflag = 0;
							// note: tgt_speed is in mm/s.
							float reference_speed = gPositionPID.out / (PWM_DUTY_REFERENCE / 2);
							tgt_speed = reference_speed / 2.0;
							PID_calc(&gSpeedPID, tgt_speed, cur_speed, NULL,
							SPEED_PIDPeriod_dt);
						}

						gPWMNum++;
						if (gPWMNum >= 0x7fffL) {
							gPWMNum = 0;
						}
						if (gPWMNum
								<= PWM_DUTY_RATIO_CTRL_CNT * PWM_ONE_DUTY_CNT) {
							for (int i = 0; i < PWM_DUTY_RATIO_CTRL_CNT ; i++) {
								if (gPWMNum > (i * PWM_ONE_DUTY_CNT)
										&& gPWMNum
												<= ((i + 1) * PWM_ONE_DUTY_CNT)) {
									gSpeedPID.out *= (1
											- PWM_DUTY_RATIO_STEP * (i + 1));
									break;
								}
							}
						} else {
							gSpeedPID.out *= PWM_DUTY_RATIO_MIN;
						}

						uint8_t is_out_finish =
								-SpeedPID_out_Thresh
										<= gSpeedPID.out&& gSpeedPID.out <= SpeedPID_out_Thresh;
						if (1 == is_out_finish) {
							out_ccnt++;
						} else if (0 == is_out_finish) {
							out_ccnt = 0;
						}
						if (out_ccnt >= SpeedPID_out_CCNT) {
							stop_ctrl = 1;
							mReason |= SR_PWM_OUT_CCNT_REACHED;
						}
						printf(
								"tgt_pos: %f, cur_pos: %ld, gPositionPID.out: %f, tgt_speed: %f, cur_speed: %f, gSpeedPID.out: %f\r\n",
								gPositionTarget, cur_pos, gPositionPID.out,
								tgt_speed, cur_speed, gSpeedPID.out);
						Set_Motor_Speed_By_Duty(gSpeedPID.out);
					}
				} else {
					printf(
							"Target Reached!  tgt_pos: %f, cur_pos: %ld, tgt_speed: %f, cur_speed: %f, out_ccnt: %d, mReason: 0x%08x\r\n",
							gPositionTarget, cur_pos, tgt_speed, cur_speed,
							out_ccnt, mReason);
					PID_reset(&gPositionPID);
					PID_reset(&gSpeedPID);
					hasSetNewBrake = 0;
					gStartCtrlTicks = 0;
					forward_over_vol_ccnt = 0;
					reverse_over_vol_ccnt = 0;
					out_ccnt = 0;
					gPosition_PIDflag = 0;
					gSpeed_PIDflag = 0;
					mReason = SR_UNKNOWN;
					Set_Motor_Break();
				}
			} else {
				Set_Motor_Break();
				if (0 == cur_500us_ticks % LOG_PERIOD) {
					printf(
							"main | hasBackToZERO(1) -> wait new brake value\r\n");
				}
			}
		} else {
			uint8_t lo_init_finish = 0;
			if (forward_over_vol_ccnt >= Over_Vol_continue_CNT
					|| reverse_over_vol_ccnt >= Over_Vol_continue_CNT) {
				printf("Back_To_Zero Over Voltage, Stop!!!\r\n");
				lo_init_finish = 1;
			} else if (backToZeroStartTicks
					> 0&& cur_500us_ticks - backToZeroStartTicks>= MAX_INIT_BACKTOZERO_500us_Ticks) {
				lo_init_finish = 1;
				backToZeroStartTicks = 0;
				recordFirstBackToZERO = 0;
				printf(
						"Back_To_Zero timeout, backToZeroStartTicks: %ld, cur_500us_ticks: %ld\r\n",
						backToZeroStartTicks, cur_500us_ticks);
			} else if (0 == photoelec) {
				printf("Back_To_Zero init OK\r\n");
				lo_init_finish = 1;
				backToZeroStartTicks = 0;
				recordFirstBackToZERO = 0;
			} else {
				if (0 == cur_500us_ticks % (LOG_PERIOD / 2)) {
					printf("main | Motor_Forward at INIT\r\n");
				}
				if (!recordFirstBackToZERO) {
					recordFirstBackToZERO = 1;
					backToZeroStartTicks = cur_500us_ticks;
				}
				Set_Motor_Forward();
			}
			if (lo_init_finish) {
				gRefCnt = Encoder_GetAbs();
				hasBackToZERO = 1;
				hasSetNewBrake = 0;
				Set_Motor_Break();
				printf("main | Anyway, init finished, gRefCnt: %ld\r\n",
						gRefCnt);
			}
		}

		init_last = 1;
		lastAbs = curAbs;
		last_500us_ticks = cur_500us_ticks;
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		HAL_Delay(1);
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
	RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
	PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV8;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void) {

	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */
	/** Common config
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
	hadc1.Init.ContinuousConvMode = ENABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 1;
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		Error_Handler();
	}
	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_0;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

}

/**
 * @brief CAN Initialization Function
 * @param None
 * @retval None
 */
static void MX_CAN_Init(void) {

	/* USER CODE BEGIN CAN_Init 0 */

	/* USER CODE END CAN_Init 0 */

	/* USER CODE BEGIN CAN_Init 1 */

	/* USER CODE END CAN_Init 1 */
	hcan.Instance = CAN1;
	hcan.Init.Prescaler = 4;
	hcan.Init.Mode = CAN_MODE_NORMAL;
	hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
	hcan.Init.TimeSeg1 = CAN_BS1_13TQ;
	hcan.Init.TimeSeg2 = CAN_BS2_4TQ;
	hcan.Init.TimeTriggeredMode = DISABLE;
	hcan.Init.AutoBusOff = ENABLE;
	hcan.Init.AutoWakeUp = ENABLE;
	hcan.Init.AutoRetransmission = DISABLE;
	hcan.Init.ReceiveFifoLocked = DISABLE;
	hcan.Init.TransmitFifoPriority = DISABLE;
	if (HAL_CAN_Init(&hcan) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN CAN_Init 2 */

	/* USER CODE END CAN_Init 2 */

}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void) {

	/* USER CODE BEGIN TIM1_Init 0 */

	/* USER CODE END TIM1_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = { 0 };

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 1;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 899;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 810;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3)
			!= HAL_OK) {
		Error_Handler();
	}
	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 6;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM1_Init 2 */

	/* USER CODE END TIM1_Init 2 */
	HAL_TIM_MspPostInit(&htim1);

}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void) {

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 14399;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 999;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */

}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void) {

	/* USER CODE BEGIN TIM3_Init 0 */

	/* USER CODE END TIM3_Init 0 */

	TIM_Encoder_InitTypeDef sConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 1;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 65535;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
	sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC1Filter = 0;
	sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC2Filter = 0;
	if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */
	/* USER CODE END TIM3_Init 2 */

}

/**
 * @brief TIM4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM4_Init(void) {

	/* USER CODE BEGIN TIM4_Init 0 */

	/* USER CODE END TIM4_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM4_Init 1 */

	/* USER CODE END TIM4_Init 1 */
	htim4.Instance = TIM4;
	htim4.Init.Prescaler = 71;
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim4.Init.Period = 499;
	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim4) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim4) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM4_Init 2 */

	/* USER CODE END TIM4_Init 2 */
	HAL_TIM_MspPostInit(&htim4);

}

/**
 * @brief USART3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART3_Init(void) {

	/* USER CODE BEGIN USART3_Init 0 */

	/* USER CODE END USART3_Init 0 */

	/* USER CODE BEGIN USART3_Init 1 */

	/* USER CODE END USART3_Init 1 */
	husart3.Instance = USART3;
	husart3.Init.BaudRate = 115200;
	husart3.Init.WordLength = USART_WORDLENGTH_8B;
	husart3.Init.StopBits = USART_STOPBITS_1;
	husart3.Init.Parity = USART_PARITY_NONE;
	husart3.Init.Mode = USART_MODE_TX_RX;
	husart3.Init.CLKPolarity = USART_POLARITY_LOW;
	husart3.Init.CLKPhase = USART_PHASE_1EDGE;
	husart3.Init.CLKLastBit = USART_LASTBIT_DISABLE;
	if (HAL_USART_Init(&husart3) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART3_Init 2 */

	/* USER CODE END USART3_Init 2 */

}

/** 
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {

	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Channel1_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 9, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);

	/*Configure GPIO pin : PA15 */
	GPIO_InitStruct.Pin = GPIO_PIN_15;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : PB5 */
	GPIO_InitStruct.Pin = GPIO_PIN_5;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
uint8_t Get_Photoeletric_Status(void) {
	uint8_t ret;
	if (GPIO_PIN_SET == HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15)) {
		ret = 1;
	} else {
		ret = 0;
	}
	return ret;
}

void Send_Brake_Status(void) {
	status_frame.ecuStatus = (uint8_t) Brake_EBS;
	status_frame.brakeValueH = 0x01;
	status_frame.brakeValueL = 0x10;
	status_frame.checksum = 0x5A;

	// prototype: HAL_StatusTypeDef CAN_SendData(uint16_t ID, uint64_t IDtype, CAN_BrakeStatusFrame Frame, uint8_t Len)
	if (CAN_SendData(0x111, CAN_ID_STD, status_frame, 8) == HAL_OK) {
		status_frame.count++;
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (1 == hasBackToZERO) {
		Send_Brake_Status();
	}
}

/**
 * This callback is called when the buffer #ADC1_ConvertedVal[ADC_BUF_LEN] is filled.
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
	is_adc_complt = 1;
}

/*void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
 // char tmp_msg[40];
 switch (GPIO_Pin) {
 case GPIO_PIN_15:
 if (GPIO_PIN_SET == HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15)) {
 uint16_t cur_enc;
 cur_enc = (uint16_t) (__HAL_TIM_GET_COUNTER(&htim3));
 memset(tmp_msg, '\0', 60);
 sprintf(tmp_msg, "Callback | Exit PA15(High)!  %d\r\n", cur_enc);
 HAL_UART_Transmit(&huart3, (uint8_t*) tmp_msg, strlen(tmp_msg),
 HAL_MAX_DELAY);
 printf("EXTI_Callback | SET\r\n");
 }
 else if (GPIO_PIN_RESET == HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15)) {
 Set_Motor_Break();
 gReferenceCnt = (uint16_t) (__HAL_TIM_GET_COUNTER(&htim3));
 memset(tmp_msg, '\0', 60);
 sprintf(tmp_msg, "Callback | Enter PA15(Low)!  %d\r\n", gReferenceCnt);
 HAL_UART_Transmit(&huart3, (uint8_t*) tmp_msg, strlen(tmp_msg),
 HAL_MAX_DELAY);
 hasBackToZERO = 1;
 printf("EXTI_Callback | RESET\r\n");
 }
 __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
 HAL_NVIC_ClearPendingIRQ(EXTI15_10_IRQn);
 __HAL_GPIO_EXTI_CLEAR_FLAG(GPIO_PIN_15);
 __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_15);
 break;
 default:
 break;
 }
 }*/
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */

	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
