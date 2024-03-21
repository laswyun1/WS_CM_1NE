/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#ifdef WALKON5_CM_ENABLED
#include "cmsis_os.h"
#include "adc.h"
#include "bdma.h"
#include "dma.h"
#include "fdcan.h"
#include "i2c.h"
#include "lwip.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

#include "WS_am_comm_hdlr.h"
#include "WS_data_ctrl.h"
#include "WS_ext_dev_ctrl.h"
#include "WS_gait_ctrl.h"
#include "WS_imu_ctrl.h"
#include "WS_system_ctrl.h"
#include "WS_whole_body_ctrl.h"

#include "data_object_dictionaries.h"
#endif /* WALKON5_CM_ENABLED */

#ifdef L30_CM_ENABLED
#include "cmsis_os.h"
#include "adc.h"
#include "bdma.h"
#include "dma.h"
#include "fdcan.h"
#include "i2c.h"
#include "lwip.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

#include "L30_am_comm_hdlr.h"
#include "L30_dev_comm_hdlr.h"
#include "L30_ext_dev_ctrl.h"
#include "L30_gait_ctrl.h"
#include "L30_imu_ctrl.h"
#include "L30_system_ctrl.h"
#include "L30_whole_body_ctrl.h"

#include "data_object_dictionaries.h"
#endif /* L30_CM_ENABLED */

#ifdef SUIT_MINICM_ENABLED
#include "cmsis_os.h"
#include "bdma.h"
#include "dma.h"
#include "fatfs.h"
#include "fdcan.h"
#include "i2c.h"
#include "sai.h"
#include "sdmmc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

#include "AS_ble_comm_hdlr.h"
#include "AS_debug_ctrl.h"
#include "AS_ext_dev_ctrl.h"
#include "AS_gait_ctrl.h"
#include "AS_imu_ctrl.h"
#include "AS_system_ctrl.h"
#include "AS_whole_body_ctrl.h"

#include "data_object_dictionaries.h"

#include "IOIF_Audio_WavPlay_SAI.h"
#endif /* SUIT_MINICM_ENABLED */

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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MPU_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#ifdef WALKON5_CM_ENABLED

#endif /* WALKON5_CM_ENABLED */

#ifdef L30_CM_ENABLED

#endif /* L30_CM_ENABLED */

#ifdef SUIT_MINICM_ENABLED
IOIF_WavPlayState_t init_state;
IOIF_WavPlayState_t play_ok;
wavfile_header_t test_wheader;

SystemState prevState_Audio = SYSTEM_UNKNOWN;
SystemState curState_Audio = SYSTEM_UNKNOWN;

IncDecState beepCur_Audio = UNKNOWN_ASSIST;
IncDecState beepPrev_Audio = UNKNOWN_ASSIST;

/* File system basic test */
//FATFS fs;
//FATFS *pfs = &fs;
//FIL file;
//FILE* wfil;

//IOIF_FATFS_t fs;
//IOIF_FATFS_ptr_t pfs = &fs;
//IOIF_FILE_t  file;
//
//
//uint8_t SD_readData[32*8];
//uint8_t SDcardDet = 0;
//uint32_t sd_total_size;
//
//char *test_str = "A10 SD Card Initialization success!!\r\n";
//char *SD_WriteTestData = "A10 SD Card Write Test Done!!\r\n";
//char *SD_WriteTestData2 = "A10 SD Card Write Operation Test Done!!\r\n";

/* File read/write basic test */

//FRESULT sd_open_res, sd_write_res, sd_read_res, seek_res;
//IOIF_SD_Status_t sd_init_res;
//IOIF_fCMD_res_t sd_mount_res, sd_open_res, sd_write_res, sd_read_res, seek_res;
//
//uint32_t w_byte = 1;
//uint32_t r_byte;
//
//float sd_freespace_test;
//uint32_t sd_totalspace_test;
//uint64_t freespace_size;

/* 11.02 FW */
uint8_t initSysTaskRes = INIT_OK;
uint8_t initWholeTaskRes = INIT_OK;
uint8_t startTaskRes = INIT_OK;
#endif /* SUIT_MINICM_ENABLED */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */
#ifdef WALKON5_CM_ENABLED

//  MX_GPIO_Init();
//  MX_DMA_Init();
//  MX_BDMA_Init();
//  MX_I2C1_Init();
//  MX_I2C2_Init();
//  MX_I2C3_Init();
//  MX_I2C4_Init();
//  MX_TIM14_Init();
//  MX_TIM16_Init();
//  MX_TIM17_Init();
//  MX_FDCAN1_Init();
//  MX_SPI3_Init();
//  MX_ADC3_Init();
//  MX_TIM13_Init();
//  MX_ADC1_Init();
//  MX_SPI4_Init();
//  MX_TIM3_Init();
//  MX_SPI5_Init();
//  MX_FDCAN2_Init();
//  MX_ADC2_Init();

#endif /* WALKON5_CM_ENABLED */

#if defined (L30_CM_ENABLED) || defined (WALKON5_CM_ENABLED)
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_BDMA_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_I2C3_Init();
  MX_I2C4_Init();
  MX_TIM16_Init();
  MX_FDCAN1_Init();
  MX_SPI3_Init();
  MX_ADC3_Init();
  MX_TIM13_Init();
  MX_ADC1_Init();
  MX_SPI4_Init();
  MX_TIM3_Init();
  MX_FDCAN2_Init();
  MX_TIM15_Init();
  MX_TIM2_Init();
  MX_TIM7_Init();
  MX_TIM6_Init();
  MX_SPI5_Init();
  MX_ADC2_Init();
#endif /* L30_CM_ENABLED */

#ifdef SUIT_MINICM_ENABLED
  MX_GPIO_Init();
  MX_BDMA_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_I2C4_Init();
  MX_TIM12_Init();
  MX_TIM15_Init();
  MX_SAI1_Init();
  MX_SPI1_Init();
  MX_SPI3_Init();
  MX_FDCAN1_Init();
  MX_I2C2_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_TIM16_Init();
  MX_SDMMC1_SD_Init();
  //   MX_FATFS_Init();
  MX_TIM2_Init();
  MX_SPI2_Init();
  MX_USART1_UART_Init();
#endif /* SUIT_MINICM_ENABLED */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */

  /* USER CODE BEGIN 2 */
#ifdef WALKON5_CM_ENABLED
//  BeepAlert(BEEP_ALERT_FREQ_VERY_HIGH, BEEP_ALERT_DUTY, BEEP_ALERT_TIME_VERY_SHORT, BEEP_ALERT_TIME_VERY_SHORT);
//  BeepAlert(BEEP_ALERT_FREQ_VERY_HIGH, BEEP_ALERT_DUTY, BEEP_ALERT_TIME_VERY_SHORT, BEEP_ALERT_TIME_VERY_SHORT);
#endif /* WALKON5_CM_ENABLED */

#if defined (L30_CM_ENABLED) || defined (WALKON5_CM_ENABLED)
//  BeepAlert(BEEP_ALERT_FREQ_VERY_HIGH, BEEP_ALERT_DUTY, BEEP_ALERT_TIME_VERY_SHORT, BEEP_ALERT_TIME_VERY_SHORT);
//  BeepAlert(BEEP_ALERT_FREQ_VERY_HIGH, BEEP_ALERT_DUTY, BEEP_ALERT_TIME_VERY_SHORT, BEEP_ALERT_TIME_VERY_SHORT);
#endif /* L30_CM_ENABLED */

#ifdef SUIT_MINICM_ENABLED
// Init Notification

  /* Basic SD Card Test using IOIF */

//  sd_init_res = IOIF_SD_Init(IOIF_SD1);
//
//  if(sd_init_res == IOIF_MSD_OK)
//  {
  ////////////////////////////////////////////////////////////////////////////////
  /* 1. Mount & Read */
//  sd_mount_res = IOIF_FileMount(&fs, (uint8_t*)"");
//  sd_open_res = IOIF_FileOpenCreate(&file, (uint8_t*)"test.txt");
//  sd_read_res = IOIF_fRead(&file, SD_readData, sizeof(SD_readData), &r_byte);


  /* 2. Mount & Write */
//  sd_mount_res = IOIF_FileMount(&fs, (uint8_t*)"");
//  sd_open_res = IOIF_FileOpenCreate(&file, (uint8_t*)"A10_ioif_test.txt");
//  sd_write_res = IOIF_fWrite(&file, SD_WriteTestData, strlen(SD_WriteTestData), &w_byte);


   /* 3. Calculate Card Space */
//   sd_mount_res = IOIF_FileMount(&fs, (uint8_t*)"");
//   sd_freespace_test = IOIF_Disk_GetFreeSpace((uint8_t*)"", &fs);
//   sd_totalspace_test = IOIF_Disk_TotalSpace(&fs);
//
//  IOIF_FileClose(&file);
  ///////////////////////////////////////////////////////////////////////////////

  /* 4. Read Operation */
//  sd_mount_res = IOIF_FileMount(&fs, (uint8_t*)"");
//  sd_read_res = IOIF_FileRead(&file, (uint8_t*)"A10_ioif_test.txt", SD_readData, sizeof(SD_readData), &r_byte);

  /* 5. Write Operation */
//  sd_mount_res = IOIF_FileMount(&fs, (uint8_t*)"");
//  sd_write_res = IOIF_FileWrite(&file, (uint8_t*)"A10_ioif_writeop.txt", SD_WriteTestData2, strlen(SD_WriteTestData2), &w_byte);
//
//  sd_mount_res = IOIF_FileUnmount((uint8_t*)"");
//  }

//  initSysTaskRes = InitSysMngtTask();
//  HAL_Delay(500);
  // Init_debug_task();
  // InitExtDevCtrl();
  // InitImuCtrl();
  // Init_comm_task();

//  initWholeTaskRes = InitWholeBodyCtrl();
//
//  if (initSysTaskRes || initWholeTaskRes != INIT_OK) {
//    systemStateFlag = SYSTEM_ERROR;
//  }
//
//  HAL_Delay(1000);
//  init_state = WavAudio_FS_Init((uint8_t*)"");	 //audio init.
//
//  startTaskRes = StartTask();
//  if(startTaskRes != INIT_OK) {
//    systemStateFlag = SYSTEM_ERROR;
//  }
#endif /* SUIT_MINICM_ENABLED */
  
  DOP_CreatePDOTable();
  DOP_CreateSDOTable();

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
#ifdef WALKON5_CM_ENABLED

#endif /* WALKON5_CM_ENABLED */

#ifdef L30_CM_ENABLED

#endif /* L30_CM_ENABLED */

#ifdef SUIT_MINICM_ENABLED
    /* 11.02 FW */
    
    /* Audio Play Loop Time in background */

//    if(init_state == IOIF_WAVPLAY_STATUS_OK)
//    {
//      curState_Audio = systemStateFlag;
//      if (prevState_Audio != curState_Audio) {
//          switch (curState_Audio) {
//              case SYSTEM_OFF:
//                  // SYSTEM_OFF
//                  play_ok = PlayWaveFile((uint8_t*)"", (uint8_t*)"/voice1102/99_system_off.wav"); //wave file play
//                  HAL_Delay(1000);
//                  play_ok = IOIF_WAVPLAY_STATUS_ERROR;
//                  break;
//
//              case SYSTEM_STANDBY:
//                  // SYSTEM_STANDBY
//                  play_ok = PlayWaveFile((uint8_t*)"", (uint8_t*)"/voice1102/01_system_ready.wav"); //wave file play
//                  HAL_Delay(1000);
//                  play_ok = IOIF_WAVPLAY_STATUS_ERROR;
//                  break;
//
//              case SYSTEM_ENABLE:
//                  // SYSTEM_ENABLE
//                  play_ok = PlayWaveFile((uint8_t*)"", (uint8_t*)"/voice1102/02_gait_start.wav"); //wave file play
//                  HAL_Delay(1000);
//                  play_ok = IOIF_WAVPLAY_STATUS_ERROR;
//                  break;
//
//              case SYSTEM_ERROR:
//                  // SYSTEM_ERROR
//                  play_ok = PlayWaveFile((uint8_t*)"", (uint8_t*)"/voice1102/98_system_error.wav"); //wave file play
//                  HAL_Delay(1000);
//                  play_ok = IOIF_WAVPLAY_STATUS_ERROR;
//                  break;
//
//              default:
//                  // SYSTEM_UNKNOWN
//                  break;
//          }
//          prevState_Audio = curState_Audio;
//      }
//
//      // beepCur_Audio = assistFlag;
//      // if (beepPrev_Audio != beepCur_Audio) {
//      //     switch (beepCur_Audio) {
//      //         case INCREASE_ASSIST:
//      //             // ASSIST INCREASE
//      //             play_ok = PlayWaveFile((uint8_t*)"", (uint8_t*)"/voice1102/05_assist_increase.wav"); //wave file play
//      //             HAL_Delay(1000);
//      //             play_ok = IOIF_WAVPLAY_STATUS_ERROR;
//      //             break;
//
//      //         case DECREASE_ASSIST:
//      //             // ASSIST DECREASE
//      //             play_ok = PlayWaveFile((uint8_t*)"", (uint8_t*)"/voice1102/06_assist_decrease.wav"); //wave file play
//      //             HAL_Delay(1000);
//      //             play_ok = IOIF_WAVPLAY_STATUS_ERROR;
//      //             break;
//
//      //         default:
//      //             // ASSIST UNKNOWN
//      //             break;
//      //     }
//      //     beepPrev_Audio = beepCur_Audio;
//      // }
//    }
//
#endif /* SUIT_MINICM_ENABLED */
  }

  /* USER CODE END 3 */
}

/* USER CODE BEGIN 4 */
#ifdef L30_CM_ENABLED
void BeepAlert(uint32_t freq, uint32_t duty, uint32_t on_time, uint32_t off_time)
{
	htim13.Instance->ARR = (BEEP_ALERT_TIM_FREQ / freq) - 1;

	// Beep On
	htim13.Instance->CCR1 = (htim13.Instance->ARR * duty / 100) - 1;
	HAL_TIM_PWM_Start(&htim13, TIM_CHANNEL_1);
	HAL_Delay(on_time);

	// Beep Off
	htim13.Instance->CCR1 = 0;
	HAL_TIM_PWM_Stop(&htim13, TIM_CHANNEL_1);
	HAL_Delay(off_time);
}
#endif /* WALKON5_CM_ENABLED & L30_CM_ENABLED */


#ifdef WALKON5_CM_ENABLED
void BeepAlert(uint32_t freq, uint32_t duty, uint32_t on_time, uint32_t off_time)
{
	htim13.Instance->ARR = (BEEP_ALERT_TIM_FREQ / freq) - 1;

	// Beep On
	htim13.Instance->CCR1 = (htim13.Instance->ARR * duty / 100) - 1;
	HAL_TIM_PWM_Start(&htim13, TIM_CHANNEL_1);
	HAL_Delay(on_time);
//	osDelay(on_time);

	// Beep Off
	htim13.Instance->CCR1 = 0;
	HAL_TIM_PWM_Stop(&htim13, TIM_CHANNEL_1);
	HAL_Delay(off_time);
//	osDelay(off_time);
}
#endif /* WALKON5_CM_ENABLED & L30_CM_ENABLED */


#ifdef WALKON5_CM_ENABLED
/**
  * @brief System Clock Configuration
  * @retval None
  */
//void SystemClock_Config(void)
//{
//  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
//  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
//
//  /** Supply configuration update enable
//  */
//  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
//
//  /** Configure the main internal regulator output voltage
//  */
//  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
//
//  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
//
//  __HAL_RCC_SYSCFG_CLK_ENABLE();
//  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);
//
//  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
//
//  /** Initializes the RCC Oscillators according to the specified parameters
//  * in the RCC_OscInitTypeDef structure.
//  */
//  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
//  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
//  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
//  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
//  RCC_OscInitStruct.PLL.PLLM = 5;
//  RCC_OscInitStruct.PLL.PLLN = 192;
//  RCC_OscInitStruct.PLL.PLLP = 2;
//  RCC_OscInitStruct.PLL.PLLQ = 12;
//  RCC_OscInitStruct.PLL.PLLR = 2;
//  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
//  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
//  RCC_OscInitStruct.PLL.PLLFRACN = 0;
//  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
//  {
//    Error_Handler();
//  }
//
//  /** Initializes the CPU, AHB and APB buses clocks
//  */
//  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
//                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
//                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
//  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
//  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
//  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
//  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
//  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
//  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
//  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;
//
//  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
//  {
//    Error_Handler();
//  }
//}
//
///**
//  * @brief Peripherals Common Clock Configuration
//  * @retval None
//  */
//void PeriphCommonClock_Config(void)
//{
//  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
//
//  /** Initializes the peripherals clock
//  */
//  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_ADC;
//  PeriphClkInitStruct.PLL2.PLL2M = 5;
//  PeriphClkInitStruct.PLL2.PLL2N = 80;
//  PeriphClkInitStruct.PLL2.PLL2P = 5;
//  PeriphClkInitStruct.PLL2.PLL2Q = 2;
//  PeriphClkInitStruct.PLL2.PLL2R = 2;
//  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_2;
////  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
//  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOMEDIUM;
//  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
//  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;
//
//  PeriphClkInitStruct.PLL3.PLL3M = 5;
//  PeriphClkInitStruct.PLL3.PLL3N = 192;
//  PeriphClkInitStruct.PLL3.PLL3P = 12;
//  PeriphClkInitStruct.PLL3.PLL3Q = 20;
//  PeriphClkInitStruct.PLL3.PLL3R = 2;
//  PeriphClkInitStruct.PLL3.PLL3RGE = RCC_PLL3VCIRANGE_2;
//  PeriphClkInitStruct.PLL3.PLL3VCOSEL = RCC_PLL3VCOWIDE;
//  PeriphClkInitStruct.PLL3.PLL3FRACN = 0;
//  //PeriphClkInitStruct.Sai1ClockSelection = RCC_SAI1CLKSOURCE_PLL3;
//  PeriphClkInitStruct.UsbClockSelection = RCC_USBCLKSOURCE_PLL3;
//
//  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
//  {
//    Error_Handler();
//  }
//}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 12;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_ADC;
  PeriphClkInitStruct.PLL2.PLL2M = 5;
  PeriphClkInitStruct.PLL2.PLL2N = 80;
  PeriphClkInitStruct.PLL2.PLL2P = 5;
  PeriphClkInitStruct.PLL2.PLL2Q = 2;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_2;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOMEDIUM;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;

  PeriphClkInitStruct.PLL3.PLL3M = 5;
  PeriphClkInitStruct.PLL3.PLL3N = 192;
  PeriphClkInitStruct.PLL3.PLL3P = 12;
  PeriphClkInitStruct.PLL3.PLL3Q = 20;
  PeriphClkInitStruct.PLL3.PLL3R = 2;
  PeriphClkInitStruct.PLL3.PLL3RGE = RCC_PLL3VCIRANGE_2;
  PeriphClkInitStruct.PLL3.PLL3VCOSEL = RCC_PLL3VCOWIDE;
  PeriphClkInitStruct.PLL3.PLL3FRACN = 0;
  //PeriphClkInitStruct.Sai1ClockSelection = RCC_SAI1CLKSOURCE_PLL3;
  PeriphClkInitStruct.UsbClockSelection = RCC_USBCLKSOURCE_PLL3;


  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x30000000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_32KB;
  MPU_InitStruct.SubRegionDisable = 0x0;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL1;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Number = MPU_REGION_NUMBER1;
  MPU_InitStruct.BaseAddress = 0x38000000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_64KB;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Number = MPU_REGION_NUMBER2;
  MPU_InitStruct.BaseAddress = 0x24069ae0;
//  MPU_InitStruct.BaseAddress = 0x24030000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_32KB;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}
#endif /* WALKON5_CM_ENABLED */

#ifdef L30_CM_ENABLED
/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 12;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_ADC;
  PeriphClkInitStruct.PLL2.PLL2M = 5;
  PeriphClkInitStruct.PLL2.PLL2N = 80;
  PeriphClkInitStruct.PLL2.PLL2P = 5;
  PeriphClkInitStruct.PLL2.PLL2Q = 2;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_2;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOMEDIUM;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;

  PeriphClkInitStruct.PLL3.PLL3M = 5;
  PeriphClkInitStruct.PLL3.PLL3N = 192;
  PeriphClkInitStruct.PLL3.PLL3P = 12;
  PeriphClkInitStruct.PLL3.PLL3Q = 20;
  PeriphClkInitStruct.PLL3.PLL3R = 2;
  PeriphClkInitStruct.PLL3.PLL3RGE = RCC_PLL3VCIRANGE_2;
  PeriphClkInitStruct.PLL3.PLL3VCOSEL = RCC_PLL3VCOWIDE;
  PeriphClkInitStruct.PLL3.PLL3FRACN = 0;
  //PeriphClkInitStruct.Sai1ClockSelection = RCC_SAI1CLKSOURCE_PLL3;
  PeriphClkInitStruct.UsbClockSelection = RCC_USBCLKSOURCE_PLL3;


  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}
/* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x30000000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_32KB;
  MPU_InitStruct.SubRegionDisable = 0x0;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL1;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Number = MPU_REGION_NUMBER1;
  MPU_InitStruct.BaseAddress = 0x38000000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_64KB;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Number = MPU_REGION_NUMBER2;
  MPU_InitStruct.BaseAddress = 0x24030000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_32KB;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}
#endif /* L30_CM_ENABLED */

#ifdef SUIT_MINICM_ENABLED
/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 12;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_SAI1;
  PeriphClkInitStruct.PLL3.PLL3M = 5;
  PeriphClkInitStruct.PLL3.PLL3N = 192;
  PeriphClkInitStruct.PLL3.PLL3P = 12;
  PeriphClkInitStruct.PLL3.PLL3Q = 20;
  PeriphClkInitStruct.PLL3.PLL3R = 2;
  PeriphClkInitStruct.PLL3.PLL3RGE = RCC_PLL3VCIRANGE_2;
  PeriphClkInitStruct.PLL3.PLL3VCOSEL = RCC_PLL3VCOWIDE;
  PeriphClkInitStruct.PLL3.PLL3FRACN = 0;
  PeriphClkInitStruct.Sai1ClockSelection = RCC_SAI1CLKSOURCE_PLL3;
  PeriphClkInitStruct.UsbClockSelection = RCC_USBCLKSOURCE_PLL3;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x30000000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_32KB;
  MPU_InitStruct.SubRegionDisable = 0x0;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL1;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Number = MPU_REGION_NUMBER1;
  MPU_InitStruct.BaseAddress = 0x38000000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_64KB;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Number = MPU_REGION_NUMBER2;
  MPU_InitStruct.BaseAddress = 0x24030000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_32KB;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

#endif /* SUIT_MINICM_ENABLED */
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */

#ifdef SUIT_MINICM_ENABLED
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
#endif

#ifdef L30_CM_ENABLED
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
#endif

#ifdef WALKON5_CM_ENABLED
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
#endif

  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  /* User can add his own implementation to report the HAL error return state */

  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
