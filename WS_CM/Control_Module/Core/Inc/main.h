/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "module.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
#ifdef WALKON5_CM_ENABLED
#define BEEP_ALERT_TIM_FREQ        1000000U
#define BEEP_ALERT_FREQ_VERY_HIGH     1200U
#define BEEP_ALERT_FREQ_HIGH          1000U
#define BEEP_ALERT_FREQ_LOW            800U
#define BEEP_ALERT_DUTY                 50U
#define BEEP_ALERT_TIME_LONG           500U
#define BEEP_ALERT_TIME_SHORT          100U
#define BEEP_ALERT_TIME_VERY_SHORT      50U
void BeepAlert(uint32_t freq, uint32_t duty, uint32_t on_time, uint32_t off_time);
#endif /* WALKON5_CM_ENABLED */

#ifdef L30_CM_ENABLED
#define BEEP_ALERT_TIM_FREQ        1000000U
#define BEEP_ALERT_FREQ_VERY_HIGH     1200U
#define BEEP_ALERT_FREQ_HIGH          1000U
#define BEEP_ALERT_FREQ_LOW            800U
#define BEEP_ALERT_DUTY                 50U
#define BEEP_ALERT_TIME_LONG           500U
#define BEEP_ALERT_TIME_SHORT          100U
#define BEEP_ALERT_TIME_VERY_SHORT      50U
void BeepAlert(uint32_t freq, uint32_t duty, uint32_t on_time, uint32_t off_time);
#endif /* L30_CM_ENABLED */
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/


/* USER CODE BEGIN Private defines */
#ifdef WALKON5_CM_ENABLED
//#define HIP_L_L_SW_P_Pin GPIO_PIN_2
//#define HIP_L_L_SW_P_GPIO_Port GPIOE
//#define HIP_L_L_SW_N_Pin GPIO_PIN_3
//#define HIP_L_L_SW_N_GPIO_Port GPIOE
//#define HIP_R_D_SW_P_Pin GPIO_PIN_4
//#define HIP_R_D_SW_P_GPIO_Port GPIOE
//#define TP68_Pin GPIO_PIN_5
//#define TP68_GPIO_Port GPIOE
//#define HIP_R_D_SW_N_Pin GPIO_PIN_6
//#define HIP_R_D_SW_N_GPIO_Port GPIOE
//#define MCU_EMR_Pin GPIO_PIN_13
//#define MCU_EMR_GPIO_Port GPIOC
//#define IMU_6AXIS_SDA_Pin GPIO_PIN_0
//#define IMU_6AXIS_SDA_GPIO_Port GPIOF
//#define IMU_6AXIS_SCL_Pin GPIO_PIN_1
//#define IMU_6AXIS_SCL_GPIO_Port GPIOF
//#define FET_SW_ON_Pin GPIO_PIN_2
//#define FET_SW_ON_GPIO_Port GPIOF
//#define SW_IC_INT_Pin GPIO_PIN_3
//#define SW_IC_INT_GPIO_Port GPIOF
//#define SW_IC_INT_EXTI_IRQn EXTI3_IRQn
//#define ADD_R_AbsENC_CLK_Pin GPIO_PIN_7
//#define ADD_R_AbsENC_CLK_GPIO_Port GPIOF
//#define ADD_R_AbsENC_DATA_Pin GPIO_PIN_8
//#define ADD_R_AbsENC_DATA_GPIO_Port GPIOF
//#define HIP_L_L_ADC_Pin GPIO_PIN_10
//#define HIP_L_L_ADC_GPIO_Port GPIOF
//#define CURRENT_SENSOR_ADC_Pin GPIO_PIN_0
//#define CURRENT_SENSOR_ADC_GPIO_Port GPIOC
//#define MCU_PHY_MDC_Pin GPIO_PIN_1
//#define MCU_PHY_MDC_GPIO_Port GPIOC
//#define MCU_RMII_REF_CLK_Pin GPIO_PIN_1
//#define MCU_RMII_REF_CLK_GPIO_Port GPIOA
//#define MCU_PHY_MDIO_Pin GPIO_PIN_2
//#define MCU_PHY_MDIO_GPIO_Port GPIOA
//#define CAN0_SILENT_Pin GPIO_PIN_4
//#define CAN0_SILENT_GPIO_Port GPIOA
//#define CAN1_SILENT_Pin GPIO_PIN_5
//#define CAN1_SILENT_GPIO_Port GPIOA
//#define MCU_BUZZER_Pin GPIO_PIN_6
//#define MCU_BUZZER_GPIO_Port GPIOA
//#define MCU_RMII_CRS_DV_Pin GPIO_PIN_7
//#define MCU_RMII_CRS_DV_GPIO_Port GPIOA
//#define MCU_RMII_RXD0_Pin GPIO_PIN_4
//#define MCU_RMII_RXD0_GPIO_Port GPIOC
//#define MCU_RMII_RXD1_Pin GPIO_PIN_5
//#define MCU_RMII_RXD1_GPIO_Port GPIOC
//#define HIP_MOTOR_L_L_IN1_Pin GPIO_PIN_0
//#define HIP_MOTOR_L_L_IN1_GPIO_Port GPIOB
//#define HIP_MOTOR_L_L_IN2_Pin GPIO_PIN_1
//#define HIP_MOTOR_L_L_IN2_GPIO_Port GPIOB
//#define HIP_MOTOR_L_D_IN1_Pin GPIO_PIN_2
//#define HIP_MOTOR_L_D_IN1_GPIO_Port GPIOB
//#define HIP_L_D_ADC_Pin GPIO_PIN_11
//#define HIP_L_D_ADC_GPIO_Port GPIOF
//#define HIP_R_L_ADC_Pin GPIO_PIN_12
//#define HIP_R_L_ADC_GPIO_Port GPIOF
//#define HIP_R_D_ADC_Pin GPIO_PIN_13
//#define HIP_R_D_ADC_GPIO_Port GPIOF
//#define MCU_BAT_MON_I2C4_SCL_Pin GPIO_PIN_14
//#define MCU_BAT_MON_I2C4_SCL_GPIO_Port GPIOF
//#define MCU_BAT_MON_I2C4_SDA_Pin GPIO_PIN_15
//#define MCU_BAT_MON_I2C4_SDA_GPIO_Port GPIOF
//#define HIP_R_L_SW_P_Pin GPIO_PIN_7
//#define HIP_R_L_SW_P_GPIO_Port GPIOE
//#define HIP_R_L_SW_N_Pin GPIO_PIN_8
//#define HIP_R_L_SW_N_GPIO_Port GPIOE
//#define TP97_Pin GPIO_PIN_9
//#define TP97_GPIO_Port GPIOE
//#define TP98_Pin GPIO_PIN_10
//#define TP98_GPIO_Port GPIOE
//#define TP99_Pin GPIO_PIN_11
//#define TP99_GPIO_Port GPIOE
//#define ADD_L_AbsENC_CLK_Pin GPIO_PIN_12
//#define ADD_L_AbsENC_CLK_GPIO_Port GPIOE
//#define ADD_L_AbsENC_DATA_Pin GPIO_PIN_13
//#define ADD_L_AbsENC_DATA_GPIO_Port GPIOE
//#define HIP_MOTOR_R_D_IN2_Pin GPIO_PIN_10
//#define HIP_MOTOR_R_D_IN2_GPIO_Port GPIOB
//#define MCU_RMII_TX_EN_Pin GPIO_PIN_11
//#define MCU_RMII_TX_EN_GPIO_Port GPIOB
//#define MCU_FDCAN2_RX_Pin GPIO_PIN_12
//#define MCU_FDCAN2_RX_GPIO_Port GPIOB
//#define MCU_FDCAN2_TX_Pin GPIO_PIN_13
//#define MCU_FDCAN2_TX_GPIO_Port GPIOB
//#define XAVIER_MCU_SLEEP_Pin GPIO_PIN_8
//#define XAVIER_MCU_SLEEP_GPIO_Port GPIOD
//#define XAVIER_MCU_RST_Pin GPIO_PIN_9
//#define XAVIER_MCU_RST_GPIO_Port GPIOD
//#define XAVIER_MCU_OFF_REQ_Pin GPIO_PIN_10
//#define XAVIER_MCU_OFF_REQ_GPIO_Port GPIOD
//#define XAVIER_MCU_OFF_REQ_EXTI_IRQn EXTI15_10_IRQn
//#define MCU_XAVIER_PWR_EN_Pin GPIO_PIN_11
//#define MCU_XAVIER_PWR_EN_GPIO_Port GPIOD
//#define MCU_5V_PWR_EN_Pin GPIO_PIN_12
//#define MCU_5V_PWR_EN_GPIO_Port GPIOD
//#define MCU_48V_MOTOR_ON_Pin GPIO_PIN_13
//#define MCU_48V_MOTOR_ON_GPIO_Port GPIOD
//#define MCU_3P3V_PWR_EN_Pin GPIO_PIN_14
//#define MCU_3P3V_PWR_EN_GPIO_Port GPIOD
//#define TP92_Pin GPIO_PIN_15
//#define TP92_GPIO_Port GPIOD
//#define MCU_INT_BUTTON1_Pin GPIO_PIN_2
//#define MCU_INT_BUTTON1_GPIO_Port GPIOG
//#define MCU_EMRG3_Pin GPIO_PIN_3
//#define MCU_EMRG3_GPIO_Port GPIOG
//#define MCU_12V_ON_Pin GPIO_PIN_5
//#define MCU_12V_ON_GPIO_Port GPIOG
//#define MCU_SW_CLR_Pin GPIO_PIN_7
//#define MCU_SW_CLR_GPIO_Port GPIOG
//#define MCU_INT_BUTTON2_Pin GPIO_PIN_8
//#define MCU_INT_BUTTON2_GPIO_Port GPIOG
//#define STAT_LED_NZR_L_Pin GPIO_PIN_6
//#define STAT_LED_NZR_L_GPIO_Port GPIOC
//#define STAT_LED_NZR_R_Pin GPIO_PIN_7
//#define STAT_LED_NZR_R_GPIO_Port GPIOC
//#define IMU_3AXIS_RDY_Pin GPIO_PIN_8
//#define IMU_3AXIS_RDY_GPIO_Port GPIOC
//#define IMU_3AXIS_SDA_Pin GPIO_PIN_9
//#define IMU_3AXIS_SDA_GPIO_Port GPIOC
//#define IMU_3AXIS_SCL_Pin GPIO_PIN_8
//#define IMU_3AXIS_SCL_GPIO_Port GPIOA
//#define MCU_USB_OTG_VBUS_Pin GPIO_PIN_9
//#define MCU_USB_OTG_VBUS_GPIO_Port GPIOA
//#define MCU_USB_ID_Pin GPIO_PIN_10
//#define MCU_USB_ID_GPIO_Port GPIOA
//#define MCU_USB_FS_DN_Pin GPIO_PIN_11
//#define MCU_USB_FS_DN_GPIO_Port GPIOA
//#define MCU_USB_FS_DP_Pin GPIO_PIN_12
//#define MCU_USB_FS_DP_GPIO_Port GPIOA
//#define XAV_SPI0__SS_Pin GPIO_PIN_15
//#define XAV_SPI0__SS_GPIO_Port GPIOA
//#define XAV_SPI0_SCK_Pin GPIO_PIN_10
//#define XAV_SPI0_SCK_GPIO_Port GPIOC
//#define XAV_SPI0_MISO_Pin GPIO_PIN_11
//#define XAV_SPI0_MISO_GPIO_Port GPIOC
//#define XAV_SPI0_MOSI_Pin GPIO_PIN_12
//#define XAV_SPI0_MOSI_GPIO_Port GPIOC
//#define MCU_FDCAN1_RX_Pin GPIO_PIN_0
//#define MCU_FDCAN1_RX_GPIO_Port GPIOD
//#define MCU_FDCAN1_TX_Pin GPIO_PIN_1
//#define MCU_FDCAN1_TX_GPIO_Port GPIOD
//#define XAVIER_BOOT_COMP_Pin GPIO_PIN_2
//#define XAVIER_BOOT_COMP_GPIO_Port GPIOD
//#define MCU_BOOT_COMP_Pin GPIO_PIN_3
//#define MCU_BOOT_COMP_GPIO_Port GPIOD
//#define GUIDE_L_Pin GPIO_PIN_4
//#define GUIDE_L_GPIO_Port GPIOD
//#define GUIDE_R_Pin GPIO_PIN_5
//#define GUIDE_R_GPIO_Port GPIOD
//#define MCU_USB_PWR_ON_Pin GPIO_PIN_6
//#define MCU_USB_PWR_ON_GPIO_Port GPIOD
//#define MCU_USB_OC__DET_Pin GPIO_PIN_7
//#define MCU_USB_OC__DET_GPIO_Port GPIOD
//#define MCU_RMII_TXD0_Pin GPIO_PIN_13
//#define MCU_RMII_TXD0_GPIO_Port GPIOG
//#define MCU_RMII_TXD1_Pin GPIO_PIN_14
//#define MCU_RMII_TXD1_GPIO_Port GPIOG
//#define MCU_PHY_RST_Pin GPIO_PIN_15
//#define MCU_PHY_RST_GPIO_Port GPIOG
//#define HIP_MOTOR_L_D_IN2_Pin GPIO_PIN_3
//#define HIP_MOTOR_L_D_IN2_GPIO_Port GPIOB
//#define HIP_MOTOR_R_L_IN1_Pin GPIO_PIN_4
//#define HIP_MOTOR_R_L_IN1_GPIO_Port GPIOB
//#define HIP_MOTOR_R_L_IN2_Pin GPIO_PIN_5
//#define HIP_MOTOR_R_L_IN2_GPIO_Port GPIOB
//#define STATUS_LED_I2C1_SCL_Pin GPIO_PIN_6
//#define STATUS_LED_I2C1_SCL_GPIO_Port GPIOB
//#define STATUS_LED_I2C1_SDA_Pin GPIO_PIN_7
//#define STATUS_LED_I2C1_SDA_GPIO_Port GPIOB
//#define STATUS_LED__RST_Pin GPIO_PIN_8
//#define STATUS_LED__RST_GPIO_Port GPIOB
//#define HIP_MOTOR_R_D_IN1_Pin GPIO_PIN_9
//#define HIP_MOTOR_R_D_IN1_GPIO_Port GPIOB
//#define HIP_L_D_SW_P_Pin GPIO_PIN_0
//#define HIP_L_D_SW_P_GPIO_Port GPIOE
//#define HIP_L_D_SW_N_Pin GPIO_PIN_1
//#define HIP_L_D_SW_N_GPIO_Port GPIOE
#endif /* WALKON5_CM_ENABLED */

#if defined (L30_CM_ENABLED) || defined (WALKON5_CM_ENABLED)
#define HIP_L_L_SW_P_Pin GPIO_PIN_2
#define HIP_L_L_SW_P_GPIO_Port GPIOE
#define HIP_L_L_SW_N_Pin GPIO_PIN_3
#define HIP_L_L_SW_N_GPIO_Port GPIOE
#define HIP_R_D_SW_P_Pin GPIO_PIN_4
#define HIP_R_D_SW_P_GPIO_Port GPIOE
#define TP68_Pin GPIO_PIN_5
#define TP68_GPIO_Port GPIOE
#define HIP_R_D_SW_N_Pin GPIO_PIN_6
#define HIP_R_D_SW_N_GPIO_Port GPIOE
#define MCU_EMR_Pin GPIO_PIN_13
#define MCU_EMR_GPIO_Port GPIOC
#define IMU_6AXIS_SDA_Pin GPIO_PIN_0
#define IMU_6AXIS_SDA_GPIO_Port GPIOF
#define IMU_6AXIS_SCL_Pin GPIO_PIN_1
#define IMU_6AXIS_SCL_GPIO_Port GPIOF
#define FET_SW_ON_Pin GPIO_PIN_2
#define FET_SW_ON_GPIO_Port GPIOF
#define SW_IC_INT_Pin GPIO_PIN_3
#define SW_IC_INT_GPIO_Port GPIOF
#define SW_IC_INT_EXTI_IRQn EXTI3_IRQn
#define ADD_R_AbsENC_CLK_Pin GPIO_PIN_7
#define ADD_R_AbsENC_CLK_GPIO_Port GPIOF
#define ADD_R_AbsENC_DATA_Pin GPIO_PIN_8
#define ADD_R_AbsENC_DATA_GPIO_Port GPIOF
#define HIP_L_L_ADC_Pin GPIO_PIN_10
#define HIP_L_L_ADC_GPIO_Port GPIOF
#define CURRENT_SENSOR_ADC_Pin GPIO_PIN_0
#define CURRENT_SENSOR_ADC_GPIO_Port GPIOC
#define MCU_PHY_MDC_Pin GPIO_PIN_1
#define MCU_PHY_MDC_GPIO_Port GPIOC
#define MCU_RMII_REF_CLK_Pin GPIO_PIN_1
#define MCU_RMII_REF_CLK_GPIO_Port GPIOA
#define MCU_PHY_MDIO_Pin GPIO_PIN_2
#define MCU_PHY_MDIO_GPIO_Port GPIOA
#define CAN0_SILENT_Pin GPIO_PIN_4
#define CAN0_SILENT_GPIO_Port GPIOA
#define CAN1_SILENT_Pin GPIO_PIN_5
#define CAN1_SILENT_GPIO_Port GPIOA
#define MCU_BUZZER_Pin GPIO_PIN_6
#define MCU_BUZZER_GPIO_Port GPIOA
#define MCU_RMII_CRS_DV_Pin GPIO_PIN_7
#define MCU_RMII_CRS_DV_GPIO_Port GPIOA
#define MCU_RMII_RXD0_Pin GPIO_PIN_4
#define MCU_RMII_RXD0_GPIO_Port GPIOC
#define MCU_RMII_RXD1_Pin GPIO_PIN_5
#define MCU_RMII_RXD1_GPIO_Port GPIOC
#define HIP_MOTOR_L_L_IN1_Pin GPIO_PIN_0
#define HIP_MOTOR_L_L_IN1_GPIO_Port GPIOB
#define HIP_MOTOR_L_L_IN2_Pin GPIO_PIN_1
#define HIP_MOTOR_L_L_IN2_GPIO_Port GPIOB
#define HIP_MOTOR_L_D_IN1_Pin GPIO_PIN_2
#define HIP_MOTOR_L_D_IN1_GPIO_Port GPIOB
#define HIP_L_D_ADC_Pin GPIO_PIN_11
#define HIP_L_D_ADC_GPIO_Port GPIOF
#define HIP_R_L_ADC_Pin GPIO_PIN_12
#define HIP_R_L_ADC_GPIO_Port GPIOF
#define HIP_R_D_ADC_Pin GPIO_PIN_13
#define HIP_R_D_ADC_GPIO_Port GPIOF
#define MCU_BAT_MON_I2C4_SCL_Pin GPIO_PIN_14
#define MCU_BAT_MON_I2C4_SCL_GPIO_Port GPIOF
#define MCU_BAT_MON_I2C4_SDA_Pin GPIO_PIN_15
#define MCU_BAT_MON_I2C4_SDA_GPIO_Port GPIOF
#define HIP_R_L_SW_P_Pin GPIO_PIN_7
#define HIP_R_L_SW_P_GPIO_Port GPIOE
#define HIP_R_L_SW_N_Pin GPIO_PIN_8
#define HIP_R_L_SW_N_GPIO_Port GPIOE
#define TP97_Pin GPIO_PIN_9
#define TP97_GPIO_Port GPIOE
#define TP98_Pin GPIO_PIN_10
#define TP98_GPIO_Port GPIOE
#define TP99_Pin GPIO_PIN_11
#define TP99_GPIO_Port GPIOE
#define ADD_L_AbsENC_CLK_Pin GPIO_PIN_12
#define ADD_L_AbsENC_CLK_GPIO_Port GPIOE
#define ADD_L_AbsENC_DATA_Pin GPIO_PIN_13
#define ADD_L_AbsENC_DATA_GPIO_Port GPIOE
#define HIP_MOTOR_R_D_IN2_Pin GPIO_PIN_10
#define HIP_MOTOR_R_D_IN2_GPIO_Port GPIOB
#define MCU_RMII_TX_EN_Pin GPIO_PIN_11
#define MCU_RMII_TX_EN_GPIO_Port GPIOB
#define MCU_FDCAN2_RX_Pin GPIO_PIN_12
#define MCU_FDCAN2_RX_GPIO_Port GPIOB
#define MCU_FDCAN2_TX_Pin GPIO_PIN_13
#define MCU_FDCAN2_TX_GPIO_Port GPIOB
#define XAVIER_MCU_SLEEP_Pin GPIO_PIN_8
#define XAVIER_MCU_SLEEP_GPIO_Port GPIOD
#define XAVIER_MCU_RST_Pin GPIO_PIN_9
#define XAVIER_MCU_RST_GPIO_Port GPIOD
#define XAVIER_MCU_OFF_REQ_Pin GPIO_PIN_10
#define XAVIER_MCU_OFF_REQ_GPIO_Port GPIOD
#define XAVIER_MCU_OFF_REQ_EXTI_IRQn EXTI15_10_IRQn
#define MCU_XAVIER_PWR_EN_Pin GPIO_PIN_11
#define MCU_XAVIER_PWR_EN_GPIO_Port GPIOD
#define MCU_5V_PWR_EN_Pin GPIO_PIN_12
#define MCU_5V_PWR_EN_GPIO_Port GPIOD
#define MCU_48V_MOTOR_ON_Pin GPIO_PIN_13
#define MCU_48V_MOTOR_ON_GPIO_Port GPIOD
#define MCU_3P3V_PWR_EN_Pin GPIO_PIN_14
#define MCU_3P3V_PWR_EN_GPIO_Port GPIOD
#define TP92_Pin GPIO_PIN_15
#define TP92_GPIO_Port GPIOD
#define MCU_INT_BUTTON1_Pin GPIO_PIN_2
#define MCU_INT_BUTTON1_GPIO_Port GPIOG
#define MCU_EMRG3_Pin GPIO_PIN_3
#define MCU_EMRG3_GPIO_Port GPIOG
#define MCU_12V_ON_Pin GPIO_PIN_5
#define MCU_12V_ON_GPIO_Port GPIOG
#define MCU_SW_CLR_Pin GPIO_PIN_7
#define MCU_SW_CLR_GPIO_Port GPIOG
#define MCU_INT_BUTTON2_Pin GPIO_PIN_8
#define MCU_INT_BUTTON2_GPIO_Port GPIOG
#define STAT_LED_NZR_L_Pin GPIO_PIN_6
#define STAT_LED_NZR_L_GPIO_Port GPIOC
#define STAT_LED_NZR_R_Pin GPIO_PIN_7
#define STAT_LED_NZR_R_GPIO_Port GPIOC
#define IMU_3AXIS_RDY_Pin GPIO_PIN_8
#define IMU_3AXIS_RDY_GPIO_Port GPIOC
#define IMU_3AXIS_SDA_Pin GPIO_PIN_9
#define IMU_3AXIS_SDA_GPIO_Port GPIOC
#define IMU_3AXIS_SCL_Pin GPIO_PIN_8
#define IMU_3AXIS_SCL_GPIO_Port GPIOA
#define MCU_USB_FS_VBUS_Pin GPIO_PIN_9
#define MCU_USB_FS_VBUS_GPIO_Port GPIOA
#define MCU_USB_ID_Pin GPIO_PIN_10
#define MCU_USB_ID_GPIO_Port GPIOA
#define MCU_USB_FS_DN_Pin GPIO_PIN_11
#define MCU_USB_FS_DN_GPIO_Port GPIOA
#define MCU_USB_FS_DP_Pin GPIO_PIN_12
#define MCU_USB_FS_DP_GPIO_Port GPIOA
#define XAV_SPI0__SS_Pin GPIO_PIN_15
#define XAV_SPI0__SS_GPIO_Port GPIOA
#define XAV_SPI0_SCK_Pin GPIO_PIN_10
#define XAV_SPI0_SCK_GPIO_Port GPIOC
#define XAV_SPI0_MISO_Pin GPIO_PIN_11
#define XAV_SPI0_MISO_GPIO_Port GPIOC
#define XAV_SPI0_MOSI_Pin GPIO_PIN_12
#define XAV_SPI0_MOSI_GPIO_Port GPIOC
#define MCU_FDCAN1_RX_Pin GPIO_PIN_0
#define MCU_FDCAN1_RX_GPIO_Port GPIOD
#define MCU_FDCAN1_TX_Pin GPIO_PIN_1
#define MCU_FDCAN1_TX_GPIO_Port GPIOD
#define XAVIER_BOOT_COMP_Pin GPIO_PIN_2
#define XAVIER_BOOT_COMP_GPIO_Port GPIOD
#define MCU_BOOT_COMP_Pin GPIO_PIN_3
#define MCU_BOOT_COMP_GPIO_Port GPIOD
#define GUIDE_L_Pin GPIO_PIN_4
#define GUIDE_L_GPIO_Port GPIOD
#define GUIDE_R_Pin GPIO_PIN_5
#define GUIDE_R_GPIO_Port GPIOD
#define MCU_USB_PWR_ON_Pin GPIO_PIN_6
#define MCU_USB_PWR_ON_GPIO_Port GPIOD
#define MCU_USB_OC__DET_Pin GPIO_PIN_7
#define MCU_USB_OC__DET_GPIO_Port GPIOD
#define MCU_RMII_TXD0_Pin GPIO_PIN_13
#define MCU_RMII_TXD0_GPIO_Port GPIOG
#define MCU_RMII_TXD1_Pin GPIO_PIN_14
#define MCU_RMII_TXD1_GPIO_Port GPIOG
#define MCU_PHY_RST_Pin GPIO_PIN_15
#define MCU_PHY_RST_GPIO_Port GPIOG
#define HIP_MOTOR_L_D_IN2_Pin GPIO_PIN_3
#define HIP_MOTOR_L_D_IN2_GPIO_Port GPIOB
#define HIP_MOTOR_R_L_IN1_Pin GPIO_PIN_4
#define HIP_MOTOR_R_L_IN1_GPIO_Port GPIOB
#define HIP_MOTOR_R_L_IN2_Pin GPIO_PIN_5
#define HIP_MOTOR_R_L_IN2_GPIO_Port GPIOB
#define STATUS_LED_I2C1_SCL_Pin GPIO_PIN_6
#define STATUS_LED_I2C1_SCL_GPIO_Port GPIOB
#define STATUS_LED_I2C1_SDA_Pin GPIO_PIN_7
#define STATUS_LED_I2C1_SDA_GPIO_Port GPIOB
#define STATUS_LED__RST_Pin GPIO_PIN_8
#define STATUS_LED__RST_GPIO_Port GPIOB
#define HIP_MOTOR_R_D_IN1_Pin GPIO_PIN_9
#define HIP_MOTOR_R_D_IN1_GPIO_Port GPIOB
#define HIP_L_D_SW_P_Pin GPIO_PIN_0
#define HIP_L_D_SW_P_GPIO_Port GPIOE
#define HIP_L_D_SW_N_Pin GPIO_PIN_1
#define HIP_L_D_SW_N_GPIO_Port GPIOE
#endif /* L30_CM_ENABLED */

#ifdef SUIT_MINICM_ENABLED
#define AUD_nSDMODE_Pin GPIO_PIN_6
#define AUD_nSDMODE_GPIO_Port GPIOE
#define IMU_6AXIS_I2C_SDA_Pin GPIO_PIN_0
#define IMU_6AXIS_I2C_SDA_GPIO_Port GPIOF
#define IMU_6AXIS_I2C_SCL_Pin GPIO_PIN_1
#define IMU_6AXIS_I2C_SCL_GPIO_Port GPIOF
#define _5V_DCDC_ON_Pin GPIO_PIN_2
#define _5V_DCDC_ON_GPIO_Port GPIOF
#define SW_IC_INT_Pin GPIO_PIN_3
#define SW_IC_INT_GPIO_Port GPIOF
#define RTC_SPI_MISO_Pin GPIO_PIN_2
#define RTC_SPI_MISO_GPIO_Port GPIOC
#define RTC_SPI_MOSI_Pin GPIO_PIN_3
#define RTC_SPI_MOSI_GPIO_Port GPIOC
#define MCU_USB_OC__DET_Pin GPIO_PIN_5
#define MCU_USB_OC__DET_GPIO_Port GPIOA
#define LTC2944_I2C_SCL_Pin GPIO_PIN_14
#define LTC2944_I2C_SCL_GPIO_Port GPIOF
#define LTC2944_I2C_SDA_Pin GPIO_PIN_15
#define LTC2944_I2C_SDA_GPIO_Port GPIOF
#define BLE_UART_EN_Pin GPIO_PIN_10
#define BLE_UART_EN_GPIO_Port GPIOE
#define BLE_RESET_Pin GPIO_PIN_11
#define BLE_RESET_GPIO_Port GPIOE
#define BLE_WAKEUP_Pin GPIO_PIN_14
#define BLE_WAKEUP_GPIO_Port GPIOE
#define BLE_nDEFAULT_Pin GPIO_PIN_15
#define BLE_nDEFAULT_GPIO_Port GPIOE
#define RTC_SPI_SCK_Pin GPIO_PIN_10
#define RTC_SPI_SCK_GPIO_Port GPIOB
#define BLE_UART_TX_Pin GPIO_PIN_14
#define BLE_UART_TX_GPIO_Port GPIOB
#define BLE_UART_RX_Pin GPIO_PIN_15
#define BLE_UART_RX_GPIO_Port GPIOB
#define LED_DRV_nRESET_Pin GPIO_PIN_8
#define LED_DRV_nRESET_GPIO_Port GPIOD
#define LED_DRV_nOE_Pin GPIO_PIN_9
#define LED_DRV_nOE_GPIO_Port GPIOD
#define ASSIST_BTN_P_Pin GPIO_PIN_10
#define ASSIST_BTN_P_GPIO_Port GPIOD
#define ASSIST_BTN_P_EXTI_IRQn EXTI15_10_IRQn
#define BLE_BT_CONNECT_Pin GPIO_PIN_11
#define BLE_BT_CONNECT_GPIO_Port GPIOD
#define BLE_BT_CONNECT_EXTI_IRQn EXTI15_10_IRQn
#define ASSIST_BTN_N_Pin GPIO_PIN_12
#define ASSIST_BTN_N_GPIO_Port GPIOD
#define ASSIST_BTN_N_EXTI_IRQn EXTI15_10_IRQn
#define SDCARD_DET_Pin GPIO_PIN_13
#define SDCARD_DET_GPIO_Port GPIOD
#define SDCARD_nWP_Pin GPIO_PIN_14
#define SDCARD_nWP_GPIO_Port GPIOD
#define MCU_24V_MOTOR_ON_Pin GPIO_PIN_2
#define MCU_24V_MOTOR_ON_GPIO_Port GPIOG
#define MC_5V_PWR_EN_Pin GPIO_PIN_3
#define MC_5V_PWR_EN_GPIO_Port GPIOG
#define WIDM_5V_PWR_EN_Pin GPIO_PIN_4
#define WIDM_5V_PWR_EN_GPIO_Port GPIOG
#define SDCARD_3V3_PWR_EN_Pin GPIO_PIN_5
#define SDCARD_3V3_PWR_EN_GPIO_Port GPIOG
#define PB_IN_MCU_Pin GPIO_PIN_6
#define PB_IN_MCU_GPIO_Port GPIOG
#define MCU_SW_CLR_Pin GPIO_PIN_7
#define MCU_SW_CLR_GPIO_Port GPIOG
#define USB_OTG_GPIO_Pin GPIO_PIN_8
#define USB_OTG_GPIO_GPIO_Port GPIOA
#define MCU_USB_FS_VBUS_Pin GPIO_PIN_9
#define MCU_USB_FS_VBUS_GPIO_Port GPIOA
#define MCU_USB_FS_ID_Pin GPIO_PIN_10
#define MCU_USB_FS_ID_GPIO_Port GPIOA
#define MCU_USB_FS_DM_Pin GPIO_PIN_11
#define MCU_USB_FS_DM_GPIO_Port GPIOA
#define MCU_USB_FS_DP_Pin GPIO_PIN_12
#define MCU_USB_FS_DP_GPIO_Port GPIOA
#define EXT_FLASH_SPI_NSS_Pin GPIO_PIN_15
#define EXT_FLASH_SPI_NSS_GPIO_Port GPIOA
#define MC_FDCAN_RX_Pin GPIO_PIN_0
#define MC_FDCAN_RX_GPIO_Port GPIOD
#define MC_FDCAN_TX_Pin GPIO_PIN_1
#define MC_FDCAN_TX_GPIO_Port GPIOD
#define RTC_nINT_Pin GPIO_PIN_4
#define RTC_nINT_GPIO_Port GPIOD
#define RTC_nRESET_Pin GPIO_PIN_5
#define RTC_nRESET_GPIO_Port GPIOD
#define MCU_USB_PWR_ON_Pin GPIO_PIN_6
#define MCU_USB_PWR_ON_GPIO_Port GPIOD
#define LED_DRIVE_SPI_MOSI_Pin GPIO_PIN_7
#define LED_DRIVE_SPI_MOSI_GPIO_Port GPIOD
#define LED_DRIVE_SPI_MISO_Pin GPIO_PIN_9
#define LED_DRIVE_SPI_MISO_GPIO_Port GPIOG
#define LED_DRIVE_SPI_NSS_Pin GPIO_PIN_10
#define LED_DRIVE_SPI_NSS_GPIO_Port GPIOG
#define LED_DRIVE_SPI_SCK_Pin GPIO_PIN_11
#define LED_DRIVE_SPI_SCK_GPIO_Port GPIOG
#define EXT_FLASH_SPI_WP__Pin GPIO_PIN_12
#define EXT_FLASH_SPI_WP__GPIO_Port GPIOG
#define EXT_FLASH_SPI_HOLD__Pin GPIO_PIN_15
#define EXT_FLASH_SPI_HOLD__GPIO_Port GPIOG
#define EXT_FLASH_SPI_SCK_Pin GPIO_PIN_3
#define EXT_FLASH_SPI_SCK_GPIO_Port GPIOB
#define EXT_FLASH_SPI_MISO_Pin GPIO_PIN_4
#define EXT_FLASH_SPI_MISO_GPIO_Port GPIOB
#define EXT_FLASH_SPI_MOSI_Pin GPIO_PIN_5
#define EXT_FLASH_SPI_MOSI_GPIO_Port GPIOB
#define IMU_3AXIS_I2C_SCL_Pin GPIO_PIN_6
#define IMU_3AXIS_I2C_SCL_GPIO_Port GPIOB
#define IMU_3AXIS_I2C_SDA_Pin GPIO_PIN_7
#define IMU_3AXIS_I2C_SDA_GPIO_Port GPIOB
#define IMU_3AXIS_I2C_RDY_Pin GPIO_PIN_8
#define IMU_3AXIS_I2C_RDY_GPIO_Port GPIOB
#define RTC_SPI_NSS_Pin GPIO_PIN_9
#define RTC_SPI_NSS_GPIO_Port GPIOB
#endif /* SUIT_MINICM_ENABLED */
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
