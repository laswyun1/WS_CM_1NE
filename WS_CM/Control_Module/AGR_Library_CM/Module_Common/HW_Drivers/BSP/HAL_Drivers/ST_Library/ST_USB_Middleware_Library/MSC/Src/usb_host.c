/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file            : usb_host.c
  * @version         : v1.0_Cube
  * @brief           : This file implements the USB Host
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

#include "module.h"

#ifdef CM_MODULE

/* Includes ------------------------------------------------------------------*/

#include "usb_host.h"
#include "usbh_core.h"
#include "usbh_msc.h"

/* USER CODE BEGIN Includes */

//#include "apps/boot_ctrl/boot_ctrl.h"

/* USER CODE END Includes */

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/


/* USER CODE END PV */

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USB Host core handle declaration */
//USBH_HandleTypeDef hUsbHostFS __attribute__((section(".USBHOST_data")));
USBH_HandleTypeDef hUsbHostFS;
ApplicationTypeDef Appli_state  = APPLICATION_IDLE;

/*
 * -- Insert your variables declaration here --
 */
/* USER CODE BEGIN 0 */

//USBH_HandleTypeDef UsbHostFS_h __attribute__((section(".FATFS_RAMD1_data")));
//USBH_HandleTypeDef UsbHostFS_h;

/* USER CODE END 0 */

/*
 * user callback declaration
 */
static void USBH_UserProcess(USBH_HandleTypeDef *phost, uint8_t id);

/*
 * -- Insert your external function declaration here --
 */
/* USER CODE BEGIN 1 */

/* USB Host Mode Initialization */

bool USB_HOST_Mode_Init(void)
{
	bool ret = true;
  /* USER CODE BEGIN USB_HOST_Init_PreTreatment */
	HAL_PWREx_EnableUSBVoltageDetector();
	Appli_state = APPLICATION_IDLE;
  /* USER CODE END USB_HOST_Init_PreTreatment */

  /* Init host Library, add supported class and start the library. */
  if (USBH_Init(&hUsbHostFS, USBH_UserProcess, HOST_FS) != USBH_OK)
  {
    ret = false;
  }
  if (USBH_RegisterClass(&hUsbHostFS, USBH_MSC_CLASS) != USBH_OK)
  {
	ret = false;
  }
  if (USBH_Start(&hUsbHostFS) != USBH_OK)
  {
	ret = false;
  }

  return ret;
}


/* USER CODE END 1 */

/**
  * Init USB host library, add supported class and start the library
  * @retval None
  */
//void MX_USB_HOST_Init(void)
//{
//  /* USER CODE BEGIN USB_HOST_Init_PreTreatment */
//	HAL_PWREx_EnableUSBVoltageDetector();
//	Appli_state = APPLICATION_IDLE;
//  /* USER CODE END USB_HOST_Init_PreTreatment */
//
//  /* Init host Library, add supported class and start the library. */
//  if (USBH_Init(&hUsbHostFS, USBH_UserProcess, HOST_FS) != USBH_OK)
//  {
//    Error_Handler();
//  }
//  if (USBH_RegisterClass(&hUsbHostFS, USBH_MSC_CLASS) != USBH_OK)
//  {
//    Error_Handler();
//  }
//  if (USBH_Start(&hUsbHostFS) != USBH_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN USB_HOST_Init_PostTreatment */
//
//  /* USER CODE END USB_HOST_Init_PostTreatment */
//}

/*
 * user callback definition
 */
static void USBH_UserProcess  (USBH_HandleTypeDef *phost, uint8_t id)
{
  /* USER CODE BEGIN CALL_BACK_1 */
  switch(id)
  {
  case HOST_USER_SELECT_CONFIGURATION:
  break;

  case HOST_USER_DISCONNECTION:
  Appli_state = APPLICATION_DISCONNECT;
  break;

  case HOST_USER_CLASS_ACTIVE:
  Appli_state = APPLICATION_READY;
  break;

  case HOST_USER_CONNECTION:
  Appli_state = APPLICATION_START;

  break;

  default:
  break;
  }
  /* USER CODE END CALL_BACK_1 */
}

/**
  * @}
  */

/**
  * @}
  */

#endif /* CM_MODULE */
