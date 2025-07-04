/**
  ******************************************************************************
  * @file    usbh_core.h
  * @author  MCD Application Team
  * @brief   Header file for usbh_core.c
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2015 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* Define to prevent recursive  ----------------------------------------------*/
#ifndef __USBH_CORE_H
#define __USBH_CORE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "module.h"

#ifdef CM_MODULE

/* Includes ------------------------------------------------------------------*/
#include "usbh_conf.h"
#include "usbh_def.h"
#include "usbh_ioreq.h"
#include "usbh_pipes.h"
#include "usbh_ctlreq.h"

/** @addtogroup USBH_LIB
  * @{
  */

/** @addtogroup USBH_LIB_CORE
  * @{
  */

/** @defgroup USBH_CORE
  * @brief This file is the Header file for usbh_core.c
  * @{
  */


/** @defgroup USBH_CORE_Exported_Defines
  * @{
  */

/**
  * @}
  */
#define HOST_USER_SELECT_CONFIGURATION          0x01U
#define HOST_USER_CLASS_ACTIVE                  0x02U
#define HOST_USER_CLASS_SELECTED                0x03U
#define HOST_USER_CONNECTION                    0x04U
#define HOST_USER_DISCONNECTION                 0x05U
#define HOST_USER_UNRECOVERED_ERROR             0x06U


/**
  * @}
  */



/** @defgroup USBH_CORE_Exported_Macros
  * @{
  */

/**
  * @}
  */

/** @defgroup USBH_CORE_Exported_Variables
  * @{
  */

/**
  * @}
  */

/** @defgroup USBH_CORE_Exported_FunctionsPrototype
  * @{
  */


USBH_StatusTypeDef  USBH_Init(USBH_HandleTypeDef *phost, void (*pUsrFunc)(USBH_HandleTypeDef *phost, uint8_t id), uint8_t id);
USBH_StatusTypeDef  USBH_DeInit(USBH_HandleTypeDef *phost);
USBH_StatusTypeDef  USBH_RegisterClass(USBH_HandleTypeDef *phost, USBH_ClassTypeDef *pclass);
USBH_StatusTypeDef  USBH_SelectInterface(USBH_HandleTypeDef *phost, uint8_t interface);
uint8_t             USBH_FindInterface(USBH_HandleTypeDef *phost,
                                       uint8_t Class,
                                       uint8_t SubClass,
                                       uint8_t Protocol);
uint8_t             USBH_GetActiveClass(USBH_HandleTypeDef *phost);

uint8_t             USBH_FindInterfaceIndex(USBH_HandleTypeDef *phost,
                                            uint8_t interface_number,
                                            uint8_t alt_settings);

uint8_t              USBH_IsPortEnabled(USBH_HandleTypeDef *phost);

USBH_StatusTypeDef  USBH_Start(USBH_HandleTypeDef *phost);
USBH_StatusTypeDef  USBH_Stop(USBH_HandleTypeDef *phost);
USBH_StatusTypeDef  USBH_Process(USBH_HandleTypeDef *phost);
USBH_StatusTypeDef  USBH_ReEnumerate(USBH_HandleTypeDef *phost);

/* USBH Low Level Driver */
USBH_StatusTypeDef   USBH_LL_Init(USBH_HandleTypeDef *phost);
USBH_StatusTypeDef   USBH_LL_DeInit(USBH_HandleTypeDef *phost);
USBH_StatusTypeDef   USBH_LL_Start(USBH_HandleTypeDef *phost);
USBH_StatusTypeDef   USBH_LL_Stop(USBH_HandleTypeDef *phost);

USBH_StatusTypeDef   USBH_LL_Connect(USBH_HandleTypeDef *phost);
USBH_StatusTypeDef   USBH_LL_Disconnect(USBH_HandleTypeDef *phost);
USBH_SpeedTypeDef    USBH_LL_GetSpeed(USBH_HandleTypeDef *phost);
USBH_StatusTypeDef   USBH_LL_ResetPort(USBH_HandleTypeDef *phost);
uint32_t             USBH_LL_GetLastXferSize(USBH_HandleTypeDef *phost,
                                             uint8_t pipe);

USBH_StatusTypeDef   USBH_LL_DriverVBUS(USBH_HandleTypeDef *phost,
                                        uint8_t state);

USBH_StatusTypeDef   USBH_LL_OpenPipe(USBH_HandleTypeDef *phost,
                                      uint8_t pipe,
                                      uint8_t epnum,
                                      uint8_t dev_address,
                                      uint8_t speed,
                                      uint8_t ep_type,
                                      uint16_t mps);

USBH_StatusTypeDef   USBH_LL_ClosePipe(USBH_HandleTypeDef *phost,
                                       uint8_t pipe);

USBH_StatusTypeDef   USBH_LL_SubmitURB(USBH_HandleTypeDef *phost,
                                       uint8_t pipe,
                                       uint8_t direction,
                                       uint8_t ep_type,
                                       uint8_t token,
                                       uint8_t *pbuff,
                                       uint16_t length,
                                       uint8_t do_ping);

USBH_URBStateTypeDef USBH_LL_GetURBState(USBH_HandleTypeDef *phost,
                                         uint8_t pipe);

#if (USBH_USE_OS == 1U)
USBH_StatusTypeDef  USBH_LL_NotifyURBChange(USBH_HandleTypeDef *phost);
#endif

USBH_StatusTypeDef USBH_LL_SetToggle(USBH_HandleTypeDef *phost,
                                     uint8_t pipe, uint8_t toggle);

uint8_t USBH_LL_GetToggle(USBH_HandleTypeDef *phost, uint8_t pipe);

void                 USBH_LL_PortDisabled(USBH_HandleTypeDef *phost);
void                 USBH_LL_PortEnabled(USBH_HandleTypeDef *phost);

/* USBH Time base */
void USBH_LL_SetTimer(USBH_HandleTypeDef *phost, uint32_t time);
void USBH_LL_IncTimer(USBH_HandleTypeDef *phost);

void USBH_Delay(uint32_t Delay);

/**
  * @}
  */

#endif /* CM_MODULE */

#ifdef __cplusplus
}
#endif

#endif /* __USBH_CORE_H */
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

