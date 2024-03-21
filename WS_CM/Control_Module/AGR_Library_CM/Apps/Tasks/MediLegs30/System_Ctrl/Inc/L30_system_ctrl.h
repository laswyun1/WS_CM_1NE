/**
 * @file system_ctrl_task.h
 * @date Created on: Jul 28, 2023
 * @author AngelRobotics FW Team
 * @brief
 * @ref
 */

#ifndef SYSTEM_CTRL_INC_L30_SYSTEM_CTRL_H_
#define SYSTEM_CTRL_INC_L30_SYSTEM_CTRL_H_

#include "main.h"

#include "module.h"

#ifdef L30_CM_ENABLED

#include "data_object_common.h"

#include "ioif_tim_common.h"
#include "ioif_gpio_common.h"
#include "ioif_ltc2944.h"

#include "L30_whole_body_ctrl.h"
#include "ioif_nzr_led.h"
#include "ioif_battery_led.h"
#include "ioif_buzzer.h"


/**
 *-----------------------------------------------------------
 *              MACROS AND PREPROCESSOR DIRECTIVES
 *-----------------------------------------------------------
 * @brief Directives and macros for readability and efficiency.
 */

#define SYSTEM_MNGT_PERIOD 10

//IN ACCORDANCE WITH PCA9531.C/H
#define BATTERY_ONECOLOR -1


/**
 *------------------------------------------------------------
 *                     TYPE DECLARATIONS
 *------------------------------------------------------------
 * @brief Custom data types and structures for the module.
 */

typedef enum _IncDecState {
    UNKNOWN_ASSIST,
    INCREASE_ASSIST,
    DECREASE_ASSIST,
    INCDEC_NUM,
} IncDecState;

typedef enum _AssistStage{
    STAGE_0 = 0,
    STAGE_1,
    STAGE_2,
    STAGE_3,
    STAGE_4,
    STAGE_5,
    STAGE_6,
    STAGE_7,
    STAGE_8,
    STAGE_9
} AssistStage;

typedef enum _SystemState {
    SYSTEM_UNKNOWN,
    SYSTEM_OFF,
    SYSTEM_STANDBY,
    SYSTEM_ENABLE,
    SYSTEM_ERROR,
    SYSTEM_NUM,
} SystemState;

typedef enum _InitState {
    INIT_OK,
    INIT_ERROR,
    INIT_BUSY,
    INIT_TIMEOUT,
    INIT_NUM,
} InitState;


/**
 *------------------------------------------------------------
 *                      GLOBAL VARIABLES
 *------------------------------------------------------------
 * @brief Extern declarations for global variables.
 */

extern uint32_t SystemCtrlLoopCnt;
extern TaskObj_t systemCtrlTask;


/**
 *------------------------------------------------------------
 *                     FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Function prototypes declaration for this module.
 */

void InitSysCtrl(void);
void RunSysCtrl(void);

#endif /* L30_CM_ENABLED */

#endif /* SYSTEM_CTRL_INC_L30_SYSTEM_CTRL_H_ */
