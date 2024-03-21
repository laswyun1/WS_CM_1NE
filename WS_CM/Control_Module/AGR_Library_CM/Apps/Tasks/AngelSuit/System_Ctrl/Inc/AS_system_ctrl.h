/**
 * @file system_ctrl_task.h
 * @date Created on: Jul 28, 2023
 * @author AngelRobotics FW Team
 * @brief
 * @ref
 */

#ifndef SYSTEM_CTRL_INC_AS_SYSTEM_CTRL_H_
#define SYSTEM_CTRL_INC_AS_SYSTEM_CTRL_H_

#include "module.h"

#ifdef SUIT_MINICM_ENABLED

#include <stdint.h>
#include <stdlib.h> // for qsort

#include "main.h"
#include "data_object_common.h"

#include "data_object_dictionaries.h"
#include "error_dictionary.h"


#include "ioif_tim_common.h"
#include "ioif_gpio_common.h"
#include "ioif_ltc2944.h"
#include "ioif_pca9957hnmp.h"
#include "ioif_usb_common.h"
#include "ioif_sysctrl.h"

#include "AS_whole_body_ctrl.h"
#include "AS_ISI.h"

#include "AS_ble_comm_hdlr.h"

/**
 *-----------------------------------------------------------
 *              MACROS AND PREPROCESSOR DIRECTIVES
 *-----------------------------------------------------------
 * @brief Directives and macros for readability and efficiency.
 */


#define BATTERY_VOLT_MAX 25.2
#define BATTERY_VOLT_MIN 18
#define DEBOUNCE_TIME 50 // 50ms의 디바운스 시간

#define BAT_VOLT_SAMPLE_SIZE 1000 // Samples 1000 times in a minute
#define BAT_VOLT_SAMPLING_INTERVAL (60000 / BAT_VOLT_SAMPLE_SIZE) // Sampling 1000 times in a minute, 60seconds * 1000 ms / 1000 Samples.


/**
 *------------------------------------------------------------
 *                     TYPE DECLARATIONS
 *------------------------------------------------------------
 * @brief Custom data types and structures for the module.
 */

typedef enum _GaitModeByBt_t {
    GAIT_MODE_STOP,
    GAIT_MODE_WALK,
    GAIT_MODE_RUN,
    GAIT_MODE_CLIMB
} GaitModeByBt_t;

typedef enum _IncDecState {
    UNKNOWN_ASSIST,
    INCREASE_ASSIST,
    DECREASE_ASSIST,
    INCDEC_NUM,
} IncDecState;

typedef enum _AssistStage {
    STAGE_0 = 0,
    STAGE_1,
    STAGE_2,
    STAGE_3,
    STAGE_4,
    STAGE_5,
    STAGE_6,
    STAGE_7,
    STAGE_8,
    STAGE_9,
    STAGE_10,
    STAGE_NUM
} AssistStage;

typedef enum _SystemState {
    SYSTEM_UNKNOWN,
    SYSTEM_IDLE,
    SYSTEM_ASSIST,
    SYSTEM_FITNESS,
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

extern IOIF_BatData_t batData;
extern float medianVolt;

extern uint8_t systemStateFlag;
extern uint8_t wholeTaskFlag;
extern float batPctg;
extern AssistStage assistStage;
extern uint8_t testFlag;
extern uint8_t assistFlag;
extern float assistForcePctg;

extern GaitModeByBt_t gaitModeState;


/**
 *------------------------------------------------------------
 *                     FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Function prototypes declaration for this module.
 */

void InitSysCtrl(void);
void RunSysCtrl(void);

void UpdateLED(void);
void PwrOffBt(void);

#endif /* SUIT_MINICM_ENABLED */

#endif /* SYSTEM_CTRL_INC_AS_SYSTEM_CTRL_H_ */
