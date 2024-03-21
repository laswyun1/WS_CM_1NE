

#ifndef DEBUG_CTRL_INC_AS_DEBUG_CTRL_H_
#define DEBUG_CTRL_INC_AS_DEBUG_CTRL_H_

#include "module.h"

#ifdef SUIT_MINICM_ENABLED

#include <stdint.h>

#include "data_object_common.h"
#include "ioif_ltc2944.h"
#include "ioif_tim_common.h"
#include "ioif_usb_common.h"

#include "IOIF_Audio_WavPlay_SAI.h"
#include "ioif_fatfs.h"

/**
 *-----------------------------------------------------------
 *              MACROS AND PREPROCESSOR DIRECTIVES
 *-----------------------------------------------------------
 * @brief Directives and macros for readability and efficiency.
 */

#define IMU_6AXIS_BUFF_SIZE 32


/**
 *------------------------------------------------------------
 *                     TYPE DECLARATIONS
 *------------------------------------------------------------
 * @brief Custom data types and structures for the module.
 */




/**
 *------------------------------------------------------------
 *                      GLOBAL VARIABLES
 *------------------------------------------------------------
 * @brief Extern declarations for global variables.
 */

//extern TaskObj_t debug_task_dvs;

extern uint32_t debug_task_loop_time;

//extern IOIF_IMU6AxisData_t imu6AxisData;
//extern IOIF_MagData_t magData;

/**
 *------------------------------------------------------------
 *                     FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Function prototypes declaration for this module.
 */

void InitDebugTask(void);
void RunDebugTask(void);

#endif /* SUIT_MINICM_ENABLED */

#endif /* DEBUG_CTRL_INC_AS_DEBUG_CTRL_H_ */
