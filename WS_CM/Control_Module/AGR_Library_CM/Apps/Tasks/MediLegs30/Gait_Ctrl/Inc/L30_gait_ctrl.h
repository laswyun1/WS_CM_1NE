

#ifndef APPS_TASKS_ANGELSUIT_GAIT_CTRL_INC_L30_GAIT_CTRL_H_
#define APPS_TASKS_ANGELSUIT_GAIT_CTRL_INC_L30_GAIT_CTRL_H_

#include "module.h"

#ifdef L30_CM_ENABLED

#include "data_object_common.h"

#include "L30_widm_algorithms.h"

#include "L30_vqf.h"

/**
 *-----------------------------------------------------------
 *              MACROS AND PREPROCESSOR DIRECTIVES
 *-----------------------------------------------------------
 * @brief Directives and macros for readability and efficiency.
 */




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




/**
 *------------------------------------------------------------
 *                     FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Function prototypes declaration for this module.
 */

void InitGaitCtrl(void);
void RunGaitCtrl(void);


#endif /* L30_CM_ENABLED */

#endif /* APPS_TASKS_ANGELSUIT_GAIT_CTRL_INC_L30_GAIT_CTRL_H_ */
