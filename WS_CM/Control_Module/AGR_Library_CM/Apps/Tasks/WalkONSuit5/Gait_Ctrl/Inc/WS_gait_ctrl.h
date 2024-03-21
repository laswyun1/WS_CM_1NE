

#ifndef APPS_TASKS_ANGELSUIT_GAIT_CTRL_INC_WS_GAIT_CTRL_H_
#define APPS_TASKS_ANGELSUIT_GAIT_CTRL_INC_WS_GAIT_CTRL_H_

#include "module.h"

#ifdef WALKON5_CM_ENABLED

#include "data_object_common.h"

#include "WS_widm_algorithms.h"

#include "WS_vqf.h"

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


#endif /* WALKON5_CM_ENABLED */

#endif /* APPS_TASKS_ANGELSUIT_GAIT_CTRL_INC_WS_GAIT_CTRL_H_ */
