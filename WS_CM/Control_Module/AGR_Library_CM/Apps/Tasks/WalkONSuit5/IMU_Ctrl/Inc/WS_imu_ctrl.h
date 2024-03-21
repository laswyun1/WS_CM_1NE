

#ifndef APPS_TASKS_ANGELSUIT_IMU_CTRL_INC_WS_IMU_CTRL_H_
#define APPS_TASKS_ANGELSUIT_IMU_CTRL_INC_WS_IMU_CTRL_H_

#include "module.h"

#ifdef WALKON5_CM_ENABLED

#include <stdbool.h>

#include "data_object_common.h"

#include "ioif_tim_common.h"
#include "ioif_gpio_common.h"

#include "ioif_icm20608g.h"
#include "ioif_bm1422agmv.h"

#include "WS_dev_mngr.h"
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

void InitIMUCtrl(void);
void RunIMUCtrl(void);


#endif /* WALKON5_CM_ENABLED */

#endif /* APPS_TASKS_ANGELSUIT_IMU_CTRL_INC_WS_IMU_CTRL_H_ */
