

#ifndef APPS_TASKS_ANGELSUIT_IMU_CTRL_INC_AS_IMU_CTRL_H_
#define APPS_TASKS_ANGELSUIT_IMU_CTRL_INC_AS_IMU_CTRL_H_

#include "module.h"

#ifdef SUIT_MINICM_ENABLED

#include <stdbool.h>

#include "data_object_common.h"
#include "data_object_dictionaries.h"

#include "ioif_gpio_common.h"
#include "ioif_icm20608g.h"
#include "ioif_bm1422agmv.h"

#include "AS_vqf.h"

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

extern TaskObj_t imuCtrlTask;

extern IOIF_6AxisData_t CM_imu6AxisDataObj;
extern IOIF_MagData_t   CM_magDataObj;

extern VQF_MagCalib_t CM_vqfMagCalibObj;


/**
 *------------------------------------------------------------
 *                     FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Function prototypes declaration for this module.
 */

void InitIMUCtrl(void);
void RunIMUCtrl(void);


#endif /* SUIT_MINICM_ENABLED */

#endif /* APPS_TASKS_ANGELSUIT_IMU_CTRL_INC_AS_IMU_CTRL_H_ */
