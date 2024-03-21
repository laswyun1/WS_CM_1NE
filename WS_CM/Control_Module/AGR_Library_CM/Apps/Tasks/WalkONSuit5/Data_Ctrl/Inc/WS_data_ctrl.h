#ifndef APPS_TASKS_WALKONSUIT5_DATA_CTRL_INC_WS_DATA_CTRL_H_
#define APPS_TASKS_WALKONSUIT5_DATA_CTRL_INC_WS_DATA_CTRL_H_

#include "main.h"

#include "module.h"

#ifdef WALKON5_CM_ENABLED

#include <stdbool.h>

#include "data_object_common.h"
#include "ioif_fatfs.h"
#include "ring_buffer.h"

#include "ioif_usb_common.h"
#include "ioif_fatfs.h"

#include "WS_whole_body_ctrl.h"


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

void InitDataCtrl(void);
void RunDataCtrl(void);


#endif /* WALKON5_CM_ENABLED */

#endif /* APPS_TASKS_WALKONSUIT5_DATA_CTRL_INC_WS_DATA_CTRL_H_ */
