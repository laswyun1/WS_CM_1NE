#ifndef DATA_CTRL_INC_L30_DATA_CTRL_H_
#define DATA_CTRL_INC_L30_DATA_CTRL_H_

#include "main.h"

#include "module.h"

#ifdef L30_CM_ENABLED

#include <stdbool.h>

#include "data_object_common.h"
#include "ioif_fatfs.h"
#include "ring_buffer.h"

/* FATFS test */
#include "ioif_usb_common.h"
#include "ioif_fatfs.h"

#include "L30_whole_body_ctrl.h"


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


#endif /* L30_CM_ENABLED */

#endif /* DATA_CTRL_INC_L30_DATA_CTRL_H_ */
