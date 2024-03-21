

#ifndef DATA_CTRL_INC_AS_DATA_CTRL_H_
#define DATA_CTRL_INC_AS_DATA_CTRL_H_

#include "main.h"

#include "module.h"

#ifdef SUIT_MINICM_ENABLED

#include "data_object_common.h"
#include "IOIF_Audio_WavPlay_SAI.h"
#include "ioif_fatfs.h"
#include "ring_buffer.h"

#include "AS_whole_body_ctrl.h"
#include "ioif_pca9957hnmp.h"


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


extern osMutexId_t SDIOmutexHandle;



/**
 *------------------------------------------------------------
 *                     FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Function prototypes declaration for this module.
 */

void InitDataCtrl(void);
void RunDataCtrl(void);


#endif /* SUIT_MINICM_ENABLED */

#endif /* APPS_TASKS_ANGELSUIT_DATA_CTRL_INC_AS_DATA_CTRL_H_ */
