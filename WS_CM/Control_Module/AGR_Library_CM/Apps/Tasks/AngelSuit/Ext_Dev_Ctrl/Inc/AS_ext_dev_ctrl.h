

#ifndef EXT_DEV_CTRL_INC_AS_EXT_DEV_CTRL_H_
#define EXT_DEV_CTRL_INC_AS_EXT_DEV_CTRL_H_

#include "module.h"

#ifdef SUIT_MINICM_ENABLED

#include "data_object_common.h"
#include "task_mngr.h"

#include "ioif_tim_common.h"
#include "ioif_gpio_common.h"
#include "ioif_adc_common.h"

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

extern TaskObj_t extDevCtrl;



/**
 *------------------------------------------------------------
 *                     FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Function prototypes declaration for this module.
 */

void InitExtDevCtrl(void);
void RunExtDevCtrl(void);


#endif /* SUIT_MINICM_ENABLED */

#endif /* EXT_DEV_CTRL_INC_AS_EXT_DEV_CTRL_H_ */
