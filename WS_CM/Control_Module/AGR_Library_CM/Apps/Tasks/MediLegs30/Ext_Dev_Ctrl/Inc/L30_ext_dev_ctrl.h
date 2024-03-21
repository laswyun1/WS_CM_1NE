

#ifndef EXT_DEV_CTRL_INC_L30_EXT_DEV_CTRL_H_
#define EXT_DEV_CTRL_INC_L30_EXT_DEV_CTRL_H_

#include "module.h"

#ifdef L30_CM_ENABLED

#include "data_object_common.h"
#include "task_mngr.h"

#include "ioif_tim_common.h"
#include "ioif_gpio_common.h"
#include "ioif_adc_common.h"

#include "ioif_tb67h450fngel.h"
#include "ioif_tp_lm_sensor.h"

/**
 *-----------------------------------------------------------
 *              MACROS AND PREPROCESSOR DIRECTIVES
 *-----------------------------------------------------------
 * @brief Directives and macros for readability and efficiency.
 */

#define PELV_UPRIGHT_OFFSET_LEN 258
#define PELV_UPRIGHT_OFFSET_DEP 260
#define PELV_UPRIGHT_SLOPE  	-0.001524


/**
 *------------------------------------------------------------
 *                     TYPE DECLARATIONS
 *------------------------------------------------------------
 * @brief Custom data types and structures for the module.
 */

typedef enum _PelvicUprightEnum {
	LEFT_LENGTH = 0,
	LEFT_DEPTH,
	RIGHT_LENGTH,
	RIGHT_DEPTH,
	PELVIC_UPRIGHT_NUM
} PelvicUprightEnum;


/**
 *------------------------------------------------------------
 *                      GLOBAL VARIABLES
 *------------------------------------------------------------
 * @brief Extern declarations for global variables.
 */

extern TaskObj_t extDevCtrl;

extern IOIF_UPRIGHT_t pelvicUpright[PELVIC_UPRIGHT_NUM];
extern uint8_t Upright_state;


/**
 *------------------------------------------------------------
 *                     FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Function prototypes declaration for this module.
 */

void InitExtDevCtrl(void);
void RunExtDevCtrl(void);


#endif /* L30_CM_ENABLED */

#endif /* EXT_DEV_CTRL_INC_L30_EXT_DEV_CTRL_H_ */
