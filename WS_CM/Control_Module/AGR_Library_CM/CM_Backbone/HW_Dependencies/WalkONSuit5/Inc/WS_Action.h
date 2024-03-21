

#ifndef CM_BACKBONE_HW_DEPENDENCIES_WALKONSUIT5_WS_ACTION_H_
#define CM_BACKBONE_HW_DEPENDENCIES_WALKONSUIT5_WS_ACTION_H_

#include "module.h"

#ifdef WALKON5_CM_ENABLED

#include "main.h"

#include <stdint.h>

/**
 *-----------------------------------------------------------
 *              MACROS AND PREPROCESSOR DIRECTIVES
 *-----------------------------------------------------------
 * @brief Directives and macros for readability and efficiency.
 */

#define FSM_BEEP_ALERT_TIM_FREQ        1000000U
#define FSM_BEEP_ALERT_FREQ_VERY_HIGH     1200U
#define FSM_BEEP_ALERT_FREQ_HIGH          1000U
#define FSM_BEEP_ALERT_FREQ_LOW            800U
#define FSM_BEEP_ALERT_DUTY                 50U
#define FSM_BEEP_ALERT_TIME_LONG           500U
#define FSM_BEEP_ALERT_TIME_SHORT          100U
#define FSM_BEEP_ALERT_TIME_VERY_SHORT      50U


/**
 *------------------------------------------------------------
 *                     TYPE DECLARATIONS
 *------------------------------------------------------------
 * @brief Custom data types and structures for the module.
 */

typedef enum _ActionID {

	NONE,
	BEEP1,
	BEEP2,
	BEEP3,
	BEEP4,
	BEEP5,
	BEEP6,
	BEEP7,
	BEEP8,
	BEEP9,
	BEEP10,


	ACTION_ID_NUM
} ActionID;


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

void Run_Audio_Action(uint16_t id);


#endif /* WALKON5_CM_ENABLED */

#endif /* CM_BACKBONE_HW_DEPENDENCIES_WALKONSUIT5_WS_ACTION_H_ */
