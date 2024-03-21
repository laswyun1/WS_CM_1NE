
#include "WS_Action.h"

#ifdef WALKON5_CM_ENABLED

/**
 *-----------------------------------------------------------
 *              TYPE DEFINITIONS AND ENUMERATIONS
 *-----------------------------------------------------------
 * @brief Enumerated types and structures central to this module.
 */




/**
 *------------------------------------------------------------
 *                      GLOBAL VARIABLES
 *------------------------------------------------------------
 * @brief Variables accessible throughout the application.
 */




/**
 *------------------------------------------------------------
 *                      STATIC VARIABLES
 *------------------------------------------------------------
 * @brief Variables local to this module.
 */




/**
 *------------------------------------------------------------
 *                 STATIC FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Static Function prototypes for this module.
 */




/**
 *------------------------------------------------------------
 *                      PUBLIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions that interface with this module.
 */

void Run_Audio_Action(uint16_t id)
{
	if      (id == BEEP1)
	{
        BeepAlert(FSM_BEEP_ALERT_FREQ_HIGH, FSM_BEEP_ALERT_DUTY, FSM_BEEP_ALERT_TIME_VERY_SHORT, FSM_BEEP_ALERT_TIME_VERY_SHORT);
	}
	else if (id == BEEP2)
	{
		BeepAlert(FSM_BEEP_ALERT_FREQ_HIGH, FSM_BEEP_ALERT_DUTY, FSM_BEEP_ALERT_TIME_VERY_SHORT, FSM_BEEP_ALERT_TIME_VERY_SHORT);
		BeepAlert(FSM_BEEP_ALERT_FREQ_HIGH, FSM_BEEP_ALERT_DUTY, FSM_BEEP_ALERT_TIME_VERY_SHORT, FSM_BEEP_ALERT_TIME_VERY_SHORT);
	}
	else if (id == BEEP3)
	{
		BeepAlert(FSM_BEEP_ALERT_FREQ_HIGH, FSM_BEEP_ALERT_DUTY, FSM_BEEP_ALERT_TIME_VERY_SHORT, FSM_BEEP_ALERT_TIME_VERY_SHORT);
		BeepAlert(FSM_BEEP_ALERT_FREQ_HIGH, FSM_BEEP_ALERT_DUTY, FSM_BEEP_ALERT_TIME_VERY_SHORT, FSM_BEEP_ALERT_TIME_VERY_SHORT);
		BeepAlert(FSM_BEEP_ALERT_FREQ_HIGH, FSM_BEEP_ALERT_DUTY, FSM_BEEP_ALERT_TIME_VERY_SHORT, FSM_BEEP_ALERT_TIME_VERY_SHORT);
	}

	else if (id == BEEP4)
	{
		BeepAlert(FSM_BEEP_ALERT_FREQ_HIGH, FSM_BEEP_ALERT_DUTY, FSM_BEEP_ALERT_TIME_VERY_SHORT, FSM_BEEP_ALERT_TIME_VERY_SHORT);
		BeepAlert(FSM_BEEP_ALERT_FREQ_HIGH, FSM_BEEP_ALERT_DUTY, FSM_BEEP_ALERT_TIME_VERY_SHORT, FSM_BEEP_ALERT_TIME_VERY_SHORT);
		BeepAlert(FSM_BEEP_ALERT_FREQ_HIGH, FSM_BEEP_ALERT_DUTY, FSM_BEEP_ALERT_TIME_VERY_SHORT, FSM_BEEP_ALERT_TIME_VERY_SHORT);
		BeepAlert(FSM_BEEP_ALERT_FREQ_HIGH, FSM_BEEP_ALERT_DUTY, FSM_BEEP_ALERT_TIME_VERY_SHORT, FSM_BEEP_ALERT_TIME_VERY_SHORT);
	}

	else if (id == BEEP5)
	{
		BeepAlert(FSM_BEEP_ALERT_FREQ_HIGH, FSM_BEEP_ALERT_DUTY, FSM_BEEP_ALERT_TIME_LONG, FSM_BEEP_ALERT_TIME_VERY_SHORT);
	}

	else if (id == BEEP6)
	{
		BeepAlert(FSM_BEEP_ALERT_FREQ_HIGH, FSM_BEEP_ALERT_DUTY, FSM_BEEP_ALERT_TIME_LONG, FSM_BEEP_ALERT_TIME_VERY_SHORT);
		BeepAlert(FSM_BEEP_ALERT_FREQ_HIGH, FSM_BEEP_ALERT_DUTY, FSM_BEEP_ALERT_TIME_LONG, FSM_BEEP_ALERT_TIME_VERY_SHORT);
	}

	else if (id == BEEP7)
	{
		BeepAlert(FSM_BEEP_ALERT_FREQ_HIGH, FSM_BEEP_ALERT_DUTY, FSM_BEEP_ALERT_TIME_VERY_SHORT, FSM_BEEP_ALERT_TIME_VERY_SHORT);
		BeepAlert(FSM_BEEP_ALERT_FREQ_LOW, FSM_BEEP_ALERT_DUTY,  FSM_BEEP_ALERT_TIME_VERY_SHORT, FSM_BEEP_ALERT_TIME_VERY_SHORT);
	}

	else if (id == BEEP8)
	{
		BeepAlert(FSM_BEEP_ALERT_FREQ_HIGH, FSM_BEEP_ALERT_DUTY, FSM_BEEP_ALERT_TIME_VERY_SHORT, FSM_BEEP_ALERT_TIME_VERY_SHORT);
		BeepAlert(FSM_BEEP_ALERT_FREQ_LOW, FSM_BEEP_ALERT_DUTY,  FSM_BEEP_ALERT_TIME_VERY_SHORT, FSM_BEEP_ALERT_TIME_VERY_SHORT);
		BeepAlert(FSM_BEEP_ALERT_FREQ_HIGH, FSM_BEEP_ALERT_DUTY, FSM_BEEP_ALERT_TIME_VERY_SHORT, FSM_BEEP_ALERT_TIME_VERY_SHORT);
		BeepAlert(FSM_BEEP_ALERT_FREQ_LOW, FSM_BEEP_ALERT_DUTY,  FSM_BEEP_ALERT_TIME_VERY_SHORT, FSM_BEEP_ALERT_TIME_VERY_SHORT);
	}
}


/**
 *------------------------------------------------------------
 *                      STATIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions intended for internal use within this module.
 */




#endif /* WALKON5_CM_ENABLED */