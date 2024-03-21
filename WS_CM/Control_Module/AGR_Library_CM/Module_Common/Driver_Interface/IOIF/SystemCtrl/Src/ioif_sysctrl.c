/**
 *-----------------------------------------------------------
 *             System Control : LED & Power IC control
 *-----------------------------------------------------------
 * @file ioif_sysctrl.c
 * @date Created on: Dec 20, 2023
 * @author AngelRobotics HW Team
 * @brief
 *
 * @ref
 */

#include "ioif_sysctrl.h"


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

static volatile bool pwr_btn_pressed = false;


/**
 *------------------------------------------------------------
 *                 STATIC FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Static Function prototypes for this module.
 */

static void SetPWRICCB(uint16_t gpioPin);

/**
 *------------------------------------------------------------
 *                      PUBLIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions that interface with this module.
 */

IOIF_SysCtrl_t IOIF_SysCtrlInit(void)
{
	IOIF_SysCtrl_t ret = IOIF_SYS_CTRL_OK;

#ifdef CM_MODULE
	/* Power IC GPIO Callback Registration */
	ret = IOIF_SetGPIOCB(SW_IC_INT_Pin, IOIF_GPIO_EXTI_CALLBACK, SetPWRICCB);
#endif

	return ret;
}

bool IOIF_IsPwrBtnPressed(void)
{
	bool ret = false;

	if(pwr_btn_pressed == true)
		ret = true;
	else ret = false;

	return ret;
}


void IOIF_SysPwrOff(void)
{
#ifdef CM_MODULE
	IOIF_WriteGPIOPin(IOIF_GPIO_PORT_G, MCU_SW_CLR_Pin, LOW);
#endif
}

/*void IOIF_MA_CtrlLEDforBattery(uint16_t LED_Pin, IOIF_LEDStatus_t onoff)
{
	if (onoff != IOIF_LED_TOGGLE)
		IOIF_WriteGPIOPin(IOIF_GPIO_PORT_E, LED_Pin, (IOIF_GPIOPinState_t)onoff);
	else
		IOIF_ToggleGPIOPin(IOIF_GPIO_PORT_E, LED_Pin);
}

void IOIF_MA_CtrlLedforBoot(uint16_t LED_Pin, IOIF_LEDStatus_t onoff)
{
	switch(LED_Pin){
	case LED_BOOT_BLUE_Pin:
		if (onoff != IOIF_LED_TOGGLE)
			IOIF_WriteGPIOPin(IOIF_GPIO_PORT_E, LED_BOOT_BLUE_Pin, (IOIF_GPIOPinState_t)onoff);
		else
			IOIF_ToggleGPIOPin(IOIF_GPIO_PORT_E, LED_BOOT_BLUE_Pin);
		break;
	case MCU_BOOT_Pin:
		if (onoff != IOIF_LED_TOGGLE)
			IOIF_WriteGPIOPin(IOIF_GPIO_PORT_C, MCU_BOOT_Pin, (IOIF_GPIOPinState_t)onoff);
		else
			IOIF_ToggleGPIOPin(IOIF_GPIO_PORT_C, MCU_BOOT_Pin);
		break;
	default:
		break;
	}
}*/


/**
 *------------------------------------------------------------
 *                      STATIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions intended for internal use within this module.
 */

/* ------------------- Function ------------------- */

static void SetPWRICCB(uint16_t gpioPin)
{
#ifdef CM_MODULE
	if(gpioPin == SW_IC_INT_Pin)
		pwr_btn_pressed = true;
#endif
}

