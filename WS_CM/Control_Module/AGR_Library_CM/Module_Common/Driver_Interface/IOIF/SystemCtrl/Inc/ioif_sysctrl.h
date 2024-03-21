/**
 *-----------------------------------------------------------
 *             System Control : LED & Power IC control
 *-----------------------------------------------------------
 * @file ioif_sysctrl.h
 * @date Created on: Dec 20, 2023
 * @author AngelRobotics HW Team
 * @brief
 *
 * @ref
 */

#ifndef IOIF_SYSTEMCTRL_IOIF_SYSCTRL_H_
#define IOIF_SYSTEMCTRL_IOIF_SYSCTRL_H_

#include "module.h"
#include "ioif_gpio_common.h"

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

typedef enum _IOIF_SysCtrl_t {
    IOIF_SYS_CTRL_OK = 0,
	IOIF_SYS_CTRL_ERROR,
} IOIF_SysCtrl_t;


typedef enum _IOIF_LEDStatus_t{
	IOIF_LED_OFF = 0,
	IOIF_LED_ON,
	IOIF_LED_TOGGLE,
} IOIF_LEDStatus_t;

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

IOIF_SysCtrl_t  IOIF_SysCtrlInit(void);
void 			IOIF_SysPwrOff(void);
bool 			IOIF_IsPwrBtnPressed(void);

void 			IOIF_MA_CtrlLEDforBattery(uint16_t LED_Pin, IOIF_LEDStatus_t onoff);
void 			IOIF_MA_CtrlLedforBoot(uint16_t LED_Pin, IOIF_LEDStatus_t onoff);


#endif /* IOIF_BSP_HEADER_BSP_CPU_TEMPERATURE_SENSING_H_ */
