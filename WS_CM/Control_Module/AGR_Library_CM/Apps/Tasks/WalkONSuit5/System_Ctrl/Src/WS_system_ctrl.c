/**
 * @file system_ctrl_task.c
 * @date Created on: Jul 28, 2023
 * @author AngelRobotics FW Team
 * @brief
 * @ref
 */

#include "WS_system_ctrl.h"

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

TaskObj_t sysCtrlTask;

IOIF_BatData_t batData;
float batPctg;
uint32_t sysCtrlLoopCnt;

extern USBH_HandleTypeDef hUsbHostFS;
volatile static uint8_t sbcPowerState=0;

uint8_t power_state;
uint8_t powerOff;


/**
 *------------------------------------------------------------
 *                      STATIC VARIABLES
 *------------------------------------------------------------
 * @brief Variables local to this module.
 */

// for debug
uint8_t bmInitRes = IOIF_BAT_STATUS_OK;
static uint8_t bmRes = IOIF_BAT_STATUS_OK;

static uint16_t powerOff_cnt;
static uint8_t d10_power_state;
static IOIF_NzrLedObj_t nzrLedObj[2];
static IOIF_BuzzerObj_t buzzerObj;

static uint16_t buzzer_test[4] = {100, 200, 300, 400};

uint8_t IsRecoverI2C4 = false;


/**
 *------------------------------------------------------------
 *                 STATIC FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Static Function prototypes for this module.
 */

/* ------------------- STATE FUNCTIONS ------------------- */
static void StateOff_Run(void);

static void StateStandby_Run(void);

static void StateEnable_Ent(void);
static void StateEnable_Run(void);
static void StateEnable_Ext(void);

static void StateError_Run(void);

/* ------------------- BOOT SEQUENCE ------------------- */
static void SBC_BootSequence(void);
static void D10_BootSequence(void);

/* ------------------- GET LTC2944 VALUE ------------------- */
static int GetBatVal_Ent(void);
static int GetBatVal_Run(void);

/* ------------------- Init Baterry Monitor ------------------- */
static void InitBatteryMonitor(void);


/**
 *------------------------------------------------------------
 *                      PUBLIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions that interface with this module.
 */

/* ------------------- SDO CALLBACK ------------------- */
DOP_COMMON_SDO_CB(sysCtrlTask)

uint8_t led_driver_init_flag = 0;

void InitSysCtrl(void)
{
    // init
    InitTask(&sysCtrlTask);
	DOPC_AssignTaskID(&sysCtrlTask, TASK_IDX_SYSTEM_CTRL);
	sysCtrlTask.period = 10;

    /* State Definition */
	TASK_CREATE_STATE(&sysCtrlTask, TASK_STATE_OFF,     NULL,				StateOff_Run,    	NULL,             false);
	TASK_CREATE_STATE(&sysCtrlTask, TASK_STATE_STANDBY, NULL,				StateStandby_Run,	NULL,             true);
	TASK_CREATE_STATE(&sysCtrlTask, TASK_STATE_ENABLE,  StateEnable_Ent,	StateEnable_Run, 	StateEnable_Ext,  false);
	TASK_CREATE_STATE(&sysCtrlTask, TASK_STATE_ERROR,   NULL,				StateError_Run,  	NULL,             false);

    /* Routine Definition */

    /* DOD Definition */
	// DOD
    DOP_CreateDOD(TASK_IDX_SYSTEM_CTRL);
    // PDO
    DOP_COMMON_PDO_CREATE(TASK_IDX_SYSTEM_CTRL, sysCtrlTask);

    // SDO
    DOP_COMMON_SDO_CREATE(TASK_IDX_SYSTEM_CTRL)

	InitBatteryMonitor();
}

void RunSysCtrl(void)
{
	RunTask(&sysCtrlTask);
}

/**
 *------------------------------------------------------------
 *                      STATIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions intended for internal use within this module.
 */

static void StateOff_Run(void)
{

}

static void StateStandby_Run(void)
{
	// WS_StartSensorStreaming();

//	/************* DEMO: Ready MTR  *************/
	//BeepAlert(BEEP_ALERT_FREQ_LOW,  BEEP_ALERT_DUTY, BEEP_ALERT_TIME_VERY_SHORT, BEEP_ALERT_TIME_VERY_SHORT);
	//BeepAlert(BEEP_ALERT_FREQ_HIGH, BEEP_ALERT_DUTY, BEEP_ALERT_TIME_VERY_SHORT, BEEP_ALERT_TIME_VERY_SHORT);
//	/********************************************/
	if (sbcPowerState == 0) {
		SBC_BootSequence();
	} else if (d10_power_state == 0) {
		D10_BootSequence();
	} else {
		StateTransition(&sysCtrlTask.stateMachine, TASK_STATE_ENABLE);
	}
	sysCtrlLoopCnt++;
	TaskDelay(&sysCtrlTask);
}

static void StateEnable_Ent(void)
{
	sysCtrlLoopCnt = 0;
	EntRoutines(&sysCtrlTask.routine);
}

static void StateEnable_Run(void)
{
	if (power_state == 0 && IOIF_ReadGPIOPin(IOIF_GPIO_PORT_D, IOIF_GPIO_PIN_10) == IOIF_GPIO_PIN_SET) {
		SBC_BootSequence();
	}

	if (power_state == 1) {
		D10_BootSequence();
	}

	GetBatVal_Run();

	//	Run_Routines(&sysCtrlTask.routine);

	sysCtrlLoopCnt++;
}

static void StateEnable_Ext(void)
{
	ExtRoutines(&sysCtrlTask.routine);
}

static void StateError_Run(void)
{
	// TODO : Error Handle
	// TaskDelay(&sysCtrlTask);
}

static void CheckBootPin(uint16_t gpioPins)
{
	if (gpioPins == IOIF_GPIO_PIN_3) { //SW_IC_INT_Pin, Physically Pull-up
		// Reset SW_CLR
		powerOff = 1;
	}

	if (gpioPins == IOIF_GPIO_PIN_10) { // SBC_OFF_REQ_Pin, Physically Pull-Down
		IOIF_WriteGPIOPin(IOIF_GPIO_PORT_D, IOIF_GPIO_PIN_11, IOIF_GPIO_PIN_RESET); // SBC_PWR_ENB_GPIO_Port, SBC_PWR_ENB_Pin, GPIO_PIN_RESET
		IOIF_WriteGPIOPin(IOIF_GPIO_PORT_D, IOIF_GPIO_PIN_12, IOIF_GPIO_PIN_RESET); // PWR_5V_ENB_GPIO_Port,  PWR_5V_ENB_Pin,  GPIO_PIN_RESET
		IOIF_WriteGPIOPin(IOIF_GPIO_PORT_D, IOIF_GPIO_PIN_14, IOIF_GPIO_PIN_RESET); // PWR_3V3_ENB_GPIO_Port, PWR_3V3_ENB_Pin, GPIO_PIN_RESET
		IOIF_WriteGPIOPin(IOIF_GPIO_PORT_G, IOIF_GPIO_PIN_5,  IOIF_GPIO_PIN_RESET); // PWR_12V_ENB_GPIO_Port, PWR_12V_ENB_Pin, GPIO_PIN_RESET
		IOIF_WriteGPIOPin(IOIF_GPIO_PORT_D, IOIF_GPIO_PIN_13, IOIF_GPIO_PIN_RESET); // PWR_48V_ENB_GPIO_Port, PWR_48V_ENB_Pin, GPIO_PIN_RESET
		power_state = 0;
	}
}

// Boot Sequence
static void SBC_BootSequence()
{
    // 1. Enable 5V
	IOIF_WriteGPIOPin(IOIF_GPIO_PORT_D, IOIF_GPIO_PIN_12, IOIF_GPIO_PIN_SET); // PWR_5V_ENB_Pin, HIGH

    // 2-1. Wait for OFF_REQ to rise
//    while(IOIF_ReadGPIOPin(IOIF_GPIO_PORT_D, IOIF_GPIO_PIN_10) != IOIF_GPIO_PIN_SET) {
//        osDelay(10);
//    }

    // 2-2. Wait 500ms
    osDelay(500);

    // 3. Enable SBC Power
    IOIF_WriteGPIOPin(IOIF_GPIO_PORT_D, IOIF_GPIO_PIN_11, IOIF_GPIO_PIN_SET);

    // 4-1. Wait for SBC_RST to rise
//    while(IOIF_ReadGPIOPin(IOIF_GPIO_PORT_D, IOIF_GPIO_PIN_9) != IOIF_GPIO_PIN_SET) {
//        osDelay(10);
//    }

    // 4-2. Wait 500ms
    osDelay(100);

    // 5. Enable 3.3V
    IOIF_WriteGPIOPin(IOIF_GPIO_PORT_D, IOIF_GPIO_PIN_14, IOIF_GPIO_PIN_SET);

    sbcPowerState = 1;
}

static void D10_BootSequence()
{
	static uint32_t tCnt;
	static uint8_t tStep = 0;

	/* Motor Power ON */
	if (tStep == 0) {
		IOIF_WriteGPIOPin(IOIF_GPIO_PORT_D, IOIF_GPIO_PIN_13, IOIF_GPIO_PIN_SET); // PWR_48V_ENB_Pin, HIGH
		tStep = 1;
    	tCnt = sysCtrlLoopCnt;
    	return;
	}

	/* Delay 500ms */
	if (tStep == 1) {
	    if( (sysCtrlLoopCnt - tCnt) < 50) {
			return;
		} else {
			tStep = 2;
		}
	}

	/* Motor Driver Power ON */
	if (tStep == 2) {
		IOIF_WriteGPIOPin(IOIF_GPIO_PORT_G, IOIF_GPIO_PIN_5, IOIF_GPIO_PIN_SET); // PWR_12V_ENB_Pin, HIGH
		power_state = 2;
		tStep = 0;
	}
	d10_power_state = 1;

}

/* ------------------- GET LTC2944 VALUE ------------------- */
static int GetBatVal_Ent(void)
{
	memset(&batData, 0, sizeof(batData));

	return 0;
}

static int GetBatVal_Run(void)
{
	bmRes = IOIF_GetBatValue(&batData);

	return bmRes;
}

// TODO : BSP!!
static void InitBatteryMonitor(void)
{
	bmInitRes = IOIF_InitBat(IOIF_I2C4);
	if (bmInitRes != IOIF_BAT_STATUS_OK) { //i2c revocery
		// TODO : Error Handling
		__HAL_RCC_I2C4_CLK_DISABLE();
		__HAL_I2C_DISABLE(&hi2c4);
		HAL_I2C_DeInit(&hi2c4);
		HAL_I2C_MspDeInit(&hi2c4);

		HAL_I2C_BusReset(&hi2c4, GPIOF, IOIF_GPIO_PIN_15, GPIOF, IOIF_GPIO_PIN_14);

		HAL_I2C_Init(&hi2c4);
		HAL_I2C_MspInit(&hi2c4);
		__HAL_I2C_ENABLE(&hi2c4);
		__HAL_RCC_I2C4_CLK_ENABLE();

		IsRecoverI2C4 = true;

		bmInitRes = IOIF_InitBat(IOIF_I2C4);
	}

	if (bmInitRes == IOIF_BAT_STATUS_OK) {

	}
}

#endif /* WALKON5_CM_ENABLED */
