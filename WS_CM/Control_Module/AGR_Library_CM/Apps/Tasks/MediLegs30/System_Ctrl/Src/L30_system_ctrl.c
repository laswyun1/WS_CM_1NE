/**
 * @file system_ctrl_task.c
 * @date Created on: Jul 28, 2023
 * @author AngelRobotics FW Team
 * @brief
 * @ref
 */

#include "L30_system_ctrl.h"

#ifdef L30_CM_ENABLED

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

//uint8_t sbcPowerState;
static volatile uint8_t powerOff;
static IOIF_NzrLedObj_t nzrObj1;
static IOIF_NzrLedObj_t nzrObj2;

// for debug
static uint8_t testTimRes = IOIF_TIM_OK;
//static uint8_t testUPRes = IOIF_UPRIGHT_STATUS_OK;
//static uint8_t testLSRes = IOIF_LS_STATUS_OK;
//static uint8_t testFSRRes = IOIF_FSR_STATUS_OK;
//static uint8_t testLPRes = IOIF_LP_STATUS_OK;
static uint8_t testNzrRes = IOIF_NZRLED_STATUS_OK;

/**
 *------------------------------------------------------------
 *                      STATIC VARIABLES
 *------------------------------------------------------------
 * @brief Variables local to this module.
 */

// for debug
uint8_t bmInitRes = IOIF_BAT_STATUS_OK;
static uint8_t bmRes = IOIF_BAT_STATUS_OK;

static uint8_t sbcPowerState=0;

static uint16_t powerOff_cnt=0;
static uint8_t d10_power_state=0;
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
static void CheckBootPin(uint16_t gpioPins);
static void SBC_BootSequence(void);
static void D10_BootSequence(void);

static void Set_Battery_LED(float t_percentage);

/* ------------------- ROUTINES ------------------- */
/* ------------------- GET LTC2944 VALUE ------------------- */
static int GetBatVal_Ent(void);
static int GetBatVal_Run(void);

/* ------------------- STATUS LED (NZR LED) ------------------- */
void Init_LEDs(void);
static int Run_Status_LED(void);

/* ------------------- BATTERY STATUS(I2C) ------------------- */
static int Run_Battery_LED(void);

/* ------------------- BUZZER ------------------- */
static int Run_Buzzer(void);

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

	/* State Definition */
	TASK_CREATE_STATE(&sysCtrlTask, TASK_STATE_OFF,      NULL,				StateOff_Run,       NULL,       		false);
	TASK_CREATE_STATE(&sysCtrlTask, TASK_STATE_STANDBY,  NULL,				StateStandby_Run,	NULL,       		true);
	TASK_CREATE_STATE(&sysCtrlTask, TASK_STATE_ENABLE,   StateEnable_Ent,	StateEnable_Run, 	StateEnable_Ext,  	false);
	TASK_CREATE_STATE(&sysCtrlTask, TASK_STATE_ERROR,    NULL,				StateError_Run,    	NULL,				false);

	/* Routine Definition */
	//    TASK_CREATE_ROUTINE(&sysCtrlTask, ROUTINE_ID_SYSMNGT_RUN_STATUS_LED, 	NULL, Run_Status_LED, 	NULL);
	TASK_CREATE_ROUTINE(&sysCtrlTask, ROUTINE_ID_SYSMNGT_RUN_BATTERY_LED, 	NULL, Run_Battery_LED, 	NULL);
	TASK_CREATE_ROUTINE(&sysCtrlTask, ROUTINE_ID_SYSMNGT_RUN_BUZZER, 		NULL, Run_Buzzer,		NULL);

	//	Push_Routine(&sysCtrlTask.routine, ROUTINE_ID_SYSMNGT_RUN_STATUS_LED);
	PushRoutine(&sysCtrlTask.routine, ROUTINE_ID_SYSMNGT_RUN_BATTERY_LED);
	PushRoutine(&sysCtrlTask.routine, ROUTINE_ID_SYSMNGT_RUN_BUZZER);

	/* DOD Definition */
	// DOD
	DOP_CreateDOD(TASK_IDX_SYSTEM_CTRL);

	// PDO
	DOP_COMMON_PDO_CREATE(TASK_IDX_SYSTEM_CTRL, sysCtrlTask);
	DOP_CreatePDO(TASK_IDX_SYSTEM_CTRL, PDO_ID_SYSTEM_VOLT, DOP_FLOAT32, 1, &batData.batVol);
	DOP_CreatePDO(TASK_IDX_SYSTEM_CTRL, PDO_ID_SYSTEM_CURR, DOP_FLOAT32, 1, &batData.batCurr);
	DOP_CreatePDO(TASK_IDX_SYSTEM_CTRL, PDO_ID_SYSTEM_TEMP, DOP_FLOAT32, 1, &batData.brdTemp);
	//    DOP_CreatePDO(TASK_IDX_SYSTEM_CTRL, PDO_ID_SYSTEM_PCTG, DOP_FLOAT32, 1, &batData.percentage);

	// SDO
	DOP_COMMON_SDO_CREATE(TASK_IDX_SYSTEM_CTRL)
	// DOP_CreateSDO(TASK_IDX_SYSTEM_CTRL, object_id, DOP_FLOAT32, SDOCallback);

	// Init LED
	/*	Init_LEDs();
	IOIF_InitBatteryLED(IOIF_I2C1);

	IOIF_InitNZRLED(IOIF_TIM3, IOIF_TIM_CHANNEL_1, &nzrLedObj[0], SYSTEM_MNGT_PERIOD);
	IOIF_InitNZRLED(IOIF_TIM3, IOIF_TIM_CHANNEL_2, &nzrLedObj[1], SYSTEM_MNGT_PERIOD);
	 */

	IOIF_InitNZRLED(IOIF_TIM3, IOIF_TIM_CHANNEL_1, &nzrObj1, SYSTEM_MNGT_PERIOD);
	//IOIF_InitNZRLED(IOIF_TIM3, IOIF_TIM_CHANNEL_2, &nzrObj2, SYSTEM_MNGT_PERIOD);
    IOIF_ClearNZRLED(IOIF_TIM_CHANNEL_1, &nzrObj1);


//	IOIF_SetNZRLED(&nzrObj1, IOIF_NZR_LED1, IOIF_COLOR_WHITE, 0);

//	IOIF_SetNZRLED(&nzrObj1, IOIF_NZR_LED2, IOIF_COLOR_WHITE, 0);

	//IOIF_S/etNZRLED(&nzrObj2, IOIF_NZR_LED1, IOIF_COLOR_WHITE, 0);

	//IOIF_SetNZRLED(&nzrObj2, IOIF_NZR_LED2, IOIF_COLOR_WHITE, 0);

	// Init Buzzer
	IOIF_InitBuzzer(IOIF_TIM3, IOIF_TIM_CHANNEL_1, &buzzerObj, SYSTEM_MNGT_PERIOD);

	IOIF_SetGPIOCB(IOIF_GPIO_PIN_3, IOIF_GPIO_EXTI_CALLBACK, CheckBootPin);
	IOIF_SetGPIOCB(IOIF_GPIO_PIN_10, IOIF_GPIO_EXTI_CALLBACK, CheckBootPin);

	InitBatteryMonitor();
}

void RunSysCtrl(void)
{
	if (powerOff==1) {
		if (powerOff_cnt == 0) {
			IOIF_ConfigBuzzer(&buzzerObj, 500, buzzer_test, 4);
		} else if (powerOff_cnt == 100) {
			test_d10 = 7;
		} else if(powerOff_cnt == 300) {
			IOIF_WriteGPIOPin(IOIF_GPIO_PORT_G, IOIF_GPIO_PIN_7, IOIF_GPIO_PIN_RESET); // SW_IC_CLR_GPIO_Port, SW_IC_CLR_Pin, GPIO_PIN_RESET
		}
		powerOff_cnt++;
	}

	RunTask(&sysCtrlTask);
}

/* ------------------- STATUS LED (NZR LED) ------------------- */
void Init_LEDs()
{
	IOIF_InitBatteryLED(IOIF_I2C1);

	IOIF_InitNZRLED(IOIF_TIM3, IOIF_TIM_CHANNEL_1, &nzrLedObj[0], SYSTEM_MNGT_PERIOD);
	IOIF_InitNZRLED(IOIF_TIM3, IOIF_TIM_CHANNEL_2, &nzrLedObj[1], SYSTEM_MNGT_PERIOD);
}

static int Run_Status_LED()
{
	static uint32_t t_cnt, t_loop_cnt = 0;

	t_cnt = sysCtrlLoopCnt % 250;

	if(t_cnt == 0){
		switch(t_loop_cnt){
		case 0:
			IOIF_SetNZRLED(&nzrLedObj[0], 0, 0x0F000000, 2500);
			IOIF_SetNZRLED(&nzrLedObj[0], 1, 0x0F000000, 2500);
			IOIF_SetNZRLED(&nzrLedObj[1], 0, 0x0F000000, 2500);
			IOIF_SetNZRLED(&nzrLedObj[1], 1, 0x0F000000, 2500);
			t_loop_cnt = 1;	break;
		case 1:
			IOIF_SetNZRLED(&nzrLedObj[0], 0, 0x0F0F0000, 2500);
			IOIF_SetNZRLED(&nzrLedObj[0], 1, 0x0F0F0000, 2500);
			IOIF_SetNZRLED(&nzrLedObj[1], 0, 0x0F0F0000, 2500);
			IOIF_SetNZRLED(&nzrLedObj[1], 1, 0x0F0F0000, 2500);
			t_loop_cnt = 2;	break;
		case 2:
			IOIF_SetNZRLED(&nzrLedObj[0], 0, 0x000F0000, 2500);
			IOIF_SetNZRLED(&nzrLedObj[0], 1, 0x000F0000, 2500);
			IOIF_SetNZRLED(&nzrLedObj[1], 0, 0x000F0000, 2500);
			IOIF_SetNZRLED(&nzrLedObj[1], 1, 0x000F0000, 2500);
			t_loop_cnt = 3;	break;
		case 3:
			IOIF_SetNZRLED(&nzrLedObj[0], 0, 0x00000F00, 2500);
			IOIF_SetNZRLED(&nzrLedObj[0], 1, 0x00000F00, 2500);
			IOIF_SetNZRLED(&nzrLedObj[1], 0, 0x00000F00, 2500);
			IOIF_SetNZRLED(&nzrLedObj[1], 1, 0x00000F00, 2500);
			t_loop_cnt = 4;	break;
		case 4:
			IOIF_SetNZRLED(&nzrLedObj[0], 0, 0x00000F0F, 2500);
			IOIF_SetNZRLED(&nzrLedObj[0], 1, 0x00000F0F, 2500);
			IOIF_SetNZRLED(&nzrLedObj[1], 0, 0x00000F0F, 2500);
			IOIF_SetNZRLED(&nzrLedObj[1], 1, 0x00000F0F, 2500);
			t_loop_cnt = 5;	break;
		case 5:
			IOIF_SetNZRLED(&nzrLedObj[0], 0, 0x0000000F, 2500);
			IOIF_SetNZRLED(&nzrLedObj[0], 1, 0x0000000F, 2500);
			IOIF_SetNZRLED(&nzrLedObj[1], 0, 0x0000000F, 2500);
			IOIF_SetNZRLED(&nzrLedObj[1], 1, 0x0000000F, 2500);
			t_loop_cnt = 6;	break;
		case 6:
			IOIF_SetNZRLED(&nzrLedObj[0], 0, 0x0F00000F, 2500);
			IOIF_SetNZRLED(&nzrLedObj[0], 1, 0x0F00000F, 2500);
			IOIF_SetNZRLED(&nzrLedObj[1], 0, 0x0F00000F, 2500);
			IOIF_SetNZRLED(&nzrLedObj[1], 1, 0x0F00000F, 2500);
			t_loop_cnt = 7;	break;
		case 7:
			IOIF_SetNZRLED(&nzrLedObj[0], 0, 0x000F000F, 2500);
			IOIF_SetNZRLED(&nzrLedObj[0], 1, 0x000F000F, 2500);
			IOIF_SetNZRLED(&nzrLedObj[1], 0, 0x000F000F, 2500);
			IOIF_SetNZRLED(&nzrLedObj[1], 1, 0x000F000F, 2500);
			t_loop_cnt = 0;	break;
		default: break;
		}
	}
	IOIF_RunNZRLED(IOIF_TIM_CHANNEL_1, &nzrLedObj[0]);
	IOIF_RunNZRLED(IOIF_TIM_CHANNEL_2, &nzrLedObj[1]);
	return 0;
}

static int Run_Battery_LED()
{
	//	Set_Battery_LED(batData.percentage);
	return 0;
}

/* ------------------- BUZZER ------------------- */
static int Run_Buzzer()
{
	IOIF_ExecuteBuzzer(IOIF_TIM3, &buzzerObj);
	return 0;
}


/**
 *------------------------------------------------------------
 *                      STATIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions intended for internal use within this module.
 */

static void StateOff_Run(void)
{
	TaskDelay(&sysCtrlTask);
}

static void StateStandby_Run(void)
{
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

int test_cmd=0;
static void StateEnable_Run(void)
{
	if (sbcPowerState == 0 && IOIF_ReadGPIOPin(IOIF_GPIO_PORT_D, IOIF_GPIO_PIN_10) == IOIF_GPIO_PIN_SET) { // SBC_OFF_REQ_Pin
		SBC_BootSequence();
	}

	if (d10_power_state == 0) {
		D10_BootSequence();
	}

	// Battery Monitor
	GetBatVal_Run();

	//	Run_Routines(&sysCtrlTask.routine);
	 // IOIF_SetNZRLED(&nzrObj1, IOIF_NZR_LED1, IOIF_COLOR_WHITE, 0);
	//	IOIF_SetNZRLED(&nzrObj1, IOIF_NZR_LED2, IOIF_COLOR_WHITE, 0);
		//IOIF_SetNZRLED(&nzrObj2, IOIF_NZR_LED1, IOIF_COLOR_WHITE, 0);
		//IOIF_SetNZRLED(&nzrObj2, IOIF_NZR_LED2, IOIF_COLOR_WHITE, 0);

	IOIF_RunNZRLED(IOIF_TIM_CHANNEL_1, &nzrObj1);
	IOIF_RunNZRLED(IOIF_TIM_CHANNEL_2, &nzrObj1);

	if(test_cmd == 1) {
		IOIF_SetNZRLED(&nzrObj1, IOIF_NZR_LED1, IOIF_COLOR_RED, 0);
		IOIF_SetNZRLED(&nzrObj1, IOIF_NZR_LED2, IOIF_COLOR_RED, 0);
	} else if (test_cmd == 2) {
		IOIF_SetNZRLED(&nzrObj1, IOIF_NZR_LED1, IOIF_COLOR_GREEN, 0);
		IOIF_SetNZRLED(&nzrObj1, IOIF_NZR_LED2, IOIF_COLOR_GREEN, 0);

	} else if (test_cmd == 3) {
		IOIF_SetNZRLED(&nzrObj1, IOIF_NZR_LED1, IOIF_COLOR_BLUE, 0);
		IOIF_SetNZRLED(&nzrObj1, IOIF_NZR_LED2, IOIF_COLOR_BLUE, 0);

	} else if (test_cmd == 4) {
		IOIF_SetNZRLED(&nzrObj1, IOIF_NZR_LED1, IOIF_COLOR_WHITE, 0);
		IOIF_SetNZRLED(&nzrObj1, IOIF_NZR_LED2, IOIF_COLOR_WHITE, 0);

	}/* else if (test_cmd == 5) {
		IOIF_SetNZRLED(&nzrObj, IOIF_NZR_LED1, IOIF_COLOR_WHITE, 0);
		IOIF_SetNZRLED(&nzrObj, IOIF_NZR_LED2, IOIF_COLOR_WHITE, 0);
		IOIF_SetNZRLED(&nzrObj, IOIF_NZR_LED3, IOIF_COLOR_WHITE, 0);
		IOIF_SetNZRLED(&nzrObj, IOIF_NZR_LED4, IOIF_COLOR_WHITE, 0);
	}
*/
	sysCtrlLoopCnt++;
	TaskDelay(&sysCtrlTask);
}

static void StateEnable_Ext(void)
{
	ExtRoutines(&sysCtrlTask.routine);
}

static void StateError_Run(void)
{
	// TODO : Error Handle
	TaskDelay(&sysCtrlTask);
}

/* ------------------- BOOT SEQUENCE ------------------- */
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
		sbcPowerState = 0;
	}
}

static void SBC_BootSequence()
{
	static uint32_t tCnt;
	static uint8_t tStep = 0;

	// 1. Enable 5V
	if (tStep == 0) {
		IOIF_WriteGPIOPin(IOIF_GPIO_PORT_D, IOIF_GPIO_PIN_12, IOIF_GPIO_PIN_SET); // PWR_5V_ENB_Pin, HIGH
		tStep = 1;
	}

	// 2-1. Wait for OFF_REQ to rise
	if(tStep == 1){
		if (IOIF_ReadGPIOPin(IOIF_GPIO_PORT_D, IOIF_GPIO_PIN_10) != IOIF_GPIO_PIN_SET) { // SBC_OFF_REQ_Pin
			return;
		} else {
			tCnt = sysCtrlLoopCnt;
			tStep = 2;
		}
	}

	// 2-2. Wait 500ms
	if (tStep == 2) {
		if ((sysCtrlLoopCnt - tCnt) < 50) {
			return;
		} else {
			tStep = 3;
		}
	}

	// 3. Enable SBC Power
	if (tStep == 3) {
		IOIF_WriteGPIOPin(IOIF_GPIO_PORT_D, IOIF_GPIO_PIN_11, IOIF_GPIO_PIN_SET); // SBC_PWR_ENB_Pin, HIGH
		tStep = 4;
	}

	// 4-1. Wait for SBC_RST to rise
	if (tStep == 4) {
		if (IOIF_ReadGPIOPin(IOIF_GPIO_PORT_D, IOIF_GPIO_PIN_9) != IOIF_GPIO_PIN_SET) {	// SBC_RST_Pin
			return;
		} else {
			tCnt = sysCtrlLoopCnt;
			tStep = 5;
		}
	}

	// 4-2. Wait 200ms
	if (tStep == 5) {
		if ((sysCtrlLoopCnt - tCnt) < 20) {
			return;
		} else {
			tStep = 6;
		}
	}

	// 5. Enable 3.3V
	if (tStep == 6) {
		IOIF_WriteGPIOPin(IOIF_GPIO_PORT_D, IOIF_GPIO_PIN_14, IOIF_GPIO_PIN_SET); // PWR_3V3_ENB_Pin, HIGH
		sbcPowerState = 1;
		tStep = 0;
	}
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
		d10_power_state = 1;
		tStep = 0;
	}
}

/* ------------------- BATTERY STATUS(I2C) ------------------- */
static void Set_Battery_LED(float t_percentage)
{
	//	if(t_percentage < 25)		{On_Battery_LED(0b00000001);}
	//	else if(t_percentage < 50)	{On_Battery_LED(0b00000011);}
	//	else if(t_percentage < 75)	{On_Battery_LED(0b00000111);}
	//	else if(t_percentage < 100)	{On_Battery_LED(0b00001111);}
	IOIF_RunBatteryLED(0b00001111);
}

/* ------------------- ROUTINES ------------------- */
/* ------------------- GET LTC2944 VALUE ------------------- */
static int GetBatVal_Ent(void)
{
	memset(&batData, 0, sizeof(batData));

	return 0;
}

static int GetBatVal_Run(void)
{
	bmRes = IOIF_GetBatValue(&batData);

	float per = (batData.batVol - 42)/(48 - 42);

	per = (per > 1.0) ? 1.0 : per;
	per = (per < 0.0) ? 0.0 : per;
	per *= 100;

	batData.percentage = per;


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

		// GPIOF, MCU_BAT_MON_I2C4_SDA_Pin(GPIO_PIN_15), MCU_BAT_MON_I2C4_SCL_Pin(GPIO_PIN_14)
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

#endif
