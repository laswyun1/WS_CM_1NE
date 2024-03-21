/**
 * @file system_ctrl_task.c
 * @date Created on: Jul 28, 2023
 * @author AngelRobotics FW Team
 * @brief
 * @ref
 */

#include "AS_system_ctrl.h"

#ifdef SUIT_MINICM_ENABLED

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
float voltSamples[BAT_VOLT_SAMPLE_SIZE];
float medianVolt;
float batPctg;
uint32_t sysCtrlLoopCnt;

GaitModeByBt_t gaitModeState = GAIT_MODE_WALK;

extern USBH_HandleTypeDef hUsbHostFS;

uint8_t systemStateFlag = SYSTEM_IDLE;
uint8_t wholeTaskFlag = 0;

AssistStage assistStage = STAGE_0;
uint8_t assistFlag = 0;
float assistForcePctg = 0.0;

uint8_t led_driver_init_flag = 0;

uint8_t led_error_flag[2];
bool Is_led_output_error = false;
bool Is_led_overtemp_error =false;

uint8_t ledg0_error_flag[2];
uint8_t ledg1_error_flag[2];
uint8_t ledg2_error_flag[2];
uint8_t ledg3_error_flag[2];
uint8_t ledg4_error_flag[2];


/**
 *------------------------------------------------------------
 *                      STATIC VARIABLES
 *------------------------------------------------------------
 * @brief Variables local to this module.
 */

// for debug
uint8_t bmInitRes = IOIF_BAT_STATUS_OK;
static uint8_t bmRes = IOIF_BAT_STATUS_OK;

static int voltSampleIndex = 0;
static uint32_t lastSamplingTime = 0; // Last Sampling Time.

static uint32_t lastAssistBtPressTime = 0;
static uint8_t lastAssistIncBtState = IOIF_GPIO_PIN_SET;
static uint8_t lastAssistDecBtState = IOIF_GPIO_PIN_SET;

static uint32_t lastPwrBtPressTime = 0;
static uint8_t lastPwrBtState = IOIF_GPIO_PIN_SET;

static uint8_t IsRecoverI2C4 = false;


/**
 *------------------------------------------------------------
 *                 STATIC FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Static Function prototypes for this module.
 */

/* -------------------- STATE FUNCTION -------------------- */
static void StateOff_Run(void);

static void StateStandby_Run(void);

static void StateEnable_Ent(void);
static void StateEnable_Run(void);
static void StateEnable_Ext(void);

static void StateError_Run(void);

/* ----------------------- FUNCTION ----------------------- */
static void InitBatteryMonitor(void);

static void CalForcePctg(void);

static void StartBoot(void);

static void GetBatVal_Run(void);
static int QsortCompare_Float(const void* firstFloatPtr, const void* secondFloatPtr);
static float CalMedian(float* sampleData, int numElements);

static float GetBatState(void);

static void UpdatePwrLED(void);

static void UpdateBatLED(void);

static void handleAssistStage(AssistStage stage);
static void UpdateAuxLED(void);

static void BlinkBLELED(void);

/* ----------------------- ROUTINE ------------------------ */

/* --------------------- SDO CALLBACK --------------------- */

/**
 *------------------------------------------------------------
 *                      PUBLIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions that interface with this module.
 */
/* --------------------- SDO CALLBACK --------------------- */
DOP_COMMON_SDO_CB(sysCtrlTask)

/* ------------------------- MAIN ------------------------- */
void InitSysCtrl(void)
{
	/* Init Task */
    InitTask(&sysCtrlTask);
	DOPC_AssignTaskID(&sysCtrlTask, TASK_IDX_SYSTEM_CTRL);

	/* Init Device */
	IOIF_SysCtrlInit();

	/* State Definition */
	TASK_CREATE_STATE(&sysCtrlTask, TASK_STATE_OFF,      NULL,				StateOff_Run,       NULL,				true);
	TASK_CREATE_STATE(&sysCtrlTask, TASK_STATE_STANDBY,  NULL,				StateStandby_Run,	NULL,       		false);
	TASK_CREATE_STATE(&sysCtrlTask, TASK_STATE_ENABLE,   StateEnable_Ent,	StateEnable_Run, 	StateEnable_Ext,  	false);
	TASK_CREATE_STATE(&sysCtrlTask, TASK_STATE_ERROR,    NULL,				StateError_Run,    	NULL,				false);

	/* Routine Definition */
	// TASK_CREATE_ROUTINE(&sysCtrlTask, ROUTINE_ID_SYSMNGT_GET_POWER_VALUE, 	NULL, 	GetBatVal_Run, 	NULL);

	// Push_Routine(&sysCtrlTask.routine, ROUTINE_ID_SYSMNGT_GET_POWER_VALUE);

	/* DOD Definition */
	// DOD
	DOP_CreateDOD(TASK_IDX_SYSTEM_CTRL);

	IOIF_InitUSB(IOIF_USBD_CDC, IOIF_USB_TIM_NULL);

   	// PDO
	DOP_COMMON_PDO_CREATE(TASK_IDX_SYSTEM_CTRL, sysCtrlTask);
	// DOP_CreatePDO(TASK_IDX_SYSTEM_CTRL, object_id, DOP_FLOAT32, length, pDataAddr);

	// SDO
	DOP_COMMON_SDO_CREATE(TASK_IDX_SYSTEM_CTRL)
	// DOP_CreateSDO(TASK_IDX_SYSTEM_CTRL, object_id, DOP_FLOAT32, SDOCallback);

	led_driver_init_flag = IOIF_LED24chInit(IOIF_SPI1);

	InitBatteryMonitor();

	/* USB File-system Test */
//		if(Appli_state == APPLICATION_READY)
//		{
//			mount_usb_res = IOIF_FileMount(&fatfs_usb_test, (uint8_t*)"1:");
//			disk_free_usb = IOIF_Disk_GetFreeSpace((uint8_t*)"1:", &fatfs_usb_test);
//			write_usb_res = IOIF_FileWrite(&file_usb_t, (uint8_t*)"1:A10_ioif_test2.txt", SD_writeData, sizeof(SD_writeData), &w_byte);
//		}
}

/* ----------------------- FUNCTION ----------------------- */
void RunSysCtrl(void)
{
	RunTask(&sysCtrlTask);
}

void UpdateLED(void)
{
	UpdatePwrLED();
	UpdateBatLED();
	BlinkBLELED();
	UpdateAuxLED();
}

void PwrOffBt(void)
{
	uint8_t pwrBt = IOIF_ReadGPIOPin(IOIF_GPIO_PORT_G, IOIF_GPIO_PIN_6); // PB_SW_INT#
	uint32_t currentTime = IOIF_GetTick();

	if (pwrBt == IOIF_GPIO_PIN_RESET && lastPwrBtState == 1 && (currentTime - lastPwrBtPressTime > DEBOUNCE_TIME)) {
		// Next State Transition (STOP -> WALKING -> RUN -> CLIMB -> STOP)
		// gaitModeState = (gaitModeState + 1) % (GAIT_MODE_CLIMB + 1);

		// State Transition (STOP <-> WALKING)
		gaitModeState = (gaitModeState == GAIT_MODE_STOP) ? GAIT_MODE_WALK : GAIT_MODE_STOP;
		ISIFlags_Bt.I22_Mode = gaitModeState;

		if (gaitModeState == GAIT_MODE_STOP) {
			systemStateFlag = SYSTEM_IDLE;
		} else {
			systemStateFlag = SYSTEM_ASSIST;
		}

		lastPwrBtPressTime = currentTime;
	}

	// 버튼의 이전 상태를 업데이트
	lastPwrBtState = pwrBt;
}


/**
 *------------------------------------------------------------
 *                      STATIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions intended for internal use within this module.
 */

/* -------------------- STATE FUNCTION -------------------- */
static void StateOff_Run(void)
{
	StartBoot();
	sysCtrlLoopCnt++;
}

static void StateStandby_Run(void)
{
	bmRes = IOIF_GetBatValue(&batData);
	medianVolt = batData.batVol;
	if (bmRes == IOIF_BAT_STATUS_OK) {
		wholeTaskFlag = 1;
		StateTransition(&sysCtrlTask.stateMachine, TASK_STATE_ENABLE);
	} else {
		systemStateFlag = SYSTEM_ERROR;
	}
}

static void StateEnable_Ent(void)
{
	sysCtrlLoopCnt = 0;
	EntRoutines(&sysCtrlTask.routine);
}

static void StateEnable_Run(void)
{
	PwrOffBt();
	CalForcePctg();
	GetBatVal_Run();
	UpdateLED();

	//LED TEST
	// uint8_t color = RED;
	// IOIF_LED24chBattery(1, color, 0x80);
	// IOIF_LED24chBattery(2, color, 0x80);
	// IOIF_LED24chBattery(3, color, 0x80);
	// IOIF_LED24chError(RED, 0x50);
	// IOIF_LED24chAssist(10, 0x20);
	// IOIF_LED24chBluetooth(0x50);
	// IOIF_LED24chMode(BLUE, 0x20);

	// /* LED Error Detect */
	IOIF_LED24chRead(MODE2, 0xff, led_error_flag);
	//if error detected,
	if (led_error_flag[1] == 0b01001001) {
		Is_led_output_error = true;	//breakpoint
		IOIF_LED24chRead(EFLAG0, 0xff, ledg0_error_flag);
		IOIF_LED24chRead(EFLAG1, 0xff, ledg1_error_flag);
		IOIF_LED24chRead(EFLAG2, 0xff, ledg2_error_flag);
		IOIF_LED24chRead(EFLAG3, 0xff, ledg3_error_flag);
		IOIF_LED24chRead(EFLAG4, 0xff, ledg4_error_flag);
	} else if (led_error_flag[1] == 0b10001001) {
		Is_led_overtemp_error = true; //breakpoint
	}

//	RunRoutines(&sysCtrlTask.routine);

	if (IOIF_IsPwrBtnPressed() == true) {
		IOIF_SysPwrOff();
	}

	sysCtrlLoopCnt++;
}

static void StateEnable_Ext(void)
{
	ExtRoutines(&sysCtrlTask.routine);
}

static void StateError_Run(void)
{
	// TODO : Error Handle
}
/* --------------------- SDO CALLBACK --------------------- */

/* ----------------------- FUNCTION ----------------------- */

static void CalForcePctg(void) 
{
	switch (assistStage) {
			case(STAGE_0) :
				assistForcePctg = 0.0;
				break;
			case(STAGE_1) :
				assistForcePctg = 0.1;
				break;
			case(STAGE_2) :
				assistForcePctg = 0.2;
				break;
			case(STAGE_3) :
				assistForcePctg = 0.3;
				break;
			case(STAGE_4) :
				assistForcePctg = 0.4;
				break;
			case(STAGE_5) :
				assistForcePctg = 0.5;
				break;
			case(STAGE_6) :
				assistForcePctg = 0.6;
				break;
			case(STAGE_7) :
				assistForcePctg = 0.7;
				break;
			case(STAGE_8) :
				assistForcePctg = 0.8;
				break;
			case(STAGE_9) :
				assistForcePctg = 0.9;
			case(STAGE_10) :
				assistForcePctg = 1;
				break;
			default :
				break;
		}
}

/* ------------------- 11.02 SBS FW ------------------- */
static void StartBoot(void)
{
	// Pwr ON
	if (sysCtrlLoopCnt == 0) {
		IOIF_WriteGPIOPin(IOIF_GPIO_PORT_G, IOIF_GPIO_PIN_2, IOIF_GPIO_PIN_SET); // Port : MCU_24V_MOTOR_ON_GPIO_Port, Pin : MCU_24V_MOTOR_ON_Pin
	} else if (sysCtrlLoopCnt == 300) {
		IOIF_WriteGPIOPin(IOIF_GPIO_PORT_G, IOIF_GPIO_PIN_3, IOIF_GPIO_PIN_SET); // Port : MC_5V_PWR_EN_GPIO_Port, Pin : MC_5V_PWR_EN_Pin
		IOIF_WriteGPIOPin(IOIF_GPIO_PORT_G, IOIF_GPIO_PIN_4, IOIF_GPIO_PIN_SET); // Port : WIDM_5V_PWR_EN_GPIO_Port Pin : WIDM_5V_PWR_EN_Pin
		StateTransition(&sysCtrlTask.stateMachine, TASK_STATE_STANDBY);
	}
}

static void GetBatVal_Run(void)
{
	// Perform sampling only if the difference between the current time and the last time is greater than or equal to the sampling interval.
	if (sysCtrlLoopCnt - lastSamplingTime >= BAT_VOLT_SAMPLING_INTERVAL) { // Store sample data at intervals of every 60ms.
		bmRes = IOIF_GetBatValue(&batData); // Read battery data

		// Store the battery voltage sample in the array.
		if (voltSampleIndex < BAT_VOLT_SAMPLE_SIZE) {
			voltSamples[voltSampleIndex++] = batData.batVol;
		}

 		// Once the sample array is full, calculate the median voltage.
		if (voltSampleIndex == BAT_VOLT_SAMPLE_SIZE) {
			medianVolt = CalMedian(voltSamples, BAT_VOLT_SAMPLE_SIZE);
			voltSampleIndex = 0; // Reset sample index for the next round of sampling.
		}

		// Update to the last sampling time
		lastSamplingTime = sysCtrlLoopCnt;
	}
}

/**
 * @brief Compare two floating-point numbers for qsort.
 * 
 * This function is utilized by the qsort function to compare two elements. It takes
 * two const void pointers to floating-point numbers, calculates the difference 
 * between them, and returns an integer representing their relative ordering.
 * 
 * @param firstFloatPtr Pointer to the first floating-point number.
 * @param secondFloatPtr Pointer to the second floating-point number.
 * @return int Returns positive if the first number is greater than the second, 
 * negative if the first is less than the second, and 0 if they are equal.
 */
static int QsortCompare_Float(const void* firstFloatPtr, const void* secondFloatPtr)
{
	// Convert void pointers to float pointers, then dereference to get the float values and calculate the difference
	float diff = *(float*)firstFloatPtr - *(float*)secondFloatPtr;

	// If diff is positive, return 1; if diff is negative, return -1; if equal, return 0.
	// This determines the order for the qsort function to sort the array in ascending order.
	return (diff > 0) - (diff < 0);
}

/**
 * @brief Calculate the median of a sorted array of floating-point numbers.
 * 
 * This function calculates the median value from a given array of floating-point 
 * numbers (data) by first sorting the array using qsort and then finding the 
 * middle value or the average of the two middle values depending on the total 
 * number of elements in the array.
 * 
 * @param sampleData Array of floating-point numbers to find the median of.
 * @param numElements The number of elements in the array.
 * @return float The median value of the array.
 */
static float CalMedian(float* sampleData, int numSampleData)
{
	// Use qsort to sort the samples array in ascending order.
	// The QsortCompare function is passsed as the comparator.
	qsort(sampleData, numSampleData, sizeof(float), QsortCompare_Float);

	// If the number of elements(n) is even, the median is the average of the two median numbers.
	if (numSampleData % 2 == 0) {
		return (sampleData[numSampleData/2 - 1] + sampleData[numSampleData/2]) / 2.0;
	} else { // If n is odd, the median is the middle number directly.
		return sampleData[numSampleData/2];
	}
}

static float GetBatState(void)
{
	float per = (medianVolt - BATTERY_VOLT_MIN)/(BATTERY_VOLT_MAX - BATTERY_VOLT_MIN);
    per = (per > 1.0) ? 1.0 : per;
    per = (per < 0.0) ? 0.0 : per;
    per *= 100;

	return per;
}

static void UpdatePwrLED(void)
{
	static uint8_t lastPwrLEDState = 0; // 0 means not set
	if (systemStateFlag == SYSTEM_IDLE && lastPwrLEDState != 1) {
		IOIF_LED24chMode(WHITE, 0x00);
		IOIF_LED24chMode(BLUE, 0x00);
		lastPwrLEDState = 1; // Update the last state to current
	} else if (systemStateFlag == SYSTEM_ASSIST && lastPwrLEDState != 2) {
		IOIF_LED24chMode(WHITE, 0x20);
		lastPwrLEDState = 2; // Update the last state to current
	} else if (systemStateFlag == SYSTEM_FITNESS && lastPwrLEDState != 3) {
		IOIF_LED24chMode(BLUE, 0x20);
		lastPwrLEDState = 3; // Update the last state to current
	} else {

	}
}

static void UpdateBatLED(void)
{
    static uint8_t lastBatLEDState = 0; // 0 means not set
    batPctg = GetBatState();

    // Check if the current state is different from the last state to update LEDs only when the state changes
    if ((batPctg >= 0 && batPctg <= 5) && lastBatLEDState != 1) {
        IOIF_LED24chBattery(1, RED, 0x80);
        lastBatLEDState = 1; // Update the last state to current
    } else if ((batPctg >= 6 && batPctg <= 15) && lastBatLEDState != 2) {
        IOIF_LED24chBattery(1, RED, 0x80);
        lastBatLEDState = 2; // Update the last state to current
    } else if ((batPctg >= 16 && batPctg <= 30) && lastBatLEDState != 3) {
        IOIF_LED24chBattery(1, GREEN, 0x80);
        lastBatLEDState = 3; // Update the last state to current
    } else if ((batPctg >= 31 && batPctg <= 60) && lastBatLEDState != 4) {
        IOIF_LED24chBattery(2, GREEN, 0x80);
        lastBatLEDState = 4; // Update the last state to current
    } else if ((batPctg >= 61 && batPctg <= 100) && lastBatLEDState != 5) {
        IOIF_LED24chBattery(3, GREEN, 0x80);
        lastBatLEDState = 5; // Update the last state to current
    }
}

static void BlinkBLELED(void)
{
	static uint8_t lastBLELEDState = 0; // 0 means not set
	if (isBLEConnect == BLE_CONNECTED && lastBLELEDState != 1) {
		IOIF_LED24chBluetooth(0x50);
		lastBLELEDState = 1;
	} else if (isBLEConnect == BLE_PARING && lastBLELEDState != 2) {
		IOIF_LED24chBluetooth(0x50);
		lastBLELEDState = 2;
	} else if (isBLEConnect == BLE_NOT_CONNECTED && lastBLELEDState != 3) { // BLE 미연결 시
		IOIF_LED24chBluetooth(0x00);
		lastBLELEDState = 3;
	} else {

	}
}

static void handleAssistStage(AssistStage stage)
{
	static uint8_t lastAssistLEDState = -1; // -1 means not set

	if (stage == STAGE_0 && lastAssistLEDState != 0) {
		IOIF_LED24chAssist(10, 0x00);
		lastAssistLEDState = 0;
	} else if (stage == STAGE_1 && lastAssistLEDState != 1) {
		IOIF_LED24chAssist(1, 0x20);
		lastAssistLEDState = 1;
	} else if (stage == STAGE_2 && lastAssistLEDState != 2) {
		IOIF_LED24chAssist(2, 0x20);
		lastAssistLEDState = 2;
	} else if (stage == STAGE_3 && lastAssistLEDState != 3) {
		IOIF_LED24chAssist(3, 0x20);
		lastAssistLEDState = 3;
	} else if (stage == STAGE_4 && lastAssistLEDState != 4) {
		IOIF_LED24chAssist(4, 0x20);
		lastAssistLEDState = 4;
	} else if (stage == STAGE_5 && lastAssistLEDState != 5) {
		IOIF_LED24chAssist(5, 0x20);
		lastAssistLEDState = 5;
	} else if (stage == STAGE_6 && lastAssistLEDState != 6) {
		IOIF_LED24chAssist(6, 0x20);
		lastAssistLEDState = 6;
	} else if (stage == STAGE_7 && lastAssistLEDState != 7) {
		IOIF_LED24chAssist(7, 0x20);
		lastAssistLEDState = 7;
	} else if (stage == STAGE_8 && lastAssistLEDState != 8) {
		IOIF_LED24chAssist(8, 0x20);
		lastAssistLEDState = 8;
	} else if (stage == STAGE_9 && lastAssistLEDState != 9) {
		IOIF_LED24chAssist(9, 0x20);
		lastAssistLEDState = 9;
	} else if (stage == STAGE_10 && lastAssistLEDState != 10) {
		IOIF_LED24chAssist(10, 0x20);
		lastAssistLEDState = 10;
	} else {

	}
}

static void UpdateAuxLED(void)
{
	uint8_t assistIncBt = IOIF_ReadGPIOPin(IOIF_GPIO_PORT_D, IOIF_GPIO_PIN_10);	// ASSIST_BTN_P
	uint8_t assistDecBt = IOIF_ReadGPIOPin(IOIF_GPIO_PORT_D, IOIF_GPIO_PIN_12); // ASSIST_BTN_N
	uint32_t currentTime = HAL_GetTick();

	if (systemStateFlag == SYSTEM_ASSIST) {
		// Assist Force Increment 버튼이 눌렸고, 이전 상태는 떼어져 있었던 경우
		if (assistIncBt == IOIF_GPIO_PIN_RESET && lastAssistIncBtState == 1 && (currentTime - lastAssistBtPressTime > DEBOUNCE_TIME)) {
			if (assistStage < STAGE_10) {
				assistStage++;
				assistFlag = 1;
			}
			handleAssistStage(assistStage);
			lastAssistBtPressTime = currentTime;
		} else if (assistDecBt == IOIF_GPIO_PIN_RESET && lastAssistDecBtState == 1 && (currentTime - lastAssistBtPressTime > DEBOUNCE_TIME)) {
			if (assistStage > STAGE_0) {
				assistStage--;
				assistFlag = 2;
			}
			handleAssistStage(assistStage);
			lastAssistBtPressTime = currentTime;
		}
	} else {
		assistStage = 0;
		handleAssistStage(assistStage);
		lastAssistBtPressTime = currentTime;
	}

	// 버튼의 이전 상태를 업데이트
	lastAssistIncBtState = assistIncBt;
	lastAssistDecBtState = assistDecBt;
}

// TODO : BSP!!
static void InitBatteryMonitor(void)
{
	bmInitRes = IOIF_InitBat(IOIF_I2C4);
	if (bmInitRes != IOIF_BAT_STATUS_OK) { //i2c revocery
		systemStateFlag = SYSTEM_ERROR;
		// TODO : Error Handling
		__HAL_RCC_I2C4_CLK_DISABLE();
		__HAL_I2C_DISABLE(&hi2c4);
		HAL_I2C_DeInit(&hi2c4);
		HAL_I2C_MspDeInit(&hi2c4);

		// GPIOF, LTC2944_I2C_SDA_Pin(GPIO_PIN_15), LTC2944_I2C_SCL_Pin(GPIO_PIN_14)
		HAL_I2C_BusReset(&hi2c4, GPIOF, IOIF_GPIO_PIN_15, GPIOF, IOIF_GPIO_PIN_14);

		HAL_I2C_Init(&hi2c4);
		HAL_I2C_MspInit(&hi2c4);
		__HAL_I2C_ENABLE(&hi2c4);
		__HAL_RCC_I2C4_CLK_ENABLE();

		IsRecoverI2C4 = true;

		bmInitRes = IOIF_InitBat(IOIF_I2C4);
	}
}

/* ----------------------- ROUTINE ------------------------ */
#endif /* SUIT_MINICM_ENABLED */
