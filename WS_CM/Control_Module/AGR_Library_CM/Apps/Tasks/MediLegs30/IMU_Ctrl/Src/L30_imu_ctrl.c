

#include "L30_imu_ctrl.h"

#ifdef L30_CM_ENABLED

/**
 *-----------------------------------------------------------
 *              TYPE DEFINITIONS AND ENUMERATIONS
 *-----------------------------------------------------------
 * @brief Enumerated types and structures central to this module.
 */




/**
 *------------------------------------------------------------
 *                          VARIABLES
 *------------------------------------------------------------
 * @brief Variables accessible throughout the application.
 */

TaskObj_t IMUCtrlTask;
uint32_t IMUCtrlLoopCnt;

IOIF_6AxisData_t    imu6AxisDataObj;
IOIF_MagData_t 		magDataObj;

// For Debug //
static bool IsRecoverI2C2 = false;
static bool IsRecoverI2C3 = false;

static uint8_t testImu6AxisRes 	= IOIF_I2C_STATUS_OK;
static uint8_t testImu3AxisRes 	= IOIF_I2C_STATUS_OK;


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

/* ------------------- INITIALIZAIONS ------------------- */
static void InitializeIMU(void);

/* ------------------- ROUTINES ------------------- */
static int RunGetIMUFunction(void);
static int32_t IMU_SimpleRPY(float* rpy, float* acc);

/**
 *------------------------------------------------------------
 *                      PUBLIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions that interface with this module.
 */


/* ------------------- SDO CALLBACK ------------------- */
DOP_COMMON_SDO_CB(IMUCtrlTask)

void InitIMUCtrl(void)
{
    // init
    InitTask(&IMUCtrlTask);
	DOPC_AssignTaskID(&IMUCtrlTask, TASK_IDX_IMU_CTRL);

	/* State Definition */
	TASK_CREATE_STATE(&IMUCtrlTask, TASK_STATE_OFF,      NULL,				StateOff_Run,       NULL,				false);
	TASK_CREATE_STATE(&IMUCtrlTask, TASK_STATE_STANDBY,  NULL,				StateStandby_Run,	NULL,       		true);
	TASK_CREATE_STATE(&IMUCtrlTask, TASK_STATE_ENABLE,   StateEnable_Ent,	StateEnable_Run, 	StateEnable_Ext,  	false);
	TASK_CREATE_STATE(&IMUCtrlTask, TASK_STATE_ERROR,    NULL,				StateError_Run,    	NULL,				false);

	/* Routine Definition */

	/* DOD Definition */
	// DOD
	DOP_CreateDOD(TASK_IDX_IMU_CTRL);

   	// PDO
	DOP_COMMON_PDO_CREATE(TASK_IDX_IMU_CTRL, IMUCtrlTask);

	// SDO
	DOP_COMMON_SDO_CREATE(TASK_IDX_IMU_CTRL)

    /* Init 6axis & 3axis IMU */
	InitializeIMU();
}

void RunIMUCtrl(void)
{
	RunTask(&IMUCtrlTask);
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

}

static void StateEnable_Ent(void)
{
	// Ent_Routines(&IMUCtrlTask.routine);
}

static void StateEnable_Run(void)
{
    RunGetIMUFunction();
	// Run_Routines(&IMUCtrlTask.routine);
}

static void StateEnable_Ext(void)
{
	// Ext_Routines(&IMUCtrlTask.routine);
}

static void StateError_Run(void)
{
	// TODO : Error Handle
}

/* Initialize 6Axis & 3Axis IMU */
static void InitializeIMU(void)
{
	testImu6AxisRes = IOIF_Init6Axis(IOIF_I2C2);
	if (testImu6AxisRes != IOIF_IMU6AXIS_STATUS_OK) { //i2c revocery
		// TODO : Error Handling
		__HAL_RCC_I2C2_CLK_DISABLE();
		__HAL_I2C_DISABLE(&hi2c2);
		HAL_I2C_DeInit(&hi2c2);
		HAL_I2C_MspDeInit(&hi2c2);

		// GPIOF, 6X_IMU_SDA(GPIO_PIN_0), 6X_IMU_SCL(GPIO_PIN_1)
		HAL_I2C_BusReset(&hi2c2, GPIOF, IOIF_GPIO_PIN_0, GPIOF, IOIF_GPIO_PIN_1);

		HAL_I2C_Init(&hi2c2);
		HAL_I2C_MspInit(&hi2c2);
		__HAL_I2C_ENABLE(&hi2c2);
		__HAL_RCC_I2C2_CLK_ENABLE();

		IsRecoverI2C2 = true;

		testImu6AxisRes = IOIF_Init6Axis(IOIF_I2C2);
	}

	if (testImu6AxisRes == IOIF_IMU6AXIS_STATUS_OK) {

	}

	testImu3AxisRes = IOIF_InitMag(IOIF_I2C3);
	if (testImu3AxisRes != IOIF_IMU3AXIS_STATUS_OK) { //i2c revocery
		// TODO : Error Handling
		__HAL_RCC_I2C3_CLK_DISABLE();
		__HAL_I2C_DISABLE(&hi2c3);
		HAL_I2C_DeInit(&hi2c3);
		HAL_I2C_MspDeInit(&hi2c3);

		// GPIOC, 3X_IMU_SDA(GPIO_PIN_9), GPIOA, 3X_IMU_SCL(GPIO_PIN_8)
		HAL_I2C_BusReset(&hi2c3, GPIOC, IOIF_GPIO_PIN_9, GPIOA, IOIF_GPIO_PIN_8);

		HAL_I2C_Init(&hi2c3);
		HAL_I2C_MspInit(&hi2c3);
		__HAL_I2C_ENABLE(&hi2c3);
		__HAL_RCC_I2C3_CLK_ENABLE();

		IsRecoverI2C3 = true;

		testImu3AxisRes = IOIF_InitMag(IOIF_I2C3);
	}

	if (testImu3AxisRes == IOIF_IMU3AXIS_STATUS_OK) {

	}
}

/* Just get 6Axis & 3Axis IMU values */
static int RunGetIMUFunction(void)
{
	testImu6AxisRes = IOIF_Get6AxisValue(&imu6AxisDataObj);
	testImu3AxisRes = IOIF_GetMagValue(&magDataObj);

	return 0;
}

static int32_t IMU_SimpleRPY(float* rpy, float* acc)
{
	static float r_buff=0, p_buff=0;
	float lpf = 0.99;

	float ax = acc[2];
	float ay = acc[0];
	float az = acc[1];

	float amp = sqrtf(ax*ax + ay*ay + az*az);
	if (amp == 0) {
		return -1;
	}
	float uax = ax/amp;
	float uay = ay/amp;
	float uaz = az/amp;

	float r = asinf(-uay);
	float p = 0;
	if (r != M_PI/2) {
		p = atan2f(uax, uaz);
	}

	float res_r = lpf*r_buff + (1-lpf)*r;
	float res_p = lpf*p_buff + (1-lpf)*p;
	r_buff = res_r;
	p_buff = res_p;

	rpy[0] = res_r / M_PI * 180;
	rpy[1] = res_p / M_PI * 180;

	return 0;
}



#endif /* L30_CM_ENABLED */
