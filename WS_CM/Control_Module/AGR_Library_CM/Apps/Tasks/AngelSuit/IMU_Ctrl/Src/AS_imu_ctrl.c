

#include "AS_imu_ctrl.h"

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

TaskObj_t imuCtrlTask;
uint32_t imuCtrlLoopCnt;

IOIF_6AxisData_t    CM_imu6AxisDataObj;
IOIF_MagData_t 		CM_magDataObj;

// For Quaternion //
VQF_MagCalib_t CM_vqfMagCalibObj;
VQF_t vqfObj;

float magValueCal[3];
VQF_Real_t vqfGyr[3];
VQF_Real_t vqfAcc[3];
VQF_Real_t vqfMag[3];
VQF_Real_t vqfQuat[4];
int16_t q_send[4];

float A_inv[3][3] = {  	{ 0.905e-03,  -0.0069e-03, -0.00083e-03},
						{-0.0069e-03,   0.971e-03,  -0.018e-03},
						{-0.00083e-03,   -0.0181e-03,  0.9629e-03}   };

float ironErr[3] = {0.635e+03, -0.703e+03, -1.42e+03};

float RealMagX;
float RealMagY;
float RealMagZ;


/**
 *------------------------------------------------------------
 *                      STATIC VARIABLES
 *------------------------------------------------------------
 * @brief Variables local to this module.
 */

static bool IsRecoverI2C1 = false;
static bool IsRecoverI2C2 = false;

static uint8_t testImu6AxisRes 	= IOIF_I2C_STATUS_OK;
static uint8_t testImu3AxisRes 	= IOIF_I2C_STATUS_OK;


/**
 *------------------------------------------------------------
 *                 STATIC FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Static Function prototypes for this module.
 */

/* --------------------- SDO CALLBACK --------------------- */
static void SetMagInvAInfo(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);
static void SetMagIronErrorInfo(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);

/* ----------------------- FUNCTION ----------------------- */
static void InitializeIMU(void);

static int RunGetIMUFunction(void);
static void GetInitValueIMU(void);

// Quaternion //
static void InitMagInfo(void);
static int EntGetQuaternion(void);
static int RunGetQuaternion(void);

/* ----------------------- ROUTINE ------------------------ */

/* -------------------- STATE FUNCTION -------------------- */
static void StateOff_Run(void);

static void StateStandby_Run(void);

static void StateEnable_Ent(void);
static void StateEnable_Run(void);
static void StateEnable_Ext(void);

static void StateError_Run(void);


/**
 *------------------------------------------------------------
 *                      PUBLIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions that interface with this module.
 */
/* --------------------- SDO CALLBACK --------------------- */
DOP_COMMON_SDO_CB(imuCtrlTask)

/* ------------------------- MAIN ------------------------- */
void InitIMUCtrl(void)
{
	/* Init Task */
    InitTask(&imuCtrlTask);
	DOPC_AssignTaskID(&imuCtrlTask, TASK_IDX_IMU_CTRL);

	/* Init Device */

	/* State Definition */
	TASK_CREATE_STATE(&imuCtrlTask, TASK_STATE_OFF,      NULL,   			StateOff_Run,       NULL,         		 true);
	TASK_CREATE_STATE(&imuCtrlTask, TASK_STATE_STANDBY,  NULL,   			StateStandby_Run,	NULL,         		 false);
	TASK_CREATE_STATE(&imuCtrlTask, TASK_STATE_ENABLE,   StateEnable_Ent,	StateEnable_Run, 	StateEnable_Ext,	 false);
	TASK_CREATE_STATE(&imuCtrlTask, TASK_STATE_ERROR,    NULL,				StateError_Run,    	NULL,				 false);

	/* Routine Definition */
	// TASK_CREATE_ROUTINE(&imuCtrlTask, ROUTINE_ID_SYSMNGT_GET_POWER_VALUE, 	NULL, 	GetBatVal_Run, 	NULL);

	/* DOD Definition */
	// DOD
	DOP_CreateDOD(TASK_IDX_IMU_CTRL);

   	// PDO
	DOP_COMMON_PDO_CREATE(TASK_IDX_IMU_CTRL, imuCtrlTask);
	// DOP_CreatePDO(TASK_IDX_IMU_CTRL, object_id, DOP_FLOAT32, length, pDataAddr);

	// SDO
	DOP_COMMON_SDO_CREATE(TASK_IDX_IMU_CTRL)
	// DOP_CreateSDO(TASK_IDX_IMU_CTRL, object_id, DOP_FLOAT32, SDOCallback);

	/* Initial stage of get angle */
    InitializeIMU();
}

void RunIMUCtrl(void)
{
	RunTask(&imuCtrlTask);
}

/* ----------------------- FUNCTION ----------------------- */

/**
 *------------------------------------------------------------
 *                      STATIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions intended for internal use within this module.
 */

/* -------------------- STATE FUNCTION -------------------- */
static void StateOff_Run(void)
{
	StateTransition(&imuCtrlTask.stateMachine, TASK_STATE_STANDBY);
}

static void StateStandby_Run(void)
{
    GetInitValueIMU();  // for DMA Read
	if (testImu6AxisRes == IOIF_IMU6AXIS_STATUS_OK || testImu3AxisRes == IOIF_IMU3AXIS_STATUS_OK) {
        StateTransition(&imuCtrlTask.stateMachine, TASK_STATE_ENABLE);
	}
}

static void StateEnable_Ent(void)
{
    imuCtrlLoopCnt = 0;
	EntRoutines(&imuCtrlTask.routine);
}

static void StateEnable_Run(void)
{
    RunGetIMUFunction();
	RunRoutines(&imuCtrlTask.routine);
	imuCtrlLoopCnt++;
}

static void StateEnable_Ext(void)
{
    imuCtrlLoopCnt = 0;
    ExtRoutines(&imuCtrlTask.routine);
}

static void StateError_Run(void)
{

}

/* --------------------- SDO CALLBACK --------------------- */

/* ----------------------- FUNCTION ----------------------- */
static void InitializeIMU(void)
{
	testImu6AxisRes = IOIF_Init6Axis(IOIF_I2C2);
	if (testImu6AxisRes != IOIF_IMU6AXIS_STATUS_OK) { //i2c revocery
		// TODO : Error Handling
		__HAL_RCC_I2C2_CLK_DISABLE();
		__HAL_I2C_DISABLE(&hi2c2);
		HAL_I2C_DeInit(&hi2c2);
		HAL_I2C_MspDeInit(&hi2c2);

		// GPIOB, 6X_IMU_I2C_SDA(GPIO_PIN_0), 6X_IMU_I2C_SCL(GPIO_PIN_1)
		HAL_I2C_BusReset(&hi2c2, GPIOF, IOIF_GPIO_PIN_0, GPIOF, IOIF_GPIO_PIN_1);

		HAL_I2C_Init(&hi2c2);
		HAL_I2C_MspInit(&hi2c2);
		__HAL_I2C_ENABLE(&hi2c2);
		__HAL_RCC_I2C2_CLK_ENABLE();

		IsRecoverI2C2 = true;

		testImu6AxisRes = IOIF_Init6Axis(IOIF_I2C2);
	}

	testImu3AxisRes = IOIF_InitMag(IOIF_I2C1);
	if (testImu3AxisRes != IOIF_IMU3AXIS_STATUS_OK) { //i2c revocery
		// TODO : Error Handling
		__HAL_RCC_I2C1_CLK_DISABLE();
		__HAL_I2C_DISABLE(&hi2c1);
		HAL_I2C_DeInit(&hi2c1);
		HAL_I2C_MspDeInit(&hi2c1);

		// GPIOB, 3X_IMU_SDA(GPIO_PIN_7), 3X_IMU_SCL(GPIO_PIN_6)
		HAL_I2C_BusReset(&hi2c1, GPIOB, IOIF_GPIO_PIN_7, GPIOB, IOIF_GPIO_PIN_6);

		HAL_I2C_Init(&hi2c1);
		HAL_I2C_MspInit(&hi2c1);
		__HAL_I2C_ENABLE(&hi2c1);
		__HAL_RCC_I2C1_CLK_ENABLE();

		IsRecoverI2C1 = true;

		testImu3AxisRes = IOIF_InitMag(IOIF_I2C1);
	}
}


static void GetInitValueIMU(void)
{
    for (uint16_t j = 0; j < 30000; j++) {
        // For delay of DMA reading
    }

    testImu6AxisRes = IOIF_Get6AxisValue(&CM_imu6AxisDataObj);
	testImu3AxisRes = IOIF_GetMagValue(&CM_magDataObj);
}

/* Just get 6Axis & 3Axis IMU values */
static int RunGetIMUFunction(void)
{
	testImu6AxisRes = IOIF_Get6AxisValue(&CM_imu6AxisDataObj);
	testImu3AxisRes = IOIF_GetMagValue(&CM_magDataObj);

	return 0;
}

// Quaternion //
static void InitMagInfo(void)
{
	CM_vqfMagCalibObj.A_inv[0][0] = CM_vqfMagCalibObj.a11;
	CM_vqfMagCalibObj.A_inv[0][1] = CM_vqfMagCalibObj.a12;
	CM_vqfMagCalibObj.A_inv[0][2] = CM_vqfMagCalibObj.a13;
	CM_vqfMagCalibObj.A_inv[1][0] = CM_vqfMagCalibObj.a21;
	CM_vqfMagCalibObj.A_inv[1][1] = CM_vqfMagCalibObj.a22;
	CM_vqfMagCalibObj.A_inv[1][2] = CM_vqfMagCalibObj.a23;
	CM_vqfMagCalibObj.A_inv[2][0] = CM_vqfMagCalibObj.a31;
	CM_vqfMagCalibObj.A_inv[2][1] = CM_vqfMagCalibObj.a32;
	CM_vqfMagCalibObj.A_inv[2][2] = CM_vqfMagCalibObj.a33;

	CM_vqfMagCalibObj.ironErr[0] = CM_vqfMagCalibObj.b1;
	CM_vqfMagCalibObj.ironErr[1] = CM_vqfMagCalibObj.b2;
	CM_vqfMagCalibObj.ironErr[2] = CM_vqfMagCalibObj.b3;
}

static int EntGetQuaternion(void)
{
	VQF_Params_t params = { 0 };
	vqfObj.params = params;

	VQF_Init(&vqfObj, 0.001, 0.001, 0.001);
	return 0;
}

static int RunGetQuaternion(void)
{
	//uint8_t t6AxisRes
	//uint8_t t3AxisRes

	IOIF_Get6AxisValue(&CM_imu6AxisDataObj);
	IOIF_GetMagValue(&CM_magDataObj);

	//if (t6AxisRes != IOIF_IMU6AXIS_STATUS_OK) { return t6AxisRes; }
	//if (t3AxisRes != IOIF_IMU3AXIS_STATUS_OK) { return t3AxisRes; }

	float accX = CM_imu6AxisDataObj.accX;
	float accY = CM_imu6AxisDataObj.accY;
	float accZ = CM_imu6AxisDataObj.accZ;
	float gyrX = CM_imu6AxisDataObj.gyrX * 0.017453f; //degree to radian
	float gyrY = CM_imu6AxisDataObj.gyrY * 0.017453f; //degree to radian
	float gyrZ = CM_imu6AxisDataObj.gyrZ * 0.017453f; //degree to radian

	float magX_r = -CM_magDataObj.magY;
	float magY_r = CM_magDataObj.magX;
	float magZ_r = CM_magDataObj.magZ;

	RealMagX = magX_r;
	RealMagY = magY_r;
	RealMagZ = magZ_r;

	float magValueRaw[3] = {magX_r, magY_r, magZ_r};

	for (int i = 0; i < 3; i++) {
		magValueCal[i] = 0;
		for (int j = 0; j < 3; j++) {
			magValueCal[i] += A_inv[i][j] * (magValueRaw[j] - ironErr[j]);
//			magValueCal[i] += vqfMagCalibInfo.A_inv[i][j] * (magValueRaw[j] - mag_calib_info.iron_err[j]);
		}
	}
    
	float magnorm = sqrt(magValueCal[0]*magValueCal[0] + magValueCal[1]*magValueCal[1] + magValueCal[2]*magValueCal[2]);
	magValueCal[0] = magValueCal[0] / magnorm;
	magValueCal[1] = magValueCal[1] / magnorm;
	magValueCal[2] = magValueCal[2] / magnorm;

	float magX = magValueCal[0];
	float magY = magValueCal[1];
	float magZ = magValueCal[2];

    VQF_Real_t gyr[3] = {gyrX, gyrY, gyrZ};
    VQF_Real_t acc[3] = {accX, accY, accZ};
    VQF_Real_t mag[3] = {magX, magY, magZ};

	VQF_UpdateWithMag(&vqfObj, gyr, acc, mag);
	VQF_GetQuat9D(&vqfObj, vqfQuat);
	int i;
	for (i = 0; i < 4; i++) { q_send[i] = (int16_t)(vqfQuat[i] * 32768); }

	return 0;
}

/* SDO Callback */
static void SetMagInvAInfo(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res)
{
	memcpy(&CM_vqfMagCalibObj.a11, t_req->data, 9*4);

	CM_vqfMagCalibObj.A_inv[0][0] = CM_vqfMagCalibObj.a11;
	CM_vqfMagCalibObj.A_inv[0][1] = CM_vqfMagCalibObj.a12;
	CM_vqfMagCalibObj.A_inv[0][2] = CM_vqfMagCalibObj.a13;
	CM_vqfMagCalibObj.A_inv[1][0] = CM_vqfMagCalibObj.a21;
	CM_vqfMagCalibObj.A_inv[1][1] = CM_vqfMagCalibObj.a22;
	CM_vqfMagCalibObj.A_inv[1][2] = CM_vqfMagCalibObj.a23;
	CM_vqfMagCalibObj.A_inv[2][0] = CM_vqfMagCalibObj.a31;
	CM_vqfMagCalibObj.A_inv[2][1] = CM_vqfMagCalibObj.a32;
	CM_vqfMagCalibObj.A_inv[2][2] = CM_vqfMagCalibObj.a33;

   t_res->dataSize = 0;
   t_res->status = DOP_SDO_SUCC;
}

static void SetMagIronErrorInfo(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res)
{
	memcpy(&CM_vqfMagCalibObj.b1, t_req->data, 3*4);

	CM_vqfMagCalibObj.ironErr[0] = CM_vqfMagCalibObj.b1;
	CM_vqfMagCalibObj.ironErr[1] = CM_vqfMagCalibObj.b2;
	CM_vqfMagCalibObj.ironErr[2] = CM_vqfMagCalibObj.b3;

   t_res->dataSize = 0;
   t_res->status = DOP_SDO_SUCC;
}


/* ----------------------- ROUTINE ------------------------ */

#endif /* SUIT_MINICM_ENABLED */
