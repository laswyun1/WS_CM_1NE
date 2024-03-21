

#include "WS_ext_dev_ctrl.h"

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

TaskObj_t extDevCtrl;

IOIF_UPRIGHT_t pelvicUpright[PELVIC_UPRIGHT_NUM];
uint8_t Upright_state;


/**
 *------------------------------------------------------------
 *                      STATIC VARIABLES
 *------------------------------------------------------------
 * @brief Variables local to this module.
 */

// for debug
static uint8_t testTimRes = IOIF_TIM_OK;

static uint16_t* rawAdc1 = NULL;  // 0 => CM Board Total Current Sensor, 1 => Hip Left Depth ADC, 2 => Hip Right Length ADC
static uint16_t* rawAdc2 = NULL;  // 0 => Hip Right Depth ADC
static uint16_t* rawAdc3 = NULL;  // 0 => Hip Left Length ADC

static uint8_t guide_left_bt;
static uint8_t guide_right_bt;
static uint8_t EMR_bt;
static uint8_t test_pos;
static uint8_t test_neg;

static uint32_t extDevCtrlLoopCnt;

static IOIF_LS_t lsObj[PELVIC_UPRIGHT_NUM];

static float userSlope, userOffset;


/**
 *------------------------------------------------------------
 *                 STATIC FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Static Function prototypes for this module.
 */

/* ------------------- STATE FUNCTION ------------------- */
static void StateOff_Ent(void);
static void StateOff_Run(void);

static void StateStandby_Run(void);

static void StateEnable_Ent(void);
static void StateEnable_Run(void);
static void StateEnable_Ext(void);

static void StateError_Run(void);
/* ------------------- SDO CALLBACK ------------------- */
/* ------------------- ROUTINE ------------------- */
static int GetPelvicUprightLength(void);
static int RunPelvicUprightCtrlbyLen(void);
static int RunPelvicUprightCtrlbyBT(void);
static int RunGuideSWPolling(void);


/**
 *------------------------------------------------------------
 *                      PUBLIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions that interface with this module.
 */

/* ------------------- SDO CALLBACK ------------------- */
DOP_COMMON_SDO_CB(extDevCtrl)

void InitExtDevCtrl(void)
{
	/* Init */
	InitTask(&extDevCtrl);
	DOPC_AssignTaskID(&extDevCtrl, TASK_IDX_EXT_DEV_CTRL);
	extDevCtrl.period = 10;		//ms

	/* State Definition */
	TASK_CREATE_STATE(&extDevCtrl, TASK_STATE_OFF,     NULL,				StateOff_Run,    	NULL,				false);
	TASK_CREATE_STATE(&extDevCtrl, TASK_STATE_STANDBY, NULL, 				StateStandby_Run,	NULL,				true);
	TASK_CREATE_STATE(&extDevCtrl, TASK_STATE_ENABLE,  StateEnable_Ent,  	StateEnable_Run, 	StateEnable_Ext,	false);
	TASK_CREATE_STATE(&extDevCtrl, TASK_STATE_ERROR,   NULL,             	StateError_Run,  	NULL,				false);

	/* Routine Definition */
	// TODO : Do not use 6,7,8,9,10!!
	TASK_CREATE_ROUTINE(&extDevCtrl, 6, 	NULL, GetPelvicUprightLength,			NULL);
	TASK_CREATE_ROUTINE(&extDevCtrl, 8, 	NULL, RunPelvicUprightCtrlbyLen,		NULL);
	TASK_CREATE_ROUTINE(&extDevCtrl, 9, 	NULL, RunPelvicUprightCtrlbyBT,			NULL);
	TASK_CREATE_ROUTINE(&extDevCtrl, 10,	NULL, RunGuideSWPolling,				NULL);

//	PushRoutine(&extDevCtrl.routine, 9);

	/* DOD Definition */
	// DOD
	DOP_CreateDOD(TASK_IDX_EXT_DEV_CTRL);

	// PDO
	DOP_COMMON_PDO_CREATE(TASK_IDX_EXT_DEV_CTRL, extDevCtrl);

	// SDO
	DOP_COMMON_SDO_CREATE(TASK_IDX_EXT_DEV_CTRL)
	DOP_CreatePDO(TASK_IDX_EXT_DEV_CTRL, 1000, DOP_FLOAT32, 1, &pelvicUpright[0].lengthRef); //"left_length",
	DOP_CreatePDO(TASK_IDX_EXT_DEV_CTRL, 1001, DOP_FLOAT32, 1, &pelvicUpright[1].lengthRef); //"left_depth",
	DOP_CreatePDO(TASK_IDX_EXT_DEV_CTRL, 1002, DOP_FLOAT32, 1, &pelvicUpright[2].lengthRef); //"right_length",
	DOP_CreatePDO(TASK_IDX_EXT_DEV_CTRL, 1003, DOP_FLOAT32, 1, &pelvicUpright[3].lengthRef); //"right_depth",

	Upright_state = 1;
	guide_right_bt = 1;
	guide_left_bt = 1;
	EMR_bt = 1;

	IOIF_InitUpright(&pelvicUpright[0], 0, 1, 2, IOIF_GPIO_PIN_3, IOIF_GPIO_PIN_2, IOIF_GPIO_PIN_0, IOIF_GPIO_PIN_1); 	// HIP_L_L_SW_N_Pin HIP_L_L_SW_P_Pin HIP_L_L_MT_IN1_Pin HIP_L_L_MT_IN2_Pin
	IOIF_InitUpright(&pelvicUpright[1], 1, 1, 2, IOIF_GPIO_PIN_0, IOIF_GPIO_PIN_1, IOIF_GPIO_PIN_2, IOIF_GPIO_PIN_3);	// HIP_L_D_SW_P_Pin HIP_L_D_SW_N_Pin HIP_L_D_MT_IN1_Pin HIP_L_D_MT_IN2_Pin
	IOIF_InitUpright(&pelvicUpright[2], 2, 1, 2, IOIF_GPIO_PIN_7, IOIF_GPIO_PIN_8, IOIF_GPIO_PIN_4, IOIF_GPIO_PIN_5);	// HIP_R_L_SW_P_Pin HIP_R_L_SW_N_Pin HIP_R_L_MT_IN1_Pin HIP_R_L_MT_IN2_Pin
	IOIF_InitUpright(&pelvicUpright[3], 3, 1, 2, IOIF_GPIO_PIN_6, IOIF_GPIO_PIN_4, IOIF_GPIO_PIN_9, IOIF_GPIO_PIN_10);	// HIP_R_D_SW_N_Pin HIP_R_D_SW_P_Pin HIP_R_D_MT_IN1_Pin HIP_R_D_MT_IN2_Pin

	// TODO : Check Direction!!
	IOIF_InitLS(&lsObj[0], PELV_UPRIGHT_SLOPE_LL, PELV_UPRIGHT_OFFSET_LL); //L_L
	IOIF_InitLS(&lsObj[1], PELV_UPRIGHT_SLOPE_LD, PELV_UPRIGHT_OFFSET_LD); //L_D
	IOIF_InitLS(&lsObj[2], PELV_UPRIGHT_SLOPE_RL, PELV_UPRIGHT_OFFSET_RL); //R_L
	IOIF_InitLS(&lsObj[3], PELV_UPRIGHT_SLOPE_RD, PELV_UPRIGHT_OFFSET_RD); //R_D

//	if (IOIF_StartADCDMA(IOIF_ADC1, &rawAdc1, IOIF_ADC1_BUFFER_LENGTH)) {
//		//TODO: Error Process
//	}
//	if (IOIF_StartADCDMA(IOIF_ADC2, &rawAdc2, IOIF_ADC2_BUFFER_LENGTH)) {
//		//TODO: Error Process
//	}
//	if (IOIF_StartADCDMA(IOIF_ADC3, &rawAdc3, IOIF_ADC3_BUFFER_LENGTH)) {
//		//TODO: Error Process
//	}
}

void RunExtDevCtrl(void)
{
	RunTask(&extDevCtrl);
}


/**
 *------------------------------------------------------------
 *                      STATIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions intended for internal use within this module.
 */

/* ------------------- STATE FUNCTION ------------------- */
static void StateOff_Run()
{

}

static void StateStandby_Run()
{
	StateTransition(&extDevCtrl.stateMachine, TASK_STATE_ENABLE);
}

static void StateEnable_Ent()
{
    EntRoutines(&extDevCtrl.routine);
}

static void StateEnable_Run()
{
	GetPelvicUprightLength();
    RunPelvicUprightCtrlbyBT();
    RunGuideSWPolling();
	pelvicUpright[0].lengthAct=200;
	pelvicUpright[1].lengthAct=200;
//    RunRoutines(&extDevCtrl.routine);
}

static void StateEnable_Ext()
{
    ClearRoutines(&extDevCtrl.routine);
}

static void StateError_Run(void)
{

}

/* ------------------- SDO CALLBACK ------------------- */
// static void Set_DcMotor_Length_Cmd(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res)
// {
//     memcpy(&uprightObj.lengthRef, (float*)req->data, sizeof(uprightObj.lengthRef));
//     res->size = 0;
//     res->status = DOP_SDO_SUCC;
// }

/* ------------------- ROUTINE ------------------- */
static int GetPelvicUprightLength(void)
{
	pelvicUpright[0].lengthAct= IOIF_GetLSmm(&lsObj[0], rawAdc3[0]); //L_L
	pelvicUpright[1].lengthAct= IOIF_GetLSmm(&lsObj[1], rawAdc1[1]); //L_D
	pelvicUpright[2].lengthAct= IOIF_GetLSmm(&lsObj[2], rawAdc1[2]); //R_L
	pelvicUpright[3].lengthAct= IOIF_GetLSmm(&lsObj[3], rawAdc2[0]); //R_D
	return 0;
}

static int RunPelvicUprightCtrlbyLen(void)
{
	for (int i = 0; i < PELVIC_UPRIGHT_NUM; i++) {
		IOIF_SetUprightLen(&pelvicUpright[i]);
		IOIF_MoveUpright(&pelvicUpright[i]);
	}
//
//    		Clear_Routines(&extDevCtrl.routine);
//    		PushRoutine(&extDevCtrl.routine, 6);
//    		PushRoutine(&extDevCtrl.routine, 9);
//    }

	return 0;
}

static int RunPelvicUprightCtrlbyBT(void)
{
	//	if (Upright_state == 0) {
		for(int i = 0; i < PELVIC_UPRIGHT_NUM; i++) {
			IOIF_SetUprightBT(&pelvicUpright[i]);

			if (i == RIGHT_LENGTH){
				if(pelvicUpright[LEFT_LENGTH].DirCmd != UPRIGHT_MOVE_STOP){
					pelvicUpright[RIGHT_LENGTH].DirCmd = pelvicUpright[LEFT_LENGTH].DirCmd;
				}
				else if(pelvicUpright[RIGHT_LENGTH].DirCmd != UPRIGHT_MOVE_STOP){
					pelvicUpright[LEFT_LENGTH].DirCmd = pelvicUpright[RIGHT_LENGTH].DirCmd;
				}
			}
			else if(i == RIGHT_DEPTH){
				if(pelvicUpright[LEFT_DEPTH].DirCmd != UPRIGHT_MOVE_STOP){
					pelvicUpright[RIGHT_DEPTH].DirCmd = pelvicUpright[LEFT_LENGTH].DirCmd;
				}
				else if(pelvicUpright[RIGHT_DEPTH].DirCmd != UPRIGHT_MOVE_STOP){
					pelvicUpright[LEFT_DEPTH].DirCmd = pelvicUpright[RIGHT_DEPTH].DirCmd;
				}
			}
			IOIF_MoveUpright(&pelvicUpright[i]);
			}
	return 0;
}

static int RunGuideSWPolling(void)
{
	guide_left_bt	= IOIF_ReadGPIOPin(IOIF_GPIO_PORT_D, IOIF_GPIO_PIN_4);	// GUIDE_SW_LEFT_Pin
	guide_right_bt	= IOIF_ReadGPIOPin(IOIF_GPIO_PORT_D, IOIF_GPIO_PIN_5);	// GUIDE_SW_RIGHT_Pin
	EMR_bt = IOIF_ReadGPIOPin(IOIF_GPIO_PORT_C, IOIF_GPIO_PIN_13); 	// EMR_Pin

//	test_neg=Get_GPIOState(HIP_L_D_SW_N_Pin);
//	test_pos=Get_GPIOState(HIP_L_D_SW_P_Pin);

	return 0;
}

#endif /* WALKON5_CM_ENABLED */
