

#include "AS_dev_mngr.h"

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

DOPI_SDOMsg_t sdo_msg;
DOPI_PDOMsg_t pdo_msg;
AS_CtrlObj_t userCtrlObj[AS_DEV_MAX_NUM];

/* [MD->MiniCM] Raw Data version */
// RH //
float widmDegFinal_RH = 0;
float widmVelFinal_RH = 0;

// LH //
float widmDegFinal_LH = 0;
float widmVelFinal_LH = 0;

// RK //
float widmDegFinal_RK = 0;
float widmVelFinal_RK = 0;
float widmGyrZ_RK = 0;
float widmDegINC_RK = 0;
float widmVelINC_RK = 0;

// LK //
float widmDegFinal_LK = 0;
float widmVelFinal_LK = 0;
float widmGyrZ_LK = 0;
float widmDegINC_LK = 0;
float widmVelINC_LK = 0;

// Current Ref, Cur
float CurrentRef_RH = 0;
float CurrentRef_LH = 0;
float CurrentRef_RK = 0;
float CurrentRef_LK = 0;

float CurrentAct_RH = 0;
float CurrentAct_LH = 0;
float CurrentAct_RK = 0;
float CurrentAct_LK = 0;


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

static int TxSDO(int MD_idx);
static int TxPDO(int MD_idx);
static void SetRoutine(AS_TaskData_t* taskObj, uint8_t routine_id);
static void ClearRoutine(AS_TaskData_t* taskObj);

static void SetMDRoutines(int DevIdx);
static void StandbyStates(int DevIdx);
static void EnableStates(int DevIdx);
static void OffStates(int DevIdx);

static void SetupDOD(AS_CtrlObj_t* obj);
static int AS_FDCAN_CB(uint16_t wasp_id, uint8_t* rx_data);


/**
 *------------------------------------------------------------
 *                      PUBLIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions that interface with this module.
 */

/**
 * @brief Initialize Motor Driver Object(CTRLObj) and FDCAN communication
 */
void InitFDCANDevMngr(void)
{
	static uint8_t isDevInit = 0;

	if (isDevInit == 0) {
		/* FDCAN Init */
		IOIF_InitFDCAN1(NODE_ID_CM);

    	/* Allocate RX Callback*/
		IOIF_SetFDCANRxCB(IOIF_FDCAN1, IOIF_FDCAN_RXFIFO0CALLBACK, AS_FDCAN_CB);

		/* Driver Init */
		for (int DevIdx = 0; DevIdx < AS_DEV_MAX_NUM; DevIdx++) {
			uint8_t tUsage = RS_File.MD_setting[DevIdx].usage;
			
			if (tUsage == 1) {
				uint16_t tempNodeID = RS_File.MD_setting[DevIdx].FDCAN_ID;
				uint8_t  tempFDCANCH = RS_File.MD_setting[DevIdx].FDCAN_CH;

				AS_FDCAN_TxFncPtr txfnc = NULL;

				if (tempFDCANCH == 1) {
					txfnc = IOIF_TransmitFDCAN1;
				} else if (tempFDCANCH == 2) {
					txfnc = IOIF_TransmitFDCAN2;
				} else {
					//error handler
				}

				userCtrlObj[DevIdx].sdo_tx_id = SDO | (NODE_ID_CM << 4) | tempNodeID;
				userCtrlObj[DevIdx].pdo_tx_id = PDO | (NODE_ID_CM << 4) | tempNodeID;
				userCtrlObj[DevIdx].tx_fnc = txfnc;

				memset(&userCtrlObj[DevIdx].data, 0, sizeof(userCtrlObj[DevIdx].data));
				DOPI_InitDevObj(&userCtrlObj[DevIdx].devObj, tempNodeID);
				SetupDOD(&userCtrlObj[DevIdx]);
			}
		}

		isDevInit = 1;
	}
}

/* For SUIT Data Collection */
void SetPDO_H10(int DevIdx)
{
 	DOPI_SDOUnit_t sdoUnit;
 	DOPI_ClearSDO(&sdo_msg);

 	static uint8_t PDOList[16] = {TASK_ID_GAIT, PDO_ID_GAIT_DEG,
								 TASK_ID_GAIT, PDO_ID_GAIT_VEL,
								 TASK_ID_GAIT, PDO_ID_GAIT_GYR_Z,
								 TASK_ID_GAIT, PDO_ID_GAIT_DEG_INC,
								 TASK_ID_GAIT, PDO_ID_GAIT_VEL_INC,

								 TASK_ID_LOWLEVEL, PDO_ID_LOWLEVEL_TOTAL_CURRENT_INPUT,
								 TASK_ID_LOWLEVEL, PDO_ID_LOWLEVEL_CURRENT_OUTPUT,
								 TASK_ID_MIDLEVEL, PDO_ID_MIDLEVEL_VELOCITY_ESTIMATED,
								//   TASK_ID_LOWLEVEL, PDO_ID_LOWLEVEL_MID_CTRL_INPUT,
								//   TASK_ID_LOWLEVEL, PDO_ID_LOWLEVEL_ANALYZER_INPUT,
								//   TASK_ID_LOWLEVEL, PDO_ID_LOWLEVEL_POSITION,
								//   TASK_ID_LOWLEVEL, PDO_ID_LOWLEVEL_VELOCITY,
 								};
 	memcpy(userCtrlObj[DevIdx].data.pdo_list, PDOList, sizeof(PDOList));
 	sdoUnit = DOPI_CreateSDOUnit(&userCtrlObj[DevIdx].devObj, TASK_ID_MSG, SDO_ID_MSG_PDO_LIST, SDO_REQU, sizeof(PDOList)/2);
 	DOPI_AppendSDO(&sdoUnit, &sdo_msg);

 	TxSDO(DevIdx);
}

void SetPDO_K10(int DevIdx)
{
 	DOPI_SDOUnit_t sdoUnit;
 	DOPI_ClearSDO(&sdo_msg);

 	static uint8_t PDOList[16] = {TASK_ID_GAIT, PDO_ID_GAIT_DEG,
								  TASK_ID_GAIT, PDO_ID_GAIT_VEL,
								  TASK_ID_GAIT, PDO_ID_GAIT_GYR_Z,
								  TASK_ID_GAIT, PDO_ID_GAIT_DEG_INC,
								  TASK_ID_GAIT, PDO_ID_GAIT_VEL_INC,
								  
								  TASK_ID_LOWLEVEL, PDO_ID_LOWLEVEL_TOTAL_CURRENT_INPUT,
								  TASK_ID_LOWLEVEL, PDO_ID_LOWLEVEL_CURRENT_OUTPUT,
								  TASK_ID_MIDLEVEL, PDO_ID_MIDLEVEL_VELOCITY_ESTIMATED,
								//   TASK_ID_LOWLEVEL, PDO_ID_LOWLEVEL_MID_CTRL_INPUT,
								//   TASK_ID_LOWLEVEL, PDO_ID_LOWLEVEL_ANALYZER_INPUT,
								//   TASK_ID_LOWLEVEL, PDO_ID_LOWLEVEL_POSITION,
								//   TASK_ID_LOWLEVEL, PDO_ID_LOWLEVEL_VELOCITY,
 								};
 	memcpy(userCtrlObj[DevIdx].data.pdo_list, PDOList, sizeof(PDOList));
 	sdoUnit = DOPI_CreateSDOUnit(&userCtrlObj[DevIdx].devObj, TASK_ID_MSG, SDO_ID_MSG_PDO_LIST, SDO_REQU, sizeof(PDOList)/2);
 	DOPI_AppendSDO(&sdoUnit, &sdo_msg);

 	TxSDO(DevIdx);
}

void SendAuxInput(int DevIdx, float motorAuxIn, float assistPctg)
{
	DOPI_SDOUnit_t sdoUnit;
	DOPI_ClearSDO(&sdo_msg);

	userCtrlObj[DevIdx].data.motor_auxiliary_input = motorAuxIn * assistPctg;
	sdoUnit = DOPI_CreateSDOUnit(&userCtrlObj[DevIdx].devObj, TASK_ID_LOWLEVEL, SDO_ID_LOWLEVEL_SET_AUX_INPUT, SDO_REQU, 1);
	DOPI_AppendSDO(&sdoUnit, &sdo_msg);

	TxSDO(DevIdx);
}

void Send_F_Vector(int DevIdx, uint8_t modeIdx, int16_t TauMax, uint16_t delay)
{
	DOPI_SDOUnit_t sdoUnit;
	DOPI_ClearSDO(&sdo_msg);

	userCtrlObj[DevIdx].data.ModeIdx = modeIdx;
	sdoUnit = DOPI_CreateSDOUnit(&userCtrlObj[DevIdx].devObj, TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_F_VECTOR_MODE_IDX, SDO_REQU, 1);
	DOPI_AppendSDO(&sdoUnit, &sdo_msg);

	userCtrlObj[DevIdx].data.TauMax = TauMax * assistForcePctg;
	sdoUnit = DOPI_CreateSDOUnit(&userCtrlObj[DevIdx].devObj, TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_F_VECTOR_TMAX, SDO_REQU, 1);
	DOPI_AppendSDO(&sdoUnit, &sdo_msg);

	userCtrlObj[DevIdx].data.Delay = delay;
	sdoUnit = DOPI_CreateSDOUnit(&userCtrlObj[DevIdx].devObj, TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_F_VECTOR_DELAY, SDO_REQU, 1);
	DOPI_AppendSDO(&sdoUnit, &sdo_msg);

	TxSDO(DevIdx);
}

/*******************************************************************/
void AllDevOffStates(void)
{
	for (int DevIdx = 0; DevIdx < AS_DEV_MAX_NUM; DevIdx++) {
		if (RS_File.MD_setting[DevIdx].usage == 1) {
			OffStates(DevIdx);
		}
	}
}

void AllDevStandbyStates(void)
{
	for (int DevIdx = 0; DevIdx < AS_DEV_MAX_NUM; DevIdx++) {
		if (RS_File.MD_setting[DevIdx].usage == 1) {
			StandbyStates(DevIdx);
		}
	}
}

void AllDevEnableStates(void)
{
	for (int DevIdx = 0; DevIdx < AS_DEV_MAX_NUM; DevIdx++) {
		if (RS_File.MD_setting[DevIdx].usage == 1) {
			EnableStates(DevIdx);
		}
	}
}

void AllDevSetRoutines(void)
{
	for (int DevIdx = 0; DevIdx < AS_DEV_MAX_NUM; DevIdx++) {
		if (RS_File.MD_setting[DevIdx].usage == 1) {
			SetMDRoutines(DevIdx);
		}
	}
}

AS_DevData_t* GetDevDataSet(int DevIdx)
{
    return &userCtrlObj[DevIdx].data;
}


/**
 *------------------------------------------------------------
 *                      STATIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions intended for internal use within this module.
 */

/* IO */
static int TxSDO(int MD_idx)
{
	uint16_t md_fdcan_idx = RS_File.MD_setting[MD_idx].FDCAN_ID;

	userCtrlObj[MD_idx].sdo_tx_id = SDO | (NODE_ID_CM << 4) | md_fdcan_idx;
    return userCtrlObj[MD_idx].tx_fnc(userCtrlObj[MD_idx].sdo_tx_id, sdo_msg.txBuf, sdo_msg.msgLength);
}

static int TxPDO(int MD_idx)
{
	uint16_t md_fdcan_idx = RS_File.MD_setting[MD_idx].FDCAN_ID;

	userCtrlObj[MD_idx].pdo_tx_id = PDO | (NODE_ID_CM << 4) | md_fdcan_idx;
    return userCtrlObj[MD_idx].tx_fnc(userCtrlObj[MD_idx].pdo_tx_id, pdo_msg.txBuf, pdo_msg.msgLength);
}

static void SetRoutine(AS_TaskData_t* taskObj, uint8_t routine_id)
{
    uint8_t idx = taskObj->n_routines;

    if (idx > 0) {
        for (int i = 0; i < idx; ++i) { // Check if the routine is already set
            if (taskObj->routines[i] == routine_id) {
                return;
            }
        }
    }

    if (idx < AS_DEV_MAX_ROUTINES) {
        taskObj->routines[idx] = routine_id;
        ++taskObj->n_routines;
    }
}

static void ClearRoutine(AS_TaskData_t* taskObj)
{
	memset(&taskObj->routines, 0, sizeof(taskObj->routines[0])*AS_DEV_MAX_ROUTINES);
	taskObj->n_routines = 0;
}

static void SetMDRoutines(int DevIdx)
{
	DOPI_SDOUnit_t sdoUnit;
	DOPI_ClearSDO(&sdo_msg);

	/* (1) Set MSG Routines */
	SetRoutine(&userCtrlObj[DevIdx].data.msg_hdlr_task, ROUTINE_ID_MSG_PDO_SEND);
    sdoUnit = DOPI_CreateSDOUnit(&userCtrlObj[DevIdx].devObj, TASK_ID_MSG, SDO_ID_MSG_SET_ROUTINE, SDO_REQU, 1);
    DOPI_AppendSDO(&sdoUnit, &sdo_msg);

	/* (2) Set Low Level Ctrl Routines */
	// Activate low&mid Routine (Current Ctrl, Torque Generator)
	SetRoutine(&userCtrlObj[DevIdx].data.low_level_ctrl_task, ROUTINE_ID_LOWLEVEL_CURRENT_CTRL);
	sdoUnit = DOPI_CreateSDOUnit(&userCtrlObj[DevIdx].devObj, TASK_ID_LOWLEVEL, SDO_ID_LOWLEVEL_SET_ROUTINE, SDO_REQU, 1);
	DOPI_AppendSDO(&sdoUnit, &sdo_msg);

	// SetRoutine(&userCtrlObj[DevIdx].data.low_level_ctrl_task, ROUTINE_ID_LOWLEVEL_ADV_FRICTION_COMPENSATION);
	// sdoUnit = DOPI_CreateSDOUnit(&userCtrlObj[DevIdx].devObj, TASK_ID_LOWLEVEL, SDO_ID_LOWLEVEL_SET_ROUTINE, SDO_REQU, 1);
	// DOPI_AppendSDO(&sdoUnit, &sdo_msg);

	/* (3) Set Mid Level Ctrl Routines */
	// SetRoutine(&userCtrlObj[DevIdx].data.mid_level_ctrl_task, ROUTINE_ID_MIDLEVEL_IRC);
	// sdoUnit = DOPI_CreateSDOUnit(&userCtrlObj[DevIdx].devObj, TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_SET_ROUTINE, SDO_REQU, 1);
	// DOPI_AppendSDO(&sdoUnit, &sdo_msg);

	SetRoutine(&userCtrlObj[DevIdx].data.mid_level_ctrl_task, ROUTINE_ID_MIDLEVEL_F_VECTOR_DECODER);
	sdoUnit = DOPI_CreateSDOUnit(&userCtrlObj[DevIdx].devObj, TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_SET_ROUTINE, SDO_REQU, 1);
	DOPI_AppendSDO(&sdoUnit, &sdo_msg);

	/* (4) Set Imu Ctrl Routines */
	// SetRoutine(&userCtrlObj[DevIdx].data.imu_ctrl_task, ROUTINE_ID_IMU_QUATERNION);
    // sdo_unit = DOPI_CreateSDOUnit(&userCtrlObj[DevIdx].devObj, TASK_ID_IMU, SDO_ID_IMU_SET_ROUTINE, SDO_REQU, userCtrlObj[DevIdx].data.imu_ctrl_task.n_routines);
    // DOPI_AppendSDO(&sdo_unit, &sdo_msg);

	/* (5) Set Gait Ctrl Routines */
	SetRoutine(&userCtrlObj[DevIdx].data.gait_ctrl_task, ROUTINE_ID_GAIT_TOTAL_FUNCTION);
	sdoUnit = DOPI_CreateSDOUnit(&userCtrlObj[DevIdx].devObj, TASK_ID_GAIT, SDO_ID_GAIT_SET_ROUTINE, SDO_REQU, userCtrlObj[DevIdx].data.gait_ctrl_task.n_routines);
	DOPI_AppendSDO(&sdoUnit, &sdo_msg);

	TxSDO(DevIdx);
}

static void StandbyStates(int DevIdx)
{
    DOPI_SDOUnit_t sdoUnit;
    DOPI_ClearSDO(&sdo_msg);

    userCtrlObj[DevIdx].data.low_level_ctrl_task.state = TASK_STATE_STANDBY;
    sdoUnit = DOPI_CreateSDOUnit(&userCtrlObj[DevIdx].devObj, TASK_ID_LOWLEVEL, SDO_ID_LOWLEVEL_SET_STATE, SDO_REQU, 1);
    DOPI_AppendSDO(&sdoUnit, &sdo_msg);

//    userCtrlObj[DevIdx].data.mid_level_ctrl_task.state = TASK_STATE_STANDBY;
//    sdo_unit = DOPI_CreateSDOUnit(&userCtrlObj[DevIdx].devObj, TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_SET_STATE, SDO_REQU, 1);
//    DOPI_AppendSDO(&sdoUnit, &sdo_msg);
//
//    userCtrlObj[DevIdx].data.msg_hdlr_task.state = TASK_STATE_STANDBY;
//    sdo_unit = DOPI_CreateSDOUnit(&userCtrlObj[DevIdx].devObj, TASK_ID_MSG, SDO_ID_MSG_SET_STATE, SDO_REQU, 1);
//    DOPI_AppendSDO(&sdoUnit, &sdo_msg);

    TxSDO(DevIdx);
}

static void EnableStates(int DevIdx)
{
    DOPI_SDOUnit_t sdoUnit;
    DOPI_ClearSDO(&sdo_msg);

    userCtrlObj[DevIdx].data.low_level_ctrl_task.state = TASK_STATE_ENABLE;
    sdoUnit = DOPI_CreateSDOUnit(&userCtrlObj[DevIdx].devObj, TASK_ID_LOWLEVEL, SDO_ID_LOWLEVEL_SET_STATE, SDO_REQU, 1);
    DOPI_AppendSDO(&sdoUnit, &sdo_msg);

    userCtrlObj[DevIdx].data.mid_level_ctrl_task.state = TASK_STATE_ENABLE;
    sdoUnit = DOPI_CreateSDOUnit(&userCtrlObj[DevIdx].devObj, TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_SET_STATE, SDO_REQU, 1);
    DOPI_AppendSDO(&sdoUnit, &sdo_msg);

    userCtrlObj[DevIdx].data.gait_ctrl_task.state = TASK_STATE_ENABLE;
    sdoUnit = DOPI_CreateSDOUnit(&userCtrlObj[DevIdx].devObj, TASK_ID_GAIT, SDO_ID_GAIT_SET_STATE, SDO_REQU, 1);
    DOPI_AppendSDO(&sdoUnit, &sdo_msg);

    userCtrlObj[DevIdx].data.msg_hdlr_task.state = TASK_STATE_ENABLE;
    sdoUnit = DOPI_CreateSDOUnit(&userCtrlObj[DevIdx].devObj, TASK_ID_MSG, SDO_ID_MSG_SET_STATE, SDO_REQU, 1);
    DOPI_AppendSDO(&sdoUnit, &sdo_msg);

    TxSDO(DevIdx);
}

static void OffStates(int DevIdx)
{
    DOPI_SDOUnit_t sdoUnit;
    DOPI_ClearSDO(&sdo_msg);

    userCtrlObj[DevIdx].data.low_level_ctrl_task.state = TASK_STATE_OFF;
    sdoUnit = DOPI_CreateSDOUnit(&userCtrlObj[DevIdx].devObj, TASK_ID_LOWLEVEL, SDO_ID_LOWLEVEL_SET_STATE, SDO_REQU, 1);
    DOPI_AppendSDO(&sdoUnit, &sdo_msg);

    userCtrlObj[DevIdx].data.mid_level_ctrl_task.state = TASK_STATE_OFF;
    sdoUnit = DOPI_CreateSDOUnit(&userCtrlObj[DevIdx].devObj, TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_SET_STATE, SDO_REQU, 1);
    DOPI_AppendSDO(&sdoUnit, &sdo_msg);

    userCtrlObj[DevIdx].data.gait_ctrl_task.state = TASK_STATE_OFF;
    sdoUnit = DOPI_CreateSDOUnit(&userCtrlObj[DevIdx].devObj, TASK_ID_GAIT, SDO_ID_GAIT_SET_STATE, SDO_REQU, 1);
    DOPI_AppendSDO(&sdoUnit, &sdo_msg);

    userCtrlObj[DevIdx].data.msg_hdlr_task.state = TASK_STATE_OFF;
    sdoUnit = DOPI_CreateSDOUnit(&userCtrlObj[DevIdx].devObj, TASK_ID_MSG, SDO_ID_MSG_SET_STATE, SDO_REQU, 1);
    DOPI_AppendSDO(&sdoUnit, &sdo_msg);

    TxSDO(DevIdx);
}

/**
 * @brief Set the address of SDO/PDO which you want to use
 * @param MD Object
 */
static void SetupDOD(AS_CtrlObj_t* obj)
{
	// /* LOW LEVEL */
    // SDO
	DOPI_SetSDOAddr(&obj->devObj, TASK_ID_LOWLEVEL, SDO_ID_LOWLEVEL_SET_STATE,			&obj->data.low_level_ctrl_task.state);
	DOPI_SetSDOAddr(&obj->devObj, TASK_ID_LOWLEVEL, SDO_ID_LOWLEVEL_SET_ROUTINE,		obj->data.low_level_ctrl_task.routines);

	DOPI_SetSDOAddr(&obj->devObj, TASK_ID_LOWLEVEL, SDO_ID_LOWLEVEL_SET_AUX_INPUT,		&obj->data.motor_auxiliary_input);

	// PDO
	DOPI_SetPDOAddr(&obj->devObj, TASK_ID_LOWLEVEL,	PDO_ID_LOWLEVEL_TOTAL_CURRENT_INPUT,	&obj->data.totalCurrentInput);
	DOPI_SetPDOAddr(&obj->devObj, TASK_ID_LOWLEVEL,	PDO_ID_LOWLEVEL_CURRENT_OUTPUT,  		&obj->data.CurrentOutput);

    // /* MID LEVEL */
    // SDO
	DOPI_SetSDOAddr(&obj->devObj, TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_SET_STATE,			&obj->data.mid_level_ctrl_task.state);
	DOPI_SetSDOAddr(&obj->devObj, TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_SET_ROUTINE,		obj->data.mid_level_ctrl_task.routines);

	DOPI_SetSDOAddr(&obj->devObj, TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_F_VECTOR_MODE_IDX,	&obj->data.ModeIdx);
	DOPI_SetSDOAddr(&obj->devObj, TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_F_VECTOR_TMAX,		&obj->data.TauMax);
	DOPI_SetSDOAddr(&obj->devObj, TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_F_VECTOR_DELAY,		&obj->data.Delay);

    // PDO
	DOPI_SetPDOAddr(&obj->devObj, TASK_ID_MIDLEVEL, PDO_ID_MIDLEVEL_VELOCITY_ESTIMATED, &obj->data.velEstimated);


	// /* Msg Hdlr */
    // SDO
    DOPI_SetSDOAddr(&obj->devObj, TASK_ID_MSG, SDO_ID_MSG_SET_STATE,		&obj->data.msg_hdlr_task.state);
    DOPI_SetSDOAddr(&obj->devObj, TASK_ID_MSG, SDO_ID_MSG_SET_ROUTINE,		obj->data.msg_hdlr_task.routines);
	DOPI_SetSDOAddr(&obj->devObj, TASK_ID_MSG, SDO_ID_MSG_PDO_LIST,		obj->data.pdo_list);

	// PDO

	// /* Gait Ctrl */
    // SDO
	DOPI_SetSDOAddr(&obj->devObj, TASK_ID_GAIT, SDO_ID_GAIT_SET_STATE,		&obj->data.gait_ctrl_task.state);
	DOPI_SetSDOAddr(&obj->devObj, TASK_ID_GAIT, SDO_ID_GAIT_SET_ROUTINE,	obj->data.gait_ctrl_task.routines);

	// PDO
	DOPI_SetPDOAddr(&obj->devObj, TASK_ID_GAIT,	PDO_ID_GAIT_DEG,		&obj->data.SAM_degfinal);
	DOPI_SetPDOAddr(&obj->devObj, TASK_ID_GAIT,	PDO_ID_GAIT_VEL,		&obj->data.SAM_velfinal);
	DOPI_SetPDOAddr(&obj->devObj, TASK_ID_GAIT,	PDO_ID_GAIT_DEG_INC,	&obj->data.SAM_degINC);
	DOPI_SetPDOAddr(&obj->devObj, TASK_ID_GAIT,	PDO_ID_GAIT_VEL_INC,	&obj->data.SAM_velINC);
	DOPI_SetPDOAddr(&obj->devObj, TASK_ID_GAIT,	PDO_ID_GAIT_GYR_Z,		&obj->data.SAM_gyrZ);

    // /* Imu Ctrl */
    // SDO
    // PDO

    /* System Mngt */
	// SDO
	// PDO

    /* Ext Dev Ctrl */
	// SDO
	// PDO

	
}

/**
 * @brief FDCAN RxCallback function which executes when the ori_node is matched
 * @param wasp_id = 11bit identifier of FDCAN, rx_data = data packet
 */
static int AS_FDCAN_CB(uint16_t wasp_id, uint8_t* rx_data)
{
    uint16_t fncCode = wasp_id & 0x700;
    uint16_t md_fdcan_idx = (wasp_id & 0x0F0)>>4;

    uint16_t tempDevIdx = 0;
    for (int i = 0; i < AS_DEV_MAX_NUM; i++) {
    	if (RS_File.MD_setting[i].FDCAN_ID == md_fdcan_idx) {
    		tempDevIdx = md_fdcan_idx;
    		break;
    	}
    }

    switch(fncCode) {
        case EMCY: break;
        case SDO: DOPI_UnpackSDO(&userCtrlObj[tempDevIdx].devObj, rx_data); break;
        case PDO: DOPI_UnpackPDO(&userCtrlObj[tempDevIdx].devObj, rx_data); break;
        default: break;
    }

    return 0;
}


#endif /* SUIT_MINICM_ENABLED */
