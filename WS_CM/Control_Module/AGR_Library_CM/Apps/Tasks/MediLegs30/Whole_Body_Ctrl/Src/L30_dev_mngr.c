

#include "L30_dev_mngr.h"

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

L30Struct d10Obj[e_L30_NUM];
L30Struct* d10Ptr[e_L30_NUM];
float** test_addr;

/**
 *------------------------------------------------------------
 *                      STATIC VARIABLES
 *------------------------------------------------------------
 * @brief Variables local to this module.
 */

static uint8_t test_monitor[64];
static int abs_enc_init;

DOPI_SDOMsg_t sdo_msg;
DOPI_PDOMsg_t pdo_msg;

/*
int hipListPDO_size = 5;
static uint8_t hipListPDO[D10_MAX_PDO * 2] = {
        TASK_ID_LOWLEVEL,  PDO_ID_LOWLEVEL_PHASES_CURRENT,
        TASK_ID_MIDLEVEL, PDO_ID_MIDLEVEL_ABS_180,
        TASK_ID_MIDLEVEL, PDO_ID_MIDLEVEL_ABS_MULTITURN,
        TASK_ID_EXTDEV,   PDO_ID_EXTDEV_DC_LENGTH_REF,
        TASK_ID_EXTDEV,   PDO_ID_EXTDEV_DC_BUTTON,
        };

static int kneeListPDO_size = 8;
static uint8_t kneeListPDO[D10_MAX_PDO * 2] = {
        TASK_ID_LOWLEVEL,  PDO_ID_LOWLEVEL_PHASES_CURRENT,
        TASK_ID_MIDLEVEL, PDO_ID_MIDLEVEL_ABS_180,
        TASK_ID_MIDLEVEL, PDO_ID_MIDLEVEL_ABS_MULTITURN,
        TASK_ID_EXTDEV,   PDO_ID_EXTDEV_DC_LENGTH_REF,
        TASK_ID_EXTDEV,   PDO_ID_EXTDEV_DC_BUTTON,
        TASK_ID_EXTDEV,   PDO_ID_EXTDEV_FSR ,
        TASK_ID_EXTDEV,   PDO_ID_EXTDEV_LP ,
        };
*/

/**
 *------------------------------------------------------------
 *                 STATIC FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Static Function prototypes for this module.
 */

static int TxSDO(L30Struct* t_obj);
static int TxPDO(L30Struct* t_obj);
static void SetRoutine(L30_MD_TaskData* task_obj, uint8_t routine_id);
static void ClearRoutine(L30_MD_TaskData* task_obj);
static void UnsetRoutine(L30_MD_TaskData* task_obj, uint8_t routine_id);
static void Setup_L30_DOD(L30Struct* obj);
static int D10_Mngr_FDCAN_Callback(uint16_t wasp_id, uint8_t* rx_data);
static void Init_L30_Instance(int obj_idx);


/**
 *------------------------------------------------------------
 *                      PUBLIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions that interface with this module.
 */

void Init_L30_Mngr()
{
	static uint8_t is_md_init = 0;

	for (int i = 0; i < e_L30_NUM; i++) {
		d10Ptr[i] = &d10Obj[i];
	}

	if (is_md_init == 0) {
		abs_enc_init = 0;

		/* FDCAN Init */
	    IOIF_InitFDCAN1(NODE_ID_CM);
	    IOIF_InitFDCAN2(NODE_ID_CM);

    	/* Allocate RX Callback*/
	    IOIF_SetFDCANRxCB(IOIF_FDCAN1, IOIF_FDCAN_RXFIFO0CALLBACK, D10_Mngr_FDCAN_Callback);
	    IOIF_SetFDCANRxCB(IOIF_FDCAN2, IOIF_FDCAN_RXFIFO0CALLBACK, D10_Mngr_FDCAN_Callback);

		/* D10 Init */
		Init_L30_Instance(e_L30_LH_IDX);
		Init_L30_Instance(e_L30_RH_IDX);
		Init_L30_Instance(e_L30_LK_IDX);
		Init_L30_Instance(e_L30_RK_IDX);

		is_md_init = 1;
	}
}

void Send_PDO_Nothing(L30Struct* t_obj)
{
    DOPI_ClearPDO(&pdo_msg);

    TxPDO(t_obj);
}

//void Start_PDO_Streaming()
//{
//        StartPDOStreaming(e_L30_RH_IDX);
//        StartPDOStreaming(e_L30_RK_IDX);
//}

/* ------------------- UPRIGHT ------------------- */
void Enable_L30_Upright(L30Struct* t_obj)
{

    DOPI_SDOUnit_t sdo_unit;
    DOPI_ClearSDO(&sdo_msg);

    t_obj->data.ext_dev_ctrl_task.state = TASK_STATE_STANDBY;
    sdo_unit = DOPI_CreateSDOUnit(&t_obj->d10_obj, TASK_ID_EXTDEV, SDO_ID_EXTDEV_SET_STATE, SDO_REQU, 1);
    DOPI_AppendSDO(&sdo_unit, &sdo_msg);

    SetRoutine(&t_obj->data.ext_dev_ctrl_task, ROUTINE_ID_EXTDEV_DC_LENGTH_UPDATE);
    SetRoutine(&t_obj->data.ext_dev_ctrl_task, ROUTINE_ID_EXTDEV_DC_BUTTON_CMD);
    sdo_unit = DOPI_CreateSDOUnit(&t_obj->d10_obj, TASK_ID_EXTDEV, SDO_ID_EXTDEV_SET_ROUTINE, SDO_REQU, t_obj->data.ext_dev_ctrl_task.n_routines);
    DOPI_AppendSDO(&sdo_unit, &sdo_msg);

    /* STATE ENABLE */
    t_obj->data.ext_dev_ctrl_task.state = TASK_STATE_ENABLE;
    sdo_unit = DOPI_CreateSDOUnit(&t_obj->d10_obj, TASK_ID_EXTDEV, SDO_ID_EXTDEV_SET_STATE, SDO_REQU, 1);
    DOPI_AppendSDO(&sdo_unit, &sdo_msg);

    TxSDO(t_obj);
}

void Enable_Upright_lenth_cmd(L30Struct* t_obj)
{

    DOPI_SDOUnit_t sdo_unit;
    DOPI_ClearSDO(&sdo_msg);

    t_obj->data.ext_dev_ctrl_task.state = TASK_STATE_STANDBY;
    sdo_unit = DOPI_CreateSDOUnit(&t_obj->d10_obj, TASK_ID_EXTDEV, SDO_ID_EXTDEV_SET_STATE, SDO_REQU, 1);
    DOPI_AppendSDO(&sdo_unit, &sdo_msg);

    SetRoutine(&t_obj->data.ext_dev_ctrl_task, ROUTINE_ID_EXTDEV_DC_LENGTH_UPDATE);
    SetRoutine(&t_obj->data.ext_dev_ctrl_task, ROUTINE_ID_EXTDEV_DC_LENGTH_CMD);
    //SetRoutine(&t_obj->data.ext_dev_ctrl_task, ROUTINE_ID_EXTDEV_DC_BUTTON_CMD);

    sdo_unit = DOPI_CreateSDOUnit(&t_obj->d10_obj, TASK_ID_EXTDEV, SDO_ID_EXTDEV_SET_ROUTINE, SDO_REQU, t_obj->data.ext_dev_ctrl_task.n_routines);
    DOPI_AppendSDO(&sdo_unit, &sdo_msg);

    t_obj->data.ext_dev_ctrl_task.state = TASK_STATE_ENABLE;
    sdo_unit = DOPI_CreateSDOUnit(&t_obj->d10_obj, TASK_ID_EXTDEV, SDO_ID_EXTDEV_SET_STATE, SDO_REQU, 1);
    DOPI_AppendSDO(&sdo_unit, &sdo_msg);

    TxSDO(t_obj);
}

void Disable_L30_Upright(L30Struct* t_obj)
{
    DOPI_SDOUnit_t sdo_unit;
    DOPI_ClearSDO(&sdo_msg);

    ClearRoutine(&t_obj->data.ext_dev_ctrl_task);

    t_obj->data.ext_dev_ctrl_task.state = TASK_STATE_STANDBY;
    sdo_unit = DOPI_CreateSDOUnit(&t_obj->d10_obj, TASK_ID_EXTDEV, SDO_ID_EXTDEV_SET_STATE, SDO_REQU, 1);
    DOPI_AppendSDO(&sdo_unit, &sdo_msg);

    TxSDO(t_obj);
}

void Send_L30_Upright_DIR_CMD(L30Struct* t_obj, IOIF_UprightMoveDir_t* t_dir)
{
    DOPI_PDOUnit_t pdo_unit;
    DOPI_ClearPDO(&pdo_msg);

    pdo_unit = DOPI_CreatePDOUnit(TASK_ID_EXTDEV, PDO_ID_EXTDEV_DC_DIRECTION_CMD, (void*)t_dir);
    DOPI_AppendPDO(&pdo_unit, &pdo_msg);

    TxPDO(t_obj);
}

void Send_L30_Upright_Length_CMD(L30Struct* t_obj, float* t_length_cmd)
{
    DOPI_PDOUnit_t pdo_unit;
    DOPI_ClearPDO(&pdo_msg);

    pdo_unit = DOPI_CreatePDOUnit(TASK_ID_EXTDEV, PDO_ID_EXTDEV_DC_LENGTH_REF, (void*)t_length_cmd);
    DOPI_AppendPDO(&pdo_unit, &pdo_msg);

    TxPDO(t_obj);
}

void Enable_L30_LengthAct(L30Struct* t_obj)
{
	static uint8_t t_pdo_list[2] = {TASK_ID_EXTDEV, PDO_ID_EXTDEV_DC_LENGTH_ACT};
	DOPI_SDOUnit_t sdo_unit;
	DOPI_ClearSDO(&sdo_msg);

	/* EXT DEV ROUTINE SET */

    t_obj->data.ext_dev_ctrl_task.state = TASK_STATE_STANDBY;
    sdo_unit = DOPI_CreateSDOUnit(&t_obj->d10_obj, TASK_ID_EXTDEV, SDO_ID_EXTDEV_SET_STATE, SDO_REQU, 1);
    DOPI_AppendSDO(&sdo_unit, &sdo_msg);

	t_obj->data.ext_dev_ctrl_task.state = TASK_STATE_STANDBY;
	sdo_unit = DOPI_CreateSDOUnit(&t_obj->d10_obj, TASK_ID_EXTDEV, SDO_ID_EXTDEV_SET_STATE, SDO_REQU, 1);
	DOPI_AppendSDO(&sdo_unit, &sdo_msg);

    SetRoutine(&t_obj->data.ext_dev_ctrl_task, ROUTINE_ID_EXTDEV_DC_LENGTH_UPDATE);
//    SetRoutine(&t_obj->data.ext_dev_ctrl_task, ROUTINE_ID_EXTDEV_DC_LENGTH_CMD);
    SetRoutine(&t_obj->data.ext_dev_ctrl_task, ROUTINE_ID_EXTDEV_DC_BUTTON_CMD);

    sdo_unit = DOPI_CreateSDOUnit(&t_obj->d10_obj, TASK_ID_EXTDEV, SDO_ID_EXTDEV_SET_ROUTINE, SDO_REQU, t_obj->data.ext_dev_ctrl_task.n_routines);
    DOPI_AppendSDO(&sdo_unit, &sdo_msg);

	/* PDO REQUEST */
	memcpy(t_obj->data.pdo_list, t_pdo_list, sizeof(t_pdo_list));
	sdo_unit = DOPI_CreateSDOUnit(&t_obj->d10_obj, TASK_ID_MSG, SDO_ID_MSG_PDO_LIST, SDO_REQU, sizeof(t_pdo_list)/2);
	DOPI_AppendSDO(&sdo_unit, &sdo_msg);

    t_obj->data.ext_dev_ctrl_task.state = TASK_STATE_ENABLE;
    sdo_unit = DOPI_CreateSDOUnit(&t_obj->d10_obj, TASK_ID_EXTDEV, SDO_ID_EXTDEV_SET_STATE, SDO_REQU, 1);
    DOPI_AppendSDO(&sdo_unit, &sdo_msg);

	/* STATE ENABLE */
	t_obj->data.ext_dev_ctrl_task.state = TASK_STATE_ENABLE;
	sdo_unit = DOPI_CreateSDOUnit(&t_obj->d10_obj, TASK_ID_EXTDEV, SDO_ID_EXTDEV_SET_STATE, SDO_REQU, 1);
	DOPI_AppendSDO(&sdo_unit, &sdo_msg);

	TxSDO(t_obj);
}

void Disable_L30_LengthAct(L30Struct* t_obj)
{
    DOPI_SDOUnit_t sdo_unit;
    DOPI_ClearSDO(&sdo_msg);

    ClearRoutine(&t_obj->data.ext_dev_ctrl_task);
    ClearRoutine(&t_obj->data.msg_hdlr_task);

    t_obj->data.ext_dev_ctrl_task.state = TASK_STATE_STANDBY;
    sdo_unit = DOPI_CreateSDOUnit(&t_obj->d10_obj, TASK_ID_EXTDEV, SDO_ID_EXTDEV_SET_STATE, SDO_REQU, 1);
    DOPI_AppendSDO(&sdo_unit, &sdo_msg);

    TxSDO(t_obj);
}

/* ------------------- GRF ------------------- */
void Enable_L30_GRF(L30Struct* t_obj)
{
	static uint8_t t_pdo_list[4] = {TASK_ID_EXTDEV, PDO_ID_EXTDEV_FSR, \
									TASK_ID_EXTDEV, PDO_ID_EXTDEV_LP};
    DOPI_SDOUnit_t sdo_unit;
    DOPI_ClearSDO(&sdo_msg);

    /* EXT DEV ROUTINE SET */
    t_obj->data.ext_dev_ctrl_task.state = TASK_STATE_STANDBY;
    sdo_unit = DOPI_CreateSDOUnit(&t_obj->d10_obj, TASK_ID_EXTDEV, SDO_ID_EXTDEV_SET_STATE, SDO_REQU, 1);
    DOPI_AppendSDO(&sdo_unit, &sdo_msg);

    /* PDO REQUEST */
    memcpy(t_obj->data.pdo_list, t_pdo_list, sizeof(t_pdo_list));
    sdo_unit = DOPI_CreateSDOUnit(&t_obj->d10_obj, TASK_ID_MSG, SDO_ID_MSG_PDO_LIST, SDO_REQU, sizeof(t_pdo_list)/2);
    DOPI_AppendSDO(&sdo_unit, &sdo_msg);

    /* STATE ENABLE */
    t_obj->data.ext_dev_ctrl_task.state = TASK_STATE_ENABLE;
    sdo_unit = DOPI_CreateSDOUnit(&t_obj->d10_obj, TASK_ID_EXTDEV, SDO_ID_EXTDEV_SET_STATE, SDO_REQU, 1);
    DOPI_AppendSDO(&sdo_unit, &sdo_msg);

    TxSDO(t_obj);
}

void Disable_L30_GRF(L30Struct* t_obj)
{
    DOPI_SDOUnit_t sdo_unit;
    DOPI_ClearSDO(&sdo_msg);

    ClearRoutine(&t_obj->data.ext_dev_ctrl_task);
    ClearRoutine(&t_obj->data.msg_hdlr_task);

    t_obj->data.ext_dev_ctrl_task.state = TASK_STATE_STANDBY;
    sdo_unit = DOPI_CreateSDOUnit(&t_obj->d10_obj, TASK_ID_EXTDEV, SDO_ID_EXTDEV_SET_STATE, SDO_REQU, 1);
    DOPI_AppendSDO(&sdo_unit, &sdo_msg);

    TxSDO(t_obj);
}


void Enable_L30_Joints_Abs(L30Struct* t_obj)
{
}

void Disable_L30_Joints_Abs(L30Struct* t_obj)
{
}

/* ------------------- POSITION CONTROL ------------------- */
void Ready_L30_Position_Ctrl(L30Struct* t_obj)
{
	static uint8_t t_pdo_list[14] = {/*TASK_ID_LOWLEVEL, PDO_ID_LOWLEVEL_PHASES_CURRENT,*/\
			/*TASK_ID_MIDLEVEL, PDO_ID_MIDLEVEL_ABS_POSITION,*/ \
			TASK_ID_MIDLEVEL, PDO_ID_MIDLEVEL_ACTUAL_POSITION,\
			TASK_ID_SYSMNGT, PDO_ID_SYSTEM_VOLT,\
			TASK_ID_SYSMNGT, PDO_ID_SYSTEM_CURR,\
			TASK_ID_SYSMNGT, PDO_ID_SYSTEM_TEMP,\
			TASK_ID_EXTDEV,PDO_ID_EXTDEV_NTC_MOTOR_TEMP,\
	};

    DOPI_SDOUnit_t sdo_unit;
    DOPI_ClearSDO(&sdo_msg);

//    SetRoutine(&t_obj->data.msg_hdlr_task, ROUTINE_ID_MSG_PDO_SEND);
//    sdo_unit = DOPI_CreateSDOUnit(&t_obj->d10_obj, TASK_ID_MSG, SDO_ID_MSG_SET_ROUTINE, SDO_REQU, 1);
//    DOPI_AppendSDO(&sdo_unit, &sdo_msg);

//    SetRoutine(&t_obj->data.low_level_ctrl_task, ROUTINE_ID_LOWLEVEL_FRICTION_COMPENSATION);
    SetRoutine(&t_obj->data.low_level_ctrl_task, ROUTINE_ID_LOWLEVEL_CURRENT_CTRL);
    sdo_unit = DOPI_CreateSDOUnit(&t_obj->d10_obj, TASK_ID_LOWLEVEL, SDO_ID_LOWLEVEL_SET_ROUTINE, SDO_REQU, 2);
    DOPI_AppendSDO(&sdo_unit, &sdo_msg);

//   SetRoutine(&t_obj->data.msg_hdlr_task, ROUTINE_ID_MSG_PDO_SEND);
//    sdo_unit = DOPI_CreateSDOUnit(&t_obj->d10_obj, TASK_ID_MSG, SDO_ID_MSG_SET_ROUTINE, SDO_REQU, 1);
//    DOPI_AppendSDO(&sdo_unit, &sdo_msg);

//    t_obj->data.msg_hdlr_task.synch_onoff = 1;
//    sdo_unit = DOPI_CreateSDOUnit(&t_obj->d10_obj, TASK_ID_MSG, SDO_ID_MSG_SYNCH_MODE_ONOFF, SDO_REQU, 1);
//    DOPI_AppendSDO(&sdo_unit, &sdo_msg);

	t_obj->data.msg_hdlr_task.state = TASK_STATE_STANDBY;
	sdo_unit = DOPI_CreateSDOUnit(&t_obj->d10_obj, TASK_ID_MSG, SDO_ID_MSG_SET_STATE, SDO_REQU, 1);
	DOPI_AppendSDO(&sdo_unit, &sdo_msg);

    t_obj->data.low_level_ctrl_task.state = TASK_STATE_STANDBY;
    sdo_unit = DOPI_CreateSDOUnit(&t_obj->d10_obj, TASK_ID_LOWLEVEL, SDO_ID_LOWLEVEL_SET_STATE, SDO_REQU, 1);
    DOPI_AppendSDO(&sdo_unit, &sdo_msg);

    t_obj->data.mid_level_ctrl_task.state = TASK_STATE_STANDBY;
    sdo_unit = DOPI_CreateSDOUnit(&t_obj->d10_obj, TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_SET_STATE, SDO_REQU, 1);
    DOPI_AppendSDO(&sdo_unit, &sdo_msg);

    TxSDO(t_obj);
    DOPI_ClearSDO(&sdo_msg);

    t_obj->data.msg_hdlr_task.state = TASK_STATE_ENABLE;
    sdo_unit = DOPI_CreateSDOUnit(&t_obj->d10_obj, TASK_ID_MSG, SDO_ID_MSG_SET_STATE, SDO_REQU, 1);
    DOPI_AppendSDO(&sdo_unit, &sdo_msg);

//    if(abs_enc_init==0){
//		t_obj->data.abs_offset = 0;
//		sdo_unit = DOPI_CreateSDOUnit(&t_obj->d10_obj, TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_ABS_ENCODER_OFFSET, SDO_REQU, 1);
//		DOPI_AppendSDO(&sdo_unit, &sdo_msg);
//		t_obj->data.abs_orign = 1;
//		sdo_unit = DOPI_CreateSDOUnit(&t_obj->d10_obj, TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_SET_ORIGIN, SDO_REQU, 1);
//		DOPI_AppendSDO(&sdo_unit, &sdo_msg);
//    }


    TxSDO(t_obj);
    DOPI_ClearSDO(&sdo_msg);

//    SetRoutine(&t_obj->data.ext_dev_ctrl_task, ROUTINE_ID_EXTDEV_DC_BUTTON_CMD);
//    sdo_unit = DOPI_CreateSDOUnit(&t_obj->d10_obj, TASK_ID_EXTDEV, SDO_ID_EXTDEV_SET_ROUTINE, SDO_REQU, t_obj->data.ext_dev_ctrl_task.n_routines);
//    DOPI_AppendSDO(&sdo_unit, &sdo_msg);


    t_obj->data.mid_level_ctrl_task.state = TASK_STATE_ENABLE;
    sdo_unit = DOPI_CreateSDOUnit(&t_obj->d10_obj, TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_SET_STATE, SDO_REQU, 1);
    DOPI_AppendSDO(&sdo_unit, &sdo_msg);

    /* PDO REQUEST */
    memcpy(t_obj->data.pdo_list, t_pdo_list, sizeof(t_pdo_list));
    sdo_unit = DOPI_CreateSDOUnit(&t_obj->d10_obj, TASK_ID_MSG, SDO_ID_MSG_PDO_LIST, SDO_REQU, sizeof(t_pdo_list)/2);
    DOPI_AppendSDO(&sdo_unit, &sdo_msg);

    TxSDO(t_obj);
}


void Enable_L30_Position_Ctrl(L30Struct* t_obj)
{
    DOPI_SDOUnit_t sdo_unit;
    DOPI_ClearSDO(&sdo_msg);

    /* STATE ENABLE */

    t_obj->data.low_level_ctrl_task.state = TASK_STATE_ENABLE;
    sdo_unit = DOPI_CreateSDOUnit(&t_obj->d10_obj, TASK_ID_LOWLEVEL, SDO_ID_LOWLEVEL_SET_STATE, SDO_REQU, 1);
    DOPI_AppendSDO(&sdo_unit, &sdo_msg);

    TxSDO(t_obj);
}

//void Test_Led(L30Struct* t_obj)
//{
//    DOPI_SDOUnit_t sdo_unit;
//    DOPI_ClearSDO(&sdo_msg);
//
//    /* STATE ENABLE */
//
//    t_obj->data.low_level_ctrl_task.state = TASK_STATE_ENABLE;
//    sdo_unit = DOPI_CreateSDOUnit(&t_obj->d10_obj, TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_TEST_LED, SDO_REQU, 1);
//    DOPI_AppendSDO(&sdo_unit, &sdo_msg);
//
//    TxSDO(t_obj);
//}

void Disable_L30_Position_Ctrl(L30Struct* t_obj)
{
    DOPI_SDOUnit_t sdo_unit;

    DOPI_ClearSDO(&sdo_msg);

//    ClearRoutine(&t_obj->data.mid_level_ctrl_task);
    ClearRoutine(&t_obj->data.low_level_ctrl_task);

//    t_obj->data.msg_hdlr_task.synch_onoff = 0;
//    sdo_unit = DOPI_CreateSDOUnit(&t_obj->d10_obj, TASK_ID_MSG, SDO_ID_MSG_SYNCH_MODE_ONOFF, SDO_REQU, 1);
//    DOPI_AppendSDO(&sdo_unit, &sdo_msg);

    t_obj->data.low_level_ctrl_task.state = TASK_STATE_STANDBY;
    sdo_unit = DOPI_CreateSDOUnit(&t_obj->d10_obj, TASK_ID_LOWLEVEL, SDO_ID_LOWLEVEL_SET_STATE, SDO_REQU, 1);
    DOPI_AppendSDO(&sdo_unit, &sdo_msg);

//    t_obj->data.mid_level_ctrl_task.state = TASK_STATE_STANDBY;
//    sdo_unit = DOPI_CreateSDOUnit(&t_obj->d10_obj, TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_SET_STATE, SDO_REQU, 1);
//    DOPI_AppendSDO(&sdo_unit, &sdo_msg);

    TxSDO(t_obj);
}

void Send_L30_Position_Ref(L30Struct* t_obj, float t_ref)
{
    DOPI_PDOUnit_t pdo_unit;
    DOPI_ClearPDO(&pdo_msg);
	if(t_ref>8){
		t_ref=8;
	}else if(t_ref<-8){
		t_ref=-8;
	}
	t_obj->data.target_current=t_ref;

    pdo_unit = DOPI_CreatePDOUnit(TASK_ID_LOWLEVEL, PDO_ID_LOWLEVEL_TOTAL_CURRENT_INPUT, (void*)(&t_ref));
//    pdo_unit = DOPI_CreatePDOUnit(TASK_ID_MIDLEVEL, PDO_ID_MIDLEVEL_REF_POSITION, (void*)(&t_ref));

    DOPI_AppendPDO(&pdo_unit, &pdo_msg);

    TxPDO(t_obj);
}

//static void StartPDOStreaming(int obj_idx)
//{
//    DOPI_SDOUnit_t sdo_unit;
//    BumbleBee_ClearSDO(&sdo_msg);
//
//    sdo_unit = BumbleBee_CreateSDOUnit(&d10Obj[obj_idx].d10_obj, TASK_ID_COMM, SDO_ID_COMM_PDO_LIST,  SDO_REQU, testListPDO_size);
//    BumbleBee_AppendSDO(&sdo_unit, &sdo_msg);
//
//    TxSDO(obj_idx);
//}

void Set_dc_set_length(L30Struct* t_obj, float t_data)
{
	static int size_data;
//	L30Struct* t_obj;
//
//	t_obj=*t_ptr;

	size_data = sizeof(L30_MD_Data);
    DOPI_SDOUnit_t sdo_unit;
    DOPI_ClearSDO(&sdo_msg);
    *test_addr = &t_obj->data.upright_length_command;
    t_obj->data.upright_length_command = t_data;
    sdo_unit = DOPI_CreateSDOUnit(&t_obj->d10_obj, TASK_ID_EXTDEV, SDO_ID_EXTDEV_DC_SET_LENGTH, SDO_REQU, 1);
    DOPI_AppendSDO(&sdo_unit, &sdo_msg);
    TxSDO(t_obj);
}


/* ------------------- PIF Vector (WalkONSuit5) ------------------- */
// function description: send P-vector to MD
// param1: MD ID
// param2: desired position (unit: deg)
// param3: duration         (unit: ms)
// param4: normalized acceleration (unit: deg/s^2)
// param5: normalized deceleration (unit: deg/s^2)
void Send_P_Vector(int obj_idx, float yd, uint16_t L, uint8_t s0, uint8_t sd)
{
    DOPI_SDOUnit_t sdo_unit;
    DOPI_ClearSDO(&sdo_msg);

    d10Obj[obj_idx].data.p_vector.yd = 8.726646259971647 * yd;  // = 8.726646259971647 = M_PI*500 /180; (tx: x500, rx: x0.002)

    sdo_unit = DOPI_CreateSDOUnit(&d10Obj[obj_idx].d10_obj, TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_P_VECTOR_YD, SDO_REQU, 1);
    DOPI_AppendSDO(&sdo_unit, &sdo_msg);

    d10Obj[obj_idx].data.p_vector.L = L;
    sdo_unit = DOPI_CreateSDOUnit(&d10Obj[obj_idx].d10_obj, TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_P_VECTOR_L, SDO_REQU, 1);
    DOPI_AppendSDO(&sdo_unit, &sdo_msg);

    d10Obj[obj_idx].data.p_vector.s0 = s0;
    sdo_unit = DOPI_CreateSDOUnit(&d10Obj[obj_idx].d10_obj, TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_P_VECTOR_S0, SDO_REQU, 1);
    DOPI_AppendSDO(&sdo_unit, &sdo_msg);

    d10Obj[obj_idx].data.p_vector.sd = sd;
    sdo_unit = DOPI_CreateSDOUnit(&d10Obj[obj_idx].d10_obj, TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_P_VECTOR_SD, SDO_REQU, 1);
    DOPI_AppendSDO(&sdo_unit, &sdo_msg);

    TxSDO(&d10Obj[obj_idx]);
}

// function description: send F-vector to MD
// param1: MD ID
// param2: torque mode idx (unit: -)
// param3: maximum torque  (unit: A)
// param4: delay           (unit: ms)
void Send_F_Vector(int obj_idx, uint8_t mode_idx, float tau_max, uint16_t delay)
{
    DOPI_SDOUnit_t sdo_unit;
    DOPI_ClearSDO(&sdo_msg);

    d10Obj[obj_idx].data.f_vector.mode_idx = mode_idx;

    sdo_unit = DOPI_CreateSDOUnit(&d10Obj[obj_idx].d10_obj, TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_F_VECTOR_MODE_IDX, SDO_REQU, 1);
    DOPI_AppendSDO(&sdo_unit, &sdo_msg);

    d10Obj[obj_idx].data.f_vector.tau_max = (int16_t)(tau_max*100);
    sdo_unit = DOPI_CreateSDOUnit(&d10Obj[obj_idx].d10_obj, TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_F_VECTOR_TMAX, SDO_REQU, 1);
    DOPI_AppendSDO(&sdo_unit, &sdo_msg);

    d10Obj[obj_idx].data.f_vector.delay = delay;
    sdo_unit = DOPI_CreateSDOUnit(&d10Obj[obj_idx].d10_obj, TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_F_VECTOR_DELAY, SDO_REQU, 1);
    DOPI_AppendSDO(&sdo_unit, &sdo_msg);

    TxSDO(&d10Obj[obj_idx]);
}

// function description: send impedance controller setting to MD
// param1: MD ID
// param2: Half width of the Corridor      (unit: deg)
// param3: Magnitude of the Virtual Spring (unit: %)
// param4: Magnitude of the Virtual Damper (unit: %)
// param5: Impedance Ratio in the Corridor (0~2)
// param6: Duration for Transition         (unit: ms for dT = 1ms)
void Send_I_Vector(int obj_idx, float t_epsilon, float t_Kp, float t_Kd, float t_lambda, uint16_t duration)
{
    DOPI_SDOUnit_t sdo_unit;
    DOPI_ClearSDO(&sdo_msg);

    d10Obj[obj_idx].data.i_vector.epsilon_target = (uint8_t)(10 * t_epsilon);
    sdo_unit = DOPI_CreateSDOUnit(&d10Obj[obj_idx].d10_obj, TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_I_VECTOR_EPSILON, SDO_REQU, 1);
    DOPI_AppendSDO(&sdo_unit, &sdo_msg);

    d10Obj[obj_idx].data.i_vector.Kp_target = (uint8_t)(2.55 * t_Kp);
    sdo_unit = DOPI_CreateSDOUnit(&d10Obj[obj_idx].d10_obj, TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_I_VECTOR_KP, SDO_REQU, 1);
    DOPI_AppendSDO(&sdo_unit, &sdo_msg);

    d10Obj[obj_idx].data.i_vector.Kd_target = (uint8_t)(2.55 * t_Kd);
    sdo_unit = DOPI_CreateSDOUnit(&d10Obj[obj_idx].d10_obj, TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_I_VECTOR_KD, SDO_REQU, 1);
    DOPI_AppendSDO(&sdo_unit, &sdo_msg);

    d10Obj[obj_idx].data.i_vector.lambda_target = (uint8_t)(100 * t_lambda);
    sdo_unit = DOPI_CreateSDOUnit(&d10Obj[obj_idx].d10_obj, TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_I_VECTOR_LAMBDA, SDO_REQU, 1);
    DOPI_AppendSDO(&sdo_unit, &sdo_msg);

    d10Obj[obj_idx].data.i_vector.duration = duration;
    sdo_unit = DOPI_CreateSDOUnit(&d10Obj[obj_idx].d10_obj, TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_I_VECTOR_DURATION, SDO_REQU, 1);
    DOPI_AppendSDO(&sdo_unit, &sdo_msg);

    TxSDO(&d10Obj[obj_idx]);
}

void Set_I_Vector_Kp_Max(int obj_idx, float Kp_max)
{
    DOPI_SDOUnit_t sdo_unit;
    DOPI_ClearSDO(&sdo_msg);

    d10Obj[obj_idx].data.i_vector_Kp_max = Kp_max;
    sdo_unit = DOPI_CreateSDOUnit(&d10Obj[obj_idx].d10_obj, TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_I_VECTOR_KP_MAX, SDO_REQU, 1);
    DOPI_AppendSDO(&sdo_unit, &sdo_msg);
    TxSDO(&d10Obj[obj_idx]);
}

void Set_I_Vector_Kd_Max(int obj_idx, float Kd_max)
{
    DOPI_SDOUnit_t sdo_unit;
    DOPI_ClearSDO(&sdo_msg);

    d10Obj[obj_idx].data.i_vector_Kd_max = Kd_max;
    sdo_unit = DOPI_CreateSDOUnit(&d10Obj[obj_idx].d10_obj, TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_I_VECTOR_KD_MAX, SDO_REQU, 1);
    DOPI_AppendSDO(&sdo_unit, &sdo_msg);
    TxSDO(&d10Obj[obj_idx]);
}


/**
 *------------------------------------------------------------
 *                      STATIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions intended for internal use within this module.
 */

/* IO */
static int TxSDO(L30Struct* t_obj)
{
	int t_res;
    if(t_obj->tx_fnc != NULL)
		t_res = t_obj->tx_fnc(t_obj->sdo_tx_id, sdo_msg.txBuf, sdo_msg.msgLength);
	else
		t_res = 0;		//error return

    return t_res;
}

static int TxPDO(L30Struct* t_obj)
{
    return t_obj->tx_fnc(t_obj->pdo_tx_id, pdo_msg.txBuf, pdo_msg.msgLength);
}

/* Functionalities */
static void SetRoutine(L30_MD_TaskData* task_obj, uint8_t routine_id)
{
    uint8_t idx = task_obj->n_routines;

    if (idx > 0) {
        for (int i = 0; i < idx; ++i) { // Check if the routine is already set
            if (task_obj->routines[i] == routine_id) {
                return;
            }
        }
    }

    if (idx < L30_MAX_ROUTINES) {
        task_obj->routines[idx] = routine_id;
        ++task_obj->n_routines;
    }
}

static void ClearRoutine(L30_MD_TaskData* task_obj)
{
    memset(&task_obj->routines, 0, sizeof(task_obj->routines[0])*L30_MAX_ROUTINES);
    task_obj->n_routines = 0;
}

static void UnsetRoutine(L30_MD_TaskData* task_obj, uint8_t routine_id)
{
    uint8_t n = task_obj->n_routines;

    if (n == 0) {
        return;
    }

    uint8_t tmp[n];
    memcpy(tmp, task_obj->routines, n);
    ClearRoutine(task_obj);

    int cursor = 0;
    for (int i = 0; i < n; ++i) {
        if (tmp[i] == routine_id) {
            task_obj->n_routines = n-1;
        } else {
            task_obj->routines[cursor++] = tmp[i];
        }
    }
}

static void Setup_L30_DOD(L30Struct* obj)
{
    /* Common */
    // SDO
	DOPI_SetSDOAddr(&obj->d10_obj, TASK_ID_MSG, SDO_ID_MSG_SET_STATE,   &obj->data.msg_hdlr_task.state);
	DOPI_SetSDOAddr(&obj->d10_obj, TASK_ID_MSG, SDO_ID_MSG_SET_ROUTINE, obj->data.msg_hdlr_task.routines);
    DOPI_SetSDOAddr(&obj->d10_obj, TASK_ID_MSG, SDO_ID_MSG_PDO_LIST,    obj->data.pdo_list);
    DOPI_SetSDOAddr(&obj->d10_obj, TASK_ID_MSG, SDO_ID_MSG_SYNCH_MODE_ONOFF,    &obj->data.msg_hdlr_task.synch_onoff);


    /* BLDC */
    // SDO
	DOPI_SetSDOAddr(&obj->d10_obj, TASK_ID_LOWLEVEL, SDO_ID_LOWLEVEL_SET_STATE,      &obj->data.low_level_ctrl_task.state);
	DOPI_SetSDOAddr(&obj->d10_obj, TASK_ID_LOWLEVEL, SDO_ID_LOWLEVEL_SET_ROUTINE,    obj->data.low_level_ctrl_task.routines);
    // PDO
    DOPI_SetPDOAddr(&obj->d10_obj, TASK_ID_LOWLEVEL, PDO_ID_LOWLEVEL_CURRENT_OUTPUT, &obj->data.actual_current);


    /* Joint */
    // SDO
    DOPI_SetSDOAddr(&obj->d10_obj, TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_SET_STATE,        &obj->data.mid_level_ctrl_task.state);
    DOPI_SetSDOAddr(&obj->d10_obj, TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_SET_ROUTINE,      obj->data.mid_level_ctrl_task.routines);
    // DOPI_SetSDOAddr(&obj->d10_obj, TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_POSITION_PERIODIC_SIG_FREQ,  &obj->data.target_sine_freq);
    // DOPI_SetSDOAddr(&obj->d10_obj, TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_POSITION_PERIODIC_SIG_AMP,  &obj->data.target_sine_amp);
    // DOPI_SetSDOAddr(&obj->d10_obj, TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_ABS_ENCODER_OFFSET,  &obj->data.abs_offset);
    // DOPI_SetSDOAddr(&obj->d10_obj, TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_SET_ORIGIN,    &obj->data.abs_orign);
    // PDO

    DOPI_SetPDOAddr(&obj->d10_obj, TASK_ID_MIDLEVEL, PDO_ID_MIDLEVEL_ACTUAL_POSITION,   		&obj->data.actual_position);
    // DOPI_SetPDOAddr(&obj->d10_obj, TASK_ID_MIDLEVEL, PDO_ID_MIDLEVEL_ABS_360, 			&obj->data.actual_velocity);
    //DOPI_SetPDOAddr(&obj->d10_obj, TASK_ID_MIDLEVEL, PDO_ID_MIDLEVEL_ABS_POSITION,   		&obj->data.abs_position);
    // DOPI_SetPDOAddr(&obj->d10_obj, TASK_ID_MIDLEVEL, PDO_ID_MIDLEVEL_TIME_INDEX_SEND, 	&obj->data.time_send);
    // DOPI_SetPDOAddr(&obj->d10_obj, TASK_ID_MIDLEVEL, PDO_ID_MIDLEVEL_TIME_INDEX_RECEIVE, &obj->data.time_receive);
    // DOPI_SetPDOAddr(&obj->d10_obj, TASK_ID_MIDLEVEL, PDO_ID_MIDLEVEL_ACTUAL_POSITION, &obj->data.inc_encoder_cnt);
    if (obj->is_hip == 0) {
//    	DOPI_SetPDOAddr(&obj->d10_obj, TASK_ID_MIDLEVEL, PDO_ID_MIDLEVEL_ABS_ANKLE_DEGREE, &obj->data.ankle_encoder);
    }

    /* Core */
    DOPI_SetPDOAddr(&obj->d10_obj, TASK_ID_SYSMNGT, PDO_ID_SYSTEM_VOLT,		&obj->data.PS_voltage);
    DOPI_SetPDOAddr(&obj->d10_obj, TASK_ID_SYSMNGT, PDO_ID_SYSTEM_CURR,     &obj->data.PS_current);
    DOPI_SetPDOAddr(&obj->d10_obj, TASK_ID_SYSMNGT, PDO_ID_SYSTEM_TEMP,    	&obj->data.PS_temperature);

    /* Exts */
    // SDO
    DOPI_SetSDOAddr(&obj->d10_obj, TASK_ID_EXTDEV, SDO_ID_EXTDEV_SET_STATE,     &obj->data.ext_dev_ctrl_task.state);
    DOPI_SetSDOAddr(&obj->d10_obj, TASK_ID_EXTDEV, SDO_ID_EXTDEV_SET_ROUTINE,   obj->data.ext_dev_ctrl_task.routines);
    DOPI_SetSDOAddr(&obj->d10_obj, TASK_ID_EXTDEV, SDO_ID_EXTDEV_DC_SET_LENGTH, &obj->data.upright_length_command);
    DOPI_SetSDOAddr(&obj->d10_obj, TASK_ID_EXTDEV, SDO_ID_EXTDEV_DC_SET_DIRECT, &obj->data.upright_dir_command);
    // PDO
    DOPI_SetPDOAddr(&obj->d10_obj, TASK_ID_EXTDEV, PDO_ID_EXTDEV_DC_LENGTH_ACT,		&obj->data.upright_length_actual);
    DOPI_SetPDOAddr(&obj->d10_obj, TASK_ID_EXTDEV, PDO_ID_EXTDEV_DC_DIRECTION_ACT,     &obj->data.upright_dir_actual);
    DOPI_SetPDOAddr(&obj->d10_obj, TASK_ID_EXTDEV, PDO_ID_EXTDEV_DC_BUTTON_STATE,    	&obj->data.upright_bt_state);
    DOPI_SetPDOAddr(&obj->d10_obj, TASK_ID_EXTDEV, PDO_ID_EXTDEV_NTC_MOTOR_TEMP,    	&obj->data.motor_temp);

   // DOPI_SetPDOAddr(&obj->d10_obj, TASK_ID_EXTDEV, PDO_ID_EXTDEV_DC_LENGTH_REF,		&obj->data.upright_length_command);

    if (obj->is_hip == 0) {
    	DOPI_SetPDOAddr(&obj->d10_obj, TASK_ID_EXTDEV, PDO_ID_EXTDEV_FSR, &obj->data.FSR);
    	DOPI_SetPDOAddr(&obj->d10_obj, TASK_ID_EXTDEV, PDO_ID_EXTDEV_LP,  &obj->data.linear_potentiometer);
    }
}

/* RxCallback */
static int D10_Mngr_FDCAN_Callback(uint16_t wasp_id, uint8_t* rx_data)
{
    uint16_t t_fnc_code = wasp_id & 0xF00;
    uint16_t t_ori_node = (wasp_id & 0x0F0)>>4;
    memcpy(&test_monitor, rx_data, 64);

    int t_obj_idx = 0;
    switch (t_ori_node) {
        case NODE_ID_LH_SAG: t_obj_idx = e_L30_LH_IDX; break;
//        case NODE_ID_LK: t_obj_idx = e_L30_LK_IDX;  	Toggle_GPIOG_Sate(TX_IT_Pin);	break;
        case NODE_ID_LK: t_obj_idx = e_L30_LK_IDX; break;
        case NODE_ID_RH_SAG: t_obj_idx = e_L30_RH_IDX; break;
        case NODE_ID_RK: t_obj_idx = e_L30_RK_IDX; break;
        default: return -1;
    }

    switch(t_fnc_code){
        case EMCY: DOPI_UnpackEMCY(&d10Ptr[t_obj_idx]->data.err_code, rx_data); break;
//    	case EMCY: ; break;
    	case SDO: DOPI_UnpackSDO(&d10Ptr[t_obj_idx]->d10_obj, rx_data); break;
        case PDO: DOPI_UnpackPDO(&d10Ptr[t_obj_idx]->d10_obj, rx_data); break;
        default: break;
    }

    return 0;
}

static void Init_L30_Instance(int obj_idx)
{
	static uint8_t node_id = 0;
	static uint8_t is_hip = 0;
	L30_FDCAN_TxFncPtr txfnc;

	switch (obj_idx) {
		case e_L30_LH_IDX: node_id = NODE_ID_LH_SAG; is_hip = 1; txfnc = IOIF_TransmitFDCAN2; break;
		case e_L30_RH_IDX: node_id = NODE_ID_RH_SAG; is_hip = 1; txfnc = IOIF_TransmitFDCAN1; break;
		case e_L30_LK_IDX: node_id = NODE_ID_LK; is_hip = 0; txfnc = IOIF_TransmitFDCAN2; break;
		case e_L30_RK_IDX: node_id = NODE_ID_RK; is_hip = 0; txfnc = IOIF_TransmitFDCAN1; break;
//	case e_L30_LH_IDX: node_id = NODE_ID_LH; is_hip = 1; txfnc = IOIF_TransmitFDCAN; break;
//	case e_L30_LK_IDX: node_id = NODE_ID_RH; is_hip = 0; txfnc = IOIF_TransmitFDCAN; break;
//	case e_L30_RH_IDX: node_id = NODE_ID_LK; is_hip = 1; txfnc = IOIF_TransmitFDCAN; break;
//	case e_L30_RK_IDX: node_id = NODE_ID_RK; is_hip = 0; txfnc = IOIF_TransmitFDCAN; break;
		default:
	return;
	}

	d10Ptr[obj_idx]->sdo_tx_id = SDO | (NODE_ID_CM << 4) | node_id;
	d10Ptr[obj_idx]->pdo_tx_id = PDO | (NODE_ID_CM << 4) | node_id;
	d10Ptr[obj_idx]->tx_fnc = txfnc;
	d10Ptr[obj_idx]->is_hip = is_hip;

	memset(&d10Ptr[obj_idx]->data, 0, sizeof(d10Ptr[obj_idx]->data));

    DOPI_InitDevObj(&d10Ptr[obj_idx]->d10_obj, node_id);
    Setup_L30_DOD(d10Ptr[obj_idx]);
}

#endif /* L30_CM_ENABLED */
