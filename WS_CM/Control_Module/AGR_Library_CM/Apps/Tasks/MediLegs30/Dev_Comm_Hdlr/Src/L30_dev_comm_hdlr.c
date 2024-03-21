

#include "L30_dev_comm_hdlr.h"

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

TaskObj_t devCommHdlr;
MainSequence_Enum MS_enum;


/**
 *------------------------------------------------------------
 *                      STATIC VARIABLES
 *------------------------------------------------------------
 * @brief Variables local to this module.
 */

static float msg_test[10];
static int spi_cw;
static int divider;
static int spi_sw;
static uint8_t spi_success_case = 0;
static uint8_t debug_AM_PC_select = 0; //0 : AM , 1: PC

static uint32_t devCommHdlrLoopCnt;

uint16_t commu_num;
COMMType comm_type;
uint32_t MSG_time_elapsed;
uint8_t GUI_onoff;
uint8_t GUI_command;

cvector_vector_type(DOP_Header_t) pdo_send_list;
cvector_vector_type(DOP_Header_t) sdo_req_list;
cvector_vector_type(DOP_Header_t) sdo_res_list;

float test1;
float test2;
uint8_t test_odd;

uint32_t traj_idx;

uint8_t node_id;
uint8_t ori_node;
uint32_t fnc_code;
uint32_t err_code;
uint8_t fdcanTxData[64];
uint8_t fdcanRxData[64];
uint8_t usbRxData[64];
int test_routine; //test

static uint8_t spi_rx_complete_flag;

static float d10_lengthact;
/**
 *------------------------------------------------------------
 *                 STATIC FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Static Function prototypes for this module.
 */

/* ------------------- STATE FUNCTION ------------------- */
static void StateOff_Ent(void);
static void StateStandby_Ent(void);
static void StateStandby_Run();
static void StateStandby_Ext();
static void StateEnable_Ent(void);
static void StateEnable_Run(void);
static void StateEnable_Ext(void);
static void StateError_Run(void);

/* ------------------- ROUTINE ------------------- */
static DOP_Header_t Get_Header(uint8_t* t_byte_arr);
static void SendTimeOut(void);
static void SendRxSuccess(void);

static void Send_curr_State_stand(void);
static void Send_curr_State_sit(void);
static void init_buffef(void);

/* ------------------- EMCY ------------------- */
static void Recv_EMCY(uint8_t* t_byte_arr, uint32_t* t_err_code);

/* ------------------- SDO RX ------------------- */
static DOP_SDOArgs_t Convert_Bytes_to_SDO_req(uint8_t* t_byte_arr, uint16_t* t_byte_len);
static int Read_SDO(uint8_t* t_byte_arr);
static int Unpack_SDO(uint8_t* t_byte_arr);

/* ------------------- SDO TX ------------------- */
static int Convert_SDOres_to_Bytes(DOP_Header_t* t_header, uint8_t* t_byte_arr);
static int Pack_SDO(uint8_t* t_byte_arr, uint8_t* t_byte_len);
static int Send_SDO(uint8_t t_dest_node);
//static int routine_test(uint8_t t_dest_node);

/* ------------------- PDO RX ------------------- */
static int Convert_Bytes_to_PDO(uint8_t* t_byte_arr);
static int Unpack_PDO(uint8_t* t_byte_arr);

/* ------------------- PDO TX ------------------- */
static int Convert_PDO_to_Bytes(DOP_Header_t* t_header, uint8_t* t_byte_arr);
static int Pack_PDO(uint8_t* t_byte_arr, uint8_t* t_byte_len);
static int Send_PDO(void);

/* ------------------- LIST MANAGEMENT ------------------- */
static void Add_PDO_to_Send(uint8_t t_dictID, uint8_t t_objID);
static void Clear_PDO_to_Send(void);

/* ------------------- MESSAGE HANDLER ------------------- */
static int USB_Rx_Hdlr(uint8_t* t_Buf, uint32_t* t_Len);
static int Fdcan_Rx_Hdlr(uint16_t t_wasp_id, uint8_t* t_rx_data);
static int request_send_PDO(void);

/* ------------------- SDO CALLBACK ------------------- */
static void Set_Send_PDO_List(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);
static void Set_MS_Enum(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);
static void Set_GUI_COMM_OnOff(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);
static void Set_GUI_COMM_Command(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);
static void DETECT_ACTIVE_MD(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);
static void DETECT_receive_ID(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);
static void Set_D10_UprightLength_Command(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);
//static void Send_AM_D10_UprightLength_Act(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);

static void MD_PDO_Enable();
static void MD_send_traj();

/* ------------------- SPI CALLBACK ------------------- */
static void SpiTxRxCB(void* params);


/**
 *------------------------------------------------------------
 *                      PUBLIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions that interface with this module.
 */

DOP_COMMON_SDO_CB(devCommHdlr)

// Main
void InitDevCommHdlr(void)
{
    InitTask(&devCommHdlr);

    DOPC_AssignTaskID(&devCommHdlr, TASK_IDX_DEV_COMM_HDLR);

    init_buffef();

    // TODO : SPI 3 TXRX Completed Callback!
    IOIF_SetSPICB(IOIF_SPI3, IOIF_SPI_TXRX_CPLT_CALLBACK, SpiTxRxCB, NULL);

    traj_idx = 0;
    test1 = 0;
    test2 = 0;
    test_odd = 0;
    ori_node = 0x00;
    node_id = 1;
    commu_num = 0;
    comm_type = 1;
    spi_cw = 0;
    divider = 10;

    for (int i = 0; i < 256; i++) {
    	spi3DmaTxBuff[i] = (uint8_t)(i % 256);
    	spi3DmaRxBuff[i] = (uint8_t)(i % 256);
    }

    /* State Definition */
    TASK_CREATE_STATE(&devCommHdlr, TASK_STATE_OFF,     StateOff_Ent,     NULL,              NULL, 			    false);
    TASK_CREATE_STATE(&devCommHdlr, TASK_STATE_STANDBY, StateStandby_Ent, StateStandby_Run,  StateStandby_Ext,   true);
    TASK_CREATE_STATE(&devCommHdlr, TASK_STATE_ENABLE,  StateEnable_Ent,  StateEnable_Run,   StateEnable_Ext,    false);
    TASK_CREATE_STATE(&devCommHdlr, TASK_STATE_ERROR,   NULL,             StateError_Run,    NULL, 			    false);

	/* Routine Definition */
    // TODO : Don't use 0, 1 !!
    TASK_CREATE_ROUTINE(&devCommHdlr, 0, NULL, Send_PDO, NULL);
//    TASK_CREATE_ROUTINE(&devCommHdlr, 1, NULL, routine_test, NULL);

    /* DOD Definition */
	// DOD
    DOP_CreateDOD(TASK_IDX_DEV_COMM_HDLR);

    // PDO
    DOP_COMMON_PDO_CREATE(TASK_IDX_DEV_COMM_HDLR, devCommHdlr);
    DOP_CreatePDO(TASK_IDX_DEV_COMM_HDLR, PDO_ID_MSG_TEST1,   DOP_FLOAT32, 1, &msg_test[0]);
    DOP_CreatePDO(TASK_IDX_DEV_COMM_HDLR, PDO_ID_MSG_TEST2,   DOP_FLOAT32, 1, &msg_test[1]);
    DOP_CreatePDO(TASK_IDX_DEV_COMM_HDLR, PDO_ID_MSG_TEST3,   DOP_FLOAT32, 1, &msg_test[2]);
    DOP_CreatePDO(TASK_IDX_DEV_COMM_HDLR, PDO_ID_MSG_TEST4,   DOP_FLOAT32, 1, &msg_test[3]);
    DOP_CreatePDO(TASK_IDX_DEV_COMM_HDLR, PDO_ID_MSG_TEST5,   DOP_FLOAT32, 1, &msg_test[4]);
    DOP_CreatePDO(TASK_IDX_DEV_COMM_HDLR, PDO_ID_MSG_TEST6,   DOP_FLOAT32, 1, &msg_test[5]);
    DOP_CreatePDO(TASK_IDX_DEV_COMM_HDLR, PDO_ID_MSG_TEST7,   DOP_FLOAT32, 1, &msg_test[6]);
    DOP_CreatePDO(TASK_IDX_DEV_COMM_HDLR, PDO_ID_MSG_TEST8,   DOP_FLOAT32, 1, &msg_test[7]);
    DOP_CreatePDO(TASK_IDX_DEV_COMM_HDLR, PDO_ID_MSG_TEST9,   DOP_FLOAT32, 1, &msg_test[8]);
    DOP_CreatePDO(TASK_IDX_DEV_COMM_HDLR, PDO_ID_MSG_TEST10,  DOP_FLOAT32, 1, &msg_test[9]);


    // SDO
    // TODO : Don't use 5 !!
    DOP_COMMON_SDO_CREATE(TASK_IDX_DEV_COMM_HDLR);
    DOP_CreateSDO(TASK_IDX_DEV_COMM_HDLR, SDO_ID_MSG_PDO_LIST,                 DOP_UINT16,    Set_Send_PDO_List);
    DOP_CreateSDO(TASK_IDX_DEV_COMM_HDLR, SDO_ID_MSG_GUI_QT_DETECT,            DOP_UINT8,     DETECT_ACTIVE_MD);
    DOP_CreateSDO(TASK_IDX_DEV_COMM_HDLR, SDO_ID_MSG_GUI_QT_ID_RECEIVE,        DOP_UINT8,     DETECT_receive_ID);
    DOP_CreateSDO(TASK_IDX_DEV_COMM_HDLR, SDO_ID_MSG_MD_PDO_ENABLE,            DOP_UINT8,     MD_PDO_Enable);
    DOP_CreateSDO(TASK_IDX_DEV_COMM_HDLR, SDO_ID_MSG_MD_DC_LENGTH_COMMAND,     DOP_FLOAT32,   Set_D10_UprightLength_Command);

    // Clear_PDO_to_Send();
    // Add_PDO_to_Send(6, 2);

    // TODO : Don't use 0 !!
//    Push_Routine(&devCommHdlr.routine, 0);

    // TODO : USB Rx Callback Refactoring !!
//    ioif_usbRxCBPtr = USB_Rx_Hdlr;
    // fdcan1_rx_callback_ptr = Fdcan_Rx_Hdlr;
    // IOIF_FDCAN1_SetRxCallback(Fdcan_Rx_Hdlr);
}

void RunDevCommHdlr(void)
{
    RunTask(&devCommHdlr);
}

/* ------------------- EMCY ------------------- */
void Send_EMCY(uint32_t* t_err_code)
{
    uint8_t* t_tx_data = malloc(sizeof(uint8_t) * 4);
    uint16_t t_identifier = EMCY | (node_id << 4);

    memcpy(t_tx_data, t_err_code, DOP_ERR_CODE_SIZE);

    if (Send_MSG(t_identifier, 4, t_tx_data) != 0)
    {
        // TODO: MSG TX ERROR
    }

    free(t_tx_data);
    t_tx_data = NULL;
}

/* ------------------- MESSAGE HANDLER ------------------- */
int Send_MSG(uint16_t t_COB_ID, uint32_t t_len, uint8_t* t_tx_data)
{
    int t_check = 0;
    uint8_t t_txBuf[67];

    uint8_t t_fnc_code = (t_COB_ID & 0xF00) >> 8;
    memcpy(t_txBuf, &t_fnc_code, 1);

    uint8_t t_node_id = (t_COB_ID & 0xFF);
    memcpy(&t_txBuf[1], &t_node_id, 1);
    memcpy(&t_txBuf[2], t_tx_data, t_len);

    t_txBuf[t_len + 2] = '\r';
    t_txBuf[t_len + 3] = '\n';

//    	if(comm_type == e_FDCAN){
//    		if(Transmit_IOIF_Fdcan(t_COB_ID, Convert_Byte_Length(t_len), t_tx_data) != 0){
//    				return t_check;
//    				//TODO: MSG TX ERROR
//    		}
//    	} else if(comm_type == e_USB){
    if (CDC_Transmit_FS(t_txBuf, t_len + 4) != 0)
    {
        return t_check;
        // TODO: MSG TX ERROR
    }
    //	}

    return -1;
}

int Send_MSG_FDCAN(uint16_t t_COB_ID, uint32_t t_len, uint8_t* t_tx_data)
{

    uint8_t t_txBuf[67];

    uint8_t t_fnc_code = (t_COB_ID & 0xF00) >> 8;
    memcpy(t_txBuf, &t_fnc_code, 1);

    uint8_t t_node_ids = (t_COB_ID & 0xFF);
    memcpy(&t_txBuf[1], &t_node_ids, 1);
    memcpy(&t_txBuf[2], t_tx_data, t_len);

    t_txBuf[t_len + 2] = '\r';
    t_txBuf[t_len + 3] = '\n';

    IOIF_TransmitFDCAN1(t_COB_ID, t_tx_data, (uint8_t)t_len);

    return -1;
}

/* -------------------  ------------------- */
/*void SendEMCY()
{
	spi_success_case=1;
	spi3DmaTxBuff[255]=3;
}

void SendEMCY_disable()
{
	spi_success_case=1;
	spi3DmaTxBuff[255]=4;
}

void Send_power_off()
{
	spi_success_case=1;
	spi3DmaTxBuff[255]=5;
}
*/

/**
 *------------------------------------------------------------
 *                      STATIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions intended for internal use within this module.
 */

/* ------------------- STATE FUNCTION ------------------- */
static void StateOff_Ent(void)
{
    GUI_onoff = 0;
    GUI_command = 0;
    // No OFF state. Transit to Standby automatically.
	StateTransition(&devCommHdlr.stateMachine, TASK_STATE_STANDBY);
}

static void StateStandby_Ent(void)
{
    // No OFF state. Transit to Enable automatically.
}

int debug=0;
int cur=0;

static void StateStandby_Run()
{
/*	uint8_t temp=0;
	float temp1=0;
	if(debug==1){
		temp = 2; //fnc code
		memcpy(&spi3DmaRxBuff[0] + cur, &temp, 1);
		cur++;
		temp = 17; //node id
		memcpy(&spi3DmaRxBuff[0] + cur, &temp, 1);
		cur++;
		temp = 1; //n sdo
		memcpy(&spi3DmaRxBuff[0] + cur, &temp, 1);
		cur++;
		temp = 4;// TASK_IDX_DEV_COMM_HDLR
		memcpy(&spi3DmaRxBuff[0] + cur, &temp, 1);
		cur++;
		temp = 13;// SDO_ID_MSG_MD_DC_LENGTH_COMMAND
		memcpy(&spi3DmaRxBuff[0] + cur, &temp, 1);
		cur++;
		temp = 1;// status
		memcpy(&spi3DmaRxBuff[0] + cur, &temp, 1);
		cur++;

		temp = 1;//4 size
		memcpy(&spi3DmaRxBuff[0] + cur, &temp, 2);
		cur+=2;
		temp1 = 210.5;//(원하는 length reference 값: 200~270);//data
		memcpy(&spi3DmaRxBuff[0] + cur, &temp1, 4);
		cur++;

	*/	StateTransition(&devCommHdlr.stateMachine, TASK_STATE_ENABLE);
		/*debug=0;
	}
*/
}

static void StateStandby_Ext()
{
}

static void StateEnable_Ent(void)
{
    EntRoutines(&devCommHdlr.routine);
    devCommHdlrLoopCnt = 0;
}

static void StateEnable_Run(void)
{
    int t_check;
    uint8_t t_byte_len;
    uint32_t t_cursor = 0;
    uint16_t sdo_tx_id = SDO | (NODE_ID_LK << 4) | (uint8_t)0;
    uint8_t t_txBuf[64];

    if (spi_sw == 0)
    {
        if(debug_AM_PC_select==0){
            spi3DmaTxBuff[0] = 3;
            spi3DmaTxBuff[1] = 10;
            t_check = Pack_PDO(&spi3DmaTxBuff[2], &t_byte_len);
        }
        else
        {
            t_check = Pack_PDO(t_txBuf, &t_byte_len);
        }

    }



    if (spi_cw++ % divider == 0)
    {
        if(debug_AM_PC_select==0){
//            HAL_SPI_TransmitReceive_DMA(&hspi3, spi3DmaTxBuff, spi3DmaRxBuff, 4096);

            if(spi3DmaTxBuff[255]==7){
                  	for(int i=0;i< 2;i++){}
                  }

                HAL_SPI_TransmitReceive(&hspi3, spi3DmaTxBuff, spi3DmaRxBuff, 256, 5);
                d10Ptr[0]->data.err_code=0;
                d10Ptr[1]->data.err_code=0;
                d10Ptr[2]->data.err_code=0;
                d10Ptr[3]->data.err_code=0;
                spi3DmaTxBuff[255]=0;

                fnc_code = ((uint16_t)spi3DmaRxBuff[0]);
                t_cursor+=2;



                switch (fnc_code)
                {
                    case 2:
                        if (Unpack_SDO(&spi3DmaRxBuff[t_cursor]) < 0)
                        {
                            return SDO_RX_ERR;
                        }
                        else
                        {
                            SendRxSuccess();
                        }
                        break;
                    default:
                        break;
                }


        }
        else
        {
//        	Send_MSG(sdo_tx_id, (uint32_t)t_byte_len, t_txBuf);
        }
    }



//    Run_Routines(&devCommHdlr.routine);
    devCommHdlrLoopCnt++;
}

static void StateEnable_Ext(void)
{
    GUI_onoff = 0;
    GUI_command = 0;
    ExtRoutines(&devCommHdlr.routine);
}

static void StateError_Run(void)
{

}

/* ------------------- SPI CALLBACK ------------------- */
static void SpiTxRxCB(void* params)
{
    spi_rx_complete_flag = 1;
}

/* ------------------- ROUTINE ------------------- */
static DOP_Header_t Get_Header(uint8_t* t_byte_arr)
{
    DOP_Header_t t_header;
    memcpy(&t_header, t_byte_arr, sizeof(DOP_Header_t));
    return t_header;
}

//static void SendTimeOut()
//{
//    static uint8_t t_tx_data[4] = {'t', 'i', 'm', 'e'};
//    uint16_t t_identifier = TIMEOUT | (node_id << 4) | ori_node;
//
//    if (Send_MSG(t_identifier, 4, t_tx_data) != 0)
//    {
//        // TODO: MSG TX ERROR
//    }
//}

static void SendRxSuccess()
{
	spi_success_case=1;
    spi3DmaTxBuff[255]=1;
}
void SendMotionDone()
{
	spi_success_case=1;
	spi3DmaTxBuff[255]=2;
}

void SendEMCY()
{
	spi_success_case=1;
	spi3DmaTxBuff[255]=3;
}

void SendEMCY_disable()
{
	spi_success_case=1;
	spi3DmaTxBuff[255]=4;
}

void SendRobotOff()
{
	spi_success_case=1;
	spi3DmaTxBuff[255]=5;
}

void SendAutofittingDone()
{
	spi_success_case=1;
	spi3DmaTxBuff[255]=6;
}

void SendAutofittingStopDone()
{
	spi_success_case=1;
	spi3DmaTxBuff[255]=7;
}

void Send_curr_State_stand()
{
	spi3DmaTxBuff[254]=1;
}

void Send_curr_State_sit()
{
	spi3DmaTxBuff[254]=2;
}


static void init_buffef()
{

    for (int i = 0; i < 64; i++)
    {
        spi3DmaTxBuff[i] = 0;
        spi3DmaRxBuff[i] = 0;
        spi3DmaTxTestBuff[i] = 0;
    }
}

/* ------------------- EMCY ------------------- */
static void Recv_EMCY(uint8_t* t_byte_arr, uint32_t* t_err_code)
{
    memcpy(t_err_code, t_byte_arr, DOP_ERR_CODE_SIZE);
}

/* ------------------- SDO RX ------------------- */
static DOP_SDOArgs_t Convert_Bytes_to_SDO_req(uint8_t* t_byte_arr, uint16_t* t_byte_len)
{
    DOP_SDOArgs_t t_req;
   * t_byte_len = 0;

    int t_idx = sizeof(t_req.status);
    int t_len = sizeof(t_req.dataSize);

    memcpy(&t_req.dataSize, &t_byte_arr[t_idx], t_len);
   * t_byte_len += t_len;

    t_req.data = &t_byte_arr[t_idx + t_len];

    t_req.status = t_byte_arr[0];
   * t_byte_len += 1;

    return t_req;
}

static int Read_SDO(uint8_t* t_byte_arr)
{
    int t_byte_read = 0;
    DOP_Header_t t_header = Get_Header(t_byte_arr);
    t_byte_read += sizeof(DOP_Header_t);
    DOP_SDO_t* t_sdo = DOP_FindSDO(t_header.dictID, t_header.objID);
    if (t_sdo == NULL)
    {
        // TODO: Cannot Find SDO ERROR
        return -2;
    }

    uint16_t t_req_bytes = 0;
    DOP_SDOArgs_t t_req = Convert_Bytes_to_SDO_req(t_byte_arr + t_byte_read, &t_req_bytes);
    t_req.typeSize = t_sdo->args.typeSize; // Copy SDO info
    t_byte_read += t_req_bytes;

    uint16_t t_n_bytes = 0;
    if (t_req.status == DOP_SDO_REQU)
    {
        t_n_bytes = DOP_CallSDO(t_sdo, &t_req);
        cvector_push_back(sdo_res_list, t_header); // Assign Response
    }
    else if (t_req.status == DOP_SDO_SUCC || t_req.status == DOP_SDO_FAIL)
    {
        t_n_bytes = DOP_SetSDOArgs(t_sdo, &t_req);
        if (t_n_bytes < 0)
        {
            // TODO: Set SDO Argument ERROR
            return -1;
        }
    }
    else
    {
        // TODO: Read SDO Status ERROR
        return -1;
    }

    t_byte_read += t_n_bytes;
    return t_byte_read;
}

static int Unpack_SDO(uint8_t* t_byte_arr)
{
    int t_cursor = 0;
    // Get # of SDOs
    uint16_t t_n_sdo = 0;
    memcpy(&t_n_sdo, &t_byte_arr[t_cursor], DOP_OBJ_NUMS_SIZE);
    t_cursor += DOP_OBJ_NUMS_SIZE;

    // Call & Respond SDOs
    if (t_n_sdo > 0)
    {
        for (int i = 0; i < t_n_sdo; ++i)
        {
            int temp_cursor = Read_SDO(&t_byte_arr[t_cursor]);
            if (temp_cursor > 0)
            {
                t_cursor += temp_cursor;
            }
            else if (temp_cursor < 0)
            {
                // TODO: Unpack SDO ERROR
                return DOP_SDO_FAULT;
            }
        }
    }

    return DOP_SUCCESS;
}

/* ------------------- SDO TX ------------------- */
static int Convert_SDOres_to_Bytes(DOP_Header_t* t_header, uint8_t* t_byte_arr)
{
    int t_byte_written = 0;
    // Set SDO Header
    memcpy(t_byte_arr, t_header, sizeof(DOP_Header_t));
    t_byte_written += sizeof(DOP_Header_t);

    // Return Response
    DOP_SDO_t* t_sdo = DOP_FindSDO(t_header->dictID, t_header->objID);
    if (t_sdo == NULL)
    {
        // TODO: Cannot Find SDO ERROR
        return -2;
    }

    memcpy(t_byte_arr + t_byte_written, &t_sdo->args.status, sizeof(t_sdo->args.status));
    t_byte_written += sizeof(t_sdo->args.status);
    memcpy(t_byte_arr + t_byte_written, &t_sdo->args.dataSize, sizeof(t_sdo->args.dataSize));
    t_byte_written += sizeof(t_sdo->args.dataSize);

    int t_data_len = t_sdo->args.dataSize * t_sdo->args.typeSize;
    memcpy(t_byte_arr + t_byte_written, t_sdo->args.data, t_data_len);

    t_byte_written += t_data_len;

    return t_byte_written;
}

static int Pack_SDO(uint8_t* t_byte_arr, uint8_t* t_byte_len)
{
    // check send list whether these are empty or not
    if ((sdo_res_list == NULL) && (sdo_req_list == NULL))
    {
        return DOP_SDO_NOTHING;
    }

    // Message Packaging
    int t_cursor = 0;

    // Res SDOs
    int t_n_sdo_cursor = t_cursor;
    t_cursor += DOP_OBJ_NUMS_SIZE;

    uint8_t t_n_sdo = 0;

    if (sdo_res_list != NULL)
    {
        for (int i = 0; i < cvector_size(sdo_res_list); ++i)
        {
            int temp_cursor = Convert_SDOres_to_Bytes(&sdo_res_list[i], &t_byte_arr[t_cursor]);
            if (temp_cursor > 0)
            {
                t_cursor += temp_cursor;
                ++t_n_sdo;
            }
            else if (temp_cursor < 0)
            {
                // TODO: Pack Response SDO Error
                return DOP_SDO_FAULT;
            }
        }
        cvector_free(sdo_res_list);
        sdo_res_list = NULL;
    }

    // Req SDOs
    if (sdo_req_list != NULL)
    {
        for (int i = 0; i < cvector_size(sdo_req_list); ++i)
        {
            int temp_cursor = Convert_SDOres_to_Bytes(&sdo_req_list[i], &t_byte_arr[t_cursor]);
            if (temp_cursor > 0)
            {
                t_cursor += temp_cursor;
                ++t_n_sdo;
            }
            else if (temp_cursor < 0)
            {
                // TODO: Pack Request SDO Error
                return DOP_SDO_FAULT;
            }
        }
        cvector_free(sdo_req_list);
        sdo_req_list = NULL;
    }

    // Set # of SDOs
    memcpy(&t_byte_arr[t_n_sdo_cursor], &t_n_sdo, DOP_OBJ_NUMS_SIZE);

   * t_byte_len = t_cursor;

    return DOP_SUCCESS;
}

static int Send_SDO(uint8_t t_dest_node)
{
    uint8_t t_byte_len;
    uint16_t t_identifier = SDO | (node_id << 4) | t_dest_node;

    int t_check = Pack_SDO(fdcanRxData, &t_byte_len);

    if (t_check < 0)
    {
        // TODO: Send SDO Error
        return t_check;
    }
    else if (t_check)
    {
        return t_check;
    }

    if (t_byte_len > 64)
    {
        // TODO: TX MESSAGE TOO LONG ERROR
    }

    if (Send_MSG(t_identifier, t_byte_len, fdcanRxData) != 0)
    {
        return t_check;
        // TODO: MSG TX ERROR
    }

    return t_check;
}

//static int routine_test(uint8_t t_dest_node)
//{
//    test_routine++;
//
//    return 0;
//}

/* ------------------- PDO RX ------------------- */
static int Convert_Bytes_to_PDO(uint8_t* t_byte_arr)
{
    int t_byte_read = 0;

    DOP_Header_t t_header = Get_Header(t_byte_arr);
    t_byte_read += sizeof(DOP_Header_t);

    DOP_PDO_t* t_pdo = DOP_FindPDO(t_header.dictID, t_header.objID);
    if (t_pdo == NULL)
    {
        // TODO: Cannot Find PDO Error
        return -2;
    }

    uint16_t t_n_bytes = DOP_GetPDO(t_pdo, (void *)(t_byte_arr + t_byte_read));
    if (t_n_bytes < 0)
    {
        // TODO: Copy PDO to Receive Error
        return -1;
    }
    t_byte_read += t_n_bytes;
    return t_byte_read;
}

static int Unpack_PDO(uint8_t* t_byte_arr)
{
    int t_cursor = 0;

    // Get # of PDOs
    uint8_t t_n_pdo = 0;
    memcpy(&t_n_pdo, &t_byte_arr[t_cursor], DOP_OBJ_NUMS_SIZE);
    t_cursor += DOP_OBJ_NUMS_SIZE;

    if (t_n_pdo > 0)
    {
        for (int i = 0; i < t_n_pdo; ++i)
        {
            int temp_cursor = Convert_Bytes_to_PDO(&t_byte_arr[t_cursor]);
            if (temp_cursor > 0)
            {
                t_cursor += temp_cursor;
            }
            else if (temp_cursor < 0)
            {
                // TODO: Unpack PDO Error
                return DOP_PDO_FAULT;
            }
        }
    }

    return DOP_SUCCESS;
}

/* ------------------- PDO TX ------------------- */
static int Convert_PDO_to_Bytes(DOP_Header_t* t_header, uint8_t* t_byte_arr)
{

    int t_header_size = sizeof(DOP_Header_t);
    // Publish PDO
    DOP_PDO_t* t_pdo = DOP_FindPDO(t_header->dictID, t_header->objID);
    if (t_pdo == NULL)
    {
        // TODO: Cannot Find PDO
        return -2;
    }

    uint16_t t_n_bytes = DOP_SetPDO(t_pdo, t_byte_arr + t_header_size);
    if (t_n_bytes < 0)
    {
        // TODO: Copy PDO to Send
        return -1;
    }
    else if (t_n_bytes == 0)
    { // Nothing to publish
        return 0;
    }

    memcpy(t_byte_arr, t_header, t_header_size);
    return t_header_size + t_n_bytes;
}

static int Pack_PDO(uint8_t* t_byte_arr, uint8_t* t_byte_len)
{
    // check send list whether these are empty or not
    if (pdo_send_list == NULL)
    {
        return 0;
    }

    int t_cursor = 0;

    // Pub PDO
    int t_n_pdo_cursor = t_cursor;
    t_cursor += DOP_OBJ_NUMS_SIZE;

    uint8_t t_n_pdo = 0;

    if (pdo_send_list != NULL)
    {
        for (int i = 0; i < cvector_size(pdo_send_list); ++i)
        {

            int temp_cursor = Convert_PDO_to_Bytes(&pdo_send_list[i], &t_byte_arr[t_cursor]);
            if (temp_cursor > 0)
            {
                t_cursor += temp_cursor;
                ++t_n_pdo;
            }
            else if (temp_cursor < 0)
            {
                // TODO: Pack PDO Error
                return temp_cursor;
            }
        }
    }

    // Set # of PDOs
    memcpy(&t_byte_arr[t_n_pdo_cursor], &t_n_pdo, DOP_OBJ_NUMS_SIZE);

   * t_byte_len = t_cursor;

    return DOP_SUCCESS;
}

static int Send_PDO()
{
    int t_check;
    uint8_t t_byte_len;
    uint32_t t_cursor = 0;
    uint16_t sdo_tx_id = SDO | (NODE_ID_LK << 4) | (uint8_t)0;
    uint8_t t_txBuf[64];

    if (spi_sw == 0)
    {
    	if(debug_AM_PC_select==0){
			spi3DmaTxBuff[0] = 3;
			spi3DmaTxBuff[1] = 10;
			t_check = Pack_PDO(&spi3DmaTxBuff[2], &t_byte_len);
    	}
    	else
    	{
    		t_check = Pack_PDO(t_txBuf, &t_byte_len);
    	}

    }

    fnc_code = ((uint16_t)spi3DmaRxBuff[0]);
    t_cursor+=2;

    if(spi3DmaRxBuff[2]==1){
    	for(int i = 1; i<2; i++);
    }

    if(spi3DmaRxBuff[2]==5){
    	for(int i = 1; i<2; i++);
    }

    if (spi_cw++ % divider == 0)
    {
        if(debug_AM_PC_select==0){
//            HAL_SPI_TransmitReceive_DMA(&hspi3, spi3DmaTxBuff, spi3DmaRxBuff, 4096);
        		HAL_SPI_TransmitReceive(&hspi3, spi3DmaTxBuff, spi3DmaRxBuff, 256, 5);
        }
        else
        {
//        	Send_MSG(sdo_tx_id, (uint32_t)t_byte_len, t_txBuf);
        }

    }
//    for (int i = 0; i < 256; i++)
//    {
//        spi3DmaRxBuff[i] = 0;
//    }

	if(spi_rx_complete_flag == 1){



		if(spi_success_case==1){
			d10Ptr[0]->data.err_code=0;
			d10Ptr[1]->data.err_code=0;
			d10Ptr[2]->data.err_code=0;
			d10Ptr[3]->data.err_code=0;
			spi3DmaTxBuff[255]=0;
			spi_success_case=0;
		}


	    switch (fnc_code)
	    {
	    case 2:
	        if (Unpack_SDO(&spi3DmaRxBuff[t_cursor]) < 0)
	        {
	            return SDO_RX_ERR;
	        }
	        else
	        {
//	        	spi_success_case=1;
	        	SendRxSuccess();
	        }
	        break;
	    default:
	        break;
	    }

	    spi_rx_complete_flag = 0;
	}

    return t_check;
}

/* ------------------- LIST MANAGEMENT ------------------- */
static void Add_PDO_to_Send(uint8_t t_dictID, uint8_t t_objID)
{
    DOP_PDO_t* temp_pdo = DOP_FindPDO(t_dictID, t_objID);
    if (temp_pdo == NULL)
    {
        // TODO: Cannot Find PDO Error
        return;
    }

    DOP_Header_t t_pdo = {t_dictID, t_objID};

    for (int i = 0; i < cvector_size(pdo_send_list); ++i)
    {
        if ((pdo_send_list[i].dictID == t_dictID) && (pdo_send_list[i].objID == t_objID))
        {
            return;
        }
    }
    cvector_push_back(pdo_send_list, t_pdo);
}

static void Clear_PDO_to_Send()
{
    cvector_free(pdo_send_list);
    pdo_send_list = NULL;
}

/* ------------------- MESSAGE HANDLER ------------------- */
static int USB_Rx_Hdlr(uint8_t* t_Buf, uint32_t* t_Len)
{
    uint32_t t_cursor = 0;
    uint16_t sdo_tx_id = SDO | (NODE_ID_LK << 4) | (uint8_t)0;
    uint8_t t_txBuf[67];

    fnc_code = ((uint16_t)*t_Buf) << 8;
    t_cursor++;

    ori_node = ((*(t_Buf + t_cursor)) & 0xF0) >> 4;
    t_cursor++;
    // bug? length -2 required

    memcpy(usbRxData, &t_Buf[t_cursor],* t_Len - 2);

    switch (fnc_code)
    {

    case EMCY:
        Recv_EMCY(usbRxData, &err_code);
        // TODO: ERROR Process
        break;

    case SDO:
        if (Unpack_SDO(usbRxData) < 0)
        {
            return SDO_RX_ERR;
        }
        else
        {
            SendRxSuccess();
            // dh DH Send_SDO(ori_node);
        }
        break;

    case PDO:
        if (Unpack_PDO(usbRxData) < 0)
        {
            return PDO_RX_ERR;
        }
        break;
    case QT_BYPASS:
        sdo_tx_id = SDO | (NODE_ID_CM << 4) | t_Buf[3];
        t_cursor += 2;
        memcpy(t_txBuf, &t_Buf[t_cursor],* t_Len - 4);
        IOIF_TransmitFDCAN1(sdo_tx_id,  &t_Buf[t_cursor], (uint8_t)((*t_Len) - 4));

        break;

    default:
        break;
    }

    return 0;
}

static int Fdcan_Rx_Hdlr(uint16_t t_wasp_id, uint8_t* t_rx_data)
{
    fnc_code = t_wasp_id & 0x700;
    ori_node = (t_wasp_id & 0x0F0) >> 4;
    uint8_t t_txBuf[67];
    memcpy(t_txBuf, t_rx_data, 11);
    switch (fnc_code)
    {

    case EMCY:
        Recv_EMCY(t_rx_data, &err_code);
        // TODO: ERROR Process
        break;

    case SDO:
        //            if (DETECT_FD_receive_ID(t_rx_data) < 0) {
        if (Unpack_SDO(t_rx_data) < 0)
        {
            return SDO_RX_ERR;
        }
        else
        {
            //                Send_SDO(ori_node);
        }
        break;

    case PDO:
        if (Unpack_PDO(t_rx_data) < 0)
        {
            return PDO_RX_ERR;
        }

        break;
    case QT_BYPASS:
        CDC_Transmit_FS(t_txBuf, 11);
        // TODO: ERROR Process
        break;

    case SEND_TRAJ:
        if (traj_idx < 9995)
        {
            MD_send_traj();
        }
        else
        {
            traj_idx = 0;
        }
        // recursion calling
        // traj_transfer_dh

        break;
    default:
        break;
    }

    return 0;
}

static int request_send_PDO()
{
    uint8_t t_byte_len = 2;
    uint8_t t_dest_node = 2;
    uint16_t t_identifier;
    uint8_t t_node_id = 1;
    uint8_t t_buf[2] = {0};

    t_byte_len = 4;
    t_dest_node = 2;
    t_identifier = PDO | (t_node_id << 4) | t_dest_node;
    IOIF_TransmitFDCAN1(t_identifier, t_buf, (uint8_t)0);

    t_dest_node = 4;
    t_identifier = PDO | (t_node_id << 4) | t_dest_node;
    IOIF_TransmitFDCAN1(t_identifier, t_buf, (uint8_t)0);

    return 0;
}

/* ------------------- SDO CALLBACK ------------------- */
static void Set_Send_PDO_List(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res)
{
    Clear_PDO_to_Send();

    int t_cursor = 0;
    uint8_t* t_ids = (uint8_t *)t_req->data;
    while (t_cursor < 2 * t_req->dataSize)
    {
        uint8_t t_dictID = t_ids[t_cursor++];
        uint8_t t_objID = t_ids[t_cursor++];
        Add_PDO_to_Send(t_dictID, t_objID);
    }

    t_res->status = DOP_SDO_SUCC;
}

static void Set_MS_Enum(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res)
{
    memcpy(&MS_enum, t_req->data, 1);

    t_res->dataSize = 1;
    t_res->status = DOP_SDO_SUCC;
}

static void Set_GUI_COMM_OnOff(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res)
{
    memcpy(&GUI_onoff, t_req->data, 1);

    t_res->dataSize = 1;
    t_res->status = DOP_SDO_SUCC;
}

static void Set_GUI_COMM_Command(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res)
{
    memcpy(&GUI_command, t_req->data, 1);

    t_res->dataSize = 1;
    t_res->status = DOP_SDO_SUCC;
}

static void Set_D10_UprightLength_Command(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res)
{
	static float data;
	memcpy(&data, t_req->data, 4);
 	Enable_Upright_lenth_cmd(d10Ptr[e_L30_RH_IDX]);
    Set_dc_set_length(d10Ptr[e_L30_RH_IDX], data);
	return;
}

/*static void Send_AM_D10_UprightLength_Act(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res)//?
{
	spi3DmaTxBuff[0]=lengthact;
}*/


// TODO: erase
static void DETECT_ACTIVE_MD(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res)
{
    uint8_t txBuf[64];
    uint8_t node_id = 0;
    uint16_t sdo_tx_id = SDO | (NODE_ID_CM << 4) | node_id;
    uint8_t n_sdo = 1;
    uint8_t task_id = 2;
    uint8_t sdo_id = 9;
    uint8_t sdo_status = 1;
    uint16_t num_of_data = 1;
    uint8_t data = 1;
    uint32_t cursor = 0;

    txBuf[cursor++] = n_sdo;
    txBuf[cursor++] = task_id;
    txBuf[cursor++] = sdo_id;
    txBuf[cursor++] = sdo_status;
    txBuf[cursor] = num_of_data;
    cursor += 2;
    txBuf[cursor++] = data;
    IOIF_TransmitFDCAN1(sdo_tx_id, txBuf, cursor);

    t_res->dataSize = 1;
    t_res->status = DOP_SDO_SUCC;
}

static void DETECT_receive_ID(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res)
{

    uint16_t t_identifier = 0xff;
    uint8_t txBuf[64] = {0};
    uint8_t n_sdo = 1;
    uint8_t task_id = 2;
    uint8_t sdo_id = 9;
    uint8_t sdo_status = 1;
    uint8_t num_of_data = 1;
    uint8_t data = 0;
    memcpy(&data, t_req->data, 1);
    if (test_odd++ % 2 == 0)
    {
        test1 = data;
    }
    else
    {
        test2 = data;
    }
    uint32_t cursor = 0;
    txBuf[cursor++] = n_sdo;
    txBuf[cursor++] = task_id;
    txBuf[cursor++] = sdo_id;
    txBuf[cursor++] = sdo_status;
    txBuf[cursor++] = num_of_data;
    txBuf[cursor++] = data;
    Send_MSG(t_identifier, cursor, txBuf);

    t_res->dataSize = 1;
    t_res->status = DOP_SDO_SUCC;
}

static void MD_PDO_Enable()
{
    //	Start_PDO_Streaming();
}

static void MD_send_traj()
{
    uint8_t txBuf[64];
    uint8_t node_id = 3;
    uint16_t sdo_tx_id = SEND_TRAJ | (NODE_ID_CM << 4) | node_id;
    uint32_t cursor = 0;
    float data = 0;
    HAL_StatusTypeDef error;

    memcpy(&txBuf[cursor], &traj_idx, 4); // traj_idx
    cursor += 4;

    if (traj_idx < 9989)
    { // traj_send
        for (int i = 0; i < 15; i++)
        {
            data = sin(test1++ * 3.14 / (float)180 * 360 / 10000);
            memcpy(&txBuf[cursor], &test1, 4);
            //			test1++;
            cursor += 4;
        }
        traj_idx += 15;
    }
    else
    { // traj_send_exception
        for (int i = 0; i < 10; i++)
        {
            data = sin(test1++ * 3.14 / (float)180 * 360 / 10000);
            memcpy(&txBuf[cursor], &test1, 4);
            //			test1++;
            cursor += 4;
        }
        traj_idx += 10;
    }

    for (int i = 0; i < 1000; i++);

    if (traj_idx > 7100 && traj_idx < 7200)
    {

        data = 1;
    }
    error = IOIF_TransmitFDCAN1(sdo_tx_id, txBuf, cursor);
}
// traj_transfer_dh

#endif /* #ifdef L30_CM_ENABLED */
