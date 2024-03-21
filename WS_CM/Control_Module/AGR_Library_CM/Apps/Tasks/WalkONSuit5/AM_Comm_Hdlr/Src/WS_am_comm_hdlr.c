#include "WS_am_comm_hdlr.h"

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

uint8_t RxData[64];
uint8_t RxData_Pre[64];
uint8_t Ending_Message[64];
uint8_t tx_check_message[20];

uint8_t flag;
uint8_t flag2;
uint8_t flag3;
uint8_t flag4;
uint8_t flag5;
int equality;
int equality2;

AMtoCMBuffer AMtoCMdatas;
CMtoAMBuffer CMtoAMdatas;

extern RobotSettingFileInfo RS_File_Read;
extern MotionMapFileInfo MotionMap_File_Read;

uint8_t *eth_rx_data;
uint8_t rx_array;
uint16_t tx_len, rx_len;
uint32_t mode_stack;
uint32_t state_stack;

/*static uint32_t writeAddr;
static uint8_t status;*/

NetconnHandle hnet;
float cnt_eth = 0.0;
TaskObj_t AMCommHdlr;


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

static void StateOff_Ent(void);

static void StateStandby_Run(void);

static void StateEnable_Run(void);
static void StateEnable_Ext(void);

static void StateError_Ent(void);

static int32_t NetconnSrvCreate(void);
static int32_t NetconnSrvConnect(void);

static void NetconnEnd(void);
static void NetconnSend(uint8_t* txdata, u16_t txlen);
static void NetconnRecv(uint8_t** rxdata, u16_t* rxlen);
static void NetconnClearBuff(void);

static int32_t AMSession(void);

static int Rx_Callback(uint8_t rx_data[64]);


/**
 *------------------------------------------------------------
 *                      PUBLIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions that interface with this module.
 */

// DOP_COMMON_SDO_CB(AMCommHdlr)

void InitAMCommHdlr(void)
{
//	DOP_Init();

    InitTask(&AMCommHdlr);
    DOPC_AssignTaskID(&AMCommHdlr, TASK_IDX_AM_COMM_HDLR);

    /* State Definition */
    TASK_CREATE_STATE(&AMCommHdlr, TASK_STATE_OFF,     StateOff_Ent,   NULL,             NULL,            false);
    TASK_CREATE_STATE(&AMCommHdlr, TASK_STATE_STANDBY, NULL,           StateStandby_Run, NULL,            true);
    TASK_CREATE_STATE(&AMCommHdlr, TASK_STATE_ENABLE,  NULL,           StateEnable_Run,  StateEnable_Ext, false);
    TASK_CREATE_STATE(&AMCommHdlr, TASK_STATE_ERROR,   StateError_Ent, NULL,             NULL,            false);

    // Routine

    // DOD
    // DOP_CreateDOD(TASK_IDX_AM_COMM_HDLR);

    // PDO
    // DOP_COMMON_PDO_CREATE(TASK_IDX_AM_COMM_HDLR, AMCommHdlr);

    // SDO
    // DOP_COMMON_SDO_CREATE(TASK_IDX_AM_COMM_HDLR)
}

void RunAMCommHdlr(void)
{
    RunTask(&AMCommHdlr);
}


/**
 *------------------------------------------------------------
 *                      STATIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions intended for internal use within this module.
 */

static void StateOff_Ent(void)
{
    // No OFF state. Transit to Standby automatically.
    StateTransition(&AMCommHdlr.stateMachine, TASK_STATE_STANDBY);
}

static void StateStandby_Run(void)
{
    if (!hnet.srv) { // If server is not created
        NetconnSrvCreate();
    }
    if (NetconnSrvConnect() < 0) {
        StateTransition(&AMCommHdlr.stateMachine, TASK_STATE_ERROR);
    } else {
        StateTransition(&AMCommHdlr.stateMachine, TASK_STATE_ENABLE);
    }
}

static void StateEnable_Run(void)
{
//	if (DOPSession()) {
//        StateTransition(&AMCommHdlr.stateMachine, TASK_STATE_ERROR);
//        return;
//    }
	if (AMSession()) {
        StateTransition(&AMCommHdlr.stateMachine, TASK_STATE_ERROR);
        return;
    }
}

static void StateEnable_Ext(void)
{
    NetconnClearBuff();
    NetconnEnd();
    DOP_ClearPDOtoSend();
}

static void StateError_Ent(void)
{
    NetconnClearBuff();
    NetconnEnd();
    StateTransition(&AMCommHdlr.stateMachine, TASK_STATE_STANDBY);
}

/* Netconn Helper */
static int32_t NetconnSrvCreate()
{
    // Create
	if (hnet.srv != NULL) { // Clear if not empty
		free(hnet.srv);
		hnet.srv = NULL;
	}

    hnet.srv = netconn_new(NETCONN_TCP);
    if (hnet.srv == NULL) {
        return -1;
    }
    // Bind
    if (netconn_bind(hnet.srv, IP4_ADDR_ANY, ETH_CTRL_TCP_PORT) != ERR_OK) {
        return -2;
    }

    // Listen
    if (netconn_listen(hnet.srv) != ERR_OK) {
        return -3;
    }

    return 0;
}

static int32_t NetconnSrvConnect()
{
    // Accept
    if (netconn_accept(hnet.srv, &hnet.cli) != ERR_OK) {
        return -4;
    }

    return 0;
}

static void NetconnEnd()
{
    if (hnet.cli) {
        hnet.err = netconn_close(hnet.cli);
        hnet.err = netconn_delete(hnet.cli);
        hnet.cli = NULL;
    }
}

static void NetconnSend(uint8_t* txdata, u16_t txlen)
{
    hnet.err = netconn_write(hnet.cli, (void*)txdata, txlen, NETCONN_COPY);
}

static void NetconnRecv(uint8_t** rxdata, u16_t* rxlen)
{
    hnet.err = netconn_recv(hnet.cli, &hnet.buff);
    if (hnet.err != ERR_OK) {
        return;
    }
    hnet.err = netbuf_data(hnet.buff, (void**)rxdata, rxlen);
}

static void NetconnClearBuff()
{
    // This must be called before next Recv
    netbuf_delete(hnet.buff);
}

/* Data Object Protocol */
//static int32_t DOPSession()
//{
//
//    // Rx
//    NetconnRecv(&eth_rx_data, &rx_len);
//
//    if (hnet.err != ERR_OK) {
//        return ETH_CTRL_TCP_RX_ERR;
//    }
//
//    if (DOP_Rx(eth_rx_data, rx_len) < 0) {
//        return ETH_CTRL_DOP_RX_ERR;
//    }
//
//    // Tx
//    if (DOP_Tx(hnet.tx_msg, (uint16_t*)&tx_len) < 0) {
//        return ETH_CTRL_DOP_TX_ERR;
//    }
//
//    NetconnSend(hnet.tx_msg, tx_len);
//
//
//    if (hnet.err != ERR_OK) {
//        return ETH_CTRL_TCP_TX_ERR;
//    }
//
//    NetconnClearBuff();
//
//    return ETH_CTRL_OK;
//}

static int32_t AMSession()
{
	cnt_eth++;

	NetconnRecv(&eth_rx_data, &rx_len);
	// memcpy(&AMtoCMdatas, eth_rx_data, rx_len);

	for (int i = 0;i < 64; i++) {
		RxData[i] = *(eth_rx_data + i);
	}

	Rx_Callback(RxData);

	/*
	if (AMtoCMdatas.Mode != 0){
		mode_stack = AMtoCMdatas.Mode;
	}
	if (AMtoCMdatas.State != 0){
		state_stack = AMtoCMdatas.State;
	}
	*/

    if (hnet.err != ERR_OK) {
        return ETH_CTRL_TCP_RX_ERR;
    }

    tx_len = ETH_CTRL_TX_BUFF_SIZE;

    /*

    float frequency = 2000.0; // Set the frequency of the sinusoidal signal
    float amplitude = 10.0; // Set the amplitude of the sinusoidal signal

    for (uint32_t i = 0; i < sizeof(CMtoAMdatas) / sizeof(CMtoAMdatas.RH_index); i++) {
        if (i % 5 == 0) {
            ((uint32_t*)&CMtoAMdatas)[i] = cnt_eth;
        }
        else {
        	float signal = amplitude * sin(2.0 * M_PI * cnt_eth/frequency);
            ((float*)&CMtoAMdatas)[i] = signal;
            frequency -= 5.0;
        }
    }

    */

    flag5 = 0;

	for(int i = 0; i < 20; i++) {
		tx_check_message[i] = 255;
	}

	for(int i = 0; i < 20; i++) {
		if (RxData[i] == tx_check_message[i]) {
			flag5 += 1;
		}
	}
	if (flag5 == 20) {
		equality2 = 1;
	}
	else {
		equality2 = 0;
	}

	if (equality2 == 1) {

		if ((RxData[20] == 1) & (RxData[21] == 1)) {
			RobotSettingFileInfo_check robotsettingfileinfo_check;

			memset(&robotsettingfileinfo_check, 0, sizeof(RobotSettingFileInfo_check));

			robotsettingfileinfo_check.FileID = 1;
			robotsettingfileinfo_check.mode = 1;

			robotsettingfileinfo_check.robot_id = RS_File_Read.robot_id;
			robotsettingfileinfo_check.file_version = RS_File_Read.file_version;

			memcpy(hnet.tx_msg, &robotsettingfileinfo_check, ETH_CTRL_TX_BUFF_SIZE);
		}

		if ((RxData[20] == 1) & (RxData[21] == 2)) {
			RobotSettingData_check robotsettingdata_check;

			memset(&robotsettingdata_check, 0, sizeof(RobotSettingFileInfo_check));

			robotsettingdata_check.FileID = 1;
			robotsettingdata_check.mode = 2;

			int device_list = RxData[22];

			robotsettingdata_check.usage = RS_File_Read.vec[device_list].usage;
			robotsettingdata_check.FDCAN_CH = RS_File_Read.vec[device_list].FDCAN_CH;
			robotsettingdata_check.FDCAN_ID = RS_File_Read.vec[device_list].FDCAN_ID;
			for (int i = 0; i < 20; i++) {
				robotsettingdata_check.name[i] = RS_File_Read.vec[device_list].name[i];
    		}

			memcpy(hnet.tx_msg, &robotsettingdata_check, ETH_CTRL_TX_BUFF_SIZE);
		}

		if ((RxData[20] == 5) & (RxData[21] == 1)) {
			MotionMapFileInfo_check motionmapfileinfo_check;

			memset(&motionmapfileinfo_check, 0, sizeof(MotionMapFileInfo_check));

			motionmapfileinfo_check.FileID = 5;
			motionmapfileinfo_check.mode = 1;

			motionmapfileinfo_check.robot_id = MotionMap_File_Read.robot_id;
			motionmapfileinfo_check.file_version = MotionMap_File_Read.file_version;
			motionmapfileinfo_check.num_ms = MotionMap_File_Read.num_ms;
			motionmapfileinfo_check.cnt = MotionMap_File_Read.cnt;
			motionmapfileinfo_check.send_state = MotionMap_File_Read.send_state;

			memcpy(hnet.tx_msg, &motionmapfileinfo_check, ETH_CTRL_TX_BUFF_SIZE);
		}


		if ((RxData[20] == 5) & (RxData[21] == 1)) {
			MotionMapFileInfo_check motionmapfileinfo_check;

			memset(&motionmapfileinfo_check, 0, sizeof(MotionMapFileInfo_check));

			motionmapfileinfo_check.FileID = 5;
			motionmapfileinfo_check.mode = 1;

			motionmapfileinfo_check.robot_id = MotionMap_File_Read.robot_id;
			motionmapfileinfo_check.file_version = MotionMap_File_Read.file_version;
			motionmapfileinfo_check.num_ms = MotionMap_File_Read.num_ms;
			motionmapfileinfo_check.cnt = MotionMap_File_Read.cnt;
			motionmapfileinfo_check.send_state = MotionMap_File_Read.send_state;

			memcpy(hnet.tx_msg, &motionmapfileinfo_check, ETH_CTRL_TX_BUFF_SIZE);

		}
		if ((RxData[20] == 5) & (RxData[21] == 2)) {
			MotionMapCtrlSetting_check motionmapctrlsetting_check;

			memset(&motionmapctrlsetting_check, 0, sizeof(MotionMapCtrlSetting_check));

			uint8_t MS_ID_temp = RxData[22];
			uint8_t MD_ID_temp = RxData[23];

			motionmapctrlsetting_check.FileID = 5;
			motionmapctrlsetting_check.mode = 2;

			motionmapctrlsetting_check.MS_ID = MS_ID_temp;
			motionmapctrlsetting_check.MD_ID = MD_ID_temp;
			motionmapctrlsetting_check.FF = MotionMap_File_Read.MS[MS_ID_temp].MD[MD_ID_temp].c_vector[0].FF_gain;
			motionmapctrlsetting_check.PD = MotionMap_File_Read.MS[MS_ID_temp].MD[MD_ID_temp].c_vector[0].PD_gain;
			motionmapctrlsetting_check.IC = MotionMap_File_Read.MS[MS_ID_temp].MD[MD_ID_temp].c_vector[0].IC_gain;
			motionmapctrlsetting_check.DOB = MotionMap_File_Read.MS[MS_ID_temp].MD[MD_ID_temp].c_vector[0].DOB_gain;
			motionmapctrlsetting_check.IRC = MotionMap_File_Read.MS[MS_ID_temp].MD[MD_ID_temp].c_vector[0].IRC_gain;
			motionmapctrlsetting_check.FC = MotionMap_File_Read.MS[MS_ID_temp].MD[MD_ID_temp].c_vector[0].FC_gain;

			memcpy(hnet.tx_msg, &motionmapctrlsetting_check, ETH_CTRL_TX_BUFF_SIZE);
		}
		if ((RxData[20] == 5) & (RxData[21] == 3)) {
			MotionMapPVector_check motionmapPvector_check;

			memset(&motionmapPvector_check, 0, sizeof(MotionMapPVector_check));

			uint8_t MS_ID_temp = RxData[22];
			uint8_t MD_ID_temp = RxData[23];

			motionmapPvector_check.FileID = 5;
			motionmapPvector_check.mode = 3;

			motionmapPvector_check.MS_ID = MS_ID_temp;
			motionmapPvector_check.MD_ID = MD_ID_temp;

			for (int i = 0; i < 10; i++) {
				motionmapPvector_check.p_vector[i].yd = MotionMap_File_Read.MS[MS_ID_temp].MD[MD_ID_temp].p_vector[i].yd;
				motionmapPvector_check.p_vector[i].L = MotionMap_File_Read.MS[MS_ID_temp].MD[MD_ID_temp].p_vector[i].L;
				motionmapPvector_check.p_vector[i].s0 = MotionMap_File_Read.MS[MS_ID_temp].MD[MD_ID_temp].p_vector[i].s0;
				motionmapPvector_check.p_vector[i].sd = MotionMap_File_Read.MS[MS_ID_temp].MD[MD_ID_temp].p_vector[i].sd;
			}

			memcpy(hnet.tx_msg, &motionmapPvector_check, ETH_CTRL_TX_BUFF_SIZE);
		}
		if ((RxData[20] == 5) & (RxData[21] == 4)) {
			MotionMapFVector_check motionmapFvector_check;

			memset(&motionmapFvector_check, 0, sizeof(MotionMapFVector_check));

			uint8_t MS_ID_temp = RxData[22];
			uint8_t MD_ID_temp = RxData[23];

			motionmapFvector_check.FileID = 5;
			motionmapFvector_check.mode = 4;

			motionmapFvector_check.MS_ID = MS_ID_temp;
			motionmapFvector_check.MD_ID = MD_ID_temp;

			for (int i = 0; i < 10; i++) {
				motionmapFvector_check.f_vector[i].mode_idx = MotionMap_File_Read.MS[MS_ID_temp].MD[MD_ID_temp].f_vector[i].mode_idx;
				motionmapFvector_check.f_vector[i].tau_max = MotionMap_File_Read.MS[MS_ID_temp].MD[MD_ID_temp].f_vector[i].tau_max;
				motionmapFvector_check.f_vector[i].delay = MotionMap_File_Read.MS[MS_ID_temp].MD[MD_ID_temp].f_vector[i].delay;
			}

			memcpy(hnet.tx_msg, &motionmapFvector_check, ETH_CTRL_TX_BUFF_SIZE);
		}
		if ((RxData[20] == 5) & (RxData[21] == 5)) {
			MotionMapIVector_check motionmapIvector_check;

			memset(&motionmapIvector_check, 0, sizeof(MotionMapIVector_check));

			uint8_t MS_ID_temp = RxData[22];
			uint8_t MD_ID_temp = RxData[23];

			motionmapIvector_check.FileID = 5;
			motionmapIvector_check.mode = 5;

			motionmapIvector_check.MS_ID = MS_ID_temp;
			motionmapIvector_check.MD_ID = MD_ID_temp;

			for (int i = 0; i < 10; i++) {
				motionmapIvector_check.i_vector[i].epsilon_target = MotionMap_File_Read.MS[MS_ID_temp].MD[MD_ID_temp].i_vector[i].epsilon_target;
				motionmapIvector_check.i_vector[i].Kp_target = MotionMap_File_Read.MS[MS_ID_temp].MD[MD_ID_temp].i_vector[i].Kp_target;
				motionmapIvector_check.i_vector[i].Kd_target = MotionMap_File_Read.MS[MS_ID_temp].MD[MD_ID_temp].i_vector[i].Kd_target;
				motionmapIvector_check.i_vector[i].lambda_target = MotionMap_File_Read.MS[MS_ID_temp].MD[MD_ID_temp].i_vector[i].lambda_target;
				motionmapIvector_check.i_vector[i].duration = MotionMap_File_Read.MS[MS_ID_temp].MD[MD_ID_temp].i_vector[i].duration;
			}

			memcpy(hnet.tx_msg, &motionmapIvector_check, ETH_CTRL_TX_BUFF_SIZE);
		}

		NetconnSend(hnet.tx_msg, tx_len);

		if (hnet.err != ERR_OK) {
			return ETH_CTRL_TCP_TX_ERR;
		}

		NetconnClearBuff();

	} else {

		memcpy(hnet.tx_msg, &CMtoAMdatas, ETH_CTRL_TX_BUFF_SIZE);

		NetconnSend(hnet.tx_msg, tx_len);

		if (hnet.err != ERR_OK) {
			return ETH_CTRL_TCP_TX_ERR;
		}

		NetconnClearBuff();

	}

	return ETH_CTRL_OK;
}

uint32_t test_idx;

static int Rx_Callback(uint8_t eth_rx_data[64])
{
	flag4 = 0;
	test_idx++;
	//equality = array_equal(RxData,Ending_Message,64);

	for(int i = 0; i < 64; i++) {
		Ending_Message[i] = 63 - i;
	}

	for(int i = 0; i < 64; i++) {
		if (eth_rx_data[i] == Ending_Message[i]) {
			flag4 += 1;
		}
	}
	if (flag4 == 64) {
		equality = 1;
	}
	else {
		equality = 0;
	}

/*********************************(Robot Setting File)******************************************/

        if (eth_rx_data[0] == 1) {             // Robot setting data
        	if (eth_rx_data[1] == 1) {         // Initiator packet for robot setting data
        		RS_File.robot_id     = eth_rx_data[2];
        		RS_File.file_version = eth_rx_data[3];
        	}
        	else if (eth_rx_data[1] == 2) {    // Data packet for robot setting data
        		uint16_t t_row = (eth_rx_data[2] << 8) | (eth_rx_data[3]);
        		RS_File.vec[t_row].usage    = eth_rx_data[4];
        		RS_File.vec[t_row].FDCAN_ID = (eth_rx_data[5] << 8) | (eth_rx_data[6]);
        		RS_File.vec[t_row].FDCAN_CH = eth_rx_data[7];
        		for (int i = 0; i < 20; i++) {
        			RS_File.vec[t_row].name[i] = eth_rx_data[i+8];
        		}
        	}
        }
        if ((RxData_Pre[0] == 1) & (equality == 1)) {
         		Save_RobotSetting();
        		Download_RobotSetting();

        		BeepAlert(BEEP_ALERT_FREQ_HIGH, BEEP_ALERT_DUTY, BEEP_ALERT_TIME_VERY_SHORT, BEEP_ALERT_TIME_VERY_SHORT);
        		BeepAlert(BEEP_ALERT_FREQ_LOW,  BEEP_ALERT_DUTY, BEEP_ALERT_TIME_VERY_SHORT, BEEP_ALERT_TIME_VERY_SHORT);

        	    All_MD_Init();
        	    Make_Overall_PDODataList();
        }

/*********************************(FSM File)******************************************/

        if (eth_rx_data[0] == 2) {             // FSM data
        	if (eth_rx_data[1] == 1) {         // Initiator packet for FSM data
        		FSM_File.robot_id =     eth_rx_data[2];
        		FSM_File.file_version = eth_rx_data[3];
        	}
        	else if (eth_rx_data[1] == 2) {    // Data packet for FSM data
        		uint16_t t_row = (eth_rx_data[2] << 8) | (eth_rx_data[3]);
        		FSM_File.vec[t_row].StateID              = eth_rx_data[4];
        		FSM_File.vec[t_row].MotionSetID          = eth_rx_data[5];
        		FSM_File.vec[t_row].ActionID             = (eth_rx_data[6] << 24) | (eth_rx_data[7] << 16) | (eth_rx_data[8] << 8) | (eth_rx_data[9]);
        		FSM_File.vec[t_row].TabletModeID         = eth_rx_data[10];
        		FSM_File.vec[t_row].TimeOut              = (eth_rx_data[11] << 8) | (eth_rx_data[12]);
        		FSM_File.vec[t_row].DefaultTargetStateID = eth_rx_data[13];
        		for (int i = 0; i < MAX_N_TRANSITION; i++) {
        			FSM_File.vec[t_row].ExitConditionID[i] = eth_rx_data[i+14];
          		}
        		for (int i = 0; i < MAX_N_TRANSITION; i++) {
          		    FSM_File.vec[t_row].TargetStateID[i] = eth_rx_data[i+34];
          		}
        	}
        }
        if ((RxData_Pre[0] == 2) & (equality == 1)) {   // End of transmission and saving
            BeepAlert(BEEP_ALERT_FREQ_HIGH, BEEP_ALERT_DUTY, BEEP_ALERT_TIME_VERY_SHORT, BEEP_ALERT_TIME_VERY_SHORT);
            BeepAlert(BEEP_ALERT_FREQ_LOW,  BEEP_ALERT_DUTY, BEEP_ALERT_TIME_VERY_SHORT, BEEP_ALERT_TIME_VERY_SHORT);
            BeepAlert(BEEP_ALERT_FREQ_HIGH, BEEP_ALERT_DUTY, BEEP_ALERT_TIME_VERY_SHORT, BEEP_ALERT_TIME_VERY_SHORT);
            BeepAlert(BEEP_ALERT_FREQ_LOW,  BEEP_ALERT_DUTY, BEEP_ALERT_TIME_VERY_SHORT, BEEP_ALERT_TIME_VERY_SHORT);
        	Save_FSM1();
        	Download_FSM1();
        }

/*********************************(DMS File)******************************************/

        if (eth_rx_data[0] == 4) {             // DMS data
        	if (eth_rx_data[1] == 1) {         // Initiator packet for DMS data
        		DMS_File.robot_id =     eth_rx_data[2];
        		DMS_File.file_version = eth_rx_data[3];
        	}
        	else if (eth_rx_data[1] == 2) {    // Data packet for DMS data
        		uint16_t t_row = (eth_rx_data[2] << 8) | (eth_rx_data[3]);
        		DMS_File.vec[t_row].Enable      = eth_rx_data[4];
        		DMS_File.vec[t_row].DeviceID    = eth_rx_data[5];
        		DMS_File.vec[t_row].CM_Save_Opt = eth_rx_data[6];
        		DMS_File.vec[t_row].AM_Send_Opt = eth_rx_data[7];
        	}
        }
        if ((RxData_Pre[0] == 4) & (equality == 1)) {   // End of transmission and saving
            BeepAlert(BEEP_ALERT_FREQ_HIGH, BEEP_ALERT_DUTY, BEEP_ALERT_TIME_VERY_SHORT, BEEP_ALERT_TIME_VERY_SHORT);
            BeepAlert(BEEP_ALERT_FREQ_LOW,  BEEP_ALERT_DUTY, BEEP_ALERT_TIME_VERY_SHORT, BEEP_ALERT_TIME_VERY_SHORT);
            BeepAlert(BEEP_ALERT_FREQ_HIGH, BEEP_ALERT_DUTY, BEEP_ALERT_TIME_VERY_SHORT, BEEP_ALERT_TIME_VERY_SHORT);
            BeepAlert(BEEP_ALERT_FREQ_LOW,  BEEP_ALERT_DUTY, BEEP_ALERT_TIME_VERY_SHORT, BEEP_ALERT_TIME_VERY_SHORT);
            BeepAlert(BEEP_ALERT_FREQ_HIGH, BEEP_ALERT_DUTY, BEEP_ALERT_TIME_VERY_SHORT, BEEP_ALERT_TIME_VERY_SHORT);
            BeepAlert(BEEP_ALERT_FREQ_LOW,  BEEP_ALERT_DUTY, BEEP_ALERT_TIME_VERY_SHORT, BEEP_ALERT_TIME_VERY_SHORT);
        	Save_DMS();
        	Download_DMS();

            All_MD_Init();
            Make_Overall_PDODataList();

        }

/*********************************(MotionMap File)******************************************/

        if (eth_rx_data[0] == 5) {               // Motion Map data
        	if (eth_rx_data[1] == 1) {           // Initiator packet for Motion Map data
        		MotionMap_File.robot_id =     eth_rx_data[2];
        		MotionMap_File.file_version = eth_rx_data[3];
        	}
        	else if (eth_rx_data[1] == 2) {      // Control setting packet for Motion Map data
        		uint16_t t_row_MS = eth_rx_data[2]-1;
        		uint16_t t_row_MD = eth_rx_data[6]-1;
        		MotionMap_File.MS[t_row_MS].MS_ID             = eth_rx_data[2];
        		MotionMap_File.MS[t_row_MS].max_p_vectors_len = eth_rx_data[3];
        		MotionMap_File.MS[t_row_MS].max_f_vectors_len = eth_rx_data[4];
        		MotionMap_File.MS[t_row_MS].max_i_vectors_len = eth_rx_data[5];
        		MotionMap_File.MS[t_row_MS].MD[t_row_MD].c_vector[0].FF_gain  = eth_rx_data[7];
        		MotionMap_File.MS[t_row_MS].MD[t_row_MD].c_vector[0].PD_gain  = eth_rx_data[8];
        		MotionMap_File.MS[t_row_MS].MD[t_row_MD].c_vector[0].IC_gain = eth_rx_data[9];
        		MotionMap_File.MS[t_row_MS].MD[t_row_MD].c_vector[0].DOB_gain  = eth_rx_data[10];
        		MotionMap_File.MS[t_row_MS].MD[t_row_MD].c_vector[0].IRC_gain = eth_rx_data[11];
        		MotionMap_File.MS[t_row_MS].MD[t_row_MD].c_vector[0].FC_gain = eth_rx_data[12];
        	}
        	else if (eth_rx_data[1] == 3) {
        		uint16_t t_row_MS = eth_rx_data[2]-1;
        		uint16_t t_row_MD = eth_rx_data[6]-1;
        		MotionMap_File.MS[t_row_MS].MS_ID             = eth_rx_data[2];
        		MotionMap_File.MS[t_row_MS].max_p_vectors_len = eth_rx_data[3];
        		MotionMap_File.MS[t_row_MS].max_f_vectors_len = eth_rx_data[4];
        		MotionMap_File.MS[t_row_MS].max_i_vectors_len = eth_rx_data[5];
        		for (int i = 0; i < HALF_N_P_VECTORS; i++) {
        			MotionMap_File.MS[t_row_MS].MD[t_row_MD].p_vector[i].yd = (eth_rx_data[6*i+7] << 8) | (eth_rx_data[6*i+8]);
        			MotionMap_File.MS[t_row_MS].MD[t_row_MD].p_vector[i].L  = (eth_rx_data[6*i+9] << 8) | (eth_rx_data[6*i+10]);
        			MotionMap_File.MS[t_row_MS].MD[t_row_MD].p_vector[i].s0 = eth_rx_data[6*i+11];
        			MotionMap_File.MS[t_row_MS].MD[t_row_MD].p_vector[i].sd = eth_rx_data[6*i+12];
        		}

        	}
        	else if (eth_rx_data[1] == 4) {
        		uint16_t t_row_MS = eth_rx_data[2]-1;
        		uint16_t t_row_MD = eth_rx_data[6]-1;
        		MotionMap_File.MS[t_row_MS].MS_ID             = eth_rx_data[2];
        		MotionMap_File.MS[t_row_MS].max_p_vectors_len = eth_rx_data[3];
        		MotionMap_File.MS[t_row_MS].max_f_vectors_len = eth_rx_data[4];
        		MotionMap_File.MS[t_row_MS].max_i_vectors_len = eth_rx_data[5];
        		for (int i = HALF_N_P_VECTORS; i < MAX_N_P_VECTORS; i++) {
        			MotionMap_File.MS[t_row_MS].MD[t_row_MD].p_vector[i].yd = (eth_rx_data[6*(i-5)+7] << 8) | (eth_rx_data[6*(i-5)+8]);
        			MotionMap_File.MS[t_row_MS].MD[t_row_MD].p_vector[i].L  = (eth_rx_data[6*(i-5)+9] << 8) | (eth_rx_data[6*(i-5)+10]);
        			MotionMap_File.MS[t_row_MS].MD[t_row_MD].p_vector[i].s0 = eth_rx_data[6*(i-5)+11];
        			MotionMap_File.MS[t_row_MS].MD[t_row_MD].p_vector[i].sd = eth_rx_data[6*(i-5)+12];
        		}

        	}
        	else if (eth_rx_data[1] == 5) {
        		uint16_t t_row_MS = eth_rx_data[2]-1;
        		uint16_t t_row_MD = eth_rx_data[6]-1;
        		MotionMap_File.MS[t_row_MS].MS_ID             = eth_rx_data[2];
        		MotionMap_File.MS[t_row_MS].max_p_vectors_len = eth_rx_data[3];
        		MotionMap_File.MS[t_row_MS].max_f_vectors_len = eth_rx_data[4];
        		MotionMap_File.MS[t_row_MS].max_i_vectors_len = eth_rx_data[5];
        		for (int i = 0; i < HALF_N_F_VECTORS; i++) {
        			MotionMap_File.MS[t_row_MS].MD[t_row_MD].f_vector[i].mode_idx = (eth_rx_data[6*i+7] << 8) | (eth_rx_data[6*i+8]);
        			MotionMap_File.MS[t_row_MS].MD[t_row_MD].f_vector[i].tau_max  = (eth_rx_data[6*i+9] << 8)  | (eth_rx_data[6*i+10]);
        			MotionMap_File.MS[t_row_MS].MD[t_row_MD].f_vector[i].delay    = (eth_rx_data[6*i+11] << 8) | (eth_rx_data[6*i+12]);
        		}

        	}
        	else if (eth_rx_data[1] == 6) {
        		uint16_t t_row_MS = eth_rx_data[2]-1;
        		uint16_t t_row_MD = eth_rx_data[6]-1;
        		MotionMap_File.MS[t_row_MS].MS_ID             = eth_rx_data[2];
        		MotionMap_File.MS[t_row_MS].max_p_vectors_len = eth_rx_data[3];
        		MotionMap_File.MS[t_row_MS].max_f_vectors_len = eth_rx_data[4];
        		MotionMap_File.MS[t_row_MS].max_i_vectors_len = eth_rx_data[5];
        		for (int i = HALF_N_F_VECTORS; i < MAX_N_F_VECTORS; i++) {
        			MotionMap_File.MS[t_row_MS].MD[t_row_MD].f_vector[i].mode_idx = (eth_rx_data[6*(i-5)+7] << 8) | (eth_rx_data[6*(i-5)+8]);
        			MotionMap_File.MS[t_row_MS].MD[t_row_MD].f_vector[i].tau_max  = (eth_rx_data[6*(i-5)+9] << 8)  | (eth_rx_data[6*(i-5)+10]);
        			MotionMap_File.MS[t_row_MS].MD[t_row_MD].f_vector[i].delay    = (eth_rx_data[6*(i-5)+11] << 8) | (eth_rx_data[6*(i-5)+12]);
        		}

        	}
        	else if (eth_rx_data[1] == 7) {
        		uint16_t t_row_MS = eth_rx_data[2]-1;
        		uint16_t t_row_MD = eth_rx_data[6]-1;
        		MotionMap_File.MS[t_row_MS].MS_ID             = eth_rx_data[2];
        		MotionMap_File.MS[t_row_MS].max_p_vectors_len = eth_rx_data[3];
        		MotionMap_File.MS[t_row_MS].max_f_vectors_len = eth_rx_data[4];
        		MotionMap_File.MS[t_row_MS].max_i_vectors_len = eth_rx_data[5];
        		for (int i = 0; i < HALF_N_I_VECTORS; i++) {
        			MotionMap_File.MS[t_row_MS].MD[t_row_MD].i_vector[i].epsilon_target = eth_rx_data[6*i+7];
        			MotionMap_File.MS[t_row_MS].MD[t_row_MD].i_vector[i].Kp_target      = eth_rx_data[6*i+8];
        			MotionMap_File.MS[t_row_MS].MD[t_row_MD].i_vector[i].Kd_target      = eth_rx_data[6*i+9];
        			MotionMap_File.MS[t_row_MS].MD[t_row_MD].i_vector[i].lambda_target  = eth_rx_data[6*i+10];
        			MotionMap_File.MS[t_row_MS].MD[t_row_MD].i_vector[i].duration       = (eth_rx_data[6*i+11] << 8) | (eth_rx_data[6*i+12]);
        		}

        	}
        	else if (eth_rx_data[1] == 8) {
        		uint16_t t_row_MS = eth_rx_data[2]-1;
        		uint16_t t_row_MD = eth_rx_data[6]-1;
        		MotionMap_File.MS[t_row_MS].MS_ID             = eth_rx_data[2];
        		MotionMap_File.MS[t_row_MS].max_p_vectors_len = eth_rx_data[3];
        		MotionMap_File.MS[t_row_MS].max_f_vectors_len = eth_rx_data[4];
        		MotionMap_File.MS[t_row_MS].max_i_vectors_len = eth_rx_data[5];
        		for (int i = HALF_N_I_VECTORS; i < MAX_N_I_VECTORS; i++) {
        			MotionMap_File.MS[t_row_MS].MD[t_row_MD].i_vector[i].epsilon_target = eth_rx_data[6*(i-5)+7];
        			MotionMap_File.MS[t_row_MS].MD[t_row_MD].i_vector[i].Kp_target      = eth_rx_data[6*(i-5)+8];
        			MotionMap_File.MS[t_row_MS].MD[t_row_MD].i_vector[i].Kd_target      = eth_rx_data[6*(i-5)+9];
        			MotionMap_File.MS[t_row_MS].MD[t_row_MD].i_vector[i].lambda_target  = eth_rx_data[6*(i-5)+10];
        			MotionMap_File.MS[t_row_MS].MD[t_row_MD].i_vector[i].duration       = (eth_rx_data[6*(i-5)+11] << 8) | (eth_rx_data[6*(i-5)+12]);
        		}

        	}
        }
        if ((RxData_Pre[0] == 5) & (equality == 1)) {   // End of transmission and saving
        	flag3 += 1;
            BeepAlert(BEEP_ALERT_FREQ_HIGH, BEEP_ALERT_DUTY, BEEP_ALERT_TIME_VERY_SHORT, BEEP_ALERT_TIME_VERY_SHORT);
            BeepAlert(BEEP_ALERT_FREQ_LOW,  BEEP_ALERT_DUTY, BEEP_ALERT_TIME_VERY_SHORT, BEEP_ALERT_TIME_VERY_SHORT);
            BeepAlert(BEEP_ALERT_FREQ_HIGH, BEEP_ALERT_DUTY, BEEP_ALERT_TIME_VERY_SHORT, BEEP_ALERT_TIME_VERY_SHORT);
            BeepAlert(BEEP_ALERT_FREQ_LOW,  BEEP_ALERT_DUTY, BEEP_ALERT_TIME_VERY_SHORT, BEEP_ALERT_TIME_VERY_SHORT);
            BeepAlert(BEEP_ALERT_FREQ_HIGH, BEEP_ALERT_DUTY, BEEP_ALERT_TIME_VERY_SHORT, BEEP_ALERT_TIME_VERY_SHORT);
            BeepAlert(BEEP_ALERT_FREQ_LOW,  BEEP_ALERT_DUTY, BEEP_ALERT_TIME_VERY_SHORT, BEEP_ALERT_TIME_VERY_SHORT);
            BeepAlert(BEEP_ALERT_FREQ_HIGH, BEEP_ALERT_DUTY, BEEP_ALERT_TIME_VERY_SHORT, BEEP_ALERT_TIME_VERY_SHORT);
            BeepAlert(BEEP_ALERT_FREQ_LOW,  BEEP_ALERT_DUTY, BEEP_ALERT_TIME_VERY_SHORT, BEEP_ALERT_TIME_VERY_SHORT);
        	Save_MotionMap();
        	Download_MotionMap();
        	Get_Max_PFI_Vectors_Length();

        }

        if (eth_rx_data[0] != 255) {
        	memcpy(RxData_Pre,eth_rx_data,64);
        }

        return 0;
}

#endif /* WALKON5_CM_ENABLED */
