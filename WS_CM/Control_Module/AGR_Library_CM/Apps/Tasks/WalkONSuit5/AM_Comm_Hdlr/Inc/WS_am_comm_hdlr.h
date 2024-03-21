#ifndef AM_COMM_HDLR_INC_WS_AM_COMM_HDLR_H_
#define AM_COMM_HDLR_INC_WS_AM_COMM_HDLR_H_

#include "module.h"

#ifdef WALKON5_CM_ENABLED

#include <math.h>

#include "robot_FSM.h"
#include "robot_DMS.h"
#include "robot_Setting.h"
#include "robot_motion_map.h"

#include "ioif_flash_common.h"

#include "lwip.h"
#include "lwip/api.h"
#include "cmsis_os.h"

#include "data_object_common.h"

/**
 *-----------------------------------------------------------
 *              MACROS AND PREPROCESSOR DIRECTIVES
 *-----------------------------------------------------------
 * @brief Directives and macros for readability and efficiency.
 */

#define ETH_CTRL_TCP_PORT 1818

#define ETH_CTRL_TX_BUFF_SIZE 64

#define ETH_CTRL_OK          0
#define ETH_CTRL_DOP_TX_ERR -1
#define ETH_CTRL_DOP_RX_ERR -2
#define ETH_CTRL_TCP_TX_ERR -3
#define ETH_CTRL_TCP_RX_ERR -4

#define DataSize 		64
#define MAX_N_FILE_TYPE 5
#define FSM_SIZE 		53
#define DMS_SIZE 		4


/**
 *------------------------------------------------------------
 *                     TYPE DECLARATIONS
 *------------------------------------------------------------
 * @brief Custom data types and structures for the module.
 */

typedef struct NetconnHandle {
	uint8_t tx_msg[ETH_CTRL_TX_BUFF_SIZE];
	err_t err;
	struct netconn *srv, *cli;
	struct netbuf *buff;
} NetconnHandle;

typedef struct CMtoAMBuffer {
	uint32_t temp1;
	uint32_t temp2;
	uint32_t temp3;
	uint32_t temp4;
	uint32_t temp5;
	uint32_t temp6;
	uint32_t temp7;
	uint32_t temp8;
	uint32_t temp9;
	uint32_t temp10;
	uint32_t temp11;
	uint32_t temp12;
	uint32_t temp13;
	uint32_t temp14;
	uint32_t temp15;
	uint32_t temp16;
} CMtoAMBuffer;


typedef struct AMtoCMBuffer {
	uint32_t temp1;
	uint32_t temp2;
	uint32_t temp3;
	uint32_t temp4;
	uint32_t temp5;
	uint32_t temp6;
	uint32_t temp7;
	uint32_t temp8;
	uint32_t temp9;
	uint32_t temp10;
	uint32_t temp11;
	uint32_t temp12;
	uint32_t temp13;
	uint32_t temp14;
	uint32_t temp15;
	uint32_t temp16;
} AMtoCMBuffer;

typedef struct _RobotSettingFileInfo_check
{
	uint8_t FileID;
	uint8_t mode;

	uint8_t robot_id;
	uint8_t file_version;

}__attribute__((packed))RobotSettingFileInfo_check;

typedef struct _RobotSettingData_check
{
	uint8_t FileID;
	uint8_t mode;

	uint8_t  usage;
	uint16_t FDCAN_ID;
	uint8_t  FDCAN_CH;

	DeviceName name;

}__attribute__((packed))RobotSettingData_check;

typedef struct _FSM1FileInfo_check
{
	uint8_t FileID;
	uint8_t mode;

	uint8_t robot_id;
	uint8_t file_version;

}__attribute__((packed))FSM1FileInfo_check;

typedef struct _FSM1Data_check
{
	uint8_t FileID;
	uint8_t mode;

	uint8_t  StateID;
	uint8_t  MotionSetID;
	uint32_t ActionID;
	uint16_t TimeOut;
	uint8_t  TabletModeID;
	uint8_t  DefaultTargetStateID;
	uint8_t  ExitConditionID[MAX_N_TRANSITION];
	uint8_t  TargetStateID[MAX_N_TRANSITION];

}__attribute__((packed))FSM1Data_check;

typedef struct _DMSFileInfo_check
{
	uint8_t FileID;
	uint8_t mode;

	uint8_t robot_id;
	uint8_t file_version;

}__attribute__((packed))DMSFileInfo_check;

typedef struct _DMSData_check
{
	uint8_t FileID;
	uint8_t mode;

	uint8_t Enable;
	uint8_t DeviceID;
	uint8_t CM_Save_Opt;
	uint8_t AM_Send_Opt;

}__attribute__((packed))DMSData_check;

typedef struct _MotionMapFileInfo_check
{
	uint8_t FileID;
	uint8_t mode;

	uint8_t robot_id;
	uint8_t file_version;
	uint16_t num_ms;
	uint16_t cnt;
	uint8_t send_state;

}__attribute__((packed))MotionMapFileInfo_check;

typedef struct _MotionMapCtrlSetting_check
{
	uint8_t FileID;
	uint8_t mode;

	uint8_t MS_ID;
	uint8_t MD_ID;

	uint8_t FF;
	uint8_t PD;
	uint8_t IC;
	uint8_t DOB;
	uint8_t IRC;
	uint8_t FC;

}__attribute__((packed))MotionMapCtrlSetting_check;

typedef struct _MotionMapPVector_check
{
	uint8_t FileID;
	uint8_t mode;

	uint8_t MS_ID;
	uint8_t MD_ID;

	P_Vector p_vector[10];

}__attribute__((packed))MotionMapPVector_check;

typedef struct _MotionMapFVector_check
{
	uint8_t FileID;
	uint8_t mode;

	uint8_t MS_ID;
	uint8_t MD_ID;

	F_Vector f_vector[10];

}__attribute__((packed))MotionMapFVector_check;

typedef struct _MotionMapIVector_check
{
	uint8_t FileID;
	uint8_t mode;

	uint8_t MS_ID;
	uint8_t MD_ID;

	I_Vector i_vector[10];

}__attribute__((packed))MotionMapIVector_check;


/**
 *------------------------------------------------------------
 *                      GLOBAL VARIABLES
 *------------------------------------------------------------
 * @brief Extern declarations for global variables.
 */

extern CMtoAMBuffer CMtoAMdatas;
extern AMtoCMBuffer AMtoCMdatas;
extern uint8_t *eth_rx_data;
extern uint8_t rx_array;
extern uint16_t tx_len, rx_len;
extern uint32_t mode_stack;
extern uint32_t state_stack;


/**
 *------------------------------------------------------------
 *                     FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Function prototypes declaration for this module.
 */

// Main
void InitAMCommHdlr(void);
void RunAMCommHdlr(void);


#endif /* WALKON5_CM_ENABLED */

#endif /* AM_COMM_HDLR_INC_WS_AM_COMM_HDLR_H_ */
