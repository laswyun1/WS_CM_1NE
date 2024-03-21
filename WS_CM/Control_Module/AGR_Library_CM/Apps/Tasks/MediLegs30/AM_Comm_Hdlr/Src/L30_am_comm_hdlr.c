/*
 * am_comm_hdlr.c
 *
 *  Created on: Nov 22, 2023
 *      Author: HyundoKim
 */

#include "L30_am_comm_hdlr.h"

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

NetconnHandle hnet;

TaskObj_t AMCommHdlr;

/* Netconn Helper */
AMtoCMBuffer AMtoCMdatas;
CMtoAMBuffer CMtoAMdatas;

uint32_t mode_stack;
uint32_t state_stack;
uint8_t *rx_data;
uint8_t rx_array;
u16_t tx_len, rx_len;


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

static int32_t NetconnSrvCreate(void)
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

static int32_t NetconnSrvConnect(void)
{
    // Accept
    if (netconn_accept(hnet.srv, &hnet.cli) != ERR_OK) {
        return -4;
    }

    return 0;
}

static void NetconnEnd(void)
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

static void NetconnClearBuff(void)
{
    // This must be called before next Recv
    netbuf_delete(hnet.buff);
}

/* Data Object Protocol */
//static int32_t DOPSession()
//{
//
//    // Rx
//    NetconnRecv(&rx_data, &rx_len);
//
//    if (hnet.err != ERR_OK) {
//        return ETH_CTRL_TCP_RX_ERR;
//    }
//
//    if (DOP_Rx(rx_data, rx_len) < 0) {
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

static int32_t AMSession(void)
{
	NetconnRecv(&rx_data, &rx_len);
	memcpy(&AMtoCMdatas, rx_data, rx_len);
	if (AMtoCMdatas.Mode != 99){
		mode_stack = AMtoCMdatas.Mode;
	}
	if (AMtoCMdatas.State != 99){

		state_stack = AMtoCMdatas.State;

	}

    if (hnet.err != ERR_OK) {
        return ETH_CTRL_TCP_RX_ERR;
    }

    tx_len = ETH_CTRL_TX_BUFF_SIZE;


	memcpy(hnet.tx_msg, &CMtoAMdatas, ETH_CTRL_TX_BUFF_SIZE);

	NetconnSend(hnet.tx_msg, tx_len);

    if (hnet.err != ERR_OK) {
        return ETH_CTRL_TCP_TX_ERR;
    }

	NetconnClearBuff();

	return ETH_CTRL_OK;
}

#endif /* L30_CM_ENABLED */
