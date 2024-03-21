

#include "AS_ble_comm_hdlr.h"

#ifdef SUIT_MINICM_ENABLED

/**
 *-----------------------------------------------------------
 *              TYPE DEFINITIONS AND ENUMERATIONS
 *-----------------------------------------------------------
 * @brief Enumerated types and structures central to this module.
 */

//Test code
#define BLE_SLAVE_WRITE_NAME_COMMAND_2 	"AT+NAMEAngelSUIT_Test"


/**
 *------------------------------------------------------------
 *                      GLOBAL VARIABLES
 *------------------------------------------------------------
 * @brief Variables accessible throughout the application.
 */
TaskObj_t BLECommHdlr;

static bool ble_init_status = false;

/*Test Code*/
uint8_t ble_debug=0;
#define TEST_SEND_PACKET		"Send Test Packet transmit, BLE is connected successfully!!\r\n"

BLEState isBLEConnect;

uint8_t powerOffCmdByBLE = 0;


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

/* --------------------- SDO CALLBACK --------------------- */

/* ----------------------- FUNCTION ----------------------- */
static bool BLEInit(uint32_t baudrate);

/* ----------------------- ROUTINE ------------------------ */

/* -------------------- STATE FUNCTION -------------------- */
static void StateOff_Ent(void);

static void StateStandby_Ent(void);

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

/* ------------------------- MAIN ------------------------- */
void InitBLEComm()
{
    /* Init Task */
    InitTask(&BLECommHdlr);
    DOPC_AssignTaskID(&BLECommHdlr, TASK_IDX_BLE_COMM_HDLR);

    /* Init Device */
    ble_init_status = BLEInit(9600);			//BLE device identifier + UART start

    /* State Definition */
    TASK_CREATE_STATE(&BLECommHdlr, TASK_STATE_OFF,     StateOff_Ent,     NULL,            	NULL, 			 true);
    TASK_CREATE_STATE(&BLECommHdlr, TASK_STATE_STANDBY, StateStandby_Ent, NULL,				NULL, 			 false);
    TASK_CREATE_STATE(&BLECommHdlr, TASK_STATE_ENABLE,  StateEnable_Ent,  StateEnable_Run, 	StateEnable_Ext, false);
    TASK_CREATE_STATE(&BLECommHdlr, TASK_STATE_ERROR,   NULL,             StateError_Run,  	NULL, 			 false);

	/* Routine Definition */

    /* DOD Definition */
	// DOD
    //DOP_CreateDOD(TASK_IDX_BLE_COMM_HDLR);

    // PDO
    //DOP_COMMON_PDO_CREATE(TASK_IDX_BLE_COMM_HDLR, BLECommHdlr);

    // SDO
    //COMMON_SDO_CREATE(TASK_IDX_BLE_COMM_HDLR)

	isBLEConnect = BLE_PARING;
}

void RunBLEComm(void)
{
    RunTask(&BLECommHdlr);

	/*Test Code*/
	static uint8_t ch = 1;
	uint8_t tick = 50;

	/*Test Code*/
	if (ble_debug == 1)
	{
		IOIF_BLE_Write_GPIO_State(MDBT42Q_GPIO_STATUS_LOW);
		osDelay(tick);
		IOIF_UART_Write(ch, IOIF_UART_MODE_POLLING,  (uint8_t*)BLE_SLAVE_READ_NAME, 	sizeof(BLE_SLAVE_READ_NAME));
		osDelay(250);
		IOIF_BLE_Write_GPIO_State(MDBT42Q_GPIO_STATUS_HIGH);

		IOIF_BLE_Write_GPIO_State(MDBT42Q_GPIO_STATUS_LOW);
		osDelay(tick);
		IOIF_UART_Write(ch, IOIF_UART_MODE_POLLING,  (uint8_t*)BLE_SLAVE_WRITE_BAUD9600, 	sizeof(BLE_SLAVE_WRITE_BAUD9600));
		//		IOIF_UART_Write(ch, IOIF_UART_MODE_POLLING,  (uint8_t*)AT_WRITE_BAUDRATE115200, 	sizeof(AT_WRITE_BAUDRATE115200));
		osDelay(250);
		IOIF_BLE_Write_GPIO_State(MDBT42Q_GPIO_STATUS_HIGH);

		IOIF_BLE_Write_GPIO_State(MDBT42Q_GPIO_STATUS_LOW);
		osDelay(tick);
		IOIF_UART_Write(ch, IOIF_UART_MODE_POLLING,  (uint8_t*)BLE_SLAVE_READ_BAUDRATE, 	sizeof(BLE_SLAVE_READ_BAUDRATE));
		osDelay(250);
		IOIF_BLE_Write_GPIO_State(MDBT42Q_GPIO_STATUS_HIGH);

		ble_debug = 0;

	} else if (ble_debug == 2) {
		MDBT42Q_AT_ReadCMD((uint8_t*)&BLE_SLAVE_READ_NAME[0], sizeof(BLE_SLAVE_READ_NAME));
		//		MDBT42Q_AT_WriteCMD((uint8_t*)&BLE_SLAVE_WRITE_BAUD9600[0], sizeof(BLE_SLAVE_WRITE_BAUD9600), Save_Active);
		MDBT42Q_AT_WriteCMD((uint8_t*)&BLE_SLAVE_WRITE_BAUD115200[0], sizeof(BLE_SLAVE_WRITE_BAUD115200), Save_Active);
		MDBT42Q_AT_ReadCMD((uint8_t*)&BLE_SLAVE_READ_BAUDRATE[0], sizeof(BLE_SLAVE_READ_BAUDRATE));
		ble_debug = 0;

	} else if (ble_debug == 3) {
//		IOIF_BLE_ConfigParameters_Set();
		MDBT42Q_AT_WriteCMD((uint8_t*)&BLE_SLAVE_WRITE_NAME_COMMAND_2, sizeof(BLE_SLAVE_WRITE_NAME_COMMAND_2), Save_Active);
		ble_debug = 0;

	} else if (ble_debug == 4) {

		IOIF_BLE_Ready();
		IOIF_UART_Start(IOIF_UART_1, IOIF_UART_MODE_DMA, 115200);
		ble_debug = 0;

	} else if (ble_debug == 5) {
		IOIF_BLE_Ready();
		IOIF_UART_Start(IOIF_UART_1, IOIF_UART_MODE_DMA, 9600);
		ble_debug = 0;

	} else if (ble_debug == 6) {
//		MDBT42Q_AT_ReadCMD((uint8_t*)&BLE_SLAVE_READ_BAUDRATE[0], sizeof(BLE_SLAVE_READ_BAUDRATE));

		IOIF_BLE_Write_GPIO_State(MDBT42Q_GPIO_STATUS_LOW);
		osDelay(tick);
		IOIF_UART_Write(ch, IOIF_UART_MODE_POLLING,  (uint8_t*)BLE_SLAVE_READ_NAME, 	sizeof(BLE_SLAVE_READ_NAME));
		osDelay(250);
		IOIF_BLE_Write_GPIO_State(MDBT42Q_GPIO_STATUS_HIGH);

		ble_debug = 0;

	}


}

/* ----------------------- FUNCTION ----------------------- */

/**
 *------------------------------------------------------------
 *                      STATIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions intended for internal use within this module.
 */

/* -------------------- STATE MACHINE --------------------- */
static void StateOff_Ent(void)
{
    // No OFF state. Transit to Standby automatically.
	StateTransition(&BLECommHdlr.stateMachine, TASK_STATE_STANDBY);
}

static void StateStandby_Ent(void)
{
    // No OFF state. Transit to Enable automatically.
	StateTransition(&BLECommHdlr.stateMachine, TASK_STATE_ENABLE);
}

static void StateEnable_Ent(void)
{
    EntRoutines(&BLECommHdlr.routine);
}

static void StateEnable_Run(void)
{
	//Todo : 테스트 코드 말고 사용할 수 있도록
	if (IOIF_IsBLEConnected() == true) {
		isBLEConnect = BLE_CONNECTED;
		IOIF_BLE_DataWrite_Test();
		IOIF_BLE_DataRead_Test();
		//			IOIF_BLE_Ack_Test();
	}
	//	MDBT42Q_TransmitData((uint8_t*)IOIF_BLE_TEST_SEND_PACKET, sizeof(IOIF_BLE_TEST_SEND_PACKET));
}

static void StateEnable_Ext(void)
{
    ExtRoutines(&BLECommHdlr.routine);
}

static void StateError_Run(void)
{

}

/* --------------------- SDO CALLBACK --------------------- */

/* ----------------------- FUNCTION ----------------------- */

static bool BLEInit(uint32_t baudrate)
{
	volatile bool ret = false;

	IOIF_BLE_ConnectGPIO_CB_Set();
	ret = IOIF_BLE_Ready();

	/* UART Start */
	if (IOIF_UART_Start(IOIF_UART_1, IOIF_UART_MODE_DMA, baudrate) != IOIF_UART_START_OK)			// Transmit : Normal, Received : DMA
	{ return ret = false;}
//	ret = IOIF_BLE_ConfigParameters_Set();

	return ret;
}

/* ----------------------- ROUTINE ------------------------ */

#endif /* SUIT_MINICM_ENABLED */
