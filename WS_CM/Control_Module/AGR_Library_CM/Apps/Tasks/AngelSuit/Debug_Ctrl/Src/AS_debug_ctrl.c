

#include "AS_debug_ctrl.h"

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
TaskObj_t debugCtrlTask;

uint32_t debug_task_loop_time;

extern uint32_t imuCtrlLoopCnt;			//Imu ctrl loop time check
extern uint32_t SysCtrlLoopCnt;			//Sys ctrl loop time check
extern IOIF_BatData_t batData;
extern uint8_t IsI2Crecovered;
extern uint8_t bmInitRes;
extern uint8_t IsRecoverI2C4;

IOIF_WavPlayState_t audio_test_init = IOIF_WAVPLAY_STATUS_ERROR, audio_play_test = IOIF_WAVPLAY_STATUS_ERROR;
uint8_t audio_outmode = 0;


/* FATFS basic fnc. test */

// IOIF_SD_Status_t sd_test_init_res = IOIF_MSD_ERROR;
// IOIF_fCMD_res_t mount_res = FR_NOT_READY, open_res = FR_NOT_READY, read_res = FR_NOT_READY, write_res= FR_NOT_READY;
IOIF_fCMD_res_t mount_usb_res = FR_NOT_READY, open_usb_res = FR_NOT_READY, read_usb_res = FR_NOT_READY, write_usb_res= FR_NOT_READY;

// FATFS 		  fatfs_test, fatfs_usb_test;
// IOIF_FILE_t   file_t, file_usb_t;

uint8_t SD_readData[30];
uint8_t SD_writeData[30] = "A10_WRITE_TEST_OK!\r\n";

// uint32_t r_byte, w_byte;
bool usb_test_write_state = 0;

float disk_free, disk_free_usb;


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
 * @brief Function prototypes for this module.
 */
/* --------------------- SDO CALLBACK --------------------- */

/* ----------------------- FUNCTION ----------------------- */
static void SendTestMsg(void);

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
void InitDebugTask()
{
	/* Init Task */
	InitTask(&debugCtrlTask);
	DOPC_AssignTaskID(&debugCtrlTask, TASK_IDX_DEBUG_CTRL);

	/* Init Device */
	IOIF_InitUSB(IOIF_USBD_CDC, IOIF_USB_TIM_NULL);
	//     IOIF_InitUSB(IOIF_USBH_MSC, IOIF_USB_TIM_NULL);

	/* State Definition */
	TASK_CREATE_STATE(&debugCtrlTask, TASK_STATE_OFF,     StateOff_Ent,     NULL,            NULL,               false);
	TASK_CREATE_STATE(&debugCtrlTask, TASK_STATE_STANDBY, StateStandby_Ent, NULL,            NULL,               true);
	TASK_CREATE_STATE(&debugCtrlTask, TASK_STATE_ENABLE,  StateEnable_Ent,  StateEnable_Run, StateEnable_Ext,    false);
	TASK_CREATE_STATE(&debugCtrlTask, TASK_STATE_ERROR,   NULL,             StateError_Run,  NULL,               false);

	/* Routine Definition */
	// TASK_CREATE_ROUTINE(&debugCtrlTask,   1, NULL,   	Send_Debug_Msg,  	NULL);

	/* DOD Definition */
	// DOD
	// DOP_CreateDOD(TASK_IDX_DEBUG_CTRL);

	// PDO
	// DOP_COMMON_PDO_CREATE(TASK_IDX_DEBUG_CTRL, debugCtrlTask);

	// SDO
	// DOP_COMMON_SDO_CREATE(TASK_IDX_DEBUG_CTRL)


	/* File-system_test */

	//    sd_test_init_res = IOIF_FATFS_SD_Init(IOIF_SD1, (uint8_t*) "0:");
	//
	//    mount_res = IOIF_FileMount(&fatfs_test, (uint8_t*)"0:");
	//    //mount_usb_res = IOIF_FileMount(&fatfs_usb_test, (uint8_t*)"1:");
	//
	//    disk_free = IOIF_Disk_GetFreeSpace((uint8_t*)"0:", &fatfs_test);
	//    //disk_free_usb = IOIF_Disk_GetFreeSpace((uint8_t*)"1:", &fatfs_usb_test);
	//
	//    read_res      = IOIF_FileRead(&file_t, (uint8_t*)"0:A10_ioif_test.txt", SD_readData, sizeof(SD_readData), &r_byte);
	//    //write_res	  = IOIF_FileWrite(&file_t, (uint8_t*)"0:A10_ioif_test2.txt", SD_writeData, sizeof(SD_writeData), &w_byte);
	//    //write_usb_res = IOIF_FileWrite(&file_usb_t, (uint8_t*)"1:A10_ioif_test.txt", SD_readData, sizeof(SD_readData), &w_byte);



	//     audio_test_init = WavAudio_FS_Init((uint8_t*)"0:", IOIF_WAVPLAY_SD);	 //audio init.

	//PlayWaveFile((uint8_t*)"0:", (uint8_t*)"0:/voice1102/01_system_ready.wav");

	//audio_play_test = PlayWaveFile((uint8_t*)"0:", (uint8_t*)"0:/voice1102/01_system_ready.wav"); //wave file play
}

void RunDebugTask(void)
{
	RunTask(&debugCtrlTask);
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
	StateTransition(&debugCtrlTask.stateMachine, TASK_STATE_STANDBY);
}

static void StateStandby_Ent(void)
{
	//     PlayWaveFile((uint8_t*)"0:", (uint8_t*)"0:/voice1102/01_system_ready.wav");
	// 	osDelay(3000);
	StateTransition(&debugCtrlTask.stateMachine, TASK_STATE_ENABLE);
}

static void StateEnable_Ent(void)
{
	debug_task_loop_time = 0;
	// Ent_Routines(&debugCtrlTask.routine);
}

static void StateEnable_Run(void)
{
	SendTestMsg();
	//    if(audio_test_init == IOIF_WAVPLAY_STATUS_OK && audio_outmode == 1)
	//    {
	//PlayWaveFile((uint8_t*)"0:", (uint8_t*)"0:/voice1102/01_system_ready.wav");
	//    	audio_outmode = 0;
	//    }

	//osDelay(3000);
	//Run_Routines(&debugCtrlTask.routine);
	debug_task_loop_time++;
}

static void StateEnable_Ext(void)
{

}

static void StateError_Run(void)
{

}

/* --------------------- SDO CALLBACK --------------------- */

/* ----------------------- FUNCTION ----------------------- */

static void SendTestMsg(void)
{
	/* IOIF_USBD_Printf((uint8_t*)"%d %f %f %f \r\n", 	\
			 debug_task_loop_time, 		\
			 test_roll, 				\
			 test_pitch,				\
			 test_yaw					);*/
}

/* ----------------------- ROUTINE ------------------------ */
#endif /* SUIT_MINICM_ENABLED */
