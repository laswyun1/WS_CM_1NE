

#include "AS_data_ctrl.h"

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

TaskObj_t dataCtrlTask;
uint32_t dataCtrlLoopCnt;

/* FATFS basic fnc. test */
IOIF_SD_Status_t sd_test_init_res = IOIF_MSD_ERROR;
IOIF_fCMD_res_t mount_res = FR_NOT_READY, open_res = FR_NOT_READY, read_res = FR_NOT_READY, write_res = FR_NOT_READY, sync_res = FR_NOT_READY;

IOIF_fCMD_res_t loopend_FileClose_res = FR_NOT_READY;

FATFS 		  fatfs_test __attribute__((section(".FATFS_RAMD1_data")));
IOIF_FILE_t   file_t	 __attribute__((section(".FATFS_RAMD1_data")));

uint32_t w_byte = 1;
uint32_t r_byte;

uint32_t writeFailCnt;
uint32_t mountFailCnt;

// mount time
TickType_t mountStartTime;
TickType_t mountEndTime;
TickType_t mountExecutionTime;
uint32_t mountExecutionTimeMs;
// for time
TickType_t forStartTime;
TickType_t forEndTime;
TickType_t forExecutionTime;
uint32_t forExecutionTimeMs;
// fwrite time
TickType_t fwriteStartTime;
TickType_t fwriteEndTime;
TickType_t fwriteExecutionTime;
uint32_t fwriteExecutionTimeMs;
// unmount time
TickType_t unmountStartTime;
TickType_t unmountEndTime;
TickType_t unmountExecutionTime;
uint32_t unmountExecutionTimeMs;


/* Data Storage Test : 20240128 */
//////////////////////////////////////////////////////////////////////////////////////////
char *title_str = "** AS Data Storage Test **\r\n";

uint32_t sd_storage_total_byte;						//size of storage byte
bool 	 file_ConsecutiveWrite = false;
//////////////////////////////////////////////////////////////////////////////////////////

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
DOP_COMMON_SDO_CB(dataCtrlTask)

/* ------------------------- MAIN ------------------------- */
void InitDataCtrl(void)
{
	 /* Init Task */
    InitTask(&dataCtrlTask);
	DOPC_AssignTaskID(&dataCtrlTask, TASK_IDX_DATA_CTRL);

	/* Init Device */

	/* State Definition */
	TASK_CREATE_STATE(&dataCtrlTask, TASK_STATE_OFF,      NULL,				StateOff_Run,       NULL,				false);
	TASK_CREATE_STATE(&dataCtrlTask, TASK_STATE_STANDBY,  NULL,				StateStandby_Run,	NULL,       		true);
	TASK_CREATE_STATE(&dataCtrlTask, TASK_STATE_ENABLE,   StateEnable_Ent,	StateEnable_Run, 	StateEnable_Ext,  	false);
	TASK_CREATE_STATE(&dataCtrlTask, TASK_STATE_ERROR,    NULL,				StateError_Run,    	NULL,				false);

	/* Routine Definition */

	/* DOD Definition */
	// DOD
	DOP_CreateDOD(TASK_IDX_DATA_CTRL);

   	// PDO
	DOP_COMMON_PDO_CREATE(TASK_IDX_DATA_CTRL, dataCtrlTask);

	// SDO
	DOP_COMMON_SDO_CREATE(TASK_IDX_DATA_CTRL)

    /* File-system_test */

	IOIF_WriteGPIOPin(IOIF_GPIO_PORT_G, IOIF_GPIO_PIN_5, IOIF_GPIO_PIN_SET); //Enable SD Card Power On

    sd_test_init_res = IOIF_FATFS_SD_Init(IOIF_SD1, (uint8_t*) "0:");

	if (sd_test_init_res == IOIF_MSD_OK) {
		mount_res = IOIF_FileMount(&fatfs_test, (uint8_t*)"0:");
	}

	if (mount_res == FR_OK) {
		write_res = IOIF_FileWrite(&file_t, (uint8_t*)"0:SUIT_H10_GAIT_DATA_01.txt", title_str, strlen(title_str), &w_byte);
	}

	if (mount_res == FR_OK) {
		mount_res = IOIF_FileUnmount((uint8_t*)"0:");
	}
	
	osDelay(1);
}

void RunDataCtrl(void)
{
	RunTask(&dataCtrlTask);
}


/* ----------------------- FUNCTION ----------------------- */

/**
 *------------------------------------------------------------
 *                      STATIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions intended for internal use within this module.
 */

/* -------------------- STATE MACHINE --------------------- */
static void StateOff_Run(void)
{

}

static void StateStandby_Run(void)
{
	dataCtrlLoopCnt = 0;
	if (wholeTaskFlag == 1) {
		if (mount_res == FR_OK) {
			mount_res = IOIF_FileMount(&fatfs_test, (uint8_t*)"0:");  		// Mount the file system
		}
		if (mount_res != FR_OK) {
			mountFailCnt++;
		}
		StateTransition(&dataCtrlTask.stateMachine, TASK_STATE_ENABLE);
	}
}

static void StateEnable_Ent(void)
{
	// Ent_Routines(&dataCtrlTask.routine);
}

static uint8_t led_turn_on = 0;

static void StateEnable_Run(void)
{
	/* Data Storage Test : 20240128 */
	osSemaphoreAcquire(sdio_sync_semaphore, osWaitForever);						// sync-up with sd buffer swap(switch),

	/* LED indicating for aging */
	//////////////////////////////////////////////
	// if((dataCtrlLoopCnt != 0) && !(dataCtrlLoopCnt % 20) && (writeFailCnt == 0))
	// 	led_turn_on ^= 0x01;
	// else if(mountFailCnt > 0 || writeFailCnt > 0)
	// 	led_turn_on = 0x02;			//error
	// else
	// 	led_turn_on = 0;			//test is done

	/*if(led_turn_on == 0)
		Drive_LED_Battery(BLUE, 0xFF);
	else if(led_turn_on == 1)
		Drive_LED_Battery(BLUE, 0x00);
	else
		Drive_LED_Battery(RED, 0xFF);*/
	//////////////////////////////////////////////

	if (mount_res == FR_OK && file_ConsecutiveWrite == false)
		open_res = IOIF_FileOpenCreateAppend(&file_t, (uint8_t*)"0:SUIT_H10_GAIT_DATA_01.txt");

	if (open_res == FR_OK) {
		if (dataCtrlLoopCnt < (12208)) { // 21MByte/(1ms*70Byte*45) = 6666.666Cnt (@1ms * MIN_TO_MILLISEC * SEND_DATA_SIZE(Byte) * DATA_SAVE_TIME(Min) : 21MByte Transfer within 5 Min.)
			osMutexAcquire(SDIOmutexHandle, BUFFER_SD_TIMEOUT);					// Mutex Lock

			if (open_res == FR_OK) {
				if(sd_buf_cur == sd_buf_1)
					write_res = IOIF_fWrite(&file_t, sd_buf_2, sizeof(sd_buf_2), &w_byte);
				else
					write_res = IOIF_fWrite(&file_t, sd_buf_1, sizeof(sd_buf_1), &w_byte);
			}

			sync_res = IOIF_FileSync(&file_t);									// file-sync

			if (sync_res == FR_OK)
				file_ConsecutiveWrite = true;
			else
				file_ConsecutiveWrite = false;									// retry file new-open

			osMutexRelease(SDIOmutexHandle);									// Mutex un-lock

			if (write_res != FR_OK) {
				writeFailCnt++;
			}

			dataCtrlLoopCnt++;
		} else {																// loop test done
			loopend_FileClose_res = IOIF_FileClose(&file_t);					// closed file if test is done
			mount_res = IOIF_FileUnmount((uint8_t*)"0:");						// file un-mount
		}

	}

	// Run_Routines(&dataCtrlTask.routine);
}

static void StateEnable_Ext(void)
{
	// Ext_Routines(&dataCtrlTask.routine);
}

static void StateError_Run(void)
{
	// TODO : Error Handle
}

/* --------------------- SDO CALLBACK --------------------- */

/* ----------------------- FUNCTION ----------------------- */

/* ----------------------- ROUTINE ------------------------ */

#endif /* SUIT_MINICM_ENABLED */
