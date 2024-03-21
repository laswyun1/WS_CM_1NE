
#include "WS_gait_ctrl.h"

#ifdef WALKON5_CM_ENABLED

/**
 *-----------------------------------------------------------
 *              TYPE DEFINITIONS AND ENUMERATIONS
 *-----------------------------------------------------------
 * @brief Enumerated types and structures central to this module.
 */




/**
 *------------------------------------------------------------
 *                          VARIABLES
 *------------------------------------------------------------
 * @brief Variables accessible throughout the application.
 */

TaskObj_t gaitCtrlTask;
uint32_t gaitCtrlLoopCnt;


/**
 *------------------------------------------------------------
 *                 STATIC FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Static Function prototypes for this module.
 */

/* ------------------- STATE FUNCTIONS ------------------- */
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


/* ------------------- SDO CALLBACK ------------------- */
DOP_COMMON_SDO_CB(gaitCtrlTask)

void InitGaitCtrl(void)
{
    // init
    InitTask(&gaitCtrlTask);
	DOPC_AssignTaskID(&gaitCtrlTask, TASK_IDX_GAIT_CTRL);

	/* State Definition */
	TASK_CREATE_STATE(&gaitCtrlTask, TASK_STATE_OFF,      NULL,				StateOff_Run,       NULL,				false);
	TASK_CREATE_STATE(&gaitCtrlTask, TASK_STATE_STANDBY,  NULL,				StateStandby_Run,	NULL,       		true);
	TASK_CREATE_STATE(&gaitCtrlTask, TASK_STATE_ENABLE,   StateEnable_Ent,	StateEnable_Run, 	StateEnable_Ext,  	false);
	TASK_CREATE_STATE(&gaitCtrlTask, TASK_STATE_ERROR,    NULL,				StateError_Run,    	NULL,				false);

	/* Routine Definition */

	/* DOD Definition */
	// DOD
	DOP_CreateDOD(TASK_IDX_GAIT_CTRL);

   	// PDO
	DOP_COMMON_PDO_CREATE(TASK_IDX_GAIT_CTRL, gaitCtrlTask);

	// SDO
	DOP_COMMON_SDO_CREATE(TASK_IDX_GAIT_CTRL)
}

void RunGaitCtrl(void)
{
	RunTask(&gaitCtrlTask);
}


/**
 *------------------------------------------------------------
 *                      STATIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions intended for internal use within this module.
 */

static void StateOff_Run(void)
{

}

static void StateStandby_Run(void)
{

}

static void StateEnable_Ent(void)
{
	// Ent_Routines(&gaitCtrlTask.routine);
}

static void StateEnable_Run(void)
{
	// Run_Routines(&gaitCtrlTask.routine);
}

static void StateEnable_Ext(void)
{
	// Ext_Routines(&gaitCtrlTask.routine);
}

static void StateError_Run(void)
{
	// TODO : Error Handle
}


#endif /* WALKON5_CM_ENABLED */
