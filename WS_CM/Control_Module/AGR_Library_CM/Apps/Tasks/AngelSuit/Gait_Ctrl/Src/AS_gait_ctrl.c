

#include "AS_gait_ctrl.h"

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

TaskObj_t gaitCtrlTask;

GaitObj_t gaitObj_RH;
GaitObj_t gaitObj_LH;
GaitObj_t gaitObj_RK;
GaitObj_t gaitObj_LK;

WIDM_BPFState   BPFState_Deg_RH;
WIDM_BPFState   BPFState_Deg_LH;
WIDM_BPFState   BPFState_Vel_RH;
WIDM_BPFState   BPFState_Vel_LH;
WIDM_LPFState   LPFState_RH;
WIDM_LPFState   LPFState_LH;

WIDM_BPFState   BPFState_Deg_RK;
WIDM_BPFState   BPFState_Deg_LK;
WIDM_BPFState   BPFState_Vel_RK;
WIDM_BPFState   BPFState_Vel_LK;
WIDM_LPFState   LPFState_RK;
WIDM_LPFState   LPFState_LK;


/**
 *------------------------------------------------------------
 *                      STATIC VARIABLES
 *------------------------------------------------------------
 * @brief Variables local to this module.
 */

// Loop Time Count //
static uint32_t gaitCtrlLoopCnt;


/**
 *------------------------------------------------------------
 *                 STATIC FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Static Function prototypes for this module.
 */

/* --------------------- SDO CALLBACK --------------------- */

/* ----------------------- FUNCTION ----------------------- */
// Prof //
static void InitGaitFunction(GaitObj_t* gaitObj);
static void UpdateSAMData(void);
static void RunGaitFunction_K10(GaitObj_t* gaitObj, ISI_Flag_t* ISIFlags);
static void RunGaitFunction_H10(GaitObj_t* gaitObj, ISI_Flag_t* ISIFlags, WIDM_BPFState* BPFState_Deg, WIDM_BPFState* BPFState_Vel, WIDM_LPFState* LPFState);
static void CheckWalkingState_Prof(void);

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
DOP_COMMON_SDO_CB(gaitCtrlTask)

/* ------------------------- MAIN ------------------------- */
void InitGaitCtrl(void)
{
	/* Init Task */
	InitTask(&gaitCtrlTask);
	DOPC_AssignTaskID(&gaitCtrlTask, TASK_IDX_GAIT_CTRL);

	/* Init Device */

	/* State Definition */
	TASK_CREATE_STATE(&gaitCtrlTask, TASK_STATE_OFF,      NULL,   			StateOff_Run,       NULL,         		 true);
	TASK_CREATE_STATE(&gaitCtrlTask, TASK_STATE_STANDBY,  NULL,   			StateStandby_Run,	NULL,         		 false);
	TASK_CREATE_STATE(&gaitCtrlTask, TASK_STATE_ENABLE,   StateEnable_Ent,	StateEnable_Run, 	StateEnable_Ext,	 false);
	TASK_CREATE_STATE(&gaitCtrlTask, TASK_STATE_ERROR,    NULL,				StateError_Run,    	NULL,				 false);

	/* Routine Definition */
	// TASK_CREATE_ROUTINE(&gaitCtrlTask, ROUTINE_ID_SYSMNGT_GET_POWER_VALUE, 	NULL, 	GetBatVal_Run, 	NULL);

	/* DOD Definition */
	// DOD
	DOP_CreateDOD(TASK_IDX_GAIT_CTRL);

   	// PDO
	DOP_COMMON_PDO_CREATE(TASK_IDX_GAIT_CTRL, gaitCtrlTask);
	// DOP_CreatePDO(TASK_IDX_GAIT_CTRL, object_id, DOP_FLOAT32, length, pDataAddr);

	// SDO
	DOP_COMMON_SDO_CREATE(TASK_IDX_GAIT_CTRL)
	// DOP_CreateSDO(TASK_IDX_GAIT_CTRL, object_id, DOP_FLOAT32, SDOCallback);
#ifdef SUIT_H10
	InitGaitFunction(&gaitObj_RH);
	InitGaitFunction(&gaitObj_LH);
#endif
#ifdef SUIT_K10
	InitGaitFunction(&gaitObj_RK);
	InitGaitFunction(&gaitObj_LK);
#endif
#ifdef SUIT_A10

#endif

}

void RunGaitCtrl(void)
{
	RunTask(&gaitCtrlTask);
}

/* ----------------------- FUNCTION ----------------------- */

/**
 *------------------------------------------------------------
 *                      STATIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions intended for internal use within this module.
 */

/* -------------------- STATE FUNCTION -------------------- */

static void StateOff_Run(void)
{
    StateTransition(&gaitCtrlTask.stateMachine, TASK_STATE_STANDBY);
}

static void StateStandby_Run(void)
{
    StateTransition(&gaitCtrlTask.stateMachine, TASK_STATE_ENABLE);
}

static void StateEnable_Ent(void)
{
	gaitCtrlLoopCnt = 0;
	EntRoutines(&gaitCtrlTask.routine);
}

static void StateEnable_Run(void)
{
	UpdateSAMData();

#ifdef SUIT_H10
	RunGaitFunction_H10(&gaitObj_RH, &ISIFlags_RH, &BPFState_Deg_RH, &BPFState_Vel_RH, &LPFState_RH);
	RunGaitFunction_H10(&gaitObj_LH, &ISIFlags_LH, &BPFState_Deg_LH, &BPFState_Vel_LH, &LPFState_LH);
#endif

#ifdef SUIT_K10
	RunGaitFunction_K10(&gaitObj_RK, &ISIFlags_RK);
	RunGaitFunction_K10(&gaitObj_LK, &ISIFlags_LK);
#endif

#ifdef SUIT_A10

#endif

	RunRoutines(&gaitCtrlTask.routine);
	gaitCtrlLoopCnt++;
}

static void StateEnable_Ext(void)
{
    gaitCtrlLoopCnt = 0;
    ExtRoutines(&gaitCtrlTask.routine);
}

static void StateError_Run(void)
{

}

/* --------------------- SDO CALLBACK --------------------- */

/* ----------------------- FUNCTION ----------------------- */
static void InitGaitFunction(GaitObj_t* gaitObj)
{
	gaitObj->GaitFilter.cutoffFreq = 6.0;
	gaitObj->GaitFilter.cutoffFreqSmooth = 6.0;
	gaitObj->GaitFilter.PeakAmp = 10;
	gaitObj->GaitFilter.PeakAmpSmooth = 10;
	gaitObj->GaitFilter.PeakWAmp = 70;
	gaitObj->GaitFilter.PeakWAmpSmooth = 70;

	gaitObj->Period = 2000;
	gaitObj->firstPeriodCheck = 1;
	gaitObj->phaseThreshold1 = 10;
	gaitObj->phaseThreshold2 = 20;
	gaitObj->phaseThreshold3 = 30;
	gaitObj->phaseThreshold4 = 40;
	gaitObj->phaseThreshold5 = 50;
	gaitObj->phaseThreshold6 = 60;
	gaitObj->phaseThreshold7 = 88;
	gaitObj->phaseThreshold8 = 80;
	gaitObj->phaseThreshold9 = 90;
}

static void UpdateSAMData(void)
{
#ifdef SUIT_H10
	gaitObj_RH.AngleData.degFinal = userCtrlObj[AS_MD_IDX_1].data.SAM_degfinal;
	gaitObj_RH.VelData.velFinal = userCtrlObj[AS_MD_IDX_1].data.SAM_velfinal;

	gaitObj_LH.AngleData.degFinal = userCtrlObj[AS_MD_IDX_2].data.SAM_degfinal;
	gaitObj_LH.VelData.velFinal = userCtrlObj[AS_MD_IDX_2].data.SAM_velfinal;
#endif
#ifdef SUIT_K10
	gaitObj_RK.AngleData.degFinal = userCtrlObj[AS_MD_IDX_1].data.SAM_degfinal;
	gaitObj_RK.AngleData.incDegRaw = userCtrlObj[AS_MD_IDX_1].data.SAM_degINC;
	gaitObj_RK.VelData.velFinal = userCtrlObj[AS_MD_IDX_1].data.SAM_velfinal;
	gaitObj_RK.VelData.incVelRaw = userCtrlObj[AS_MD_IDX_1].data.SAM_velINC;
	gaitObj_RK.gyrZRaw[0] = userCtrlObj[AS_MD_IDX_1].data.SAM_gyrZ;

	gaitObj_LK.AngleData.degFinal = userCtrlObj[AS_MD_IDX_2].data.SAM_degfinal;
	gaitObj_LK.AngleData.incDegRaw = userCtrlObj[AS_MD_IDX_2].data.SAM_degINC;
	gaitObj_LK.VelData.velFinal = userCtrlObj[AS_MD_IDX_2].data.SAM_velfinal;
	gaitObj_LK.VelData.incVelRaw = userCtrlObj[AS_MD_IDX_2].data.SAM_velINC;
	gaitObj_LK.gyrZRaw[0] = userCtrlObj[AS_MD_IDX_2].data.SAM_gyrZ;
#endif
#ifdef SUIT_K10

#endif
}

static void RunGaitFunction_K10(GaitObj_t* gaitObj, ISI_Flag_t* ISIFlags)
{
	gaitObj->gaitFuncLoopCnt++; 			                			// Check this Loop Count(1ms)
	gaitObj->gaitFuncLoopCnt_1sec = gaitObj->gaitFuncLoopCnt / 1000;    // For Debug, check each 1sec

	gaitObj->VelData.velLPF_K10 = WIDM_LPF_walking_Prof(gaitObj->VelData.velFinal);    // velLPF : velRaw -> Low-pass filtering
	gaitObj->VelData.velLPFAbs = WIDM_Abs_double(gaitObj->VelData.velLPF_K10);
	double tempVelLPFAbs = WIDM_GetMaxValue_double(0, gaitObj->VelData.velLPFAbs-15);

	// GetINCLinkData(&widmAngleDataObj);

	if (gaitObj->gaitFuncLoopCnt > 1) {
		gaitObj->AngleData.incDegLPF[0] = 0.98 * gaitObj->AngleData.incDegLPF[1] + 0.02 * gaitObj->AngleData.incDegRaw;
	}

//	gaitObj->AngleData.incDegTrig[0] = gaitObj->AngleData.incDegRaw;
//	gaitObj->VelData.incVel = gaitObj->VelData.incVelRaw;
//
//	gaitObj->AngleData.filteredIncDeg = WIDM_Abs_double(gaitObj->AngleData.incDegRaw);
//	gaitObj->AngleData.filteredIncDeg = WIDM_LPF_walking_Prof(gaitObj->AngleData.incDegRaw);
//	gaitObj->AngleData.incDegTrig[0] = gaitObj->AngleData.filteredIncDeg;
//
//	gaitObj->VelData.filteredIncVel = WIDM_Abs_double(gaitObj->VelData.incVel);
//	gaitObj->VelData.filteredIncVel = WIDM_LPF_walking_Prof(gaitObj->VelData.incVel);
//	gaitObj->VelData.incVelTrig[0] = gaitObj->VelData.filteredIncVel;

	gaitObj->GaitMode.modeCheck = 0.993 * gaitObj->GaitMode.modeCheck + 0.007 * tempVelLPFAbs;

	if (gaitObj->gaitFuncLoopCnt > 1){
		gaitObj->AngleData.degLPF[0] = 0.98 * gaitObj->AngleData.degLPF[1] + 0.02 * gaitObj->AngleData.degFinal;

		gaitObj->GaitMode.mode[0] = gaitObj->GaitMode.mode[1];

		if (gaitObj->gaitFuncLoopCnt > 1000){
			if (gaitObj->GaitMode.mode[1] == 0 && gaitObj->GaitMode.modeCheck > 4){     // mode = 1 : Walking, mode = 0 : Stop  // modeCheck = 4
				gaitObj->GaitMode.mode[0] = 1;
			}
			else if (gaitObj->GaitMode.mode[1] == 1 && gaitObj->GaitMode.modeCheck < 1){
				gaitObj->GaitMode.mode[0] = 0;
			}
		}

		gaitObj->gyroLPF[0] = 0.98 * gaitObj->gyroLPF[1] + 0.02 * gaitObj->gyrZRaw[0];
//		gaitObj->gyroLPF[0] = 0.997 * gaitObj->gyroLPF[1] + 0.003 * gaitObj->gyrZRaw[0];		// 1NE changed
		gaitObj->VelData.velLPF2[0] = 0.9997 * gaitObj->VelData.velLPF2[1] + 0.0003 * gaitObj->gyroLPF[0];
	}

	if (gaitObj->gaitFuncLoopCnt > 10){
		if (gaitObj->GaitMode.mode[0] == 1){
			if (gaitObj->VelData.velLPF2[1] > 0 && gaitObj->VelData.velLPF2[0] < 0){								// Leg front->back moving point
				if (gaitObj->firstPeriodCheck == 0){
					gaitObj->Period = gaitObj->gaitFuncLoopCnt - gaitObj->timeStampPrev;
					if (gaitObj->Period > 2000){
						gaitObj->Period = 2000;
					}
					else if (gaitObj->Period < 200){
						gaitObj->Period = 200;
					}
				}
				else{
					gaitObj->firstPeriodCheck = 0;
				}

				gaitObj->GaitFilter.cutoffFreq = 2 * WIDM_PI / (gaitObj->Period * 0.001);
				gaitObj->timeStampPrev = gaitObj->gaitFuncLoopCnt;
			}
		}
		else if (gaitObj->GaitMode.mode[0] == 0){
			gaitObj->firstPeriodCheck = 1;
		}
	}

	gaitObj->GaitFilter.cutoffFreqSmooth = 0.992 * gaitObj->GaitFilter.cutoffFreqSmooth + 0.008 * gaitObj->GaitFilter.cutoffFreq;
	gaitObj->AngleData.degBPF[0] = WIDM_BPF_Peak_Prof_1(gaitObj->AngleData.degFinal, gaitObj->GaitFilter.cutoffFreqSmooth);
	gaitObj->AngleData.degBPF0ABS = WIDM_Abs_double(gaitObj->AngleData.degBPF[0]);

	if (gaitObj->gaitFuncLoopCnt > 3){
		if (gaitObj->GaitMode.mode[0] == 1 && gaitObj->firstPeriodCheck == 0 && gaitObj->AngleData.degBPF[1] < gaitObj->AngleData.degBPF[2] && gaitObj->AngleData.degBPF[0] > gaitObj->AngleData.degBPF[1]){
			gaitObj->GaitFilter.PeakAmp = gaitObj->AngleData.degBPF0ABS;	// At Leg Backward->Forward transition, get Amplitude of angle(deg)
		}

		gaitObj->VelData.velBPF = WIDM_BPF_Peak_Prof_2(gaitObj->VelData.velFinal, gaitObj->GaitFilter.cutoffFreqSmooth);
		gaitObj->VelData.velBPF0ABS = WIDM_Abs_double(gaitObj->VelData.velBPF);

		if (gaitObj->GaitMode.mode[0] == 1 && gaitObj->firstPeriodCheck == 0 && gaitObj->AngleData.degBPF[1] < 0 && gaitObj->AngleData.degBPF[0] > 0){
			gaitObj->GaitFilter.PeakWAmp = gaitObj->VelData.velBPF0ABS;			// At maximum slope of degBPF, get Amplitude of velocity(deg/s)
		}
	}

	if (gaitObj->GaitMode.mode[0] == 1){
		gaitObj->GaitFilter.PeakAmpSmooth = 0.998 * gaitObj->GaitFilter.PeakAmpSmooth + 0.002 * gaitObj->GaitFilter.PeakAmp;
		gaitObj->GaitFilter.PeakWAmpSmooth = 0.998 * gaitObj->GaitFilter.PeakWAmpSmooth + 0.002 * gaitObj->GaitFilter.PeakWAmp;
	}

	if (gaitObj->gaitFuncLoopCnt > 1000){
		gaitObj->AngleData.angleNorm = gaitObj->AngleData.degBPF[0] / gaitObj->GaitFilter.PeakAmpSmooth;
		gaitObj->VelData.velocityNorm = - gaitObj->VelData.velBPF / gaitObj->GaitFilter.PeakWAmpSmooth;
	}

	// 0~100%		// "1" : 50~75%, "2" : 75~100%, "3" : 0~25%, "4" : 25~50% ....
	gaitObj->gaitPhase = gaitObj->GaitMode.mode[0] * (atan2( (-1)*gaitObj->VelData.velocityNorm, (-1)*gaitObj->AngleData.angleNorm)) / (2*WIDM_PI) * 100 + 50;

	ISIFlags->I2_Flag = 0;		// 10%
	ISIFlags->I3_Flag = 0;		// 20%
	ISIFlags->I4_Flag = 0;		// 30%
	ISIFlags->I5_Flag = 0;		// 40%
	ISIFlags->I6_Flag = 0;		// 50%

	ISIFlags->I7_Flag = 0;		// 0%	 Leg front->back	&   Gait Count ++
	ISIFlags->I8_Flag = 0;		// 50%	 Leg back->front
	ISIFlags->I9_Flag = 0;		// 75%	 Leg middle->front
	ISIFlags->I10_Flag = 0;		// 25%	 Leg middle->back

	ISIFlags->I11_Flag = 0;		// 60%
	ISIFlags->I12_Flag = 0;		// 70%
	ISIFlags->I13_Flag = 0;		// 80%
	ISIFlags->I14_Flag = 0;		// 90%

	ISIFlags->I15_Flag = 0;		// after B8 Flag on, swing for extension
	ISIFlags->I16_Flag = 0;		// 1NE's revised Extension timing


	if (gaitObj->gaitFuncLoopCnt > 1000 && gaitObj->GaitMode.mode[0] == 1){
		if (gaitObj->gaitPhase > gaitObj->phaseThreshold1 && gaitObj->gaitPhasePrev < gaitObj->phaseThreshold1){
			ISIFlags->I2_Flag = 1;
		}
		if (gaitObj->gaitPhase > gaitObj->phaseThreshold2 && gaitObj->gaitPhasePrev < gaitObj->phaseThreshold2){
			ISIFlags->I3_Flag = 1;
		}
		if (gaitObj->gaitPhase > gaitObj->phaseThreshold3 && gaitObj->gaitPhasePrev < gaitObj->phaseThreshold3){
			ISIFlags->I4_Flag = 1;
		}
		if (gaitObj->gaitPhase > gaitObj->phaseThreshold4 && gaitObj->gaitPhasePrev < gaitObj->phaseThreshold4){
			ISIFlags->I5_Flag = 1;
		}
		if (gaitObj->gaitPhase > gaitObj->phaseThreshold5 && gaitObj->gaitPhasePrev < gaitObj->phaseThreshold5){
			ISIFlags->I6_Flag = 1;
		}
		if (gaitObj->gaitPhase > gaitObj->phaseThreshold6 && gaitObj->gaitPhasePrev < gaitObj->phaseThreshold6){
			ISIFlags->I11_Flag = 1;
		}
		if (gaitObj->gaitPhase > gaitObj->phaseThreshold7 && gaitObj->gaitPhasePrev < gaitObj->phaseThreshold7){
			ISIFlags->I12_Flag = 1;
		}
		if (gaitObj->gaitPhase > gaitObj->phaseThreshold8 && gaitObj->gaitPhasePrev < gaitObj->phaseThreshold8){
			ISIFlags->I13_Flag = 1;
		}
		if (gaitObj->gaitPhase > gaitObj->phaseThreshold9 && gaitObj->gaitPhasePrev < gaitObj->phaseThreshold9){
			ISIFlags->I14_Flag = 1;
		}
	}

	gaitObj->AngleData.degLPF0ABS = WIDM_Abs_double(gaitObj->AngleData.degLPF[0]);

	if (gaitObj->AngleData.degLPF0ABS < 20 && gaitObj->GaitMode.mode[0] == 0) {
		gaitObj->NeutralPosture = 0.999 * gaitObj->NeutralPosture + 0.001 * gaitObj->AngleData.degLPF[0];				// Get (Stop & Standing) Posture's Angle
	}

	gaitObj->AngleData.degUnbiased[0] = gaitObj->AngleData.degLPF[0] - gaitObj->NeutralPosture;

	if (gaitObj->gaitFuncLoopCnt > 1000) {
		if (gaitObj->AngleData.degUnbiased[0] > 10 && gaitObj->gyroLPF[1] > 0 && gaitObj->gyroLPF[0] < 0 && ISIFlags->I7_Finished == 0 && gaitObj->GaitMode.mode[0] == 1){
			ISIFlags->I7_Flag = 1;
			ISIFlags->I7_Finished = 1;
			gaitObj->gaitCount++;
		}
		if (gaitObj->AngleData.degUnbiased[0] < 3 && gaitObj->gyroLPF[1] < 0 && gaitObj->gyroLPF[0] > 0 && ISIFlags->I8_Finished == 0 && gaitObj->GaitMode.mode[0] == 1){
			ISIFlags->I8_Flag = 1;
			ISIFlags->I8_Finished = 1;
			// B8stack++;
		}
		if (gaitObj->AngleData.degUnbiased[0] < 3 && gaitObj->AngleData.degUnbiased[1] > 3) {
			ISIFlags->I7_Finished = 0;
		}
		if (gaitObj->AngleData.degUnbiased[0] > 5 && gaitObj->AngleData.degUnbiased[1] < 5) {
			ISIFlags->I8_Finished = 0;
		}

        if (gaitObj->gyroLPF[0] > 5) {
            ISIFlags->I9_Flag = 1;
        }
        if (gaitObj->gyroLPF[0] < -5) {
            ISIFlags->I10_Flag = 1;
        }

		if (ISIFlags->I8_Finished == 1) {
			if (gaitObj->AngleData.incDegTrig[0] < gaitObj->AngleData.incDegTrig[1] && gaitObj->AngleData.incDegTrig[1] > gaitObj->AngleData.incDegTrig[2] && gaitObj->AngleData.incDegTrig[0] > 40) {
			// if (gaitObj->VelData.incVelTrig[0] < gaitObj->VelData.incVelTrig[1] && gaitObj->VelData.incVelTrig[1] > gaitObj->VelData.incVelTrig[2]) {
			// if (gaitObj->AngleData.incDegTrig[0] < gaitObj->AngleData.incDegTrig[1] && gaitObj->AngleData.incDegTrig[1] > gaitObj->AngleData.incDegTrig[2]) {
				// if (gaitObj->VelData.incVelTrig[0] < 0 && gaitObj->VelData.incVelTrig[1] > 0 && gaitObj->VelData.incVelTrig[1] > gaitObj->VelData.incVelTrig[2]) {
        			ISIFlags->I15_Flag = 1;
					// B15stack++;
    			// }
				// ISIFlags->I15_Flag = 1;
				// B15stack++;
			}
		}

		// Added by 1NE //
		gaitObj->chk1Prev = gaitObj->chk1;
		if (gaitObj->gaitFuncLoopCnt - gaitObj->chk1 > 0.6 * gaitObj->Period){
			if (gaitObj->AngleData.incDegLPF[2] < gaitObj->AngleData.incDegLPF[1] && gaitObj->AngleData.incDegLPF[0]  < gaitObj->AngleData.incDegLPF[1] && gaitObj->AngleData.incDegLPF[1] > 10){
				ISIFlags->I16_Flag = 1;
				// B16_stack++;
				gaitObj->chk1 = gaitObj->gaitFuncLoopCnt;
			}
			else{
				gaitObj->chk1 = gaitObj->chk1Prev;
			}
		}

	}

	// Update previous values//
	gaitObj->GaitMode.mode[1] = gaitObj->GaitMode.mode[0];
	ISIFlags->I1_Flag = gaitObj->GaitMode.mode[0];
	gaitObj->gaitPhasePrev = gaitObj->gaitPhase;
	gaitObj->AngleData.degLPF[1] = gaitObj->AngleData.degLPF[0];
	gaitObj->gyroLPF[1] = gaitObj->gyroLPF[0];
	gaitObj->VelData.velLPF2[1] = gaitObj->VelData.velLPF2[0];
	gaitObj->AngleData.degBPF[2] = gaitObj->AngleData.degBPF[1];
	gaitObj->AngleData.degBPF[1] = gaitObj->AngleData.degBPF[0];
	gaitObj->AngleData.degUnbiased[1] = gaitObj->AngleData.degUnbiased[0];

	gaitObj->AngleData.incDegTrig[2] = gaitObj->AngleData.incDegTrig[1];
	gaitObj->AngleData.incDegTrig[1] = gaitObj->AngleData.incDegTrig[0];

	gaitObj->VelData.incVelTrig[2] = gaitObj->VelData.incVelTrig[1];
	gaitObj->VelData.incVelTrig[1] = gaitObj->VelData.incVelTrig[0];

	gaitObj->AngleData.incDegLPF[2] = gaitObj->AngleData.incDegLPF[1];
	gaitObj->AngleData.incDegLPF[1] = gaitObj->AngleData.incDegLPF[0];
}

static void RunGaitFunction_H10(GaitObj_t* gaitObj, ISI_Flag_t* ISIFlags, WIDM_BPFState* BPFState_Deg, WIDM_BPFState* BPFState_Vel, WIDM_LPFState* LPFState)
{
	gaitObj->gaitFuncLoopCnt++; 										// Check this Loop Count
	gaitObj->gaitFuncLoopCnt_1sec = gaitObj->gaitFuncLoopCnt / 1000;	// For Debug, check each 1sec

	gaitObj->VelData.velLPF[0] = UpdateLPF_CustomAlpha(LPFState, gaitObj->VelData.velFinal, LPF_SMOOTHING_FACTOR);	// velLPF : velRaw -> Low-pass filtering
	// gaitObj->VelData.velLPF[0] = WIDM_LPF_walking_Prof(gaitObj->VelData.velFinal);	// velLPF : velRaw -> Low-pass filtering
	if (gaitObj->gaitFuncLoopCnt < 1000) {
		gaitObj->VelData.velLPF[0] = 0;
	}

	gaitObj->VelData.velLPFAbs = WIDM_Abs_double(gaitObj->VelData.velLPF[0]);
	double tempVelLPFAbs = WIDM_GetMaxValue_double(0, gaitObj->VelData.velLPFAbs-15);
	gaitObj->GaitMode.modeCheck = 0.993 * gaitObj->GaitMode.modeCheck + 0.007 * tempVelLPFAbs;

	if (gaitObj->gaitFuncLoopCnt > 1) {
		gaitObj->AngleData.degLPF[0] = 0.98 * gaitObj->AngleData.degLPF[1] + 0.02 * gaitObj->AngleData.degFinal;

		gaitObj->GaitMode.mode[0] = gaitObj->GaitMode.mode[1];

		if (gaitObj->gaitFuncLoopCnt > 1000) {
			if (gaitObj->GaitMode.mode[1] == 0 && gaitObj->GaitMode.modeCheck > 10) {	// mode = 1 : Walking, mode = 0 : Stop
				gaitObj->GaitMode.mode[0] = 1;
			} else if (gaitObj->GaitMode.mode[1] == 1 && gaitObj->GaitMode.modeCheck < 1) {
				gaitObj->GaitMode.mode[0] = 0;
			}
		}

		gaitObj->VelData.velLPF2[0] = 0.9997 * gaitObj->VelData.velLPF2[1] + 0.0003 * gaitObj->VelData.velLPF[0];
	}

	if (gaitObj->gaitFuncLoopCnt > 10) {
		if (gaitObj->GaitMode.mode[0] == 1) {
			if (gaitObj->VelData.velLPF2[1] > 0 && gaitObj->VelData.velLPF2[0] < 0) {	// Leg front->back moving point
				if (gaitObj->firstPeriodCheck == 0) {
					gaitObj->Period = gaitObj->gaitFuncLoopCnt - gaitObj->timeStampPrev;
					if (gaitObj->Period > 2000) {
						gaitObj->Period = 2000;
					} else if (gaitObj->Period < 200) {
						gaitObj->Period = 200;
					}
				} else {
					gaitObj->firstPeriodCheck = 0;
				}

				gaitObj->GaitFilter.cutoffFreq = 2 * WIDM_PI / (gaitObj->Period * 0.001);
				gaitObj->timeStampPrev = gaitObj->gaitFuncLoopCnt;
			}
		} else if (gaitObj->GaitMode.mode[0] == 0) {
			gaitObj->firstPeriodCheck = 1;
		}
	}

	gaitObj->GaitFilter.cutoffFreqSmooth = 0.992 * gaitObj->GaitFilter.cutoffFreqSmooth + 0.008 * gaitObj->GaitFilter.cutoffFreq;
	// gaitObj->AngleData.degBPF[0] = WIDM_BPF_Peak_Prof_1(gaitObj->AngleData.degFinal, gaitObj->GaitFilter.cutoffFreqSmooth);
	gaitObj->AngleData.degBPF[0] = UpdateBPF_PeakCutoff_ForDeg(BPFState_Deg, gaitObj->AngleData.degFinal, gaitObj->GaitFilter.cutoffFreqSmooth);
	gaitObj->AngleData.degBPF0ABS = WIDM_Abs_double(gaitObj->AngleData.degBPF[0]);

	if (gaitObj->gaitFuncLoopCnt > 3) {
		if (gaitObj->GaitMode.mode[0] == 1 && gaitObj->firstPeriodCheck == 0 && gaitObj->AngleData.degBPF[1] < gaitObj->AngleData.degBPF[2] && gaitObj->AngleData.degBPF[0] > gaitObj->AngleData.degBPF[1]) {
			gaitObj->GaitFilter.PeakAmp = gaitObj->AngleData.degBPF0ABS;	// At Leg Backward->Forward transition, get Amplitude of angle(deg)
		}

		gaitObj->VelData.velBPF = UpdateBPF_PeakCutoff_ForVel(BPFState_Vel, gaitObj->VelData.velFinal, gaitObj->GaitFilter.cutoffFreqSmooth);
		// gaitObj->VelData.velBPF = WIDM_BPF_Peak_Prof_2(gaitObj->VelData.velFinal, gaitObj->GaitFilter.cutoffFreqSmooth);
		gaitObj->VelData.velBPF0ABS = WIDM_Abs_double(gaitObj->VelData.velBPF);

		if (gaitObj->GaitMode.mode[0] == 1 && gaitObj->firstPeriodCheck == 0 && gaitObj->AngleData.degBPF[1] < 0 && gaitObj->AngleData.degBPF[0] > 0) {
			gaitObj->GaitFilter.PeakWAmp = gaitObj->VelData.velBPF0ABS;		// At maximum slope of gaitObj->AngleData.degBPF, get Amplitude of velocity(deg/s)
		}
	}

	if (gaitObj->GaitMode.mode[0] == 1) {
		gaitObj->GaitFilter.PeakAmpSmooth = 0.998 * gaitObj->GaitFilter.PeakAmpSmooth + 0.002 * gaitObj->GaitFilter.PeakAmp;
		gaitObj->GaitFilter.PeakWAmpSmooth = 0.998 * gaitObj->GaitFilter.PeakWAmpSmooth + 0.002 * gaitObj->GaitFilter.PeakWAmp;
	}

	if (gaitObj->gaitFuncLoopCnt > 1000) {
		gaitObj->AngleData.angleNorm = gaitObj->AngleData.degBPF[0] / gaitObj->GaitFilter.PeakAmpSmooth;
		gaitObj->VelData.velocityNorm = - gaitObj->VelData.velBPF / gaitObj->GaitFilter.PeakWAmpSmooth;
	}

	gaitObj->gaitPhase = gaitObj->GaitMode.mode[0] * (atan2( (-1)*gaitObj->VelData.velocityNorm, (-1)*gaitObj->AngleData.angleNorm)) / (2*WIDM_PI) * 100 + 50;  // 0~100%		// "1" : 50~75%, "2" : 75~100%, "3" : 0~25%, "4" : 25~50% ....

	ISIFlags->I2_Flag = 0;	// 10%
	ISIFlags->I3_Flag = 0;	// 20%
	ISIFlags->I4_Flag = 0;	// 30%
	ISIFlags->I5_Flag = 0;	// 40%
	ISIFlags->I6_Flag = 0;	// 50%

	ISIFlags->I7_Flag = 0;	// 0%	 Leg front->back	&   Gait Count ++
	ISIFlags->I8_Flag = 0;	// 50%	 Leg back->front
	ISIFlags->I9_Flag = 0;	// 75%	 Leg middle->front
	ISIFlags->I10_Flag = 0;	// 25%	 Leg middle->back

	ISIFlags->I11_Flag = 0;	// 60%
	ISIFlags->I12_Flag = 0;	// 70%
	ISIFlags->I13_Flag = 0;	// 80%
	ISIFlags->I14_Flag = 0;	// 90%

	ISIFlags->I17_Flag = 0;	// 1.8 ~ 2 sec
	ISIFlags->I18_Flag = 0;	// 1.5 ~ 1.8 sec
	ISIFlags->I19_Flag = 0;	// 1.2 ~ 1.5 sec
	ISIFlags->I20_Flag = 0;	// 0.8 ~ 1.2 sec
	ISIFlags->I21_Flag = 0;	// 0 ~ 0.8 sec

	if (gaitObj->gaitFuncLoopCnt > 1000 && gaitObj->GaitMode.mode[0] == 1) {
		if (gaitObj->gaitPhase > gaitObj->phaseThreshold1 && gaitObj->gaitPhasePrev < gaitObj->phaseThreshold1) {
			ISIFlags->I2_Flag = 1;
		}
		if (gaitObj->gaitPhase > gaitObj->phaseThreshold2 && gaitObj->gaitPhase <  gaitObj->phaseThreshold2) {
			ISIFlags->I3_Flag = 1;
		}
		if (gaitObj->gaitPhase >  gaitObj->phaseThreshold3 && gaitObj->gaitPhase <  gaitObj->phaseThreshold3) {
			ISIFlags->I4_Flag = 1;
		}
		if (gaitObj->gaitPhase >  gaitObj->phaseThreshold4 && gaitObj->gaitPhase <  gaitObj->phaseThreshold4) {
			ISIFlags->I5_Flag = 1;
		}
		if (gaitObj->gaitPhase >  gaitObj->phaseThreshold5 && gaitObj->gaitPhase <  gaitObj->phaseThreshold5) {
			ISIFlags->I6_Flag = 1;
		}
		if (gaitObj->gaitPhase >  gaitObj->phaseThreshold6 && gaitObj->gaitPhase <  gaitObj->phaseThreshold6) {
			ISIFlags->I11_Flag = 1;
		}
		if (gaitObj->gaitPhase >  gaitObj->phaseThreshold7 && gaitObj->gaitPhase <  gaitObj->phaseThreshold7) {
			ISIFlags->I12_Flag = 1;
		}
		if (gaitObj->gaitPhase >  gaitObj->phaseThreshold8 && gaitObj->gaitPhase <  gaitObj->phaseThreshold8) {
			ISIFlags->I13_Flag = 1;
		}
		if (gaitObj->gaitPhase >  gaitObj->phaseThreshold9 && gaitObj->gaitPhase <  gaitObj->phaseThreshold9) {
			ISIFlags->I14_Flag = 1;
		}
	}

	gaitObj->AngleData.degLPF0ABS = WIDM_Abs_double(gaitObj->AngleData.degLPF[0]);

	if (gaitObj->AngleData.degLPF0ABS < 20 && gaitObj->GaitMode.mode[0] == 0) {
		gaitObj->NeutralPosture = 0.99 * gaitObj->NeutralPosture + 0.01 * gaitObj->AngleData.degLPF[0];	// Get (Stop & Standing) Posture's Angle
	}

	gaitObj->AngleData.degUnbiased[0] = gaitObj->AngleData.degLPF[0] - gaitObj->NeutralPosture;
	gaitObj->AngleData.degUnbias = (float)gaitObj->AngleData.degUnbiased[0];

	gaitObj->AngleData.degBPF_LPF[0] = gaitObj->AngleData.degBPF_LPF[1] * 0.99 + gaitObj->AngleData.degBPF[0] * 0.01;
	gaitObj->AngleData.degBPF_LPF[1] = gaitObj->AngleData.degBPF_LPF[0];

	if (gaitObj->GaitMode.mode[0] == 1 && (gaitObj->AngleData.degBPF[0]*gaitObj->AngleData.degBPF[0]*gaitObj->AngleData.degBPF[0]*gaitObj->AngleData.degBPF[0]) > 10000 ) {
		if (gaitObj->first1NE == 1) {
			gaitObj->walkCurr = 1;
			gaitObj->walkPrev = gaitObj->gaitFuncLoopCnt;
			gaitObj->first1NE = 0;
		} else {
			gaitObj->walkCurr = 1;
			gaitObj->walkPrev = gaitObj->gaitFuncLoopCnt;
		}
	} else {
		if (gaitObj->GaitMode.mode[0] == 1 && (gaitObj->AngleData.degBPF[0]*gaitObj->AngleData.degBPF[0]*gaitObj->AngleData.degBPF[0]*gaitObj->AngleData.degBPF[0]) < 10000 && gaitObj->first1NE == 0 ) {
			if (gaitObj->gaitFuncLoopCnt - gaitObj->walkPrev < 1100) {
				gaitObj->walkCurr = 1;
			} else {
				gaitObj->walkCurr = 0;
				gaitObj->first1NE = 1;
			}
		}
	}

	if (gaitObj->gaitPhase > 2 && gaitObj->gaitPhase < 98) {
		if (gaitObj->gaitPhase > gaitObj->gaitPhase) {
			gaitObj->gaitPhase = gaitObj->gaitPhase;
		} else {
			gaitObj->gaitPhase = gaitObj->gaitPhase;
		}
	}

	if (gaitObj->gaitFuncLoopCnt > 1000) {
		if (gaitObj->AngleData.degUnbiased[0] > 10 && gaitObj->VelData.velLPF[1] > 0 && gaitObj->VelData.velLPF[0] < 0 && ISIFlags->I7_Finished == 0 && gaitObj->GaitMode.mode[0] == 1) {
			if (gaitObj->walkCurr == 1) {
				ISIFlags->I7_Flag = 1;
				ISIFlags->I7_Finished = 1;
				gaitObj->gaitCount++;
			}

			/* 1/15 Tuning */
//			if (abs(gaitObj->AngleData.degBPF_LPF[0]) > 8) {
//				ISIFlags->I7_Flag = 1;
//				ISIFlags->I7_Finished = 1;
//				gaitObj->gaitCount++;
//			}

			/* Original */
//			ISIFlags->I7_Flag = 1;
//			ISIFlags->I7_Finished = 1;
//			gaitObj->gaitCount++;
		}
		if (gaitObj->AngleData.degUnbiased[0] < 3 && gaitObj->VelData.velLPF[1] < 0 && gaitObj->VelData.velLPF[0] > 0 && ISIFlags->I8_Finished == 0 && gaitObj->GaitMode.mode[0] == 1) {
			if (gaitObj->walkCurr == 1) {
				ISIFlags->I8_Flag = 1;
				ISIFlags->I8_Finished = 1;
			}

			/* 1/15 Tuning */
//			if (abs(gaitObj->AngleData.degBPF_LPF[0]) > 8) {
//				ISIFlags->I8_Flag = 1;
//				ISIFlags->I8_Finished = 1;
//			}

			/* Original */
//			ISIFlags->I8_Flag = 1;
//			ISIFlags->I8_Finished = 1;
		}
		if (gaitObj->AngleData.degUnbiased[0] < 3 && gaitObj->AngleData.degUnbiased[1] > 3) {
			ISIFlags->I7_Finished = 0;
		}
		if (gaitObj->AngleData.degUnbiased[0] > 5 && gaitObj->AngleData.degUnbiased[1] < 5) {
			ISIFlags->I8_Finished = 0;
		}
        if (gaitObj->VelData.velLPF[0] > 8) {
            ISIFlags->I9_Flag = 1;
        }
        if (gaitObj->VelData.velLPF[0] < -8) {
            ISIFlags->I10_Flag = 1;
        }
	}

	if (gaitObj->gaitFuncLoopCnt > 1000) {
//        if (gaitObj->Period > 1800 && gaitObj->Period <= 2000) {
//        	ISIFlags->I17_Flag = 1;
//        }
//        else if (gaitObj->Period > 1400 && gaitObj->Period <= 1800) {
//        	ISIFlags->I18_Flag = 1;
//        }
//        else if (gaitObj->Period > 1100 && gaitObj->Period <= 1400) {
//        	ISIFlags->I19_Flag = 1;
//        }
//        else if (gaitObj->Period > 700 && gaitObj->Period <= 1100) {
//        	ISIFlags->I20_Flag = 1;
//        }
//        else if (gaitObj->Period <= 700) {
//        	ISIFlags->I21_Flag = 1;
//        }
	}

	// Update previous values//
	gaitObj->GaitMode.mode[1] = gaitObj->GaitMode.mode[0];
	ISIFlags->I1_Flag = gaitObj->GaitMode.mode[0];
	gaitObj->gaitPhasePrev = gaitObj->gaitPhase;
	gaitObj->VelData.velLPF[1] = gaitObj->VelData.velLPF[0];
	gaitObj->AngleData.degLPF[1] = gaitObj->AngleData.degLPF[0];
	gaitObj->gyroLPF[1] = gaitObj->gyroLPF[0];
	gaitObj->VelData.velLPF2[1] = gaitObj->VelData.velLPF2[0];
	gaitObj->AngleData.degBPF[2] = gaitObj->AngleData.degBPF[1];
	gaitObj->AngleData.degBPF[1] = gaitObj->AngleData.degBPF[0];
	gaitObj->AngleData.degUnbiased[1] = gaitObj->AngleData.degUnbiased[0];
}

static void CheckWalkingState_Prof(void)
{
// 		if (B2Flag == 1){
// //			B2_chk[B2stack] = (uint8_t)gaitPhase;		// 10%
// 			B2stack++;
// 		}
// 		// B3Flag //
// 		if (B3Flag == 1){
// //			B3_chk[B3stack] = (uint8_t)gaitPhase;		// 20%
// 			B3stack++;
// 		}
// 		// B4Flag //
// 		if (B4Flag == 1){
// //			B4_chk[B4stack] = (uint8_t)gaitPhase;		// 30%
// 			B4stack++;
// 		}
// 		// B5Flag//
// 		if (B5Flag == 1){
// //			B5_chk[B5stack] = (uint8_t)gaitPhase;		// 40%
// 			B5stack++;
// 		}
// 		// B6Flag //
// 		if (B6Flag == 1){
// //			B6_chk[B6stack] = (uint8_t)gaitPhase;		// 50%
// 			B6stack++;
// 		}
// 		// B7Flag //
// 		if (B7Flag == 1){
// //			B7_chk[B7stack] = (uint8_t)gaitPhase;		// ~~ 0% FB transition
// 			B7stack++;
// 		}
// 		// B8Flag //
// 		if (B8Flag == 1){
// //			B8_chk[B8stack] = (uint8_t)gaitPhase;		// ~~ 50% BF transition
// 			B8stack++;
// 		}
// 		// B9Flag //
// 		if (B9Flag == 1){
// //			B9_chk[B9stack] = (uint8_t)gaitPhase;		// Backward -> Forward
// 			B9stack++;
// 		}
// 		// B10Flag //
// 		if (B10Flag == 1){
// //			B10_chk[B10stack] = (uint8_t)gaitPhase;		// Forward -> Backward
// 			B10stack++;
// 		}
}
/* ----------------------- ROUTINE ------------------------ */

#endif /* SUIT_MINICM_ENABLED */
