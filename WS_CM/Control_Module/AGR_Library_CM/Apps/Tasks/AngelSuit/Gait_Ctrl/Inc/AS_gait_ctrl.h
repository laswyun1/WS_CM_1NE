

#ifndef APPS_TASKS_ANGELSUIT_GAIT_CTRL_INC_AS_GAIT_CTRL_H_
#define APPS_TASKS_ANGELSUIT_GAIT_CTRL_INC_AS_GAIT_CTRL_H_

#include "module.h"

#ifdef SUIT_MINICM_ENABLED

#include "data_object_common.h"
#include "data_object_dictionaries.h"

#include "AS_dev_mngr.h"
#include "AS_widm_algorithms.h"
#include "AS_imu_ctrl.h"

#include "AS_ISI.h"


/**
 *-----------------------------------------------------------
 *              MACROS AND PREPROCESSOR DIRECTIVES
 *-----------------------------------------------------------
 * @brief Directives and macros for readability and efficiency.
 */

#define NORM_CUTOFF_FREQ        3   
#define LPF_SMOOTHING_FACTOR    0.02


/**
 *------------------------------------------------------------
 *                     TYPE DECLARATIONS
 *------------------------------------------------------------
 * @brief Custom data types and structures for the module.
 */

typedef struct _GaitMode_t {
    uint8_t mode[2]; // 0: current, 1: previous
    float modeCheck;
} GaitMode_t;

typedef struct _AngleData_t {
    float degLPF[2]; // 0: current, 1: previous
    float degLPF0ABS;
    float degBPF[3]; // 0: current, 1: previous, 2: previous of previous // center frequency BPF
    float degBPF_LPF[2];
    float degUnbiased[2];
    float degUnbias;
    float degBPF0ABS;
    float angleNorm;

    // For K10
    float incDegRaw;        
    float incDegLPF[3];    // Example for incremental leg position filtering
    float incDegTrig[3];   // Trigger points for incremental degrees
    float filteredIncDeg;

    float degFinal;
} AngleData_t;

typedef struct _VelData_t {
    float velLPF[2]; // 0: current, 1: previous
    float velLPFAbs;
    float velBPF;
    float velBPF0ABS;
    float velocityNorm;

    float velLPF2[2]; // 0: current, 1: previous

    // For K10
    float incVelRaw;
    float velLPF_K10;
    float incVel;       // Incremental velocity
    float incVelTrig[3];
    float filteredIncVel;

    float velFinal;
} VelData_t;

typedef struct _GaitFilter_t {
    float cutoffFreq;
    float cutoffFreqSmooth;
    float PeakAmp;
    float PeakWAmp;
    float PeakAmpSmooth;
    float PeakWAmpSmooth;
} GaitFilter_t;

typedef struct _GaitObj_t {
    GaitMode_t      GaitMode;
    AngleData_t     AngleData;
    VelData_t       VelData;
    GaitFilter_t    GaitFilter;

    float gyroLPF[2];   // 0: current, 1: previous
    float gyrZRaw[2];   // SAM IMU Gyro Z  0: current, 1: previous

    float gaitPhase;
    float gaitPhasePrev;
    float NeutralPosture;

    uint32_t gaitFuncLoopCnt;
    uint32_t gaitFuncLoopCnt_1sec;
    uint32_t timeStampPrev;

    uint32_t walkCurr, walkPrev;

    uint32_t chk1, chk1Prev;

    uint16_t Period;

    uint8_t firstPeriodCheck;

    uint8_t first1NE; // First non-empty check

    uint8_t phaseThreshold1;
    uint8_t phaseThreshold2;
    uint8_t phaseThreshold3;
    uint8_t phaseThreshold4;
    uint8_t phaseThreshold5;
    uint8_t phaseThreshold6;
    uint8_t phaseThreshold7;
    uint8_t phaseThreshold8;
    uint8_t phaseThreshold9;

    uint32_t gaitCount;
} GaitObj_t;


/**
 *------------------------------------------------------------
 *                      GLOBAL VARIABLES
 *------------------------------------------------------------
 * @brief Extern declarations for global variables.
 */

extern GaitObj_t gaitObj_RH;
extern GaitObj_t gaitObj_LH;
extern GaitObj_t gaitObj_RK;
extern GaitObj_t gaitObj_LK;


/**
 *------------------------------------------------------------
 *                     FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Function prototypes declaration for this module.
 */

void InitGaitCtrl(void);
void RunGaitCtrl(void);


#endif /* SUIT_MINICM_ENABLED */

#endif /* APPS_TASKS_ANGELSUIT_GAIT_CTRL_INC_AS_GAIT_CTRL_H_ */
