

#ifndef AS_DEV_MNGR_INC_AS_DEV_MNGR_H_
#define AS_DEV_MNGR_INC_AS_DEV_MNGR_H_

#include "module.h"

#ifdef SUIT_MINICM_ENABLED

#include "robot_setting.h"
#include "device_id.h"

#include "data_object_dictionaries.h"
#include "data_object_interface.h"
#include "ioif_fdcan_common.h"

#include "AS_gait_ctrl.h"

#include "AS_system_ctrl.h"

/**
 *-----------------------------------------------------------
 *              MACROS AND PREPROCESSOR DIRECTIVES
 *-----------------------------------------------------------
 * @brief Directives and macros for readability and efficiency.
 */

#define AS_DEV_MAX_ROUTINES 8
#define AS_DEV_MAX_PDO_LIST 40

#define AS_DEV_MAX_PDO		40


/**
 *------------------------------------------------------------
 *                     TYPE DECLARATIONS
 *------------------------------------------------------------
 * @brief Custom data types and structures for the module.
 */
typedef uint8_t (*AS_FDCAN_TxFncPtr) (uint16_t, uint8_t*, uint32_t); // ID, Data, Len

typedef enum _AS_IDX_t {
#ifdef SUIT_H10
    AS_MD_IDX_1 = 6,    // RH
    AS_MD_IDX_2,        // LH
#endif
#ifdef SUIT_K10
    AS_MD_IDX_1 = 8,    // RK
    AS_MD_IDX_2,        // LK
#endif 
#ifdef SUIT_A10
    AS_MD_IDX_1 = 10,   // RA
    AS_MD_IDX_2,        // LA
#endif 
	AS_DEV_MAX_NUM
} AS_IDX_t;

//#pragma pack(push, 1)
typedef struct _AS_TaskData_t {
    uint8_t state;
    uint8_t routines[AS_DEV_MAX_ROUTINES];
    uint8_t n_routines;
} AS_TaskData_t;

#pragma pack(push, 1)
typedef struct _C_Vector {
	uint8_t FF_gain;
	uint8_t PD_gain;
	uint8_t IC_gain;
	uint8_t DOB_gain;
	uint8_t IRC_gain;
	uint8_t FC_gain;
} C_Vector;

typedef struct _P_Vector {
	int16_t yd;  // desired position  (deg)
	uint16_t L;  // trajectory length (ms)
	uint8_t s0;  // acceleration      (deg/s^2)
	uint8_t sd;  // deceleration      (deg/s^2)
} P_Vector;

typedef struct _F_Vector {
	uint16_t mode_idx;     // mode
	int16_t  tau_max;      // 100*Nm
	uint16_t delay;        // delay (ms)
} F_Vector;

typedef struct _I_Vector {
	uint8_t epsilon_target;     // Half width of the Corridor      (x10)
	uint8_t Kp_target;          // Magnitude of Virtual Spring
	uint8_t Kd_target;          // Magnitude of Virtual Damper
	uint8_t lambda_target;      // Impedance Ratio in the Corridor (x100)
	uint16_t duration;          // Duration for translation
} I_Vector;
#pragma pack(pop)
typedef struct _Quaternion {
	int16_t q_x;
	int16_t q_y;
	int16_t q_z;
	int16_t q_w;
} Quaternion;

typedef struct _AS_DevData_t {
    AS_TaskData_t msg_hdlr_task;
	AS_TaskData_t low_level_ctrl_task;
	AS_TaskData_t mid_level_ctrl_task;
    AS_TaskData_t imu_ctrl_task;
    AS_TaskData_t gait_ctrl_task;
    AS_TaskData_t ext_dev_ctrl_task;

	uint8_t pdo_list[AS_DEV_MAX_PDO*2];

    // SDO
    float motor_auxiliary_input;

    int16_t TauMax;
    uint16_t Delay;
    uint8_t ModeIdx;

    // PDO 
    float SAM_degfinal;
    float SAM_velfinal;
    float SAM_degINC;
    float SAM_velINC;
    float SAM_gyrZ;

    float totalCurrentInput;
    float CurrentOutput;
    float velEstimated;
} AS_DevData_t;

typedef struct _AS_CtrlObj_t {
    // IO Properties
    uint16_t sdo_tx_id;
    uint16_t pdo_tx_id;
    AS_FDCAN_TxFncPtr tx_fnc;

    // Device Data
    AS_DevData_t data;

    // Device Object
	DOPI_DevObj_t devObj;
} AS_CtrlObj_t;
//#pragma pack(pop)


/**
 *------------------------------------------------------------
 *                      GLOBAL VARIABLES
 *------------------------------------------------------------
 * @brief Extern declarations for global variables.
 */

extern AS_CtrlObj_t userCtrlObj[AS_DEV_MAX_NUM];


/**
 *------------------------------------------------------------
 *                     FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Function prototypes declaration for this module.
 */

/* Init */
void InitFDCANDevMngr(void);

void SetPDO_H10(int DevIdx);
void SetPDO_K10(int DevIdx);

void SendAuxInput(int DevIdx, float motorAuxIn, float assistPctg);
void Send_F_Vector(int DevIdx, uint8_t modeIdx, int16_t TauMax, uint16_t delay);

void AllDevOffStates(void);
void AllDevStandbyStates(void);
void AllDevEnableStates(void);
void AllDevSetRoutines(void);

AS_DevData_t* GetDevDataSet(int DevIdx);

#endif /* SUIT_MINICM_ENABLED */

#endif /* AS_DEV_MNGR_INC_AS_DEV_MNGR_H_ */
