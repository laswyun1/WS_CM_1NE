

#ifndef L30_DEV_MNGR_INC_L30_DEV_MNGR_H_
#define L30_DEV_MNGR_INC_L30_DEV_MNGR_H_

#include "module.h"

#ifdef L30_CM_ENABLED

#include "data_object_dictionaries.h"
#include "data_object_interface.h"
#include "ioif_fdcan_common.h"

#include "ioif_tb67h450fngel.h"
#include "data_object.h"

/**
 *-----------------------------------------------------------
 *              MACROS AND PREPROCESSOR DIRECTIVES
 *-----------------------------------------------------------
 * @brief Directives and macros for readability and efficiency.
 */

#define L30_MAX_ROUTINES    8
#define L30_MAX_PDO_LIST    20

#define L30_MAX_PDO         20

#define L30_ENABLE          1
#define L30_DISABLE         0


/**
 *------------------------------------------------------------
 *                     TYPE DECLARATIONS
 *------------------------------------------------------------
 * @brief Custom data types and structures for the module.
 */

typedef enum _L30IdxEnum {
	e_L30_RH_IDX = 6,
	e_L30_LH_IDX,
	e_L30_RK_IDX,
	e_L30_LK_IDX,
	e_L30_NUM
} L30IdxEnum;

typedef enum _SynchOnOffEnum {
	e_Synch_Off = 0,
	e_Synch_On
} SynchOnOffEnum;

#pragma pack(push, 1)
typedef struct _L30_MD_TaskData {
    uint8_t state;
    uint8_t routines[L30_MAX_ROUTINES];
    uint8_t n_routines;
    uint8_t synch_onoff;
} L30_MD_TaskData;


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

typedef struct _L30_MD_Data {
	L30_MD_TaskData msg_hdlr_task;
	L30_MD_TaskData low_level_ctrl_task;
	L30_MD_TaskData mid_level_ctrl_task;
	L30_MD_TaskData ext_dev_ctrl_task;

	uint8_t pdo_list[L30_MAX_PDO_LIST*2];

    float target_position;
    float actual_position;

    float target_velocity;
    float actual_velocity;

    float target_current;
    float actual_current;

    float abs_position;
    float abs_offset;
    float abs_dir;
    uint8_t abs_orign;
    int32_t inc_encoder_cnt;

    float target_sine_amp;
    float target_sine_freq;

    float FSR;
    float linear_potentiometer;
    float ankle_encoder;

    float upright_length_command;
    float upright_length_actual;

    uint8_t upright_dir_command;
    uint8_t upright_dir_actual;
    uint8_t upright_bt_state;

    float time_send;
    float time_receive;

    float PS_voltage;
    float PS_current;
    float PS_temperature;
    float motor_temp;

    uint32_t err_code;

    /*(DEMO) ******************************************/
    float velocity_periodic_sig_amp;
    float velocity_periodic_sig_freq;

    float current_periodic_sig_amp;
    float current_periodic_sig_freq;

    float trape_link_sys_id_min_vel;
    float trape_link_sys_id_max_vel;
    float trape_link_sys_id_amp;
	float trape_link_sys_id_deadtime;
	float trape_link_sys_id_N_samples;
	float trape_link_sys_id_offset;

	P_Vector   p_vector;
	F_Vector   f_vector;
	I_Vector   i_vector;
	float      i_vector_Kp_max;
	float      i_vector_Kd_max;

	float      initial_angle; // initialized angle
	Quaternion quaternion;

} L30_MD_Data;

typedef uint8_t (*L30_FDCAN_TxFncPtr) (uint16_t,  uint8_t*, uint32_t); // fdcan, ID, Data, Len

typedef struct _L30Struct{
    // Driver Object
	DOPI_DevObj_t d10_obj;

    // IO Properties
    uint16_t sdo_tx_id;
    uint16_t pdo_tx_id;
    L30_FDCAN_TxFncPtr tx_fnc;

    // Driver Data
    L30_MD_Data data;

    // Misc.
    uint8_t is_hip;
} L30Struct;


/**
 *------------------------------------------------------------
 *                      GLOBAL VARIABLES
 *------------------------------------------------------------
 * @brief Extern declarations for global variables.
 */

extern L30Struct* d10Ptr[e_L30_NUM];


/**
 *------------------------------------------------------------
 *                     FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Function prototypes declaration for this module.
 */
/* Initialize */
void Init_L30_Mngr();

/* Common */
void Send_PDO_Nothing(L30Struct* t_obj);

/* Leg Upright*/
void Enable_L30_LengthAct(L30Struct* t_obj);
void Disable_L30_LengthAct(L30Struct* t_obj);
void Enable_L30_Upright(L30Struct* t_obj);
void Disable_L30_Upright(L30Struct* t_obj);
void Send_L30_Upright_DIR_CMD(L30Struct* t_obj, IOIF_UprightMoveDir_t* t_dir);
void Send_L30_Upright_Length_CMD(L30Struct* t_obj, float* t_length_cmd);
void Set_dc_set_length(L30Struct* t_obj, float t_data);
void Enable_Upright_lenth_cmd(L30Struct* t_obj);

/* GRF */
void Enable_L30_GRF(L30Struct* t_obj);
void Disable_L30_GRF(L30Struct* t_obj);

/* Joints Abas */
void Enable_L30_Joints_Abs(L30Struct* t_obj);
void Disable_L30_Joints_Abs(L30Struct* t_obj);

/* Position Ctrl */
void Ready_L30_Position_Ctrl(L30Struct* t_obj);
void Enable_L30_Position_Ctrl(L30Struct* t_obj);
void Disable_L30_Position_Ctrl(L30Struct* t_obj);
//void Test_Led(L30Struct* t_obj);
void Send_L30_Position_Ref(L30Struct* t_obj, float t_ref);
void Pack_Sync_MSG(DOPI_SYNCMsg_t* t_sync_msg, uint8_t t_node_id, uint8_t t_index, float t_ref);
void Send_Sync_MSG(DOPI_SYNCMsg_t* t_sync_msg, uint8_t t_can_channel);

/* IMU */
void L30_StartSensorStreaming();

/* Joint Control */
void L30_EnableCurrnetControl(uint8_t enable);
void L30_SetTargetCurrent(float* curr);

/* ETC */
L30_MD_Data* L30_GetDataset(int obj_idx);
void Start_PDO_Streaming();
void Enable_SEND_PDO(L30Struct* t_obj);

/* ------------------- PIF Vector (WalkONSuit5) ------------------- */
void Send_P_Vector(int obj_idx, float yd, uint16_t L, uint8_t s0, uint8_t sd);
void Send_F_Vector(int obj_idx, uint8_t mode_idx, float max_torque, uint16_t delay);
void Send_I_Vector(int obj_idx, float t_epsilon, float t_Kp, float t_Kd, float t_lambda, uint16_t duration);
void Set_I_Vector_Kp_Max(int obj_idx, float Kp_max);
void Set_I_Vector_Kd_Max(int obj_idx, float Kd_max);


#endif /* L30_CM_ENABLED */

#endif /* L30_DEV_MNGR_INC_L30_DEV_MNGR_H_ */
