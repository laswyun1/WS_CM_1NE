

#ifndef WHOLE_BODY_CTRL_INC_WS_DEV_MNGR_H_
#define WHOLE_BODY_CTRL_INC_WS_DEV_MNGR_H_

#include "module.h"

#ifdef WALKON5_CM_ENABLED

#include "robot_setting.h"
#include "device_id.h"

#include "data_object_interface.h"
#include "ioif_fdcan_common.h"


/**
 *-----------------------------------------------------------
 *              MACROS AND PREPROCESSOR DIRECTIVES
 *-----------------------------------------------------------
 * @brief Directives and macros for readability and efficiency.
 */

#define MD_NUM 16

#define MD_IDX_RH_COR  0
#define MD_IDX_LH_COR  1
#define MD_IDX_RH_ROT  2
#define MD_IDX_LH_ROT  3
#define MD_IDX_RH_SAG  4
#define MD_IDX_LH_SAG  5
#define MD_IDX_RK  	   6
#define MD_IDX_LK      7
#define MD_IDX_RA_MED  8
#define MD_IDX_LA_MED  9
#define MD_IDX_RA_LAT  10
#define MD_IDX_LA_LAT  11

#define MD_MAX_ROUTINES 8
#define MD_MAX_PDO     30

#define MD_ENABLE  1
#define MD_DISABLE 0


/**
 *------------------------------------------------------------
 *                     TYPE DECLARATIONS
 *------------------------------------------------------------
 * @brief Custom data types and structures for the module.
 */

typedef struct {
    uint8_t state;
    uint8_t routines[MD_MAX_ROUTINES];
    uint8_t n_routines;
} MD_TaskData;

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

typedef struct {
    MD_TaskData task_msg;
    MD_TaskData task_lowlevel;
    MD_TaskData task_midlevel;
    MD_TaskData task_imu;

    MD_TaskData task_ext;


    float target_position;       // CM -> MD
    float target_position_echo;  // MD -> CM

    float inc_actual_position;
    float abs1_actual_position;
    float abs2_actual_position;

    float overall_fb_gain_ctrl[2];  // 1st: desired gain, 2nd: duration

    float Kp;
    float Kd;

    float IRC_J_gain;
    float IRC_B_gain;

    float J;
    float B;

    float mu_c;
    float mu_v;
    float mu_g;

    float target_velocity;
    float actual_velocity;

    float position_ref;

    float current_ref;

    float aux_input;
    float current_act;

    float fsr;
    float bump;
    float length_sensor;
    float ankle_encoder;

    float length_command;
    float position_offset;

    int8_t lm_dir_command;
    int8_t lm_dir_switch;

    uint32_t cm2md_index;
    uint32_t md2cm_index;

    float noise_magnitude;

    float peakCurr_limit_DR;
    float contCurr_limit_DR;

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

	C_Vector  c_vector;
	P_Vector  p_vector;
	F_Vector  f_vector;
	I_Vector  i_vector;
	float     i_vector_Kp_max;
	float     i_vector_Kd_max;

	float      initial_angle; // initialized angle
	Quaternion quaternion;

	uint32_t ControlLoopTime;
	uint32_t CommLoopTime;
	float tilt_angle;

	uint8_t pdo_list[MD_MAX_PDO*2];
} MD_Data;

typedef uint8_t (*FDCAN_TxFncPtr) (uint16_t, uint8_t*, uint32_t); // ID, Data, Len

typedef struct {
    // Driver Object
	DOPI_DevObj_t bb;

    // IO Properties
    uint16_t tx_id;
    FDCAN_TxFncPtr tx_fnc;

    // Driver Data
    MD_Data data;

    // Misc.
    uint8_t is_hip;
} MD_Obj;


/**
 *------------------------------------------------------------
 *                      GLOBAL VARIABLES
 *------------------------------------------------------------
 * @brief Extern declarations for global variables.
 */




/**
 *------------------------------------------------------------
 *                     FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Function prototypes declaration for this module.
 */

void MD_Init();
void All_MD_Init();
void Activate_All_MD_Msg_Hdlr_task();

void All_MD_OffStates();
void All_MD_StandbyStates();
void All_MD_EnableStates();

void MD_StartSensorStreaming();
void MD_EnableGetData(uint8_t enable);
void MD_EnableLM(uint8_t enable);

void MD_ActivateRoutines();
void MD_ActivateCurrentControl();
void MD_ActivatePositionControl();

void SetOverallFeedbackGain(int obj_idx, float K, float dT);

void MD_AppendFrictionCompensation();

void MD_AppendMsgPDOSend();

void MD_StandbyLowLevelCtrl();
void MD_EnableLowLevelCtrl();
void MD_StandbyMidLevelCtrl();
void MD_EnableMidLevelCtrl();
void MD_StandbyMsgHdlr();
void MD_EnableMsgHdlr();
void Right_MD_StandbyStates();
void Left_MD_StandbyStates();
void MD_EnableCurrentControl(uint8_t enable);
void MD_EnablePositionControl(uint8_t enable);
void MD_SetTargetCurrent(float* curr);
void MD_SetTargetPosition(float* pos_ref);
void MD_SetTargetDir(int8_t* dir);
void MD_SetGravityCompesnationCurrent(float* cur_ref);

MD_Data* MD_GetDataset(int obj_idx);

/*(DEMO)******************/

void Set_Peak_Limit_DR(int obj_idx, float max_peak_current);
void Set_Continu_Limit_DR(int obj_idx, float max_cont_current);

void Set_J(int obj_idx, float t_J);
void Set_B(int obj_idx, float t_B);
void Set_Mu_C(int obj_idx, float mu_c);
void Set_Mu_V(int obj_idx, float mu_v);
void Set_Mu_G(int obj_idx, float mu_g);

void Set_IncEncoder_Init_Angle(int obj_idx, float t_angle);

void Set_ID_Param(int obj_idx, float min_vel, float max_vel, float amp, float deadtime, float N_sample, float offset);
void Set_ID_Mode(int obj_idx);
void Set_ID_Msg(int obj_idx);
void Set_Pos_Fix_Mode(int obj_idx);
void Set_Adv_Friction_Compensation_Mode(int obj_idx);
void Set_Adv_Friction_Compensation_Msg(int obj_idx);

void Set_Friction_Compensation_Mode(int obj_idx);

void Activate_MD_Msg_Hdlr_task(int obj_idx);

void Set_MD_Position_Ctrl_Routines(int obj_idx);
void Set_MD_Current_Ctrl_Routines(int obj_idx);
void Set_MD_Impedance_Ctrl_Routines(int obj_idx);
void Set_All_MD_Position_Ctrl_Routines();
void Set_All_MD_Continu_Limit_DR(void);

void Send_Auxilirary_Input(int obj_idx, float aux_input);
void Send_Target_Current(int obj_idx, float target_current);
void Send_Target_Position(int obj_idx, float target_position);

void Send_C_Vector(int obj_idx, uint8_t K_FF, uint8_t K_PD, uint8_t K_IC, uint8_t K_DOB, uint8_t K_IRC, uint8_t K_FC);
void Send_P_Vector(int obj_idx, float yd, uint16_t L, uint8_t s0, uint8_t sd);
void Send_F_Vector(int obj_idx, uint8_t mode_idx, float max_torque, uint16_t delay);
void Send_I_Vector(int obj_idx, float t_epsilon, float t_Kp, float t_Kd, float t_lambda, uint16_t duration);
void Set_I_Vector_Kp_Max(int obj_idx, float Kp_max);
void Set_I_Vector_Kd_Max(int obj_idx, float Kd_max);

void Set_IRC_J_Gain(int obj_idx, float t_gain);
void Set_IRC_B_Gain(int obj_idx, float t_gain);
void Set_Noise_Magnitude(int obj_idx, float t_mag);

void Set_Velocity_Periodic_Sig_Amp(int obj_idx, float t_amp);
void Set_Velocity_Periodic_Sig_Freq(int obj_idx, float t_freq);
void Set_Current_Periodic_Sig_Amp(int obj_idx, float t_amp);
void Set_Current_Periodic_Sig_Freq(int obj_idx, float t_freq);

void Set_Kp(int obj_idx, float t_Kp);
void Set_Kd(int obj_idx, float t_Kd);

void SetPDO_1NE(int MD_idx);
void SetALLRoutines_1NE(void);
void SetALLEnableStates_1NE(void);

void Send_Elapsed_Time(int obj_idx, uint32_t control_time, uint32_t comm_time);


#endif /* WALKON5_CM_ENABLED */

#endif /* WHOLE_BODY_CTRL_INC_WS_DEV_MNGR_H_ */
