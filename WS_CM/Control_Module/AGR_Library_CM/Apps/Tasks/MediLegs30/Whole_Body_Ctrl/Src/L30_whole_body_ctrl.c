

#include "L30_whole_body_ctrl.h"

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

TaskObj_t wholeBodyCtrl;
uint8_t test_d10 = 0;
uint8_t AM_ON_BEEP;


/**
 *------------------------------------------------------------
 *                      STATIC VARIABLES
 *------------------------------------------------------------
 * @brief Variables local to this module.
 */

float gain_amp = 1.0;
float mag_amp_set = 1.0;
float assistance_threshold = 40;
float inter_para[6] = {0, 0, 0, 0.5, 0, 1};

uint32_t whole_body_loop_time = 0;
DOPI_SYNCMsg_t sync_msg1;
DOPI_SYNCMsg_t sync_msg2;

uint8_t mode_select;
uint8_t traj_select;
uint8_t r_sw_traj_enable;
uint8_t l_sw_traj_enable;
uint32_t traj_cnt;
uint8_t mode_status[8];
uint8_t EMI_MAG = 10;

uint32_t assistance_cnt;
uint32_t motion_end_beep_cnt;
uint8_t boot_cnt;

float assistance_temp_angle[4];
PIDObject pid_motor[4][4];
float mag_amp[4][4];
int time_offset[4];
float time_amp[4];
float walk_IDLE_angle[4];
float traj_para[4][4][6];
float temp_ref[4];
float test_ref;
int test_i;
float walking_assistance_iter[4];
float stand_assistance_iter[4];
float sit_assistance_iter[4];
float test_dc_length[6] = {420, 420, 450, 450, 400, 400};

float fsm_vector[6][5] = {
	{1, 1, 1, 2, 0},
	{2, 2, 2, 4, 2},
	{33, 3, 1, 255, 5},
	{4, 4, 2, 5, 9},
	{5, 5, 2, 33, 10},
	{255, 255, 255, 1, 255},
};

int ind = 0;
int cur_state = 1;
int cur_audio_id = 1;
int cur_motion_set = 0;

int input[3] = {
	0,
};

float* test_addr1;

uint8_t autofit_sw;

uint8_t autofit_done[4];

uint8_t move_sw=0;
uint8_t move_motor_num=0;
int move_cnt=0;

extern osSemaphoreId_t sdio_sync_semaphore;			//sdio transmit sync semaphore
extern osMutexId_t mutex_sd_buffering;				//buffering mutex

uint8_t sd_buf_1[SWAP_SD_BUF_SIZE] __attribute__((section(".FATFS_RAMD1_data")));	// switching sd buffer 1
uint8_t sd_buf_2[SWAP_SD_BUF_SIZE] __attribute__((section(".FATFS_RAMD1_data")));	// switching sd buffer 2

uint_fast16_t sd_buf_1_pos = 0;						//sd buf 1 position(index)
uint_fast16_t sd_buf_2_pos = 0;						//sd buf 2 position(index)

uint8_t *sd_buf_cur = sd_buf_1;						//current buffer pointer : start with sd buf 1
uint8_t *sd_buf_next = sd_buf_2;					//next buffer pointer

// uint8_t sdSendData[SEND_DATA_SIZE] ;                 //data buffer
/* For H10 Battery Usage Data Logging */
uint8_t sdSendData[SEND_DATA_SIZE] __attribute__((section(".FATFS_RAMD1_data")));   //data buffer

int convert_res = 0;
uint32_t dataSaveCnt = 0;

static uint32_t lastDataSaveTime = 0;
static uint32_t oneMinuteCnt = 0;
uint8_t dataSaveFinished = 0;
/**
 *------------------------------------------------------------
 *                 STATIC FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Static Function prototypes for this module.
 */

/* ------------------- STATE FUNCTIONS ------------------- */
static void StateOff_Ent(void);
static void StateOff_Run(void);
static void StateStandby_Ent(void);
static void StateEnable_Ent(void);
static void StateEnable_Run(void);
static void StateEnable_Exit(void);
static void StateError_Run(void);

/* ------------------- SDO CALLBACK ------------------- */
static void Mode_State(DOP_SDOArgs_t *t_req, DOP_SDOArgs_t *t_res);
static void Set_test_d10(DOP_SDOArgs_t *t_req, DOP_SDOArgs_t *t_res);
static void Stop_upright_move(DOP_SDOArgs_t *t_req, DOP_SDOArgs_t *t_res);
static void Set_upright_length(DOP_SDOArgs_t *t_req, DOP_SDOArgs_t *t_res);

/* ------------------- ROUTINES ------------------- */
static int Lengthen_Legs_Upright(void);
static int Run_Positon_Control(void);
static int Get_GRF_Data(void);
static int Get_Joints_Absolute_Angle(void);
static int Get_D10_Upright_ActLength(void);

static int Get_Joints_Absolute_Angle(void);
static int Get_D10_Upright_ActLength(void);

static void SaveDataMassStorage(void);
static inline void _SD_DBuffered_Transmit(uint8_t *data, uint_fast16_t data_len);
static void fillArrayWithVariable(int var, char byteArray[], int arraySize);				// data generator
static void intToStr(int num, char str[], int maxSize);										// integer to string translator

/* Data Processing 2024.01.30 By HD (Not used sprintf Ver, to be deleted soon) */
static void ReverseStr(char* str, int length);
static void IntToStringRec(int value, char* output, int* index);
static void IntToString(int value, char* output);
static int FindStringLength(char* str, int index);
static void FloatToString(float value, char* output, int afterpoint);
static void FillBufferData(DataPoint data[], int numDataPoints, char buffer[], int bufferSize);
/**
 *------------------------------------------------------------
 *                      PUBLIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions that interface with this module.
 */

/* ------------------- SDO CALLBACK ------------------- */
DOP_COMMON_SDO_CB(wholeBodyCtrl)

void InitWholeBodyCtrl(void)
{
	/* Init */
	InitTask(&wholeBodyCtrl);
	DOPC_AssignTaskID(&wholeBodyCtrl, TASK_IDX_WHOLE_BODY_CTRL);

	Init_L30_Mngr();
	Define_Traj();
	Init_Training_Routine();
	Upright_state = 2;

	/* State Definition */
	TASK_CREATE_STATE(&wholeBodyCtrl, TASK_STATE_OFF, StateOff_Ent, StateOff_Run, NULL, false);
	TASK_CREATE_STATE(&wholeBodyCtrl, TASK_STATE_STANDBY, StateStandby_Ent, NULL, NULL, true);
	TASK_CREATE_STATE(&wholeBodyCtrl, TASK_STATE_ENABLE, StateEnable_Ent, StateEnable_Run, StateEnable_Exit, false);
	TASK_CREATE_STATE(&wholeBodyCtrl, TASK_STATE_ERROR, NULL, StateError_Run, NULL, false);

	/* Routine Definition */
	TASK_CREATE_ROUTINE(&wholeBodyCtrl, 6, NULL, Lengthen_Legs_Upright, NULL);
	TASK_CREATE_ROUTINE(&wholeBodyCtrl, 7, NULL, Run_Positon_Control, NULL);
	TASK_CREATE_ROUTINE(&wholeBodyCtrl, 8, NULL, Get_GRF_Data, NULL);
	TASK_CREATE_ROUTINE(&wholeBodyCtrl, 9, NULL, Get_Joints_Absolute_Angle, NULL);
	TASK_CREATE_ROUTINE(&wholeBodyCtrl, 10, NULL, Get_D10_Upright_ActLength, NULL);


	/* DOD Definition */
	// DOD
	DOP_CreateDOD(TASK_IDX_WHOLE_BODY_CTRL);
	// PDO
	// DOP_COMMON_PDO_CREATE(TASK_IDX_WHOLE_BODY_CTRL, wholeBodyCtrl);
	// SDO
	DOP_COMMON_SDO_CREATE(TASK_IDX_WHOLE_BODY_CTRL)

//	DOP_CreateSDO(TASK_IDX_WHOLE_BODY_CTRL, SDO_ID_BODYCTRL_MODE_STATE, DOP_UINT8, Mode_State); //"state_chage",
	DOP_CreateSDO(TASK_IDX_WHOLE_BODY_CTRL, 6,  DOP_UINT8, Set_test_d10);
	DOP_CreateSDO(TASK_IDX_WHOLE_BODY_CTRL, 7,  DOP_FLOAT32, Set_upright_length);
	DOP_CreateSDO(TASK_IDX_WHOLE_BODY_CTRL, 8,  DOP_UINT8, Stop_upright_move);
//	DOP_CreateSDO(TASK_IDX_WHOLE_BODY_CTRL, SDO_ID_BODYCTRL_AMP_CHANGE, DOP_UINT8, Set_AMP); //"amp change",

	DOP_CreatePDO(TASK_IDX_WHOLE_BODY_CTRL, 0, DOP_FLOAT32, 1, &d10Ptr[0]->data.target_current); //"lh_current_ref",
	DOP_CreatePDO(TASK_IDX_WHOLE_BODY_CTRL, 1, DOP_FLOAT32, 1, &d10Ptr[1]->data.target_current); //"rh_current_ref",
	DOP_CreatePDO(TASK_IDX_WHOLE_BODY_CTRL, 2, DOP_FLOAT32, 1, &d10Ptr[2]->data.target_current); //"lk_current_ref",
	DOP_CreatePDO(TASK_IDX_WHOLE_BODY_CTRL, 3, DOP_FLOAT32, 1, &d10Ptr[3]->data.target_current); //"rk_current_ref",

	DOP_CreatePDO(TASK_IDX_WHOLE_BODY_CTRL, 4, DOP_FLOAT32, 1, &d10Ptr[0]->data.actual_current); //"lh_current_act",
	DOP_CreatePDO(TASK_IDX_WHOLE_BODY_CTRL, 5, DOP_FLOAT32, 1, &d10Ptr[1]->data.actual_current); //"rh_current_act",
	DOP_CreatePDO(TASK_IDX_WHOLE_BODY_CTRL, 6, DOP_FLOAT32, 1, &d10Ptr[2]->data.actual_current); //"lk_current_act",
	DOP_CreatePDO(TASK_IDX_WHOLE_BODY_CTRL, 7, DOP_FLOAT32, 1, &d10Ptr[3]->data.actual_current); //"rk_current_act",

	DOP_CreatePDO(TASK_IDX_WHOLE_BODY_CTRL, 8,  DOP_FLOAT32, 1, &d10Ptr[0]->data.abs_position); //"lh_abs_encoder_act",
	DOP_CreatePDO(TASK_IDX_WHOLE_BODY_CTRL, 9,  DOP_FLOAT32, 1, &d10Ptr[1]->data.abs_position); //"rh_abs_encoder_act",
	DOP_CreatePDO(TASK_IDX_WHOLE_BODY_CTRL, 10, DOP_FLOAT32, 1, &d10Ptr[2]->data.abs_position); // "lk_abs_encoder_act",
	DOP_CreatePDO(TASK_IDX_WHOLE_BODY_CTRL, 11, DOP_FLOAT32, 1, &d10Ptr[3]->data.abs_position); // "rk_abs_encoder_act",

	DOP_CreatePDO(TASK_IDX_WHOLE_BODY_CTRL, 12, DOP_INT32, 1, &d10Ptr[0]->data.inc_encoder_cnt); //"lh_inc_encoder_cnt",
	DOP_CreatePDO(TASK_IDX_WHOLE_BODY_CTRL, 13, DOP_INT32, 1, &d10Ptr[1]->data.inc_encoder_cnt); //"rh_inc_encoder_cnt",
	DOP_CreatePDO(TASK_IDX_WHOLE_BODY_CTRL, 14, DOP_INT32, 1, &d10Ptr[2]->data.inc_encoder_cnt); //"lk_inc_encoder_cnt",
	DOP_CreatePDO(TASK_IDX_WHOLE_BODY_CTRL, 15, DOP_INT32, 1, &d10Ptr[3]->data.inc_encoder_cnt); //"rk_inc_encoder_cnt",

	DOP_CreatePDO(TASK_IDX_WHOLE_BODY_CTRL, 16, DOP_FLOAT32, 1, &d10Ptr[0]->data.PS_voltage); //"lh_md_voltage",
	DOP_CreatePDO(TASK_IDX_WHOLE_BODY_CTRL, 17, DOP_FLOAT32, 1, &d10Ptr[1]->data.PS_voltage); //"rh_md_voltage",
	DOP_CreatePDO(TASK_IDX_WHOLE_BODY_CTRL, 18, DOP_FLOAT32, 1, &d10Ptr[2]->data.PS_voltage); //"lk_md_voltage",
	DOP_CreatePDO(TASK_IDX_WHOLE_BODY_CTRL, 19, DOP_FLOAT32, 1, &d10Ptr[3]->data.PS_voltage); //"rk_md_voltage",
	DOP_CreatePDO(TASK_IDX_WHOLE_BODY_CTRL, 20, DOP_FLOAT32, 1, &d10Ptr[0]->data.PS_current); //"lh_md_current",
	DOP_CreatePDO(TASK_IDX_WHOLE_BODY_CTRL, 21, DOP_FLOAT32, 1, &d10Ptr[1]->data.PS_current); //"rh_md_current",
	DOP_CreatePDO(TASK_IDX_WHOLE_BODY_CTRL, 22, DOP_FLOAT32, 1, &d10Ptr[6]->data.upright_length_actual); //"rh_length_act",
	DOP_CreatePDO(TASK_IDX_WHOLE_BODY_CTRL, 23, DOP_FLOAT32, 1, &d10Ptr[7]->data.upright_length_actual); //"lh_length_act",
	DOP_CreatePDO(TASK_IDX_WHOLE_BODY_CTRL, 24, DOP_FLOAT32, 1, &d10Ptr[8]->data.upright_length_actual); //"rk_length_act",
	DOP_CreatePDO(TASK_IDX_WHOLE_BODY_CTRL, 25, DOP_FLOAT32, 1, &d10Ptr[9]->data.upright_length_actual); //"kk_length_act",
	DOP_CreatePDO(TASK_IDX_WHOLE_BODY_CTRL, 26, DOP_FLOAT32, 1, &pelvicUpright[0].lengthAct); //"depth_length_act",
	DOP_CreatePDO(TASK_IDX_WHOLE_BODY_CTRL, 27, DOP_FLOAT32, 1, &pelvicUpright[1].lengthAct); //"width_length_act",
	DOP_CreatePDO(TASK_IDX_WHOLE_BODY_CTRL, 28, DOP_UINT32, 1, &d10Ptr[0]->data.err_code); //"lh_current_ref",
	DOP_CreatePDO(TASK_IDX_WHOLE_BODY_CTRL, 29, DOP_UINT32, 1, &d10Ptr[1]->data.err_code); //"rh_current_ref",
	DOP_CreatePDO(TASK_IDX_WHOLE_BODY_CTRL, 30, DOP_UINT32, 1, &d10Ptr[2]->data.err_code); //"lk_current_ref",
	DOP_CreatePDO(TASK_IDX_WHOLE_BODY_CTRL, 31, DOP_UINT32, 1, &d10Ptr[3]->data.err_code); //"rk_current_ref",
	DOP_CreatePDO(TASK_IDX_WHOLE_BODY_CTRL, 32, DOP_FLOAT32, 1, &d10Ptr[0]->data.motor_temp); //"lh_current_ref",
	DOP_CreatePDO(TASK_IDX_WHOLE_BODY_CTRL, 33, DOP_FLOAT32, 1, &d10Ptr[1]->data.motor_temp); //"rh_current_ref",
	DOP_CreatePDO(TASK_IDX_WHOLE_BODY_CTRL, 34, DOP_FLOAT32, 1, &d10Ptr[2]->data.motor_temp); //"lk_current_ref",
	DOP_CreatePDO(TASK_IDX_WHOLE_BODY_CTRL, 35, DOP_FLOAT32, 1, &d10Ptr[3]->data.motor_temp); //"rk_current_ref",

	//    DOP_CreatePDO(TASK_IDX_WHOLE_BODY_CTRL, 1000, DOP_FLOAT32, 1, &pelvic_upright[0].upright_obj.ref); //"left_length",
	//    DOP_CreatePDO(TASK_IDX_WHOLE_BODY_CTRL, 1001, DOP_FLOAT32, 1, &pelvic_upright[1].upright_obj.ref); //"left_depth",
	//    DOP_CreatePDO(TASK_IDX_WHOLE_BODY_CTRL, 1002, DOP_FLOAT32, 1, &pelvic_upright[2].upright_obj.ref); //"right_length",
	//    DOP_CreatePDO(TASK_IDX_WHOLE_BODY_CTRL, 1003, DOP_FLOAT32, 1, &pelvic_upright[3].upright_obj.ref); //"right_depth",

	//	Push_Routine(&wholeBodyCtrl.routine, 7);
	//	Push_Routine(&ext_dev_ctrl_task.routine, 10);
}

void RunWholeBodyCtrl(void)
{
	RunTask(&wholeBodyCtrl);
}

void Define_Traj(void)
{
	//  traj, id, para

	//  traj
	//  0 sit to stand
	//  1 right swing
	//  2 left swing
	//  3 stand to sit

	//  id
	//  0 left hip
	//  1 right hip
	//  2 left knee
	//  3 right knee

	//  0 sit to stand

	traj_para[0][0][0] = 0.8;
	traj_para[0][0][1] = 0.25;
	traj_para[0][0][2] = 0.7;
	traj_para[0][0][3] = 0.25;
	traj_para[0][0][4] = 0;
	traj_para[0][0][5] = -1.0;

	traj_para[0][1][0] = 0.8;
	traj_para[0][1][1] = 0.25;
	traj_para[0][1][2] = 0.7;
	traj_para[0][1][3] = 0.25;
	traj_para[0][1][4] = 0;
	traj_para[0][1][5] = -1.0;

	traj_para[0][2][0] = 0;
	traj_para[0][2][1] = 0.2;
	traj_para[0][2][2] = 1.3;
	traj_para[0][2][3] = 0.5;
	traj_para[0][2][4] = 0;
	traj_para[0][2][5] = -7;

	traj_para[0][3][0] = 0;
	traj_para[0][3][1] = 0.2;
	traj_para[0][3][2] = 1.3;
	traj_para[0][3][3] = 0.5;
	traj_para[0][3][4] = 0;
	traj_para[0][3][5] = -7;

	//  1 right swing

	traj_para[1][0][0] = 0;
	traj_para[1][0][1] = 0.1;
	traj_para[1][0][2] = 0.7;
	traj_para[1][0][3] = 0.0;
	traj_para[1][0][4] = 0;
	traj_para[1][0][5] = -4;

	traj_para[1][1][0] = 0;
	traj_para[1][1][1] = 0.2;
	traj_para[1][1][2] = 0.6;
	traj_para[1][1][3] = 0.0;
	traj_para[1][1][4] = 0;
	traj_para[1][1][5] = 4.0;

	traj_para[1][2][0] = 0;
	traj_para[1][2][1] = 0.2;
	traj_para[1][2][2] = 0.3;
	traj_para[1][2][3] = 0.3;
	traj_para[1][2][4] = 0.0;
	traj_para[1][2][5] = 0.0;

	traj_para[1][3][0] = 0;
	traj_para[1][3][1] = 0.2;
	traj_para[1][3][2] = 0.3;
	traj_para[1][3][3] = 0.3;
	traj_para[1][3][4] = 0.0;
	traj_para[1][3][5] = 2.0;

	//  2 left swing
	traj_para[2][0][0] = 0;
	traj_para[2][0][1] = 0.2;
	traj_para[2][0][2] = 0.6;
	traj_para[2][0][3] = 0.0;
	traj_para[2][0][4] = 0;
	traj_para[2][0][5] = 4.5;

	traj_para[2][1][0] = 0;
	traj_para[2][1][1] = 0.1;
	traj_para[2][1][2] = 0.7;
	traj_para[2][1][3] = 0.0;
	traj_para[2][1][4] = 0;
	traj_para[2][1][5] = -4;

	traj_para[2][2][0] = 0;
	traj_para[2][2][1] = 0.2;
	traj_para[2][2][2] = 0.3;
	traj_para[2][2][3] = 0.3;
	traj_para[2][2][4] = 0.0;
	traj_para[2][2][5] = 2.5;

	traj_para[2][3][0] = 0;
	traj_para[2][3][1] = 0.2;
	traj_para[2][3][2] = 0.3;
	traj_para[2][3][3] = 0.3;
	traj_para[2][3][4] = 0.0;
	traj_para[2][3][5] = 0.0;

	//  3 stand to sit
	traj_para[3][0][0] = 0.25;
	traj_para[3][0][1] = 0.25;
	traj_para[3][0][2] = 2;
	traj_para[3][0][3] = 0.5;
	traj_para[3][0][4] = 0;
	traj_para[3][0][5] = -4;

	traj_para[3][1][0] = 0.25;
	traj_para[3][1][1] = 0.25;
	traj_para[3][1][2] = 2;
	traj_para[3][1][3] = 0.5;
	traj_para[3][1][4] = 0;
	traj_para[3][1][5] = -4;

	traj_para[3][2][0] = 0;
	traj_para[3][2][1] = 0.2;
	traj_para[3][2][2] = 2.3;
	traj_para[3][2][3] = 0.5;
	traj_para[3][2][4] = 0;
	traj_para[3][2][5] = -6;

	traj_para[3][3][0] = 0;
	traj_para[3][3][1] = 0.2;
	traj_para[3][3][2] = 2.3;
	traj_para[3][3][3] = 0.5;
	traj_para[3][3][4] = 0;
	traj_para[3][3][5] = -6;
}

void Init_Training_Routine(void)
{
	traj_select = 0;
	traj_cnt = 0;

	for (int i = 0; i < 8; i++)
	{
		mode_status[i] = 0;
	}

	for (int i = 0; i < 4; i++)
	{
		time_offset[i] = 0;
		time_amp[i] = 1;
		for (int j = 0; j < 4; j++)
		{
			mag_amp[i][j] = mag_amp_set;
		}
	}

	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			if (i == 3)
			{
				pid_motor[i][j].Kp = 0.07 * gain_amp;
				pid_motor[i][j].Kd = 0.01 * gain_amp;
			}
			else
			{
				pid_motor[i][j].Kp = 1.5 * gain_amp;
				pid_motor[i][j].Kd = 0.01 * gain_amp;
			}
		}
	}

	pid_motor[1][2].Kp = 3 * gain_amp;
	pid_motor[1][3].Kd = 0.01 * gain_amp;
	pid_motor[2][2].Kp = 3 * gain_amp;
	pid_motor[2][3].Kd = 0.01 * gain_amp;
};

float traj_calcul(float traj_para[], int time, float time_amp, float mag_amp, int time_offset)
{
	float traj_time_to_0 = (traj_para[0]) * 1000 * time_amp;
	float traj_time_to_1 = (traj_para[0] + traj_para[1]) * 1000 * time_amp;
	float traj_time_to_2 = (traj_para[0] + traj_para[1] + traj_para[2]) * 1000 * time_amp;
	float traj_time_to_3 = (traj_para[0] + traj_para[1] + traj_para[2] + traj_para[3]) * 1000 * time_amp;
	float traj_time_to_4 = (traj_para[0] + traj_para[1] + traj_para[2] + traj_para[3] + traj_para[4]) * 1000 * time_amp;
	float res = 0;
	float y_0;
	float y_d;
	float s_0;
	float s_d;
	float a_0;
	float a_1;
	float a_2;
	float a_3;
	float a_4;
	float a_5;

	if (time - time_offset < traj_time_to_0)
	{
		res = 0;
	}
	else if (time - time_offset >= traj_time_to_0 && time - time_offset < traj_time_to_1)
	{
		y_0 = 0;
		y_d = traj_para[5] * mag_amp;
		s_0 = 0;
		s_d = 0;
		a_0 = y_0;
		a_2 = 0.5 * s_0 * (y_d - y_0);
		a_3 = (10 - 1.5 * s_0 + 0.5 * s_d) * (y_d - y_0);
		a_4 = (-15 + 1.5 * s_0 - s_d) * (y_d - y_0);
		a_5 = (6 - 0.5 * s_0 + 0.5 * s_d) * (y_d - y_0);
		res = a_0 + a_2 * ((double)(time - traj_time_to_0 - time_offset) / (traj_para[1] * 1000 * time_amp)) * ((double)(time - traj_time_to_0 - time_offset) / (traj_para[1] * 1000 * time_amp)) + a_3 * ((double)(time - traj_time_to_0 - time_offset) / (traj_para[1] * 1000 * time_amp)) * ((double)(time - traj_time_to_0 - time_offset) / (traj_para[1] * 1000 * time_amp)) * ((double)(time - traj_time_to_0 - time_offset) / (traj_para[1] * 1000 * time_amp)) + a_4 * ((double)(time - traj_time_to_0 - time_offset) / (traj_para[1] * 1000 * time_amp)) * ((double)(time - traj_time_to_0 - time_offset) / (traj_para[1] * 1000 * time_amp)) * ((double)(time - traj_time_to_0 - time_offset) / (traj_para[1] * 1000 * time_amp)) * ((double)(time - traj_time_to_0 - time_offset) / (traj_para[1] * 1000 * time_amp)) + a_5 * ((double)(time - traj_time_to_0 - time_offset) / (traj_para[1] * 1000 * time_amp)) * ((double)(time - traj_time_to_0 - time_offset) / (traj_para[1] * 1000 * time_amp)) * ((double)(time - traj_time_to_0 - time_offset) / (traj_para[1] * 1000 * time_amp)) * ((double)(time - traj_time_to_0 - time_offset) / (traj_para[1] * 1000 * time_amp)) * ((double)(time - traj_time_to_0 - time_offset) / (traj_para[1] * 1000 * time_amp));
	}
	else if (time - time_offset >= traj_time_to_1 && time - time_offset < traj_time_to_2)
	{
		res = traj_para[5] * mag_amp;
	}
	else if (time - time_offset >= traj_time_to_2 && time - time_offset < traj_time_to_3)
	{
		y_0 = traj_para[5] * mag_amp;
		y_d = 0;
		s_0 = 0;
		s_d = 0;
		a_0 = y_0;
		a_2 = 0.5 * s_0 * (y_d - y_0);
		a_3 = (10 - 1.5 * s_0 + 0.5 * s_d) * (y_d - y_0);
		a_4 = (-15 + 1.5 * s_0 - s_d) * (y_d - y_0);
		a_5 = (6 - 0.5 * s_0 + 0.5 * s_d) * (y_d - y_0);
		res = a_0 + a_2 * ((double)(time - traj_time_to_2 - time_offset) / (traj_para[3] * 1000 * time_amp)) * ((double)(time - traj_time_to_2 - time_offset) / (traj_para[3] * 1000 * time_amp)) + a_3 * ((double)(time - traj_time_to_2 - time_offset) / (traj_para[3] * 1000 * time_amp)) * ((double)(time - traj_time_to_2 - time_offset) / (traj_para[3] * 1000 * time_amp)) * ((double)(time - traj_time_to_2 - time_offset) / (traj_para[3] * 1000 * time_amp)) + a_4 * ((double)(time - traj_time_to_2 - time_offset) / (traj_para[3] * 1000 * time_amp)) * ((double)(time - traj_time_to_2 - time_offset) / (traj_para[3] * 1000 * time_amp)) * ((double)(time - traj_time_to_2 - time_offset) / (traj_para[3] * 1000 * time_amp)) * ((double)(time - traj_time_to_2 - time_offset) / (traj_para[3] * 1000 * time_amp)) + a_5 * ((double)(time - traj_time_to_2 - time_offset) / (traj_para[3] * 1000 * time_amp)) * ((double)(time - traj_time_to_2 - time_offset) / (traj_para[3] * 1000 * time_amp)) * ((double)(time - traj_time_to_2 - time_offset) / (traj_para[3] * 1000 * time_amp)) * ((double)(time - traj_time_to_2 - time_offset) / (traj_para[3] * 1000 * time_amp)) * ((double)(time - traj_time_to_2 - time_offset) / (traj_para[3] * 1000 * time_amp));
	}
	else if (time - time_offset >= traj_time_to_3 && time - time_offset < traj_time_to_4)
	{
		res = 0;
	}

	if (traj_para[5] > 0)
	{
		if (res > traj_para[5] * mag_amp || res < -traj_para[5] * mag_amp)
		{
			res = 0;
		}
	}
	else
	{
		if (res > -traj_para[5] * mag_amp || res < traj_para[5] * mag_amp)
		{
			res = 0;
		}
	}
	return res;
}

void Run_PID_Control(PIDObject *t_PID_obj, float t_ref, float t_actual, float t_period)
{
	float t_err;

	t_PID_obj->ref = t_ref;
	t_PID_obj->act = t_actual;

	t_err = t_ref - t_actual;

	t_PID_obj->err_sum += t_err * t_period;
	t_PID_obj->err_diff = (t_err - t_PID_obj->err) / t_period;
	t_PID_obj->err = t_err;

	t_PID_obj->control_input = t_PID_obj->Kp * t_PID_obj->err + t_PID_obj->Ki * t_PID_obj->err_sum + t_PID_obj->Kd * t_PID_obj->err_diff;
	if (t_PID_obj->control_input > 10)
	{
		t_PID_obj->control_input = 10;
	}
	else if (t_PID_obj->control_input < -10)
	{
		t_PID_obj->control_input = -10;
	}
}

/**
 *------------------------------------------------------------
 *                      STATIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions intended for internal use within this module.
 */

/* ------------------- STATE FUNCTIONS ------------------- */

static void StateOff_Ent()
{

}

static void StateOff_Run()
{
	StateTransition(&wholeBodyCtrl.stateMachine, TASK_STATE_STANDBY);
}

static void StateStandby_Ent()
{
	StateTransition(&wholeBodyCtrl.stateMachine, TASK_STATE_ENABLE);
}

static void StateEnable_Ent()
{
	EntRoutines(&wholeBodyCtrl.routine);
	whole_body_loop_time = 0;
}

static void StateEnable_Run()
{
	static int length_cnt = 0;
	static int id_cnt = 6;//0;
	SaveDataMassStorage();
	/*if(whole_body_loop_time==20000){
		test_d10=8;
	}*/

//	if (AM_ON_BEEP > 0 && AM_ON_BEEP < 100)
//	{
//		// Beep ON
//		htim13.Instance->ARR = (BEEP_ALERT_TIM_FREQ / BEEP_ALERT_FREQ_VERY_HIGH) - 1;
//		htim13.Instance->CCR1 = (htim13.Instance->ARR * BEEP_ALERT_DUTY / 100) - 1;
//		HAL_TIM_PWM_Start(&htim13, TIM_CHANNEL_1);
//		AM_ON_BEEP++;
//	}
//	else if (AM_ON_BEEP >= 100 && AM_ON_BEEP < 200)
//	{
//		// Beep Off
//		htim13.Instance->CCR1 = 0;
//		HAL_TIM_PWM_Stop(&htim13, TIM_CHANNEL_1);
//		AM_ON_BEEP++;
//	}
//	else if (AM_ON_BEEP >= 200 && AM_ON_BEEP < 300)
//	{
//		// Beep ON
//		htim13.Instance->ARR = (BEEP_ALERT_TIM_FREQ / BEEP_ALERT_FREQ_VERY_HIGH) - 1;
//		htim13.Instance->CCR1 = (htim13.Instance->ARR * BEEP_ALERT_DUTY / 100) - 1;
//		HAL_TIM_PWM_Start(&htim13, TIM_CHANNEL_1);
//		AM_ON_BEEP++;
//	}
//	else if (AM_ON_BEEP >= 300 && AM_ON_BEEP < 400)
//	{
//		// Beep Off
//		htim13.Instance->CCR1 = 0;
//		HAL_TIM_PWM_Stop(&htim13, TIM_CHANNEL_1);
//		AM_ON_BEEP++;
//	}
//	else if (AM_ON_BEEP >= 400 && AM_ON_BEEP < 500)
//	{
//		// Beep ON
//		htim13.Instance->ARR = (BEEP_ALERT_TIM_FREQ / BEEP_ALERT_FREQ_VERY_HIGH) - 1;
//		htim13.Instance->CCR1 = (htim13.Instance->ARR * BEEP_ALERT_DUTY / 100) - 1;
//		HAL_TIM_PWM_Start(&htim13, TIM_CHANNEL_1);
//		AM_ON_BEEP++;
//	}
//	else if (AM_ON_BEEP >= 500 && AM_ON_BEEP < 600)
//	{
//		// Beep Off
//		htim13.Instance->CCR1 = 0;
//		HAL_TIM_PWM_Stop(&htim13, TIM_CHANNEL_1);
//		AM_ON_BEEP++;
//	}
//	else
//	{
//		AM_ON_BEEP = 0;
//	}

	//	test_ref = traj_calcul(traj_para[traj_select][test_i], traj_cnt, time_amp[traj_select], mag_amp[traj_select][test_i], time_offset[traj_select] * time_amp[traj_select]);
	if (test_d10 == 1)
	{
		for (int i = 6; i < e_L30_NUM; i++)
		{
			Enable_L30_Upright(d10Ptr[i]);
		}
		test_d10 = 0;
	}
	else if (test_d10 == 2)
	{
		for (int i = 6; i < e_L30_NUM; i++)
		{
			Disable_L30_Upright(d10Ptr[i]);
		}
		test_d10 = 0;
	}
	else if (test_d10 == 3)
	{
//
//		test_dc_length[0]=d10Ptr[6]->data.upright_length_actual;
//
//		if(test_dc_length[0]>270){
//			test_dc_length[0]=270;
//		}
//		if(test_dc_length[0]<200){
//			test_dc_length[0]=200;
//		}
//
//		Enable_L30_LengthAct(d10Ptr[e_L30_RH_IDX]);
//		Enable_Upright_lenth_cmd(d10Ptr[e_L30_RH_IDX]);
//		Set_dc_set_length(d10Ptr[e_L30_RH_IDX], test_dc_length[0]);
//		SendAutofittingStopDone();
//
//		autofit_sw=0;

		test_d10 = 0;
	}
	else if (test_d10 == 4)
	{
//		Enable_L30_Upright(d10Ptr[e_L30_RH_IDX]);
//		Enable_L30_LengthAct(d10Ptr[e_L30_RH_IDX]);

		test_d10 = 0;

	}
	else if (test_d10 == 5)
	{


		//		Ready_D10_Position_Ctrl(d10Ptr[0]);
		//		Ready_D10_Position_Ctrl(d10Ptr[1]);
		//		Ready_D10_Position_Ctrl(d10Ptr[2]);
		//		Ready_D10_Position_Ctrl(d10Ptr[3]);
		//
		//		for (int i = 0; i < e_L30_NUM; i++)
		//		{
		//			Enable_L30_Upright(d10Ptr[i]);
		//		}
		//
		//		Upright_state = 0;
		//
		//		Enable_D10_Position_Ctrl(d10Ptr[0]);
		//		Enable_D10_Position_Ctrl(d10Ptr[1]);
		//		Enable_D10_Position_Ctrl(d10Ptr[2]);
		//		Enable_D10_Position_Ctrl(d10Ptr[3]);

//		Push_Routine(&wholeBodyCtrl.routine, 7);
		//		Push_Routine(&ext_dev_ctrl_task.routine, 10);

		test_d10 = 0;
	}
	else if (test_d10 == 6)
	{
//				test_dc_length[0] = test_dc_length[0] - 140;
//
//				if(test_dc_length[0]>270){
//					test_dc_length[0]=270;
//				}
//				if(test_dc_length[0]<200){
//					test_dc_length[0]=200;
//				}
//
//				Enable_L30_LengthAct(d10Ptr[e_L30_RH_IDX]);
//				Enable_Upright_lenth_cmd(d10Ptr[e_L30_RH_IDX]);
//				Set_dc_set_length(d10Ptr[e_L30_RH_IDX], test_dc_length[0]);
//
//				autofit_sw=1;


//		boot_cnt++;
//		AM_ON_BEEP = 1;
		test_d10 = 0;
	}
	else if (test_d10 == 7)
	{
//		Test_Led(d10Ptr[e_L30_RH_IDX]);
//		Test_Led(d10Ptr[e_L30_LH_IDX]);
//		Test_Led(d10Ptr[e_L30_RK_IDX]);
//		Test_Led(d10Ptr[e_L30_LK_IDX]);

		test_d10 = 0;
	}
	else if (test_d10 == 8)
	{
		test_dc_length[0] = 250;
		/*test_dc_length[1] = 220;
		test_dc_length[2] = 220;
		test_dc_length[3] = 220;
*/
		if (length_cnt % 20 == 0)
		{
		test_addr1=&d10Ptr[6]->data.upright_length_command;
			Enable_Upright_lenth_cmd(d10Ptr[e_L30_RH_IDX]);
			Set_dc_set_length(d10Ptr[e_L30_RH_IDX], test_dc_length[0]);
			id_cnt++;
		}

		if (length_cnt++ == 79)
		{
			length_cnt = 0;
			test_d10 = 0;
			id_cnt = 0;
		}
	}
	/*if (test_d10 == 8)
	{
		test_dc_length[0] = 440;
		test_dc_length[1] = 440;
		test_dc_length[2] = 490;
		test_dc_length[3] = 490;

		if (length_cnt % 20 == 0)
		{
			Enable_Upright_lenth_cmd(d10Ptr[id_cnt]);
			Set_dc_set_length(d10Ptr[id_cnt], test_dc_length[id_cnt]);
			id_cnt++;
		}

		if (length_cnt++ == 79)
		{
			length_cnt = 0;
			test_d10 = 0;
			id_cnt = 6;//0;
		}
	}*/

	else if (test_d10 == 9)
	{
		test_dc_length[0] = 230;
//		test_dc_length[1] = 390;
//		test_dc_length[2] = 440;
//		test_dc_length[3] = 440;

		if (length_cnt % 20 == 0)
		{
		test_addr1=&d10Ptr[6]->data.upright_length_command;
			Enable_Upright_lenth_cmd(d10Ptr[e_L30_RH_IDX]);
			Set_dc_set_length(d10Ptr[e_L30_RH_IDX], test_dc_length[0]);
			id_cnt++;
		}

		if (length_cnt++ == 79)
		{
			length_cnt = 0;
			test_d10 = 0;
			id_cnt = 0;
		}

//		if (length_cnt % 20 == 0)
//		{
//			Enable_Upright_lenth_cmd(d10Ptr[id_cnt]);
//			Set_dc_set_length(d10Ptr[id_cnt], test_dc_length[id_cnt]);
//			id_cnt++;
//		}
//
//		if (length_cnt++ == 79)
//		{
//			length_cnt = 0;
//			test_d10 = 0;
//			id_cnt = 6;
//		}
	}

	else if(test_d10==10){
		Enable_L30_LengthAct(d10Ptr[e_L30_RH_IDX]);
		test_d10=0;
	//	Send_L30_Upright_Length_ACT(d10Ptr[0],&d10Ptr[0]->data.upright_length_actual);

	}

	else if(test_d10==11){
		Enable_L30_Upright(d10Ptr[e_L30_RH_IDX]);
		test_d10=0;
	}

//	Run_Routines(&wholeBodyCtrl.routine);

	for(int i=6; i<10; i++){
		Send_PDO_Nothing(d10Ptr[i]);
	}




	if(autofit_sw==1){
		for(int i=0; i <4; i++){
			if(test_dc_length[i] - 1 < d10Ptr[i+6]->data.upright_length_actual && test_dc_length[i] + 1 > d10Ptr[i+6]->data.upright_length_actual){
				Enable_L30_Upright(d10Ptr[i+6]);
	//			Enable_L30_Upright(d10Ptr[e_L30_RH_IDX]);
				autofit_done[i]=1;
			}
		}
	}


	if(autofit_done[0]==1 && autofit_done[1]==1 && autofit_done[2]==1 && autofit_done[3]==1){
		autofit_sw=0;
		SendAutofittingDone();
		for(int i=0; i <4; i++){
			autofit_done[i]=0;
		}
	}

	if(move_sw==1){
		if(move_cnt%50 == 0){
			Enable_L30_LengthAct(d10Ptr[move_motor_num+6]);
			Enable_Upright_lenth_cmd(d10Ptr[move_motor_num+6]);
			Set_dc_set_length(d10Ptr[move_motor_num+6], test_dc_length[move_motor_num]);
			move_motor_num++;
		}
		move_cnt++;
		if(move_motor_num==4){
			move_cnt=0;
			move_motor_num=0;
			move_sw=0;
		}
	}
//
//	d10Ptr[7]->data.upright_length_actual = d10Ptr[6]->data.upright_length_actual + 100;
//	d10Ptr[8]->data.upright_length_actual = d10Ptr[6]->data.upright_length_actual + 200;
//	d10Ptr[9]->data.upright_length_actual = d10Ptr[6]->data.upright_length_actual + 300;

	whole_body_loop_time++;
}

static void StateEnable_Exit()
{
	ExtRoutines(&wholeBodyCtrl.routine);
}

static void StateError_Run()
{

}

/* ------------------- SDO CALLBACK ------------------- */
static void Mode_State(DOP_SDOArgs_t *t_req, DOP_SDOArgs_t *t_res)
{
	int t_cursor = 0;
	uint8_t *t_ids = (uint8_t *)t_req->data;

	for (t_cursor = 0; t_cursor < t_req->dataSize; t_cursor++)
	{
		mode_status[t_cursor + 1] = t_ids[t_cursor];
		if (mode_status[t_cursor + 1] == 2)
		{
			switch (t_cursor + 1)
			{
			case 0:
				mode_select = 0;
				assistance_cnt = 0;
				break;
			case 1:
				mode_select = 1;
				assistance_cnt = 0;
				break;
			case 2:
				mode_select = 2;
				break;
			case 3:
				mode_select = 3;
				assistance_cnt = 0;
				break;
			case 4:
				mode_select = 4;
				break;
			case 5:
				mode_select = 5;
				break;
			case 6:
				mode_select = 6;
				break;
			case 7:
				mode_select = 7;
				assistance_cnt = 0;
				break;
			default:
				mode_select = 0;
				assistance_cnt = 0;
				break;
			}
			break;
		}
		else
		{
			mode_select = 0;
		}
	}

	t_res->dataSize = 1;
	t_res->status = DOP_SDO_SUCC;
}

static void Set_upright_length(DOP_SDOArgs_t *t_req, DOP_SDOArgs_t *t_res)
{
//	static uint8_t monitor[64];
//	memcpy(monitor, t_req->data, 12);
//
	int t_cursor = 0;
	uint8_t* t_ids = (uint8_t *)t_req->data;

	for(int i=0; i<2; i++){
		memcpy(&test_dc_length[i], &t_ids[t_cursor], 4);
		t_cursor+=4;
		if(test_dc_length[i]>440){
			test_dc_length[i]=440;
		}
		if(test_dc_length[i]<340){
			test_dc_length[i]=340;
		}
	}

	for(int i=2; i<4; i++){
		memcpy(&test_dc_length[i], &t_ids[t_cursor], 4);
		t_cursor+=4;
		if(test_dc_length[i]>510){
			test_dc_length[i]=510;
		}
		if(test_dc_length[i]<410){
			test_dc_length[i]=410;
		}
	}

	for(int i=4; i<6; i++){
		memcpy(&test_dc_length[i], &t_ids[t_cursor], 4);
		t_cursor+=4;
		if(test_dc_length[i]>270){
			test_dc_length[i]=270;
		}
		if(test_dc_length[i]<200){
			test_dc_length[i]=200;
		}
	}

//	test_dc_length[0] = test_dc_length[0] - 140;

//   for(int i =6; i<10; i++){
//		Enable_L30_LengthAct(d10Ptr[i]);
//		Enable_Upright_lenth_cmd(d10Ptr[i]);
//		Set_dc_set_length(d10Ptr[i], test_dc_length[i-6]);
//   }

	move_sw=1;
	autofit_sw=1;

	t_res->dataSize = 0;
	t_res->status = DOP_SDO_SUCC;
}

static void Set_test_d10(DOP_SDOArgs_t *t_req, DOP_SDOArgs_t *t_res)
{
	memcpy(&test_d10, t_req->data, 1);

	for(int i=6; i<10; i++){
		Enable_L30_Upright(d10Ptr[i]);
		Enable_L30_LengthAct(d10Ptr[i]);
	}
	t_res->dataSize = 0;
	t_res->status = DOP_SDO_SUCC;
}

static void Stop_upright_move(DOP_SDOArgs_t *t_req, DOP_SDOArgs_t *t_res)
{

	for(int i=0; i<2; i++){
		test_dc_length[i]=d10Ptr[i+6]->data.upright_length_actual;
		if(test_dc_length[i]>440){
			test_dc_length[i]=440;
		}
		if(test_dc_length[i]<340){
			test_dc_length[i]=340;
		}
		Enable_L30_LengthAct(d10Ptr[i+6]);
		Enable_Upright_lenth_cmd(d10Ptr[i+6]);
		Set_dc_set_length(d10Ptr[i+6], test_dc_length[i]);
	}

	for(int i=2; i<4; i++){
		test_dc_length[i]=d10Ptr[i+6]->data.upright_length_actual;
		if(test_dc_length[i]>510){
			test_dc_length[i]=510;
		}
		if(test_dc_length[i]<410){
			test_dc_length[i]=410;
		}
		Enable_L30_LengthAct(d10Ptr[i+6]);
		Enable_Upright_lenth_cmd(d10Ptr[i+6]);
		Set_dc_set_length(d10Ptr[i+6], test_dc_length[i]);
	}

	for(int i=4; i<6; i++){
		if(test_dc_length[i]>270){
			test_dc_length[i]=270;
		}
		if(test_dc_length[i]<200){
			test_dc_length[i]=200;
		}
	}

//	SendAutofittingStopDone();

//	autofit_sw=0;

	t_res->dataSize = 0;
	t_res->status = DOP_SDO_SUCC;
}

/* ------------------- ROUTINES ------------------- */
static int Lengthen_Legs_Upright()
{
	return 0;
}

static int Run_Positon_Control()
{

	static int input_size = sizeof(input) / sizeof(int);
	static int fsm_size = sizeof(fsm_vector) / sizeof(fsm_vector[0]);

	for (int j = 0; j < input_size; j++)
	{
		if (input[j] == 1)
		{
			if (j == fsm_vector[ind][2])
			{

				for (int i = 0; i < fsm_size; i++)
				{
					if (fsm_vector[i][0] == fsm_vector[ind][3])
					{
						ind = i;
						break;
					}
				}

				cur_state = fsm_vector[ind][0];
				cur_audio_id = fsm_vector[ind][1];
				cur_motion_set = fsm_vector[ind][4];
			}

			break;
		}
	}

	for (int j = 0; j < input_size; j++){
		input[j] = 0;
	}

	return 0;
}

static int Get_GRF_Data()
{
	for (int i = 0; i < e_L30_NUM; i++)
	{
		Send_PDO_Nothing(d10Ptr[i]);
	}

	return 0;
}

static int Get_Joints_Absolute_Angle()
{
	for (int i = 0; i < e_L30_NUM; i++)
	{
		Send_PDO_Nothing(d10Ptr[i]);
	}

	return 0;
}

static int Get_D10_Upright_ActLength()
{
	/*for (int i = 0; i < e_L30_NUM; i++)
	{*/
		Send_PDO_Nothing(d10Ptr[e_L30_RH_IDX]);
	//}

	return 0;
/*	uint8_t txBuf[64];
	uint8_t node_id = 0;
	uint16_t sdo_tx_id = SDO | (NODE_ID_CM << 4) | node_id;
    uint8_t n_sdo = 1;
    uint8_t task_id = TASK_ID_MSG;
    uint8_t sdo_id = SDO_ID_MSG_PDO_LIST;
	uint8_t sdo_status = DOP_SDO_REQU;
	uint16_t num_of_data = 1;
	uint32_t data1 = TASK_ID_EXTDEV;
	uint32_t data2= PDO_ID_EXTDEV_DC_LENGTH_ACT;

	uint32_t cursor = 0;

    txBuf[cursor++] = n_sdo;
    txBuf[cursor++] = task_id;
    txBuf[cursor++] = sdo_id;
    txBuf[cursor++] = sdo_status;
    txBuf[cursor++] = num_of_data;
    //cursor += 2;
    txBuf[cursor++] = data1;
    txBuf[cursor++] = data2;
    IOIF_TransmitFDCAN1(sdo_tx_id, cursor, txBuf);

    t_res->dataSize = 1;
    t_res->status = DOP_SDO_SUCC;
*/
}
static void SaveDataMassStorage(void)
{
	static float test = 0;
//    if((dataCtrlLoopCnt - WHOLE_BODY_LOOP_START_CNT + 1) < (MIN_TO_MILLISEC * DATA_SAVE_TIME)) { //@1ms * MIN_TO_MILLISEC * SEND_DATA_SIZE(Byte) * DATA_SAVE_TIME(Min) : 21MByte Transfer within 5 Min.
        /* Data Conversion To String & Save Buffer */
        memset(sdSendData, 0, sizeof(sdSendData));
//        dataSaveCnt = whole_body_loop_time - WHOLE_BODY_LOOP_START_CNT + 1;
        convert_res = snprintf((char*)sdSendData, sizeof(sdSendData) - 2, "%"PRIu32"; %.2f;%.2f;%.2f;%.2f;",
        		whole_body_loop_time, test, test+0.1, test+0.2, test+0.3, test+0.4);
        test++;
        // Check if there was enough space for the content and the CRLF, and null terminator.
        if (convert_res >= 0 && convert_res < sizeof(sdSendData) - 2) {
            // Append '\r', '\n', and null terminator '\0'.
            sdSendData[convert_res] = '\r'; // Carriage Return
            sdSendData[convert_res + 1] = '\n'; // Line Feed
            // sdSendData[convert_res + 2] = '\0'; // Null terminator to mark end of the string
        }

        /* Buffered Transmit to SD */
        _SD_DBuffered_Transmit(sdSendData, sizeof(sdSendData));
//    }
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/* Key Function : SDIO Transmit with double buffer & swap(Switching) buffer algorithm */
static inline void _SD_DBuffered_Transmit(uint8_t *data, uint_fast16_t data_len)
{
    osMutexAcquire(mutex_sd_buffering, BUFFER_SD_TIMEOUT);					// Mutex lock

    uint_fast16_t *cur_buf_pos = (sd_buf_cur == sd_buf_1) ? &sd_buf_1_pos : &sd_buf_2_pos;
    uint_fast16_t cur_buf_space = SWAP_SD_BUF_SIZE - *cur_buf_pos;

    if (cur_buf_space >= data_len) {
        memcpy(sd_buf_cur + *cur_buf_pos, data, data_len);
        *cur_buf_pos += data_len;
    } else {
        memcpy(sd_buf_cur + *cur_buf_pos, data, cur_buf_space);
        uint_fast16_t remaining_data_len = data_len - cur_buf_space;

        if (sd_buf_cur == sd_buf_1) {
            sd_buf_1_pos += cur_buf_space;
            sd_buf_cur = sd_buf_2;
            cur_buf_pos = &sd_buf_2_pos;
            sd_buf_2_pos = 0;
        } else {
            sd_buf_2_pos += cur_buf_space;
            sd_buf_cur = sd_buf_1;
            cur_buf_pos = &sd_buf_1_pos;
            sd_buf_1_pos = 0;
        }

        memcpy(sd_buf_cur, data + cur_buf_space, remaining_data_len);
        *cur_buf_pos = remaining_data_len;

        osSemaphoreRelease(sdio_sync_semaphore);							// release semaphore : start writing buffered data to SD card
    }

    osMutexRelease(mutex_sd_buffering);										// Mutex unlock
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/* Data Store Test by TJ */
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
static void intToStr(int num, char str[], int maxSize)
{
    int i = 0;
    int isNegative = 0;

    // 음수 체크
    if (num < 0) {
        isNegative = 1;
        num = -num;
    }

    // 숫자를 역순으로 추출하여 문자열에 저장
    do {
        str[i++] = num % 10 + '0';
        num /= 10;
    } while (num > 0);

    // 음수일 경우 '-'를 추가
    if (isNegative) {
        str[i++] = '-';
    }

    // 문자열 뒤집기
    int len = i;
    for (int j = 0; j < len / 2; j++) {
        char temp = str[j];
        str[j] = str[len - j - 1];
        str[len - j - 1] = temp;
    }

    // 널 종료 문자('\0') 추가
    str[i] = '\0';

    // 문자열 길이가 maxSize보다 작을 경우 나머지를 널 바이트로 채움
    for (int j = len; j < maxSize; j++) {
        str[j] = '\0';
    }
}

static void fillArrayWithVariable(int var, char byteArray[], int arraySize)
{
    // 배열을 널 바이트로 초기화합니다.
    memset(byteArray, 0, arraySize);

    // 변수를 문자열로 변환합니다.
    char var_str[21];  // 최대 20자리 문자열 + 널 종료 문자('\0') 고려
    intToStr(var, var_str, sizeof(var_str));

    // 문자열을 배열에 복사합니다.
    int i;
    for (i = 0; i < arraySize - 2; i++) { // 2 바이트를 더 남겨두고 "\r\n"을 추가합니다.
        byteArray[i] = var_str[i];
    }

    // 끝에 "\r\n" 추가합니다.
    byteArray[i++] = '\r';
    byteArray[i++] = '\n';
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/* Data Processing 2024.01.30 */
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
static void ReverseStr(char* str, int length)
{
    int start = 0;
    int end = length - 1;
    while (start < end) {
        char temp = str[start];
        str[start] = str[end];
        str[end] = temp;
        start++;
        end--;
    }
}

static void IntToStringRec(int value, char* output, int* index)
{
    if (value == 0) return;

    int digit = value % 10;
    if (digit < 0) digit = -digit;

    output[(*index)++] = digit + '0';

    IntToStringRec(value / 10, output, index);
}

static void IntToString(int value, char* output)
{
    int index = 0;
    if (value < 0) {
        output[index++] = '-'; // 음수 부호 추가
        value = -value; // 값의 절대값을 사용합니다.
    }

    if (value == 0) {
        output[index++] = '0';
    } else {
        IntToStringRec(value, output, &index);
        if (value < 0) {
            ReverseStr(output + 1, index - 1);
        } else {
            ReverseStr(output, index);
        }
    }
    output[index] = '\0';
}

static int FindStringLength(char* str, int index)
{
    if (str[index] == '\0') {
        return index;
    }
    return FindStringLength(str, index + 1);
}

static void FloatToString(float value, char* output, int afterpoint)
{
    // Extract integer part and convert
    int ipart = (int)value;
    IntToString(ipart, output);

    // 문자열 길이 찾기
    int i = 0;
    while (output[i] != '\0') i++;

    // Handle fractional part
    if (afterpoint > 0) {
        output[i++] = '.';
        float fpart = value - (float)ipart;
        if (value < 0) fpart = -fpart; // 음수 처리

        // 소수점 이하 값을 문자열로 변환
        for (int j = 0; j < afterpoint; j++) {
            fpart *= 10.0;
            int digit = (int)fpart;
            output[i++] = '0' + digit;
            fpart -= digit;
        }
        output[i] = '\0';
    }
}

static void FillBufferData(DataPoint data[], int numDataPoints, char buffer[], int bufferSize)
{
    // 배열을 널 바이트로 초기화합니다.
    memset(buffer, 0, bufferSize);

    int i = 0;

    for (int j = 0; j < numDataPoints; j++) {
        char tempStr[16]; // 임시 문자열 버퍼(각 데이터마다 1:1대응), float의 최대 표현 자리수(16자리수)
        memset(tempStr, 0, sizeof(tempStr)); // tempStr을 0으로 초기화

        osMutexAcquire(mutex_sd_buffering, BUFFER_SD_TIMEOUT);    // Mutex lock

        // 각 데이터 타입에 따른 처리
        switch (data[j].DataTp) {
            case DATATYPE_UINT32:
                IntToString((int)data[j].uint32Data, tempStr);
                break;
            case DATATYPE_INT:
                IntToString(data[j].intData, tempStr);
                break;
            case DATATYPE_INT16:
                IntToString((int)data[j].int16Data, tempStr);
                break;
            case DATATYPE_FLOAT:
                FloatToString(data[j].floatData, tempStr, 2); // 소수점 이하 2자리
                break;
            default:
                tempStr[0] = '\0'; // 알 수 없는 데이터 타입 처리
        }

        // 버퍼에 문자열과 구분자 추가
        int len = strlen(tempStr);
        if (i + len + 2 < bufferSize - 2) { // +2는 구분자 공간, -2는 CRLF 공간
            strncpy(&buffer[i], tempStr, len);
            i += len;

            // 모든 데이터에 구분자 추가
            buffer[i++] = ';';
            buffer[i++] = ' ';
        }

        osMutexRelease(mutex_sd_buffering);    // Mutex unlock
    }

    osMutexAcquire(mutex_sd_buffering, BUFFER_SD_TIMEOUT);    // Mutex lock

    // 버퍼의 나머지 부분을 null로 채우기
    while (i < bufferSize - 2) {
        buffer[i++] = '\0';
    }

    // 버퍼의 끝에 "\r\n" 추가
    buffer[i++] = '\r';
    buffer[i++] = '\n';

    osMutexRelease(mutex_sd_buffering);    // Mutex unlock
}
#endif /* L30_CM_ENABLED */ 
