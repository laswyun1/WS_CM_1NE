

#ifndef WHOLE_BODY_CTRL_TASK_INC_L30_WHOLE_BODY_CTRL_TASK_H_
#define WHOLE_BODY_CTRL_TASK_INC_L30_WHOLE_BODY_CTRL_TASK_H_

#include "module.h"

#ifdef L30_CM_ENABLED

#include "data_object_common.h"
#include "ioif_tim_common.h"
#include "L30_dev_mngr.h"

#include "L30_dev_comm_hdlr.h"
#include "L30_ext_dev_ctrl.h" //dh_temp
#include <inttypes.h>

/**
 *-----------------------------------------------------------
 *              MACROS AND PREPROCESSOR DIRECTIVES
 *-----------------------------------------------------------
 * @brief Directives and macros for readability and efficiency.
 */

#define MAX_ZERO(value) ((value) > 0 ? (value) : 0)
#define WHOLE_BODY_LOOP_START_CNT       5000

/* For Data Test */
#define SWAP_SD_BUF_SIZE  			4096
#define BUFFER_SD_TIMEOUT			5000
#define SEND_DATA_SIZE              50
#define MIN_TO_SEC                  60
#define MIN_TO_MILLISEC             (MIN_TO_SEC * MILLISEC_TO_SEC)
#define DATA_SAVE_TIME              5


/**
 *------------------------------------------------------------
 *                     TYPE DECLARATIONS
 *------------------------------------------------------------
 * @brief Custom data types and structures for the module.
 */

typedef struct _PIDObject {
	float ref;
	float act;

	float Kp;
	float Ki;
	float Kd;

	float R;		// Input penalty in LQ,   q1=1, q2=0

	float control_input;

	float err;
	float err_sum;
	float err_diff;
} PIDObject;

typedef enum DataType_t {
    DATATYPE_UINT32,
    DATATYPE_INT,
    DATATYPE_INT16,
	DATATYPE_FLOAT
} SaveDataTp;

// Structure to represent a data point
typedef struct DataPoint_t {
    int intData;
    float floatData;
    uint32_t uint32Data;
    int16_t int16Data;

    SaveDataTp DataTp;
} DataPoint;
/**
 *------------------------------------------------------------
 *                      GLOBAL VARIABLES
 *------------------------------------------------------------
 * @brief Extern declarations for global variables.
 */

extern TaskObj_t wholeBodyCtrlTask;
extern uint8_t test_d10;
extern uint8_t AM_ON_BEEP;

extern osSemaphoreId_t sdio_sync_semaphore;

extern uint8_t *sd_buf_cur;
extern uint8_t *sd_buf_next;

extern uint8_t sd_buf_1[SWAP_SD_BUF_SIZE];
extern uint8_t sd_buf_2[SWAP_SD_BUF_SIZE];

extern uint8_t sdSendData[SEND_DATA_SIZE];

extern uint8_t dataSaveFinished;
/**
 *------------------------------------------------------------
 *                     FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Function prototypes declaration for this module.
 */

void InitWholeBodyCtrl(void);
void RunWholeBodyCtrl(void);

void Define_Traj(void);
void Init_Training_Routine(void);
float traj_calcul(float traj_para[], int time, float time_amp, float mag_amp, int time_offset);
void Run_PID_Control(PIDObject *t_PID_obj, float t_ref, float t_actual, float t_period);


#endif /* L30_CM_ENABLED */

#endif /* WHOLE_BODY_CTRL_TASK_INC_L30_WHOLE_BODY_CTRL_TASK_H_ */
