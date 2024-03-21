#ifndef WHOLE_BODY_CTRL_INC_WS_WHOLE_BODY_CTRL_H_
#define WHOLE_BODY_CTRL_INC_WS_WHOLE_BODY_CTRL_H_

#include "module.h"

#ifdef WALKON5_CM_ENABLED

#include "main.h"

#include <math.h>

#include "data_object_common.h"

#include "robot_DMS.h"
#include "robot_FSM.h"
#include "robot_motion_map.h"
#include "robot_setting.h"
#include "WS_ISI.h"
#include "device_id.h"

#include "WS_dev_mngr.h"
#include "WS_ext_dev_ctrl.h"
#include "WS_am_comm_hdlr.h"

#include <inttypes.h>
#include "grav_comp.h"
#include "WalkONV.h"




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

typedef struct
{
	float abs_pos[MAX_N_MD];
    float inc_pos[MAX_N_MD];
    float vel[MAX_N_MD];
    float cur_act[MAX_N_MD];
    float cur_ref[MAX_N_MD];
    float pos_ref[MAX_N_MD];

    float auxilirary_input[MAX_N_MD];

    uint32_t index[MAX_N_MD];
    Quaternion quaternion[MAX_N_MD];

    float tilt_angle[MAX_N_MD];
} JointStruct;


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

extern uint8_t power_state;

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

// Main
void InitWholeBodyCtrl();
void RunWholeBodyCtrl();


#endif /* WALKON5_CM_ENABLED */

#endif /* WHOLE_BODY_CTRL_INC_WS_WHOLE_BODY_CTRL_H_ */
