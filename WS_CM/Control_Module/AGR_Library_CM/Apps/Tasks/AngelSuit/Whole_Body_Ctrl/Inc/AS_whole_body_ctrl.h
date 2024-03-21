

#ifndef WHOLE_BODY_CTRL_INC_AS_WHOLE_BODY_CTRL_H_
#define WHOLE_BODY_CTRL_INC_AS_WHOLE_BODY_CTRL_H_

#include "module.h"

#ifdef SUIT_MINICM_ENABLED

#include <inttypes.h> // for uint32_t printf type
#include <stdint.h>
#include <string.h>

#include "robot_setting.h"

#include "AS_dev_mngr.h"
#include "data_object_common.h"
#include "ring_buffer.h"

#include "ioif_tim_common.h"
#include "ioif_gpio_common.h"

#include "AS_system_ctrl.h"

#include "AS_gait_ctrl.h"
#include "AS_imu_ctrl.h"

#include "AS_ISI.h"

#include "AS_grav_comp.h"

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
#define BUFFER_SD_TIMEOUT			1000
#define SEND_DATA_SIZE              50
#define MILLISEC_TO_SEC             1000
#define MIN_TO_SEC                  60
#define MIN_TO_MILLISEC             (MIN_TO_SEC * MILLISEC_TO_SEC)
#define DATA_SAVE_TIME              5

 
/**
 *------------------------------------------------------------
 *                     TYPE DECLARATIONS
 *------------------------------------------------------------
 * @brief Custom data types and structures for the module.
 */

// Enum to represent data types
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

extern uint8_t pwrBtPushed;

/* For Data Test */
/* extern variable */
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


#endif /* SUIT_MINICM_ENABLED */

#endif /* WHOLE_BODY_CTRL_INC_AS_WHOLE_BODY_CTRL_H_ */
