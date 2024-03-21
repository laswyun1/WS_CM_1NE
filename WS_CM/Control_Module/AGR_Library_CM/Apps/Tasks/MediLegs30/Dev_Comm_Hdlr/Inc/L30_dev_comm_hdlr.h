

#ifndef DEV_COMM_HDLR_INC_L30_DEV_COMM_HDLR_H_
#define DEV_COMM_HDLR_INC_L30_DEV_COMM_HDLR_H_

#include "module.h"

#ifdef L30_CM_ENABLED

#include <stdint.h>

#include "usbd_cdc_if.h"
#include "data_object_common.h"
#include "L30_dev_mngr.h"
#include "ioif_tim_common.h"
#include "ioif_gpio_common.h"
#include "ioif_spi_common.h"
#include "ioif_fdcan_common.h"

/**
 *-----------------------------------------------------------
 *              MACROS AND PREPROCESSOR DIRECTIVES
 *-----------------------------------------------------------
 * @brief Directives and macros for readability and efficiency.
 */

#define MEMORY_SECOND_HAND_CHECK	1


/**
 *------------------------------------------------------------
 *                     TYPE DECLARATIONS
 *------------------------------------------------------------
 * @brief Custom data types and structures for the module.
 */

typedef enum {
	FIRST_USE = 0x01,
	SECOND_USE,
	MEMORY_UI_SYNC1,
	MEMORY_UI_SYNC2,
	MEMORY_UI_SYNC3,
	MEMORY_UI_SYNC4,
	E_SYS_BATCH,
	E_SYS_BEMF,
	M_SYS_DATA_COMM,
	E_VERIFY_DATA_COMM,
	BW_CHECK,
	GAIN_TUNER,
	GET_ENCODER,
	GET_VELOCITY_CTRL,
	GET_POSITION_CTRL,
	GET_REF,
	GET_FRICTION_ID,
	GET_MECH_SYS_ID_SBS,
	GET_VELOCITY_KALMAN,
	GET_ABS_ENC
} GUISequence_Enum;

typedef enum {
	IDLE,
	UPLOAD_PROPERTIES,
	SAVE_PROPERTIES,
	DOWNLOAD_PROPERTIES,
	ELECTRICAL_SYSTEM_ID,
	BEMF_ID,
	ELECTRICAL_SYSTEM_VERIFY,
	MECHANICAL_SYSTEM_ID,
	CURRENT_BANDWIDTH_CHECK,
	AUTO_TUNING,
	FRICTION_ID_LSTSQ
} MainSequence_Enum;

typedef enum _COMMType {
	e_FDCAN = 0U,
	e_USB
} COMMType;


/**
 *------------------------------------------------------------
 *                      GLOBAL VARIABLES
 *------------------------------------------------------------
 * @brief Extern declarations for global variables.
 */

extern TaskObj_t devCommHdlr;
extern MainSequence_Enum MS_enum;


/**
 *------------------------------------------------------------
 *                     FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Function prototypes declaration for this module.
 */

void InitDevCommHdlr(void);
void RunDevCommHdlr(void);

void Send_power_off(void);
void SendEMCY(void);
void SendEMCY_disable(void);
void SendMotionDone(void);
void SendRobotOff(void);
void SendAutofittingDone(void);

void Send_EMCY(uint32_t* f_err_code);
int Send_MSG(uint16_t COB_ID, uint32_t len, uint8_t* tx_data);
int Send_MSG_FDCAN(uint16_t COB_ID, uint32_t len, uint8_t* tx_data);

#endif /* L30_CM_ENABLED */ 

#endif /* DEV_COMM_HDLR_INC_L30_DEV_COMM_HDLR_H_ */
