

#ifndef BLE_COMM_HDLR_INC_AS_BLE_COMM_HDLR_H_
#define BLE_COMM_HDLR_INC_AS_BLE_COMM_HDLR_H_

#include "module.h"

#ifdef SUIT_MINICM_ENABLED

#include <stdint.h>

#include "data_object_common.h"

#include "data_object_dictionaries.h"

#include "ioif_tim_common.h"
#include "ioif_mdbt42q-at.h"

/**
 *-----------------------------------------------------------
 *              MACROS AND PREPROCESSOR DIRECTIVES
 *-----------------------------------------------------------
 * @brief Directives and macros for readability and efficiency.
 */

#define TEST_SEND_PACKET		"Send Test Packet transmit, BLE is connected successfully!!\r\n"


/**
 *------------------------------------------------------------
 *                     TYPE DECLARATIONS
 *------------------------------------------------------------
 * @brief Custom data types and structures for the module.
 */

typedef enum _BLEState {
    BLE_NOT_CONNECTED,
    BLE_CONNECTED,
    BLE_PARING,
    BLE_ERROR,
} BLEState;


/**
 *------------------------------------------------------------
 *                      GLOBAL VARIABLES
 *------------------------------------------------------------
 * @brief Extern declarations for global variables.
 */

extern BLEState isBLEConnect;
extern uint8_t powerOffCmdByBLE;

/**
 *------------------------------------------------------------
 *                     FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Function prototypes declaration for this module.
 */

void InitBLEComm(void);
void RunBLEComm(void);


#endif /* SUIT_MINICM_ENABLED */

#endif /* BLE_COMM_HDLR_INC_AS_BLE_COMM_HDLR_H_ */
