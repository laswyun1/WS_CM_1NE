

#ifndef CM_BACKBONE_HW_DEPENDENCIES_WALKONSUIT5_WS_ISI_H_
#define CM_BACKBONE_HW_DEPENDENCIES_WALKONSUIT5_WS_ISI_H_

#include "module.h"

#ifdef WALKON5_CM_ENABLED

#include <stdint.h>
#include "data_object_common.h"
#include "ioif_gpio_common.h"

/**
 *-----------------------------------------------------------
 *              MACROS AND PREPROCESSOR DIRECTIVES
 *-----------------------------------------------------------
 * @brief Directives and macros for readability and efficiency.
 */

#define ACTIVE_LOW 			0
#define ACTIVE_HIGH 		1
#define ISI_INPUT_N_MAX 	30
#define ISI_OUTPUT_N_MAX 	30


/**
 *------------------------------------------------------------
 *                     TYPE DECLARATIONS
 *------------------------------------------------------------
 * @brief Custom data types and structures for the module.
 */

typedef enum _ExtCondID {

	EXT0,
	EXT1,
	EXT2,
	EXT3,
	EXT4,
	EXT5,
	EXT6,
	EXT7,

	EXT_COND_ID_NUM
} ExtCondID;


typedef enum _InputID {

	BUTTON0,
	BUTTON1,

	INPUT_ID_NUM
} InputID;

typedef struct _ButtonObj {
	GPIO_TypeDef* GPIO_port;
	uint16_t GPIO_pin;

	uint8_t idle_state; // 0: Active LOW, 1: Active HIGH

	uint8_t state_curr;
	uint8_t state_prev;
	uint32_t on_time;

} ButtonObj;


/**
 *------------------------------------------------------------
 *                      GLOBAL VARIABLES
 *------------------------------------------------------------
 * @brief Extern declarations for global variables.
 */

extern uint8_t isi_input_vectors[ISI_INPUT_N_MAX];
extern uint8_t isi_output_vectors[ISI_OUTPUT_N_MAX];


/**
 *------------------------------------------------------------
 *                     FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Function prototypes declaration for this module.
 */

void Init_Button_Interface(void);
void Check_ISI(void);
void Flush_ISI(void);


#endif /* WALKON5_CM_ENABLED */

#endif /* CM_BACKBONE_HW_DEPENDENCIES_WALKONSUIT5_WS_ISI_H_ */
