

#ifndef CM_BACKBONE_HW_DEPENDENCIES_ANGELSUIT_AS_ISI_H_
#define CM_BACKBONE_HW_DEPENDENCIES_ANGELSUIT_AS_ISI_H_

#include "module.h"

#ifdef SUIT_MINICM_ENABLED

#include <stdint.h>
#include "data_object_common.h"

#include "AS_dev_mngr.h"

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

typedef struct _ISI_Flag_t {
    uint8_t I1_Flag;    // walking
    uint8_t I2_Flag;    // Gait Phase 10%
    uint8_t I3_Flag;    // Gait Phase 20%
    uint8_t I4_Flag;    // Gait Phase 30%
    uint8_t I5_Flag;    // Gait Phase 40%
    uint8_t I6_Flag;    // Gait Phase 50%
    uint8_t I7_Flag;    // Gait Phase 0%  Fowared->Back Transition(Leg front->back & Gait Count ++)
    uint8_t I8_Flag;    // Gait Phase 50% Back->Foward Transition(Leg back->front)
    uint8_t I9_Flag;    // Gait Phase 75% Back->Foward Moving(Leg middle->front)
    uint8_t I10_Flag;   // Gait Phase 25% Foward->Back Moving(Leg middle->back)
    uint8_t I11_Flag;   // Gait Phase 60%
    uint8_t I12_Flag;   // Gait Phase 70%
    uint8_t I13_Flag;   // Gait Phase 80%
    uint8_t I14_Flag;   // Gait Phase 90%
    uint8_t I15_Flag;   // I8 -> for extension incVel == 0
    uint8_t I16_Flag;   // 1NE's revised Extension timing
    uint8_t I17_Flag;   // 1.8 ~ 2 sec
    uint8_t I18_Flag;   // 1.5 ~ 1.8 sec
    uint8_t I19_Flag;   // 1.2 ~ 1.5 sec
    uint8_t I20_Flag;   // 0.8 ~ 1.2 sec
    uint8_t I21_Flag;   // 0 ~ 0.8 sec

    uint8_t I22_Mode; // Mode

    uint8_t I7_Finished;
    uint8_t I8_Finished;
} ISI_Flag_t;

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


/**
 *------------------------------------------------------------
 *                      GLOBAL VARIABLES
 *------------------------------------------------------------
 * @brief Extern declarations for global variables.
 */

extern ISI_Flag_t ISIFlags_Bt;

extern ISI_Flag_t ISIFlags_RH;
extern ISI_Flag_t ISIFlags_LH;

extern ISI_Flag_t ISIFlags_RK;
extern ISI_Flag_t ISIFlags_LK;

extern ISI_Flag_t ISIFlags_RA;
extern ISI_Flag_t ISIFlags_LA;

extern uint8_t isi_input_vectors[ISI_INPUT_N_MAX];
extern uint8_t isi_output_vectors[ISI_OUTPUT_N_MAX];


/**
 *------------------------------------------------------------
 *                     FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Function prototypes declaration for this module.
 */

void Check_ISI(void);
void Flush_ISI(void);


#endif /* SUIT_MINICM_ENABLED */

#endif /* CM_BACKBONE_HW_DEPENDENCIES_ANGELSUIT_AS_ISI_H_ */
