#ifndef WHOLE_BODY_CTRL_INC_WALKONV_H_
#define WHOLE_BODY_CTRL_INC_WALKONV_H_

#include "module.h"

#ifdef WALKON5_CM_ENABLED

#include <math.h>
#include <stdint.h>

/**
 *-----------------------------------------------------------
 *              MACROS AND PREPROCESSOR DIRECTIVES
 *-----------------------------------------------------------
 * @brief Directives and macros for readability and efficiency.
 */


/* Kinematics Transformation */
#define TRANSFORM_LUT_SIZE         4097

#define MAX_ANGLE_HIP_ABDUCTION      15    // unit: deg (+)
#define MAX_ANGLE_HIP_ADDUCTION      10    // unit: deg (-)
#define MAX_ANGLE_HIP_FLEXION       112    // unit: deg (+)
#define MAX_ANGLE_HIP_EXTENSION      30    // unit: deg (-)
#define MAX_ANGLE_KNEE_FLEXION      120    // unit: deg (+)
#define MAX_ANGLE_KNEE_EXTENSION      0    // unit: deg (-)
#define MAX_ANGLE_ANKLE_EVERSION_    10    // unit: deg (+)
#define MAX_ANGLE_ANKLE_INVERSION    15    // unit: deg (-)
#define MAX_ANGLE ANKLE_DORSI        20    // unit: deg (+)
#define MAX_ANGLE_ANKLE_PLANTAR      30    // unit: deg (-)


/**
 *------------------------------------------------------------
 *                     TYPE DECLARATIONS
 *------------------------------------------------------------
 * @brief Custom data types and structures for the module.
 */

typedef enum _ErrorID {

	ERROR_RH_COR_INIT_POSTURE,
	ERROR_LH_COR_INIT_POSTURE,
	ERROR_RH_ROT_INIT_POSTURE,
	ERROR_LH_ROT_INIT_POSTURE,
	ERROR_RH_SAG_INIT_POSTURE,
	ERROR_LH_SAG_INIT_POSTURE,
	ERROR_RK_INIT_POSTURE,
	ERROR_LK_INIT_POSTURE,
	ERROR_RA_INVEV_INIT_POSTURE,
	ERROR_LA_INVEV_INIT_POSTURE,
	ERROR_RA_DORSIPLANTAR_INIT_POSTURE,
	ERROR_LA_DORSIPLANTAR_INIT_POSTURE,

	ERROR_ID_NUM
} ErrorID;

typedef struct _Init_IncEnc_Angle {
	float RH_COR_init_inc_angle;
	float LH_COR_init_inc_angle;
	float RH_ROT_init_inc_angle;
	float LH_ROT_init_inc_angle;
	float RH_SAG_init_inc_angle;
	float LH_SAG_init_inc_angle;
	float RK_init_inc_angle;
	float LK_init_inc_angle;
	float RA_MED_init_inc_angle;
	float LA_MED_init_inc_angle;
	float RA_LAT_init_inc_angle;
	float LA_LAT_init_inc_angle;
} Init_IncEnc_Angle;


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

float abs2inc_RH_COR(float x);
float abs2inc_LH_COR(float x);
float abs2inc_RH_ROT(float x);
float abs2inc_LH_ROT(float x);
float abs2inc_RH_SAG(float x);
float abs2inc_LH_SAG(float x);
float abs2inc_RK(float x);
float abs2inc_LK(float x);
float abs2inc_RA_MED(float x_ie, float x_dp);
float abs2inc_RA_LAT(float x_ie, float x_dp);
float abs2inc_LA_MED(float x_ie, float x_dp);
float abs2inc_LA_LAT(float x_ie, float x_dp);


#endif /* WALKON5_CM_ENABLED */

#endif /* WHOLE_BODY_CTRL_INC_WALKONV_H_ */
