#ifndef WHOLE_BODY_CTRL_INC_GRAV_COMP_H_
#define WHOLE_BODY_CTRL_INC_GRAV_COMP_H_

#include "module.h"

#ifdef WALKON5_CM_ENABLED

#include <math.h>
#include <stdint.h>
#include <string.h>

/**
 *-----------------------------------------------------------
 *              MACROS AND PREPROCESSOR DIRECTIVES
 *-----------------------------------------------------------
 * @brief Directives and macros for readability and efficiency.
 */

#define GRAV_COMP_N_JOINTS 4
#define GRAV_COMP_CONST_GRAV_ACC 9.81

#define GRAV_COMP_DIR_FIX_LH -1
#define GRAV_COMP_DIR_FIX_LK -1
#define GRAV_COMP_DIR_FIX_RH  1
#define GRAV_COMP_DIR_FIX_RK  1

#define GRAV_COMP_2D_GRAV_FORWARD  1
#define GRAV_COMP_2D_GRAV_BACKWARD 0

#define GRAV_COMP_BASIC_OPTION_HANGED_ENB 0b01
#define GRAV_COMP_BASIC_OPTION_STANCE_ENB 0b10

#define GRAV_COMP_STANCE_NONE      0
#define GRAV_COMP_STANCE_SINGLE_LT 1
#define GRAV_COMP_STANCE_SINGLE_RT 2
#define GRAV_COMP_STANCE_DOUBLE    3

#define GRAV_COMP_DEFAULT_PARAM_TORSO_M          2.0 // [kg]
#define GRAV_COMP_DEFAULT_PARAM_TORSO_L          0.0 // [m]
#define GRAV_COMP_DEFAULT_PARAM_TORSO_LC         0.2 // [m]
#define GRAV_COMP_DEFAULT_PARAM_TORSO_TH (M_PI*0.75) // [rad]

#define GRAV_COMP_DEFAULT_PARAM_THIGH_M          1.0 // [kg]
#define GRAV_COMP_DEFAULT_PARAM_THIGH_L          0.4 // [m]
#define GRAV_COMP_DEFAULT_PARAM_THIGH_LC         0.3 // [m]
#define GRAV_COMP_DEFAULT_PARAM_THIGH_TH         0.0 // [rad]

#define GRAV_COMP_DEFAULT_PARAM_SHANK_M          0.5 // [kg]
#define GRAV_COMP_DEFAULT_PARAM_SHANK_L          0.4 // [m]
#define GRAV_COMP_DEFAULT_PARAM_SHANK_LC         0.3 // [m]
#define GRAV_COMP_DEFAULT_PARAM_SHANK_TH         0.0 // [rad]


/**
 *------------------------------------------------------------
 *                     TYPE DECLARATIONS
 *------------------------------------------------------------
 * @brief Custom data types and structures for the module.
 */

typedef enum {
    DoubleStance,
    LtSingleStance,
    RtSingleStance
} GravComp_CompType;

#pragma pack(1)

typedef struct {
    float m;  // mass [kg]
    float lc; // origin-COM distance [m]
    float l;  // origin-endpoint distance [m]
    float th; // COM displacement angle [rad]
} GravComp_BodyParams;

typedef struct {
    GravComp_BodyParams torso;
    GravComp_BodyParams thigh_lt;
    GravComp_BodyParams thigh_rt;
    GravComp_BodyParams shank_lt;
    GravComp_BodyParams shank_rt;
} GravComp_Params;

typedef struct {
    float f; // Force  [N]
    float t; // Torque [Nm]
} GravComp_ForceTorque;


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

void GravComp_Init();
void GravComp_SetParams(GravComp_Params* param_in);
GravComp_Params* GravComp_GetParams();

void GravCmopTest(float* res,
    float torso_roll_deg, float torso_pitch_deg,
    float* pos_deg);

void GravCmopTest2(float* res,
    float torso_roll_deg, float torso_pitch_deg,
    float* pos_deg,
	uint8_t is_left);

void GravCmopTest3(float* res,
    float torso_roll_deg, float torso_pitch_deg,
    float* pos_deg,
	float left_ratio);

void GravComp_BasicGravComp(float* res,
    float* torso_rpy, float* pelvic_abd,
    float* joint_pos,
    uint8_t* ground_contact, float* ground_reaction,
    uint8_t option);


#endif /* WALKON5_CM_ENABLED */
    
#endif /* WHOLE_BODY_CTRL_INC_GRAV_COMP_H_ */
