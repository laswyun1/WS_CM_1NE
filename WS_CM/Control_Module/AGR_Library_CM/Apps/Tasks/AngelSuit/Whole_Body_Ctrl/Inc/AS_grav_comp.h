#ifndef APPS_TASKS_ANGELSUIT_WHOLE_BODY_CTRL_INC_AS_GRAV_COMP_H_
#define APPS_TASKS_ANGELSUIT_WHOLE_BODY_CTRL_INC_AS_GRAV_COMP_H_

#include "module.h"

#ifdef SUIT_MINICM_ENABLED

#include <math.h>
#include <stdint.h>

/**
 *-----------------------------------------------------------
 *              MACROS AND PREPROCESSOR DIRECTIVES
 *-----------------------------------------------------------
 * @brief Directives and macros for readability and efficiency.
 */

// Common Parameters
#define GRAV_COMP_N_JOINTS          2       // joint
#define GRAV_COMP_CONST_GRAV_ACC    9.81    // m/s^2
#define GRAV_COMP_DELTA_T           0.001   // 1ms
#define GRAV_COMP_TORQUE_CONSTANT   0.085   // Nm/A
#define GRAV_COMP_GEAR_RATIO        18.75   // Motor Gear Ratio
#define GRAV_COMP_SIGN_EPSILON      0.2     // 조정 가능

// Individual Pendulum Parameters
#define GRAV_COMP_PARAM_MASS             0.171   // kg
#define GRAV_COMP_PARAM_LINK_LENGTH      0.2845  // m


/**
 *------------------------------------------------------------
 *                     TYPE DECLARATIONS
 *------------------------------------------------------------
 * @brief Custom data types and structures for the module.
 */

typedef struct _GravComp_ForceTorque_t {
    float resForce;     // Force  [N]
    float resTorque;    // Torque [Nm]
    float resCurrent;   // Current [A]
} GravComp_ForceTorqueCurr_t;

typedef struct _GravCompObj_t {
    GravComp_ForceTorqueCurr_t  FTC;
    float mass;                 // mass [kg]
    float lengthOriginToCOM;    // origin-COM distance [m]
    float lengthOriginToEnd;    // origin-endpoint distance [m]
    float COMtheta;             // COM displacement angle [rad]
} GravCompObj_t;


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

float Deg2Rad(float deg);
float Vel2RadPerSec(float vel);
void InitGravityCompParams(GravCompObj_t* gravCompObj);
void GravityCompensator(GravCompObj_t* gravCompObj, float deg);


#endif /* SUIT_MINICM_ENABLED */

#endif /* APPS_TASKS_ANGELSUIT_WHOLE_BODY_CTRL_INC_AS_GRAV_COMP_H_ */
