#include "AS_grav_comp.h"

#ifdef SUIT_MINICM_ENABLED

/**
 *-----------------------------------------------------------
 *              TYPE DEFINITIONS AND ENUMERATIONS
 *-----------------------------------------------------------
 * @brief Enumerated types and structures central to this module.
 */




/**
 *------------------------------------------------------------
 *                          VARIABLES
 *------------------------------------------------------------
 * @brief Variables accessible throughout the application.
 */



/**
 *------------------------------------------------------------
 *                 STATIC FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Static Function prototypes for this module.
 */



/**
 *------------------------------------------------------------
 *                      PUBLIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions that interface with this module.
 */

void InitGravityCompParams(GravCompObj_t* gravCompObj)
{
    gravCompObj->mass = GRAV_COMP_PARAM_MASS;
    gravCompObj->lengthOriginToCOM = GRAV_COMP_PARAM_LINK_LENGTH;
}

float Deg2Rad(float deg)
{
    return (deg * M_PI / 180.0);
}

float Vel2RadPerSec(float vel)
{
    return (vel * M_PI / 180.0);
}

void GravityCompensator(GravCompObj_t* gravCompObj, float deg)
{
    float rad = Deg2Rad(deg); // 각도를 라디안으로 변환

    // 중력항 Torque 계산. sin(rad) 자체가 방향(부호)를 결정합니다.
    gravCompObj->FTC.resTorque = gravCompObj->mass * GRAV_COMP_CONST_GRAV_ACC * gravCompObj->lengthOriginToCOM * sin(rad);

    // 모터의 감소 비율을 고려하여 필요한 전류 입력 계산
    // 여기서는 sin(rad)의 부호를 직접 사용하여 중력을 상쇄하는 방향으로 토크가 적용됩니다.
    gravCompObj->FTC.resCurrent = (gravCompObj->FTC.resTorque) / (GRAV_COMP_TORQUE_CONSTANT * GRAV_COMP_GEAR_RATIO);
}


/**
 *------------------------------------------------------------
 *                      STATIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions intended for internal use within this module.
 */




#endif /* SUIT_MINICM_ENABLED */
