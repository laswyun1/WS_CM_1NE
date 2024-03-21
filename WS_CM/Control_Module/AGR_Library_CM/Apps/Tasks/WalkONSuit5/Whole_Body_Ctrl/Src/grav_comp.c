#include "grav_comp.h"

#ifdef WALKON5_CM_ENABLED

/**
 *-----------------------------------------------------------
 *              TYPE DEFINITIONS AND ENUMERATIONS
 *-----------------------------------------------------------
 * @brief Enumerated types and structures central to this module.
 */




/**
 *------------------------------------------------------------
 *                         VARIABLES
 *------------------------------------------------------------
 * @brief Variables accessible throughout the application.
 */

GravComp_Params param;


/**
 *------------------------------------------------------------
 *                 STATIC FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Static Function prototypes for this module.
 */

static GravComp_ForceTorque Get2DGrav(float glob_pos, float mass_force_coef, 
                        GravComp_BodyParams* p, GravComp_ForceTorque* propagated_ft,
                        uint8_t is_forward);
static void HangedLegGrav(GravComp_ForceTorque* hip_ft, GravComp_ForceTorque* knee_ft,
    float torso_roll_rad, float torso_pitch_rad, 
    float pelv_roll_rad,
    float hip_rot_rad, float knee_rot_rad,
    GravComp_BodyParams* thigh, GravComp_BodyParams* shank);
static void StanceLegGrav(GravComp_ForceTorque* hip_ft, GravComp_ForceTorque* knee_ft,
    float torso_roll_rad, float torso_pitch_rad, 
    float pelv_roll_rad,
    float hip_rot_rad,
    GravComp_BodyParams* torso, GravComp_BodyParams* thigh,
    float other_pelv_roll_rad, GravComp_ForceTorque* other_leg_ft);


/**
 *------------------------------------------------------------
 *                      PUBLIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions that interface with this module.
 */


void GravComp_Init()
{
    GravComp_Params p;
    memset(&p, 0, sizeof(p));

    p.torso.m  = GRAV_COMP_DEFAULT_PARAM_TORSO_M;
    p.torso.l  = GRAV_COMP_DEFAULT_PARAM_TORSO_L;
    p.torso.lc = GRAV_COMP_DEFAULT_PARAM_TORSO_LC;
    p.torso.th = GRAV_COMP_DEFAULT_PARAM_TORSO_TH;

    p.thigh_lt.m  = GRAV_COMP_DEFAULT_PARAM_THIGH_M;
    p.thigh_lt.l  = GRAV_COMP_DEFAULT_PARAM_THIGH_L;
    p.thigh_lt.lc = GRAV_COMP_DEFAULT_PARAM_THIGH_LC;
    p.thigh_lt.th = GRAV_COMP_DEFAULT_PARAM_THIGH_TH;

    p.thigh_rt.m  = GRAV_COMP_DEFAULT_PARAM_THIGH_M;
    p.thigh_rt.l  = GRAV_COMP_DEFAULT_PARAM_THIGH_L;
    p.thigh_rt.lc = GRAV_COMP_DEFAULT_PARAM_THIGH_LC;
    p.thigh_rt.th = GRAV_COMP_DEFAULT_PARAM_THIGH_TH;

    p.shank_lt.m  = GRAV_COMP_DEFAULT_PARAM_SHANK_M;
    p.shank_lt.l  = GRAV_COMP_DEFAULT_PARAM_SHANK_L;
    p.shank_lt.lc = GRAV_COMP_DEFAULT_PARAM_SHANK_LC;
    p.shank_lt.th = GRAV_COMP_DEFAULT_PARAM_SHANK_TH;

    p.shank_rt.m  = GRAV_COMP_DEFAULT_PARAM_SHANK_M;
    p.shank_rt.l  = GRAV_COMP_DEFAULT_PARAM_SHANK_L;
    p.shank_rt.lc = GRAV_COMP_DEFAULT_PARAM_SHANK_LC;
    p.shank_rt.th = GRAV_COMP_DEFAULT_PARAM_SHANK_TH;

    memset(&param, 0, sizeof(param));
    GravComp_SetParams(&p);
}

void GravComp_SetParams(GravComp_Params* param_in)
{
    memcpy(&param, param_in, sizeof(param));
}

GravComp_Params* GravComp_GetParams()
{
    return &param;
}

void GravCmopTest(float* res,
    float torso_roll_deg, float torso_pitch_deg,
    float* pos_deg)
{
    float tr = torso_roll_deg  * M_PI / 180;
    float tp = torso_pitch_deg * M_PI / 180;
    float hl = pos_deg[0]      * M_PI / 180 * GRAV_COMP_DIR_FIX_LH;
    float kl = pos_deg[1]      * M_PI / 180 * GRAV_COMP_DIR_FIX_LK;
    float hr = pos_deg[2]      * M_PI / 180 * GRAV_COMP_DIR_FIX_RH;
    float kr = pos_deg[3]      * M_PI / 180 * GRAV_COMP_DIR_FIX_RK;

    GravComp_ForceTorque grav_lh, grav_lk, grav_rh, grav_rk;
    HangedLegGrav(&grav_lh, &grav_lk, tr, tp, 0, hl, kl, &param.thigh_lt, &param.shank_lt);
    HangedLegGrav(&grav_rh, &grav_rk, tr, tp, 0, hr, kr, &param.thigh_rt, &param.shank_rt);

    res[0] = -grav_lh.t * GRAV_COMP_DIR_FIX_LH;
    res[1] = -grav_lk.t * GRAV_COMP_DIR_FIX_LK;
    res[2] = -grav_rh.t * GRAV_COMP_DIR_FIX_RH;
    res[3] = -grav_rk.t * GRAV_COMP_DIR_FIX_RK;
}

void GravCmopTest2(float* res,
    float torso_roll_deg, float torso_pitch_deg,
    float* pos_deg,
	uint8_t is_left)
{
    float tr = torso_roll_deg  * M_PI / 180;
    float tp = torso_pitch_deg * M_PI / 180;
    float hl = pos_deg[0]      * M_PI / 180 * GRAV_COMP_DIR_FIX_LH;
    float kl = pos_deg[1]      * M_PI / 180 * GRAV_COMP_DIR_FIX_LK;
    float hr = pos_deg[2]      * M_PI / 180 * GRAV_COMP_DIR_FIX_RH;
    float kr = pos_deg[3]      * M_PI / 180 * GRAV_COMP_DIR_FIX_RK;

    GravComp_ForceTorque grav_lh, grav_lk, grav_rh, grav_rk;
    if (is_left) {
        HangedLegGrav(&grav_rh, &grav_rk, tr, tp, 0, hr, kr, &param.thigh_rt, &param.shank_rt);
        StanceLegGrav(&grav_lh, &grav_lk, tr, tp, 0, hl, &param.torso, &param.thigh_lt, 0, &grav_rh);
    } else {
        HangedLegGrav(&grav_lh, &grav_lk, tr, tp, 0, hl, kl, &param.thigh_lt, &param.shank_lt);
        StanceLegGrav(&grav_rh, &grav_rk, tr, tp, 0, hr, &param.torso, &param.thigh_rt, 0, &grav_lh);
    }

    res[0] = -grav_lh.t * GRAV_COMP_DIR_FIX_LH;
    res[1] = -grav_lk.t * GRAV_COMP_DIR_FIX_LK;
    res[2] = -grav_rh.t * GRAV_COMP_DIR_FIX_RH;
    res[3] = -grav_rk.t * GRAV_COMP_DIR_FIX_RK;
}

void GravCmopTest3(float* res,
    float torso_roll_deg, float torso_pitch_deg,
    float* pos_deg,
	float left_ratio)
{
    float tr = torso_roll_deg  * M_PI / 180;
    float tp = torso_pitch_deg * M_PI / 180;
    float hl = pos_deg[0]      * M_PI / 180 * GRAV_COMP_DIR_FIX_LH;
    float hr = pos_deg[2]      * M_PI / 180 * GRAV_COMP_DIR_FIX_RH;

    GravComp_ForceTorque grav_lh, grav_lk, grav_rh, grav_rk;

    GravComp_BodyParams ltbody = param.torso, rtbody = param.torso;
    ltbody.m *= left_ratio;
    rtbody.m *= (1 - left_ratio);

    StanceLegGrav(&grav_lh, &grav_lk, tr, tp, 0, hl, &ltbody, &param.thigh_lt, 0, NULL);
    StanceLegGrav(&grav_rh, &grav_rk, tr, tp, 0, hr, &rtbody, &param.thigh_rt, 0, NULL);

    res[0] = -grav_lh.t * GRAV_COMP_DIR_FIX_LH;
    res[1] = -grav_lk.t * GRAV_COMP_DIR_FIX_LK;
    res[2] = -grav_rh.t * GRAV_COMP_DIR_FIX_RH;
    res[3] = -grav_rk.t * GRAV_COMP_DIR_FIX_RK;
}

void GravComp_BasicGravComp(float* res,
    float* torso_rpy, float* pelvic_abd,
    float* joint_pos,
    uint8_t* ground_contact, float* ground_reaction,
    uint8_t option)
{
    /* Position Parsing */
    // Passive joints
    float tr = torso_rpy[0]  * M_PI / 180; // torso roll
    float tp = torso_rpy[1]  * M_PI / 180; // torso pitch
    float pl = pelvic_abd[0] * M_PI / 180; // pelvic ab/aduction lt.
    float pr = pelvic_abd[1] * M_PI / 180; // pelvic ab/aduction rt.
    // Active joints
    float lh = joint_pos[0]  * M_PI / 180 * GRAV_COMP_DIR_FIX_LH;
    float lk = joint_pos[1]  * M_PI / 180 * GRAV_COMP_DIR_FIX_LK;
    float rh = joint_pos[2]  * M_PI / 180 * GRAV_COMP_DIR_FIX_RH;
    float rk = joint_pos[3]  * M_PI / 180 * GRAV_COMP_DIR_FIX_RK;

    /* Stance Type */
    uint8_t stance_state = GRAV_COMP_STANCE_NONE;
    if ((ground_contact[0] != 0) && (ground_contact[1] != 0)) {
        stance_state = GRAV_COMP_STANCE_DOUBLE;
    } else if (ground_contact[0] != 0) {
        stance_state = GRAV_COMP_STANCE_SINGLE_LT;
    } else if (ground_contact[1] != 0) {
        stance_state = GRAV_COMP_STANCE_SINGLE_RT;
    }

    /* Weight Distribution */
    float weight_ratio[2] = {0.0, 0.0};
    float wgt_sum = ground_reaction[0] + ground_reaction[1];
    float reliable_wgt = 0.1; // [kg]

    switch (stance_state) {
    case GRAV_COMP_STANCE_DOUBLE:
        if (wgt_sum > 0) {
            for (int i = 0; i < 2; ++i) {
                float tmp = 0;
                if (wgt_sum < reliable_wgt) { // Linear reliability
                    tmp = ground_reaction[i] / reliable_wgt;
                } else {
                    tmp = ground_reaction[i] / wgt_sum;
                }
                // Limit
                weight_ratio[i] = (tmp > 1.0) ? 1.0 : tmp;
                weight_ratio[i] = (tmp < 0.0) ? 0.0 : tmp;
            }
        }
        break;

    case GRAV_COMP_STANCE_SINGLE_LT:
        weight_ratio[0] = 1.0;
        break;

    case GRAV_COMP_STANCE_SINGLE_RT:
        weight_ratio[1] = 1.0;
        break;

    default: // No stance
        break;
    }
    
    /* Gravity Compensation */    
    GravComp_ForceTorque ft_res_lh = {0.0, 0.0};
    GravComp_ForceTorque ft_res_lk = {0.0, 0.0};
    GravComp_ForceTorque ft_res_rh = {0.0, 0.0};
    GravComp_ForceTorque ft_res_rk = {0.0, 0.0};

    GravComp_BodyParams tmp_torso[2];
    for (int i = 0; i < 2; ++i) {
        memcpy(&tmp_torso[i], &param.torso, sizeof(param.torso));
        tmp_torso[i].m *= weight_ratio[i];
    }

    switch (stance_state) {
    case GRAV_COMP_STANCE_DOUBLE:
        if ((option & GRAV_COMP_BASIC_OPTION_STANCE_ENB) != 0) {
            StanceLegGrav(&ft_res_lh, &ft_res_lk, tr, tp, pl, lh, &tmp_torso[0], &param.thigh_lt, 0, NULL);
            StanceLegGrav(&ft_res_rh, &ft_res_rk, tr, tp, pr, rh, &tmp_torso[1], &param.thigh_rt, 0, NULL);
        }
        break;

    case GRAV_COMP_STANCE_SINGLE_LT:
        if ((option & GRAV_COMP_BASIC_OPTION_HANGED_ENB) != 0) {
            HangedLegGrav(&ft_res_rh, &ft_res_rk, tr, tp, pr, rh, rk, &param.thigh_rt, &param.shank_rt);
        }
        if ((option & GRAV_COMP_BASIC_OPTION_STANCE_ENB) != 0) {
            StanceLegGrav(&ft_res_lh, &ft_res_lk, tr, tp, pl, lh, &tmp_torso[0], &param.thigh_lt, pr, &ft_res_rh);
        }
        break;

    case GRAV_COMP_STANCE_SINGLE_RT:
        if ((option & GRAV_COMP_BASIC_OPTION_HANGED_ENB) != 0) {
            HangedLegGrav(&ft_res_lh, &ft_res_lk, tr, tp, pl, lh, lk, &param.thigh_lt, &param.shank_lt);
        }
        if ((option & GRAV_COMP_BASIC_OPTION_STANCE_ENB) != 0) {
            StanceLegGrav(&ft_res_rh, &ft_res_rk, tr, tp, pr, rh, &tmp_torso[1], &param.thigh_rt, pl, &ft_res_lh);
        }
        break;

    case GRAV_COMP_STANCE_NONE:
        if ((option & GRAV_COMP_BASIC_OPTION_HANGED_ENB) != 0) {
            HangedLegGrav(&ft_res_lh, &ft_res_lk, tr, tp, pl, lh, lk, &param.thigh_lt, &param.shank_lt);
            HangedLegGrav(&ft_res_rh, &ft_res_rk, tr, tp, pr, rh, rk, &param.thigh_rt, &param.shank_rt);
        }
        break;
    }

    res[0] = -ft_res_lh.t * GRAV_COMP_DIR_FIX_LH;
    res[1] = -ft_res_lk.t * GRAV_COMP_DIR_FIX_LK;
    res[2] = -ft_res_rh.t * GRAV_COMP_DIR_FIX_RH;
    res[3] = -ft_res_rk.t * GRAV_COMP_DIR_FIX_RK;
}


/**
 *------------------------------------------------------------
 *                      STATIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions intended for internal use within this module.
 */


static GravComp_ForceTorque Get2DGrav(float glob_pos, float mass_force_coef, 
                        GravComp_BodyParams* p, GravComp_ForceTorque* propagated_ft,
                        uint8_t is_forward)
{
    float com_len = (is_forward == GRAV_COMP_2D_GRAV_FORWARD) ? p->lc : (p->l - p->lc);
    float com_pos = (is_forward == GRAV_COMP_2D_GRAV_FORWARD) ?     0 :       0 + M_PI;
    float link_pos = glob_pos + com_pos;

    float body_grav_force  = -p->m * mass_force_coef; // -z dir
    float body_grav_dist   = -com_len * sin(link_pos + p->th); // rot +y, 0deg == -z vector. +rot -> -x dir
    float body_grav_torque = -(body_grav_dist * body_grav_force); // note the direction
    
    float propagated_grav_force  = 0;
    float propagated_grav_dist   = 0;
    float propagated_grav_torque = 0;

    if (propagated_ft != NULL) {
        propagated_grav_force  = propagated_ft->f;
        propagated_grav_dist   = -p->l * sin(link_pos);
        propagated_grav_torque = -(propagated_grav_dist * propagated_grav_force)
                                 + propagated_ft->t;
    }

    GravComp_ForceTorque res;
    res.f = body_grav_force  + propagated_grav_force;
    res.t = body_grav_torque + propagated_grav_torque;

    return res;
}

static void HangedLegGrav(GravComp_ForceTorque* hip_ft, GravComp_ForceTorque* knee_ft,
    float torso_roll_rad, float torso_pitch_rad, 
    float pelv_roll_rad,
    float hip_rot_rad, float knee_rot_rad,
    GravComp_BodyParams* thigh, GravComp_BodyParams* shank)
{
    // Global Rotation Angles
    float leg_plane_roll = torso_roll_rad + pelv_roll_rad;
    float thigh_gra = torso_pitch_rad + hip_rot_rad;
    float shank_gra = thigh_gra + knee_rot_rad;

    // Mass-Force Coefficient
    float mfc = GRAV_COMP_CONST_GRAV_ACC * cos(leg_plane_roll); // Gravity Projected on Leg Plane

    // Get Gravity
    GravComp_ForceTorque hip_ft_res, knee_ft_res;
    knee_ft_res = Get2DGrav(shank_gra, mfc, shank, NULL,         GRAV_COMP_2D_GRAV_FORWARD);
    hip_ft_res  = Get2DGrav(thigh_gra, mfc, thigh, &knee_ft_res, GRAV_COMP_2D_GRAV_FORWARD);

    memcpy(hip_ft,  &hip_ft_res,  sizeof(hip_ft_res));
    memcpy(knee_ft, &knee_ft_res, sizeof(knee_ft_res));
}

static void StanceLegGrav(GravComp_ForceTorque* hip_ft, GravComp_ForceTorque* knee_ft,
    float torso_roll_rad, float torso_pitch_rad, 
    float pelv_roll_rad,
    float hip_rot_rad,
    GravComp_BodyParams* torso, GravComp_BodyParams* thigh,
    float other_pelv_roll_rad, GravComp_ForceTorque* other_leg_ft)
{
    // Global Rotation Angles
    float leg_plane_gra       = torso_roll_rad + pelv_roll_rad;
    float other_leg_plane_gra = torso_roll_rad + other_pelv_roll_rad;
    float torso_gra = torso_pitch_rad;
    float thigh_gra = torso_pitch_rad + hip_rot_rad;

    // Mass-Force Coefficient
    float mfc       = GRAV_COMP_CONST_GRAV_ACC * cos(leg_plane_gra); // Gravity Projected on Leg Plane
    float other_mfc = GRAV_COMP_CONST_GRAV_ACC * cos(other_leg_plane_gra);

    // Get Gravity
    GravComp_ForceTorque hip_ft_res, knee_ft_res;

    GravComp_ForceTorque other_leg_ft_proj = {0, 0}; // Projected other-leg Force-troque
    if (other_leg_ft != NULL) {
    	float tolerance = 0.01;
        float other2torso_mfc_ratio = (fabs(other_mfc) > tolerance) ? (mfc/other_mfc) : 0;
        other_leg_ft_proj.f = other_leg_ft->f * other2torso_mfc_ratio;
        other_leg_ft_proj.t = other_leg_ft->t * other2torso_mfc_ratio;
    }

    hip_ft_res  = Get2DGrav(torso_gra, mfc, torso, &other_leg_ft_proj, GRAV_COMP_2D_GRAV_FORWARD);
    knee_ft_res = Get2DGrav(thigh_gra, mfc, thigh, &hip_ft_res,        GRAV_COMP_2D_GRAV_BACKWARD);

    // Torque by reaction
    hip_ft_res.t  *= -1;
    knee_ft_res.t *= -1;

    memcpy(hip_ft,  &hip_ft_res,  sizeof(hip_ft_res));
	memcpy(knee_ft, &knee_ft_res, sizeof(knee_ft_res));
}


#endif /* WALKON5_CM_ENABLED */