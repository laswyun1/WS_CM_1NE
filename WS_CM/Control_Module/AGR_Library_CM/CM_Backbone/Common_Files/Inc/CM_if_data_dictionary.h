#ifndef CM_BACKBONE_COMMON_FILES_CM_IF_DATA_DICTIONARY_H_
#define CM_BACKBONE_COMMON_FILES_CM_IF_DATA_DICTIONARY_H_

/**
 *-----------------------------------------------------------
 *              MACROS AND PREPROCESSOR DIRECTIVES
 *-----------------------------------------------------------
 * @brief Directives and macros for readability and efficiency.
 */

#define CMIF_BUTTON_OFFSET         0x000
#define CMIF_ENCODER_OFFSET        0x080
#define CMIF_IMU_OFFSET            0x100
#define CMIF_EMG_OFFSET            0x180
#define CMIF_PMMG_OFFSET           0x200
#define CMIF_EXTRA_SENSORS_OFFSET  0x280
#define CTRL_VARIABLE_GAP          0x020
#define CMIF_CTRL_VARIABLE_OFFSET  0x300
#define CMIF_CM_GENERATION_OFFSET  0x500
#define CMIF_AM_SEND_OFFSET        0x600


/**
 *------------------------------------------------------------
 *                     TYPE DECLARATIONS
 *------------------------------------------------------------
 * @brief Custom data types and structures for the module.
 */

/*********************** PHYSICAL BUTTON (0x000 ~ 0x07F) ***********************/
typedef enum _CMIFID_Button {
	CMIF_ID_BUTTON_POWER = CMIF_BUTTON_OFFSET,
	CMIF_ID_BUTTON_EMCY,
	CMIF_ID_BUTTON_RESET,
	CMIF_ID_BUTTON_BT_PARING,
	CMIF_ID_BUTTON_LEFT_GUIDE,
	CMIF_ID_BUTTON_RIGHT_GUIDE,
	CMIF_ID_BUTTON_ASSISTANCE_INC,
	CMIF_ID_BUTTON_ASSISTANCE_DEC,
	CMIF_ID_BUTTON_CRUTCH1,
	CMIF_ID_BUTTON_CRUTCH2,
	CMIF_ID_BUTTON_CRUTCH3,
	CMIF_ID_BUTTON_CRUTCH4,
	CMIF_ID_BUTTON_R_FOOT_DOCKING,
	CMIF_ID_BUTTON_L_FOOT_DOCKING,
	CMIF_ID_BUTTON_R_SHANK_DOCKING,
	CMIF_ID_BUTTON_L_SHANK_DOCKING,
	CMIF_ID_BUTTON_R_THIGH_DOCKING,
	CMIF_ID_BUTTON_L_THIGH_DOCKING,
	CMIF_ID_BUTTON_R_PELVIC_DOCKING,
	CMIF_ID_BUTTON_L_PELVIC_DOCKING,
	CMIF_ID_BUTTON_TRUNK_DOCKING,
	CMIF_ID_BUTTON_MD_EXTRA1,
	CMIF_ID_BUTTON_MD_EXTRA2,
	CMIF_ID_BUTTON_MD_EXTRA3,
	CMIF_ID_BUTTON_MD_EXTRA4,
	CMIF_ID_BUTTON_MD_EXTRA5,
	CMIF_ID_BUTTON_MD_EXTRA6,
	CMIF_ID_BUTTON_MD_EXTRA7,
	CMIF_ID_BUTTON_MD_EXTRA8,
	CMIF_ID_BUTTON_MD_EXTRA9,
	CMIF_ID_BUTTON_MD_EXTRA10,
	CMIF_ID_BUTTON_MD_EXTRA11,
	CMIF_ID_BUTTON_MD_EXTRA12,
	CMIF_ID_BUTTON_MD_EXTRA13,
	CMIF_ID_BUTTON_MD_EXTRA14,
	CMIF_ID_BUTTON_MD_EXTRA15,
	CMIF_ID_BUTTON_MD_EXTRA16,

	CMIF_ID_BUTTON_NUM
} CMIFID_Button;

/*********************** ENCODER (0x080 ~ 0x0FF) ***********************/
typedef enum _CMIFID_Encoder {
	CMIF_ID_ENCODER_JOINT_ANGLE_RH_ABADD = CMIF_ENCODER_OFFSET,
	CMIF_ID_ENCODER_JOINT_VEL_RH_ABADD,
	CMIF_ID_ENCODER_JOINT_ANGLE_LH_ABADD,
	CMIF_ID_ENCODER_JOINT_VEL_LH_ABADD,
	CMIF_ID_ENCODER_JOINT_ANGLE_RH_INTEXT_ROT,
	CMIF_ID_ENCODER_JOINT_VEL_RH_INTEXT_ROT,
	CMIF_ID_ENCODER_JOINT_ANGLE_LH_INTEXT_ROT,
	CMIF_ID_ENCODER_JOINT_VEL_LH_INTEXT_ROT,
	CMIF_ID_ENCODER_JOINT_ANGLE_RH_FLEXEXT,
	CMIF_ID_ENCODER_JOINT_VEL_RH_FLEXEXT,
	CMIF_ID_ENCODER_JOINT_ANGLE_LH_FLEXEXT,
	CMIF_ID_ENCODER_JOINT_VEL_LH_FLEXEXT,
	CMIF_ID_ENCODER_JOINT_ANGLE_RK_FLEXEXT,
	CMIF_ID_ENCODER_JOINT_VEL_RK_FLEXEXT,
	CMIF_ID_ENCODER_JOINT_ANGLE_LK_FLEXEXT,
	CMIF_ID_ENCODER_JOINT_VEL_LK_FLEXEXT,
	CMIF_ID_ENCODER_JOINT_ANGLE_RA_INVEV,
	CMIF_ID_ENCODER_JOINT_VEL_RA_INVEV,
	CMIF_ID_ENCODER_JOINT_ANGLE_LA_INVEV,
	CMIF_ID_ENCODER_JOINT_VEL_LA_INVEV,
	CMIF_ID_ENCODER_JOINT_ANGLE_RA_DORSIPLANTAR,
	CMIF_ID_ENCODER_JOINT_VEL_RA_DORSIPLANTAR,
	CMIF_ID_ENCODER_JOINT_ANGLE_LA_DORSIPLANTAR,
	CMIF_ID_ENCODER_JOINT_VEL_LA_DORSIPLANTAR,

	CMIF_ID_ENCODER_ACTUATOR1_ANGLE,
	CMIF_ID_ENCODER_ACTUATOR1_VEL,
	CMIF_ID_ENCODER_ACTUATOR2_ANGLE,
	CMIF_ID_ENCODER_ACTUATOR2_VEL,
	CMIF_ID_ENCODER_ACTUATOR3_ANGLE,
	CMIF_ID_ENCODER_ACTUATOR3_VEL,
	CMIF_ID_ENCODER_ACTUATOR4_ANGLE,
	CMIF_ID_ENCODER_ACTUATOR4_VEL,
	CMIF_ID_ENCODER_ACTUATOR5_ANGLE,
	CMIF_ID_ENCODER_ACTUATOR5_VEL,
	CMIF_ID_ENCODER_ACTUATOR6_ANGLE,
	CMIF_ID_ENCODER_ACTUATOR6_VEL,
	CMIF_ID_ENCODER_ACTUATOR7_ANGLE,
	CMIF_ID_ENCODER_ACTUATOR7_VEL,
	CMIF_ID_ENCODER_ACTUATOR8_ANGLE,
	CMIF_ID_ENCODER_ACTUATOR8_VEL,
	CMIF_ID_ENCODER_ACTUATOR9_ANGLE,
	CMIF_ID_ENCODER_ACTUATOR9_VEL,
	CMIF_ID_ENCODER_ACTUATOR10_ANGLE,
	CMIF_ID_ENCODER_ACTUATOR10_VEL,
	CMIF_ID_ENCODER_ACTUATOR11_ANGLE,
	CMIF_ID_ENCODER_ACTUATOR11_VEL,
	CMIF_ID_ENCODER_ACTUATOR12_ANGLE,
	CMIF_ID_ENCODER_ACTUATOR12_VEL,
	CMIF_ID_ENCODER_ACTUATOR13_ANGLE,
	CMIF_ID_ENCODER_ACTUATOR13_VEL,
	CMIF_ID_ENCODER_ACTUATOR14_ANGLE,
	CMIF_ID_ENCODER_ACTUATOR14_VEL,
	CMIF_ID_ENCODER_ACTUATOR15_ANGLE,
	CMIF_ID_ENCODER_ACTUATOR15_VEL,
	CMIF_ID_ENCODER_ACTUATOR16_ANGLE,
	CMIF_ID_ENCODER_ACTUATOR16_VEL,

	CMIF_ID_ENCODER_NUM
} CMIFID_Encoder;

/*********************** IMU (0x100 ~ 0x17F) ***********************/
typedef enum _CMIFID_IMU {
	CMIF_ID_IMU_QUAT_TRUNK = CMIF_IMU_OFFSET,
	CMIF_ID_IMU_QUAT_R_PELVIS,
	CMIF_ID_IMU_QUAT_L_PELVIS,
	CMIF_ID_IMU_QUAT_R_THIGH,
	CMIF_ID_IMU_QUAT_L_THIGH,
	CMIF_ID_IMU_QUAT_R_SHANK,
	CMIF_ID_IMU_QUAT_L_SHANK,
	CMIF_ID_IMU_QUAT_R_FOOT,
	CMIF_ID_IMU_QUAT_L_FOOT,
	CMIF_ID_IMU_QUAT_R_CRUTCH,
	CMIF_ID_IMU_QUAT_L_CRUTCH,

	CMIF_ID_IMU_EULER_TRUNK,
	CMIF_ID_IMU_EULER_R_PELVIS,
	CMIF_ID_IMU_EULER_L_PELVIS,
	CMIF_ID_IMU_EULER_R_THIGH,
	CMIF_ID_IMU_EULER_L_THIGH,
	CMIF_ID_IMU_EULER_R_SHANK,
	CMIF_ID_IMU_EULER_L_SHANK,
	CMIF_ID_IMU_EULER_R_FOOT,
	CMIF_ID_IMU_EULER_L_FOOT,
	CMIF_ID_IMU_EULER_R_CRUTCH,
	CMIF_ID_IMU_EULER_L_CRUTCH,

	CMIF_ID_IMU_NUM
} CMIFID_IMU;

/*********************** EMG (0x180 ~ 0x1FF) ***********************/
typedef enum _CMIFID_EMG {
	CMIF_ID_EMG_FR = CMIF_EMG_OFFSET,
	CMIF_ID_EMG_OC,
	CMIF_ID_EMG_CR,
	CMIF_ID_EMG_BU,
	CMIF_ID_EMG_FA,
	CMIF_ID_EMG_AU,
	CMIF_ID_EMG_NA,
	CMIF_ID_EMG_CE,
	CMIF_ID_EMG_OR,
	CMIF_ID_EMG_ME,
	CMIF_ID_EMG_TH,
	CMIF_ID_EMG_AX,
	CMIF_ID_EMG_MA,
	CMIF_ID_EMG_BR,
	CMIF_ID_EMG_ABR,
	CMIF_ID_EMG_CA,
	CMIF_ID_EMG_PO,
	CMIF_ID_EMG_PA,
	CMIF_ID_EMG_DI,
	CMIF_ID_EMG_AB,
	CMIF_ID_EMG_UM,
	CMIF_ID_EMG_HI,
	CMIF_ID_EMG_PE,
	CMIF_ID_EMG_IN,
	CMIF_ID_EMG_PU,
	CMIF_ID_EMG_FE,
	CMIF_ID_EMG_PAT,
	CMIF_ID_EMG_CRU,
	CMIF_ID_EMG_TA,
	CMIF_ID_EMG_TO,
	CMIF_ID_EMG_HA,
	CMIF_ID_EMG_BHE,
	CMIF_ID_EMG_SH,
	CMIF_ID_EMG_BCE,
	CMIF_ID_EMG_DO,
	CMIF_ID_EMG_BBR,
	CMIF_ID_EMG_OL,
	CMIF_ID_EMG_LU,
	CMIF_ID_EMG_SA,
	CMIF_ID_EMG_BABR,
	CMIF_ID_EMG_GL,
	CMIF_ID_EMG_BFE,
	CMIF_ID_EMG_POP,
	CMIF_ID_EMG_SU,

	CMIF_ID_EMG_NUM
} CMIFID_EMG;

/*********************** PMMG (0x200 ~ 0x27F) ***********************/
typedef enum _CMIFID_PMMG {
	CMIF_ID_PMMG_FR = CMIF_PMMG_OFFSET,
	CMIF_ID_PMMG_OC,
	CMIF_ID_PMMG_CR,
	CMIF_ID_PMMG_BU,
	CMIF_ID_PMMG_FA,
	CMIF_ID_PMMG_AU,
	CMIF_ID_PMMG_NA,
	CMIF_ID_PMMG_CE,
	CMIF_ID_PMMG_OR,
	CMIF_ID_PMMG_ME,
	CMIF_ID_PMMG_TH,
	CMIF_ID_PMMG_AX,
	CMIF_ID_PMMG_MA,
	CMIF_ID_PMMG_BR,
	CMIF_ID_PMMG_ABR,
	CMIF_ID_PMMG_CA,
	CMIF_ID_PMMG_PO,
	CMIF_ID_PMMG_PA,
	CMIF_ID_PMMG_DI,
	CMIF_ID_PMMG_AB,
	CMIF_ID_PMMG_UM,
	CMIF_ID_PMMG_HI,
	CMIF_ID_PMMG_PE,
	CMIF_ID_PMMG_IN,
	CMIF_ID_PMMG_PU,
	CMIF_ID_PMMG_FE,
	CMIF_ID_PMMG_PAT,
	CMIF_ID_PMMG_CRU,
	CMIF_ID_PMMG_TA,
	CMIF_ID_PMMG_TO,
	CMIF_ID_PMMG_HA,
	CMIF_ID_PMMG_BHE,
	CMIF_ID_PMMG_SH,
	CMIF_ID_PMMG_BCE,
	CMIF_ID_PMMG_DO,
	CMIF_ID_PMMG_BBR,
	CMIF_ID_PMMG_OL,
	CMIF_ID_PMMG_LU,
	CMIF_ID_PMMG_SA,
	CMIF_ID_PMMG_BABR,
	CMIF_ID_PMMG_GL,
	CMIF_ID_PMMG_BFE,
	CMIF_ID_PMMG_POP,
	CMIF_ID_PMMG_SU,

	CMIF_ID_PMMG_NUM
} CMIFID_PMMG;

/*********************** EXTRA SENSORS (0x280 ~ 0xF7F) ***********************/
typedef enum _CMIFID_EXTRA_SENSORS {
	CMIF_ID_EXTRA_SENSORS_R_GRF_COP = CMIF_EXTRA_SENSORS_OFFSET,
	CMIF_ID_EXTRA_SENSORS_R_GRF_MAG,
	CMIF_ID_EXTRA_SENSORS_L_GRF_COP,
	CMIF_ID_EXTRA_SENSORS_L_GRF_MAG,
	CMIF_ID_EXTRA_SENSORS_R_CRUTCH_GRF_MAG,
	CMIF_ID_EXTRA_SENSORS_L_CRUTCH_GRF_MAG,
	CMIF_ID_EXTRA_SENSORS_R_CRUTCH_FSR_PRESSURE,
	CMIF_ID_EXTRA_SENSORS_L_CRUTCH_FSR_PRESSURE,
	CMIF_ID_EXTRA_SENSORS_PELVIS_WIDTH,
	CMIF_ID_EXTRA_SENSORS_THIGH_LENGTH,
	CMIF_ID_EXTRA_SENSORS_SHANK_LENGTH,
	CMIF_ID_EXTRA_SENSORS_PM_BAT_VOLTAGE,
	CMIF_ID_EXTRA_SENSORS_PM_BAT_CURRENT,
	CMIF_ID_EXTRA_SENSORS_PM_BAT_SOC,
	CMIF_ID_EXTRA_SENSORS_ACTUATOR1_TORQUE,
	CMIF_ID_EXTRA_SENSORS_ACTUATOR2_TORQUE,
	CMIF_ID_EXTRA_SENSORS_ACTUATOR3_TORQUE,
	CMIF_ID_EXTRA_SENSORS_ACTUATOR4_TORQUE,
	CMIF_ID_EXTRA_SENSORS_ACTUATOR5_TORQUE,
	CMIF_ID_EXTRA_SENSORS_ACTUATOR6_TORQUE,
	CMIF_ID_EXTRA_SENSORS_ACTUATOR7_TORQUE,
	CMIF_ID_EXTRA_SENSORS_ACTUATOR8_TORQUE,
	CMIF_ID_EXTRA_SENSORS_ACTUATOR9_TORQUE,
	CMIF_ID_EXTRA_SENSORS_ACTUATOR10_TORQUE,
	CMIF_ID_EXTRA_SENSORS_ACTUATOR11_TORQUE,
	CMIF_ID_EXTRA_SENSORS_ACTUATOR12_TORQUE,
	CMIF_ID_EXTRA_SENSORS_ACTUATOR13_TORQUE,
	CMIF_ID_EXTRA_SENSORS_ACTUATOR14_TORQUE,
	CMIF_ID_EXTRA_SENSORS_ACTUATOR15_TORQUE,
	CMIF_ID_EXTRA_SENSORS_ACTUATOR16_TORQUE,

	CMIF_ID_EXTRA_SENSORS_NUM
} CMIFID_EXTRA_SENSORS;

/*********************** CONTROL VARIABLES (0x300 ~ 0x4FF) ***********************/
typedef enum _CMIFID_CTRL_VARIABLES {
	CMIF_ID_CTRL_VAR_MD1_STATE = CMIF_CTRL_VARIABLE_OFFSET,
	CMIF_ID_CTRL_VAR_MD1_ERROR_CODE,
	CMIF_ID_CTRL_VAR_ACTUATOR1_POSITION_REF,
	CMIF_ID_CTRL_VAR_ACTUATOR1_VELOCITY_REF,
	CMIF_ID_CTRL_VAR_ACTUATOR1_TORQUE_REF,
	CMIF_ID_CTRL_VAR_ACTUATOR1_CURRENT_REF,
	CMIF_ID_CTRL_VAR_ACTUATOR1_CURRENT_ACT,
	CMIF_ID_CTRL_VAR_ACTUATOR1_AUXILIRARY_INPUT,
	CMIF_ID_CTRL_VAR_ACTUATOR1_FC_GAIN,
	CMIF_ID_CTRL_VAR_ACTUATOR1_IC_GAIN,
	CMIF_ID_CTRL_VAR_ACTUATOR1_DOB_GAIN,
	CMIF_ID_CTRL_VAR_ACTUATOR1_FF_GAIN,
	CMIF_ID_CTRL_VAR_ACTUATOR1_IRC_GAIN,
	CMIF_ID_CTRL_VAR_ACTUATOR1_PD_GAIN,
	CMIF_ID_CTRL_VAR_ACTUATOR1_IC_CORRIDOR_HALF_WIDTH,
	CMIF_ID_CTRL_VAR_ACTUATOR1_IC_VIRTUAL_SPRING,
	CMIF_ID_CTRL_VAR_ACTUATOR1_IC_VIRTUAL_DAMPER,
	CMIF_ID_CTRL_VAR_ACTUATOR1_IC_IMP_RATIO,

	CMIF_ID_CTRL_VAR_MD2_STATE = CMIF_CTRL_VARIABLE_OFFSET + CTRL_VARIABLE_GAP,
	CMIF_ID_CTRL_VAR_MD2_ERROR_CODE,
	CMIF_ID_CTRL_VAR_ACTUATOR2_POSITION_REF,
	CMIF_ID_CTRL_VAR_ACTUATOR2_VELOCITY_REF,
	CMIF_ID_CTRL_VAR_ACTUATOR2_TORQUE_REF,
	CMIF_ID_CTRL_VAR_ACTUATOR2_CURRENT_REF,
	CMIF_ID_CTRL_VAR_ACTUATOR2_CURRENT_ACT,
	CMIF_ID_CTRL_VAR_ACTUATOR2_AUXILIRARY_INPUT,
	CMIF_ID_CTRL_VAR_ACTUATOR2_FC_GAIN,
	CMIF_ID_CTRL_VAR_ACTUATOR2_IC_GAIN,
	CMIF_ID_CTRL_VAR_ACTUATOR2_DOB_GAIN,
	CMIF_ID_CTRL_VAR_ACTUATOR2_FF_GAIN,
	CMIF_ID_CTRL_VAR_ACTUATOR2_IRC_GAIN,
	CMIF_ID_CTRL_VAR_ACTUATOR2_PD_GAIN,
	CMIF_ID_CTRL_VAR_ACTUATOR2_IC_CORRIDOR_HALF_WIDTH,
	CMIF_ID_CTRL_VAR_ACTUATOR2_IC_VIRTUAL_SPRING,
	CMIF_ID_CTRL_VAR_ACTUATOR2_IC_VIRTUAL_DAMPER,
	CMIF_ID_CTRL_VAR_ACTUATOR2_IC_IMP_RATIO,

	CMIF_ID_CTRL_VAR_MD3_STATE = CMIF_CTRL_VARIABLE_OFFSET + 2*CTRL_VARIABLE_GAP,
	CMIF_ID_CTRL_VAR_MD3_ERROR_CODE,
	CMIF_ID_CTRL_VAR_ACTUATOR3_POSITION_REF,
	CMIF_ID_CTRL_VAR_ACTUATOR3_VELOCITY_REF,
	CMIF_ID_CTRL_VAR_ACTUATOR3_TORQUE_REF,
	CMIF_ID_CTRL_VAR_ACTUATOR3_CURRENT_REF,
	CMIF_ID_CTRL_VAR_ACTUATOR3_CURRENT_ACT,
	CMIF_ID_CTRL_VAR_ACTUATOR3_AUXILIRARY_INPUT,
	CMIF_ID_CTRL_VAR_ACTUATOR3_FC_GAIN,
	CMIF_ID_CTRL_VAR_ACTUATOR3_IC_GAIN,
	CMIF_ID_CTRL_VAR_ACTUATOR3_DOB_GAIN,
	CMIF_ID_CTRL_VAR_ACTUATOR3_FF_GAIN,
	CMIF_ID_CTRL_VAR_ACTUATOR3_IRC_GAIN,
	CMIF_ID_CTRL_VAR_ACTUATOR3_PD_GAIN,
	CMIF_ID_CTRL_VAR_ACTUATOR3_IC_CORRIDOR_HALF_WIDTH,
	CMIF_ID_CTRL_VAR_ACTUATOR3_IC_VIRTUAL_SPRING,
	CMIF_ID_CTRL_VAR_ACTUATOR3_IC_VIRTUAL_DAMPER,
	CMIF_ID_CTRL_VAR_ACTUATOR3_IC_IMP_RATIO,

	CMIF_ID_CTRL_VAR_MD4_STATE = CMIF_CTRL_VARIABLE_OFFSET + 3*CTRL_VARIABLE_GAP,
	CMIF_ID_CTRL_VAR_MD4_ERROR_CODE,
	CMIF_ID_CTRL_VAR_ACTUATOR4_POSITION_REF,
	CMIF_ID_CTRL_VAR_ACTUATOR4_VELOCITY_REF,
	CMIF_ID_CTRL_VAR_ACTUATOR4_TORQUE_REF,
	CMIF_ID_CTRL_VAR_ACTUATOR4_CURRENT_REF,
	CMIF_ID_CTRL_VAR_ACTUATOR4_CURRENT_ACT,
	CMIF_ID_CTRL_VAR_ACTUATOR4_AUXILIRARY_INPUT,
	CMIF_ID_CTRL_VAR_ACTUATOR4_FC_GAIN,
	CMIF_ID_CTRL_VAR_ACTUATOR4_IC_GAIN,
	CMIF_ID_CTRL_VAR_ACTUATOR4_DOB_GAIN,
	CMIF_ID_CTRL_VAR_ACTUATOR4_FF_GAIN,
	CMIF_ID_CTRL_VAR_ACTUATOR4_IRC_GAIN,
	CMIF_ID_CTRL_VAR_ACTUATOR4_PD_GAIN,
	CMIF_ID_CTRL_VAR_ACTUATOR4_IC_CORRIDOR_HALF_WIDTH,
	CMIF_ID_CTRL_VAR_ACTUATOR4_IC_VIRTUAL_SPRING,
	CMIF_ID_CTRL_VAR_ACTUATOR4_IC_VIRTUAL_DAMPER,
	CMIF_ID_CTRL_VAR_ACTUATOR4_IC_IMP_RATIO,

	CMIF_ID_CTRL_VAR_MD5_STATE = CMIF_CTRL_VARIABLE_OFFSET + 4*CTRL_VARIABLE_GAP,
	CMIF_ID_CTRL_VAR_MD5_ERROR_CODE,
	CMIF_ID_CTRL_VAR_ACTUATOR5_POSITION_REF,
	CMIF_ID_CTRL_VAR_ACTUATOR5_VELOCITY_REF,
	CMIF_ID_CTRL_VAR_ACTUATOR5_TORQUE_REF,
	CMIF_ID_CTRL_VAR_ACTUATOR5_CURRENT_REF,
	CMIF_ID_CTRL_VAR_ACTUATOR5_CURRENT_ACT,
	CMIF_ID_CTRL_VAR_ACTUATOR5_AUXILIRARY_INPUT,
	CMIF_ID_CTRL_VAR_ACTUATOR5_FC_GAIN,
	CMIF_ID_CTRL_VAR_ACTUATOR5_IC_GAIN,
	CMIF_ID_CTRL_VAR_ACTUATOR5_DOB_GAIN,
	CMIF_ID_CTRL_VAR_ACTUATOR5_FF_GAIN,
	CMIF_ID_CTRL_VAR_ACTUATOR5_IRC_GAIN,
	CMIF_ID_CTRL_VAR_ACTUATOR5_PD_GAIN,
	CMIF_ID_CTRL_VAR_ACTUATOR5_IC_CORRIDOR_HALF_WIDTH,
	CMIF_ID_CTRL_VAR_ACTUATOR5_IC_VIRTUAL_SPRING,
	CMIF_ID_CTRL_VAR_ACTUATOR5_IC_VIRTUAL_DAMPER,
	CMIF_ID_CTRL_VAR_ACTUATOR5_IC_IMP_RATIO,

	CMIF_ID_CTRL_VAR_MD6_STATE = CMIF_CTRL_VARIABLE_OFFSET + 5*CTRL_VARIABLE_GAP,
	CMIF_ID_CTRL_VAR_MD6_ERROR_CODE,
	CMIF_ID_CTRL_VAR_ACTUATOR6_POSITION_REF,
	CMIF_ID_CTRL_VAR_ACTUATOR6_VELOCITY_REF,
	CMIF_ID_CTRL_VAR_ACTUATOR6_TORQUE_REF,
	CMIF_ID_CTRL_VAR_ACTUATOR6_CURRENT_REF,
	CMIF_ID_CTRL_VAR_ACTUATOR6_CURRENT_ACT,
	CMIF_ID_CTRL_VAR_ACTUATOR6_AUXILIRARY_INPUT,
	CMIF_ID_CTRL_VAR_ACTUATOR6_FC_GAIN,
	CMIF_ID_CTRL_VAR_ACTUATOR6_IC_GAIN,
	CMIF_ID_CTRL_VAR_ACTUATOR6_DOB_GAIN,
	CMIF_ID_CTRL_VAR_ACTUATOR6_FF_GAIN,
	CMIF_ID_CTRL_VAR_ACTUATOR6_IRC_GAIN,
	CMIF_ID_CTRL_VAR_ACTUATOR6_PD_GAIN,
	CMIF_ID_CTRL_VAR_ACTUATOR6_IC_CORRIDOR_HALF_WIDTH,
	CMIF_ID_CTRL_VAR_ACTUATOR6_IC_VIRTUAL_SPRING,
	CMIF_ID_CTRL_VAR_ACTUATOR6_IC_VIRTUAL_DAMPER,
	CMIF_ID_CTRL_VAR_ACTUATOR6_IC_IMP_RATIO,

	CMIF_ID_CTRL_VAR_MD7_STATE = CMIF_CTRL_VARIABLE_OFFSET + 6*CTRL_VARIABLE_GAP,
	CMIF_ID_CTRL_VAR_MD7_ERROR_CODE,
	CMIF_ID_CTRL_VAR_ACTUATOR7_POSITION_REF,
	CMIF_ID_CTRL_VAR_ACTUATOR7_VELOCITY_REF,
	CMIF_ID_CTRL_VAR_ACTUATOR7_TORQUE_REF,
	CMIF_ID_CTRL_VAR_ACTUATOR7_CURRENT_REF,
	CMIF_ID_CTRL_VAR_ACTUATOR7_CURRENT_ACT,
	CMIF_ID_CTRL_VAR_ACTUATOR7_AUXILIRARY_INPUT,
	CMIF_ID_CTRL_VAR_ACTUATOR7_FC_GAIN,
	CMIF_ID_CTRL_VAR_ACTUATOR7_IC_GAIN,
	CMIF_ID_CTRL_VAR_ACTUATOR7_DOB_GAIN,
	CMIF_ID_CTRL_VAR_ACTUATOR7_FF_GAIN,
	CMIF_ID_CTRL_VAR_ACTUATOR7_IRC_GAIN,
	CMIF_ID_CTRL_VAR_ACTUATOR7_PD_GAIN,
	CMIF_ID_CTRL_VAR_ACTUATOR7_IC_CORRIDOR_HALF_WIDTH,
	CMIF_ID_CTRL_VAR_ACTUATOR7_IC_VIRTUAL_SPRING,
	CMIF_ID_CTRL_VAR_ACTUATOR7_IC_VIRTUAL_DAMPER,
	CMIF_ID_CTRL_VAR_ACTUATOR7_IC_IMP_RATIO,

	CMIF_ID_CTRL_VAR_MD8_STATE = CMIF_CTRL_VARIABLE_OFFSET + 7*CTRL_VARIABLE_GAP,
	CMIF_ID_CTRL_VAR_MD8_ERROR_CODE,
	CMIF_ID_CTRL_VAR_ACTUATOR8_POSITION_REF,
	CMIF_ID_CTRL_VAR_ACTUATOR8_VELOCITY_REF,
	CMIF_ID_CTRL_VAR_ACTUATOR8_TORQUE_REF,
	CMIF_ID_CTRL_VAR_ACTUATOR8_CURRENT_REF,
	CMIF_ID_CTRL_VAR_ACTUATOR8_CURRENT_ACT,
	CMIF_ID_CTRL_VAR_ACTUATOR8_AUXILIRARY_INPUT,
	CMIF_ID_CTRL_VAR_ACTUATOR8_FC_GAIN,
	CMIF_ID_CTRL_VAR_ACTUATOR8_IC_GAIN,
	CMIF_ID_CTRL_VAR_ACTUATOR8_DOB_GAIN,
	CMIF_ID_CTRL_VAR_ACTUATOR8_FF_GAIN,
	CMIF_ID_CTRL_VAR_ACTUATOR8_IRC_GAIN,
	CMIF_ID_CTRL_VAR_ACTUATOR8_PD_GAIN,
	CMIF_ID_CTRL_VAR_ACTUATOR8_IC_CORRIDOR_HALF_WIDTH,
	CMIF_ID_CTRL_VAR_ACTUATOR8_IC_VIRTUAL_SPRING,
	CMIF_ID_CTRL_VAR_ACTUATOR8_IC_VIRTUAL_DAMPER,
	CMIF_ID_CTRL_VAR_ACTUATOR8_IC_IMP_RATIO,

	CMIF_ID_CTRL_VAR_MD9_STATE = CMIF_CTRL_VARIABLE_OFFSET + 8*CTRL_VARIABLE_GAP,
	CMIF_ID_CTRL_VAR_MD9_ERROR_CODE,
	CMIF_ID_CTRL_VAR_ACTUATOR9_POSITION_REF,
	CMIF_ID_CTRL_VAR_ACTUATOR9_VELOCITY_REF,
	CMIF_ID_CTRL_VAR_ACTUATOR9_TORQUE_REF,
	CMIF_ID_CTRL_VAR_ACTUATOR9_CURRENT_REF,
	CMIF_ID_CTRL_VAR_ACTUATOR9_CURRENT_ACT,
	CMIF_ID_CTRL_VAR_ACTUATOR9_AUXILIRARY_INPUT,
	CMIF_ID_CTRL_VAR_ACTUATOR9_FC_GAIN,
	CMIF_ID_CTRL_VAR_ACTUATOR9_IC_GAIN,
	CMIF_ID_CTRL_VAR_ACTUATOR9_DOB_GAIN,
	CMIF_ID_CTRL_VAR_ACTUATOR9_FF_GAIN,
	CMIF_ID_CTRL_VAR_ACTUATOR9_IRC_GAIN,
	CMIF_ID_CTRL_VAR_ACTUATOR9_PD_GAIN,
	CMIF_ID_CTRL_VAR_ACTUATOR9_IC_CORRIDOR_HALF_WIDTH,
	CMIF_ID_CTRL_VAR_ACTUATOR9_IC_VIRTUAL_SPRING,
	CMIF_ID_CTRL_VAR_ACTUATOR9_IC_VIRTUAL_DAMPER,
	CMIF_ID_CTRL_VAR_ACTUATOR9_IC_IMP_RATIO,

	CMIF_ID_CTRL_VAR_MD10_STATE = CMIF_CTRL_VARIABLE_OFFSET + 9*CTRL_VARIABLE_GAP,
	CMIF_ID_CTRL_VAR_MD10_ERROR_CODE,
	CMIF_ID_CTRL_VAR_ACTUATOR10_POSITION_REF,
	CMIF_ID_CTRL_VAR_ACTUATOR10_VELOCITY_REF,
	CMIF_ID_CTRL_VAR_ACTUATOR10_TORQUE_REF,
	CMIF_ID_CTRL_VAR_ACTUATOR10_CURRENT_REF,
	CMIF_ID_CTRL_VAR_ACTUATOR10_CURRENT_ACT,
	CMIF_ID_CTRL_VAR_ACTUATOR10_AUXILIRARY_INPUT,
	CMIF_ID_CTRL_VAR_ACTUATOR10_FC_GAIN,
	CMIF_ID_CTRL_VAR_ACTUATOR10_IC_GAIN,
	CMIF_ID_CTRL_VAR_ACTUATOR10_DOB_GAIN,
	CMIF_ID_CTRL_VAR_ACTUATOR10_FF_GAIN,
	CMIF_ID_CTRL_VAR_ACTUATOR10_IRC_GAIN,
	CMIF_ID_CTRL_VAR_ACTUATOR10_PD_GAIN,
	CMIF_ID_CTRL_VAR_ACTUATOR10_IC_CORRIDOR_HALF_WIDTH,
	CMIF_ID_CTRL_VAR_ACTUATOR10_IC_VIRTUAL_SPRING,
	CMIF_ID_CTRL_VAR_ACTUATOR10_IC_VIRTUAL_DAMPER,
	CMIF_ID_CTRL_VAR_ACTUATOR10_IC_IMP_RATIO,

	CMIF_ID_CTRL_VAR_MD11_STATE = CMIF_CTRL_VARIABLE_OFFSET + 10*CTRL_VARIABLE_GAP,
	CMIF_ID_CTRL_VAR_MD11_ERROR_CODE,
	CMIF_ID_CTRL_VAR_ACTUATOR11_POSITION_REF,
	CMIF_ID_CTRL_VAR_ACTUATOR11_VELOCITY_REF,
	CMIF_ID_CTRL_VAR_ACTUATOR11_TORQUE_REF,
	CMIF_ID_CTRL_VAR_ACTUATOR11_CURRENT_REF,
	CMIF_ID_CTRL_VAR_ACTUATOR11_CURRENT_ACT,
	CMIF_ID_CTRL_VAR_ACTUATOR11_AUXILIRARY_INPUT,
	CMIF_ID_CTRL_VAR_ACTUATOR11_FC_GAIN,
	CMIF_ID_CTRL_VAR_ACTUATOR11_IC_GAIN,
	CMIF_ID_CTRL_VAR_ACTUATOR11_DOB_GAIN,
	CMIF_ID_CTRL_VAR_ACTUATOR11_FF_GAIN,
	CMIF_ID_CTRL_VAR_ACTUATOR11_IRC_GAIN,
	CMIF_ID_CTRL_VAR_ACTUATOR11_PD_GAIN,
	CMIF_ID_CTRL_VAR_ACTUATOR11_IC_CORRIDOR_HALF_WIDTH,
	CMIF_ID_CTRL_VAR_ACTUATOR11_IC_VIRTUAL_SPRING,
	CMIF_ID_CTRL_VAR_ACTUATOR11_IC_VIRTUAL_DAMPER,
	CMIF_ID_CTRL_VAR_ACTUATOR11_IC_IMP_RATIO,

	CMIF_ID_CTRL_VAR_MD12_STATE = CMIF_CTRL_VARIABLE_OFFSET + 11*CTRL_VARIABLE_GAP,
	CMIF_ID_CTRL_VAR_MD12_ERROR_CODE,
	CMIF_ID_CTRL_VAR_ACTUATOR12_POSITION_REF,
	CMIF_ID_CTRL_VAR_ACTUATOR12_VELOCITY_REF,
	CMIF_ID_CTRL_VAR_ACTUATOR12_TORQUE_REF,
	CMIF_ID_CTRL_VAR_ACTUATOR12_CURRENT_REF,
	CMIF_ID_CTRL_VAR_ACTUATOR12_CURRENT_ACT,
	CMIF_ID_CTRL_VAR_ACTUATOR12_AUXILIRARY_INPUT,
	CMIF_ID_CTRL_VAR_ACTUATOR12_FC_GAIN,
	CMIF_ID_CTRL_VAR_ACTUATOR12_IC_GAIN,
	CMIF_ID_CTRL_VAR_ACTUATOR12_DOB_GAIN,
	CMIF_ID_CTRL_VAR_ACTUATOR12_FF_GAIN,
	CMIF_ID_CTRL_VAR_ACTUATOR12_IRC_GAIN,
	CMIF_ID_CTRL_VAR_ACTUATOR12_PD_GAIN,
	CMIF_ID_CTRL_VAR_ACTUATOR12_IC_CORRIDOR_HALF_WIDTH,
	CMIF_ID_CTRL_VAR_ACTUATOR12_IC_VIRTUAL_SPRING,
	CMIF_ID_CTRL_VAR_ACTUATOR12_IC_VIRTUAL_DAMPER,
	CMIF_ID_CTRL_VAR_ACTUATOR12_IC_IMP_RATIO,

	CMIF_ID_CTRL_VAR_MD13_STATE = CMIF_CTRL_VARIABLE_OFFSET + 12*CTRL_VARIABLE_GAP,
	CMIF_ID_CTRL_VAR_MD13_ERROR_CODE,
	CMIF_ID_CTRL_VAR_ACTUATOR13_POSITION_REF,
	CMIF_ID_CTRL_VAR_ACTUATOR13_VELOCITY_REF,
	CMIF_ID_CTRL_VAR_ACTUATOR13_TORQUE_REF,
	CMIF_ID_CTRL_VAR_ACTUATOR13_CURRENT_REF,
	CMIF_ID_CTRL_VAR_ACTUATOR13_CURRENT_ACT,
	CMIF_ID_CTRL_VAR_ACTUATOR13_AUXILIRARY_INPUT,
	CMIF_ID_CTRL_VAR_ACTUATOR13_FC_GAIN,
	CMIF_ID_CTRL_VAR_ACTUATOR13_IC_GAIN,
	CMIF_ID_CTRL_VAR_ACTUATOR13_DOB_GAIN,
	CMIF_ID_CTRL_VAR_ACTUATOR13_FF_GAIN,
	CMIF_ID_CTRL_VAR_ACTUATOR13_IRC_GAIN,
	CMIF_ID_CTRL_VAR_ACTUATOR13_PD_GAIN,
	CMIF_ID_CTRL_VAR_ACTUATOR13_IC_CORRIDOR_HALF_WIDTH,
	CMIF_ID_CTRL_VAR_ACTUATOR13_IC_VIRTUAL_SPRING,
	CMIF_ID_CTRL_VAR_ACTUATOR13_IC_VIRTUAL_DAMPER,
	CMIF_ID_CTRL_VAR_ACTUATOR13_IC_IMP_RATIO,

	CMIF_ID_CTRL_VAR_MD14_STATE = CMIF_CTRL_VARIABLE_OFFSET + 13*CTRL_VARIABLE_GAP,
	CMIF_ID_CTRL_VAR_MD14_ERROR_CODE,
	CMIF_ID_CTRL_VAR_ACTUATOR14_POSITION_REF,
	CMIF_ID_CTRL_VAR_ACTUATOR14_VELOCITY_REF,
	CMIF_ID_CTRL_VAR_ACTUATOR14_TORQUE_REF,
	CMIF_ID_CTRL_VAR_ACTUATOR14_CURRENT_REF,
	CMIF_ID_CTRL_VAR_ACTUATOR14_CURRENT_ACT,
	CMIF_ID_CTRL_VAR_ACTUATOR14_AUXILIRARY_INPUT,
	CMIF_ID_CTRL_VAR_ACTUATOR14_FC_GAIN,
	CMIF_ID_CTRL_VAR_ACTUATOR14_IC_GAIN,
	CMIF_ID_CTRL_VAR_ACTUATOR14_DOB_GAIN,
	CMIF_ID_CTRL_VAR_ACTUATOR14_FF_GAIN,
	CMIF_ID_CTRL_VAR_ACTUATOR14_IRC_GAIN,
	CMIF_ID_CTRL_VAR_ACTUATOR14_PD_GAIN,
	CMIF_ID_CTRL_VAR_ACTUATOR14_IC_CORRIDOR_HALF_WIDTH,
	CMIF_ID_CTRL_VAR_ACTUATOR14_IC_VIRTUAL_SPRING,
	CMIF_ID_CTRL_VAR_ACTUATOR14_IC_VIRTUAL_DAMPER,
	CMIF_ID_CTRL_VAR_ACTUATOR14_IC_IMP_RATIO,

	CMIF_ID_CTRL_VAR_MD15_STATE = CMIF_CTRL_VARIABLE_OFFSET + 14*CTRL_VARIABLE_GAP,
	CMIF_ID_CTRL_VAR_MD15_ERROR_CODE,
	CMIF_ID_CTRL_VAR_ACTUATOR15_POSITION_REF,
	CMIF_ID_CTRL_VAR_ACTUATOR15_VELOCITY_REF,
	CMIF_ID_CTRL_VAR_ACTUATOR15_TORQUE_REF,
	CMIF_ID_CTRL_VAR_ACTUATOR15_CURRENT_REF,
	CMIF_ID_CTRL_VAR_ACTUATOR15_CURRENT_ACT,
	CMIF_ID_CTRL_VAR_ACTUATOR15_AUXILIRARY_INPUT,
	CMIF_ID_CTRL_VAR_ACTUATOR15_FC_GAIN,
	CMIF_ID_CTRL_VAR_ACTUATOR15_IC_GAIN,
	CMIF_ID_CTRL_VAR_ACTUATOR15_DOB_GAIN,
	CMIF_ID_CTRL_VAR_ACTUATOR15_FF_GAIN,
	CMIF_ID_CTRL_VAR_ACTUATOR15_IRC_GAIN,
	CMIF_ID_CTRL_VAR_ACTUATOR15_PD_GAIN,
	CMIF_ID_CTRL_VAR_ACTUATOR15_IC_CORRIDOR_HALF_WIDTH,
	CMIF_ID_CTRL_VAR_ACTUATOR15_IC_VIRTUAL_SPRING,
	CMIF_ID_CTRL_VAR_ACTUATOR15_IC_VIRTUAL_DAMPER,
	CMIF_ID_CTRL_VAR_ACTUATOR15_IC_IMP_RATIO,

	CMIF_ID_CTRL_VAR_MD16_STATE = CMIF_CTRL_VARIABLE_OFFSET + 15*CTRL_VARIABLE_GAP,
	CMIF_ID_CTRL_VAR_MD16_ERROR_CODE,
	CMIF_ID_CTRL_VAR_ACTUATOR16_POSITION_REF,
	CMIF_ID_CTRL_VAR_ACTUATOR16_VELOCITY_REF,
	CMIF_ID_CTRL_VAR_ACTUATOR16_TORQUE_REF,
	CMIF_ID_CTRL_VAR_ACTUATOR16_CURRENT_REF,
	CMIF_ID_CTRL_VAR_ACTUATOR16_CURRENT_ACT,
	CMIF_ID_CTRL_VAR_ACTUATOR16_AUXILIRARY_INPUT,
	CMIF_ID_CTRL_VAR_ACTUATOR16_FC_GAIN,
	CMIF_ID_CTRL_VAR_ACTUATOR16_IC_GAIN,
	CMIF_ID_CTRL_VAR_ACTUATOR16_DOB_GAIN,
	CMIF_ID_CTRL_VAR_ACTUATOR16_FF_GAIN,
	CMIF_ID_CTRL_VAR_ACTUATOR16_IRC_GAIN,
	CMIF_ID_CTRL_VAR_ACTUATOR16_PD_GAIN,
	CMIF_ID_CTRL_VAR_ACTUATOR16_IC_CORRIDOR_HALF_WIDTH,
	CMIF_ID_CTRL_VAR_ACTUATOR16_IC_VIRTUAL_SPRING,
	CMIF_ID_CTRL_VAR_ACTUATOR16_IC_VIRTUAL_DAMPER,
	CMIF_ID_CTRL_VAR_ACTUATOR16_IC_IMP_RATIO,

	CMIF_ID_CTRL_VAR_NUM
} CMIFID_CTRL_VARIABLES;

/*********************** CM GENERATION (0x500 ~ 0x5FF) ***********************/
typedef enum _CMIFID_CM_GEN {
	CMIF_ID_CM_GEN_CM_FSM1_STATE_CURR = CMIF_CM_GENERATION_OFFSET,
	CMIF_ID_CM_GEN_CM_FSM1_STATE_PREV,
	CMIF_ID_CM_GEN_CM_FSM2_STATE_CURR,
	CMIF_ID_CM_GEN_CM_FSM2_STATE_PREV,
	CMIF_ID_CM_GEN_CM_ERROR_CODE,
	CMIF_ID_CM_GEN_CM_TIME_STAMP,
	CMIF_ID_CM_GEN_CM_MOTION_SET_ID,
	CMIF_ID_CM_GEN_CM_ACTION_ID,
	CMIF_ID_CM_GEN_CM_TABLET_MODE_ID,
	CMIF_ID_CM_GEN_ACTUATOR1_GRAVITY_COMP_INPUT,
	CMIF_ID_CM_GEN_ACTUATOR2_GRAVITY_COMP_INPUT,
	CMIF_ID_CM_GEN_ACTUATOR3_GRAVITY_COMP_INPUT,
	CMIF_ID_CM_GEN_ACTUATOR4_GRAVITY_COMP_INPUT,
	CMIF_ID_CM_GEN_ACTUATOR5_GRAVITY_COMP_INPUT,
	CMIF_ID_CM_GEN_ACTUATOR6_GRAVITY_COMP_INPUT,
	CMIF_ID_CM_GEN_ACTUATOR7_GRAVITY_COMP_INPUT,
	CMIF_ID_CM_GEN_ACTUATOR8_GRAVITY_COMP_INPUT,
	CMIF_ID_CM_GEN_ACTUATOR9_GRAVITY_COMP_INPUT,
	CMIF_ID_CM_GEN_ACTUATOR10_GRAVITY_COMP_INPUT,
	CMIF_ID_CM_GEN_ACTUATOR11_GRAVITY_COMP_INPUT,
	CMIF_ID_CM_GEN_ACTUATOR12_GRAVITY_COMP_INPUT,
	CMIF_ID_CM_GEN_ACTUATOR13_GRAVITY_COMP_INPUT,
	CMIF_ID_CM_GEN_ACTUATOR14_GRAVITY_COMP_INPUT,
	CMIF_ID_CM_GEN_ACTUATOR15_GRAVITY_COMP_INPUT,
	CMIF_ID_CM_GEN_ACTUATOR16_GRAVITY_COMP_INPUT,
	CMIF_ID_CM_GEN_ROBOT_COM_POSITION,
	CMIF_ID_CM_GEN_ROBOT_COM_VELOCITY,
	CMIF_ID_CM_GEN_ROBOT_COM_ACCELERATION,
	CMIF_ID_CM_GEN_ROBOT_COM_ZMP,

	CMIF_ID_CM_GEN_NUM
} CMIFID_CM_GEN;

/*********************** AM SEND (0x600 ~ 0x6FF) ***********************/
typedef enum _CMIFID_AM_SEND {
	CMIF_ID_AM_SEND_AM_STATE_CURR = CMIF_AM_SEND_OFFSET,
	CMIF_ID_AM_SEND_AM_STATE_PREV,
	CMIF_ID_AM_SEND_AM_ERROR_CODE,
	CMIF_ID_AM_SEND_AM_ABS_CLOCK_YEAR,
	CMIF_ID_AM_SEND_AM_ABS_CLOCK_MONTH,
	CMIF_ID_AM_SEND_AM_ABS_CLOCK_DAY,
	CMIF_ID_AM_SEND_AM_ABS_CLOCK_HOUR,
	CMIF_ID_AM_SEND_AM_ABS_CLOCK_MINUTE,
	CMIF_ID_AM_SEND_AM_ABS_CLOCK_SECOND,
	CMIF_ID_AM_SEND_AM_OPERATION_TIME,

	CMIF_ID_AM_SEND_AM_USER_NAME,
	CMIF_ID_AM_SEND_AM_USER_SEX,
	CMIF_ID_AM_SEND_AM_USER_AGE,
	CMIF_ID_AM_SEND_AM_USER_WEIGHT,
	CMIF_ID_AM_SEND_AM_USER_HEIGHT,
	CMIF_ID_AM_SEND_AM_USER_PELVIS_WIDTH,
	CMIF_ID_AM_SEND_AM_USER_THIGH_LENGTH,
	CMIF_ID_AM_SEND_AM_USER_SHANK_LENGTH,
	CMIF_ID_AM_SEND_AM_USER_FOOT_SIZE,
	CMIF_ID_AM_SEND_AM_USER_CHEST_SIZE,
	CMIF_ID_AM_SEND_AM_USER_WAIST_SIZE,

	CMIF_ID_AM_SEND_AM_ACTUATOR1_INPUT,
	CMIF_ID_AM_SEND_AM_ACTUATOR2_INPUT,
	CMIF_ID_AM_SEND_AM_ACTUATOR3_INPUT,
	CMIF_ID_AM_SEND_AM_ACTUATOR4_INPUT,
	CMIF_ID_AM_SEND_AM_ACTUATOR5_INPUT,
	CMIF_ID_AM_SEND_AM_ACTUATOR6_INPUT,
	CMIF_ID_AM_SEND_AM_ACTUATOR7_INPUT,
	CMIF_ID_AM_SEND_AM_ACTUATOR8_INPUT,
	CMIF_ID_AM_SEND_AM_ACTUATOR9_INPUT,
	CMIF_ID_AM_SEND_AM_ACTUATOR10_INPUT,
	CMIF_ID_AM_SEND_AM_ACTUATOR11_INPUT,
	CMIF_ID_AM_SEND_AM_ACTUATOR12_INPUT,
	CMIF_ID_AM_SEND_AM_ACTUATOR13_INPUT,
	CMIF_ID_AM_SEND_AM_ACTUATOR14_INPUT,
	CMIF_ID_AM_SEND_AM_ACTUATOR15_INPUT,
	CMIF_ID_AM_SEND_AM_ACTUATOR16_INPUT,

	CMIF_ID_AM_SEND_NUM
} CMIFID_AM_SEND;


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




#endif /* CM_BACKBONE_COMMON_FILES_CM_IF_DATA_DICTIONARY_H_ */
