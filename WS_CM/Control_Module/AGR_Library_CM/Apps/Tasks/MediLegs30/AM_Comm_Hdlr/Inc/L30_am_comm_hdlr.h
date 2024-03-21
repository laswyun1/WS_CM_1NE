/*
 * am_comm_hdlr.h
 *
 *  Created on: Nov 22, 2023
 *      Author: Angelrobotics
 */

#ifndef AM_COMM_HDLR_INC_L30_AM_COMM_HDLR_H_
#define AM_COMM_HDLR_INC_L30_AM_COMM_HDLR_H_

#include "module.h"

#ifdef L30_CM_ENABLED

#include <math.h>

#include "lwip.h"
#include "lwip/api.h"
#include "cmsis_os.h"

#include "data_object_common.h"

/**
 *-----------------------------------------------------------
 *              MACROS AND PREPROCESSOR DIRECTIVES
 *-----------------------------------------------------------
 * @brief Directives and macros for readability and efficiency.
 */

#define ETH_CTRL_TCP_PORT 1818

#define ETH_CTRL_TX_BUFF_SIZE 136

#define ETH_CTRL_OK          0
#define ETH_CTRL_DOP_TX_ERR -1
#define ETH_CTRL_DOP_RX_ERR -2
#define ETH_CTRL_TCP_TX_ERR -3
#define ETH_CTRL_TCP_RX_ERR -4


/**
 *------------------------------------------------------------
 *                     TYPE DECLARATIONS
 *------------------------------------------------------------
 * @brief Custom data types and structures for the module.
 */

typedef struct NetconnHandle {
	struct netconn *srv, *cli;
	struct netbuf *buff;
	uint8_t tx_msg[ETH_CTRL_TX_BUFF_SIZE];
	err_t err;
} NetconnHandle;

typedef struct CMtoAMBuffer {

	uint32_t RH_index;
	float RH_pos_ref;
	float RH_pos_act;
	float RH_cur_ref;
	float RH_cur_act;

	uint32_t RK_index;
	float RK_pos_ref;
	float RK_pos_act;
	float RK_cur_ref;
	float RK_cur_act;

	uint32_t RA_index;
	float RA_pos_ref;
	float RA_pos_act;
	float RA_cur_ref;
	float RA_cur_act;

	uint32_t LH_index;
	float LH_pos_ref;
	float LH_pos_act;
	float LH_cur_ref;
	float LH_cur_act;

	uint32_t LK_index;
	float LK_pos_ref;
	float LK_pos_act;
	float LK_cur_ref;
	float LK_cur_act;

	uint32_t LA_index;
	float LA_pos_ref;
	float LA_pos_act;
	float LA_cur_ref;
	float LA_cur_act;

	int Mode;
	int Submode;
	int State;
	uint32_t Step;

} CMtoAMBuffer;

typedef struct AMtoCMBuffer {

	int Mode;
	int Submode;
	int State;

	uint32_t HJ;

	uint32_t SB;
	uint32_t TY;
	uint32_t RH_pos_act;
	uint32_t RH_cur_ref;
	uint32_t RH_cur_act;

	uint32_t RK_index;
	uint32_t RK_pos_ref;
	uint32_t RK_pos_act;
	uint32_t RK_cur_ref;
	uint32_t RK_cur_act;

	uint32_t RA_index;
	uint32_t RA_pos_ref;
	uint32_t RA_pos_act;
	uint32_t RA_cur_ref;
	uint32_t RA_cur_act;

	uint32_t LH_index;
	uint32_t LH_pos_ref;
	uint32_t LH_pos_act;
	uint32_t LH_cur_ref;
	uint32_t LH_cur_act;

	uint32_t LK_index;
	uint32_t LK_pos_ref;
	uint32_t LK_pos_act;
	uint32_t LK_cur_ref;
	uint32_t LK_cur_act;

	uint32_t LA_index;
	uint32_t LA_pos_ref;
	uint32_t LA_pos_act;
	uint32_t LA_cur_ref;
	uint32_t LA_cur_act;


} AMtoCMBuffer;


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

// Main
void InitAMCommHdlr(void);
void RunAMCommHdlr(void);


#endif /* L30_CM_ENABLED */

#endif /* AM_COMM_HDLR_INC_L30_AM_COMM_HDLR_H_ */
