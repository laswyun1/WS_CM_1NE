
#include "AS_ISI.h"

#ifdef SUIT_MINICM_ENABLED

/**
 *-----------------------------------------------------------
 *              TYPE DEFINITIONS AND ENUMERATIONS
 *-----------------------------------------------------------
 * @brief Enumerated types and structures central to this module.
 */




/**
 *------------------------------------------------------------
 *                      GLOBAL VARIABLES
 *------------------------------------------------------------
 * @brief Variables accessible throughout the application.
 */

uint8_t isi_output_vectors[ISI_OUTPUT_N_MAX];

ISI_Flag_t ISIFlags_Bt;

ISI_Flag_t ISIFlags_RH;
ISI_Flag_t ISIFlags_LH;

ISI_Flag_t ISIFlags_RK;
ISI_Flag_t ISIFlags_LK;

ISI_Flag_t ISIFlags_RA;
ISI_Flag_t ISIFlags_LA;


/**
 *------------------------------------------------------------
 *                      STATIC VARIABLES
 *------------------------------------------------------------
 * @brief Variables local to this module.
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

void Flush_ISI(void)
{
	for (int i = 0; i < ISI_OUTPUT_N_MAX; i++)
	{
		isi_output_vectors[i] = 0;
	}
}

void Check_ISI(void)
{
	// B-Flag 1 Gait Mode Walk
	if (ISIFlags_Bt.I22_Mode == GAIT_MODE_WALK)
		isi_output_vectors[EXT1] = 1;

	// B-Flag 2 Gait Mode Stop
	if (ISIFlags_Bt.I22_Mode == GAIT_MODE_STOP)
		isi_output_vectors[EXT2] = 1;
}


/**
 *------------------------------------------------------------
 *                      STATIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions intended for internal use within this module.
 */




#endif /* SUIT_MINICM_ENABLED */
