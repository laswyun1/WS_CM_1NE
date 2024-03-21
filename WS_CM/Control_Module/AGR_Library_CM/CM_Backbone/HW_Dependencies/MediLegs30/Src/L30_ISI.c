
#include "L30_ISI.h"

#ifdef L30_CM_ENABLED

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

uint8_t isi_input_vectors[ISI_INPUT_N_MAX];
uint8_t isi_output_vectors[ISI_OUTPUT_N_MAX];

ButtonObj Guide_L;
ButtonObj Guide_R;

ButtonObj Crutch_Idx_Finger; // HIP_L_D_SW_P
ButtonObj Crutch_Thumb_L;    // HIP_L_D_SW_N
ButtonObj Crutch_Thumb_R;    // HIP_L_L_SW_P


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

static void Init_Button(ButtonObj* t_button, GPIO_TypeDef* t_GPIO_port, uint16_t t_GPIO_pin, uint8_t t_idle_state);


/**
 *------------------------------------------------------------
 *                      PUBLIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions that interface with this module.
 */

void Init_Button_Interface(void)
{
  	Init_Button(&Guide_L,             GPIOD, IOIF_GPIO_PIN_5, ACTIVE_LOW);
    Init_Button(&Guide_R,             GPIOD, IOIF_GPIO_PIN_4, ACTIVE_LOW);
    Init_Button(&Crutch_Thumb_L,      GPIOE, IOIF_GPIO_PIN_7, ACTIVE_HIGH);
    Init_Button(&Crutch_Thumb_R,      GPIOE, IOIF_GPIO_PIN_8, ACTIVE_HIGH);
    Init_Button(&Crutch_Idx_Finger,   GPIOE, IOIF_GPIO_PIN_4, ACTIVE_HIGH);
}

void Read_Button(ButtonObj* t_button)
{
	uint8_t t_state;
	t_state = HAL_GPIO_ReadPin(t_button->GPIO_port, t_button->GPIO_pin);

	if      (t_button->idle_state == ACTIVE_HIGH)
	{
		if (t_state) t_button->state_curr = 1;
		else         t_button->state_curr = 0;
	}
	else if (t_button->idle_state == ACTIVE_LOW)
	{
		if (t_state) t_button->state_curr = 0;
		else         t_button->state_curr = 1;
	}

	if (t_button->state_curr == 0)
		t_button->on_time = 0;
	else
		t_button->on_time++;
}

void Flush_ISI(void)
{
	for (int i = 0; i < ISI_OUTPUT_N_MAX; i++)
	{
		isi_output_vectors[i] = 0;
	}
	Guide_L.on_time = 0;
	Guide_R.on_time = 0;
	Crutch_Idx_Finger.on_time = 0;
	Crutch_Thumb_L.on_time = 0;
	Crutch_Thumb_R.on_time = 0;
}

void Check_ISI(void) // temporal...
{
	Read_Button(&Guide_L);
	Read_Button(&Guide_R);
	Read_Button(&Crutch_Idx_Finger);
	Read_Button(&Crutch_Thumb_L);
	Read_Button(&Crutch_Thumb_R);

	// B-Flag 0
	if (Crutch_Idx_Finger.on_time > 1500)
		isi_output_vectors[EXT1] = 1;
	else
		isi_output_vectors[EXT1] = 0;

	// B-Flag 1
	if (Crutch_Thumb_L.on_time > 1500)
		isi_output_vectors[EXT2] = 1;
	else
		isi_output_vectors[EXT2] = 0;

	// B-Flag 2
	if (Crutch_Thumb_R.on_time > 1500)
		isi_output_vectors[EXT3] = 1;
	else
		isi_output_vectors[EXT3] = 0;

	// B-Flag 3
	if (Guide_L.on_time > 2000)
		isi_output_vectors[EXT4] = 1;
	else
		isi_output_vectors[EXT4] = 0;

	// B-Flag 4
	if (Guide_R.on_time > 2000)
		isi_output_vectors[EXT5] = 1;
	else
		isi_output_vectors[EXT5] = 0;

}


/**
 *------------------------------------------------------------
 *                      STATIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions intended for internal use within this module.
 */

static void Init_Button(ButtonObj* t_button, GPIO_TypeDef* t_GPIO_port, uint16_t t_GPIO_pin, uint8_t t_idle_state)
{
	t_button->GPIO_port  = t_GPIO_port;
	t_button->GPIO_pin   = t_GPIO_pin;
	t_button->idle_state = t_idle_state;
}


#endif /* L30_CM_ENABLED */
