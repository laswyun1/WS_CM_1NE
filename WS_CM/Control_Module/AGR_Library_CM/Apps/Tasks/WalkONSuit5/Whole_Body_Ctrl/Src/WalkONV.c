#include "WalkONV.h"

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


float abs2inc_RH_COR(float x)
{
	float x_rad;
	float y_rad;

	float y1, y2;

	float a = 158.5;
	float b = 49480;
	float c = 1.358;

	x_rad = x * 0.017453292519943; //convert deg2rad

	y1 = sqrt(a*a + b*b - 2*a*b*cos(x_rad+c));
	y2 = sqrt(a*a + b*b - 2*a*b*cos(c));
	y_rad = y1 - y2;

	return y_rad;
}

float abs2inc_LH_COR(float x)
{
	float x_rad;
	float y_rad;

	float y1, y2;

	float a = 155.6;
	float b = 59988;
	float c = 1.499;

	x_rad = x * 0.017453292519943; //convert deg2rad

	y1 = sqrt(a*a + b*b - 2*a*b*cos(x_rad+c));
	y2 = sqrt(a*a + b*b - 2*a*b*cos(c));
	y_rad = y1 - y2;

	return y_rad;
}

float abs2inc_RH_ROT(float x)
{
	float x_rad;
	float y_rad;

	x_rad = x * 0.017453292519943; //convert deg2rad
	y_rad = x_rad;

	return y_rad;
}

float abs2inc_LH_ROT(float x)
{
	float x_rad;
	float y_rad;

	x_rad = x * 0.017453292519943; //convert deg2rad
	y_rad = x_rad;

	return y_rad;
}

float abs2inc_RH_SAG(float x)
{
	float x_rad;
	float y_rad;

	x_rad = x * 0.017453292519943; //convert deg2rad
	y_rad = x_rad;

	return y_rad;
}

float abs2inc_LH_SAG(float x)
{
	float x_rad;
	float y_rad;

	x_rad = x * 0.017453292519943; //convert deg2rad
	y_rad = x_rad;

	return y_rad;
}

float abs2inc_RK(float x)
{
	float x_rad;
	float y_rad;

	x_rad = x * 0.017453292519943; //convert deg2rad
	y_rad = x_rad;

	return y_rad;
}

float abs2inc_LK(float x)
{
	float x_rad;
	float y_rad;

	x_rad = x * 0.017453292519943; //convert deg2rad
	y_rad = x_rad;

	return y_rad;
}

float abs2inc_LA_MED(float x_ie, float x_dp)
{
	float ax = 0.1962;
	float ay = -0.06029;
	float az = -0.1;
	float bx = 0.2;
	float by = -0.06562;
	float bz = 0.2278;
	float l0 = 0.3231;
	float loffset = 0.054;

	float x = x_ie * 0.017453292519943; // deg2rad
	float y = x_dp * 0.017453292519943; // deg2rad
	float y_out;

	double y1, y2, y3;

	y1 = bx*cos(y)+(by*sin(-x)+bz*cos(-x))*sin(y)-ax;
	y2 = by*cos(-x)-bz*sin(-x)-ay;
	y3 = -bx*sin(y)+(by*sin(-x)+bz*cos(-x))*cos(y)-az;

	y_out = y1*y1 + y2*y2 + y3*y3 -loffset*loffset;
	if (y_out < 0)
		y_out = 0;

	y_out = (l0 - sqrt(y_out))*628.3185307179587;

	return y_out;
}

float abs2inc_LA_LAT(float x_ie, float x_dp)
{
	float ax = 0.1946;
	float ay = 0.05307;
	float az = -0.09997;
	float bx = 0.2;
	float by = 0.03242;
	float bz = 0.2268;
	float l0 = 0.3224;
	float loffset = 0.05631;

	float x = x_ie * 0.017453292519943; // deg2rad
	float y = x_dp * 0.017453292519943; // deg2rad
	float y_out;

	double y1, y2, y3;

	y1 = bx*cos(y)+(by*sin(-x)+bz*cos(-x))*sin(y)-ax;
	y2 = by*cos(-x)-bz*sin(-x)-ay;
	y3 = -bx*sin(y)+(by*sin(-x)+bz*cos(-x))*cos(y)-az;

	y_out = y1*y1 + y2*y2 + y3*y3 -loffset*loffset;
	if (y_out < 0)
		y_out = 0;

	y_out = (l0 - sqrt(y_out))*628.3185307179587;

	return y_out;
}

float abs2inc_RA_MED(float x_ie, float x_dp)
{
	float ax = 0.2003;
	float ay = -0.06095;
	float az = -0.09756;
	float bx = 0.1915;
	float by = -0.06981;
	float bz = 0.2262;
	float l0 = 0.3203;
	float loffset = 0.04864;

	float x = x_ie * 0.017453292519943; // deg2rad
	float y = x_dp * 0.017453292519943; // deg2rad
	float y_out;

	double y1, y2, y3;

	y1 = bx*cos(y)+(by*sin(-x)+bz*cos(-x))*sin(y)-ax;
	y2 = by*cos(-x)-bz*sin(-x)-ay;
	y3 = -bx*sin(y)+(by*sin(-x)+bz*cos(-x))*cos(y)-az;

	y_out = y1*y1 + y2*y2 + y3*y3 -loffset*loffset;
	if (y_out < 0)
		y_out = 0;

	y_out = (l0 - sqrt(y_out))*628.3185307179587;

	return y_out;
}

float abs2inc_RA_LAT(float x_ie, float x_dp)
{
	float ax = 0.2043;
	float ay = 0.06305;
	float az = -0.1129;
	float bx = 0.191;
	float by = 0.03386;
	float bz = 0.2242;
	float l0 = 0.3385;
	float loffset = 2.374e-07;

	float x = x_ie * 0.017453292519943;
	float y = x_dp * 0.017453292519943;
	float y_out;

	double y1, y2, y3;

	y1 = bx*cos(y)+(by*sin(-x)+bz*cos(-x))*sin(y)-ax;
	y2 = by*cos(-x)-bz*sin(-x)-ay;
	y3 = -bx*sin(y)+(by*sin(-x)+bz*cos(-x))*cos(y)-az;

	y_out = y1*y1 + y2*y2 + y3*y3 -loffset*loffset;
	if (y_out < 0)
		y_out = 0;

	y_out = (l0 - sqrt(y_out))*628.3185307179587;

	return y_out;
}


/**
 *------------------------------------------------------------
 *                      STATIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions intended for internal use within this module.
 */




#endif /* WALKON5_CM_ENABLED */
