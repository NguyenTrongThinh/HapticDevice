/****************************************************************************/
/*Delta Kinematic library.																														*/
/*This library is copyright and Protected by AMES lab		    								*/
/*Date: 07/06/2015																													*/
/*Author: Thinh Nguyen										    															*/ 																
/****************************************************************************/
#ifndef __DELTA_H__
	#define __DELTA_H__
#ifdef __cplusplus
extern "C"
{
#endif

#include <math.h>
#include <stdlib.h>

int Delta_CalcForward(float theta1, float theta2, float theta3, float *x0, float *y0, float *z0);
int Delta_CalcAngleYZ(float x0, float y0, float z0, float *theta);
int Delta_CalcInverse(float x0, float y0, float z0, float *theta1, float *theta2, float *theta3); 

	#ifdef __cplusplus
}
#endif
#endif /* __DELTA_H__ */
