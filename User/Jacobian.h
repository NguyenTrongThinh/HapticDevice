/****************************************************************************/
/*Delta Robot Jacobian library.																											*/
/*This library is copyright and Protected by AMES lab		    								*/
/*Date: 08/05/2015																													*/
/*Author: Thinh Nguyen										    															*/ 																
/****************************************************************************/
#ifndef __JACOBIAN_H__
	#define JACOBIAN_H

#include <stdlib.h>
#include <math.h>

#define MAVAILABLE 	0x03
#define MNAVAILABLE	0x04

char MomentCalculate(float Theta1[3], float Phi[3], float P[3], float F[3], float (*Moment)[3]);
void JacobianCalAngle(float Phii, float Theta1i, float px, float py, float pz, float *Theta2i, float *Theta3i);
#endif 
