#ifndef __AMES_MATRIX_H__
	#define __AMES_MATRIX_H__
#ifdef __cplusplus
extern "C"
{
#endif

#include <stdlib.h>
#include <math.h>

#define INVERSE			  0x01
#define NON_INVERSE		0x02

float Det(float Jp[3][3]);
void MultiplyMax(float MA[3][3], float MB[3][3], float (*MR)[3][3]);
char Inv(float Jp[3][3], float (*iJp)[3][3]);
void Turn(float input[3][3], float (*Output)[3][3]);



#ifdef __cplusplus
}
#endif
#endif /* __AMES_MATRIX_H__ */
