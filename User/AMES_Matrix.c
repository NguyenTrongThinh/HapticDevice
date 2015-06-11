#include "AMES_Matrix.h"

float Det(float Jp[3][3])
{
	return   (Jp[0][0])*(Jp[1][1])*(Jp[2][2]) + (Jp[0][1])*(Jp[1][2])*(Jp[2][0]) 
	       + (Jp[0][2])*(Jp[1][0])*(Jp[2][1]) - (Jp[0][2])*(Jp[1][1])*(Jp[2][0])
	       - (Jp[0][1])*(Jp[1][0])*(Jp[2][2]) - (Jp[0][0])*(Jp[1][2])*(Jp[2][1]);
}
static float Glosbe(float Jp[3][3], unsigned char row, unsigned char col)
{
	unsigned char i, j;
	unsigned char k = 0;
	float Temp[4];
	float ReturnVal;
	for (i = 0; i<3; i++)
		for(j = 0; j<3; j++)
		{
			if (i != row && j != col)
				Temp[k++] = Jp[i][j];
		}	
	ReturnVal = (Temp[0]*Temp[3] - Temp[2]*Temp[1]);
	if ((row + col + 2)%2 != 0)
		ReturnVal = -ReturnVal;
	return ReturnVal;
}
void MultiplyMax(float MA[3][3], float MB[3][3], float (*MR)[3][3])
{
	unsigned char i, j, k;
	 for(i = 0; i < 3; i++)
      	for(j = 0; j < 3; j++) 
	  	{
         (*MR)[i][j] = 0;
         for(k = 0; k < 3; k++)
            ((*MR)[i][j]) = ((*MR)[i][j])+ MA[i][k]*MB[k][j];
        }
}
void Turn(float input[3][3], float (*Output)[3][3])
{
	unsigned char i, j;
	for (i = 0; i <3; i++)
		for (j = 0; j<3; j++)
			(*Output)[i][j] = input[j][i];
}
char Inv(float Jp[3][3], float (*iJp)[3][3])
{
	float CJp[3][3];
	unsigned char i, j;
	float det = Det(Jp);
	if (det <= 0.00001 && det >= -0.00001)
		return NON_INVERSE;
	for (i = 0; i<3; i++)
		for(j = 0; j<3; j++)
		{
			CJp[i][j] = Glosbe(Jp, i, j);
		}
	for (i = 0; i<3; i++)
		for(j = 0; j<3; j++)
		{
			(*iJp)[i][j] = CJp[j][i]/det;
		}
	return INVERSE;
}
