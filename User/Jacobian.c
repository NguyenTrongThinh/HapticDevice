#include <Jacobian.h>

#define INVERSE			0x01
#define NON_INVERSE 0x02

const float  R = 73.5;
const float  a = 57.5;
const float  b = 160.0;
const float  r = 32.0;
const double Pi = 3.1415926539;

void JacobianCalAngle(float Phii, float Theta1i, float px, float py, float pz, float *Theta2i, float *Theta3i)
{
	float Td, Md, DotABBCi_;
	Phii =Phii*(float)Pi/180.0;
	Theta1i = Theta1i*(float)Pi/180.0;
	
	if (Theta1i == 0) Theta1i = 0.000001;
	Td = R*a*sin(Theta1i)*(py*cos(Phii) - px*sin(Phii));
	Md = sqrt(pow(R, 2)*pow(a, 2)*pow(sin(Theta1i), 2));
	*Theta3i = acos(abs(Td)/(Md*b));
	if (Td < 0)
		*Theta3i = Pi - acos(abs(Td)/(Md*b));
	DotABBCi_ = -a*(a + R*cos(Theta1i) - r*cos(Theta1i) + pz*sin(Theta1i) - px*cos(Theta1i)*cos(Phii)
				- py*cos(Theta1i)*sin(Phii));
	*Theta2i = acos(DotABBCi_/(a*b*sin(*Theta3i)));
}

float Det(float Jp[3][3])
{
	return   (Jp[0][0])*(Jp[1][1])*(Jp[2][2]) + (Jp[0][1])*(Jp[1][2])*(Jp[2][0]) 
	       + (Jp[0][2])*(Jp[1][0])*(Jp[2][1]) - (Jp[0][2])*(Jp[1][1])*(Jp[2][0])
	       - (Jp[0][1])*(Jp[1][0])*(Jp[2][2]) - (Jp[0][0])*(Jp[1][2])*(Jp[2][1]);
}

float Glosbe(float Jp[3][3], unsigned char row, unsigned char col)
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

static void MultiplyMax(float MA[3][3], float MB[3][3], float (*MR)[3][3])
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

static char Inv(float Jp[3][3], float (*iJp)[3][3])
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

char MomentCalculate(float Theta1[3], float Phi[3], float P[3], float F[3], float (*Moment)[3])
{
	char ErrorCode  = MNAVAILABLE;
	unsigned char i = 0, j;
	float Jp[3][3];
	float IJp[3][3];
	float Jq[3][3];
	float Theta[3][3];  
	float J[3][3];
	
	Theta[0][0] = Theta1[0];
	Theta[0][1] = Theta1[1];
	Theta[0][2] = Theta1[2];
	
	JacobianCalAngle(Phi[0], Theta[0][0], P[0], P[1], P[2], &Theta[1][0], &Theta[2][0]);
	JacobianCalAngle(Phi[1], Theta[0][1], P[0], P[1], P[2], &Theta[1][1], &Theta[2][1]);
	JacobianCalAngle(Phi[2], Theta[0][2], P[0], P[1], P[2], &Theta[1][2], &Theta[2][2]);
	for (i = 0; i<3; i++)
	{
		Phi[i] = Phi[i]*(float)Pi/180.0;
		Theta[0][i] = Theta[0][i]*(float)Pi/180.0;
	}

	for (i = 0; i<3; i++)
	{
		Jp[i][0] =  sin(Theta[2][i])*cos(Theta[1][i] + Theta[0][i])*cos(Phi[i]) + cos(Theta[2][i])*sin(Phi[i]);
		Jp[i][1] = -sin(Theta[2][i])*cos(Theta[1][i] + Theta[0][i])*cos(Phi[i]) + cos(Theta[2][i])*sin(Phi[i]);
		Jp[i][2] =  sin(Theta[2][i])*sin(Theta[1][i] + Theta[0][i]);
	}
	for(i = 0; i<3; i++)
		for(j = 0; j<3; j++)
		{
			if (i == j)
			{
				Jq[i][j] = a*sin(Theta[1][i])*sin(Theta[2][i]);
			}
			else
				Jq[i][j] = 0.0;
		}
	if (Inv(Jp, &IJp) == INVERSE)
	{
		MultiplyMax(IJp, Jq, &J);		
		(*Moment)[0] = J[0][0]*F[0] + J[0][1]*F[1] + J[0][2]*F[2];
		(*Moment)[1] = J[1][0]*F[0] + J[1][1]*F[1] + J[1][2]*F[2];
		(*Moment)[2] = J[2][0]*F[0] + J[2][1]*F[1] + J[2][2]*F[2];
		ErrorCode = MAVAILABLE;
	}
	else
	{
	  (*Moment)[0] = 0;
		(*Moment)[1] = 0;
		(*Moment)[2] = 0;
	}
	return ErrorCode;
}
