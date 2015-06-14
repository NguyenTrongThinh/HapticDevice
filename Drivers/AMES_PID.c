#include <AMES_PID.h>

float *PID_Kp, *PID_Ki, *PID_Kd;
float *PID_MinOut, *PID_MaxOut;
float *PID_Integrated;
float *PID_Prev_Input;
float *Result;

unsigned char PID_Mem_Create(unsigned char NumberOfChannel)
{
	PID_Kp = (float*)calloc(NumberOfChannel, sizeof(float));
	PID_Ki = (float*)calloc(NumberOfChannel, sizeof(float));
	PID_Kd = (float*)calloc(NumberOfChannel, sizeof(float));
	PID_MinOut = (float*)calloc(NumberOfChannel, sizeof(float));
	PID_MaxOut = (float*)calloc(NumberOfChannel, sizeof(float));
	PID_Integrated = (float*)calloc(NumberOfChannel, sizeof(float));
	PID_Prev_Input = (float*)calloc(NumberOfChannel, sizeof(float));
	Result = (float*)malloc(sizeof(float));
	if (Result == 0 ||PID_Kp == 0 || PID_Ki == 0 || PID_Kd == 0 || PID_MinOut == 0 || PID_MaxOut == 0 || PID_Integrated == 0 || PID_Prev_Input == 0)
		return  FAIL;
	else
		return PASS;
}
void PID_Free_Mem(void)
{
	free(PID_Kp);
	free(PID_Ki);
	free(PID_Kd);
	free(PID_MinOut);
	free(PID_MaxOut);
	free(PID_Integrated);
	free(PID_Prev_Input);
}
void PID_Reset(unsigned char Channel)
{
  PID_Kp[Channel]         = 0;
  PID_Ki[Channel]         = 0;
  PID_Kd[Channel]         = 0;
  PID_MinOut[Channel]     = 0;
  PID_MaxOut[Channel]     = 0;
  PID_Integrated[Channel] = 0.0;
  PID_Prev_Input[Channel] = 0.0;
}
void PID_Init(unsigned char Channel, PIDCoff Coff)
{	
  PID_Kp[Channel]         = Coff.Kp;
  PID_Ki[Channel]         = Coff.Ki;
  PID_Kd[Channel]         = Coff.Kd;
  PID_Integrated[Channel] = 0.0;
  PID_Prev_Input[Channel] = 0.0;
}
void PID_WindUp_Init(unsigned char Channel, float Max_Value)
{
	 PID_MinOut[Channel]     = -Max_Value;
   PID_MaxOut[Channel]      = Max_Value;
}
long PID_Calculate(unsigned char Channel, double SetPoint, double InputValue)
{
		float Error, ProValue, DiffValue;
    Error = SetPoint - InputValue;
    ProValue = PID_Kp[Channel]*Error;
    PID_Integrated[Channel] += PID_Ki[Channel]*Error;
    if (PID_Integrated[Channel] < PID_MinOut[Channel])
       PID_Integrated[Channel] = PID_MinOut[Channel];
    else if (PID_Integrated[Channel] > PID_MaxOut[Channel])
       PID_Integrated[Channel] = PID_MaxOut[Channel];
    DiffValue = (InputValue - PID_Prev_Input[Channel]) * PID_Kd[Channel];
    PID_Prev_Input[Channel] = InputValue;
    *Result = ProValue + PID_Integrated[Channel] - DiffValue;
    if (*Result < PID_MinOut[Channel])
       *Result = PID_MinOut[Channel];
    else if (*Result > PID_MaxOut[Channel])
       *Result = PID_MaxOut[Channel];
    return *Result;
}
