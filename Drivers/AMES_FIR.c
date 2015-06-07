#include <AMES_FIR.h>

double coeffs[ FILTER_LEN ] =
{
  -0.027732316158170525,  0.004051843939535753,  0.004804402734504417,  0.00601384538827639,
  0.007419593537679506,  0.008744370169660056,  0.009697592273285309,  0.010010620818934437,
  0.009465204694996392,  0.007925321520822445,  0.005361517184101381,  0.0018661845159506955,
  -0.002338576488055126,  -0.006912733583783749,  -0.011410636243304193,  -0.01535743714343223,
  -0.01815371830157696,  -0.019446787472739643,  -0.0186434152032973,  -0.015480109297715917,
  -0.009799600611365802,  -0.0016274697314847854,  0.008818910114966898,  0.021127549749613176,
  0.034714745276265384,  0.04886074084162252,  0.06275939798552578,  0.07557888518649461,
  0.08652072158879791,  0.0948870229680503,  0.10011939615445992,  0.10192108323678618,
  0.10011939615445992,  0.0948870229680503,  0.08652072158879791,  0.07557888518649461,
  0.06275939798552578,  0.04886074084162252,  0.03471474527626538,  0.021127549749613176,
  0.008818910114966898,  -0.0016274697314847854,  -0.009799600611365802,  -0.015480109297715917,
  -0.0186434152032973,  -0.019446787472739643,  -0.01815371830157696,  -0.01535743714343223,
  -0.011410636243304193,  -0.006912733583783749,  -0.002338576488055126,  0.0018661845159506955,
  0.005361517184101381,  0.007925321520822445,  0.009465204694996392,  0.010010620818934437,
  0.009697592273285309,  0.008744370169660056,  0.007419593537679506,  0.006013845388276394,
  0.004804402734504412,  0.00405184393953575,  -0.027732316158170525
};

double Input_SEN[NUM_CHANNEL][NUM_SAMPLES];

double Output[NUM_SAMPLES];

double InputSample[BUFFER_LEN];

void FIR_Init(void)
{
	memset(InputSample, 0, sizeof(InputSample));
	memset(Input_SEN, 0, sizeof(Input_SEN));
}
void Median_Filter(double *input, double *output, int length)
{
	unsigned char i = 0;
	unsigned char j = 0;
	unsigned char k = 0;
	unsigned char min = 0;
	double Temp;
	for (i = 2; i<length - 2; ++i)
	{
		double windows[5];
		for(j = 0; j<5; ++j)
			windows[j] = input[i - 2 + j];
		for (j = 0; j < 3; ++j)
		{
			min = j;
			for (k = j + 1; k < 5; ++k)
				if (windows[k] < windows[min])
					min = k;
				Temp = windows[j];
				windows[j] = windows[min];
				windows[min] = Temp;
		}
		output[i - 2] = windows[2];
	}
}
void FIR_Filter(double *coeffs, double *input, double *output, int length, int filterlength)
{
	double acc;
	double *coeffp;
	double *inputp;
	int n, k;
	memcpy(&InputSample[filterlength -1], input, length*sizeof(double));
	for (n = 0; n<length; n++)
	{
		coeffp = coeffs;
		inputp = &InputSample[filterlength - 1 + n];
    acc = 0;
		for ( k = 0; k < filterlength; k++ ) 
            acc += (*coeffp++) * (*inputp--);
    output[n] = acc;
	}
	memmove(&InputSample[0], &InputSample[length], (filterlength - 1)*sizeof(double));
}
char FIR_CollectData(SEN_CHANNEL Channel, double input)
{
	unsigned char i = 0;
	for (i = 0; i< (NUM_SAMPLES -1); i++)
		Input_SEN[Channel][i] = Input_SEN[Channel][i+1];
	Input_SEN[Channel][NUM_SAMPLES - 1] = input;
	return DONE;
}
double AMES_Filter(SEN_CHANNEL Channel)
{
	unsigned char i = 0;
	double Sum = 0;
	//Median_Filter(Input_SEN[Channel], Output, NUM_SAMPLES);
	FIR_Filter(coeffs, Input_SEN[Channel], Output, NUM_SAMPLES, FILTER_LEN);
	for (i = 0; i < NUM_SAMPLES; i++)
		Sum+=Output[i];
	Sum /= NUM_SAMPLES;
	return Sum;
}
