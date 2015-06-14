#include <stdlib.h>

typedef struct PID_Coff
{
	float Kp;
	float Ki;
	float Kd;
}PIDCoff;

enum{FAIL,PASS};
void PID_Init(unsigned char Channel, PIDCoff Coff);
long PID_Calculate(unsigned char Channel, double SetPoint, double InputValue);
void PID_Reset(unsigned char Channel);
unsigned char PID_Mem_Create(unsigned char NumberOfChannel);
/*
So luong kenh phai duoc khai ba truoc khi goi ham PID_Init

*/
void PID_Free_Mem(void);
/*
khi ket thuc chuong trinh nen xoa vung nho luu so kenh
*/
void PID_WindUp_Init(unsigned char Channel, float Max_Value);
