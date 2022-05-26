#include "dcc_con_base.h"
#include "math.h"
#include<Windows.h>
#include<stdio.h>

// for timer ================================================
LARGE_INTEGER nFreq;//LARGE_INTEGER��64λϵͳ����LONGLONG����32λϵͳ���Ǹߵ�����32λ��LONG����windows.h��ͨ��Ԥ�����������
LARGE_INTEGER nBeginTime;//��¼��ʼʱ�ļ�������ֵ
LARGE_INTEGER nEndTime;//��¼ֹͣʱ�ļ�������ֵ
double time_start, time_end, time_;
double j_timer = 10.0;
// for timer ================================================

/** Limitation
* InputVal: Data, Limit[min, max]
* OutputVal: DataLimited
*/
double fndLimit(double dInputData, double dLimit[2]) {
	double dOutputData;
	if (dLimit[0] == dLimit[1]) {
		return dInputData;
	}
	else {
		dOutputData = fmax(dLimit[0], dInputData);
		dOutputData = fmin(dLimit[1], dOutputData);
		return dOutputData;
	}
}
// rec dcc
/** Limitation when added to another val
* InputVal: Data, DataAddedTo, Limit[min, max]
* OutputVal: DataLimited, &dSurPassedVal
*/
double fndAddLimit(double dInputData, double dAddedData, double * dSurPassedVal, double dLimit[2]) {
	double dOutputData;
	if (dInputData + dAddedData >= dLimit[1]) {
		dOutputData = dLimit[1] - dAddedData;
	}
	else if (dInputData + dAddedData <= dLimit[0]) {
		dOutputData = dLimit[0] - dAddedData;
	}
	else dOutputData = dInputData;
	if(dSurPassedVal != NULL) *dSurPassedVal = dInputData - dOutputData;
	return dOutputData;
}
double fndThreshold(double dInVal, double dThreshold[2]) {
	double dOutVal;
	if (dInVal >= dThreshold[1]) dOutVal = dInVal - dThreshold[1];
	else if (dInVal <= dThreshold[0]) dOutVal = dInVal - dThreshold[0];
	else dOutVal = 0.0;
	return dOutVal;
}
void fnvInitInteg(IntegValLimit *strInputData) {
	strInputData->p = 0.0;
	strInputData->dp = 0.0;
	strInputData->ddp = 0.0;
}
/** Interation acc with limitation in default struct
* InputVal: DataOld[p, dp, ddp], Acc, Limits[PosMin, PosMax, VelMin, VelMax, AccMin, AccMax], CONTROL_T
* OutputVal: DataUpdated[p, dp, ddp]
*/
IntegValLimit fnstrIntegLimit(IntegValLimit strOldData, double dAcc, double dLimits[6],  double dControlT) {
	IntegValLimit strUpdatedData;
	double dPositionLimit[2];
	double dVelocityLimit[2];
	double dAccelerateLimit[2];
	for (int i = 0; i < 2; i++) {
		dPositionLimit[i] = dLimits[i];
		dVelocityLimit[i] = dLimits[i + 2];
		dAccelerateLimit[i] = dLimits[i + 4];
	}
	dAcc = fndLimit(dAcc, dAccelerateLimit);
	strUpdatedData.dp = strOldData.dp + dAcc * dControlT;
	strUpdatedData.dp = fndLimit(strUpdatedData.dp, dVelocityLimit);
	strUpdatedData.p = strOldData.p + strUpdatedData.dp * dControlT;
	strUpdatedData.p = fndLimit(strUpdatedData.p, dPositionLimit);
	strUpdatedData.dp = (strUpdatedData.p - strOldData.p) / dControlT;
	strUpdatedData.ddp = dAcc;
	return strUpdatedData;
}
/** Interation jerk with limitation
* InputVal: &PosOld, &VeloOld, &AccOld, Jerk, Limits[PosMin, PosMax, VelMin, VelMax, AccMin, AccMax], CONTROL_T
* OutputVal: &PosUpdated, &VeloUpdated, &AccUpdated
*/
void fnvJerkLimit(double *dPosIn, double *dVelIn, double *dAccIn, double dJerk, double dLimits[6], double dControlT) {
	double dPositionLimit[2];
	double dVelocityLimit[2];
	double dAccelerateLimit[2];
	double dAccOld = *dAccIn;
	double dVelOld = *dVelIn;
	double dPosOld = *dPosIn;
	double dJerkIn = dJerk;
	for (int i = 0; i < 2; i++) {
		dPositionLimit[i] = dLimits[i];
		dVelocityLimit[i] = dLimits[i + 2];
		dAccelerateLimit[i] = dLimits[i + 4];
	}
	*dAccIn = dAccOld + dJerkIn * dControlT;
	*dAccIn = fndLimit(*dAccIn, dAccelerateLimit);
	*dVelIn = dVelOld + *dAccIn * dControlT;
	*dVelIn = fndLimit(*dVelIn, dVelocityLimit);
	*dAccIn = (*dVelIn - dVelOld) / dControlT;
	*dPosIn = dPosOld + *dVelIn * dControlT;
	*dPosIn = fndLimit(*dPosIn, dPositionLimit);
	*dVelIn = (*dPosIn - dPosOld) / dControlT;
}
/** Interation acc with limitation 
* InputVal: &PosOld, &VeloOld, Acc, Limits[PosMin, PosMax, VelMin, VelMax, AccMin, AccMax], CONTROL_T
* OutputVal: &PosUpdated, &VeloUpdated
*/
void fnvIntegLimit(double *dPosIn, double *dVelIn, double dAcc, double dLimits[6], double dControlT) {
	double dPositionLimit[2];
	double dVelocityLimit[2];
	double dAccelerateLimit[2];
	double dVelOld = *dVelIn;
	double dPosOld = *dPosIn;
	double dAccIn = dAcc;
	for (int i = 0; i < 2; i++) {
		dPositionLimit[i] = dLimits[i];
		dVelocityLimit[i] = dLimits[i + 2];
		dAccelerateLimit[i] = dLimits[i + 4];
	}
	dAccIn = fndLimit(dAcc, dAccelerateLimit);
	*dVelIn = dVelOld + dAccIn * dControlT;
	*dVelIn = fndLimit(*dVelIn, dVelocityLimit);
	*dPosIn = dPosOld + *dVelIn * dControlT;
	*dPosIn = fndLimit(*dPosIn, dPositionLimit);
	*dVelIn = (*dPosIn - dPosOld) / dControlT;
}
/** Interation velo with limitation
* InputVal: &PosOld, Velo, Limits[PosMin, PosMax, VelMin, VelMax], CONTROL_T
* OutputVal: &PosUpdated
*/
void fnvVeloLimit(double *dPosIn, double dVel, double dLimits[4], double dControlT) {
	double dPositionLimit[2];
	double dVelocityLimit[2];
	double dPosOld = *dPosIn;
	double dVelIn = dVel;
	for (int i = 0; i < 2; i++) {
		dPositionLimit[i] = dLimits[i];
		dVelocityLimit[i] = dLimits[i + 2];
	}
	dVelIn = fndLimit(dVel, dVelocityLimit);
	*dPosIn = dPosOld + dVelIn * dControlT;
	*dPosIn = fndLimit(*dPosIn, dPositionLimit);
}
/** TimeLag filter
* InputVal: DataOld, DataIn, CONTROL_T, LagT
* OutputVal: &DataUpdated
*/
double fndFilterTimeLag(double filtered_in, double data_in, double control_t, double Lag_T)
{
	double filtered_out;
	filtered_out = (control_t * data_in + Lag_T * filtered_in) / (control_t + Lag_T);
	return filtered_out;
}
/** 5th Spline
* InputVal: x0, v0, a0, t0, x1, v1, a1, t1, CONTROL_T, ModeFlag
* OutputVal: &DataOut
*/
void fnvFifthSpline(double *pos_out, int maxProgN, double x0, double v0, double a0, double t0, double x1, double v1, double a1, double t1, double control_t, char mode_flag)
{
	double b_spline[6] = { 0 };
	b_spline[0] = (1.0 / pow(t0 - t1, 5.0)*((t0*t0*t0*t0*t0)*x1*2.0 - (t1*t1*t1*t1*t1)*x0*2.0 + t0*(t1*t1*t1*t1*t1)*v0*2.0 - (t0*t0*t0*t0*t0)*t1*v1*2.0 + t0*(t1*t1*t1*t1)*x0*1.0E+1 - (t0*t0*t0*t0)*t1*x1*1.0E+1 - a0*(t0*t0)*(t1*t1*t1*t1*t1) + a0*(t0*t0*t0)*(t1*t1*t1*t1)*2.0 - a0*(t0*t0*t0*t0)*(t1*t1*t1) + a1*(t0*t0*t0)*(t1*t1*t1*t1) - a1*(t0*t0*t0*t0)*(t1*t1*t1)*2.0 + a1*(t0*t0*t0*t0*t0)*(t1*t1) - (t0*t0)*(t1*t1*t1*t1)*v0*1.0E+1 + (t0*t0*t0)*(t1*t1*t1)*v0*8.0 - (t0*t0*t0)*(t1*t1*t1)*v1*8.0 + (t0*t0*t0*t0)*(t1*t1)*v1*1.0E+1 - (t0*t0)*(t1*t1*t1)*x0*2.0E+1 + (t0*t0*t0)*(t1*t1)*x1*2.0E+1)) / 2.0;
	b_spline[1] = (1.0 / pow(t0 - t1, 5.0)*((t0*t0*t0*t0*t0)*v1*2.0 - (t1*t1*t1*t1*t1)*v0*2.0 + a0*t0*(t1*t1*t1*t1*t1)*2.0 - a1*(t0*t0*t0*t0*t0)*t1*2.0 + t0*(t1*t1*t1*t1)*v0*1.0E+1 - (t0*t0*t0*t0)*t1*v1*1.0E+1 - a0*(t0*t0)*(t1*t1*t1*t1) - a0*(t0*t0*t0)*(t1*t1*t1)*4.0 + a0*(t0*t0*t0*t0)*(t1*t1)*3.0 - a1*(t0*t0)*(t1*t1*t1*t1)*3.0 + a1*(t0*t0*t0)*(t1*t1*t1)*4.0 + a1*(t0*t0*t0*t0)*(t1*t1) + (t0*t0)*(t1*t1*t1)*v0*1.6E+1 - (t0*t0*t0)*(t1*t1)*v0*2.4E+1 + (t0*t0)*(t1*t1*t1)*v1*2.4E+1 - (t0*t0*t0)*(t1*t1)*v1*1.6E+1 + (t0*t0)*(t1*t1)*x0*6.0E+1 - (t0*t0)*(t1*t1)*x1*6.0E+1)) / 2.0;
	b_spline[2] = 1.0 / pow(t0 - t1, 5.0)*(a0*(t1*t1*t1*t1*t1) - a1*(t0*t0*t0*t0*t0) + a0*t0*(t1*t1*t1*t1)*4.0 + a0*(t0*t0*t0*t0)*t1*3.0 - a1*t0*(t1*t1*t1*t1)*3.0 - a1*(t0*t0*t0*t0)*t1*4.0 + t0*(t1*t1*t1)*v0*3.6E+1 - (t0*t0*t0)*t1*v0*2.4E+1 + t0*(t1*t1*t1)*v1*2.4E+1 - (t0*t0*t0)*t1*v1*3.6E+1 + t0*(t1*t1)*x0*6.0E+1 + (t0*t0)*t1*x0*6.0E+1 - t0*(t1*t1)*x1*6.0E+1 - (t0*t0)*t1*x1*6.0E+1 - a0*(t0*t0)*(t1*t1*t1)*8.0 + a1*(t0*t0*t0)*(t1*t1)*8.0 - (t0*t0)*(t1*t1)*v0*1.2E+1 + (t0*t0)*(t1*t1)*v1*1.2E+1)*(-1.0 / 2.0);
	b_spline[3] = (1.0 / pow(t0 - t1, 5.0)*(a0*(t0*t0*t0*t0) + a0*(t1*t1*t1*t1)*3.0 - a1*(t0*t0*t0*t0)*3.0 - a1*(t1*t1*t1*t1) - (t0*t0*t0)*v0*8.0 - (t0*t0*t0)*v1*1.2E+1 + (t1*t1*t1)*v0*1.2E+1 + (t1*t1*t1)*v1*8.0 + (t0*t0)*x0*2.0E+1 - (t0*t0)*x1*2.0E+1 + (t1*t1)*x0*2.0E+1 - (t1*t1)*x1*2.0E+1 + a0*(t0*t0*t0)*t1*4.0 - a1*t0*(t1*t1*t1)*4.0 + t0*(t1*t1)*v0*2.8E+1 - (t0*t0)*t1*v0*3.2E+1 + t0*(t1*t1)*v1*3.2E+1 - (t0*t0)*t1*v1*2.8E+1 - a0*(t0*t0)*(t1*t1)*8.0 + a1*(t0*t0)*(t1*t1)*8.0 + t0*t1*x0*8.0E+1 - t0*t1*x1*8.0E+1)) / 2.0;
	b_spline[4] = 1.0 / pow(t0 - t1, 5.0)*(t0*x0*3.0E+1 - t0*x1*3.0E+1 + t1*x0*3.0E+1 - t1*x1*3.0E+1 + a0*(t0*t0*t0)*2.0 + a0*(t1*t1*t1)*3.0 - a1*(t0*t0*t0)*3.0 - a1*(t1*t1*t1)*2.0 - (t0*t0)*v0*1.4E+1 - (t0*t0)*v1*1.6E+1 + (t1*t1)*v0*1.6E+1 + (t1*t1)*v1*1.4E+1 - a0*t0*(t1*t1)*4.0 - a0*(t0*t0)*t1 + a1*t0*(t1*t1) + a1*(t0*t0)*t1*4.0 - t0*t1*v0*2.0 + t0*t1*v1*2.0)*(-1.0 / 2.0);
	b_spline[5] = (1.0 / pow(t0 - t1, 5.0)*(x0*1.2E+1 - x1*1.2E+1 - t0*v0*6.0 - t0*v1*6.0 + t1*v0*6.0 + t1*v1*6.0 + a0*(t0*t0) + a0*(t1*t1) - a1*(t0*t0) - a1*(t1*t1) - a0*t0*t1*2.0 + a1*t0*t1*2.0)) / 2.0;
	int k0 = (int)floor(t0 / control_t + 1e-8);
	int k1 = (int)floor(t1 / control_t + 1e-8);

	if (mode_flag == 'T') { // traditional: start from k = 0
		int k = 0;
		for (int i = k0; i < k1; i++) {
			pos_out[k] = b_spline[0] + b_spline[1] * i * control_t + b_spline[2] * i * control_t * i * control_t + b_spline[3] * i * control_t * i * control_t * i * control_t + b_spline[4] * i * control_t * i * control_t * i * control_t * i * control_t + b_spline[5] * i * control_t * i * control_t * i * control_t * i * control_t * i * control_t;
			k++;
		}
		for (int i = k1; i < maxProgN + k0; i++) {
			pos_out[k] = pos_out[k1 - k0 - 2];
			k++;
		}
	}
	else if (mode_flag == 'N') { // novel: start from k = k0
		for (int i = k0; i < k1; i++) {
			pos_out[i] = b_spline[0] + b_spline[1] * i * control_t + b_spline[2] * i * control_t * i * control_t + b_spline[3] * i * control_t * i * control_t * i * control_t + b_spline[4] * i * control_t * i * control_t * i * control_t * i * control_t + b_spline[5] * i * control_t * i * control_t * i * control_t * i * control_t * i * control_t;
		}
		for (int i = k1; i < maxProgN; i++) {
			pos_out[i] = pos_out[k1 - 1];
		}
	}
	if (x0 == x1 && v0 == v1 && a0 == a1) {
		for (int i = k0; i < maxProgN; i++) {
			pos_out[i] = x0;
		}
	}
}
/** dou4th Spline
* InputVal: x0, v0, a0, t0, x1, t1, x2, v2, a2, t2, CONTROL_T, ModeFlag
* OutputVal: &DataOut
*/
void fnvDouFourthSpline(double *pos_out, int maxProgN, double x0, double v0, double a0, double t0, double x1, double t1, double x2, double v2, double a2, double t2, double control_t, char mode_flag)
{
	double b_spline[10] = { 0 };
	double v1 = 0.0;
	b_spline[0] = ((t0*t0*t0*t0)*x1*2.0 + (t1*t1*t1*t1)*x0*2.0 - t0*(t1*t1*t1*t1)*v0*2.0 - (t0*t0*t0*t0)*t1*v1*2.0 - t0*(t1*t1*t1)*x0*8.0 - (t0*t0*t0)*t1*x1*8.0 + a0*(t0*t0)*(t1*t1*t1*t1) - a0*(t0*t0*t0)*(t1*t1*t1)*2.0 + a0*(t0*t0*t0*t0)*(t1*t1) + (t0*t0)*(t1*t1*t1)*v0*8.0 - (t0*t0*t0)*(t1*t1)*v0*6.0 + (t0*t0*t0)*(t1*t1)*v1*2.0 + (t0*t0)*(t1*t1)*x0*1.2E+1) / ((t0 - t1)*(t0*(t1*t1)*3.0 - (t0*t0)*t1*3.0 + t0*t0*t0 - t1*t1*t1)*2.0);
	b_spline[1] = ((t0*t0*t0*t0)*v1 + (t1*t1*t1*t1)*v0 - a0*t0*(t1*t1*t1*t1) - a0*(t0*t0*t0*t0)*t1 - t0*(t1*t1*t1)*v0*4.0 + (t0*t0*t0)*t1*v0*6.0 + (t0*t0*t0)*t1*v1*2.0 - (t0*t0)*t1*x0*1.2E+1 + (t0*t0)*t1*x1*1.2E+1 + a0*(t0*t0)*(t1*t1*t1) + a0*(t0*t0*t0)*(t1*t1) - (t0*t0)*(t1*t1)*v0*3.0 - (t0*t0)*(t1*t1)*v1*3.0) / ((t0 - t1)*(t0*(t1*t1)*3.0 - (t0*t0)*t1*3.0 + t0*t0*t0 - t1*t1*t1));
	b_spline[2] = (a0*(t0*t0*t0*t0) + a0*(t1*t1*t1*t1) - (t0*t0*t0)*v0*6.0 - (t0*t0*t0)*v1*6.0 + (t0*t0)*x0*1.2E+1 - (t0*t0)*x1*1.2E+1 + a0*t0*(t1*t1*t1)*2.0 + a0*(t0*t0*t0)*t1*2.0 + t0*(t1*t1)*v0*1.8E+1 - (t0*t0)*t1*v0*1.2E+1 + t0*(t1*t1)*v1*6.0 - a0*(t0*t0)*(t1*t1)*6.0 + t0*t1*x0*2.4E+1 - t0*t1*x1*2.4E+1) / ((t0 - t1)*(t0*(t1*t1)*3.0 - (t0*t0)*t1*3.0 + t0*t0*t0 - t1*t1*t1)*2.0);
	b_spline[3] = -(t0*x0*8.0 - t0*x1*8.0 + t1*x0*4.0 - t1*x1*4.0 + a0*(t0*t0*t0) + a0*(t1*t1*t1) - (t0*t0)*v0*5.0 - (t0*t0)*v1*3.0 + (t1*t1)*v0*3.0 + (t1*t1)*v1 - a0*t0*(t1*t1) - a0*(t0*t0)*t1 + t0*t1*v0*2.0 + t0*t1*v1*2.0) / ((t0 - t1)*(t0*(t1*t1)*3.0 - (t0*t0)*t1*3.0 + t0*t0*t0 - t1*t1*t1));
	b_spline[4] = (x0*6.0 - x1*6.0 - t0*v0*4.0 - t0*v1*2.0 + t1*v0*4.0 + t1*v1*2.0 + a0*(t0*t0) + a0*(t1*t1) - a0*t0*t1*2.0) / ((t0 - t1)*(t0*(t1*t1)*3.0 - (t0*t0)*t1*3.0 + t0*t0*t0 - t1*t1*t1)*2.0); 
	b_spline[5] = ((t1*t1*t1*t1)*x2*2.0 + (t2*t2*t2*t2)*x1*2.0 - t1*(t2*t2*t2*t2)*v1*2.0 - (t1*t1*t1*t1)*t2*v2*2.0 - t1*(t2*t2*t2)*x1*8.0 - (t1*t1*t1)*t2*x2*8.0 + a2*(t1*t1)*(t2*t2*t2*t2) - a2*(t1*t1*t1)*(t2*t2*t2)*2.0 + a2*(t1*t1*t1*t1)*(t2*t2) + (t1*t1)*(t2*t2*t2)*v1*2.0 - (t1*t1)*(t2*t2*t2)*v2*6.0 + (t1*t1*t1)*(t2*t2)*v2*8.0 + (t1*t1)*(t2*t2)*x2*1.2E+1) / ((t1 - t2)*(t1*(t2*t2)*3.0 - (t1*t1)*t2*3.0 + t1*t1*t1 - t2*t2*t2)*2.0);
	b_spline[6] = ((t1*t1*t1*t1)*v2 + (t2*t2*t2*t2)*v1 - a2*t1*(t2*t2*t2*t2) - a2*(t1*t1*t1*t1)*t2 + t1*(t2*t2*t2)*v1*2.0 + t1*(t2*t2*t2)*v2*6.0 - (t1*t1*t1)*t2*v2*4.0 + t1*(t2*t2)*x1*1.2E+1 - t1*(t2*t2)*x2*1.2E+1 + a2*(t1*t1)*(t2*t2*t2) + a2*(t1*t1*t1)*(t2*t2) - (t1*t1)*(t2*t2)*v1*3.0 - (t1*t1)*(t2*t2)*v2*3.0) / ((t1 - t2)*(t1*(t2*t2)*3.0 - (t1*t1)*t2*3.0 + t1*t1*t1 - t2*t2*t2));
	b_spline[7] = (a2*(t1*t1*t1*t1) + a2*(t2*t2*t2*t2) - (t2*t2*t2)*v1*6.0 - (t2*t2*t2)*v2*6.0 - (t2*t2)*x1*1.2E+1 + (t2*t2)*x2*1.2E+1 + a2*t1*(t2*t2*t2)*2.0 + a2*(t1*t1*t1)*t2*2.0 + (t1*t1)*t2*v1*6.0 - t1*(t2*t2)*v2*1.2E+1 + (t1*t1)*t2*v2*1.8E+1 - a2*(t1*t1)*(t2*t2)*6.0 - t1*t2*x1*2.4E+1 + t1*t2*x2*2.4E+1) / ((t1 - t2)*(t1*(t2*t2)*3.0 - (t1*t1)*t2*3.0 + t1*t1*t1 - t2*t2*t2)*2.0);
	b_spline[8] = -(t1*x1*-4.0 + t1*x2*4.0 - t2*x1*8.0 + t2*x2*8.0 + a2*(t1*t1*t1) + a2*(t2*t2*t2) + (t1*t1)*v1 + (t1*t1)*v2*3.0 - (t2*t2)*v1*3.0 - (t2*t2)*v2*5.0 - a2*t1*(t2*t2) - a2*(t1*t1)*t2 + t1*t2*v1*2.0 + t1*t2*v2*2.0) / ((t1 - t2)*(t1*(t2*t2)*3.0 - (t1*t1)*t2*3.0 + t1*t1*t1 - t2*t2*t2));
	b_spline[9] = (x1*-6.0 + x2*6.0 + t1*v1*2.0 + t1*v2*4.0 - t2*v1*2.0 - t2*v2*4.0 + a2*(t1*t1) + a2*(t2*t2) - a2*t1*t2*2.0) / ((t1 - t2)*(t1*(t2*t2)*3.0 - (t1*t1)*t2*3.0 + t1*t1*t1 - t2*t2*t2)*2.0); 
	int k0 = (int)floor(t0 / control_t + 1e-8);
	int k1 = (int)floor(t1 / control_t + 1e-8);
	int k2 = (int)floor(t2 / control_t + 1e-8);

	if (mode_flag == 'T') { // traditional: start from k = 0
		int k = 0;
		for (int i = k0; i < k1; i++) {
			pos_out[k] = b_spline[0] + b_spline[1] * i * control_t + b_spline[2] * i * control_t * i * control_t + b_spline[3] * i * control_t * i * control_t * i * control_t + b_spline[4] * i * control_t * i * control_t * i * control_t * i * control_t;
			k++;
		}
		for (int i = k1; i < k2; i++) {
			pos_out[k] = b_spline[5] + b_spline[6] * i * control_t + b_spline[7] * i * control_t * i * control_t + b_spline[8] * i * control_t * i * control_t * i * control_t + b_spline[9] * i * control_t * i * control_t * i * control_t * i * control_t;
			k++;
		}
		for (int i = k2; i < maxProgN + k0; i++) {
			pos_out[k] = pos_out[k2 - k0 - 1];
			k++;
		}
	}
	else if (mode_flag == 'N') { // novel: start from k = k0
		for (int i = k0; i < k1; i++) {
			pos_out[i] = b_spline[0] + b_spline[1] * i * control_t + b_spline[2] * i * control_t * i * control_t + b_spline[3] * i * control_t * i * control_t * i * control_t + b_spline[4] * i * control_t * i * control_t * i * control_t * i * control_t;
		}
		for (int i = k1; i < k2; i++) {
			pos_out[i] = b_spline[5] + b_spline[6] * i * control_t + b_spline[7] * i * control_t * i * control_t + b_spline[8] * i * control_t * i * control_t * i * control_t + b_spline[9] * i * control_t * i * control_t * i * control_t * i * control_t;
		}
		for (int i = k2; i < maxProgN; i++) {
			pos_out[i] = pos_out[k2 - 1];
		}
	}
	if (x0 == x1 && x1 == x2) {
		for (int i = k0; i < maxProgN; i++) {
			pos_out[i] = x0;
		}
	}
}
/** Easy Spline: from the last position of the data to the final position with no started & ended velo & acc
* InputVal: maxProgN(maxmun steps of the whole program), t0, t1, x1, CONTROL_T
* OutputVal: &DataOut
*/
void fnvEzSpline(double *pos_out, int maxProgN, double t0_in, double t1_in, double x1_in, double control_t)
{
	// int according to control period
	int k0 = (int)floor(t0_in / control_t + 1e-8);
	int k1 = (int)floor(t1_in / control_t + 1e-8);

	// map to 0s (error would be remarkable if time is too large)
	double t0 = 0.0;
	double t1 = (k1 - k0) * control_t;

	double b_spline[6] = { 0 };
	double x0 = pos_out[k0];
	double v0 = 0.0;
	double a0 = 0.0;
	double x1 = x1_in;
	double v1 = 0.0;
	double a1 = 0.0;
	double tempt = 0.0;
	int tempk = 0;
	if (k1 == k0) { // jump
		for (int i = k0; i < maxProgN; i++) {
			pos_out[i] = x1;
		}
	}
	else { // spline to x1
		b_spline[0] = (1.0 / pow(t0 - t1, 5.0)*((t0*t0*t0*t0*t0)*x1*2.0 - (t1*t1*t1*t1*t1)*x0*2.0 + t0*(t1*t1*t1*t1*t1)*v0*2.0 - (t0*t0*t0*t0*t0)*t1*v1*2.0 + t0*(t1*t1*t1*t1)*x0*1.0E+1 - (t0*t0*t0*t0)*t1*x1*1.0E+1 - a0*(t0*t0)*(t1*t1*t1*t1*t1) + a0*(t0*t0*t0)*(t1*t1*t1*t1)*2.0 - a0*(t0*t0*t0*t0)*(t1*t1*t1) + a1*(t0*t0*t0)*(t1*t1*t1*t1) - a1*(t0*t0*t0*t0)*(t1*t1*t1)*2.0 + a1*(t0*t0*t0*t0*t0)*(t1*t1) - (t0*t0)*(t1*t1*t1*t1)*v0*1.0E+1 + (t0*t0*t0)*(t1*t1*t1)*v0*8.0 - (t0*t0*t0)*(t1*t1*t1)*v1*8.0 + (t0*t0*t0*t0)*(t1*t1)*v1*1.0E+1 - (t0*t0)*(t1*t1*t1)*x0*2.0E+1 + (t0*t0*t0)*(t1*t1)*x1*2.0E+1)) / 2.0;
		b_spline[1] = (1.0 / pow(t0 - t1, 5.0)*((t0*t0*t0*t0*t0)*v1*2.0 - (t1*t1*t1*t1*t1)*v0*2.0 + a0*t0*(t1*t1*t1*t1*t1)*2.0 - a1*(t0*t0*t0*t0*t0)*t1*2.0 + t0*(t1*t1*t1*t1)*v0*1.0E+1 - (t0*t0*t0*t0)*t1*v1*1.0E+1 - a0*(t0*t0)*(t1*t1*t1*t1) - a0*(t0*t0*t0)*(t1*t1*t1)*4.0 + a0*(t0*t0*t0*t0)*(t1*t1)*3.0 - a1*(t0*t0)*(t1*t1*t1*t1)*3.0 + a1*(t0*t0*t0)*(t1*t1*t1)*4.0 + a1*(t0*t0*t0*t0)*(t1*t1) + (t0*t0)*(t1*t1*t1)*v0*1.6E+1 - (t0*t0*t0)*(t1*t1)*v0*2.4E+1 + (t0*t0)*(t1*t1*t1)*v1*2.4E+1 - (t0*t0*t0)*(t1*t1)*v1*1.6E+1 + (t0*t0)*(t1*t1)*x0*6.0E+1 - (t0*t0)*(t1*t1)*x1*6.0E+1)) / 2.0;
		b_spline[2] = 1.0 / pow(t0 - t1, 5.0)*(a0*(t1*t1*t1*t1*t1) - a1*(t0*t0*t0*t0*t0) + a0*t0*(t1*t1*t1*t1)*4.0 + a0*(t0*t0*t0*t0)*t1*3.0 - a1*t0*(t1*t1*t1*t1)*3.0 - a1*(t0*t0*t0*t0)*t1*4.0 + t0*(t1*t1*t1)*v0*3.6E+1 - (t0*t0*t0)*t1*v0*2.4E+1 + t0*(t1*t1*t1)*v1*2.4E+1 - (t0*t0*t0)*t1*v1*3.6E+1 + t0*(t1*t1)*x0*6.0E+1 + (t0*t0)*t1*x0*6.0E+1 - t0*(t1*t1)*x1*6.0E+1 - (t0*t0)*t1*x1*6.0E+1 - a0*(t0*t0)*(t1*t1*t1)*8.0 + a1*(t0*t0*t0)*(t1*t1)*8.0 - (t0*t0)*(t1*t1)*v0*1.2E+1 + (t0*t0)*(t1*t1)*v1*1.2E+1)*(-1.0 / 2.0);
		b_spline[3] = (1.0 / pow(t0 - t1, 5.0)*(a0*(t0*t0*t0*t0) + a0*(t1*t1*t1*t1)*3.0 - a1*(t0*t0*t0*t0)*3.0 - a1*(t1*t1*t1*t1) - (t0*t0*t0)*v0*8.0 - (t0*t0*t0)*v1*1.2E+1 + (t1*t1*t1)*v0*1.2E+1 + (t1*t1*t1)*v1*8.0 + (t0*t0)*x0*2.0E+1 - (t0*t0)*x1*2.0E+1 + (t1*t1)*x0*2.0E+1 - (t1*t1)*x1*2.0E+1 + a0*(t0*t0*t0)*t1*4.0 - a1*t0*(t1*t1*t1)*4.0 + t0*(t1*t1)*v0*2.8E+1 - (t0*t0)*t1*v0*3.2E+1 + t0*(t1*t1)*v1*3.2E+1 - (t0*t0)*t1*v1*2.8E+1 - a0*(t0*t0)*(t1*t1)*8.0 + a1*(t0*t0)*(t1*t1)*8.0 + t0*t1*x0*8.0E+1 - t0*t1*x1*8.0E+1)) / 2.0;
		b_spline[4] = 1.0 / pow(t0 - t1, 5.0)*(t0*x0*3.0E+1 - t0*x1*3.0E+1 + t1*x0*3.0E+1 - t1*x1*3.0E+1 + a0*(t0*t0*t0)*2.0 + a0*(t1*t1*t1)*3.0 - a1*(t0*t0*t0)*3.0 - a1*(t1*t1*t1)*2.0 - (t0*t0)*v0*1.4E+1 - (t0*t0)*v1*1.6E+1 + (t1*t1)*v0*1.6E+1 + (t1*t1)*v1*1.4E+1 - a0*t0*(t1*t1)*4.0 - a0*(t0*t0)*t1 + a1*t0*(t1*t1) + a1*(t0*t0)*t1*4.0 - t0*t1*v0*2.0 + t0*t1*v1*2.0)*(-1.0 / 2.0);
		b_spline[5] = (1.0 / pow(t0 - t1, 5.0)*(x0*1.2E+1 - x1*1.2E+1 - t0*v0*6.0 - t0*v1*6.0 + t1*v0*6.0 + t1*v1*6.0 + a0*(t0*t0) + a0*(t1*t1) - a1*(t0*t0) - a1*(t1*t1) - a0*t0*t1*2.0 + a1*t0*t1*2.0)) / 2.0;

		for (int i = k0; i < k1; i++) { // spline
			tempt = tempk * control_t;
			pos_out[i] = b_spline[0] + b_spline[1] * tempt + b_spline[2] * tempt * tempt + b_spline[3] * tempt * tempt * tempt + b_spline[4] * tempt * tempt * tempt * tempt + b_spline[5] * tempt * tempt * tempt * tempt * tempt;
			tempk++;
		}
		for (int i = k1; i < maxProgN; i++) { // hold on
			pos_out[i] = pos_out[k1 - 1];
		}
	}
}

void fnvTic() {
	QueryPerformanceFrequency(&nFreq); // ��ȡϵͳʱ��Ƶ��
	QueryPerformanceCounter(&nBeginTime); // ��ȡ��ʼʱ�̼���ֵ
}

double fnvToc(int nDispTimeFlag) {
	QueryPerformanceCounter(&nEndTime); // ��ȡֹͣʱ�̼���ֵ
	time_end = ((double)(nEndTime.QuadPart) / (double)nFreq.QuadPart);
	time_start = ((double)(nBeginTime.QuadPart) / (double)nFreq.QuadPart);
	time_ = time_end - time_start; //����ʼ-ֹͣ��/Ƶ�ʼ�Ϊ��������ȷ��С�����6λ
	if (nDispTimeFlag == 1) printf("Time = %lf\n", time_);
	return time_;
}

int fnnSlowDown(double * dptDataIn, int nDataLen, int nSlowTimes, double * dptDataOut) {
	int nKout = 0;
	for (int i = 0; i < nDataLen - 2; i++) {
		double dIncTemp = (*(dptDataIn + i + 1) - *(dptDataIn + i)) / ((double)nSlowTimes);
		for (int j = 0; j < nSlowTimes; j++) *(dptDataOut + nKout) = *(dptDataIn + i) + (double)j * dIncTemp, nKout++;
	}
	return nKout;
}

int fnnRdFile(char * sFileName, int nRow, int nCol, double * dDataName) {
	FILE * Ftp;
	double dDataTemp;
	if ((Ftp = fopen(sFileName, "r")) == NULL) {
		printf("Can't open file %s !\n", sFileName);
		return 0;
	}
	for (int i = 0; i < nRow; i++) for (int j = 0; j < nCol; j++) fscanf(Ftp, "%lf", &dDataTemp), *(dDataName + i * nCol + j) = dDataTemp;
	fclose(Ftp);
	return 1;
}

void fnvWtFile(char * sFileName, int nRow, int nCol, double * dDataName) {
	FILE * Ftp = fopen(sFileName, "w");
	double dDataTemp;
	for (int i = 0; i < nRow; i++) {
		for (int j = 0; j < nCol; j++) dDataTemp = *(dDataName + i * nCol + j), fprintf(Ftp, "%lf\t", dDataTemp);
		fprintf(Ftp, "\n");
	}
	fclose(Ftp);
}

void fnvClearPosition(dccPositional *stData) {
	for(int i = 0; i < 9; i++) *(&(stData->x) + i) = 0.0;
}

double fndAbsWeight(double dDataIn1, double dDataIn2, char cMode) {
	if(cMode == 'B') { // choose the bigger one 
		if(fabs(dDataIn1) >= fabs(dDataIn2)) return dDataIn1;
		else if(fabs(dDataIn1) < fabs(dDataIn2)) return dDataIn2;
		else {
			printf("Wrong in fndAbsWeight1!\n");
			return 0;
		}
	}
	else if(cMode == 'S') { // choose the smaller one 
		if(fabs(dDataIn1) <= fabs(dDataIn2)) return dDataIn1;
		else if(fabs(dDataIn1) > fabs(dDataIn2)) return dDataIn2;
		else {
			printf("Wrong in fndAbsWeight2!\n");
			return 0;
		}
	}
	else {
		printf("Wrong in fndAbsWeight3!\n");
		return 0;
	}
}

double fndGetSign(double dDataIn) {
	if(fabs(dDataIn) > 1e-6) return (dDataIn / fabs(dDataIn));
	else return 1.0;
}

double fndAbsLimit(double dDataIn, double dAbsLimit[2]/*should be active value*/) {
	double dDataOut = dDataIn;
	if(fabs(dDataIn) < dAbsLimit[0]) dDataOut = fndGetSign(dDataIn) * dAbsLimit[0];
	if(fabs(dDataIn) > dAbsLimit[1]) dDataOut = fndGetSign(dDataIn) * dAbsLimit[1];
	return dDataOut;
}

double fndActivate(double dDataIn, double dTrigger, char cMode) {
	if(cMode == 'B') { // bigger trigger
		if(dDataIn > dTrigger) return 1.0;
		else return 0.0;
	}
	else if(cMode == 'S') { // smaller trigger
		if(dDataIn < dTrigger) return 1.0;
		else return 0.0;
	}
	else {
		printf("Wrong in fndActivate!\n");
		return 0.0;
	}
}

void fnvLinearC2D(double * dAc, double * dBc, double * dAd, double *dBd, double dT, int nDim, int nInputNum) {
	for(int i = 0; i < nDim; i++) {
		for(int j = 0; j < nDim; j++) *(dAd + i * nDim + j) = dT * *(dAc + i * nDim + j) + (double)(i == j);
		for(int k = 0; k < nInputNum; k++) *(dBd + i * nInputNum + k) = dT * *(dBc + i * nInputNum + k);
	}
}

/** 5th Spline with position, velocity, acceleration output
* InputVal: x0, v0, a0, t0, x1, v1, a1, t1, CONTROL_T, ModeFlag
* OutputVal: &DataOut
*/
void fnvFifthSplineOutputPVA(double *pos_out, double *vel_out, double *acc_out, int maxProgN, double x0, double v0, double a0, double t0, double x1, double v1, double a1, double t1, double control_t)
{
	double b_spline[6] = { 0 };
	b_spline[0] = (1.0 / pow(t0 - t1, 5.0)*((t0*t0*t0*t0*t0)*x1*2.0 - (t1*t1*t1*t1*t1)*x0*2.0 + t0*(t1*t1*t1*t1*t1)*v0*2.0 - (t0*t0*t0*t0*t0)*t1*v1*2.0 + t0*(t1*t1*t1*t1)*x0*1.0E+1 - (t0*t0*t0*t0)*t1*x1*1.0E+1 - a0*(t0*t0)*(t1*t1*t1*t1*t1) + a0*(t0*t0*t0)*(t1*t1*t1*t1)*2.0 - a0*(t0*t0*t0*t0)*(t1*t1*t1) + a1*(t0*t0*t0)*(t1*t1*t1*t1) - a1*(t0*t0*t0*t0)*(t1*t1*t1)*2.0 + a1*(t0*t0*t0*t0*t0)*(t1*t1) - (t0*t0)*(t1*t1*t1*t1)*v0*1.0E+1 + (t0*t0*t0)*(t1*t1*t1)*v0*8.0 - (t0*t0*t0)*(t1*t1*t1)*v1*8.0 + (t0*t0*t0*t0)*(t1*t1)*v1*1.0E+1 - (t0*t0)*(t1*t1*t1)*x0*2.0E+1 + (t0*t0*t0)*(t1*t1)*x1*2.0E+1)) / 2.0;
	b_spline[1] = (1.0 / pow(t0 - t1, 5.0)*((t0*t0*t0*t0*t0)*v1*2.0 - (t1*t1*t1*t1*t1)*v0*2.0 + a0*t0*(t1*t1*t1*t1*t1)*2.0 - a1*(t0*t0*t0*t0*t0)*t1*2.0 + t0*(t1*t1*t1*t1)*v0*1.0E+1 - (t0*t0*t0*t0)*t1*v1*1.0E+1 - a0*(t0*t0)*(t1*t1*t1*t1) - a0*(t0*t0*t0)*(t1*t1*t1)*4.0 + a0*(t0*t0*t0*t0)*(t1*t1)*3.0 - a1*(t0*t0)*(t1*t1*t1*t1)*3.0 + a1*(t0*t0*t0)*(t1*t1*t1)*4.0 + a1*(t0*t0*t0*t0)*(t1*t1) + (t0*t0)*(t1*t1*t1)*v0*1.6E+1 - (t0*t0*t0)*(t1*t1)*v0*2.4E+1 + (t0*t0)*(t1*t1*t1)*v1*2.4E+1 - (t0*t0*t0)*(t1*t1)*v1*1.6E+1 + (t0*t0)*(t1*t1)*x0*6.0E+1 - (t0*t0)*(t1*t1)*x1*6.0E+1)) / 2.0;
	b_spline[2] = 1.0 / pow(t0 - t1, 5.0)*(a0*(t1*t1*t1*t1*t1) - a1*(t0*t0*t0*t0*t0) + a0*t0*(t1*t1*t1*t1)*4.0 + a0*(t0*t0*t0*t0)*t1*3.0 - a1*t0*(t1*t1*t1*t1)*3.0 - a1*(t0*t0*t0*t0)*t1*4.0 + t0*(t1*t1*t1)*v0*3.6E+1 - (t0*t0*t0)*t1*v0*2.4E+1 + t0*(t1*t1*t1)*v1*2.4E+1 - (t0*t0*t0)*t1*v1*3.6E+1 + t0*(t1*t1)*x0*6.0E+1 + (t0*t0)*t1*x0*6.0E+1 - t0*(t1*t1)*x1*6.0E+1 - (t0*t0)*t1*x1*6.0E+1 - a0*(t0*t0)*(t1*t1*t1)*8.0 + a1*(t0*t0*t0)*(t1*t1)*8.0 - (t0*t0)*(t1*t1)*v0*1.2E+1 + (t0*t0)*(t1*t1)*v1*1.2E+1)*(-1.0 / 2.0);
	b_spline[3] = (1.0 / pow(t0 - t1, 5.0)*(a0*(t0*t0*t0*t0) + a0*(t1*t1*t1*t1)*3.0 - a1*(t0*t0*t0*t0)*3.0 - a1*(t1*t1*t1*t1) - (t0*t0*t0)*v0*8.0 - (t0*t0*t0)*v1*1.2E+1 + (t1*t1*t1)*v0*1.2E+1 + (t1*t1*t1)*v1*8.0 + (t0*t0)*x0*2.0E+1 - (t0*t0)*x1*2.0E+1 + (t1*t1)*x0*2.0E+1 - (t1*t1)*x1*2.0E+1 + a0*(t0*t0*t0)*t1*4.0 - a1*t0*(t1*t1*t1)*4.0 + t0*(t1*t1)*v0*2.8E+1 - (t0*t0)*t1*v0*3.2E+1 + t0*(t1*t1)*v1*3.2E+1 - (t0*t0)*t1*v1*2.8E+1 - a0*(t0*t0)*(t1*t1)*8.0 + a1*(t0*t0)*(t1*t1)*8.0 + t0*t1*x0*8.0E+1 - t0*t1*x1*8.0E+1)) / 2.0;
	b_spline[4] = 1.0 / pow(t0 - t1, 5.0)*(t0*x0*3.0E+1 - t0*x1*3.0E+1 + t1*x0*3.0E+1 - t1*x1*3.0E+1 + a0*(t0*t0*t0)*2.0 + a0*(t1*t1*t1)*3.0 - a1*(t0*t0*t0)*3.0 - a1*(t1*t1*t1)*2.0 - (t0*t0)*v0*1.4E+1 - (t0*t0)*v1*1.6E+1 + (t1*t1)*v0*1.6E+1 + (t1*t1)*v1*1.4E+1 - a0*t0*(t1*t1)*4.0 - a0*(t0*t0)*t1 + a1*t0*(t1*t1) + a1*(t0*t0)*t1*4.0 - t0*t1*v0*2.0 + t0*t1*v1*2.0)*(-1.0 / 2.0);
	b_spline[5] = (1.0 / pow(t0 - t1, 5.0)*(x0*1.2E+1 - x1*1.2E+1 - t0*v0*6.0 - t0*v1*6.0 + t1*v0*6.0 + t1*v1*6.0 + a0*(t0*t0) + a0*(t1*t1) - a1*(t0*t0) - a1*(t1*t1) - a0*t0*t1*2.0 + a1*t0*t1*2.0)) / 2.0;
	int k0 = (int)floor(t0 / control_t + 1e-8);
	int k1 = (int)floor(t1 / control_t + 1e-8);
	int k = 0;
	for (int i = k0; i < k1; i++) {
		pos_out[k] = b_spline[0] + b_spline[1] * i * control_t + b_spline[2] * i * control_t * i * control_t + b_spline[3] * i * control_t * i * control_t * i * control_t + b_spline[4] * i * control_t * i * control_t * i * control_t * i * control_t + b_spline[5] * i * control_t * i * control_t * i * control_t * i * control_t * i * control_t;
		vel_out[k] = b_spline[1] + 2.0 * b_spline[2] * i * control_t + 3.0 * b_spline[3] * i * control_t * i * control_t + 4.0 * b_spline[4] * i * control_t * i * control_t * i * control_t + 5.0 * b_spline[5] * i * control_t * i * control_t * i * control_t * i * control_t;
		acc_out[k] = 2.0 * b_spline[2] + 6.0 * b_spline[3] * i * control_t + 12.0 * b_spline[4] * i * control_t * i * control_t + 20.0 * b_spline[5] * i * control_t * i * control_t * i * control_t;
		k++;
	}
	for (int i = k1; i < maxProgN + k0; i++) {
		pos_out[k] = pos_out[k1 - k0 - 1];
		vel_out[k] = acc_out[k] = 0.0;
		k++;
	}
}
/** Online spline: 
* InputVal: dTimeNow (from the begining of dPosIn)
* OutputVal: &DataOut
*/
void fnvOnLineSpline(double * dPosIn, double * dVelIn, double * dAccIn, double dTimeNow, double dPosCmd, double dTimeCmd, int nKTot, double dControlT) {
	int nKNow = (int)floor(dTimeNow / dControlT), nKCmd = (int)floor(dTimeCmd / dControlT), maxProgN = nKTot - nKNow;
	double x0 = dPosIn[nKNow], v0 = dVelIn[nKNow], a0 = dAccIn[nKNow];
	fnvFifthSplineOutputPVA(dPosIn + nKNow, dVelIn + nKNow, dAccIn + nKNow, maxProgN, x0, v0, a0, dTimeNow, dPosCmd, 0.0, 0.0, dTimeCmd, dControlT);
}


