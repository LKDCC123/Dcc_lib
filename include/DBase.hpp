// basic tools based on dcc_con_base.c
// 20230303 dcc <3120195094@bit.edu.cn>
#pragma once
#ifndef DBASE_HPP
#define DBASE_HPP
#include "BaseHeaders.h"
#include <DMat.hpp>
#include <DDefs.hpp>

_D_BASE_BEGIN

// for timer ================================================
static LARGE_INTEGER nFreq;
static LARGE_INTEGER nBeginTime;
static LARGE_INTEGER nEndTime;
static double time_start, time_end, time_;
static double j_timer = 10.0;
// for timer ================================================

/**
 * @brief Limitation
 * 
 * @param dInputData    INPUT
 * @param dLimit        INPUT
 * @return double       OUTPUT
 */
inline double fndLimit(double dInputData, double dLimit[2]) {
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

/**
 * @brief Limitation when added to another val: used on moment stabilization control
 * 
 * @param dInputData    INPUT 
 * @param dAddedData    INPUT 
 * @param dSurPassedVal OUTPUT surpassed value
 * @param dLimit        INPUT
 * @return double       OUTPUT limited value
 */
inline double fndAddLimit(double dInputData, double dAddedData, double * dSurPassedVal, double dLimit[2]) {
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
inline double fndThreshold(double dInVal, double dThreshold[2]) {
	double dOutVal;
	if (dInVal >= dThreshold[1]) dOutVal = dInVal - dThreshold[1];
	else if (dInVal <= dThreshold[0]) dOutVal = dInVal - dThreshold[0];
	else dOutVal = 0.0;
	return dOutVal;
}
inline double fndThresholdJump(double dInVal, double dThreshold[2]) {
	double dOutVal;
	if (dInVal >= dThreshold[1]) dOutVal = dInVal;
	else if (dInVal <= dThreshold[0]) dOutVal = dInVal;
	else dOutVal = 0.0;
	return dOutVal;
}

/**
 * @brief Interation jerk with limitation
 * 
 * @param dPosIn    UPDATE position to be updated
 * @param dVelIn    UPDATE velocity to be updated
 * @param dAccIn    UPDATE acceleration to be updated
 * @param dJerk     INPUT input jerk to be intergrated
 * @param dLimits   INPUT [nega position limit, posi position limit, nega velocity limit, posi velocity limit, nega acceleration limit, posi acceleration limit]
 * @param dControlT INPUT
 */
inline void fnvIntergJerkLimit(double *dPosIn, double *dVelIn, double *dAccIn, double dJerk, double dLimits[6], double dControlT) {
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

/**
 * @brief Interation acceleration with limitation 
 * 
 * @param dPosIn    UPDATE position to be updated
 * @param dVelIn    UPDATE velocity to be updated
 * @param dAcc      INPUT input acceleration to be intergrated
 * @param dLimits   INPUT [nega position limit, posi position limit, nega velocity limit, posi velocity limit, nega acceleration limit, posi acceleration limit]
 * @param dControlT INPUT
 */
inline void fnvIntergAccLimit(double *dPosIn, double *dVelIn, double dAcc, double dLimits[6], double dControlT) {
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

/**
 * @brief Interation velocity with limitation
 * 
 * @param dPosIn    UPDATE position to be updated
 * @param dVel      INPUT input velocity to be intergrated
 * @param dLimits   INPUT [nega position limit, posi position limit, nega velocity limit, posi velocity limit]
 * @param dControlT INPUT
 */
inline void fnvIntergVeloLimit(double *dPosIn, double dVel, double dLimits[4], double dControlT) {
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

/**
 * @brief TimeLag filter
 * 
 * @param filtered_in   INPUT filtered data from last circle
 * @param data_in       INPUT raw data of current circle
 * @param control_t     INPUT 
 * @param Lag_T         INPUT
 * @return double       OUTPUT filtered data of current circle
 */
inline double fndFilterTimeLag(double filtered_in, double data_in, double control_t, double Lag_T)
{
	double filtered_out;
	filtered_out = (control_t * data_in + Lag_T * filtered_in) / (control_t + Lag_T);
	return filtered_out;
}

class c_Filter3Aver {
public:
	c_Filter3Aver(double3 dWeight) {
		for(int i = 0; i < 3; i++) this->m_dWeight[i] = dWeight[i];
	}
	~c_Filter3Aver() {}
	void Init(double data_in) {
		for(int i = 0; i < 3; i++) this->m_dData[i] = data_in;
	}
	double Filter(double data_in) {
		for(int i = 0; i < 2; i++) this->m_dData[i] = m_dData[i + 1];
		m_dData[2] = data_in;
		double data_out = 0.0, weight_sum = 0.0;
		for(int i = 0; i < 3; i++) data_out += this->m_dWeight[i] * this->m_dData[i], weight_sum += this->m_dWeight[i];
		return (data_out / weight_sum);
	}
private:
	double m_dWeight[3];
	double m_dData[3];
};

/**
 * @brief 5th Spline: generate fifth apline trajectory
 * 
 * @param pos_out   OUTPUT
 * @param maxProgN  INPUT maxmun steps of the spline, you may not use the splined value beyond it
 * @param x0        INPUT
 * @param v0        INPUT
 * @param a0        INPUT
 * @param t0        INPUT
 * @param x1        INPUT
 * @param v1        INPUT
 * @param a1        INPUT
 * @param t1        INPUT
 * @param control_t INPUT
 * @param mode_flag INPUT 'T' traditional: start from k = 0; 'N' novel: start from k = k0 calculated by t0
 */
inline void fnvFifthSpline(double *pos_out, int maxProgN, double x0, double v0, double a0, double t0, double x1, double v1, double a1, double t1, double control_t, char mode_flag)
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

/**
 * @brief dou4th Spline: generate double fourth spline trajectory, usually used for ankle z trajectory
 * 
 * @param pos_out   OUTPUT
 * @param maxProgN  INPUT maxmun steps of the spline, you may not use the splined value beyond it
 * @param x0        INPUT
 * @param v0        INPUT
 * @param a0        INPUT
 * @param t0        INPUT
 * @param x1        INPUT
 * @param t1        INPUT
 * @param x2        INPUT
 * @param v2        INPUT
 * @param a2        INPUT
 * @param t2        INPUT
 * @param control_t INPUT
 * @param mode_flag INPUT 'T' traditional: start from k = 0; 'N' novel: start from k = k0 calculated by t0
 */
inline void fnvDouFourthSpline(double *pos_out, int maxProgN, double x0, double v0, double a0, double t0, double x1, double t1, double x2, double v2, double a2, double t2, double control_t, char mode_flag)
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
	int k0 = (int)floor(t0 / control_t + __Mic8);
	int k1 = (int)floor(t1 / control_t + __Mic8);
	int k2 = (int)floor(t2 / control_t + __Mic8);

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

/**
 * @brief Easy Spline: from the last position of the data to the final position with no started & ended velo & acc
 * 
 * @param pos_out   OUPUT
 * @param maxProgN  INPUT maxmun steps of the spline, you may not use the splined value beyond it
 * @param t0_in     INPUT
 * @param t1_in     INPUT
 * @param x1_in     INPUT
 * @param control_t INPUT
 */
inline void fnvEzSpline(double *pos_out, int maxProgN, double t0_in, double t1_in, double x1_in, double control_t)
{
	// int according to control period
	int k0 = (int)floor(t0_in / control_t + __Mic8);
	int k1 = (int)floor(t1_in / control_t + __Mic8);

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
	else if (fabs(x1 - x0) < __Mic8) { // hold
		for (int i = k0; i < maxProgN; i++) {
			pos_out[i] = x0;
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

/**
 * @brief traditional 5th Spline with position, velocity, acceleration output
 * 
 * @param pos_out   OUTPUT 
 * @param vel_out   OUTPUT
 * @param acc_out   OUTPUT
 * @param maxProgN  INPUT
 * @param x0        INPUT
 * @param v0        INPUT
 * @param a0        INPUT
 * @param t0        INPUT
 * @param x1        INPUT
 * @param v1        INPUT
 * @param a1        INPUT
 * @param t1        INPUT
 * @param control_t INPUT
 */
inline void fnvFifthSplineOutputPVA(double *pos_out, double *vel_out, double *acc_out, int maxProgN, double x0, double v0, double a0, double t0, double x1, double v1, double a1, double t1, double control_t)
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

/**
 * @brief Online spline: spline online to a specified position, with zero ending velocity and acceleration
 * 
 * @param dPosIn    UPDATE first address of the position
 * @param dVelIn    UPDATE first address of the velocity
 * @param dAccIn    UPDATE first address of the acceleration
 * @param dTimeNow  INPUT current time relate to the first address
 * @param dPosCmd   INPUT commanded ending position 
 * @param dTimeCmd  INPUT commanded ending time 
 * @param nKTot     INPUT total spline number from the first address
 * @param dControlT INPUT 
 */
inline void fnvOnLineSpline(double *dPosIn, double *dVelIn, double *dAccIn, double dTimeNow, double dPosCmd, double dTimeCmd, int nKTot, double dControlT) {
	int nKNow = (int)floor(dTimeNow / dControlT), nKCmd = (int)floor(dTimeCmd / dControlT), maxProgN = nKTot - nKNow;
	double x0 = dPosIn[nKNow], v0 = dVelIn[nKNow], a0 = dAccIn[nKNow];
	fnvFifthSplineOutputPVA(dPosIn + nKNow, dVelIn + nKNow, dAccIn + nKNow, maxProgN, x0, v0, a0, dTimeNow, dPosCmd, 0.0, 0.0, dTimeCmd, dControlT);
}

/**
 * @brief tic!
 * 
 */
inline void fnvTic() {
	QueryPerformanceFrequency(&nFreq); 
	QueryPerformanceCounter(&nBeginTime); 
}

/**
 * @brief toc!
 * 
 * @param nDispTimeFlag INPUT 1 display the time span
 * @return double       OUTPUT
 */
inline double fnvToc(int nDispTimeFlag) {
	QueryPerformanceCounter(&nEndTime);
	time_end = ((double)(nEndTime.QuadPart) / (double)nFreq.QuadPart);
	time_start = ((double)(nBeginTime.QuadPart) / (double)nFreq.QuadPart);
	time_ = time_end - time_start; 
	if (nDispTimeFlag == 1) printf("Time = %lf\n", time_);
	return time_;
}


inline int fnnSlowDown(double *dptDataIn, int nDataLen, int nSlowTimes, double *dptDataOut) {
	int nKout = 0;
	for (int i = 0; i < nDataLen - 2; i++) {
		double dIncTemp = (*(dptDataIn + i + 1) - *(dptDataIn + i)) / ((double)nSlowTimes);
		for (int j = 0; j < nSlowTimes; j++) *(dptDataOut + nKout) = *(dptDataIn + i) + (double)j * dIncTemp, nKout++;
	}
	return nKout;
}

inline int fnnRdFile(char *sFileName, int nRow, int nCol, double *dDataName) {
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

inline void fnvWtFile(char *sFileName, int nRow, int nCol, double * dDataName) {
	FILE * Ftp = fopen(sFileName, "w");
	double dDataTemp;
	for (int i = 0; i < nRow; i++) {
		for (int j = 0; j < nCol; j++) dDataTemp = *(dDataName + i * nCol + j), fprintf(Ftp, "%lf\t", dDataTemp);
		fprintf(Ftp, "\n");
	}
	fclose(Ftp);
}

inline double fndAbsWeight(double dDataIn1, double dDataIn2, char cMode) {
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

inline double fndGetSign(double dDataIn) {
	if(fabs(dDataIn) > 1e-6) return (dDataIn / fabs(dDataIn));
	else return 1.0;
}

inline double fndAbsLimit(double dDataIn, double dAbsLimit[2]/*should be active value*/) {
	double dDataOut = dDataIn;
	if(fabs(dDataIn) < dAbsLimit[0]) dDataOut = fndGetSign(dDataIn) * dAbsLimit[0];
	if(fabs(dDataIn) > dAbsLimit[1]) dDataOut = fndGetSign(dDataIn) * dAbsLimit[1];
	return dDataOut;
}

inline double fndActivate(double dDataIn, double dTrigger, char cMode) {
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

inline void fnvSwap(double *dptIn1, double *dptIn2) {
	double dTemp = *dptIn1;
	*dptIn1 = *dptIn2;
	*dptIn2 = dTemp;
}

class c_AnkSpline {
	public:
	c_AnkSpline(double dTc) {
		// this->SetStep({0.36, 0.016, 0.024, 0.0}); // [m_dPerUp, m_dTHold, m_dTLand, m_dZHold]
		this->SetStep({0.5, 0.0, 0.1, 0.0015}); // [m_dPerUp, m_dTHold, m_dTLand, m_dZHold]
		this->m_dTc = dTc;
	}
	~c_AnkSpline() {}
	void SetStep(double4 dParam) {
		this->m_dPerUp = dParam[0], this->m_dTHold = dParam[1], this->m_dTLand = dParam[2], this->m_dZHold = dParam[3];
	}
	double * GetTraX() { return this->m_AnkTraX; }
	double * GetTraY() { return this->m_AnkTraY; }
	double * GetTraZ() { return this->m_AnkTraZ; }
	void fnvUpdateAnkTra(double dTStep, double3 dP0, double3 dPe, double dZStep) {
		this->fnvGetAnkX(dTStep, dP0[0], dPe[0]);
		this->fnvGetAnkY(dTStep, dP0[1], dPe[1]);
		this->fnvGetAnkZ(dTStep, dP0[2], dPe[2], dZStep);
	}
	private:
	const static int nMaxN = 400;
	double m_AnkTraX[nMaxN], m_AnkTraY[nMaxN], m_AnkTraZ[nMaxN];
	double m_dTc, m_dPerUp, m_dTHold, m_dTLand, m_dZHold;
	void fnvGetAnkX(double dTStep, double dX0, double dXe) {
		fnvFifthSpline(this->m_AnkTraX, this->nMaxN, dX0, 0.0, 0.0, 0.0, dXe, 0.0, 0.0, dTStep, this->m_dTc, 'N');
	}
	void fnvGetAnkY(double dTStep, double dY0, double dYe) {
		fnvFifthSpline(this->m_AnkTraY, this->nMaxN, dY0, 0.0, 0.0, 0.0, dYe, 0.0, 0.0, dTStep, this->m_dTc, 'N');
	}
	void fnvGetAnkZ(double dTStep, double dZ0, double dZe, double dZStep) {
		auto dTp = this->m_dPerUp * (dTStep - m_dTHold - m_dTLand);
		auto dTh = dTStep - m_dTHold - m_dTLand;
		auto dTd = dTStep - m_dTLand;
		fnvFifthSpline(this->m_AnkTraZ, this->nMaxN, dZ0, 0.0, 0.0, 0.0, (dZStep + dZe), 0.0, 0.0, dTp, this->m_dTc, 'N');
		fnvFifthSpline(this->m_AnkTraZ, this->nMaxN, (dZStep + dZe), 0.0, 0.0, dTp, (this->m_dZHold + dZe), 0.0, 0.0, dTh, this->m_dTc, 'N');
		fnvEzSpline(this->m_AnkTraZ, this->nMaxN, dTh, dTd, (this->m_dZHold + dZe), this->m_dTc);
		fnvFifthSpline(this->m_AnkTraZ, this->nMaxN, (this->m_dZHold + dZe), 0.0, 0.0, dTd, dZe, 0.0, 0.0, dTStep, this->m_dTc, 'N');
	}
};

_D_BASE_END

#endif 