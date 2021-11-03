#pragma once 

#include <math.h>
#include "dcc_Mat.h"

#ifndef __Pi
#define __Pi 3.141592653523846
#endif
#ifndef DccR2D
#define DccR2D(a) a * 180.0 / __Pi
#endif
#ifndef DccD2R
#define DccD2R(a) a * __Pi / 180.0
#endif
#ifndef __KLeftToNow
#define __KLeftToNow 100 // reserved num of k to now when circuling
#endif
#ifndef __MaxKprog
#define __MaxKprog 10000
#endif
#ifndef MaxOf
#define MaxOf(a, b) (a > b ? a : b)
#endif
#ifndef MinOf
#define MinOf(a, b) (a < b ? a : b)
#endif

typedef struct {
	double p;
	double dp;
	double ddp;
}IntegValLimit;

typedef struct {
	double x;
	double y;
	double z;
	double dx;
	double dy;
	double dz;
	double ddx;
	double ddy;
	double ddz;
}dccPositional;

typedef struct {
	double pit;
	double rol;
	double yaw;
	double dpit;
	double drol;
	double dyaw;
	double ddpit;
	double ddrol;
	double ddyaw;
}dccRotational;

typedef struct {
	double fx;
	double fy;
	double fz;
	double tx;
	double ty;
	double tz;
}dccForceSensor;

typedef struct {
	dccPositional pos;
	dccRotational rot;
}dccSpacial;

typedef struct {
	double forw;
	double back;
	double left;
	double righ;
}dccSupPoly;// rec dcc

typedef struct {
	dccForceSensor Lfoot;
	dccForceSensor Rfoot;
	double Fsum;
}dccFootFT;

typedef struct {
	dccSpacial Lfoot;
	dccSpacial Rfoot;
}dccAnkle;

typedef struct {
	dccAnkle W;
	dccAnkle B;
}dccAnkleTwoFrame;

typedef struct {
	dccPositional W;
	dccPositional B;
}dccZMP;

typedef struct {
	dccZMP ZMP;
	dccSpacial Base;
	dccFootFT FootFT;
	dccAnkleTwoFrame Ankle;
	int SupLeg; // dou 0, rsup 1, lsup 2, fly 3
}dccRobotState;

typedef struct { // armswi
	double Ldq[6];
	double Rdq[6];
	double Lq[6];
	double Rq[6];
	double La;
	double Ra;
}dccJoints;

typedef struct {
	double x[__MaxKprog];
	double y[__MaxKprog];
	double z[__MaxKprog];
	double dx[__MaxKprog];
	double dy[__MaxKprog];
	double dz[__MaxKprog];
	double ddx[__MaxKprog];
	double ddy[__MaxKprog];
	double ddz[__MaxKprog];
}dccPositional_tra;

typedef struct {
	double pit[__MaxKprog];
	double rol[__MaxKprog];
	double yaw[__MaxKprog];
	double dpit[__MaxKprog];
	double drol[__MaxKprog];
	double dyaw[__MaxKprog];
	double ddpit[__MaxKprog];
	double ddrol[__MaxKprog];
	double ddyaw[__MaxKprog];
}dccRotational_tra;

typedef struct {
	double fx[__MaxKprog];
	double fy[__MaxKprog];
	double fz[__MaxKprog];
	double tx[__MaxKprog];
	double ty[__MaxKprog];
	double tz[__MaxKprog];
}dccForceSensor_tra;

typedef struct {
	dccPositional_tra pos;
	dccRotational_tra rot;
}dccSpacial_tra;

typedef struct {
	double forw[__MaxKprog];
	double back[__MaxKprog];
	double left[__MaxKprog];
	double righ[__MaxKprog];
}dccSupPoly_tra;// rec dcc

typedef struct {
	dccForceSensor_tra Lfoot;
	dccForceSensor_tra Rfoot;
	double Fsum[__MaxKprog];
}dccFootFT_tra;

typedef struct {
	dccSpacial_tra Lfoot;
	dccSpacial_tra Rfoot;
}dccAnkle_tra;

typedef struct {
	dccAnkle_tra W;
	dccAnkle_tra B;
}dccAnkleTwoFrame_tra;

typedef struct {
	dccPositional_tra W;
	dccPositional_tra B;
}dccZMP_tra;

typedef struct {
	dccZMP_tra ZMP;
	dccSpacial_tra Base;
	dccFootFT_tra FootFT;
	dccAnkleTwoFrame_tra Ankle;
	int SupLeg[__MaxKprog]; // dou 0, rsup 1, lsup 2, fly 3
	int WalkFlag;
}dccRobotState_tra;

double fndLimit(double dInputData, double dLimit[2]);
double fndAddLimit(double dInputData, double dAddedData, double * dSurPassedVal, double dLimit[2]); // rec dcc
double fndThreshold(double dInVal, double dThreshold[2]);
void fnvInitInteg(IntegValLimit *strInputData);
IntegValLimit fnstrIntegLimit(IntegValLimit strOldData, double dAcc, double dLimits[6], double dControlT);
void fnvJerkLimit(double *dPosIn, double *dVelIn, double *dAccIn, double dJerk, double dLimits[6], double dControlT);
void fnvIntegLimit(double *dPosIn, double *dVelIn, double dAcc, double dLimits[6], double dControlT);
void fnvVeloLimit(double *dPosIn, double dVel, double dLimits[4], double dControlT);
double fndFilterTimeLag(double filtered_in, double data_in, double control_t, double Lag_T);
void fnvFifthSpline(double *pos_out, int maxProgN, double x0, double v0, double a0, double t0, double x1, double v1, double a1, double t1, double control_t, char mode_flag);
void fnvDouFourthSpline(double *pos_out, int maxProgN, double x0, double v0, double a0, double t0, double x1, double t1, double x2, double v2, double a2, double t2, double control_t, char mode_flag);
void fnvEzSpline(double *pos_out, int maxProgN, double t0_in, double t1_in, double x1_in, double control_t);
void fnvObtainTransMat3(double * dptTransMat, char cAxis, double dQ, double dPos[3]);
void fnvTic();
double fnvToc(int nDispTimeFlag);
int fnnSlowDown(double * dptDataIn, int nDataLen, int nSlowTimes, double * dptDataOut);
int fnnRdFile(char * sFileName, int nRow, int nCol, double * dDataName);
void fnvWtFile(char * sFileName, int nRow, int nCol, double * dDataName);