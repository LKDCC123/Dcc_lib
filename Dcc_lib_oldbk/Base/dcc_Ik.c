// 20211007 bit
// this code can solve multiple IK problem with redundancy or singularity
#ifndef DCC_IK_C
#define DCC_IK_C
#include <math.h>
#include "dcc_Ik.h"
#include "dcc_Mat.h"
#include "dcc_con_base.h"

// for test
//int nDoFTest = 7;
//#define __dBaseVectorTest 0.0, 0.0, 0.0 
//#define __dLinkLengthTest 0.1, 0.2, 0.2, 0.1, 0.0, 0.0, 0.05 // joint -> *------ <- link length
//#define __cPinDirectionTest 'z', 'x', 'x', 'x', 'z', 'x', 'z'
//double dBaseVecTest[3] = { __dBaseVectorTest };
//double dLinkLenTest[__nMaxDoF] = { __dLinkLengthTest };
//char cPinDirectTest[__nMaxDoF] = { __cPinDirectionTest };

// for R leg
int nDoFRleg = 6;
#define __dBaseVectorRleg 0.08, 0.0, 0.0 
#define __dLinkLengthRleg 0.0, 0.0, 0.33, 0.32, 0.0, 0.0 // joint -> *------ <- link length
#define __cPinDirectionRleg 'z', 'y', 'x', 'x', 'x', 'y'
double dBaseVecRleg[3] = { __dBaseVectorRleg };
double dLinkLenRleg[__nMaxDoF] = { __dLinkLengthRleg };
char cPinDirectRleg[__nMaxDoF] = { __cPinDirectionRleg };

// for L leg
int nDoFLleg = 6;
#define __dBaseVectorLleg -0.08, 0.0, 0.0 
#define __dLinkLengthLleg 0.0, 0.0, 0.33, 0.32, 0.0, 0.0 // joint -> *------ <- link length
#define __cPinDirectionLleg 'z', 'y', 'x', 'x', 'x', 'y'
double dBaseVecLleg[3] = { __dBaseVectorLleg };
double dLinkLenLleg[__nMaxDoF] = { __dLinkLengthLleg };
char cPinDirectLleg[__nMaxDoF] = { __cPinDirectionLleg };
// tbc ...

// general
double dTransMat[__nMaxDoF][4][4] = { 0.0 };
#define __dWeightDoF /*x*/1.5, /*y*/1.5, /*z*/1.5, /*pit*/1.0, /*rol*/1.0, /*yaw*/1.0
#define __dWeightQ 1.0, 1.0, 1.0, 1.0, 1.0, 1.0
#define __dAmpCon 2.5
#define __dErrTol 1e-8
#define __dStabTol 1e-6
#define __nMaxIterN 1000
#define __nMaxStabN 3


void fnvSetInitQ(int nDoF, double * dQ, double * dStabFlag) {
	for (int i = 0; i < nDoF; i++) dQ[i] = 0.2;
	*dStabFlag = 0.0;
}

// a_n is obtained 
void fnvObtainAxisVec(char cPinDirect, double * dAxisVec) {
	double dAxisX[3] = { 1.0, 0.0, 0.0 };
	double dAxisY[3] = { 0.0, 1.0, 0.0 };
	double dAxisZ[3] = { 0.0, 0.0, 1.0 };
	if (cPinDirect == 'x') for (int i = 0; i < 3; i++) dAxisVec[i] = dAxisX[i];
	else if (cPinDirect == 'y') for (int i = 0; i < 3; i++) dAxisVec[i] = dAxisY[i];
	else if (cPinDirect == 'z') for (int i = 0; i < 3; i++) dAxisVec[i] = dAxisZ[i];
	else printf("Wrong pin direction in fnvObtainAxisVec!!\n");
}

// ^wa_n, ^w_P_n are obtained for any limb 
void fnvObtainJointsPosiAxis(double * dQ, int nDoF, double dLimbUp, double dBaseVec[3], double * dLinkLen, char * cPinDirect, double * dJointsPosi, double * dJointAxis, double * dEndPosi, double * dEndOrit) {
	double dTransMatTemp[4][4], dTransMatOld[4][4], dTransMatNew[4][4], dPosTemp[4], dPosWorldTemp[4];
	double dRotMatOld[3][3], dAxisTemp[3], dAxisWorldTemp[3];
	for (int i = 0; i < nDoF; i++) { // cal joints posi & axis
		if (i == 0) { // the first joint 
			// cal posi
			fnvObtainTransMat3(dTransMatNew, cPinDirect[0], dQ[0], dBaseVec);
			dcc_fnvMatCopy(dTransMatNew, 4, 4, 4, dTransMatOld);
			for (int j = 0; j < 3; j++) *(dJointsPosi + j) = dBaseVec[j];
			// cal axis
			fnvObtainAxisVec(cPinDirect[0], dAxisTemp);
			for (int j = 0; j < 3; j++) *(dJointAxis + j) = dAxisTemp[j];
		}
		else { // latter joints
			// cal posi
			dPosTemp[0] = 0.0, dPosTemp[1] = 0.0, dPosTemp[2] = dLimbUp * dLinkLen[i - 1], dPosTemp[3] = 1.0;
			fnvObtainTransMat3(dTransMatNew, cPinDirect[i], dQ[i], dPosTemp);
			dcc_fnvMatMet(dTransMatOld, dTransMatNew, 4, 4, 4, '*', dTransMatTemp);
			dcc_fnvMatMet(dTransMatOld, dPosTemp, 4, 4, 1, '*', dPosWorldTemp);
			dcc_fnvMatCopy(dTransMatTemp, 4, 4, 4, dTransMatOld);
			for (int j = 0; j < 3; j++) *(dJointsPosi + 3 * i + j) = dPosWorldTemp[j];
			// cal axis
			fnvObtainAxisVec(cPinDirect[i], dAxisTemp);
			dcc_fnvMatCopy(dTransMatOld, 4, 3, 3, dRotMatOld);
			dcc_fnvMatMet(dRotMatOld, dAxisTemp, 3, 3, 1, '*', dAxisWorldTemp);
			for (int j = 0; j < 3; j++) *(dJointAxis + 3 * i + j) = dAxisWorldTemp[j];
		}
	}
	// cal end effector posi & orit
	// cal posi 
	dPosTemp[0] = 0.0, dPosTemp[1] = 0.0, dPosTemp[2] = dLimbUp * dLinkLen[nDoF - 1], dPosTemp[3] = 1.0;
	dcc_fnvMatMet(dTransMatOld, dPosTemp, 4, 4, 1, '*', dPosWorldTemp);
	for (int j = 0; j < 3; j++) *(dEndPosi + j) = dPosWorldTemp[j];
	// cal orit
	dcc_fnvMatCopy(dTransMatOld, 4, 3, 3, dRotMatOld);
	dEndOrit[0] = atan2(dRotMatOld[2][1], dRotMatOld[2][2]);
	dEndOrit[1] = atan2(-dRotMatOld[2][0], sqrt(dRotMatOld[2][1] * dRotMatOld[2][1] + dRotMatOld[2][2] * dRotMatOld[2][2]));
	dEndOrit[2] = atan2(dRotMatOld[1][0], dRotMatOld[0][0]);
}

// Jaco is obtained 
void fnvObtainJaco(int nDoF, double * dJointsPosi, double * dJointAxis, double * dEndPosi, double * dJaco) {
	double dAxisWorldTemp[3], dPosWorldTemp[3], dDelPosWorldTemp[3], dJacoVal[3];
	for (int i = 0; i < nDoF; i++) {
		for (int j = 0; j < 3; j++) {
			dPosWorldTemp[j] = *(dJointsPosi + 3 * i + j);
			dAxisWorldTemp[j] = *(dJointAxis + 3 * i + j);
		}
		dcc_fnvMatMet(dEndPosi, dPosWorldTemp, 3, 1, 1, '-', dDelPosWorldTemp);
		dcc_fnvMatMet(dAxisWorldTemp, dDelPosWorldTemp, 3, 1, 1, 'x', dJacoVal);
		for (int j = 0; j < 3; j++) *(dJaco + nDoF * j + i) = dJacoVal[j];
		for (int j = 3; j < 6; j++) *(dJaco + nDoF * j + i) = dAxisWorldTemp[j - 3];
	}
}

// Err is obtained
void fnvObtainErr(double * dEndPosi, double * dEndOrit, double * dEndPosiCmd, double * dEndOritCmd, double * dErr) {
	for (int i = 0; i < 3; i++) *(dErr + i) = *(dEndPosiCmd + i) - *(dEndPosi + i);
	for (int i = 3; i < 6; i++) *(dErr + i) = *(dEndOritCmd + i - 3) - *(dEndOrit + i - 3);
}

// DelQ is obtained
double fndObtainDelQ(int nDoF, double * dJaco, double * dErr, double * dDelQ) {
	double dWeLine[] = { __dWeightDoF }, dWe[6][6];
	double dWnLine[] = { __dWeightQ }, dWnBar[__nMaxDoF][__nMaxDoF], dWnTemp[__nMaxDoF][__nMaxDoF], dWn[__nMaxDoF][__nMaxDoF];
	double dJacoT[__nMaxDoF][6], dErrTemp[6], dEk, dHkInv[__nMaxDoF][__nMaxDoF];
	double dGk[__nMaxDoF], dHkTemp1[__nMaxDoF][6], dHkTemp2[__nMaxDoF][__nMaxDoF], dHk[__nMaxDoF][__nMaxDoF];
	dcc_fnvMatTrans(dJaco, 6, nDoF, dJacoT);
	dcc_fnvDiag(dWeLine, 6, dWe);
	dcc_fnvMatMet(dErr, dWe, 1, 6, 6, '*', dErrTemp);
	dcc_fnvMatMet(dErrTemp, dErr, 1, 6, 1, '*', &dEk);
	dEk *= 0.5; // dEk ok!
	dcc_fnvDiag(dWeLine, 6, dWnBar);
	dcc_fnvDiag(dWnLine, nDoF, dWnBar); // dWnBar ok!
	dcc_fnvEye(dEk, nDoF, dWnTemp);
	dcc_fnvMatMet(dWnTemp, dWnBar, nDoF, nDoF, nDoF, '+', dWn); // dWn ok!
	dcc_fnvMatMet(dJacoT, dWe, nDoF, 6, 6, '*', dHkTemp1); 
	dcc_fnvMatMet(dHkTemp1, dJaco, nDoF, 6, nDoF, '*', dHkTemp2);
	dcc_fnvMatMet(dHkTemp2, dWn, nDoF, nDoF, nDoF, '+', dHk); // dHk ok!
	dcc_fnvMatMet(dHkTemp1, dErr, nDoF, 6, 1, '*', dGk); // dGk ok!
	dcc_fnvMatInv(dHk, nDoF, dHkInv);
	dcc_fnvMatMet(dHkInv, dGk, nDoF, nDoF, 1, '*', dDelQ);
	return dEk; // return as the dJc
}

// Q is updated in one step for any limb
double fndJointsUpdate(double * dQ, int nDoF, double dLimbUp, double dBaseVec[3], double * dLinkLen, char * cPinDirect, double dEndPosiCmd[3], double dEndOritCmd[3]) {
	double dJointsPosi[__nMaxDoF][3], dJointAxis[__nMaxDoF][3], dEndPosi[3], dEndOrit[3], dJaco[6][__nMaxDoF], dErr[6], dAmpCon = __dAmpCon, dDelQTemp[__nMaxDoF], dDelQ[__nMaxDoF], dJc;
	fnvObtainJointsPosiAxis(dQ, nDoF, dLimbUp, dBaseVec, dLinkLen, cPinDirect, dJointsPosi, dJointAxis, dEndPosi, dEndOrit); // ^wa_n, ^w_P_n are obtained for any limb 
	fnvObtainJaco(nDoF, dJointsPosi, dJointAxis, dEndPosi, dJaco); // Jaco is obtained 
	fnvObtainErr(dEndPosi, dEndOrit, dEndPosiCmd, dEndOritCmd, dErr); // Err is obtained
	dJc = fndObtainDelQ(nDoF, dJaco, dErr, dDelQTemp); // DelQ is obtained
	dcc_fnvMatMet(dDelQTemp, &dAmpCon, nDoF, 1, 0, '*', dDelQ);
	for (int i = 0; i < __nMaxDoF; i++) *(dQ + i) += *(dDelQ + i);
	return dJc;
}

// Q is obtained based on Q in the last control period
double fndIkSugiObtainQ(const int nDoF, const double dLimbUp, const double dBaseVec[3], const double * dLinkLen, const char * cPinDirect, const double dEndPosiCmd[3], const double dEndOritCmd[3], double * dQ, int nDispJcFlag) {
	double dJc = 1.0e8, dJcOld = dJc, dQTemp[__nMaxDoF];
	int nIterN = __nMaxIterN, nStabN = __nMaxStabN, nStabFlag = 0;
	for (int i = 0; i < nDoF; i++) dQTemp[i] = *(dQ + i); // isolation for safty sake
	while (dJc > __dErrTol && nIterN > 0 && nStabN > 0) {
		nIterN--; // if calculation is tooooo time consuming, break
		if (fabs(dJc - dJcOld) < __dStabTol) nStabN--;
		else nStabN = __nMaxStabN; // if command position is out of range, break
		dJcOld = dJc; 
		dJc = fndJointsUpdate(dQTemp, nDoF, dLimbUp, dBaseVec, dLinkLen, cPinDirect, dEndPosiCmd, dEndOritCmd); // Q is updated in one step for any limb
		if (nDispJcFlag == 1) printf("%lf\n", dJc);
	}
	if (nIterN > 0) nStabFlag = 1;
	if (nStabFlag == 1.0) for (int i = 0; i < nDoF; i++) *(dQ + i) = dQTemp[i];
	return nStabFlag;
}

// Use this in init:
void fnvIkSugiInit() {
	//fnvSetInitQ(nDoFTest, dQTest, &dStabFlagTest); // test
	fnvSetInitQ(nDoFRleg, dQRleg, &dStabFlagRLeg); // R leg
	fnvSetInitQ(nDoFLleg, dQLleg, &dStabFlagLLeg); // L leg
	// tbc...
}

// Use this in loop: give dPosi & dOrit at the first place plz!!!
void fnvIkSugiOL() {
	//dStabFlagTest = fndIkSugiObtainQ(nDoFTest, 1.0, dBaseVecTest, dLinkLenTest, cPinDirectTest, dPosiTest, dOritTest, dQTest, 0); // test
	dStabFlagRLeg = fndIkSugiObtainQ(nDoFRleg, -1.0, dBaseVecRleg, dLinkLenRleg, cPinDirectRleg, dPosiRAnk, dOritRAnk, dQRleg, 0); // R leg
	dStabFlagLLeg = fndIkSugiObtainQ(nDoFLleg, -1.0, dBaseVecLleg, dLinkLenLleg, cPinDirectLleg, dPosiLAnk, dOritLAnk, dQLleg, 0); // L leg
}

#endif

