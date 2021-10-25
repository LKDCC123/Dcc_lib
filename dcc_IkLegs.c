// 20211018 bit
// calculate IK for legs that considers upperbody orientation and ankle orientation
#ifndef DCC_IKLEG_C
#define DCC_IKLEG_C
#include "dcc_IkLegs.h"
#include "Base\dcc_con_base.h"

void fnvCoorTrans(const double * dPosHipW, const double * dOriHipW, const double * dPosAnkW,  const double * dOriAnkW, double * dPosAnkH, double * dOriAnkH) {
	double dTransHYaw[4][4], dTransHPit[4][4], dTransHRol[4][4], dTransTemp[4][4], dTransH2W[4][4], dTransH2WInv[4][4], dPosTemp[3] = { 0.0, 0.0, 0.0 }, dPosAnk2HipW[3], dPosAnk2HipWHomo[4], dPosAnkHHomo[4];
	dcc_fnvMatMet(dOriAnkW, dOriHipW, 3, 1, 1, '-', dOriAnkH);
	fnvObtainTransMat3(dTransHYaw, 'z', *(dOriHipW + 2), dPosTemp);
	fnvObtainTransMat3(dTransHPit, 'x', *(dOriHipW + 0), dPosTemp);
	fnvObtainTransMat3(dTransHRol, 'y', *(dOriHipW + 1), dPosTemp);
	dcc_fnvMatMet(dTransHYaw, dTransHPit, 4, 4, 4, '*', dTransTemp);
	dcc_fnvMatMet(dTransTemp, dTransHRol, 4, 4, 4, '*', dTransH2W);
	dcc_fnvMatInv(dTransH2W, 4, dTransH2WInv);
	dcc_fnvMatMet(dPosAnkW, dPosHipW, 3, 1, 1, '-', dPosAnk2HipW);
	for (int i = 0; i < 3; i++) dPosAnk2HipWHomo[i] = dPosAnk2HipW[i];
	dPosAnk2HipWHomo[3] = 1.0;
	dcc_fnvMatMet(dTransH2WInv, dPosAnk2HipWHomo, 4, 4, 1, '*', dPosAnkHHomo);
	for (int i = 0; i < 3; i++) *(dPosAnkH + i) = dPosAnkHHomo[i];
}


void fnvInitIkLegs() {
	fnvIkSugiInit();
	// init joints when home
	dQRleg[0] = 0.0, dQRleg[1] = 0.0, dQRleg[2] = 0.705231, dQRleg[3] = -1.437267, dQRleg[4] = 0.731885, dQRleg[5] = 0.0;
	dQLleg[0] = 0.0, dQLleg[1] = 0.0, dQLleg[2] = 0.705231, dQLleg[3] = -1.437267, dQLleg[4] = 0.731885, dQLleg[5] = 0.0;
	// init pos & ori when home
	dHipPR[0] = 0.0, dHipPR[1] = 0.0, dHipPR[2] = 0.6, dHipPR[3] = 0.0, dHipPR[4] = 0.0, dHipPR[5] = 0.0;
	dRAnkPR[0] = 0.08, dRAnkPR[1] = 0.0, dRAnkPR[2] = 0.112, dRAnkPR[3] = 0.0, dRAnkPR[4] = 0.0, dRAnkPR[5] = 0.0;
	dLAnkPR[0] = -0.08, dLAnkPR[1] = 0.0, dLAnkPR[2] = 0.112, dLAnkPR[3] = 0.0, dLAnkPR[4] = 0.0, dLAnkPR[5] = 0.0;
}

void fnvCalIkLegs() {
	fnvCoorTrans(dHipPR, &dHipPR[0] + 3, dRAnkPR, &dRAnkPR[0] + 3, dPosiRAnk, dOritRAnk);
	fnvCoorTrans(dHipPR, &dHipPR[0] + 3, dLAnkPR, &dLAnkPR[0] + 3, dPosiLAnk, dOritLAnk);
	fnvIkSugiOL();
}

#endif