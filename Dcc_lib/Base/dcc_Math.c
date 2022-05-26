#include "dcc_Mat.h"


/** Obtain transition matrix: 
*/
void fnvObtainTransMat3(double * dptTransMat, char cAxis, double dQ, double dPos[3]) {
	double dTransMat[4][4] = { 0.0 };
	if (cAxis == 'x') {
		double dTransMatTemp[4][4] = {
			1.0, 0.0, 0.0, dPos[0],
			0.0, cos(dQ), -sin(dQ), dPos[1],
			0.0, sin(dQ), cos(dQ), dPos[2],
			0.0, 0.0, 0.0, 1.0
		};
		for (int i = 0; i < 4; i++) {
			for (int j = 0; j < 4; j++) {
				*(dptTransMat + 4 * i + j) = dTransMatTemp[i][j];
			}
		}
	}
	else if (cAxis == 'y') {
		double dTransMatTemp[4][4] = {
			cos(dQ), 0.0, sin(dQ), dPos[0],
			0.0, 1.0, 0.0, dPos[1],
			-sin(dQ), 0.0, cos(dQ), dPos[2],
			0.0, 0.0, 0.0, 1.0
		};
		for (int i = 0; i < 4; i++) {
			for (int j = 0; j < 4; j++) {
				*(dptTransMat + 4 * i + j) = dTransMatTemp[i][j];
			}
		}
	}
	else if (cAxis == 'z') {
		double dTransMatTemp[4][4] = {
			cos(dQ), -sin(dQ), 0.0, dPos[0],
			sin(dQ), cos(dQ), 0.0, dPos[1],
			0.0, 0.0, 1.0, dPos[2],
			0.0, 0.0, 0.0, 1.0
		};
		for (int i = 0; i < 4; i++) {
			for (int j = 0; j < 4; j++) {
				*(dptTransMat + 4 * i + j) = dTransMatTemp[i][j];
			}
		}
	}
	else {
		printf("Wrong Axis in fnvObtainTransMat3!!!\n");
		double dTransMatTemp[4][4] = {
			1.0, 0.0, 0.0, 0.0,
			0.0, 1.0, 0.0, 0.0,
			0.0, 0.0, 1.0, 0.0,
			0.0, 0.0, 0.0, 1.0
		};
		for (int i = 0; i < 4; i++) {
			for (int j = 0; j < 4; j++) {
				*(dptTransMat + 4 * i + j) = dTransMatTemp[i][j];
			}
		}
	}
}

void fnvSO2Eul(double * dRotIn, double * dRulOut) {
    int nRowNum = 3;
    *dRulOut = atan2(*(dRotIn + 2 * nRowNum + 1), *(dRotIn + 2 * nRowNum + 2));
    *(dRulOut + 1) = atan2(-*(dRotIn + 2 * nRowNum), sqrt(*(dRotIn + 2 * nRowNum + 1) * *(dRotIn + 2 * nRowNum + 1) + *(dRotIn + 2 * nRowNum + 2) * *(dRotIn + 2 * nRowNum + 2)));
    *(dRulOut + 2) = atan2(*(dRotIn + 1 * nRowNum), *dRotIn);
}