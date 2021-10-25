#ifndef DCC_FUZZY_C
#define DCC_FUZZY_C
#endif
#include <stdio.h>
#include <math.h>
#include <string.h>
#include "dcc_FuzzyCon.h"

int fnnNormalization(double dVal_in, double dMinMax_in[2], int nMax_out) {
	int nVal_out;
	nVal_out = (int)((dVal_in - dMinMax_in[0]) * (double)nMax_out / (dMinMax_in[1] - dMinMax_in[0]));
	return nVal_out;
}

double fnnInverseNormalization(int nVal_in, int nMax_in, double dMinMax_out[2]) {
	double dVal_out;
	dVal_out = (double)nVal_in * (dMinMax_out[1] - dMinMax_out[0]) / (double)nMax_in + dMinMax_out[0];
	return dVal_out;
}

void fnvMakeTriangle(double *dptMemTriangle, int nNormMemConfig[3], char cPos, int nNormVal) {
	double dDeltaTemp;
	int i;
	// left linguist
	if (cPos == 'L') {
		for (i = nNormMemConfig[0]; i < nNormMemConfig[1]; i++) *(dptMemTriangle + i) = 1.0;
		dDeltaTemp = fabs(1.0 / (double)(nNormMemConfig[2] - nNormMemConfig[1]));
		for (i = nNormMemConfig[1]; i < nNormMemConfig[2]; i++) *(dptMemTriangle + i) = *(dptMemTriangle + i - 1) - dDeltaTemp;
		for (i = nNormMemConfig[2]; i < nNormVal; i++) *(dptMemTriangle + i) = 0.0;
	}
	// right linguist
	else if (cPos == 'R') {
		for (i = 0; i < nNormMemConfig[0]; i++) *(dptMemTriangle + i) = 0.0;
		dDeltaTemp = fabs(1.0 / (double)(nNormMemConfig[1] - nNormMemConfig[0]));
		for (i = nNormMemConfig[0]; i < nNormMemConfig[1]; i++) *(dptMemTriangle + i) = *(dptMemTriangle + i - 1) + dDeltaTemp;
		for (i = nNormMemConfig[1]; i < nNormMemConfig[2]; i++) *(dptMemTriangle + i) = 1.0;
	}
	// midle linguist
	if (cPos == 'M') {
		for (i = 0; i < nNormMemConfig[0]; i++) *(dptMemTriangle + i) = 0.0;
		dDeltaTemp = fabs(1.0 / (double)(nNormMemConfig[1] - nNormMemConfig[0]));
		for (i = nNormMemConfig[0]; i < nNormMemConfig[1]; i++) *(dptMemTriangle + i) = *(dptMemTriangle + i - 1) + dDeltaTemp;
		dDeltaTemp = fabs(1.0 / (double)(nNormMemConfig[2] - nNormMemConfig[1]));
		for (i = nNormMemConfig[1]; i < nNormMemConfig[2]; i++) *(dptMemTriangle + i) = *(dptMemTriangle + i - 1) - dDeltaTemp;
		for (i = nNormMemConfig[2]; i < nNormVal; i++) *(dptMemTriangle + i) = 0.0;
	}
}

void fnvMakeMembership() {
	int i, j;
	double dMemTempIn1[3], dMemTempIn2[3], dMemTempOut[3];
	int nNormMemConfigTempIn1[3], nNormMemConfigTempIn2[3], nNormMemConfigTempOut[3];
	// left linguist
	for (i = 0; i < 3; i++) {
		nNormMemConfigTempIn1[i] = fnnNormalization(stDccFuzzy.dMemConfigIn1[0][i], stDccFuzzy.dMinMaxIn1, __NormVal);
		nNormMemConfigTempIn2[i] = fnnNormalization(stDccFuzzy.dMemConfigIn2[0][i], stDccFuzzy.dMinMaxIn2, __NormVal);
		nNormMemConfigTempOut[i] = fnnNormalization(stDccFuzzy.dMemConfigOut[0][i], stDccFuzzy.dMinMaxOut, __NormVal);
	}
	fnvMakeTriangle(stDccFuzzy.dMemIn1[0], nNormMemConfigTempIn1, 'L', __NormVal);
	fnvMakeTriangle(stDccFuzzy.dMemIn2[0], nNormMemConfigTempIn2, 'L', __NormVal);
	fnvMakeTriangle(stDccFuzzy.dMemOut[0], nNormMemConfigTempOut, 'L', __NormVal);
	// midle linguist
	for (j = 1; j < stDccFuzzy.nLingNum[0] - 1; j++) {
		for (i = 0; i < 3; i++) nNormMemConfigTempIn1[i] = fnnNormalization(stDccFuzzy.dMemConfigIn1[j][i], stDccFuzzy.dMinMaxIn1, __NormVal);
		fnvMakeTriangle(stDccFuzzy.dMemIn1[j], nNormMemConfigTempIn1, 'M', __NormVal);
	}
	for (j = 1; j < stDccFuzzy.nLingNum[1] - 1; j++) {
		for (i = 0; i < 3; i++) nNormMemConfigTempIn2[i] = fnnNormalization(stDccFuzzy.dMemConfigIn2[j][i], stDccFuzzy.dMinMaxIn2, __NormVal);
		fnvMakeTriangle(stDccFuzzy.dMemIn2[j], nNormMemConfigTempIn2, 'M', __NormVal);
	}
	for (j = 1; j < stDccFuzzy.nLingNum[2] - 1; j++) {
		for (i = 0; i < 3; i++) nNormMemConfigTempOut[i] = fnnNormalization(stDccFuzzy.dMemConfigOut[j][i], stDccFuzzy.dMinMaxOut, __NormVal);
		fnvMakeTriangle(stDccFuzzy.dMemOut[j], nNormMemConfigTempOut, 'M', __NormVal);
	}
	// right linguist
	for (i = 0; i < 3; i++) {
		nNormMemConfigTempIn1[i] = fnnNormalization(stDccFuzzy.dMemConfigIn1[stDccFuzzy.nLingNum[0] - 1][i], stDccFuzzy.dMinMaxIn1, __NormVal);
		nNormMemConfigTempIn2[i] = fnnNormalization(stDccFuzzy.dMemConfigIn2[stDccFuzzy.nLingNum[1] - 1][i], stDccFuzzy.dMinMaxIn2, __NormVal);
		nNormMemConfigTempOut[i] = fnnNormalization(stDccFuzzy.dMemConfigOut[stDccFuzzy.nLingNum[2] - 1][i], stDccFuzzy.dMinMaxOut, __NormVal);
	}
	fnvMakeTriangle(stDccFuzzy.dMemIn1[stDccFuzzy.nLingNum[0] - 1], nNormMemConfigTempIn1, 'R', __NormVal);
	fnvMakeTriangle(stDccFuzzy.dMemIn2[stDccFuzzy.nLingNum[1] - 1], nNormMemConfigTempIn2, 'R', __NormVal);
	fnvMakeTriangle(stDccFuzzy.dMemOut[stDccFuzzy.nLingNum[2] - 1], nNormMemConfigTempOut, 'R', __NormVal);
}

/**
guidance:
#define LingNumIn1 // 3~7
#define LingNumIn2 // 3~7
#define LingNumOut // 3~7
int nLingNum_in[3] = { LingNumIn1, LingNumIn2, LingNumOut };
int nMethods[2] = { 0, 1 }; // { inference engine(0->min, 1->max), defuzzier(0->sum, 1->and) }
nptFuzzyBase_in[LingNumIn1][LingNumIn2] = {}; // Linguisitc starts from 0
double dMemConfig_in1[LingNumIn1][3] = {} // Maximun overlaped linguistic is 2
double dMemConfig_in1[LingNumIn2][3] = {} // Maximun overlaped linguistic is 2
double dMemConfig_in1[LingNumOut][3] = {} // Maximun overlaped linguistic is 2
*/
void fnvFuzzyConInit(int *nptFuzzyBase_in, double *dptMemConfig_in1, double *dptMemConfig_in2, double *dptMemConfig_out, int nLingNum_in[3], int nMethods[2]) {
	int i, j;
	// init input number
	if (dptMemConfig_in2 == NULL) stDccFuzzy.nInputNum = 1;
	else stDccFuzzy.nInputNum = 2;
	// init linguistic number
	for (i = 0; i < 3; i++) stDccFuzzy.nLingNum[i] = nLingNum_in[i];
	// init fuzzy base
	for (i = 0; i < stDccFuzzy.nLingNum[0]; i++) { 
		for (j = 0; j < stDccFuzzy.nLingNum[1]; j++) stDccFuzzy.nFuzzyBase[i][j] = *(nptFuzzyBase_in + i * stDccFuzzy.nLingNum[1] + j);
	}
	// init membership
	for (j = 0; j < 3; j++) {
		for (i = 0; i < stDccFuzzy.nLingNum[0]; i++) stDccFuzzy.dMemConfigIn1[i][j] = *(dptMemConfig_in1 + i * 3 + j);
		for (i = 0; i < stDccFuzzy.nLingNum[1]; i++) stDccFuzzy.dMemConfigIn2[i][j] = *(dptMemConfig_in2 + i * 3 + j);
		for (i = 0; i < stDccFuzzy.nLingNum[2]; i++) stDccFuzzy.dMemConfigOut[i][j] = *(dptMemConfig_out + i * 3 + j);
	}
	// init methods
	stDccFuzzy.nInferMethod = nMethods[0]; 
	stDccFuzzy.nDefuzMethod = nMethods[1];
	// init normalization
	stDccFuzzy.dMinMaxIn1[0] = stDccFuzzy.dMemConfigIn1[0][0];
	stDccFuzzy.dMinMaxIn1[1] = stDccFuzzy.dMemConfigIn1[stDccFuzzy.nLingNum[0] - 1][2];
	stDccFuzzy.dMinMaxIn2[0] = stDccFuzzy.dMemConfigIn2[0][0];
	stDccFuzzy.dMinMaxIn2[1] = stDccFuzzy.dMemConfigIn2[stDccFuzzy.nLingNum[1] - 1][2];
	stDccFuzzy.dMinMaxOut[0] = stDccFuzzy.dMemConfigOut[0][0];
	stDccFuzzy.dMinMaxOut[1] = stDccFuzzy.dMemConfigOut[stDccFuzzy.nLingNum[2] - 1][2];
	// make membership
	fnvMakeMembership();
}

/**
guidance:
double dIn1 // Input 1
double dIn2 // Input 2
int nDispFlag // 1 -> Display the tips, 0 -> Do not display
*/
double fndCalFuzzyConVal(double dIn1, double dIn2, int nDispFlag) {
	int i, j;
	int nState = 0;
	int nNormedIn1 = fnnNormalization(dIn1, stDccFuzzy.dMinMaxIn1, __NormVal);
	int nNormedIn2 = fnnNormalization(dIn2, stDccFuzzy.dMinMaxIn2, __NormVal);
	char *sMethodInfer;
	char *sMethodDefuz;
	double dSumOut[2] = { 0.0 }; // [CoM * Mass, Mass]
	double dOut;
	// inference engine
	for (i = 0; i < stDccFuzzy.nLingNum[0]; i++) {
		for (j = 0; j < stDccFuzzy.nLingNum[1]; j++) {
			if (stDccFuzzy.dMemIn1[i][nNormedIn1] > 1e-8 && stDccFuzzy.dMemIn2[j][nNormedIn2] > 1e-8) { // activated
				stDccFuzzy.nLingActivated[nState][0] = i;
				stDccFuzzy.dLingMemVal[nState][0] = stDccFuzzy.dMemIn1[i][nNormedIn1];
				stDccFuzzy.nLingActivated[nState][1] = j;
				stDccFuzzy.dLingMemVal[nState][1] = stDccFuzzy.dMemIn1[j][nNormedIn2];
				stDccFuzzy.nLingActivated[nState][2] = stDccFuzzy.nFuzzyBase[i][j];
				if (stDccFuzzy.nInferMethod == 0) stDccFuzzy.dLingMemVal[nState][2] = fmin(stDccFuzzy.dMemIn1[i][nNormedIn1], stDccFuzzy.dMemIn1[j][nNormedIn2]); // min
				else if (stDccFuzzy.nInferMethod == 1) stDccFuzzy.dLingMemVal[nState][2] = fmax(stDccFuzzy.dMemIn1[i][nNormedIn1], stDccFuzzy.dMemIn1[j][nNormedIn2]); // max
				else printf("Wrong Inference Method!\n");
				nState++;
			}			
		}
	}
	// defuzzier
	for (i = 0; i < __NormVal; i++) {
		for (nState = 0; nState < 4; nState++) stDccFuzzy.dOutMemVal[nState][i] = fmin(stDccFuzzy.dLingMemVal[nState][2], stDccFuzzy.dMemOut[stDccFuzzy.nLingActivated[nState][2]][i]);
		if (stDccFuzzy.nDefuzMethod == 0) { // sum
			stDccFuzzy.dOutMemVal[4][i] = 0.0;
			for (nState = 0; nState < 4; nState++) stDccFuzzy.dOutMemVal[4][i] += stDccFuzzy.dOutMemVal[nState][i];
		}
		else if (stDccFuzzy.nDefuzMethod == 1) { // and
			stDccFuzzy.dOutMemVal[4][i] = stDccFuzzy.dOutMemVal[0][i];
			for (nState = 1; nState < 4; nState++) stDccFuzzy.dOutMemVal[4][i] = fmax(stDccFuzzy.dOutMemVal[4][i], stDccFuzzy.dOutMemVal[nState][i]);
		}
		else printf("Wrong Defuzzier Method!\n");
	}
	for (i = 0; i < __NormVal; i++) dSumOut[0] += stDccFuzzy.dOutMemVal[4][i] * (double)i, dSumOut[1] += stDccFuzzy.dOutMemVal[4][i];
	stDccFuzzy.nOutNormed = (int)(dSumOut[0] / dSumOut[1]);
	// disp
	if (nDispFlag == 1) {
		// inference
		if (stDccFuzzy.nInferMethod == 0) sMethodInfer = "Minimize";
		else if (stDccFuzzy.nInferMethod == 1) sMethodInfer = "Maximum";
		else sMethodInfer = "Error";
		// defuzzier
		if (stDccFuzzy.nDefuzMethod == 0) sMethodDefuz = "Summation";
		else if (stDccFuzzy.nDefuzMethod == 1) sMethodDefuz = "And";
		else sMethodDefuz = "Error";

		printf("=========================================================================================\n");
		printf(" Methods: \t\t\tInference -> %s\t\tDefuzzier -> %s\n", sMethodInfer, sMethodDefuz);
		printf(" Normalized Values: \t\tIn1 -> %d\t Int2 -> %d\t Out -> %d\n", nNormedIn1, nNormedIn2, stDccFuzzy.nOutNormed);
		printf(" Activated Linguistic for In1: \t%d \t\t%d \t\t%d \t\t%d\n", stDccFuzzy.nLingActivated[0][0], stDccFuzzy.nLingActivated[1][0], stDccFuzzy.nLingActivated[2][0], stDccFuzzy.nLingActivated[3][0]);
		printf(" Activated Linguistic for In2: \t%d \t\t%d \t\t%d \t\t%d\n", stDccFuzzy.nLingActivated[0][1], stDccFuzzy.nLingActivated[1][1], stDccFuzzy.nLingActivated[2][1], stDccFuzzy.nLingActivated[3][1]);
		printf(" Activated Linguistic for Out: \t%d \t\t%d \t\t%d \t\t%d\n", stDccFuzzy.nLingActivated[0][2], stDccFuzzy.nLingActivated[1][2], stDccFuzzy.nLingActivated[2][2], stDccFuzzy.nLingActivated[3][2]);
		printf(" Membership Value of In1: \t%f \t%f \t%f \t%f\n", stDccFuzzy.dLingMemVal[0][0], stDccFuzzy.dLingMemVal[1][0], stDccFuzzy.dLingMemVal[2][0], stDccFuzzy.dLingMemVal[3][0]);
		printf(" Membership Value of In2: \t%f \t%f \t%f \t%f\n", stDccFuzzy.dLingMemVal[0][1], stDccFuzzy.dLingMemVal[1][1], stDccFuzzy.dLingMemVal[2][1], stDccFuzzy.dLingMemVal[3][1]);
		printf(" Membership Value of Out: \t%f \t%f \t%f \t%f\n\n", stDccFuzzy.dLingMemVal[0][2], stDccFuzzy.dLingMemVal[1][2], stDccFuzzy.dLingMemVal[2][2], stDccFuzzy.dLingMemVal[3][2]);
	}
	return fnnInverseNormalization(stDccFuzzy.nOutNormed, __NormVal, stDccFuzzy.dMinMaxOut);
}