// DFuzzy.hpp
// dcc fuzzy lib based on dcc_FuzzyCon.h, which can handle a 2 input linguistic and 1 output fuzzy logic problem
// 20230304 dcc <3120195094@bit.eud.cn>

#pragma once 
#ifndef DFUZZY_HPP
#define DFUZZY_HPP
#include "BaseHeaders.h"

_D_BASE_BEGIN

#define __MaxLingNum 7 // maximum linguistic number
#define __NormVal 1000 // normalize the input and output range

struct st_FuzzyInit {
    int nFuzzyBase[__MaxLingNum][__MaxLingNum];
    double dMemConfigIn1[__MaxLingNum][3];
    double dMemConfigIn2[__MaxLingNum][3];
    double dMemConfigOut[__MaxLingNum][3];
    int nLingNum[3]; // [In1, In2, Out]
    int nMethod[2]; // [nInferMethod: 0 - min, 1 - max, nDefuzMethod: 0 - sum, 1 - and]
	int nInputNum; 
};

struct st_FuzzyCalData {
	double dMemIn1[__MaxLingNum][__NormVal];
	double dMemIn2[__MaxLingNum][__NormVal];
	double dMemOut[__MaxLingNum][__NormVal];
	double dMinMaxIn1[2]; // [min, max]
	double dMinMaxIn2[2]; // [min, max]
	double dMinMaxOut[2]; // [min, max]
	int nLingActivated[4][3]; // [state: 1, 2, 3, 4][In1, In2, Out], if max overlaped ling is 2, the max state is 4
	double dLingMemVal[4][3]; // [state: 1, 2, 3, 4][In1, In2, Out], if max overlaped ling is 2, the max state is 4
	double dOutMemVal[5][__NormVal]; // [state: 1, 2, 3, 4, Out]
	int nOutNormed;
};

class c_FuzzyCon {
public:
    c_FuzzyCon(st_FuzzyInit * stFuzzyInitIn) {
        this->stptInit = stFuzzyInitIn;
        // init boarder of normalization
        this->stCalData.dMinMaxIn1[0] = this->stptInit->dMemConfigIn1[0][0];
        this->stCalData.dMinMaxIn1[1] = this->stptInit->dMemConfigIn1[this->stptInit->nLingNum[0] - 1][2];
        this->stCalData.dMinMaxIn2[0] = this->stptInit->dMemConfigIn2[0][0];
        this->stCalData.dMinMaxIn2[1] = this->stptInit->dMemConfigIn2[this->stptInit->nLingNum[1] - 1][2];
        this->stCalData.dMinMaxOut[0] = this->stptInit->dMemConfigOut[0][0];
        this->stCalData.dMinMaxOut[1] = this->stptInit->dMemConfigOut[this->stptInit->nLingNum[2] - 1][2];
        // make membership
        this->fnvMakeMembership();
    }
protected:
    st_FuzzyCalData stCalData;
    st_FuzzyInit * stptInit;
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
            nNormMemConfigTempIn1[i] = fnnNormalization(this->stptInit->dMemConfigIn1[0][i], this->stCalData.dMinMaxIn1, __NormVal);
            nNormMemConfigTempIn2[i] = fnnNormalization(this->stptInit->dMemConfigIn2[0][i], this->stCalData.dMinMaxIn2, __NormVal);
            nNormMemConfigTempOut[i] = fnnNormalization(this->stptInit->dMemConfigOut[0][i], this->stCalData.dMinMaxOut, __NormVal);
        }
        fnvMakeTriangle(this->stCalData.dMemIn1[0], nNormMemConfigTempIn1, 'L', __NormVal);
        fnvMakeTriangle(this->stCalData.dMemIn2[0], nNormMemConfigTempIn2, 'L', __NormVal);
        fnvMakeTriangle(this->stCalData.dMemOut[0], nNormMemConfigTempOut, 'L', __NormVal);
        // midle linguist
        for (j = 1; j < this->stptInit->nLingNum[0] - 1; j++) {
            for (i = 0; i < 3; i++) nNormMemConfigTempIn1[i] = fnnNormalization(this->stptInit->dMemConfigIn1[j][i], this->stCalData.dMinMaxIn1, __NormVal);
            fnvMakeTriangle(this->stCalData.dMemIn1[j], nNormMemConfigTempIn1, 'M', __NormVal);
        }
        for (j = 1; j < this->stptInit->nLingNum[1] - 1; j++) {
            for (i = 0; i < 3; i++) nNormMemConfigTempIn2[i] = fnnNormalization(this->stptInit->dMemConfigIn2[j][i], this->stCalData.dMinMaxIn2, __NormVal);
            fnvMakeTriangle(this->stCalData.dMemIn2[j], nNormMemConfigTempIn2, 'M', __NormVal);
        }
        for (j = 1; j < this->stptInit->nLingNum[2] - 1; j++) {
            for (i = 0; i < 3; i++) nNormMemConfigTempOut[i] = fnnNormalization(this->stptInit->dMemConfigOut[j][i], this->stCalData.dMinMaxOut, __NormVal);
            fnvMakeTriangle(this->stCalData.dMemOut[j], nNormMemConfigTempOut, 'M', __NormVal);
        }
        // right linguist
        for (i = 0; i < 3; i++) {
            nNormMemConfigTempIn1[i] = fnnNormalization(this->stptInit->dMemConfigIn1[this->stptInit->nLingNum[0] - 1][i], this->stCalData.dMinMaxIn1, __NormVal);
            nNormMemConfigTempIn2[i] = fnnNormalization(this->stptInit->dMemConfigIn2[this->stptInit->nLingNum[1] - 1][i], this->stCalData.dMinMaxIn2, __NormVal);
            nNormMemConfigTempOut[i] = fnnNormalization(this->stptInit->dMemConfigOut[this->stptInit->nLingNum[2] - 1][i], this->stCalData.dMinMaxOut, __NormVal);
        }
        fnvMakeTriangle(this->stCalData.dMemIn1[this->stptInit->nLingNum[0] - 1], nNormMemConfigTempIn1, 'R', __NormVal);
        fnvMakeTriangle(this->stCalData.dMemIn2[this->stptInit->nLingNum[1] - 1], nNormMemConfigTempIn2, 'R', __NormVal);
        fnvMakeTriangle(this->stCalData.dMemOut[this->stptInit->nLingNum[2] - 1], nNormMemConfigTempOut, 'R', __NormVal);
    }
};

_D_BASE_END

#endif
