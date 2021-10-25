#pragma once
#ifndef DCC_FUZZY_H
#define DCC_FUZZY_H
#endif
#ifdef DCC_FUZZY_C
#define Extern 
#else
#define Extern extern
#endif

#define __MaxLingNum 7
#define __NormVal 1000 // normalize the 

typedef struct {
	int nInputNum;
	int nLingNum[3]; // [In1, In2, Out]
	int nFuzzyBase[__MaxLingNum][__MaxLingNum]; // [In1][In2]
	double dMemConfigIn1[__MaxLingNum][3];
	double dMemConfigIn2[__MaxLingNum][3];
	double dMemConfigOut[__MaxLingNum][3];
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
	int nInferMethod; // 0 - min, 1 - max
	int nDefuzMethod; // 0 - sum, 1 - and
}dccFuzzy;

Extern dccFuzzy stDccFuzzy;

void fnvFuzzyConInit(int *nptFuzzyBase_in, double *dptMemConfig_in1, double *dptMemConfig_in2, double *dptMemConfig_out, int nLingNum_in[3], int nMethods[2]);
double fndCalFuzzyConVal(double dIn1, double dIn2, int nDispFlag);

#undef Extern
