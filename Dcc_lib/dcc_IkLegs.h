#pragma once
// 20211018 bit
#ifndef DCC_IKLEG_H
#define DCC_IKLEG_H
#ifdef DCC_IKLEG_C
#define Extern 
#else 
#define Extern extern 
#endif

Extern double dHipPR[6], dRAnkPR[6], dLAnkPR[6]; // x, y, z, pit, rol, yaw

#undef Extern

void fnvInitIkLegs();
void fnvCalIkLegs();

#include "Base\dcc_Ik.h"

#endif