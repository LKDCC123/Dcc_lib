#pragma once

#include <math.h>
#include "dcc_Mat.h"
#include "dcc_con_base.h"

void fnvObtainTransMat3(double * dptTransMat, char cAxis, double dQ, double dPos[3]);
void fnvSO32Eul(double * dRotIn, double * dRulOut);
void fnvSO32Qua(double * dRotIn, double * dQuaOut);