#pragma once
// 20211007 bit
#ifndef DCC_IK_H
#define DCC_IK_H
#ifdef DCC_IK_C
#define Extern 
#else
#define Extern extern
#endif

#define __nMaxDoF 10
Extern double dQTest[__nMaxDoF], dPosiTest[3], dOritTest[3], dStabFlagTest; // test
Extern double dQRleg[__nMaxDoF], dPosiRAnk[3], dOritRAnk[3], dStabFlagRLeg; // R leg
Extern double dQLleg[__nMaxDoF], dPosiLAnk[3], dOritLAnk[3], dStabFlagLLeg; // L leg
// tbc ...

#undef Extern

void fnvIkSugiInit();
void fnvIkSugiOL();

#endif 