#pragma once
// dcc 20210525 bit
#ifndef DCC_GET_KEY_H
#define DCC_GET_KEY_H
#ifdef DCC_GET_KEY_C
#define Extern 
#else 
#define Extern extern
#endif

#include <windows.h>
#include <winuser.h>

#ifndef __KeyNum
#define __KeyNum 26
#endif
#ifndef __KeyPressed 
#define __KeyPressed _kbhit()
#endif
#ifndef __R_Ascii
#define __R_Ascii 0x52
#endif
#ifndef __A_Ascii
#define __A_Ascii 0x41
#endif
#ifndef __S_Ascii
#define __S_Ascii 0x53
#endif
#ifndef __D_Ascii
#define __D_Ascii 0x44
#endif
#ifndef __W_Ascii
#define __W_Ascii 0x57
#endif
#ifndef __U_Ascii
#define __U_Ascii 0x55
#endif
#ifndef __J_Ascii
#define __J_Ascii 0x4A
#endif
#ifndef __I_Ascii
#define __I_Ascii 0x49
#endif
#ifndef __K_Ascii
#define __K_Ascii 0x4B
#endif
#ifndef __Z_Ascii
#define __Z_Ascii 0x5A
#endif
#ifndef __X_Ascii
#define __X_Ascii 0x58
#endif

Extern enum SustainInputKeyName { // 
	Key_A, Key_S, Key_D, Key_W, SustainInputKeyNum
};
Extern enum DiceretInputKeyName {
	Key_U, Key_J, Key_I, Key_K, Key_R, Key_Z, Key_X, DiceretInputKeyNum
};
Extern int nSusInPressFlag[__KeyNum]; // pressed flag for sustaining input keys
Extern int nDicInPressFlag[__KeyNum]; // pressed flag for dicereting input keys
Extern int nDicInReleaFlag[__KeyNum]; // release flag for dicereting input keys
Extern int nInputNum;

#undef Extern 

void fnvKeyInit();
void fnvCheckKey(int nDispFlag);

#endif