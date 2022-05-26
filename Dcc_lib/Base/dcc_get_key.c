// dcc 20210525 bit
// check key board in unreal-time and send the command to real-time
#ifndef DCC_GET_KEY_C
#define DCC_GET_KEY_C
#include "dcc_get_key.h"

int nSusInKeyAscii[] = { __A_Ascii, __S_Ascii, __D_Ascii, __W_Ascii }; // ascii table for sustaining input keys
int nDicInKeyAscii[] = { __U_Ascii, __J_Ascii, __I_Ascii, __K_Ascii, __R_Ascii, __Z_Ascii, __X_Ascii }; // ascii table for dicereting input keys

void fnvKeyInit(){
	for (int i = 0; i < SustainInputKeyNum; i++) {
		nSusInPressFlag[i] = 0;
	}
	for (int i = 0; i < DiceretInputKeyNum; i++) {
		nDicInPressFlag[i] = 0;
		nDicInReleaFlag[i] = 1;
	}
	nInputNum = 0;
}

int fndSustainInput(int nKey) {
	return ((GetKeyState(nKey) < 0) || (GetKeyState(nKey + 0x20) < 0));
}

int fndCheckRelease(int nKey) {
	return ((GetKeyState(nKey) >= 0) && (GetKeyState(nKey + 0x20) >= 0));
}

int fndDiceretInput(int nKey, int nKeyReleasedFlag) {
	return (((GetKeyState(nKey) < 0) || (GetKeyState(nKey + 0x20) < 0)) && nKeyReleasedFlag == 1);
}

void fnvCheckKey(int nDispFlag) {
	for (int i = 0; i < SustainInputKeyNum; i++) {
		nSusInPressFlag[i] = fndSustainInput(nSusInKeyAscii[i]);
		if (nSusInPressFlag[i]) {
			nInputNum++;
			if (nDispFlag == 1) printf("%c, ", nSusInKeyAscii[i]);
			if (nDispFlag == 1 && (nInputNum % 12 == 0)) printf("\n");
		}
	}
	for (int i = 0; i < DiceretInputKeyNum; i++) {
		nDicInPressFlag[i] = fndDiceretInput(nDicInKeyAscii[i], nDicInReleaFlag[i]);
		if (nDicInPressFlag[i]) {
			nInputNum++;
			if (nDispFlag == 1) printf("%c, ", nDicInKeyAscii[i]);
			if (nDispFlag == 1 && (nInputNum % 12 == 0)) printf("\n");
		}
	}
	for (int i = 0; i < DiceretInputKeyNum; i++) {
		nDicInReleaFlag[i] = fndCheckRelease(nDicInKeyAscii[i]);
	}
}

#endif