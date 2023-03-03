// basic tools based on dcc_con_base.c
// 20230303 dcc <3120195094@bit.edu.cn>
#ifndef DBASE_HPP
#define DBASE_HPP
#include "BaseHeaders.h"
#include <DMat.hpp>

_D_BASE_BEGIN

// for timer ================================================
LARGE_INTEGER nFreq;
LARGE_INTEGER nBeginTime;
LARGE_INTEGER nEndTime;
double time_start, time_end, time_;
double j_timer = 10.0;
// for timer ================================================

/** Limitation
* InputVal: Data, Limit[min, max]
* OutputVal: DataLimited
*/
double fndLimit(double dInputData, double dLimit[2]) {
	double dOutputData;
	if (dLimit[0] == dLimit[1]) {
		return dInputData;
	}
	else {
		dOutputData = fmax(dLimit[0], dInputData);
		dOutputData = fmin(dLimit[1], dOutputData);
		return dOutputData;
	}
}



_D_BASE_END

#endif 