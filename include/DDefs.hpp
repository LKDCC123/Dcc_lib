// defines 
// 20230331 dcc <3120195094@bit.edu.cn>
#pragma once
#ifndef DDEFS_HPP
#define DDEFS_HPP
#include "BaseHeaders.h"
#ifdef DDEFS_CPP
#define Extern 
#else
#define Extern extern
#endif
#ifndef __PI
#define __PI 3.1415926
#endif
#ifndef __Gravity
#define __Gravity 9.81
#endif
#ifndef __R2D
#define __R2D(rad) ((rad) * (180.0 / __PI))
#endif
#ifndef __D2R
#define __D2R(deg) ((deg) * (__PI / 180.0))
#endif
#ifndef __Rpm2Rps
#define __Rpm2Rps(rpm) ((rpm) * (__PI / 30.0))
#endif
#ifndef __Rps2Rpm
#define __Rps2Rpm(rps) ((rps) * (30.0 / __PI))
#endif
#ifndef __KLeftToNow
#define __KLeftToNow 100 // reserved num of k to now when circuling
#endif
#ifndef __MaxKprog
#define __MaxKprog 10000
#endif
#ifndef __MaxOf
#define __MaxOf(a, b) (a > b ? a : b)
#endif
#ifndef __MinOf
#define __MinOf(a, b) (a < b ? a : b)
#endif
#ifndef __Mic6
#define __Mic6 (1e-6)
#endif
#ifndef __Mic8
#define __Mic8 (1e-8)
#endif

_D_BASE_BEGIN

Extern enum en_SupSig {
    DSup, LSup, RSup, Fly
};

Extern enum en_Euler {
    __x, __y, __z, _rl, _pt, _ya
};

using double2 = std::array<double, 2>;
using double3 = std::array<double, 3>;
using double4 = std::array<double, 4>;
using double5 = std::array<double, 5>;
using double6 = std::array<double, 6>;
template<int N>
using doubleN = std::array<double, N>;

#undef Extern

_D_BASE_END

#endif