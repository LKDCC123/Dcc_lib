// BaseHeaders.h

#pragma once
#ifndef _CRT_SECURE_NO_WARNINGS
#define _CRT_SECURE_NO_WARNINGS
#endif
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <Windows.h>

#define __PI 3.1415926
#ifndef __R2D
#define __R2D(a) a * 180.0 / __PI
#endif
#ifndef __D2R
#define __D2R(a) a * __PI / 180.0
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

#define _D_BASE_BEGIN namespace Dcc { namespace BASE_FILES {
#define _D_BASE_END }}
#define _D_BASE ::Dcc::BASE_FILES::
#define _D_USING_BASE using namespace Dcc::BASE_FILES;
