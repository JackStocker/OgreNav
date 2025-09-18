/**
@defgroup detour Detour

Members in this module are wrappers around the standard math library
*/

#ifndef DETOURMATH_H
#define DETOURMATH_H

#include <math.h>
#include "RealTypes.h"

inline Real dtMathFabsf( Real x) { return fixedmath::abs(x); }
inline Real dtMathSqrtf( Real x) { return fixedmath::sqrt(x); }
inline Real dtMathFloorf( Real x) { return fixedmath::floor(x); }
inline Real dtMathCeilf( Real x) { return fixedmath::ceil(x); }
inline Real dtMathCosf( Real x) { return fixedmath::cos(x); }
inline Real dtMathSinf( Real x) { return fixedmath::sin(x); }
inline Real dtMathAtan2f( Real y, Real x) { return fixedmath::atan2(y, x); }
inline bool dtMathIsfinite( Real x) { return true; }

#endif
