#include "obcore/base/types.h"

#if _OBVIOUS_DOUBLE_PRECISION_
#define TSDGRIDMAXWEIGHT 32.0
#define TSDSPACEMAXWEIGHT 32.0
#define TSDINC 1.0
#define TSDZERO 0.0
#else
#define TSDGRIDMAXWEIGHT 32.f
#define TSDSPACEMAXWEIGHT 32.f
#define TSDINC 1.f
#define TSDZERO 0.f
#endif
