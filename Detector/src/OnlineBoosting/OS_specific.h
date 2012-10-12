#ifndef OS_SPECIFIC_H
#define OS_SPECIFIC_H

//include platform specific headers
#include <sys/time.h>
#include <unistd.h>
#include <stdint.h>
#include <vector>
#include <string>
#include <map>
#include <cstdio>
//#include <QtCore>

namespace Detector{
//MAX_PATH variable used by helmut
#ifdef MAX_PATH /* Work-around for Mingw */
#undef MAX_PATH
#endif /* MAX_PATH */
#define MAX_PATH 260
typedef int32_t  __int32;
typedef int64_t  __int64;
typedef uint32_t  __uint32;
typedef uint64_t  __uint64;
}

#endif // OS_SPECIFIC_H

