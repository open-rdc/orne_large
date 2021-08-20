#ifndef DEBUG_H
#define DEBUG_H

//#define DEBUG 1

#ifdef DEBUG

#include "ros.h"
extern ros::NodeHandle  nh;

//void DEBUG_PRINT(const char *str);
void DEBUG_PRINT(const char *format, ...);

#else

//#define DEBUG_PRINT(str)
#define DEBUG_PRINT(format, ...) {}

#endif

#endif // DEBUG_H
