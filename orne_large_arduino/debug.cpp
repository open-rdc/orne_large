#include "debug.h"

#ifdef DEBUG

//void DEBUG_PRINT(const char *str){
//    nh.loginfo(str);
//}

void DEBUG_PRINT(const char *format, ...){
    char str[256];
    va_list va;
    va_start(va, format);
    // int vprintf(const char *format, va_list ap);
    vsprintf(str, format, va);
    va_end(va);
    nh.loginfo(str);
}

#endif
