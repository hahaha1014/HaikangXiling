#ifndef SAMPLE_COMMON_COMMON_HPP_
#define SAMPLE_COMMON_COMMON_HPP_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <vector>
#include <fstream>
#include <iterator>
#include <conio.h>
#include <process.h>

#include "Mv3dRgbdApi.h"
#include "Mv3dRgbdDefine.h"
#include "Mv3dRgbdImgProc.h"

#ifndef ASSERT
#define ASSERT(x)   do{ \
                if(!(x)) { \
                    LOGE("Assert failed at %s:%d", __FILE__, __LINE__); \
                    LOGE("Source Code: " #x ); \
                    system("pause"); \
                    exit(0); \
                                } \
                        }while(0)
#endif

#ifndef ASSERT_OK
#define ASSERT_OK(x)    do{ \
                int err = (x); \
                if(err != MV3D_RGBD_OK) { \
                LOGE("Assert failed: error %#x at %s:%d", err, __FILE__, __LINE__); \
                    LOGE("Source Code: " #x ); \
                    system("pause"); \
                    exit(0); \
                                } \
                        }while(0)
#endif


#ifdef _WIN32
# include <windows.h>
# include <time.h>
static inline char* getLocalTime()
{
    static char local[26] = { 0 };
    SYSTEMTIME wtm;
    struct tm tm;
    GetLocalTime(&wtm);
    tm.tm_year = wtm.wYear - 1900;
    tm.tm_mon = wtm.wMonth - 1;
    tm.tm_mday = wtm.wDay;
    tm.tm_hour = wtm.wHour;
    tm.tm_min = wtm.wMinute;
    tm.tm_sec = wtm.wSecond;
    tm.tm_isdst = -1;

    strftime(local, 26, "%Y-%m-%d %H:%M:%S", &tm);

    return local;
}

#else
# include <sys/time.h>
# include <unistd.h>
static inline char* getLocalTime()
{
    static char local[26] = { 0 };
    time_t time;

    struct timeval tv;
    gettimeofday(&tv, NULL);

    time = tv.tv_sec;
    struct tm* p_time = localtime(&time);
    strftime(local, 26, "%Y-%m-%d %H:%M:%S", p_time);

    return local;
}
#endif

#define LOG(fmt,...)   printf(fmt "\n", ##__VA_ARGS__)
#define LOGD(fmt,...)  printf("(%s) " fmt "\n", getLocalTime(), ##__VA_ARGS__)
#define LOGE(fmt,...)  printf("(%s) Error: " fmt "\n", getLocalTime(), ##__VA_ARGS__)


#endif