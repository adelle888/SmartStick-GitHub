#ifndef GPS_H
#define GPS_H

#include <stdint.h>
#include <stdbool.h>    /* 解决 true/false 未定义 */
#include <stdlib.h>     /* 解决 atof 未定义 */

#define GPS_Buffer_Length 128

typedef struct
{
    char GPS_Buffer[GPS_Buffer_Length];
    char UTCTime[12];
    char latitude[15];
    char N_S[3];
    char longitude[15];
    char E_W[3];
    bool isGetData;
    bool isParseData;
    bool isUsefull;
} _SaveData;

/* 全局变量声明（定义在 app_freertos.c） */
extern _SaveData Save_Data;
extern char gps_lat_str[16];
extern char gps_lon_str[16];
extern uint8_t gps_valid;

/* 函数声明 */
void parseGpsBuffer(void);
void printGpsBuffer(void);

#endif
