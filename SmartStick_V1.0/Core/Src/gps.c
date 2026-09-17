#include "gps.h"
#include "usart.h"
#include "stdio.h"
#include "string.h"
#include <stdlib.h>
#include <stdbool.h> 

_SaveData Save_Data;

void parseGpsBuffer()
{
    char *subString;
    char *subStringNext;
    uint8_t i;

    if (!Save_Data.isGetData)
        return;

    Save_Data.isGetData = false;

    memset(Save_Data.latitude,0,sizeof(Save_Data.latitude));
    memset(Save_Data.longitude,0,sizeof(Save_Data.longitude));
    memset(Save_Data.N_S,0,sizeof(Save_Data.N_S));
    memset(Save_Data.E_W,0,sizeof(Save_Data.E_W));

    if(strncmp(Save_Data.GPS_Buffer,"$GNRMC",6)!=0 &&
       strncmp(Save_Data.GPS_Buffer,"$GPRMC",6)!=0)
    {
        return;
    }

    subString = Save_Data.GPS_Buffer;

    for(i=0;i<=6;i++)
    {
        subString = strchr(subString,',');

        if(subString == NULL)
            return;

        subString++;

        subStringNext = strchr(subString,',');

        if(subStringNext == NULL)
            return;

        int len = subStringNext - subString;

        switch(i)
        {
            case 0:
                if(len>sizeof(Save_Data.UTCTime)-1)
                    len = sizeof(Save_Data.UTCTime)-1;

                memcpy(Save_Data.UTCTime,subString,len);
                Save_Data.UTCTime[len] = '\0';
            break;

            case 1:
                if(subString[0]=='A')
                    Save_Data.isUsefull = true;
                else
                    Save_Data.isUsefull = false;
            break;

            case 2:
                if(len>sizeof(Save_Data.latitude)-1)
                    len = sizeof(Save_Data.latitude)-1;

                memcpy(Save_Data.latitude,subString,len);
                Save_Data.latitude[len] = '\0';
            break;

            case 3:
                memcpy(Save_Data.N_S,subString,1);
                Save_Data.N_S[1] = '\0';
            break;

            case 4:
                if(len>sizeof(Save_Data.longitude)-1)
                    len = sizeof(Save_Data.longitude)-1;

                memcpy(Save_Data.longitude,subString,len);
                Save_Data.longitude[len] = '\0';
            break;

            case 5:
                memcpy(Save_Data.E_W,subString,1);
                Save_Data.E_W[1] = '\0';
            break;

            default:
            break;
        }

        subString = subStringNext;
    }

    Save_Data.isParseData = true;
}

void printGpsBuffer()
{
    if (!Save_Data.isParseData)
        return;
    Save_Data.isParseData = false;

    if (!Save_Data.isUsefull)
    {
        printf("GPS waiting\r\n");
        gps_valid = 0;
        return;
    }

    double rawLat = atof(Save_Data.latitude);
    double rawLon = atof(Save_Data.longitude);
    int latDeg = (int)(rawLat / 100);
    int lonDeg = (int)(rawLon / 100);
    double latMin = rawLat - latDeg * 100;
    double lonMin = rawLon - lonDeg * 100;
    double lat = latDeg + latMin / 60.0;
    double lon = lonDeg + lonMin / 60.0;

    if (Save_Data.N_S[0] == 'S') lat = -lat;
    if (Save_Data.E_W[0] == 'W') lon = -lon;
    if (lat < -90  || lat > 90)  return;
    if (lon < -180 || lon > 180) return;

    printf("Lat:%.6f Lon:%.6f GPS OK\r\n", lat, lon);

    /* 转换为度分秒格式存入全局变量，供OLED和SMS使用 */
    /* 经度（lat）：度.分.秒 */
    int lat_d = (int)lat;
    int lat_m = (int)((lat - lat_d) * 60);
    int lat_s = (int)(((lat - lat_d) * 60 - lat_m) * 60);
    snprintf(gps_lat_str, sizeof(gps_lat_str), "%d.%02d.%02d", lat_d, lat_m, lat_s);

    /* 纬度（lon）：度.分.秒 */
    int lon_d = (int)lon;
    int lon_m = (int)((lon - lon_d) * 60);
    int lon_s = (int)(((lon - lon_d) * 60 - lon_m) * 60);
    snprintf(gps_lon_str, sizeof(gps_lon_str), "%d.%02d.%02d", lon_d, lon_m, lon_s);

    gps_valid = 1;
}
