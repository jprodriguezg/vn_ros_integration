/******************************************************************************/
/*****                              VNdata.h                              *****/
/*****                      Raul Tapia && Jesus Tormo                     *****/
/*****                          GRIFFIN Project                           *****/
/*****                         GRVC-Robotics Lab.                         *****/
/******************************************************************************/

/**
 * @file    VNdata.h
 * @brief   Header file for VNdata.c
 * @author  Raul Tapia && Jesus Tormo
 */

#include "UARTcom.h"

#define DEFAULT_UART_PORT "/dev/ttyS4"
#define HEADER_LENGTH     10
#define MSG_LENGTH        (140 - HEADER_LENGTH)
#define PI                3.14159265358979323846

typedef struct {
        float x;
        float y;
        float z;
} Point3F;

typedef struct {
        double x;
        double y;
        double z;
} Point3D;

typedef struct {
        float roll;
        float pitch;
        float yaw;
} RPY;

typedef struct {
        double latitude;
        double longitude;
        double altitude;
} LLA;

typedef struct {
        double gpstime;

        RPY attitude;
        Point3F gyro;
        Point3F accel;
        Point3F mag;

        LLA poslla;
        Point3D posecef;
        Point3F velbody;
        Point3F accbody;
} VNMeasures;

void init_vn(const char*);
void get_vn_data(VNMeasures*);

// ASCII OFF ---> $VNWRG,6,0,1*B1B4
const char write_ascii_off[]  = {0x24, 0x56, 0x4E, 0x57, 0x52, 0x47, 0x2C, 0x36, 0x2C, 0x30, 0x2C, 0x31, 0x2A, 0x42, 0x31, 0x42, 0x34, 0x0D, 0x0A};

// BINARY OFF ---> $VNWRG,75,1,7,0*63E9
const char write_binary_off[] = {0x24, 0x56, 0x4E, 0x57, 0x52, 0x47, 0x2C, 0x37, 0x35, 0x2C, 0x31, 0x2C, 0x37, 0x2C, 0x30, 0x2A, 0x36, 0x33, 0x45, 0x39, 0x0D, 0x0A};

// CONFIGURATION ---> $VNWRG,75,1,10,36,0002,0700,0042,000E*FFD7
const char write_vn_config[]  = {0x24, 0x56, 0x4E, 0x57, 0x52, 0x47, 0x2C, 0x37, 0x35, 0x2C, 0x31, 0x2C, 0x31, 0x30, 0x2C, 0x33, 0x36, 0x2C, 0x30, 0x30, 0x30, 0x32, 0x2C, 0x30, 0x37, 0x30, 0x30, 0x2C, 0x30, 0x30, 0x34, 0x32, 0x2C, 0x30, 0x30, 0x30, 0x45, 0x2A, 0x46, 0x46, 0x44, 0x37, 0x0D, 0x0A};
const char measures_header[]  = {0xFA, 0x36, 0x02, 0x00, 0x00, 0x07, 0x42, 0x00, 0x0E, 0x00};
