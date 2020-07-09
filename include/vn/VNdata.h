/******************************************************************************/
/*****                              UARTcom.h                             *****/
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
#include <time.h>

#define DEFAULT_UART_PORT "/dev/ttyS4"
#define HEADER_LENGTH     8
#define MSG_LENGTH        (106 - HEADER_LENGTH)

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
} Attitude;

typedef struct {
        double latitude;
        double longitude;
        double altitude;
} PosLLA;

typedef struct {
        Point3F accel;
        Point3F gyro;
        Attitude attitude;
        PosLLA poslla;
        Point3D posecef;
        Point3F velbody;
} VNMeasures;

void init_vn(const char*);
void get_vn_data(VNMeasures*);

const char write_ascii_off[]  = {0x24, 0x56, 0x4E, 0x57, 0x52, 0x47, 0x2C, 0x36, 0x2C, 0x30, 0x2C, 0x31, 0x2A, 0x42, 0x31, 0x42, 0x34, 0x0D, 0x0A};
const char write_binary_off[] = {0x24, 0x56, 0x4E, 0x57, 0x52, 0x47, 0x2C, 0x37, 0x35, 0x2C, 0x31, 0x2C, 0x37, 0x2C, 0x30, 0x2A, 0x36, 0x33, 0x45, 0x39, 0x0D, 0x0A};
const char write_vn_config[]  = {0x24, 0x56, 0x4E, 0x57, 0x52, 0x47, 0x2C, 0x37, 0x35, 0x2C, 0x31, 0x2C, 0x38, 0x2C, 0x33, 0x34, 0x2C, 0x30, 0x34, 0x30, 0x30, 0x2C, 0x30, 0x30, 0x34, 0x32, 0x2C, 0x30, 0x30, 0x30, 0x45, 0x2A, 0x46, 0x33, 0x31, 0x43, 0x0D, 0x0A};
const char measures_header[]  = {0xFA, 0x34, 0x00, 0x04, 0x42, 0x00, 0x0E, 0x00};
