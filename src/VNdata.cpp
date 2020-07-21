/******************************************************************************/
/*****                              VNdata.c                              *****/
/*****                      Raul Tapia && Jesus Tormo                     *****/
/*****                          GRIFFIN Project                           *****/
/*****                         GRVC-Robotics Lab.                         *****/
/******************************************************************************/

/**
 * @file    VNdata.c
 * @brief   Function to get measures from VectorNav sensor
 * @author  Raul Tapia && Jesus Tormo
 */

#include "VNdata.h"

int serial_port_UART = 0;

void init_vn(const char *port){
        serial_port_UART = open_uart(port);

        write(serial_port_UART, write_ascii_off, sizeof(write_ascii_off));
        write(serial_port_UART, write_binary_off, sizeof(write_binary_off));
        write(serial_port_UART, write_vn_config, sizeof(write_vn_config));
}

void get_vn_data(VNMeasures *vn){
        char read_buffer[500];
        char vn_msg[MSG_LENGTH];
        unsigned int i, k, n, ok = 0;

	while(!ok){
		do{
		        read(serial_port_UART, read_buffer, 1);
		} while(read_buffer[0] != measures_header[0]);

		ok = 1;
		for(i = 0; ok && i < HEADER_LENGTH-1; i++){
		        read(serial_port_UART, read_buffer, 1);
			if(read_buffer[0] != measures_header[i+1]){
				ok = 0;
			}
		}
	}

        for(i = 0; i < MSG_LENGTH; i++) {
		n = read(serial_port_UART, read_buffer, 1);
		vn_msg[i] = read_buffer[0];
        }

	vn->gpstime	     = *(double*)(vn_msg     );     // nsec (to sec)
        vn->mag.x            = *(float* )(vn_msg+4* 2);     // gauss
        vn->mag.y            = *(float* )(vn_msg+4* 3);     // gauss
        vn->mag.z            = *(float* )(vn_msg+4* 4);     // gauss
        vn->accel.x          = *(float* )(vn_msg+4* 5);     // m/s2
        vn->accel.y          = *(float* )(vn_msg+4* 6);     // m/s2
        vn->accel.z          = *(float* )(vn_msg+4* 7);     // m/s2
        vn->gyro.x           = *(float* )(vn_msg+4* 8);     // rad/s
        vn->gyro.y           = *(float* )(vn_msg+4* 9);     // rad/s
        vn->gyro.z           = *(float* )(vn_msg+4*10);     // rad/s
        vn->attitude.yaw     = *(float* )(vn_msg+4*11);     // deg (to rad)
        vn->attitude.pitch   = *(float* )(vn_msg+4*12);     // deg (to rad)
        vn->attitude.roll    = *(float* )(vn_msg+4*13);     // deg (to rad)
        vn->accbody.x        = *(float* )(vn_msg+4*14);     // m/s2
        vn->accbody.y        = *(float* )(vn_msg+4*15);     // m/s2
        vn->accbody.z        = *(float* )(vn_msg+4*16);     // m/s2
        vn->poslla.latitude  = *(double*)(vn_msg+4*17);     // deg
        vn->poslla.longitude = *(double*)(vn_msg+4*19);     // deg
        vn->poslla.altitude  = *(double*)(vn_msg+4*21);     // m
        vn->posecef.x        = *(double*)(vn_msg+4*23);     // m
        vn->posecef.y        = *(double*)(vn_msg+4*25);     // m
        vn->posecef.z        = *(double*)(vn_msg+4*27);     // m
        vn->velbody.x        = *(float* )(vn_msg+4*29);     // m/s
        vn->velbody.y        = *(float* )(vn_msg+4*30);     // m/s
        vn->velbody.z        = *(float* )(vn_msg+4*31);     // m/s

        vn->gpstime	     *= 1e-9;		// sec
        vn->attitude.yaw     *= PI/180.0;	// rad
        vn->attitude.pitch   *= PI/180.0;	// rad
        vn->attitude.roll    *= PI/180.0;	// rad

/*
	// DEBUG
	printf("GPS Time: %f \n", vn->gpstime);
	printf("Attitude: %f %f %f\n", vn->attitude.yaw, vn->attitude.pitch, vn->attitude.roll);
	printf("Gyro: %f %f %f\n", vn->gyro.x, vn->gyro.y, vn->gyro.z);
	printf("Accel: %f %f %f\n", vn->accel.x, vn->accel.y, vn->accel.z);
	printf("Mag: %f %f %f\n", vn->mag.x, vn->mag.y, vn->mag.z);
	printf("PosLLA: %f %f %f\n", vn->poslla.latitude, vn->poslla.longitude, vn->poslla.altitude);
	printf("PosECEF: %f %f %f\n", vn->posecef.x, vn->posecef.y, vn->posecef.z);
	printf("VelBody: %f %f %f\n", vn->velbody.x, vn->velbody.y, vn->velbody.z);
	printf("AccBody: %f %f %f\n\n", vn->accel.x, vn->accel.y, vn->accel.z);
*/
}
