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

        vn->gyro.x           = *(float* )(vn_msg     );     // rad/s
        vn->gyro.y           = *(float* )(vn_msg+4* 1);     // rad/s
        vn->gyro.z           = *(float* )(vn_msg+4* 2);     // rad/s
        vn->attitude.yaw     = *(float* )(vn_msg+4* 3);     // deg
        vn->attitude.pitch   = *(float* )(vn_msg+4* 4);     // deg
        vn->attitude.roll    = *(float* )(vn_msg+4* 5);     // deg
        vn->accel.x          = *(float* )(vn_msg+4* 6);     // m/s2
        vn->accel.y          = *(float* )(vn_msg+4* 7);     // m/s2
        vn->accel.z          = *(float* )(vn_msg+4* 8);     // m/s2
        vn->poslla.latitude  = *(double*)(vn_msg+4* 9);     // deg
        vn->poslla.longitude = *(double*)(vn_msg+4*11);     // deg
        vn->poslla.altitude  = *(double*)(vn_msg+4*13);     // m
        vn->posecef.x        = *(double*)(vn_msg+4*15);     // m
        vn->posecef.y        = *(double*)(vn_msg+4*17);     // m
        vn->posecef.z        = *(double*)(vn_msg+4*19);     // m
        vn->velbody.x        = *(float* )(vn_msg+4*21);     // m/s
        vn->velbody.y        = *(float* )(vn_msg+4*22);     // m/s
        vn->velbody.z        = *(float* )(vn_msg+4*23);     // m/s

/*
	// DEBUG
        for(i = 0; i < MSG_LENGTH; i++){
		printf("%x", vn_msg[i]);
	}
	printf("\n");

	printf("Gyro: %f %f %f\n", vn->gyro.x, vn->gyro.y, vn->gyro.z);
	printf("Attitude: %f %f %f\n", vn->attitude.yaw, vn->attitude.pitch, vn->attitude.roll);
	printf("Accel: %f %f %f\n", vn->accel.x, vn->accel.y, vn->accel.z);
	printf("PosLLA: %f %f %f\n", vn->poslla.latitude, vn->poslla.longitude, vn->poslla.altitude);
	printf("PosECEF: %f %f %f\n", vn->posecef.x, vn->posecef.y, vn->posecef.z);
	printf("VelBody: %f %f %f\n", vn->velbody.x, vn->velbody.y, vn->velbody.z);
*/
}
