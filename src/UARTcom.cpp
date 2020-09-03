/******************************************************************************/
/*****                              UARTcom.c                             *****/
/*****                      Raul Tapia && Jesus Tormo                     *****/
/*****                          GRIFFIN Project                           *****/
/*****                         GRVC-Robotics Lab.                         *****/
/******************************************************************************/

/**
 * @file    UARTcom.c
 * @author  Raul Tapia && Jesus Tormo
 * @brief   Comunication functions for UART protocol
 */

#include "UARTcom.h"

int open_uart(const char *port){
        /*** --- Open port --- ***/
        int serial_port = open(port, O_RDWR);
        if (serial_port < 0) {
                printf("Error %i: %s\n", errno, strerror(errno));
        }

        /*** --- Configuration setup --- ***/
        struct termios tty;
        memset(&tty, 0, sizeof(tty));

        if(tcgetattr(serial_port, &tty) != 0) {
                printf("Error %i: %s\n", errno, strerror(errno));
        }

        /*** --- Baud rate --- ***/
        cfsetispeed(&tty, INPUT_BAUD_RATE);
        cfsetospeed(&tty, OUTPUT_BAUD_RATE);

        /*** --- Configure modes --- ***/
        config_uart_modes(&tty);

        /*** --- Saving termios struct --- ***/
        if(tcsetattr(serial_port, TCSANOW, &tty) != 0) {
                printf("Error %i: %s\n", errno, strerror(errno));
        }

        return serial_port;
}

void config_uart_modes(struct termios *t){
        /*** --- Control modes --- ***/
        t->c_cflag SET_OFF(PARENB);
        t->c_cflag SET_OFF(CSTOPB);
        t->c_cflag SET_ON(CS8);
        t->c_cflag SET_OFF(CRTSCTS);
        t->c_cflag SET_ON(CREAD|CLOCAL);

        /*** --- Local modes --- ***/
        t->c_lflag SET_OFF(ICANON);
        t->c_lflag SET_OFF(ECHO);
        t->c_lflag SET_OFF(ECHOE);
        t->c_lflag SET_OFF(ECHONL);
        t->c_lflag SET_OFF(ISIG);

        /*** --- Input modes --- ***/
        t->c_iflag SET_OFF(IXON|IXOFF|IXANY);
        t->c_iflag SET_OFF(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);

        /*** --- Output modes --- ***/
        t->c_oflag SET_OFF(OPOST);
        t->c_oflag SET_OFF(ONLCR);
}
