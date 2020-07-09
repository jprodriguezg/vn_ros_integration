/******************************************************************************/
/*****                              UARTcom.h                             *****/
/*****                      Raul Tapia && Jesus Tormo                     *****/
/*****                          GRIFFIN Project                           *****/
/*****                         GRVC-Robotics Lab.                         *****/
/******************************************************************************/

/**
 * @file    UARTcom.h
 * @brief   Header file for UARTcom.c
 * @author  Raul Tapia && Jesus Tormo
 */

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <errno.h>

#define SET_ON |=
#define SET_OFF &=~

#define INPUT_BAUD_RATE  B115200
#define OUTPUT_BAUD_RATE B115200

int open_uart(const char*);
void config_uart_modes(struct termios*);
