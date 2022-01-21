/**
 *  @file transmission.h
 *  Implements code to wait for new data to transmit, and to send the data
 *  to a remote server
 *
 *  Created on: Jan 22, 2021
 *  Author: danieldegrasse
 */

#ifndef TRANSMISSION_H_
#define TRANSMISSION_H_

#include "common.h"
#include <xdc/std.h>

/**
 * Configuration values for the transmission task
 */
#define TRANSMISSION_TASK_STACK_MEM 4096 /**< size of the task stack */
#define TRANSMISSION_TASK_PRIORITY 1     /**< priority of task */

/**
 * This function should perform any initialization required for the transmission
 * module to function, including initializing peripherals like UART.
 */
void transmission_init();

/**
 * This function should run in a continuous loop, and handle all data
 * transmission from the device.
 * @param arg0: Unused
 * @param arg1: Unused
 */
void transmission_run(UArg arg0, UArg arg1);

/**
 * Searches for a SIM by sweeping various baud rates
 * @return: true if sim is found
 */
bool find_sim();

/**
 * Requests for the transmission task to update the RTC
 */
void request_rtc_update();

/**
 * Transmits sensor data to the backend
 * @param packet Data packet to send
 */
void transmit_sensor_data(SensorDataPacket *packet);

#endif /* TRANSMISSION_H_ */
