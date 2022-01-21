/**
 *  @file radar.h
 *  Implements communication with attached radar sensor to read water level
 *  This file should implement communication with an attached sensor, and
 *  support a notification method to make other tasks aware new sensor data
 *  was acquired
 *
 *  Created on: Jan 22, 2021
 *  Author: danieldegrasse
 */

#ifndef RADAR_H_
#define RADAR_H_

#include <xdc/std.h>

/**
 * Configuration values for the sensor task
 */
#define RADAR_TASK_STACK_MEM 2048 /**< size of the task stack */
#define RADAR_TASK_PRIORITY 1     /**< priority of task */

/**
 * This function should perform any initialization the radar sensor model needs
 * This includes setting up peripherals (like UART).
 * This is analogous to the setup() function in for Arduino
 */
void radar_init();

/**
 * This function should run forever, and poll any sensor data from the radar
 * sensor. It should then notify other processes that new sensor data is
 * available
 * @param arg0: Unused
 * @param arg1: Unused
 */
void radar_run(UArg arg0, UArg arg1);

/**
 * Asynchronously runs sampling code for radar, getting one sample
 * and transmitting it
 */
void sample_radar();

/**
 * Forces the radar to calibrate.
 */
void force_calibration();

/**
 * Enables (or disables) radar sample logging
 * @param enabled: should samples be logged to the CLI
 */
void configure_sample_logging(bool enabled);

#endif /* RADAR_H */
