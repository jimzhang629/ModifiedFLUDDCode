/**
 *  @file storage.h
 *  Implements code to wait for new data to store, and to locally store the
 *  data
 *
 *  Created on: Jan 22, 2021
 *  Author: danieldegrasse
 */

#ifndef STORAGE_H_
#define STORAGE_H_

#include "common.h"
#include <xdc/std.h>

/**
 * Configuration values for the storage task
 */
#define STORAGE_TASK_STACK_MEM 4096 /**< size of the task stack */
#define STORAGE_TASK_PRIORITY 1     /**< priority of task */

/**
 * This function should perform any initialization required for the storage
 * module to function, including initializing peripherals like SPI.
 */
void storage_init();

/**
 * This function should run in a continuous loop, and handle all data
 * storage on the device.
 * @param arg0: Unused
 * @param arg1: Unused
 */
void storage_run(UArg arg0, UArg arg1);

/**
 * Stores sensor data to the SD card.
 * @param packet Data packet to store
 */
void store_sensor_data(SensorDataPacket *packet);

/**
 * Logs data onto the SD card. Logs asynchronously.
 * @param logstr: string to log
 */
void log_sdcard(const char *logstr);

/**
 * Unmount the SD card, so it can be removed from the system.
 */
void unmount_sdcard();

/**
 * This function posts an event to the storage thread so that it will attempt
 * to mount the SD card
 */
void request_sd_mount();

/**
 * Reads the program configuration from config.txt,
 * and populates the configuration structure.
 */
void read_configuration();

/**
 * forces all open files to write to the attached disk
 */
void sync_to_disk();

/**
 * Sets the radar offset into the configuration file.
 * The strategy used here is to copy the current configuration file to
 * a new file line by line, and splice in the new offset when required. Once
 * the new config file is created, copy it over the old one.
 * This method would also work to set other configuration values.
 * @param offset: float specifying new offset
 */
void set_radar_offset(float offset);

#endif /* STORAGE_H_ */
