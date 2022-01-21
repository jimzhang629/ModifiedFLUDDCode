/**
 *  @file storage.c
 *  Implements code to wait for new data to store, and to locally store the
 *  data
 *
 *  Created on: Jan 22, 2021
 *  Author: danieldegrasse
 */

/* xdc module headers */
#include <xdc/runtime/System.h>
#include <xdc/std.h>

/* Standard libs */
#ifdef __TI_ARM__
// Only include this if using TI compiler
#include <file.h>
#endif
#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* Ti BIOS Headers */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/gates/GateMutex.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Queue.h>

/* Ti Drivers */
#include <ti/drivers/SDFatFS.h>
/* Third part Drivers */
#include <third_party/fatfs/ffcio.h>

#include "cli.h"
#include "common.h"
#include "ti_drivers_config.h"

///@{
/**
 * Configuration file keys. These values are included in a configuration file,
 * and parsed to load into a program_config structure
 */
#define HARDWARE_ID_CONFIG_KEY "HardwareIdentifier"
#define SYNTHETIC_ID_CONFIG_KEY "UniqueIdentifier"
#define RADAR_ENABLED_KEY "RadarModuleEnabled"
#define CAMERA_ENABLED_KEY "CameraModuleEnabled"
#define NETWORK_ENABLED_KEY "NetworkModuleEnabled"
#define SERVER_IP_KEY "RemoteServerIP"
#define SERVER_AUTH_KEY "ServerAuthenticationKey"
#define RADAR_SAMPLE_INTERVAL_KEY "RadarSampleInterval"
#define RADAR_SAMPLE_COUNT_KEY "RadarSampleCount"
#define RADAR_SAMPLE_OFFSET_KEY "RadarSampleOffset"
///@}

/** String conversion macro */
#define STR_(n) #n
/** Second part of string conversion macro */
#define STR(n) STR_(n)

/** Events that can trigger action in the main storage module */
#define EVT_SENSOR_DATA_AVAIL Event_Id_00 /**< new sensor data is available */
#define EVT_SDCARD_UNMOUNT Event_Id_01 /**< the SD card should be unmounted */
#define EVT_SDCARD_MOUNT Event_Id_02   /**< the SD card should be mounted */

void storage_init();
void storage_run(UArg arg0, UArg arg1);
void store_sensor_data(SensorDataPacket *packet);
void mount_sdcard();
void request_sd_mount();
void read_configuration();
void parse_config_entry(char *key, char *value);

/** Drive number used for FatFs */
#define DRIVE_NUM 0
/** must be power of 2, max elements in queue to be stored on SD card */
#define MAX_QUEUE_ELEM 32
/** maximum length of logging line to write to SD card */
#define LOG_LINE_MAX 128

/**
 * Queue element to be placed into the sensor data queue. These elements will
 * be written to the SD card in FIFO order.
 */
typedef struct SensorDataQueueElem {
    Queue_Elem elem;         /**< Queue element, used to track queue */
    SensorDataPacket packet; /**< Sensor data packet to write to SD card */
} SensorDataQueueElem;

/** Definition for the sensor data file name (cannot be longer than 8
 * characters) */
static const char sensor_data_filename[] = "distdata.csv";
/** Configuration filename */
static const char configuration_filename[] =
    "fat:" STR(DRIVE_NUM) ":config.txt";
/** log filename */
static const char log_filename[] = "fat:" STR(DRIVE_NUM) ":log.txt";

#ifdef __TI_ARM__
/** File name prefix for this filesystem for use with TI C RTS */
static char fatfsPrefix[] = "fat";
#endif

/** Data file CSV header */
static const char FILE_HEADER[] = "Timestamp,Distance\n";

static SDFatFS_Handle sdfatfsHandle = NULL;
static FILE *data_file;
static FILE *log_file = NULL;
static Event_Handle storageEventHandle;
static Queue_Handle sensorDataQueue;
static GateMutex_Handle queueMutex;
static GateMutex_Handle sdMutex;
static SensorDataQueueElem queue_elements[MAX_QUEUE_ELEM];

/**
 * This function should perform any initialization required for the storage
 * module to function, including initializing peripherals like SPI.
 */
void storage_init() {
#ifdef __TI_ARM__
    // only required for TI compiler
    /* add_device() should be called once and is used for all media types */
    add_device(fatfsPrefix, _MSA, ffcio_open, ffcio_close, ffcio_read,
               ffcio_write, ffcio_lseek, ffcio_unlink, ffcio_rename);
#endif
    // Create the required event for this task.
    storageEventHandle = Event_create(NULL, NULL);
    if (!storageEventHandle) {
        System_abort("Could not create storage event handle\n");
    }
    sensorDataQueue = Queue_create(NULL, NULL);
    if (!sensorDataQueue) {
        System_abort("Could not create sensor data queue\n");
    }
    queueMutex = GateMutex_create(NULL, NULL);
    if (!queueMutex) {
        System_abort("Failed to create queue mutex\n");
    }
    sdMutex = GateMutex_create(NULL, NULL);
    if (!sdMutex) {
        System_abort("Failed to create sd mutex\n");
    }
    // Mount the SD card before finishing initialization.
    mount_sdcard();
    System_printf("Storage init done\n");
}

/**
 * This function should run in a continuous loop, and handle all data
 * storage on the device.
 * @param arg0: Unused
 * @param arg1: Unused
 */
void storage_run(UArg arg0, UArg arg1) {
    IArg sd_mutex_key;
    SensorDataQueueElem *elem;
    struct tm *timeinfo;
    UInt events;
    int num_printed;
    char temp_linebuf[128];
    // Should the user be updated about the sd card status
    bool storage_notification = true;
    IArg mutex_key;
    System_printf("Storage task starting\n");
    Watchdog_clear(watchdogHandle);
    while (1) {
        /*
         * Wait for an event. The call will return if any of the events
         * passed in occur, and "events" will be a bitwise OR of all events
         * that occurred.
         */
        events = Event_pend(storageEventHandle, Event_Id_NONE,
                            EVT_SENSOR_DATA_AVAIL | EVT_SDCARD_UNMOUNT |
                                EVT_SDCARD_MOUNT,
                            BIOS_WAIT_FOREVER);
        if (events & EVT_SENSOR_DATA_AVAIL) {
            // While the queue of sensor data isn't empty, read from it.
            // Get the queue mutex
            mutex_key = GateMutex_enter(queueMutex);
            while (!Queue_empty(sensorDataQueue)) {
                // pop an element from the queue.
                elem = Queue_dequeue(sensorDataQueue);
                /*
                 * Format the element data into a buffer
                 * so we can write it to the SD card
                 */
                /*
                 * Convert the timestamp to a human readable value.
                 * We are using ISO 8601 timestamps,
                 * and the timezone is UTC (+00)
                 */
                timeinfo = localtime(&(elem->packet.timestamp));
                num_printed = snprintf(temp_linebuf, sizeof(temp_linebuf),
                                       "20%d-%02d-%02dT%02d:%02d:%02d, %f\n",
                                       timeinfo->tm_year - 100,
                                       timeinfo->tm_mon, timeinfo->tm_mday,
                                       timeinfo->tm_hour, timeinfo->tm_min,
                                       timeinfo->tm_sec, elem->packet.distance);
                if (!sdfatfsHandle) {
                    // SD card is unmounted. Warn user data may be missed.
                    if (storage_notification) {
                        cli_log(
                            "WARNING: SD card is unmounted but data is still "
                            "being received\n");
                        storage_notification = false;
                    }
                    // Print the data to the cli.
                    cli_log("%s", temp_linebuf);
                } else {
                    // Enter SD card mutex
                    sd_mutex_key = GateMutex_enter(sdMutex);
                    if (fwrite(temp_linebuf, num_printed, 1, data_file) != 1) {
                        cli_log("SD card write error\n");
                    }
                    GateMutex_leave(sdMutex, sd_mutex_key);
                    if (storage_notification) {
                        cli_log(
                            "SD card is mounted and data is being written\n");
                        storage_notification = false;
                    }
                }
            }
            GateMutex_leave(queueMutex, mutex_key);
        }
        if (events & EVT_SDCARD_MOUNT) {
            if (sdfatfsHandle != NULL) {
                cli_log("Cannot mount sd card, already mounted\n");
            } else {
                mount_sdcard();
                cli_log("SD Card mounted\n");
                // reset storage notification
                storage_notification = true;
            }
        }
        if (events & EVT_SDCARD_UNMOUNT) {
            if (sdfatfsHandle == NULL) {
                cli_log("SD card already unmounted\n");
            } else {
                // Close the open file handle we have, and unmount the SD card
                fclose(data_file);
                fclose(log_file);
                SDFatFS_close(sdfatfsHandle);
                sdfatfsHandle = NULL;
                cli_log("SD Card unmounted\n");
                // reset storage notification
                storage_notification = true;
            }
        }
    }
}

/**
 * Sets the radar offset into the configuration file.
 * The strategy used here is to copy the current configuration file to
 * a new file line by line, and splice in the new offset when required. Once
 * the new config file is created, copy it over the old one.
 * This method would also work to set other configuration values.
 * @param offset: float specifying new offset
 */
void set_radar_offset(float offset) {
    FRESULT ret;
    FILE *input_config_file, *output_config_file;
    IArg sd_mutex_key;
    char file_buffer[80], *config_key, *config_value, output_buffer[80];
    const char temp_config_file[] = "fat:" STR(DRIVE_NUM) ":tmp.txt";
    // Save the new offset into the program_config structure
    program_config.radar_sample_offset = offset;
    if (!sdfatfsHandle) {
        // Warn user
        System_printf(
            "Warning: SD card not available, cannot read configuration\n");
        return;
    }
    // Get SD mutex
    sd_mutex_key = GateMutex_enter(sdMutex);
    // Open the configuration file on the SD card.
    input_config_file = fopen(configuration_filename, "r");
    if (!input_config_file) {
        // Don't fail to boot, but warn user.
        cli_log("No configuration file found, cannot set radar offset\n");
        System_printf(
            "Warning: no configuration file found, cannot set radar offset\n");
        GateMutex_leave(sdMutex, sd_mutex_key);
        return;
    }
    output_config_file = fopen(temp_config_file, "w+");
    if (!output_config_file) {
        // "w" does not assume file exists, so this error is likely bigger
        System_abort("Cannot write output config file");
    }
    // Read the configuration file data.
    while (1) {
        if (fgets(file_buffer, sizeof(file_buffer), input_config_file) ==
            NULL) {
            // Check if we reached the end of the file or had an error.
            if (ferror(input_config_file)) {
                System_abort("Error while reading configuration file\n");
            } else if (feof(input_config_file)) {
                System_printf("Configuration file copy complete\n");
                break; // Exit loop
            } else {
                System_printf("Unknown error while reading config file\n");
            }
        }
        /*
         * Here, we should parse the line we read from the configuration file.
         * Note that lines starting with '#' (and empty lines) are ignored.
         */
        if (file_buffer[0] == '#' || file_buffer[0] == '\n') {
            // Transparently write the line into the new file.
            if (fwrite(file_buffer, strlen(file_buffer), 1,
                       output_config_file) != 1) {
                System_printf("Failed to write to new config file\n");
                cli_log("Failed to write to new config file\n");
                fclose(output_config_file);
                fclose(input_config_file);
                GateMutex_leave(sdMutex, sd_mutex_key);
                return;
            }
            continue; // Do not parse configuration line
        }
        /*
         * Now parse the configuration line.
         * Lines are formatted as "key : value"
         */
        config_key = strtok(file_buffer, " : ");
        // /n removes the newline from value
        config_value = strtok(NULL, " : \n");
        if ((!config_key) || !(config_value)) {
            System_printf("Invalid configuration file line\n");
            continue;
        }
        // Check if the key is the radar sample offset for this line
        if (strncmp(config_key, RADAR_SAMPLE_OFFSET_KEY,
                    strlen(RADAR_SAMPLE_OFFSET_KEY)) == 0) {
            // Splice in new configuration value.
            snprintf(output_buffer, sizeof(output_buffer), "%s: %.3f\n",
                     config_key, offset);
        } else {
            // Simply write the same config value back to the new file
            snprintf(output_buffer, sizeof(output_buffer), "%s : %s\n",
                     config_key, config_value);
        }
        if (fwrite(output_buffer, strlen(output_buffer), 1,
                   output_config_file) != 1) {
            System_printf("Failed to write to new config file\n");
            cli_log("Failed to write to new config file\n");
            fclose(output_config_file);
            fclose(input_config_file);
            GateMutex_leave(sdMutex, sd_mutex_key);
            return;
        }
    }
    // Close both files
    fclose(input_config_file);
    fclose(output_config_file);
    // Copy output config file to overwrite input one.
    ret = f_unlink("config.txt");
    if (ret != FR_OK) {
        System_printf("Could not delete old configuration file\n");
        cli_log(
            "Could not set radar sample offset, could not delete old config "
            "file\n");
        GateMutex_leave(sdMutex, sd_mutex_key);
        return;
    }
    ret = f_rename("tmp.txt", "config.txt");
    if (ret != FR_OK) {
        System_printf("Could not overwrite configuration file\n");
        cli_log("Could not set radar sample offset, could not overwrite config "
                "file\n");
        GateMutex_leave(sdMutex, sd_mutex_key);
        return;
    }
    GateMutex_leave(sdMutex, sd_mutex_key);
    cli_log("Successfully saved radar offset to sd card\n");
}

/**
 * Reads the program configuration from config.txt,
 * and populates the configuration structure.
 */
void read_configuration() {
    FILE *config_file;
    char file_buffer[80], *config_key, *config_value;
    if (!sdfatfsHandle) {
        // Warn user
        System_printf(
            "Warning: SD card not available, cannot read configuration\n");
        return;
    }
    // Open the configuration file on the SD card.
    config_file = fopen(configuration_filename, "r");
    if (!config_file) {
        // Don't fail to boot, but warn user.
        cli_log("Warning: no configuration file found, using default values\n");
        System_printf(
            "Warning: no configuration file found, using default values\n");
        return;
    }
    // Read the configuration file data.
    while (1) {
        if (fgets(file_buffer, sizeof(file_buffer), config_file) == NULL) {
            // Check if we reached the end of the file or had an error.
            if (ferror(config_file)) {
                System_abort("Error while reading configuration file\n");
            } else if (feof(config_file)) {
                System_printf("Configuration file read\n");
                break; // Exit loop
            } else {
                System_printf("Unknown error while reading config file\n");
            }
        }
        /*
         * Here, we should parse the line we read from the configuration file.
         * Note that lines starting with '#' (and empty lines) are ignored.
         */
        if (file_buffer[0] == '#' || file_buffer[0] == '\n') {
            continue; // Ignore the line
        }
        /*
         * Now parse the configuration line.
         * Lines are formatted as "key : value"
         */
        config_key = strtok(file_buffer, " : ");
        // /n removes the newline from value
        config_value = strtok(NULL, " : \n");
        if ((!config_key) || !(config_value)) {
            System_printf("Invalid configuration file line\n");
            continue;
        }
        System_printf("Config: %s = %s\n", config_key, config_value);
        parse_config_entry(config_key, config_value);
    }
    fclose(config_file);
}

/**
 * Parses a configuration file entry
 * @param key: Configuration file key
 * @param value: Configuration value
 */
void parse_config_entry(char *key, char *value) {
    // Check to see if the configuration data matches any known keys
    if (strncmp(key, HARDWARE_ID_CONFIG_KEY, strlen(HARDWARE_ID_CONFIG_KEY)) ==
        0) {
        // Copy in hardware ID
        strncpy(program_config.hardware_id, value, HARDWARE_ID_LEN);
        // Ensure null termination
        program_config.hardware_id[HARDWARE_ID_LEN - 1] = '\0';
    } else if (strncmp(key, SYNTHETIC_ID_CONFIG_KEY,
                       strlen(SYNTHETIC_ID_CONFIG_KEY)) == 0) {
        program_config.synthetic_id = atoi(value);
    } else if (strncmp(key, RADAR_ENABLED_KEY, strlen(RADAR_ENABLED_KEY)) ==
               0) {
        program_config.radar_module_enabled = (strncmp(value, "true", 4) == 0);
    } else if (strncmp(key, CAMERA_ENABLED_KEY, strlen(CAMERA_ENABLED_KEY)) ==
               0) {
        program_config.camera_module_enabled = (strncmp(value, "true", 4) == 0);
    } else if (strncmp(key, NETWORK_ENABLED_KEY, strlen(NETWORK_ENABLED_KEY)) ==
               0) {
        program_config.network_enabled = (strncmp(value, "true", 4) == 0);
    } else if (strncmp(key, SERVER_IP_KEY, strlen(SERVER_IP_KEY)) == 0) {
        strncpy(program_config.server_ip, value, IPV4_ADDR_STRLEN);
        program_config.server_ip[IPV4_ADDR_STRLEN - 1] = '\0';
    } else if (strncmp(key, SERVER_AUTH_KEY, strlen(SERVER_AUTH_KEY)) == 0) {
        strncpy(program_config.server_token, value, TOKEN_STRLEN);
        program_config.server_token[TOKEN_STRLEN - 1] = '\0';
    } else if (strncmp(key, RADAR_SAMPLE_INTERVAL_KEY,
                       strlen(RADAR_SAMPLE_INTERVAL_KEY)) == 0) {
        program_config.radar_sample_interval = atoi(value);
    } else if (strncmp(key, RADAR_SAMPLE_COUNT_KEY,
                       strlen(RADAR_SAMPLE_COUNT_KEY)) == 0) {
        program_config.radar_sample_count = atoi(value);
    } else if (strncmp(key, RADAR_SAMPLE_OFFSET_KEY,
                       strlen(RADAR_SAMPLE_OFFSET_KEY)) == 0) {
        program_config.radar_sample_offset = strtof(value, NULL);
    }
}

/**
 * This function posts an event to the storage thread so that it will attempt
 * to mount the SD card
 */
void request_sd_mount() { Event_post(storageEventHandle, EVT_SDCARD_MOUNT); }

/**
 * This function mounts the SD card, and opens the data file for read and write.
 * No attempt is made to reread the configuration file.
 */
void mount_sdcard() {
    FRESULT fr;
    FILINFO fno;
    IArg sd_mutex_key;
    unsigned int bytesWritten;
    char filename_buf[32];
    // Print the filename into the full output string we need to open files
    snprintf(filename_buf, sizeof(filename_buf), "fat:" STR(DRIVE_NUM) ":%s",
             sensor_data_filename);
    sdfatfsHandle = SDFatFS_open(CONFIG_SD_0, DRIVE_NUM);
    if (sdfatfsHandle == NULL) {
        System_abort("Could not open the SD Card\n");
    }
    // Get SD card mutex
    sd_mutex_key = GateMutex_enter(sdMutex);
    /*
     * Check if data file exists.
     * TODO: It may be possible to avoid this check, and simply open the file in
     * "append" mode
     */
    fr = f_stat(sensor_data_filename, &fno);
    switch (fr) {
    case FR_OK: // File exists
        System_printf("Found sensor data file, size: %lu\n", fno.fsize);
        data_file = fopen(filename_buf, "a");
        if (!data_file) {
            System_abort("Could not open sensor data file\n");
        }
        break;
    case FR_NO_FILE:
        System_printf("No sensor data file, making new file\n");
        data_file = fopen(filename_buf, "w+");
        if (!data_file) {
            System_abort("Could not open sensor data file\n");
        }
        // Write headers to new data file (exclude the null byte)
        bytesWritten =
            fwrite(FILE_HEADER, sizeof(FILE_HEADER) - 1, 1, data_file);
        if (!bytesWritten) {
            System_abort("SD card write error\n");
        }
        break;
    case FR_NOT_READY:
        // This error occurs when the system has no SD card. Warn user.
        System_printf("Warning: no SD card detected, data storage will not "
                      "function\n");
        cli_log(
            "Warning: without an SD card, data storage will not function\n");
        SDFatFS_close(sdfatfsHandle);
        sdfatfsHandle = NULL;
        break;
    default:
        System_abort("SD card error occurred\n");
    }
    // Open log file
    log_file = fopen(log_filename, "a");
    if (!log_file) {
        // Don't fail to boot, but warn user.
        cli_log("Warning: could not locate log file\n");
        System_printf("Warning: could not locate log file\n");
    }
    // Leave SD card mutex
    GateMutex_leave(sdMutex, sd_mutex_key);
}

/**
 * Logs data onto the SD card. Logs asynchronously.
 * @param logstr: string to log
 */
void log_sdcard(const char *logstr) {
    IArg sd_mutex_key;
    int num_printed;
    char output_buf[LOG_LINE_MAX];
    struct timespec ts;
    if (log_file && sdfatfsHandle) {
        // Get current timestamp
        clock_gettime(CLOCK_REALTIME, &ts);
        // Print timestamp and log string into buffer
        num_printed = snprintf(output_buf, sizeof(output_buf), "[%d]: %s",
                               (int)ts.tv_sec, logstr);
        // Get SD card mutex
        sd_mutex_key = GateMutex_enter(sdMutex);
        Watchdog_clear(watchdogHandle);
        if (fwrite(output_buf, num_printed, 1, log_file) != 1) {
            fclose(log_file);
            log_file = NULL;
            // Drop mutex, otherwise we'll get deadlock
            GateMutex_leave(sdMutex, sd_mutex_key);
            cli_log("Logfile SD card error\n");
            return;
        }
        // Leave SD card mutex
        GateMutex_leave(sdMutex, sd_mutex_key);
    }
}

/**
 * Unmount the SD card, so it can be removed from the system.
 */
void unmount_sdcard() { Event_post(storageEventHandle, EVT_SDCARD_UNMOUNT); }

/**
 * Stores sensor data to the SD card.
 * @param packet Data packet to store
 */
void store_sensor_data(SensorDataPacket *packet) {
    static int queue_idx = 0;
    SensorDataQueueElem *queue_elem;
    IArg mutex_key;
    // Enter the queue mutex
    mutex_key = GateMutex_enter(queueMutex);
    // Get the queue element to add.
    queue_elem =
        &queue_elements[queue_idx & (MAX_QUEUE_ELEM - 1)]; // quicker modulo
    memcpy(&(queue_elem->packet), packet, sizeof(SensorDataPacket));
    // Use the atomic enqueuing operation
    Queue_enqueue(sensorDataQueue, &(queue_elem->elem));
    GateMutex_leave(queueMutex, mutex_key);
    queue_idx++;
    // Notify storage thread about sensor data
    Event_post(storageEventHandle, EVT_SENSOR_DATA_AVAIL);
}

/**
 * forces all open files to write to the attached disk
 */
void sync_to_disk() {
    IArg sd_mutex_key;
    if (!sdfatfsHandle) {
        return; // No sd card present
    }
    // Get SD card mutex
    sd_mutex_key = GateMutex_enter(sdMutex);
    // Force files to flush to OS using fflush
    if (fflush(data_file)) {
        System_printf("SD Card write error!\n");
        System_flush();
        GateMutex_leave(sdMutex, sd_mutex_key);
        return;
    }
    if (fflush(log_file)) {
        System_printf("SD card write error\n");
        System_flush();
        GateMutex_leave(sdMutex, sd_mutex_key);
        return;
    }
    // Drop SD card mutex
    GateMutex_leave(sdMutex, sd_mutex_key);
}

/**
 *  ======== fatfs_getFatTime ========
 *  The TI FatFS library uses this function, but does *NOT* implement it, so
 *  for some ungodly reason you have to implement it yourself. This was pulled
 *  from a TI example.
 *  Why? Why TI?
 */
int32_t fatfs_getFatTime(void) {
    time_t seconds;
    uint32_t fatTime;
    struct tm *pTime;

    /*
     *  TI time() returns seconds elapsed since 1900, while other tools
     *  return seconds from 1970.  However, both TI and GNU localtime()
     *  sets tm tm_year to number of years since 1900.
     */
    seconds = time(NULL);

    pTime = localtime(&seconds);

    /*
     *  localtime() sets pTime->tm_year to number of years
     *  since 1900, so subtract 80 from tm_year to get FAT time
     *  offset from 1980.
     */
    fatTime =
        ((uint32_t)(pTime->tm_year - 80) << 25) |
        ((uint32_t)(pTime->tm_mon) << 21) | ((uint32_t)(pTime->tm_mday) << 16) |
        ((uint32_t)(pTime->tm_hour) << 11) | ((uint32_t)(pTime->tm_min) << 5) |
        ((uint32_t)(pTime->tm_sec) >> 1);

    return ((int32_t)fatTime);
}
