/**
 * @file common.h
 * Data structure definitions used across the program
 *
 * Created on: Jan 23, 2021
 * Author: danieldegrasse
 */

#ifndef COMMON_H_
#define COMMON_H_

#include <stdbool.h>
#include <stdint.h>
#include <ti/drivers/Watchdog.h>
#include <time.h>

/**
 * Packet structure for received sensor data
 */
typedef struct {
    time_t timestamp; /**< UTC timestamp when packet was read from sensor */
    float distance;   /**< Distance read from sensor in meters */
} SensorDataPacket;

#define HARDWARE_ID_LEN 8   /**< Length of the system hardware ID */
#define IPV4_ADDR_STRLEN 16 /**< Length of the IPV4 address string */
/** Length of the backend access token (plus null terminator */
#define TOKEN_STRLEN 41

/**
 * Globally accessible configuration structure.
 * The actual global instance is defined in main.c
 * All char[] members MUST be null terminated
 */
typedef struct {
    char hardware_id[HARDWARE_ID_LEN]; /**< device hardware ID */
    int synthetic_id;                  /**< device synthetic ID */
    bool radar_module_enabled;         /**< should radar samples be taken */
    bool camera_module_enabled;        /**< should camera be booted */
    bool network_enabled;              /**< should network device be booted */
    char server_ip[IPV4_ADDR_STRLEN];  /**< server IP to connect to */
    char server_token[TOKEN_STRLEN];   /**< server authentication token */
    int radar_sample_interval;         /**< given in ms */
    int radar_sample_count;            /**< number of samples to take */
    float radar_sample_offset;         /**< float offset to all radar samples */
} ProgramConfiguration;

/** Global program configuration structure, implemented in main.c */
extern ProgramConfiguration program_config;
/** Global watchdog timer handler, implemented in main.c */
extern Watchdog_Handle watchdogHandle;

#endif /* COMMON_H_ */
