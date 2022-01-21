/**
 *  @file radar.c
 *  Implements communication with attached radar sensor to read water level
 *  This file should implement communication with an attached sensor, and
 *  support a notification method to make other tasks aware new sensor data
 *  was acquired
 *
 *  Created on: Jan 22, 2021
 *  Author: danieldegrasse
 */
/* Here is a comment GLW 2021-09-08 */
/* XDC Module Headers */
#include <xdc/runtime/System.h>
#include <xdc/std.h>

/* BIOS Module headers */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Task.h>

/* Ti Driver Headers */
#include "ti_drivers_config.h"
#include <ti/drivers/GPIO.h>
#include <ti/drivers/UART.h>

/* Standard libraries */
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

#include "cli.h"
#include "common.h"
#include "storage.h"
#include "transmission.h"

/** Constant values board should reply with */
#define RADAR_CMD_DONE "Done"     /**< the board successfully ran a command */
#define RADAR_PROMPT "mmwDemo:/>" /**< new command prompt from the board */

#define RADAR_BAUDRATE 115200      /**< UART Baudrate of Radar sensor */
#define RADAR_PACKET_HEADER 0xBEEF /**< "Magic" header on every radar packet   \
                                    */
#define RADAR_DONE_TIMEOUT 7000 /**< how many ms to wait for commands to run   \
                                 */
#define RADAR_READ_TIMEOUT 100  /**< how many ms to wait for data from system  \
                                 */
#define RADAR_SAMPLE_DELAY 500  /**< how many ms to wait for radar sample */
#define RADAR_BOARD_CMD_DELAY 2 /**< how many ms to wait between commands */
#define RANGE_DELTA_MIN                                                        \
    0.8 /**< how many meters below offset to allow samples */
#define RANGE_DELTA_MAX                                                        \
    0.2 /**< how many meters above offset to allow samples */
/**
 * minimum number of samples to be averaged for a TX to go through, should
 * radar fail to return a sample
 */
#define RADAR_MIN_SAMPLES 30
#define RADAR_READ_NODATA -2   /**< Radar did not return any data */
#define RADAR_READ_NOPACKET -1 /**< Radar did not return a valid packet */
#define RADAR_SUCCESS 0        /**< Radar returned data */
#define RADAR_THRESHOLD 0      /**< Minimum value radar should return */
/**
 * max height expected (~30 ft height at 10m), throws out bogus values of
 * high magnitude (in case of bit errors on UART)
 */
#define RADAR_UPPER_THRESHOLD 10
/** how many ticks to delay between LED flashes during calibration */
#define CALIBRATION_BLINK_DELAY 750

#define EVT_RADAR_SAMPLE Event_Id_00    /**< Radar sample event ID */
#define EVT_RADAR_CALIBRATE Event_Id_01 /**< Radar calibration event ID */

/**
 * Radar data packet. Only holds distance. Note the use of struct packing
 */
typedef struct __attribute((__packed__)) RadarPacket {
    uint16_t header; /**< "Magic" Header value should go here */
    float distance;  /**< Actual distance from radar board */
} RadarPacket; /**< Radar data packet */

/* Global variables */
static UART_Params params;
static UART_Handle sensor_uart;
static Event_Handle radarEventHandle;
// Should the wait for GPIO0 to be pressed during calibration be bypassed
static bool button_press_bypassed = false;
// Should each radar sample be logged to the CLI
static bool log_radar_samples = false;

/* Static functions */
static int configure_radar();
static int wait_data(const char *expected, int len);
static int read_radar_distance(float *distance);
static void boot_radar();
static void powerdown_radar();
static int wait_boot();
static int wait_packet();
static void end_calibration_wait(uint8_t index);

static char rangelimit_cfg_command[40] = {0};

/* List of command strings to send the the radar module to configure it. */
static const char *radar_commands[] = {
    "sensorStop", "flushCfg", "dfeDataOutputMode 1", "channelCfg 1 1 0",
    "adcCfg 2 1", "adcbufCfg 0 1 0 1",
    "profileCfg 0 77 7 7 212.8 0 0  18.32 1 1024 5000 0 0 40",
    "chirpCfg 0 0 0 0 0 0 0 1", "frameCfg 0 0 1 0 500 1 0",
    "calibDcRangeSig 0 -5 5 32", "guiMonitor 1   1 0  0 0  1",
    rangelimit_cfg_command, // set in code
    "sensorStart",
    //"enableMovingAvg on 100",
    NULL /*This is REQUIRED to signal the end of the list */};

/**
 * This function should perform any initialization the radar sensor model needs
 * This includes setting up peripherals (like UART).
 * This is analogous to the setup() function in for Arduino
 */

void lidar_init() {
    // start lidar sensor!

    //turn on and write data to power pin

}
void radar_init() {
    // Set up the UART to read data from the sensor
    UART_Params_init(&params);
    params.baudRate = RADAR_BAUDRATE;
    params.readDataMode = UART_DATA_BINARY;
    params.readTimeout = RADAR_READ_TIMEOUT;

    radarEventHandle = Event_create(NULL, NULL);
    if (!radarEventHandle) {
        System_abort("Could not create radar event\n");
    }
    // Configure the Radar PMIC GPIO to power down the radar board
    GPIO_setConfig(CONFIG_GPIO_RADAR_PMIC_EN,
                   GPIO_CFG_OUTPUT | GPIO_CFG_OUT_LOW);
    // Also configure D1 indicator LED as low output
    GPIO_setConfig(CONFIG_D1_LED, GPIO_CFG_OUTPUT | GPIO_CFG_OUT_LOW);
    // Configure the button as a pulled down input
    GPIO_setConfig(CONFIG_GPIO0_BTN,
                   GPIO_CFG_INPUT | GPIO_CFG_IN_PD | GPIO_CFG_IN_INT_RISING);
    /* install Button callback */
    GPIO_setCallback(CONFIG_GPIO0_BTN, end_calibration_wait);

    // Open the sensor uart
    sensor_uart = UART_open(CONFIG_SENSOR_UART, &params);
    if (sensor_uart == NULL) {
        System_abort("sensor UART init failed, exiting\n");
    }
    System_printf("sensor init done\n");
}

/**
 * Asynchronously runs sampling code for radar, getting one sample
 * and transmitting it
 */
void sample_radar() {
    if (!program_config.radar_module_enabled) {
        return;
    }
    Event_post(radarEventHandle, EVT_RADAR_SAMPLE);
}

/**
 * Forces the radar to calibrate.
 */
void force_calibration() {
    cli_log("Forcing Radar calibration\n");
    program_config.radar_sample_offset = 0.0;
    // Force the radar to calibrate by bypassing the button press
    button_press_bypassed = true;
}

/**
 * Enables (or disables) radar sample logging
 * @param enabled: should samples be logged to the CLI
 */
void configure_sample_logging(bool enabled) { log_radar_samples = enabled; }

/**
 * This function should run forever, and poll any sensor data from the radar
 * sensor. It should then notify other processes that new sensor data is
 * available
 * @param arg0: Unused
 * @param arg1: Unused
 */
void radar_run(UArg arg0, UArg arg1) {
    int num_samples;
    UInt events;
    float num_samples_avg;
    float sum;
    SensorDataPacket packet;
    System_printf("sensor task starting\n");
    Watchdog_clear(watchdogHandle);
    cli_log("Radar task starting with distance offset of %.2f\n",
            program_config.radar_sample_offset);
    if (program_config.radar_sample_offset != 0.0) {
        // Add the calibration parameter to configuration
        snprintf(rangelimit_cfg_command, sizeof(rangelimit_cfg_command),
                 "RangeLimitCfg 1 %.1f, %.1f",
                 program_config.radar_sample_offset - RANGE_DELTA_MIN,
                 program_config.radar_sample_offset + RANGE_DELTA_MAX);
    }
    /*
     * Run Loop.
     * 1. Wait for a sample to be requested
     * 2. Boot the radar board
     * 3. Configure the board
     * 4. Take a defined number of samples
     * 5. Power down the board
     */
    while (1) {
        if (program_config.radar_sample_offset == 0.0) {
            // Use a default calibration range
            snprintf(rangelimit_cfg_command, sizeof(rangelimit_cfg_command),
                     "RangeLimitCfg 1 1.0 10.0\n");
            cli_log("Radar offset has not been configured. Press GPIO0 button "
                    "to take baseline sample\n");
            System_printf("Waiting for button press to calibrate radar\n");
            System_flush();
            // Pend on the event handle, waiting for EVT_RADAR_CALIBRATE events
            while (1) {
                /* Enable interrupt before waiting */
                GPIO_enableInt(CONFIG_GPIO0_BTN);
                events =
                    Event_pend(radarEventHandle, Event_Id_NONE,
                               EVT_RADAR_CALIBRATE, CALIBRATION_BLINK_DELAY);
                if (events & EVT_RADAR_CALIBRATE) {
                    break;
                }
                Watchdog_clear(watchdogHandle);
                // Flash the D1 LED to indicate that we are waiting
                GPIO_toggle(CONFIG_D1_LED);
                if (button_press_bypassed) {
                    cli_log("Button press was bypassed, no longer waiting\n");
                    break;
                }
            }
            // Disable interrupt since wait is over
            GPIO_disableInt(CONFIG_GPIO0_BTN);
            /*
             * Now, the calibration sample should be allowed to proceed.
             * Calibrate the board. The sampling code will calibrate the board
             * if the sample offset is zero, so simply set things up so it will
             * run.
             */
            events = EVT_RADAR_SAMPLE;
            GPIO_write(CONFIG_D1_LED, CONFIG_GPIO_LED_OFF);
            if (button_press_bypassed) {
                cli_log("Radar calibration starting!\n");
                button_press_bypassed = false; // Reset value
            } else {
                // Delay to give user time to get out of way of radar
                cli_log("Calibration will start in 10 minutes\n");
                Task_sleep(600000);
                cli_log("Booting radar for calibration\n");
            }
        } else {
            events = Event_pend(radarEventHandle, Event_Id_NONE,
                                EVT_RADAR_SAMPLE, BIOS_WAIT_FOREVER);
        }
        if (events & EVT_RADAR_SAMPLE) {
            boot_radar();
            if (wait_boot() < 0) {
                System_printf("Radar did not boot\n");
                System_flush();
                Watchdog_clear(watchdogHandle);
                cli_log("Timed out waiting for radar boot\n");
                powerdown_radar();
                continue; // Do not run rest of loop
            }
            if (configure_radar() < 0) {
                System_printf("Failed to configure radar module\n");
                System_flush();
                cli_log("Failed to configure radar module\n");
                powerdown_radar();
                continue; // Do not run rest of loop
            }
            // Wait for radar board to produce data.
            if (wait_packet() < 0) {
                System_printf("Did not get data from radar board\n");
                Watchdog_clear(watchdogHandle);
                System_flush();
                cli_log("Radar board is not producing data\n");
                powerdown_radar();
                continue; // Do not run rest of loop
            }
            struct timespec ts;
            float distance;
            num_samples = program_config.radar_sample_count;
            num_samples_avg = 0;
            sum = 0;
            while (num_samples--) {
                Task_sleep(RADAR_SAMPLE_DELAY);
                num_samples_avg = num_samples_avg + 1.0;
                if (read_radar_distance(&distance) != RADAR_SUCCESS) {
                    Watchdog_clear(watchdogHandle);
                    System_printf("Failed to get sample from radar board\n");
                    System_flush();
                    cli_log("Did not get sample from radar board\n");
                    break; // Exit loop
                }
                sum = sum + distance;
            }
            if (sum != 0 && num_samples_avg > RADAR_MIN_SAMPLES) {
                if (program_config.radar_sample_offset == 0.0) {
                    // Use this sample to set the radar offset
                    set_radar_offset(sum / num_samples_avg);
                    /*
                     * radar sample offset will now be set. Use it to
                     * determinate our rangelimit configuration. The pattern we
                     * use is to limit the range to no further than
                     * RANGE_DELTA_MAX over from the offset, and no less than
                     * RANGE_DELTA_MIN under the offset
                     */
                    snprintf(
                        rangelimit_cfg_command, sizeof(rangelimit_cfg_command),
                        "RangeLimitCfg 1 %.1f, %.1f",
                        program_config.radar_sample_offset - RANGE_DELTA_MIN,
                        program_config.radar_sample_offset + RANGE_DELTA_MAX);
                    cli_log("Setting radar offset to %.2f\n",
                            program_config.radar_sample_offset);
                } else {
                    // Send and store radar data
                    packet.distance = sum / num_samples_avg;
                    // subtract distance from offset
                    packet.distance =
                        program_config.radar_sample_offset - packet.distance;
                    // If a negative value was read, just zero it out (currently
                    // disabled) if (packet.distance < 0) packet.distance = 0;
                    clock_gettime(CLOCK_REALTIME, &ts);
                    packet.timestamp = ts.tv_sec;
                    if (log_radar_samples) {
                        cli_log("%.03f\n", packet.distance);
                    }
                    store_sensor_data(&packet);
                    transmit_sensor_data(&packet);
                }
            }
            powerdown_radar();
        }
    }
}

/**
 * Wait for the radar board to boot up
 * @return 0 on success, or negative value on error
 */
static int wait_boot() {
    /*
     * The board will print a command prompt when it boots. We need to
     * simply wait for this prompt to appear on the UART device
     */
    return wait_data(RADAR_PROMPT, strlen(RADAR_PROMPT));
}

/**
 * Wait for data to be produced by the radar module
 * @return 0 on success, or negative value on error
 */
static int wait_packet() {
    // Radar packed header is 0xBEEF
    const char BEEF[] = {0xEF, 0xBE};
    int timeout = RADAR_DONE_TIMEOUT;
    if (wait_data(BEEF, 2) < 0) {
        System_printf("Did not get first radar packet\n");
        System_flush();
        return RADAR_READ_NODATA;
    }
    /*
     * Here we do not care about the data content, just that we read a float's
     * length of data from the device.
     */
    while (UART_read(sensor_uart, NULL, sizeof(float)) == 0 && timeout)
        timeout--;
    if (!timeout)
        return RADAR_READ_NODATA;
    return RADAR_SUCCESS;
}

/**
 * Reads a sensor distance from the radar board, and forwards it to the
 * storage task
 * @param packet: populated with read packet data
 * @return 0 on success, or negative value on error
 */
static int read_radar_distance(float *distance) {
    RadarPacket read_packet;
    // struct timespec ts;
    int read_count;
    // Read a data packet from the UART
    read_count = UART_read(sensor_uart, &read_packet, sizeof(RadarPacket));
    if (read_count <= 0) {
        return RADAR_READ_NODATA; // Timed out waiting for data
    }
    // Verify that the header matches expected value
    if (read_count != sizeof(RadarPacket) ||
        read_packet.header != RADAR_PACKET_HEADER) {
        return RADAR_READ_NOPACKET; // Data was read, but no packet.
    }
    // Now, we need to parse the packet and add a timestamp.
    // Reject packets below a threshold
    if (read_packet.distance < RADAR_THRESHOLD ||
        read_packet.distance > RADAR_UPPER_THRESHOLD) {
        return RADAR_READ_NOPACKET;
    }
    *distance = read_packet.distance;
    return RADAR_SUCCESS;
}

/**
 * Configures the radar board, by sending configuration commands to it
 * @return 0 on success, or negative value on error.
 */
static int configure_radar() {
    char **command = (char **)radar_commands;
    while (*command != NULL) {
        if (*(*command) == '\0') {
            // Zero length command. skip, and do not set.
            command++;
            continue;
        }
        // Send next command to the radar board
        if (UART_write(sensor_uart, *command, strlen(*command)) < 0) {
            return -1;
        }
        // Write a newline and carriage return to send it
        if (UART_write(sensor_uart, "\r\n", 2) < 0) {
            return -1;
        }
        /*
         * We now wait to read the response to the command.
         * We want to read characters from the buffer until the sensor
         * responds with "Done"
         */
        if (wait_data(RADAR_CMD_DONE, strlen(RADAR_CMD_DONE)) < 0) {
            return -1;
        }
        // Wait for sensor to be ready for new data
        if (wait_data(RADAR_PROMPT, strlen(RADAR_PROMPT)) < 0) {
            return -1;
        }
        Task_sleep(RADAR_BOARD_CMD_DELAY);
        command++;
    }
    return RADAR_SUCCESS;
}

/**
 * Waits for a command to return, reading from the UART until the value in
 * "expected" is returned.
 * @param expected: value to wait for UART to return
 * @param len: number of bytes in expected to check
 * @return 0 if done is found before timeout, or negative value otherwise
 */
static int wait_data(const char *expected, int len) {
    int timeout, check_idx, num_read;
    char tmp;
    timeout = RADAR_DONE_TIMEOUT;
    check_idx = 0;
    while (timeout > 0) {
        num_read = UART_read(sensor_uart, &tmp, 1);
        if (num_read == 1) {
            // Some data was read, see if it matches expected
            if (expected[check_idx] == tmp) {
                check_idx++;
                if (check_idx == len)
                    break; // expected value found.
            } else if (check_idx > 0) {
                // Reset to the first char in expected.
                check_idx = 0;
            }
        }
        if (num_read <= 0) {
            // Drop timeout by read timeout, since we waited until read timed
            // out
            timeout -= RADAR_READ_TIMEOUT;
        } else if (num_read == 1 && expected[check_idx - 1] != tmp) {
            // We read a character, but not what we wanted. Drop timeout by 1.
            timeout--;
        }
    }
    if (!timeout) {
        System_printf("Timed out while waiting for radar command\n");
        return RADAR_READ_NODATA;
    } else {
        return RADAR_SUCCESS;
    }
}

/**
 * Ends the radar board's delay before calibration, and allows it to set the
 * offset
 * @param index: button index pressed
 */
static void end_calibration_wait(uint8_t index) {
    if (!program_config.radar_module_enabled) {
        return;
    }
    System_printf("Calibration wait ending due to GPIO0 button press\n");
    Event_post(radarEventHandle, EVT_RADAR_CALIBRATE);
}

/**
 * Boot the radar board by pulling PMIC_EN high
 */
static void boot_radar() {
    // Write high voltage to D1 led to signal radar boot
    GPIO_write(CONFIG_D1_LED, CONFIG_GPIO_LED_ON);
    GPIO_write(CONFIG_GPIO_RADAR_PMIC_EN, 1);
}

/**
 * Power down the radar board by pulling PMIC_EN low
 */
static void powerdown_radar() {
    // Write low voltage to D1 led to signal radar powerdown
    GPIO_write(CONFIG_D1_LED, CONFIG_GPIO_LED_OFF);
    GPIO_write(CONFIG_GPIO_RADAR_PMIC_EN, 0);
}
