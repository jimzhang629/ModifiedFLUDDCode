/**
 * @file cli.c
 * CLI implementation for the MSP432. Allows for basic commands
 * to be run for debugging.
 * Created on: Jan 23, 2021
 * Author: danieldegrasse
 */
/* Ti Drivers */
#include <ti/drivers/UART.h>

/* xdc module headers */
#include <xdc/runtime/System.h>
#include <xdc/std.h>

/* TI BIOS headers */
#include <ti/sysbios/gates/GateMutex.h>

/* DriverLib Includes */
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>

/* Standard libs */
#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ti_drivers_config.h"

#include "cli.h"
#include "radar.h"
#include "storage.h"
#include "transmission.h"

// Global variables
static GateMutex_Handle cliMutex;

void cli_init();
void cli_run(UArg arg0, UArg arg1);
void register_cli_function(char *command, char *help,
                           int (*function)(int, char *[]));
void cli_log(const char *format, ...);
void cli_write(const char *format, ...);
static int cli_help(int argc, char *argv[]);
static int unmount_sd(int argc, char *argv[]);
static int sensor_data_test(int argc, char *argv[]);
static int mount_sd(int argc, char *argv[]);
static int show_config(int argc, char *argv[]);
static int load_config(int argc, char *argv[]);
static int search_sim(int argc, char *argv[]);
static int force_timesync(int argc, char *argv[]);
static int force_radar_sample(int argc, char *argv[]);
static int system_reset(int argc, char *argv[]);
static int cli_set_radar_offset(int argc, char *argv[]);
static int set_radar_logging(int argc, char *argv[]);

/** CLI constants */
#define CLI_COMMAND_MAX_LEN 20 /**< Max chars in CLI command */
#define MAX_HELP_LEN 80        /**< Max length for CLI help string */
#define CLI_MAX_ENTRIES 16     /**< Max number of CLI commands */
#define CLI_PRINT_BUF_SIZE 128 /**< Max size of the CLI printf buffer */
#define CLI_READ_BUF_SIZE 80   /**< Max input size for CLI */
#define MAX_ARGC 8             /**< Max number of arguments for CLI */
#define CLI_UART_TIMEOUT 5000  /**< Timeout to wait for UART input on CLI */

/**
 * CLI command entry. An array of these is used to store all registed CLI
 * commands
 */
typedef struct {
    char name[CLI_COMMAND_MAX_LEN]; /**< CLI command name */
    char help[MAX_HELP_LEN];        /**< CLI command help */
    int (*function)(int, char *[]); /**< function implementing CLI command */
} CLI_COMMAND;

// Array of CLI command entries
static CLI_COMMAND commands[CLI_MAX_ENTRIES];
// CLI UART handle
static UART_Handle cli_uart;

// Tracks number of cli commands registered
static int cli_command_entry_cnt = 0;
static bool cli_init_done = false;

/**
 * Performs required initialization for the CLI, including setting up the UART.
 */
void cli_init() {
    UART_Params params;
    UART_Params_init(&params);
    params.baudRate = CLI_BAUDRATE;
    params.readDataMode = UART_DATA_TEXT;
    params.writeDataMode = UART_DATA_TEXT;
    params.readEcho = UART_ECHO_ON;
    params.readReturnMode = UART_RETURN_NEWLINE;
    params.readTimeout = CLI_UART_TIMEOUT;
    params.writeTimeout = UART_WAIT_FOREVER;
    cli_uart = UART_open(CONFIG_CLI_UART, &params);
    if (cli_uart == NULL) {
        System_abort("Error: could not open CLI UART, exiting\n");
    }
    // Register built in help function
    register_cli_function("help", "prints this help", cli_help);
    register_cli_function("unmount", "unmounts the SD card for removal",
                          unmount_sd);
    register_cli_function("sensor_testdata",
                          "sends test data format: sensor_testdata [distance]",
                          sensor_data_test);
    register_cli_function("mount", "attempts to mount the SD card", mount_sd);
    register_cli_function("showcfg", "shows configuration", show_config);
    register_cli_function("loadcfg", "load configuration from SD card",
                          load_config);
    register_cli_function("searchSIM", "searches for a connected SIM7000",
                          search_sim);
    register_cli_function("synctime", "forces an NTP time sync",
                          force_timesync);
    register_cli_function("radarsample", "forces radar board to sample",
                          force_radar_sample);
    register_cli_function("reset", "forces a system reset via watchdog",
                          system_reset);
    register_cli_function("setradaroffset", "starts radar calibration",
                          cli_set_radar_offset);
    register_cli_function("setradarlogging",
                          "enables or disables sample logging to cli",
                          set_radar_logging);
    // Create Mutex to control multithreaded access to the UART.
    cliMutex = GateMutex_create(NULL, NULL);
    if (!cliMutex) {
        System_abort("Could not create CLI mutex\n");
    }
    cli_init_done = true;
    System_printf("CLI initialization done\n");
}

/**
 * Runs the CLI.
 * @param arg0: Unused
 * @param arg1: Unused
 */
void cli_run(UArg arg0, UArg arg1) {
    IArg mutex_key;
    int i, argc, retval, num_read;
    char cli_buf[CLI_READ_BUF_SIZE], tmp;
    char *argv[MAX_ARGC];
    const char PROMPT[] = "-> ";
    while (1) {
        argc = 0;
        // Lock CLI Mutex
        mutex_key = GateMutex_enter(cliMutex);
        // Print prompt
        UART_write(cli_uart, PROMPT, sizeof(PROMPT));
        // Drop the CLI Mutex, we no longer need UART write access
        GateMutex_leave(cliMutex, mutex_key);

        /*
         * Read user input until we get a newline
         */
        i = 0;
        do {
            num_read = UART_read(cli_uart, &tmp, 1);
            if (num_read) {
                cli_buf[i] = tmp;
                i += num_read;
            }
            Watchdog_clear(watchdogHandle);
        } while (cli_buf[i - 1] != '\n');

        /*
         * null terminate input.
         * We also drop the last char here if it's a newline
         */
        cli_buf[i] = '\0';
        if (cli_buf[i - 1] == '\n') {
            cli_buf[i - 1] = '\0';
        }
        /*
         * Tokenize the input. We want to assign the first word in input as the
         * command, and the rest of the words should be arguments.
         */
        argv[argc] = strtok(cli_buf, " ");
        if (argv[argc] == NULL || *argv[argc] == '\n') {
            // command was empty. Don't process it.
            continue;
        }
        /*
         * Process the rest of the arguments to the command. Assign each
         * argument to the next index in the argv array
         */
        while (argc < (MAX_ARGC - 1)) {
            argc++;
            argv[argc] = strtok(NULL, " ");
            if (argv[argc] == NULL) {
                break; // processing is done.
            }
        }
        // check to see if the user entered a valid command
        for (i = 0; i < cli_command_entry_cnt; i++) {
            // Check to see if the command matches any we know of
            if (strncmp(commands[i].name, argv[0], CLI_COMMAND_MAX_LEN) == 0) {
                // The command matched. Execute it.
                retval = (*commands[i].function)(argc, argv);
                if (retval != 0) {
                    cli_write("Execution failed\n");
                } else {
                    cli_write("Done\n");
                }
                break; // We are done looking for matching commands.
            }
        }
        if (i == cli_command_entry_cnt) {
            // We did not find a matching command
            cli_write("%s: unknown command\n", argv[0]);
        }
    }
}

/**
 * Built in CLI help function, that prints all the available CLI commands
 * @param argc: number of arguments to the command
 * @param argv: array of all arguments passed to the command
 * @return
 */
static int cli_help(int argc, char *argv[]) {
    int i;
    if (argc != 1) {
        cli_write("Error, help does not take any arguments\n");
        return -1;
    }
    // Print all available CLI commands
    cli_write("Available Commands:\n");
    for (i = 0; i < cli_command_entry_cnt; i++) {
        cli_write("%s: %s\n", commands[i].name, commands[i].help);
    }
    return 0;
}

/**
 * Simulates the arrival of sensor data by notifying the data storage process.
 * @param argc: number of arguments
 * @param argv: argument array
 * @return 0 on sucesss, or negative value on error
 */
static int sensor_data_test(int argc, char *argv[]) {
    SensorDataPacket packet;
    struct timespec ts;
    if (argc != 2) {
        return -1;
    }
    // First argument is distance
    packet.distance = strtof(argv[1], NULL);
    // Get current RTC time
    clock_gettime(CLOCK_REALTIME, &ts);
    packet.timestamp = ts.tv_sec;
    store_sensor_data(&packet);
    transmit_sensor_data(&packet);
    return 0;
}

/**
 * Forces an NTP time sync
 * @param argc: number of arguments
 * @param argv: argument array
 * @return 0 on sucesss, or negative value on error
 */
static int force_timesync(int argc, char *argv[]) {
    request_rtc_update();
    return 0;
}

/**
 * Forces a radar sample
 * @param argc: number of arguments
 * @param argv: argument array
 * @return 0 on sucesss, or negative value on error
 */
static int force_radar_sample(int argc, char *argv[]) {
    sample_radar();
    return 0;
}

/**
 * Sets configuration of radar logging. If enabled, radar
 * task will log all samples to the CLI before transmitting or saving.
 */
static int set_radar_logging(int argc, char *argv[]) {
    bool enabled;
    if (argc != 2) {
        cli_write("Incorrect number of arguments");
        return -1;
    } else {
        enabled = (strncmp(argv[1], "enabled", 7) == 0);
        configure_sample_logging(enabled);
        cli_write("Sample logging %s\n", enabled ? "enabled" : "disabled");
        return 0;
    }
}

/**
 * Register a function with the CLI
 * @param command name of command when called from the CLI
 * @param help help string for the command
 * @param function function to run when command is called. First argument to
 *  function will be the number of arguments, second will be an array holding
 *  the arguments.
 */
void register_cli_function(char *command, char *help,
                           int (*function)(int, char *[])) {
    // Copy in command name
    strncpy(commands[cli_command_entry_cnt].name, command, CLI_COMMAND_MAX_LEN);
    strncpy(commands[cli_command_entry_cnt].help, help, MAX_HELP_LEN);
    commands[cli_command_entry_cnt].function = function;
    cli_command_entry_cnt++;
}

/**
 * Prints data to the CLI and write to the log file. Takes printf style
 * arguments.
 * @param format: printf format string
 */
void cli_log(const char *format, ...) {
    IArg mutex_key;
    va_list varargs;
    int num_chars;
    char temp_cli_buf[CLI_PRINT_BUF_SIZE]; // stores the output of snprintf
    va_start(varargs, format);
    num_chars = vsnprintf(temp_cli_buf, CLI_PRINT_BUF_SIZE, format, varargs);
    va_end(varargs);
    // Write to log file
    Watchdog_clear(watchdogHandle);
    log_sdcard(temp_cli_buf);
    if (!cli_init_done) {
        return; // CLI is not ready
    }
    // Print to CLI
    mutex_key = GateMutex_enter(cliMutex);
    // Write formatted output to UART.
    if (UART_write(cli_uart, temp_cli_buf, num_chars) != num_chars) {
        System_printf("Error while writing to CLI UART\n");
    }
    // Drop the mutex
    GateMutex_leave(cliMutex, mutex_key);
}

/**
 * Prints data to the CLI. Takes printf style arguments.
 * @param format: printf format string
 */
void cli_write(const char *format, ...) {
    IArg mutex_key;
    if (!cli_init_done) {
        return; // CLI is not ready
    }
    va_list varargs;
    int num_chars;
    char temp_cli_buf[CLI_PRINT_BUF_SIZE]; // stores the output of snprintf
    va_start(varargs, format);
    num_chars = vsnprintf(temp_cli_buf, CLI_PRINT_BUF_SIZE, format, varargs);
    va_end(varargs);
    // Lock CLI Mutex.
    mutex_key = GateMutex_enter(cliMutex);
    // Write formatted output to UART.
    if (UART_write(cli_uart, temp_cli_buf, num_chars) != num_chars) {
        System_printf("Error while writing to CLI UART\n");
    }
    // Drop the mutex
    GateMutex_leave(cliMutex, mutex_key);
}

/**
 * Unmounts the SD card
 * @param argc: number of arguments
 * @param argv: argument array
 * @return 0
 */
static int unmount_sd(int argc, char *argv[]) {
    unmount_sdcard();
    return 0;
}

/**
 * Mounts the SD card
 * @param argc: number of arguments
 * @param argv: argument array
 * @return 0
 */
static int mount_sd(int argc, char *argv[]) {
    request_sd_mount();
    return 0;
}

/**
 * Dumps all configuration values from the program_config structure
 * @param argc: number of arguments
 * @param argv: argument array
 * @return 0
 */
static int show_config(int argc, char *argv[]) {
    cli_write("Hardware ID: %s\n"
              "Unique ID: %d\n"
              "Radar: %s\n"
              "Camera: %s\n"
              "Network: %s\n"
              "Server IP: %s\n",
              program_config.hardware_id, program_config.synthetic_id,
              program_config.radar_module_enabled ? "enabled" : "disabled",
              program_config.camera_module_enabled ? "enabled" : "disabled",
              program_config.network_enabled ? "enabled" : "disabled",
              program_config.server_ip);
    // Split into multiple CLI writes so we don't overflow buffer
    cli_write("Radar Sample Interval: %i ms\n"
              "Radar Sample Count: %i\n"
              "Radar Sample offset: %.3f\n",
              program_config.radar_sample_interval,
              program_config.radar_sample_count,
              program_config.radar_sample_offset);
    return 0;
}

/**
 * Reloads the configuration values from the SD card.
 * @param argc: number of arguments
 * @param argv: argument array
 * @return 0
 */
static int load_config(int argc, char *argv[]) {
    read_configuration();
    show_config(argc, argv);
    return 0;
}

/**
 * Reloads the configuration values from the SD card.
 * @param argc: number of arguments
 * @param argv: argument array
 * @return 0
 */
static int search_sim(int argc, char *argv[]) { return find_sim() ? 0 : 255; }

/**
 * Performs a software reset
 * @param argc: number of arguments
 * @param argv: argument array
 * @return 0
 */
static int system_reset(int argc, char *argv[]) {
    MAP_SysCtl_rebootDevice();
    return 0;
}

/**
 * Sets the radar offset to the given value if one is provided, or starts
 * calibration if one is not
 * @param argc: number of arguments
 * @param argv: argument array
 * @return 0
 */
static int cli_set_radar_offset(int argc, char *argv[]) {
    // Set the radar offset via calibration
    force_calibration();
    return 0;
}

/**
 * Custom system exit handler. Writes output code to CLI.
 */
void system_exitfxn(int code) {
    int num_chars;
    char buffer[80];
    num_chars =
        snprintf(buffer, sizeof(buffer), "System exiting with code %d\n", code);
    UART_write(cli_uart, buffer, num_chars);
    exit(code);
}

/**
 * Custom system abort handler that notifies user.
 */
void system_abortfxn() {
    UART_write(cli_uart, "System abort!\n", 14);
    abort();
}
