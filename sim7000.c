/**
 * @file sim7000.c
 * Implements a support library for the SIM7000 LTE Module
 *
 * Note: This library will default to the baudrate given by SIM7000_BAUDRATE.
 * If the device fails to respond, SIM7000_search() can be used to sweep all
 * supported baudrate to find the device.
 *
 * This module is designed to make an network connection, send data, and drop
 * the connection as quickly as possible (a one shot architecture). This should
 * conserve battery life.
 *
 * Created on: Jan 27, 2021
 * Author: danieldegrasse
 */
/* XDC Module Headers */
#include <xdc/runtime/System.h>
#include <xdc/std.h>

/* TI sysbios headers */
#include <ti/sysbios/knl/Task.h>

/* TI drivers */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/UART.h>
#include <ti/drivers/Watchdog.h>

/* Standard libs */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#define __STDC_LIMIT_MACROS /**< Required definition to access UINT8_MAX */
#include <stdint.h>

#include "cli.h"
#include "common.h"
#include "sim7000.h"
#include "ti_drivers_config.h"

/** NTP time server to use for SIM NTP time sync */
#define NTP_TIMESERVER "pool.ntp.org"
/**
 * SIM7000 uses timezones in multiples of 4, to provide 15 minute granularity.
 * -24 corresponds to GMT-6 (central time)
 */
#define TIMEZONE 0 /**< use UTC time*/
#define DEBUG 1    /**< Change this definition to 1 to enable debug printouts */

/**
 * Simple macro to only print debugging info if "DEBUG" is set to 1
 */
#define Debug_printf(fmt, ...)                                                 \
    do {                                                                       \
        if (DEBUG) {                                                           \
            System_printf(fmt, __VA_ARGS__);                                   \
        }                                                                      \
    } while (0)

#define OK_REPLY "OK"                 /**< Default success reply from SIM7000 */
#define GPIO_LOW 0                    /**< GPIO output is low voltage */
#define GPIO_HIGH 1                   /**< GPIO output is high voltage */
#define SIM7000_BOOT_TIMEOUT 8000     /**< timeout for data at boot */
#define SIM7000_NETWORK_TIMEOUT 40000 /**< timeout for network operations */
#define SIM7000_LONG_TIMEOUT 7000     /**< timeout for slower commands */
#define SIM7000_TIMEOUT 1000   /**< number of ms to wait for sim to send data */
#define UART_READ_TIMEOUT 1000 /**< number of ms to wait for UART data */
/** number of ms to pulse PWR pin for to boot SIM */
#define SIM7000_PWRPULSE 200
/** number of ms to wait for sim to turn off */
#define SIM7000_POWERDOWN_DELAY 1700
/** Highest stable baudrate, 115200 may work with better signal integrity */
#define SIM7000_BAUDRATE 9600
/** number of times to "ping" SIM with AT command */
#define SIM7000_BOOT_ATTEMPTS 2

/** SIM7000 phone full functionality mode */
#define SIM_FULL_FUNCTIONALITY 1

/** Values are from SIM7000 AT command manual */
#define HTTP_GET_CODE 1   /**< HTTP GET */
#define HTTP_PUT_CODE 2   /**< HTTP PUT */
#define HTTP_POST_CODE 3  /**< HTTP POST */
#define HTTP_PATCH_CODE 4 /**< HTTP PATCH */
#define HTTP_HEAD_CODE 5  /**< HTTP HEAD */

static void delay_ms(uint32_t ms);
static void boot_sim7000a(SIM7000_Config *config);
static bool verify_boot(SIM7000_Config *handle);
static void flush_input(SIM7000_Config *handle);
static bool send_verified_reply(SIM7000_Config *handle, const char *cmd,
                                const char *expected, int timeout);
static uint8_t get_reply(SIM7000_Config *config, const char *send, int timeout);
static uint8_t sim_readline(SIM7000_Config *config, int timeout);
static bool uart_available(SIM7000_Config *config);
static void reconfigure_baud(SIM7000_Config *config, uint32_t baudrate);
static bool verified_readline(SIM7000_Config *config, const char *expected,
                              int timeout);
static int set_ip_initial(SIM7000_Config *config);
static int http_generic(SIM7000_Config *config, HTTPConnectionRequest *request,
                        int method);
static int enable_app_network(SIM7000_Config *config);
static void disable_app_network(SIM7000_Config *config);
static int connect_http(SIM7000_Config *config, HTTPConnectionRequest *request);
static void disconnect_http(SIM7000_Config *config);
static int set_http_headers(SIM7000_Config *config,
                            HTTPConnectionRequest *request);
static int read_to_buffer(SIM7000_Config *config, uint8_t *output, int len);
static int add_http_body(SIM7000_Config *config,
                         HTTPConnectionRequest *request);
static int read_http_response(SIM7000_Config *config,
                              HTTPConnectionRequest *request);
static int parse_time(char *str, struct timespec *time);
static bool set_sim_functionality(SIM7000_Config *config, int code);
static bool enable_network(SIM7000_Config *config);
static bool enable_bearer(SIM7000_Config *config);
static bool disable_result_codes(SIM7000_Config *config);

// Supported baud rates for the SIM
static const uint32_t sim7000_baudrates[] = {
    1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200, 230400, 921600};

/**
 * Sets up configuration parameters for use with the SIM7000_open function
 * @param config: Configuration structure to initialize
 */
void SIM7000_init_params(SIM7000_Config *config) {
    memset(config->replybuffer, 0, sizeof(config->replybuffer));
    config->apn[0] = '\0';
    config->uart = NULL;
    config->powerkey_pin = UINT8_MAX;
    config->reset_pin = UINT8_MAX;
    config->UART_index = UINT8_MAX;
    config->sim_running = false;
}

/**
 * Starts an instance of the SIM7000 driver
 * @param config: configuration object for the SIM7000 driver
 * @return true if opening succeeds, false otherwise
 */
bool SIM7000_open(SIM7000_Config *config) {
    if (config->uart) {
        System_printf("Error, this SIM driver is already open\n");
        return false;
    }
    // Verify parameters
    if (config->reset_pin == UINT8_MAX || config->powerkey_pin == UINT8_MAX ||
        config->UART_index == UINT8_MAX) {
        System_printf("Error, caller must set GPIO pins and UART index before "
                      "starting SIM\n");
        return false;
    }
    if (config->apn[0] == '\0') {
        System_printf("Error, caller must set apn\n");
        return false;
    }
    // Open the sim UART at default baud rate
    UART_Params params;
    UART_Params_init(&params);
    params.baudRate = SIM7000_BAUDRATE;
    params.readDataMode = UART_DATA_BINARY;
    params.writeDataMode = UART_DATA_BINARY;
    params.readTimeout = UART_READ_TIMEOUT;
    config->uart = UART_open(config->UART_index, &params);
    if (!config->uart) {
        System_abort("Failed to open SIM UART\n");
    }
    return true;
}

/**
 * Powers on the SIM7000, and verifies communication is functional
 * @param config: SIM7000 config structure
 * @return true on success, false otherwise
 */
bool SIM7000_poweron(SIM7000_Config *config) {
    // boot the sim module
    int num_attempts = SIM7000_BOOT_ATTEMPTS;
    while (num_attempts > 0) {
        boot_sim7000a(config);
        if (verify_boot(config)) {
            Debug_printf("%s", "SIM7000 Booted\n");
            config->sim_running = true;
            if (!disable_result_codes(config)) {
                // Just warn the user, the sim is still booted and functional
                cli_log("Warning: could not disable SIM result codes\n");
            }
            return true;
        } else {
            config->sim_running = false;
            num_attempts--;
        }
    }
    System_flush();
    cli_log("SIM did not boot after %d attempts\n", SIM7000_BOOT_ATTEMPTS);
    return false;
}

/**
 * Check to see if the SIM is powered on and responding to commands
 * @param config: SIM7000 config structure
 * @return true if the sim is powered on and responsive , false otherwise
 */
bool SIM7000_running(SIM7000_Config *config) {
    if (config->sim_running) {
        // Verify the SIM is actually running by sending an AT command
        if (send_verified_reply(config, "AT", OK_REPLY, SIM7000_TIMEOUT)) {
            // SIM is running and responsive
            return true;
        }
        // Our status tracking said sim was booted, but it is not responding
        config->sim_running = false;
        return false;
    }
    return false;
}

/**
 * Powers down a running SIM module
 * @param config: SIM7000 Config structure
 * @return true on successful poweroff, false otherwise
 */
bool SIM7000_poweroff(SIM7000_Config *config) {
    if (config->sim_running) {
        // Turn off the SIM by sending the normal power down commands
        if (!send_verified_reply(config, "AT+CPOWD=1", "NORMAL POWER DOWN",
                                 SIM7000_LONG_TIMEOUT)) {
            cli_log("SIM did not respond to power off request");
            System_printf("SIM in unknown state, did not power down in time\n");
            System_flush();
            config->sim_running =
                false; // Assume that SIM timed out because it's off
            return false;
        } else {
            config->sim_running = false;
            // Wait for the board to actually power down
            delay_ms(SIM7000_POWERDOWN_DELAY);
            return true;
        }
    }
    // SIM is already powered off
    return true;
}

/**
 * Sets sim into low current sleep mode. In this mode the sim remains connected
 * to the network, but uses much less power by slowing its clocks (note this
 * mode will not activate when connected to USB)
 * @param config: SIM7000 config structure
 * @param enable: If low current mode should be switched on or off
 * @return true if successful, false otherwise
 */
bool SIM7000_lowpowermode(SIM7000_Config *config, bool enable) {
    return send_verified_reply(config, enable ? "AT+CSCLK=1" : "AT+CSCLK=0",
                               OK_REPLY, SIM7000_TIMEOUT);
}

/**
 * Searches for a SIM device by sweeping all supported baud rates. If a sim
 * device is found, it will be reconfigured to run at SIM7000_BAUDRATE
 *
 * This function is useful if you have connected the SIM correctly,
 * but it's not responding.
 * @param config: SIM7000 Config structure
 * @return true if a SIM device was found and configured
 */
bool SIM7000_search(SIM7000_Config *config) {
    UART_Params params;
    uint32_t i, set_baud_rate = 0;
    // Verify parameters
    if (config->reset_pin == UINT8_MAX || config->powerkey_pin == UINT8_MAX ||
        config->UART_index == UINT8_MAX) {
        System_abort("Error, caller must set GPIO pins and UART index before "
                     "starting SIM\n");
    }
    if (config->uart) {
        UART_close(config->uart);
    }
    // Start by sweeping through baud rates.
    for (i = 0; i < (sizeof(sim7000_baudrates) / sizeof(uint32_t)); i++) {
        // Test this baud rate.
        UART_Params_init(&params);
        params.readDataMode = UART_DATA_BINARY;
        params.writeDataMode = UART_DATA_BINARY;
        params.baudRate = sim7000_baudrates[i];
        config->uart = UART_open(config->UART_index, &params);
        if (!config->uart) {
            System_abort("Could not open SIM UART\n");
        }
        // Now try to verify the boot at this baudrate
        if (verify_boot(config)) {
            // We found a working baud rate
            set_baud_rate = sim7000_baudrates[i];
            cli_log("Located sim at %d baud\n", set_baud_rate);
            break;
        }
        // Close the UART if we could not find a device at this baudrate
        cli_log("No sim at %d baud\n", sim7000_baudrates[i]);
        UART_close(config->uart);
        config->uart = NULL;
    }
    if (set_baud_rate) {
        /*
         * We found a working device at a supported baud rate.
         * Set the baud rate to the default value
         */
        reconfigure_baud(config, SIM7000_BAUDRATE);
        if (verify_boot(config)) {
            cli_log("SIM was successfully reconfigured to %d baud\n",
                    SIM7000_BAUDRATE);
            config->sim_running = true;
            return true;
        } else {
            cli_log("SIM was configured to %d baudrate, but communication "
                    "was lost\n",
                    SIM7000_BAUDRATE);
            return false;
        }
    }
    return false; // SIM was not found at any baud rate
}

/**
 * Sends a block of TCP data to the server, and reads the server's response
 * @param config: SIM7000 config structure
 * @param request: TCP Connection request structure
 * @return: number of bytes read by response (or -1 in case of error)
 */
int16_t SIM7000_tcp(SIM7000_Config *config, TCPConnectionRequest *request) {
    uint16_t data_len, reply_idx;
    char cmd[64];
    // Ensure the IP status is IP initial
    if (set_ip_initial(config) < 0) {
        System_printf("Failed to initialize IP state\n");
        return -1;
    }
    if (!enable_network(config)) {
        System_printf("Failed to enable SIM network\n");
        return -1;
    }
    // Set up the APN and network connection
    snprintf(cmd, sizeof(cmd), "AT+CSTT=\"%s\"", config->apn);
    if (!send_verified_reply(config, cmd, OK_REPLY, SIM7000_TIMEOUT)) {
        System_printf("Could not set APN\n");
        // Reset the IP Status
        get_reply(config, "AT+CIPSHUT", SIM7000_TIMEOUT);
        return -1;
    }
    if (!send_verified_reply(config, "AT+CIICR", "OK", SIM7000_TIMEOUT)) {
        cli_log("Could not connect to LTE network\n");
        get_reply(config, "AT+CIPSHUT", SIM7000_TIMEOUT);
        return -1;
    }
    // Now get an IP address and connect to the network
    if (send_verified_reply(config, "AT+CIFSR", "ERROR", SIM7000_TIMEOUT)) {
        // An ERROR response indicates failure.
        System_printf("Could not get IP address\n");
        get_reply(config, "AT+CIPSHUT", SIM7000_TIMEOUT);
        return -1;
    }
    // We got some form of IP address, verify the status is IP STATUS
    if (!send_verified_reply(config, "AT+CIPSTATUS", OK_REPLY,
                             SIM7000_TIMEOUT)) {
        System_printf("Failed to get IP status\n");
        get_reply(config, "AT+CIPSHUT", SIM7000_TIMEOUT);
        return -1;
    }
    if (!verified_readline(config, "STATE: IP STATUS", SIM7000_TIMEOUT)) {
        System_printf("Bad IP state, cannot complete TCP connection");
        get_reply(config, "AT+CIPSHUT", SIM7000_TIMEOUT);
        return -1;
    }

    /*
     * Now, we can actually complete a TCP connection. Start by opening the
     * TCP connection, then set the data length, write the data, and wait
     * for the TCP server to confirm it received all data.
     */
    // Open the TCP connection
    snprintf(cmd, sizeof(cmd), "AT+CIPSTART=\"TCP\",\"%s\",%d",
             request->endpoint, request->port);
    if (!send_verified_reply(config, cmd, OK_REPLY, SIM7000_LONG_TIMEOUT)) {
        System_printf("Could not start TCP connection\n");
        get_reply(config, "AT+CIPSHUT", SIM7000_TIMEOUT);
        return -1;
    }
    // Now, verify that the device prints "CONNECT OK"
    if (!verified_readline(config, "CONNECT OK", SIM7000_NETWORK_TIMEOUT)) {
        // Device failed to connect
        System_printf("TCP connection failed\n");
        get_reply(config, "AT+CIPSHUT", SIM7000_TIMEOUT);
        return -1;
    }

    // Tell the SIM how much data we will send
    data_len = snprintf(cmd, sizeof(cmd), "AT+CIPSEND=%d\r\n", request->len);
    if (UART_write(config->uart, cmd, data_len) < 0) {
        System_abort("Failed to write to SIM UART\n");
    }

    // Now wait for the SIM to prompt for data by writing ">"
    Watchdog_clear(watchdogHandle);
    UART_read(config->uart, cmd, 2);
    if (cmd[0] != '>') {
        System_printf("Did not get prompt for data from SIM\n");
        get_reply(config, "AT+CIPSHUT", SIM7000_TIMEOUT);
        return -1;
    }

    // Now we simply need to write the data.
    if (UART_write(config->uart, request->data, request->len) < 0) {
        System_abort("Failed to write to SIM UART\n"); // Fatal error
    }
    // We expect the SIM to print "SEND OK" when the server gets the data.
    if (!verified_readline(config, "SEND OK", SIM7000_NETWORK_TIMEOUT)) {
        System_printf("Server did not receive response in time\n");
        get_reply(config, "AT+CIPSHUT", SIM7000_TIMEOUT);
        return -1;
    }

    /*
     * Now, we can read all data from the TCP response. The response ends
     * when the SIM prints "CLOSED", so read until we see that
     */
    reply_idx = 0;
    while (reply_idx < request->out_len) {
        // Read data from SIM
        data_len = sim_readline(config, SIM7000_NETWORK_TIMEOUT);
        if (data_len == 6 &&
            (strncmp(config->replybuffer, "CLOSED", data_len) == 0)) {
            // We reached end of the TCP stream
            break;
        }
        // Otherwise, store this data as the response
        memcpy(&request->out[reply_idx], config->replybuffer, data_len);
        reply_idx += data_len;
    }

    // Read done. Reset IP state and return data length.
    set_ip_initial(config);
    return reply_idx;
}

/**
 * Makes an HTTP GET request using the SIM7000
 * @param config: SIM7000 Configuration structure
 * @param request: HTTP request structure
 * @return: length of read data on success, or negative value on failure
 */
int SIM7000_http_get(SIM7000_Config *config, HTTPConnectionRequest *request) {
    return http_generic(config, request, HTTP_GET_CODE);
}

/**
 * Makes an HTTP PUT request using the SIM7000
 * @param config: SIM7000 Configuration structure
 * @param request: HTTP request structure
 * @return: length of read data on success, or negative value on failure
 */
int SIM7000_http_put(SIM7000_Config *config, HTTPConnectionRequest *request) {
    return http_generic(config, request, HTTP_PUT_CODE);
}

/**
 * Makes an HTTP POST request using the SIM7000
 * @param config: SIM7000 Configuration structure
 * @param request: HTTP request structure
 * @return: length of read data on success, or negative value on failure
 */
int SIM7000_http_post(SIM7000_Config *config, HTTPConnectionRequest *request) {
    return http_generic(config, request, HTTP_POST_CODE);
}

/**
 * Makes an HTTP PATCH request using the SIM7000
 * @param config: SIM7000 Configuration structure
 * @param request: HTTP request structure
 * @return: length of read data on success, or negative value on failure
 */
int SIM7000_http_path(SIM7000_Config *config, HTTPConnectionRequest *request) {
    return http_generic(config, request, HTTP_PATCH_CODE);
}

/**
 * Makes an HTTP HEAD request using the SIM7000
 * @param config: SIM7000 Configuration structure
 * @param request: HTTP request structure
 * @return: length of read data on success, or negative value on failure
 */
int SIM7000_http_head(SIM7000_Config *config, HTTPConnectionRequest *request) {
    return http_generic(config, request, HTTP_HEAD_CODE);
}

/**
 * Synchronizes the SIM7000 Clock with a network time server, and updates the
 * supplied timespec struct with the current time
 * @param config: SIM7000 Config structure
 * @param time: timespec struct, populated with the current time to second
 *  accuracy
 * @return 0 on success, or negative value on error
 */
int SIM7000_ntp_time(SIM7000_Config *config, struct timespec *time) {
    int response_len;
    char cmd[80];
    if (!enable_network(config)) {
        System_printf("Failed to enable SIM network\n");
        return -1;
    }
    if (!enable_bearer(config)) {
        System_printf("Failed to enable SIM bearer\n");
        return -1;
    }
    snprintf(cmd, sizeof(cmd), "AT+CNTP=\"%s\",%d", NTP_TIMESERVER, TIMEZONE);
    if (!send_verified_reply(config, cmd, OK_REPLY, SIM7000_TIMEOUT)) {
        System_printf("Failed to set timeserver\n");
        // Close bearer
        send_verified_reply(config, "AT+SAPBR=0,1", OK_REPLY, SIM7000_TIMEOUT);
        return -1;
    }
    // Make NTP synchronization request
    if (!send_verified_reply(config, "AT+CNTP", OK_REPLY, SIM7000_TIMEOUT)) {
        System_printf("Failed to make NTP sync request\n");
        // Close bearer
        send_verified_reply(config, "AT+SAPBR=0,1", OK_REPLY, SIM7000_TIMEOUT);
        return -1;
    }
    /*
     * SIM7000 produces another line with the current time. We'll query it for
     * the time, so ignore this line.
     * Note: this line takes a VERY long time to return
     */
    if (sim_readline(config, SIM7000_NETWORK_TIMEOUT) == 0) {
        System_printf("Did not get new NTP time\n");
        // Close bearer
        send_verified_reply(config, "AT+SAPBR=0,1", OK_REPLY, SIM7000_TIMEOUT);
        return -1;
    }
    // Now, query the time and parse it.
    response_len = get_reply(config, "AT+CCLK?", SIM7000_TIMEOUT);
    if (response_len == 0) {
        System_printf("NTP time query did not get response");
        // Close bearer
        send_verified_reply(config, "AT+SAPBR=0,1", OK_REPLY, SIM7000_TIMEOUT);
        return -1;
    }
    // Copy the time out here, so we can parse after closing the bearer
    strncpy(cmd, config->replybuffer, sizeof(cmd));
    // Verify that command status is "OK"
    if (!verified_readline(config, OK_REPLY, SIM7000_TIMEOUT)) {
        System_printf("NTP time query did not respond OK\n");
        // Close bearer
        send_verified_reply(config, "AT+SAPBR=0,1", OK_REPLY, SIM7000_TIMEOUT);
        return -1;
    }
    // Close the bearer (closes network connection)
    if (!send_verified_reply(config, "AT+SAPBR=0,1", OK_REPLY,
                             SIM7000_TIMEOUT)) {
        System_printf("Could not close bearer\n");
        return -1;
    }
    /*
     * Now parse the time. Time string will be formatted like this:
     * +CCLK: "21/02/05,00:22:50-24"
     */
    return parse_time(cmd, time);
}

/**
 * Closes the connection to the SIM7000 (and powers if off if it is running)
 * @param config: SIM7000 conf structure
 */
void SIM7000_close(SIM7000_Config *config) {
    if (config->sim_running) {
        if (!SIM7000_poweroff(config)) {
            System_printf("Power down failed on SIM close\n");
            System_flush();
            cli_log("SIM driver did not power down SIM in time\n");
        }
    }
    UART_close(config->uart);
    config->uart = NULL;
}

/**
 * Sets the SIM's functionality mode
 * @param config: SIM7000 Config structure
 * @param code: SIM7000 functionality code
 * @return true on success, or false otherwise
 */
static bool set_sim_functionality(SIM7000_Config *config, int code) {
    char cmd[40];
    snprintf(cmd, sizeof(cmd), "AT+CFUN=%d", code);
    return send_verified_reply(config, cmd, OK_REPLY, SIM7000_TIMEOUT);
}

/*
 * Enables the LTE network for the SIM7000
 * @param config: SIM7000 Config structure
 * @return true on success, false otherwise
 */
static bool enable_network(SIM7000_Config *config) {
    int num_read, should_print = 0, timeout = SIM7000_NETWORK_TIMEOUT;
    bool sim_connected = false;
    // Set the sim to full functionality
    if (!set_sim_functionality(config, SIM_FULL_FUNCTIONALITY)) {
        System_printf("Failed to enable full functionality on SIM\n");
        return false;
    }
    // Set preferred mode to LTE, and prefer Cat-M1 networks
    if (!send_verified_reply(config, "AT+CNMP=38;+CMNB=1", OK_REPLY,
                             SIM7000_TIMEOUT)) {
        System_printf("Failed to set LTE preference\n");
        return false;
    }
    /**
     * Here we want to wait until the network successfully connects
     * Poll CREG value until this is true
     */
    while (timeout > 0 && !sim_connected) {
        num_read = get_reply(config, "AT+CREG?", SIM7000_LONG_TIMEOUT);
        if (num_read == 0) {
            // SIM likely powered off, exit
            System_printf("Could not see powered SIM while waiting for CREG\n");
            System_flush();
            return false;
        }
        if (strncmp(config->replybuffer, "+CREG: 0,1", 10) == 0 ||
            strncmp(config->replybuffer, "+CREG: 0,5", 10) == 0) {
            cli_log("SIM connected to network\n");
            sim_connected = true;
        } else {
            if (strncmp(config->replybuffer, "+CREG: 0,2", 10) == 0) {
                /*
                 * We use should_print to avoid notifying the user every
                 * iteration of this loop that we are searching for a network
                 */
                if (should_print == 5) {
                    cli_log("SIM Searching for network\n");
                    should_print = 0;
                }
                should_print++;
            }
            // If we make it here, sleep then decrease time remaining
            delay_ms(1000);
            timeout -= 1000;
        }
        /*
         * No matter what occured above, we still need to eat the
         * "OK" the command produced
         */
        if (!verified_readline(config, OK_REPLY, SIM7000_TIMEOUT)) {
            System_printf("Did not get OK after CREG command\n");
            System_flush();
        }
    }
    return sim_connected;
}

/**
 * Enables the bearer for the network
 * @param config: SIM7000 Config structure
 * @return true on success, or false otherwise
 */
static bool enable_bearer(SIM7000_Config *config) {
    char cmd[80];
    // Set up the IP Bearer to connect to the network
    if (!send_verified_reply(config, "AT+SAPBR=3,1,\"Contype\",\"GPRS\"",
                             OK_REPLY, SIM7000_TIMEOUT)) {
        System_printf("Failed to configure bearer for NTP\n");
        return false;
    }
    // Configure APN for bearer
    snprintf(cmd, sizeof(cmd), "AT+SAPBR=3,1,\"APN\",%s", config->apn);
    if (!send_verified_reply(config, cmd, OK_REPLY, SIM7000_TIMEOUT)) {
        System_printf("Failed to set APN for NTP bearer\n");
        return false;
    }
    delay_ms(500); // seems to help the next command succeed
    // Open bearer
    if (!send_verified_reply(config, "AT+SAPBR=1,1", OK_REPLY,
                             SIM7000_NETWORK_TIMEOUT)) {
        System_printf("Failed to open bearer for NTP sync\n");
        return false;
    }
    // Set NTP to use bearer 1
    if (!send_verified_reply(config, "AT+CNTPCID=1", OK_REPLY,
                             SIM7000_TIMEOUT)) {
        System_printf("Failed to set bearer for NTP functionality\n");
        // Close bearer
        send_verified_reply(config, "AT+SAPBR=0,1", OK_REPLY, SIM7000_TIMEOUT);
        return false;
    }
    return true;
}

/**
 * Delays a provided number of milliseconds. NOTE: this module assumes that
 * the clock tick period is 1000us, as set in "flood-msp432-firmware.cfg"
 * @param ms: number of milliseconds to delay.
 */
static void delay_ms(uint32_t ms) {
    // Since clock tick is every millisecond, task_sleep works fine.
    // this WOULD NOT be the case if the clock tick period changed.
    Task_sleep(ms);
}

/**
 * Boots the SIM7000a up.
 * to do so, we must pull the PWKKEY pin low, then pull it high.
 */
static void boot_sim7000a(SIM7000_Config *config) {
    // Set up the required GPIO pins (both should start high)
    GPIO_setConfig(config->powerkey_pin, GPIO_CFG_OUTPUT | GPIO_CFG_OUT_HIGH);
    GPIO_setConfig(config->reset_pin, GPIO_CFG_OUTPUT | GPIO_CFG_OUT_HIGH);
    // Now, write a low value to the GPIO pin
    delay_ms(50); // delay 50 ms so sim sees high pin
    GPIO_write(config->powerkey_pin, GPIO_LOW);
    delay_ms(SIM7000_PWRPULSE);
    GPIO_write(config->powerkey_pin, GPIO_HIGH);
}

/**
 * Flushes the input of a UART device
 * @param config: SIM7000 configuration structure
 */
static void flush_input(SIM7000_Config *config) {
    char tmp;
    Debug_printf("%s", "Flush:");
    if (uart_available(config)) {
        Watchdog_clear(watchdogHandle);
        while (UART_read(config->uart, &tmp, 1) == 1) {
            Debug_printf("%c", tmp);
        }
    }
    Debug_printf("%c", '\n');
}

/**
 * Sends a command on the UART handle, and verifies that the response is the
 * expected value
 * @param config: SIM7000 configuration structure
 * @param cmd: command to send
 * @param expected: expected reply
 * @param timeout: amount of time to wait for response in ms
 * @return true if output of command matches expected value
 */
static bool send_verified_reply(SIM7000_Config *config, const char *cmd,
                                const char *expected, int timeout) {
    int response_len, expected_len;
    expected_len = strlen(expected);
    // Zero out the segment of the buffer we will use, to avoid false
    // positives
    memset(config->replybuffer, 0, expected_len);
    response_len = get_reply(config, cmd, timeout);
    Debug_printf("SIM7000: expected: %s, actual %s, length %i\n", expected,
                 config->replybuffer, response_len);
    return strncmp(config->replybuffer, expected, expected_len) == 0;
}

/**
 * Sends a command to the SIM, and gets the response
 * @param config: SIM7000 configuration structure
 * @param send: command string to send
 * @param timeout: amount of time to wait for response in ms
 * @return number of characters in response
 */
static uint8_t get_reply(SIM7000_Config *config, const char *send,
                         int timeout) {
    const char newline[] = "\r\n";
    // Send entire string
    if (UART_write(config->uart, send, strlen(send)) < 0) {
        System_abort("Could not write to SIM UART\n");
    }
    // Send newline
    if (UART_write(config->uart, newline, 2) < 0) {
        System_abort("Could not write to SIM_UART\n");
    }
    return sim_readline(config, timeout);
}

/**
 * Read a single line response from the SIM
 * @param config: SIM7000 config structure
 * @return number of chars in response
 */
static uint8_t sim_readline(SIM7000_Config *config, int timeout) {
    uint16_t replyidx = 0;
    char c;
    bool complete = false;
    /*
     * Using a timeout allows us to use a shorted UART read timeout.
     * This means we can print updates for the user while we wait for data
     */
    while (timeout > 0 && !complete) {
        // while the SIM module has data, read it
        Watchdog_clear(watchdogHandle);
        while (UART_read(config->uart, &c, 1) == 1) {
            if (c == '\r')
                continue; // Ignore carriage return
            if (c == '\n' && replyidx == 0) {
                // Ignore a newline/linefeed at the start of the response
                continue;
            } else if (c == '\n') {
                // End of response. Break.
                complete = true;
                break;
            }
            config->replybuffer[replyidx] = c;
            replyidx++;
            if (replyidx >= sizeof(config->replybuffer) - 1) {
                Debug_printf("%s", "Size of reply buffer for SIM exceeded\n");
                break;
            }
        }
        if (!complete) {
            timeout -= UART_READ_TIMEOUT;
#if DEBUG
            System_printf("%d ms remain in the UART read timeout\n", timeout);
            System_flush();
#endif
            Task_yield();
        }
    }
    if (!complete) {
        System_printf("Timed out while reading data from SIM7000\n");
        return 0;
    }
    // Null terminate the reply buffer
    config->replybuffer[replyidx] = '\0';
    return replyidx;
}

/**
 * Verifies that the SIM7000 module successfully booted.
 * @param config: SIM7000 configuration structure
 * @return true if the SIM7000 booted and is responding to commands
 */
static bool verify_boot(SIM7000_Config *config) {
    int expected_outputs = 3, num_read, attempts = 6;
    /*
     * During boot the SIM sends several unprompted outputs. The only ones that
     * seem consistent are "+CFUN=1" and "+CPIN: READY", so wait for that here.
     * Other outputs I have seen:
     * DST: 0
     * *PSUTTZ: (current time)
     */
    while (expected_outputs && attempts) {
        num_read = sim_readline(config, SIM7000_BOOT_TIMEOUT);
        System_flush();
        if (strncmp("+CFUN: 1", config->replybuffer, 8) == 0 ||
            strncmp("+CPIN: READY", config->replybuffer, 12) == 0 ||
            strncmp("SMS Ready", config->replybuffer, 10) == 0) {
            expected_outputs--;
            System_printf("Got output %s\n", config->replybuffer);
        } else if (num_read > 0) {
            /**
             * Note: Don't make attempts too low, some outputs like RDY show up
             * rarely
             */
            System_printf("Unexpected output %s\n", config->replybuffer);
            System_flush();
            attempts--;
        } else {
            // SIM may already be booted
            if (send_verified_reply(config, "AT", OK_REPLY, SIM7000_TIMEOUT)) {
                /*
                 * The sim is booted and configured.
                 * We don't expect this here, but we can handle it simply by
                 * returning true
                 */
                cli_log("Warning: SIM found to be already booted when boot "
                        "was attempted\n");
                return true;
            }
            // The sim is not booted, and not responding. Break out here.
            break;
        }
    }
    // Flush the SIM before attempting the AT command
    flush_input(config);
    /*
     * The SIM7000 respond to AT with "OK" at boot. It
     * boots with the echo mode enabled, which means that it should
     * start by echoing "AT" back to the user. We therefore expect to
     * see AT first.
     */
    if (send_verified_reply(config, "AT", "AT", SIM7000_TIMEOUT)) {
        // Verify that we received the "OK"
        if (verified_readline(config, OK_REPLY, SIM7000_TIMEOUT)) {
            /*
             * We know the SIM is online, but ECHO is enabled. we don't
             * need echo, so we disable it here.
             */
            flush_input(config);
            if (send_verified_reply(config, "ATE0", "ATE0", SIM7000_TIMEOUT)) {
                // Verify we recieved the "OK"
                if (verified_readline(config, OK_REPLY, SIM7000_TIMEOUT)) {
                    Debug_printf("%s", "Successfully disabled echo on SIM\n");
                    cli_log("SIM Booted\n");
                    return true;
                } else {
                    System_printf("SIM responded to AT, but could not "
                                  "disable echo\n");
                    System_flush();
                    return false;
                }
            } else {
                System_printf("SIM responded to AT, but could not "
                              "disable echo\n");
                System_flush();
                return false;
            }
        }
    } else if (strncmp(config->replybuffer, OK_REPLY, 2) == 0) {
        /*
         * The sim is booted and configured.
         * We don't expect this here, but we can handle it simply by returning
         * true
         */
        cli_log("Warning: SIM found to be already booted when boot was "
                "attempted\n");
        return true;
    } else {
        return false;
    }
    return false;
}

/**
 * Checks if the UART device has characters available to read
 * @param config: SIM7000 config structure
 */
static bool uart_available(SIM7000_Config *config) {
    int count;
    /*
     * NOTE: UART_CMD_ISAVAILABLE should theoretically serve the same
     * purpose as getting the RX count, but it is unreliable, and sometimes
     * returns false negatives, preventing us from reading the UART. Don't
     * use it.
     */
    if (UART_control(config->uart, UART_CMD_GETRXCOUNT, &count) !=
        UART_STATUS_SUCCESS) {
        System_abort("Error getting UART_AVALIABLE\n");
    }
    return count != 0;
}

/**
 * Reconfigure the baud rate of the SIM UART
 * @param config: SIM7000 Configuration structure
 * @param baudrate: new baudrate
 */
static void reconfigure_baud(SIM7000_Config *config, uint32_t baudrate) {
    UART_Params params;
    int i;
    // command cannot be longer than 17 chars with max baud rate (4000000)
    char set_baud[17], cmd_len;
    for (i = 0; i < sizeof(sim7000_baudrates) / sizeof(uint32_t); i++) {
        if (sim7000_baudrates[i] == baudrate) {
            break;
        }
    }
    if (i == sizeof(sim7000_baudrates) / sizeof(uint32_t)) {
        System_abort("Illegal baud rate\n");
    }
    cmd_len = snprintf(set_baud, sizeof(set_baud), "AT+IPR=%lu\r\n", baudrate);
    UART_write(config->uart, set_baud, cmd_len);
    delay_ms(200);
    flush_input(config);
    UART_close(config->uart);
    // Reopen UART
    UART_Params_init(&params);
    params.baudRate = baudrate;
    params.readDataMode = UART_DATA_BINARY;
    params.writeDataMode = UART_DATA_BINARY;
    config->uart = UART_open(CONFIG_SIM_UART, &params);
    if (!config->uart) {
        System_abort("Could not reopen sim UART\n");
    }
    Debug_printf("Configured Baud Rate to %d\n", baudrate);
}

/**
 * Does NOT send data, simply reads a line from the SIM input and verifies
 * that the contents match the expected value
 * @param config: SIM config structure
 * @param expected: expected string
 * @param timeout: timeout to wait for response in ms
 * @return: true if read data matches expected, false otherwise
 */
static bool verified_readline(SIM7000_Config *config, const char *expected,
                              int timeout) {
    uint16_t num_read, expected_len;
    expected_len = strlen(expected);
    // Zero out the segment of the buffer we will use, to avoid false
    // positives
    memset(config->replybuffer, 0, expected_len);
    num_read = sim_readline(config, timeout);
    Debug_printf("SIM7000: expected: %s, actual %s, length %i\n", expected,
                 config->replybuffer, num_read);
    return (strncmp(expected, config->replybuffer, expected_len) == 0);
}

/**
 * Sends an HTTP connection request. Don't use this method externally, use
 * the SIM7000_http_* methods.
 * @param config: SIM7000 Config structure
 * @param request: Request configuration structure
 * @param method: HTTP method to use
 * @return number of bytes read in response, or negative value if error.
 */
static int http_generic(SIM7000_Config *config, HTTPConnectionRequest *request,
                        int method) {
    int data_len;
    char cmd[80];
    // Verify that IP State is initial
    set_ip_initial(config);
    disconnect_http(config);
    disable_app_network(config);
    if (!enable_network(config)) {
        System_printf("Failed to enable SIM network\n");
        return -1;
    }
    // Set up the APN
    if (enable_app_network(config) < 0) {
        System_printf("Failed to enable app network\n");
        return -1;
    }

    // Connect to the HTTP server
    if (connect_http(config, request) < 0) {
        System_printf("Failed to enable http connection\n");
        return -1;
    }

    // Set request headers.
    if (set_http_headers(config, request) < 0) {
        System_printf("Failed to set headers\n");
        // Drop http connection
        disconnect_http(config);
        disable_app_network(config);
        return -1;
    }

    /*
     * If the HTTP request is a POST, PUT, or PATCH, it should have a body.
     */
    if (request->body_len &&
        (method == HTTP_POST_CODE || method == HTTP_PUT_CODE ||
         method == HTTP_PATCH_CODE)) {
        if (add_http_body(config, request) < 0) {
            disconnect_http(config);
            disable_app_network(config);
            return -1;
        }
    }

    // Now, make the HTTP request
    snprintf(cmd, sizeof(cmd), "AT+SHREQ=\"%s\",%d", request->path, method);
    if (!send_verified_reply(config, cmd, OK_REPLY, SIM7000_NETWORK_TIMEOUT)) {
        System_printf("HTTP request failed\n");
        disconnect_http(config);
        disable_app_network(config);
        return -1;
    }

    // Read the response
    data_len = read_http_response(config, request);
    if (data_len < 0) {
        System_printf("Failed to read HTTP response\n");
        disconnect_http(config);
        disable_app_network(config);
        return -1;
    }

    // Data has been read. Close HTTP connection.
    disconnect_http(config);
    disable_app_network(config);
    return data_len;
}

/**
 * Reads data from a completed HTTP request
 * @param config: SIM7000 config structure
 * @param request: HTTP request configuration structure
 * @return number of bytes read on success, or negative value on error
 */
static int read_http_response(SIM7000_Config *config,
                              HTTPConnectionRequest *request) {
    char *token, cmd[80];
    uint16_t response_code, data_len;
    /*
     * Next line from the device will have the HTTP response code and data
     * length. Parse it.
     * reply takes the following format:
     * +SHREQ: "GET",[response code],[data length]
     */
    sim_readline(config, SIM7000_NETWORK_TIMEOUT);
    token = strtok(config->replybuffer, ",");
    if (!token) {
        System_printf("Invalid response from HTTP SHREQ\n");
        return -1;
    }
    token = strtok(NULL, ",");
    if (!token) {
        System_printf("Invalid response from HTTP SHREQ\n");
        return -1;
    }
    response_code = atoi(token);
    token = strtok(NULL, ",");
    if (!token) {
        System_printf("Invalid response from HTTP SHREQ\n");
        return -1;
    }
    data_len = atoi(token);
    request->response_code = response_code;
    /*
     * Now, read the data in. Note that this can only read 2048 bytes, see
     * AT command manual for SHREAD to see how to get more
     */
    if (data_len > 2048) {
        System_printf("Too much data to read from SIM. Read AT manual and "
                      "update codebase\n");
        return -1;
    }
    if (data_len > request->response_len) {
        System_printf("Error: output buffer too small to hold returned data\n");
        return -1;
    }
    snprintf(cmd, sizeof(cmd), "AT+SHREAD=0,%d", data_len);
    if (!send_verified_reply(config, cmd, OK_REPLY, SIM7000_TIMEOUT)) {
        System_printf("Failed to start data read\n");
        return -1;
    }
    snprintf(cmd, sizeof(cmd), "+SHREAD: %d", data_len);
    delay_ms(5000);
    if (!verified_readline(config, cmd, SIM7000_NETWORK_TIMEOUT)) {
        /*
         * Note: at this point HTTP request did succeed, we just didn't
         * get a response. See if the response was actually the data
         * (sometimes it is). If not, just exit returning 0. That way
         * calling code does not assume an error and retry transmission
         */
        System_printf("Did not get prompt before data output\n");
        if (strlen(config->replybuffer) == 0) {
            return 0;
        } else {
            // Assume that the data read was the return data from the HTTP call
            return strlen(config->replybuffer);
        }
    }

    /*
     *  Now, read from the connection.
     */
    if (read_to_buffer(config, request->response, data_len) < data_len) {
        System_printf("Not all data could be read. You may need to lower your "
                      "baud rate.\n");
        /*
         * Note: at this point HTTP request did succeed, we just didn't
         * get a response. Don't return -1, just return 0 since we didn't get
         * reponse data
         */
        return 0;
    }
    return data_len;
}

/**
 * Writes an HTTP body to the SIM module in preperation for making HTTP request
 * @param config: SIM7000 config structure
 * @param request: HTTP request configuration structure
 * @return 0 on success, or -1 on error
 */
static int add_http_body(SIM7000_Config *config,
                         HTTPConnectionRequest *request) {
    uint16_t i = 0;
    char cmd[80];
    /*
     * Rather than printing the entire body into the cmd buffer, we will
     * write it directly to save the time and space of copying a lot of
     * memory.
     */
    // Start by writing the AT command to set the body (and opening quote).
    if (UART_write(config->uart, "AT+SHBOD=\"", 10) < 0) {
        System_abort("Could not write to SIM UART\n");
    }
    /*
     * Now write the body itself. The SIM7000 expects quotation marks in the
     * body to be escaped, so we will write escape characters when we see
     * a quotation mark in the body.
     * Example of how to send body from command manual:
     * AT+SHBOD="{\"title\":\"Hello http server\"}",29
     */
    while (i < request->body_len) {
        if (request->body[i] == '"') {
            // Write escape char
            if (UART_write(config->uart, "\\", 1) < 0) {
                System_abort("Could not write to SIM UART\n");
            }
        }
        if (UART_write(config->uart, &request->body[i], 1) < 0) {
            System_abort("Could not write to SIM UART\n");
        }
        i++;
    }
    // Now write the data length (and closing quote), and submit the command.
    snprintf(cmd, sizeof(cmd), "\",%d", request->body_len);
    if (!send_verified_reply(config, cmd, OK_REPLY, SIM7000_TIMEOUT)) {
        System_printf("Failed to set http body before sending\n");
        return -1;
    }
    return 0;
}

/**
 * Reads data from the SIM until a timeout occurs or a given length is read
 * @param config: SIM7000 Config structure
 * @param output: buffer to write data into
 * @param len: length to read into buffer, when this length is hit the call will
 * return
 * @return number of bytes read
 */
static int read_to_buffer(SIM7000_Config *config, uint8_t *output, int len) {
    int buf_idx = 0, timeout = SIM7000_LONG_TIMEOUT;
    while (timeout > 0 && buf_idx < len) {
        Watchdog_clear(watchdogHandle);
        if (UART_read(config->uart, &output[buf_idx], 1)) {
            // Got data
            buf_idx++;

        } else {
// Decrease timeout
#if DEBUG
            System_printf(
                "%d ms left in read buffer timeout, with %d bytes read\n",
                timeout, buf_idx);
            System_flush();
#endif
            timeout -= UART_READ_TIMEOUT;
        }
    }
    return buf_idx;
}

/**
 * Sets headers for an HTTP request
 * @param config: SIM7000 Configuration structure
 * @param request: HTTP request structure
 * @return 0 on success, or negative value on error
 */
static int set_http_headers(SIM7000_Config *config,
                            HTTPConnectionRequest *request) {
    int i;
    char cmd[80];
    // Clear out headers
    if (!send_verified_reply(config, "AT+SHCHEAD", OK_REPLY, SIM7000_TIMEOUT)) {
        System_printf("Failed to clear headers\n");
        return -1;
    }
    // Set default headers
    if (!send_verified_reply(config, "AT+SHAHEAD=\"Connection\",\"keep-alive\"",
                             OK_REPLY, SIM7000_TIMEOUT)) {
        System_printf("Failed to set header\n");
        return -1;
    }
    // Now set custom headers, if any were provided.
    for (i = 0; i < request->header_count; i++) {
        // Cowabunga it is
        snprintf(cmd, sizeof(cmd), "AT+SHAHEAD=\"%s\",\"%s\"",
                 request->headers[i].key, request->headers[i].value);
        if (!send_verified_reply(config, cmd, OK_REPLY, SIM7000_TIMEOUT)) {
            System_printf("Failed to set header\n");
            return -1;
        }
    }
    return 0;
}

/**
 * Starts an HTTP request to remote server. Data is not send, connection is
 * simply opened.
 * @param config: SIM7000 Config structure
 * @param request: http request config structure
 * @return 0 on success, or negative value on error.
 */
static int connect_http(SIM7000_Config *config,
                        HTTPConnectionRequest *request) {
    char cmd[80];
    // Now set up HTTP connection
    snprintf(cmd, sizeof(cmd), "AT+SHCONF=\"URL\",\"http://%s:%d\"",
             request->endpoint, request->port);
    if (!send_verified_reply(config, cmd, OK_REPLY, SIM7000_TIMEOUT)) {
        System_printf("Failed to set URL\n");
        return -1;
    }
    // We must set the HTTP header length and body len
    if (!send_verified_reply(config, "AT+SHCONF=\"BODYLEN\",1024", OK_REPLY,
                             SIM7000_TIMEOUT)) {
        System_printf("Failed to set body length\n");
        return -1;
    }
    if (!send_verified_reply(config, "AT+SHCONF=\"HEADERLEN\",350", OK_REPLY,
                             SIM7000_TIMEOUT)) {
        System_printf("Failed to set header length\n");
        return -1;
    }
    // Now, connect to the remote server
    if (!send_verified_reply(config, "AT+SHCONN", OK_REPLY,
                             SIM7000_NETWORK_TIMEOUT)) {
        System_printf("Failed to connect to HTTP server\n");
        return -1;
    }
    return 0;
}

/**
 * Disconnects a running HTTP connection, if one exists
 * @param config: SIM7000 configuration structure
 */
static void disconnect_http(SIM7000_Config *config) {
    // Check if HTTP is currently connected
    get_reply(config, "AT+SHSTATE?", SIM7000_TIMEOUT);
    if (strncmp("+SHSTATE: 1", config->replybuffer, 11) == 0) {
        // There is an additional "OK" in the input, flush it
        sim_readline(config, SIM7000_TIMEOUT);
        // HTTP is enabled, disable it
        System_printf("Disabling HTTP connection\n");
        if (!send_verified_reply(config, "AT+SHDISC", OK_REPLY,
                                 SIM7000_TIMEOUT)) {
            System_abort("HTTP connection could not be disabled\n");
        }
    } else {
        // Still need to flush the "OK"
        sim_readline(config, SIM7000_TIMEOUT);
    }
}

/**
 * Resets the IP State to IP INITIAL
 * @param config: SIM7000 Config structure
 * @return -1 on error, or zero on success
 */
static int set_ip_initial(SIM7000_Config *config) {
    // Start by verifying the IP status is IP INITIAL
    if (!send_verified_reply(config, "AT+CIPSTATUS", OK_REPLY,
                             SIM7000_TIMEOUT)) {
        System_printf("Could not read IP status from sim\n");
        return -1;
    }
    // Now read the next line from the SIM, it should give the IP status
    if (!verified_readline(config, "STATE: IP INITIAL", SIM7000_TIMEOUT)) {
        Debug_printf("%s", "Resetting SIM IP STATE\n");
        // We need to reset the SIM IP state, it is not IP INITIAL
        if (!send_verified_reply(config, "AT+CIPSHUT", "SHUT OK",
                                 SIM7000_TIMEOUT)) {
            System_printf("Could not reset SIM IP status\n");
            return -1;
        }
        // Verify the IP status was reset
        if (!send_verified_reply(config, "AT+CIPSTATUS", OK_REPLY,
                                 SIM7000_TIMEOUT)) {
            System_printf("Could not read new SIM IP status\n");
            return -1;
        }
        if (!verified_readline(config, "STATE: IP INITIAL", SIM7000_TIMEOUT)) {
            System_printf("SIM did not reset IP status\n");
            return -1;
        }
    }
    return 0;
}

/**
 * Disables the SIM7000 App network
 * @param config: SIM7000 Config structure
 */
static void disable_app_network(SIM7000_Config *config) {
    // First, check the app network status
    get_reply(config, "AT+CNACT?", SIM7000_TIMEOUT);
    // Now, look at the start of the response to see if the network is on.
    if (strncmp("+CNACT: 1", config->replybuffer, 9) == 0) {
        // There will be an additional "OK" in the input.
        sim_readline(config, SIM7000_TIMEOUT);
        // Network is online. We need to disable it
        if (!send_verified_reply(config, "AT+CNACT=0", OK_REPLY,
                                 SIM7000_TIMEOUT)) {
            System_abort("Could not disable app network\n");
        }
        delay_ms(200); // This lets the network go down completely.
        if (!verified_readline(config, "+APP PDP: DEACTIVE",
                               SIM7000_NETWORK_TIMEOUT)) {
            System_abort("App network did not disable\n");
        }
    } else {
        // There will be an additional "OK" in the input.
        sim_readline(config, SIM7000_TIMEOUT);
    }
}

/**
 * Enables the SIM7000 App network
 * @param config: SIM7000 Config structure
 */
static int enable_app_network(SIM7000_Config *config) {
    char cmd[40];
    snprintf(cmd, sizeof(cmd), "AT+CGDCONT=1,\"IP\",\"%s\"", config->apn);
    if (!send_verified_reply(config, cmd, OK_REPLY, SIM7000_TIMEOUT)) {
        System_printf("Failed to set APN\n");
        return -1;
    }
    if (!send_verified_reply(config, "AT+CNACT=1", OK_REPLY, SIM7000_TIMEOUT)) {
        System_printf("Failed to connect to network\n");
        return -1;
    }
    if (!verified_readline(config, "+APP PDP: ACTIVE",
                           SIM7000_NETWORK_TIMEOUT)) {
        System_printf("SIM did not activate PDP\n");
        get_reply(config, "AT+CNACT=0",
                  SIM7000_TIMEOUT); // try to disable app network
        return -1;
    }
    return 0;
}

/**
 * Parse a time string, like the one returned from AT+CCLK?
 * @param str: String returned from AT+CCLK?
 * @param time: timespec struct to be filled by the function
 * @return 0 on success, or -1 on error
 */
static int parse_time(char *str, struct timespec *time) {
    struct tm t;
    char *token;
    /*
     * Time string will be formatted like this:
     * +CCLK: "21/02/05,00:22:50-24"
     * We'll use strtok to parse it.
     */

    // Parse past the "+CCLK"
    token = strtok(str, "\"");
    if (!token) {
        System_printf("Malformed CCLK response\n");
        return -1;
    }
    // Parse the year.
    token = strtok(NULL, "/");
    if (!token) {
        System_printf("Malformed CCLK response\n");
        return -1;
    }
    t.tm_year = atoi(token) + 100; // year since 1900
    /*
     * Note: SIMCOM apparently forgot about Y2K, and opted to format the year
     * with 2 digits. Best workaround I can think of is to reject a year before
     * 2021.
     */
    if (t.tm_year < 121) {
        System_printf(
            "The year does not make sense. Firmware was written in 2021\n");
        return -1;
    }
    // Parse the month
    token = strtok(NULL, "/");
    if (!token) {
        System_printf("Malformed CCLK response\n");
        return -1;
    }
    t.tm_mon = atoi(token);
    // Parse the day
    token = strtok(NULL, ",");
    if (!token) {
        System_printf("Malformed CCLK response\n");
        return -1;
    }
    t.tm_mday = atoi(token);
    // Parse hours
    token = strtok(NULL, ":");
    if (!token) {
        System_printf("Malformed CCLK response\n");
        return -1;
    }
    t.tm_hour = atoi(token);
    // Parse minutes
    token = strtok(NULL, ":");
    if (!token) {
        System_printf("Malformed CCLK response\n");
        return -1;
    }
    t.tm_min = atoi(token) - 1; // January should be 0, not 1
    // Parse seconds
    token = strtok(NULL, "-");
    if (!token) {
        System_printf("Malformed CCLK response\n");
        return -1;
    }
    t.tm_sec = atoi(token);
    t.tm_isdst = -1;
    /*
     * Calculate the unix timestamp. This is a timestamp represented as the
     * number of seconds since 1970.
     *
     * Source: https://www.epochconverter.com/programming/c
     */
    time->tv_nsec = 0; // we can't do nanosecond resolution
    time->tv_sec = mktime(&t);
    return 0;
}

/**
 * Disables the majority of "Unsolicited Result Codes" The SIM
 * can produce, to minimize the possibility of one being produced when we expect
 * a different output
 * @param config: SIM7000 config structure
 * @return true on success, or false on error
 */
static bool disable_result_codes(SIM7000_Config *config) {
    if (!send_verified_reply(config, "AT+CRC=0", OK_REPLY, SIM7000_TIMEOUT)) {
        System_printf("Failed to disable call indicator\n");
        return false;
    }
    if (!send_verified_reply(config, "AT+CRC=0", OK_REPLY, SIM7000_TIMEOUT)) {
        System_printf("Failed to disable call indicator\n");
        return false;
    }
    if (!send_verified_reply(config, "AT+CREG=0", OK_REPLY, SIM7000_TIMEOUT)) {
        System_printf("Failed to disable MT network indicator\n");
        return false;
    }
    if (!send_verified_reply(config, "AT+CNMI=0,0", OK_REPLY,
                             SIM7000_TIMEOUT)) {
        System_printf("Failed to disable SMS result codes\n");
        return false;
    }
    if (!send_verified_reply(config, "AT+CLTS=0", OK_REPLY, SIM7000_TIMEOUT)) {
        System_printf("Failed to disable local time result code\n");
        return false;
    }
    return true;
}
