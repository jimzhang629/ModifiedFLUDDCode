/**
 * @file sim7000.h
 * Implements a support library for the SIM7000 LTE Module
 *
 * Created on: Jan 27, 2021
 * Author: danieldegrasse
 */
#ifndef SIM7000_H_
#define SIM7000_H_

#include <stdbool.h>
#include <stdint.h>
#include <time.h>

#include <ti/drivers/UART.h>

#define REPLYBUF_LEN 256 /**< Length of buffer to store data read from SIM */
#define APN_LEN 16;      /**< Length of buffer to store GPRS APN within */

/**
 * Configuration structure for an instance of the SIM7000 driver
 */
typedef struct SIM7000_Config_ {
    uint_least8_t powerkey_pin; /*!< GPIO pin index for SIM7000 PWR pin */
    uint_least8_t reset_pin;    /*!< GPIO pin index for SIM7000 RESET pin */
    uint_least8_t UART_index; /*!< UART config idx for uart connected to sim */
    UART_Handle uart;         /*!< Stores open UART handle, used internally */
    char apn[16];             /*!< APN sim should connect using */
    char
        replybuffer[REPLYBUF_LEN]; /*!< reply buffer for sim, used internally */
    bool sim_running;              /*!< software tracker for if sim is booted */
} SIM7000_Config;

/**
 * A structure to configure a TCP connection request to send to the SIM7000.
 * Used when opening a TCP connection.
 */
typedef struct TCPConnectionRequest {
    char *endpoint;  /*!< DNS or IP address to write data to */
    uint16_t port;   /*!< port to connect to */
    uint8_t *data;   /*!< data buffer to write over connection */
    uint16_t len;    /*!< length of the data buffer */
    uint8_t *out;    /*!< output buffer to write data returned by server to */
    uint8_t out_len; /*!< length of the output buffer */
} TCPConnectionRequest;

/**
 * HTTP head structure. Used when attaching headers to HTTP connection request.
 */
typedef struct HTTPHeader {
    char *key;   /*!< HTTP header key */
    char *value; /*!< HTTP header value */
} HTTPHeader;

/**
 * HTTP connection request. Configure an instance of this structure to set up
 * an HTTP connection, than call the relevant library function (SIM7000_http_*)
 * to make the HTTP connection.
 */
typedef struct HTTPConnectionRequest {
    char *endpoint; /*!< DNS or IP address to write data to */
    char *path;     /*!< URL path component to connect to */
    uint16_t port;  /*!< port to connect to */
    uint8_t *body; /*!< http body to send (only valid for PUT, PATCH, & POST) */
    uint16_t body_len;     /*!< http body length, if one is present */
    uint8_t *response;     /*!< output buffer response will be written to */
    uint16_t response_len; /*!< length of the response output buffer */
    uint16_t
        response_code;    /*!< http response code after return from fxn call */
    HTTPHeader *headers;  /*!< pointer to array of http headers */
    uint8_t header_count; /*!< Number of headers */
} HTTPConnectionRequest;

/**
 * Starts an instance of the SIM7000 driver
 * @param config: configuration object for the SIM7000 driver
 * @return true if opening succeeds, false otherwise
 */
bool SIM7000_open(SIM7000_Config *config);

/**
 * Powers on the SIM7000, and verifies communication is functional
 * @param config: SIM7000 config structure
 * @return true on success, false otherwise
 */
bool SIM7000_poweron(SIM7000_Config *config);

/**
 * Sets up configuration parameters for use with the SIM7000_open function
 * @param config: Configuration structure to initialize
 */
void SIM7000_init_params(SIM7000_Config *config);

/**
 * Searches for a SIM device by sweeping all supported baud rates. If a sim
 * device is found, it will be reconfigured to run at SIM7000_BAUDRATE
 *
 * This function is useful if you have connected the SIM correctly,
 * but it's not responding.
 * @param config: SIM7000 Config structure
 * @return true if a SIM device was found and configured
 */
bool SIM7000_search(SIM7000_Config *config);

/**
 * Sends a block of TCP data to the server, and reads the server's response
 * @param config: SIM7000 config structure
 * @param request: TCP Connection request structure
 * @return: number of bytes read by response (or -1 in case of error)
 */
int16_t SIM7000_tcp(SIM7000_Config *config, TCPConnectionRequest *request);

/**
 * Makes an HTTP GET request using the SIM7000
 * @param config: SIM7000 Configuration structure
 * @param request: HTTP request structure
 * @return: length of read data on success, or negative value on failure
 */
int SIM7000_http_get(SIM7000_Config *config, HTTPConnectionRequest *request);

/**
 * Makes an HTTP PUT request using the SIM7000
 * @param config: SIM7000 Configuration structure
 * @param request: HTTP request structure
 * @return: length of read data on success, or negative value on failure
 */
int SIM7000_http_put(SIM7000_Config *config, HTTPConnectionRequest *request);

/**
 * Makes an HTTP POST request using the SIM7000
 * @param config: SIM7000 Configuration structure
 * @param request: HTTP request structure
 * @return: length of read data on success, or negative value on failure
 */
int SIM7000_http_post(SIM7000_Config *config, HTTPConnectionRequest *request);

/**
 * Makes an HTTP PATCH request using the SIM7000
 * @param config: SIM7000 Configuration structure
 * @param request: HTTP request structure
 * @return: length of read data on success, or negative value on failure
 */
int SIM7000_http_path(SIM7000_Config *config, HTTPConnectionRequest *request);

/**
 * Makes an HTTP HEAD request using the SIM7000
 * @param config: SIM7000 Configuration structure
 * @param request: HTTP request structure
 * @return: length of read data on success, or negative value on failure
 */
int SIM7000_http_head(SIM7000_Config *config, HTTPConnectionRequest *request);

/**
 * Synchronizes the SIM7000 Clock with a network time server, and updates the
 * supplied timespec struct with the current time
 * @param config: SIM7000 Config structure
 * @param time: timespec struct, populated with the current time to second
 *  accuracy
 * @return 0 on success, or negative value on error
 */
int SIM7000_ntp_time(SIM7000_Config *config, struct timespec *time);

/**
 * Closes the connection to the SIM7000 (and powers if off if it is running)
 * @param config: SIM7000 conf structure
 */
void SIM7000_close(SIM7000_Config *config);

/**
 * Sets sim into low current sleep mode. In this mode the sim remains connected
 * to the network, but uses much less power by slowing its clocks (note this
 * mode will not activate when connected to USB)
 * @param config: SIM7000 config structure
 * @param enable: If low current mode should be switched on or off
 * @return true if successful, false otherwise
 */
bool SIM7000_lowpowermode(SIM7000_Config *config, bool enable);

/**
 * Check to see if the SIM is powered on and responding to commands
 * @param config: SIM7000 config structure
 * @return true if the sim is powered on and responsive , false otherwise
 */
bool SIM7000_running(SIM7000_Config *config);

/**
 * Powers down a running SIM module
 * @param config: SIM7000 Config structure
 * @return true on successful poweroff, false otherwise
 */
bool SIM7000_poweroff(SIM7000_Config *config);

#endif /* SIM7000_H_ */
