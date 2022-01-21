/**
 *  @file transmission.c
 *  Implements code to wait for new data to transmit, and to send the data
 *  to a remote server
 *
 *  Created on: Jan 22, 2021
 *  Author: danieldegrasse
 */

/* XDC Module Headers */
#include <xdc/runtime/System.h>
#include <xdc/std.h>

/* BIOS module headers */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/gates/GateMutex.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Queue.h>
#include <ti/sysbios/knl/Task.h>

/* TI drivers */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/UART.h>

/* Standard libs */
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include <time.h>

#include "cli.h"
#include "common.h"
#include "sim7000.h"
#include "ti_drivers_config.h"
///@{
/** Events that can trigger action in the main transmission module */
#define EVT_TX_DATA_AVAIL Event_Id_00 /**< new radar data is available */
#define EVT_UPDATE_CLK Event_Id_03    /**< request to update clock */
///@}

/** fallback timestamp in ms  (if NTP does not work) */
#define FALLBACK_TIMESTAMP 3820858526

#define APN "hologram" /**< APN to use for network */

/** must be a power of 2, max elements in transmission queue */
#define MAX_QUEUE_ELEM 32
/** how many times to attempt to send packet */
#define TRANSMISSION_ATTEMPTS 2

static bool transmission_init_done = false;
static SIM7000_Config sim_config;

/**
 * Queue element to be placed in FIFO queue for transmission to web backend.
 */
typedef struct SensorDataQueueElem {
    Queue_Elem elem;
    SensorDataPacket packet;
} SensorDataQueueElem;

static SensorDataQueueElem queue_elements[MAX_QUEUE_ELEM];
static GateMutex_Handle queueMutex;
static Event_Handle transmissionEventHandle;
static Queue_Handle sensorDataQueue;
static void update_rtc();

/**
 * This function should perform any initialization required for the transmission
 * module to function, including initializing peripherals like UART.
 *
 * This code assumes that the connected device to the UART is a SIM7000A LTE
 */
void transmission_init() {
    /*
     * start the botletics module, and verify it responds to AT commands.
     */
    SIM7000_init_params(&sim_config);
    sim_config.powerkey_pin = CONFIG_GPIO_SIM_PWRKEY;
    sim_config.reset_pin = CONFIG_GPIO_SIM_RST;
    sim_config.UART_index = CONFIG_SIM_UART;
    Watchdog_clear(watchdogHandle);
    strncpy(sim_config.apn, APN, sizeof(sim_config.apn));
    transmissionEventHandle = Event_create(NULL, NULL);
    if (!transmissionEventHandle) {
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
    if (!SIM7000_open(&sim_config)) {
        System_abort("Could not open SIM UART\n");
    }
    // Also configure D2 indicator LED as low output
    GPIO_setConfig(CONFIG_D2_LED, GPIO_CFG_OUTPUT | GPIO_CFG_OUT_LOW);
    transmission_init_done = true;
    System_printf("Transmission init done\n");
}

/**
 * Searches for a SIM by sweeping various baud rates
 * @return: true if sim is found
 */
bool find_sim() {
    if (!program_config.network_enabled) {
        cli_log("Network module disabled\n");
        return false;
    }
    if (!transmission_init_done) {
        cli_log("Transmission not initialized\n");
        return true;
    }
    transmission_init_done = SIM7000_search(&sim_config);
    return transmission_init_done;
}

/**
 * This function should run in a continuous loop, and handle all data
 * transmission from the device.
 * @param arg0: Unused
 * @param arg1: Unused
 */
void transmission_run(UArg arg0, UArg arg1) {
    HTTPConnectionRequest request;
    HTTPHeader headers[2];
    SensorDataQueueElem *elem;
    UInt events;
    int attempts_remaining;
    int return_val;
    struct tm *time_management; // name pending
    IArg mutex_key;
    char http_post_data[128];
    char http_token[6 + TOKEN_STRLEN];
    char response[512];

    if (!transmission_init_done)
        return;
    System_printf("Transmission task starting\n");
    Watchdog_clear(watchdogHandle);
    // Create the token data string now
    snprintf(http_token, sizeof(http_token), "Token %s",
             program_config.server_token);
    // Event loop
    while (1) {
        events =
            Event_pend(transmissionEventHandle, Event_Id_NONE,
                       EVT_TX_DATA_AVAIL | EVT_UPDATE_CLK, BIOS_WAIT_FOREVER);
        /*
         * All transmission events require the SIM to be running, so boot it
         * if one occurred.
         */
        if (!SIM7000_running(&sim_config)) {
            if (!SIM7000_poweron(&sim_config)) {
                cli_log("SIM module failed to boot for transmission\n");
                continue; // No point in trying to handle events
            }
        }
        if (events & EVT_TX_DATA_AVAIL) {
            // While storage has data to be TX'd available, TX data
            while (1) {
                // Get Queue mutex
                mutex_key = GateMutex_enter(queueMutex);
                if (Queue_empty(sensorDataQueue)) {
                    GateMutex_leave(queueMutex, mutex_key);
                    break; // Exit
                }
                // pop element from queue
                elem = Queue_dequeue(sensorDataQueue);
                GateMutex_leave(queueMutex, mutex_key);
                time_management = localtime(
                    &(elem->packet.timestamp)); // Put unix time into tm struct
                snprintf(http_post_data, sizeof(http_post_data),
                         "{\"distance\": %.3f, \"timestamp\": "
                         "\"20%d-%02d-%02dT%02d:%02d:%02d\", \"sensor\": %d}",
                         elem->packet.distance, time_management->tm_year - 100,
                         time_management->tm_mon, time_management->tm_mday,
                         time_management->tm_hour, time_management->tm_min,
                         time_management->tm_sec, program_config.synthetic_id);
                /*
                 * TODO: if having weird errors, add a mutex to protect
                 * program_config
                 */
                request.endpoint = program_config.server_ip;
                request.port = 80;
                request.path = "/api/sensor-data/";
                request.body = (uint8_t *)http_post_data;
                request.body_len = strlen(http_post_data);
                request.response = (uint8_t *)response;
                request.response_code = 0;
                request.response_len = sizeof(response);
                headers[0].key = "Content-Type";
                headers[0].value = "application/json";
                headers[1].key = "Authorization";
                headers[1].value = http_token;
                request.headers = headers;
                request.header_count = 2;

                attempts_remaining = TRANSMISSION_ATTEMPTS;
                while (attempts_remaining > 0) {
                    // Set D2 Led high to indicate a transmission is being
                    // attempted
                    GPIO_write(CONFIG_D2_LED, CONFIG_GPIO_LED_ON);
                    return_val = SIM7000_http_post(&sim_config, &request);
                    // Set D2 Led high to indicate a transmission is over
                    GPIO_write(CONFIG_D2_LED, CONFIG_GPIO_LED_OFF);
                    if (return_val < 0) {
                        // had error
                        System_printf("Error while transmitting to backend\n");
                        System_flush();
                        cli_log("Error while transmitting to backend, HTTP "
                                "code %d\n",
                                request.response_code);
                        attempts_remaining--;
                    } else {
                        System_printf("Succeeded, data response len was %d "
                                      "with HTTP response code %d\n",
                                      return_val, request.response_code);
                        System_flush();
                        if (request.response_code == 201) {
                            // Request succeeded on backend. Exit loop.
                            break;
                        }
                    }
                }
                cli_log("Completed SIM transmission with return val %d and "
                        "HTTP response code %d\n",
                        return_val, request.response_code);
                if (attempts_remaining == 0) {
                    cli_log("Failed to send data to backend, will retry when "
                            "more is available\n");
                    break;
                }
            }
        }
        if (events & EVT_UPDATE_CLK) {
            // Update clock from event trigger
            update_rtc();
        }
        /*
         * This is the end of the loop. We should power down the SIM now.
         */
        if (!SIM7000_poweroff(&sim_config)) {
            cli_log("SIM did not power off, in unknown state\n");
        }
    }
}

/**
 * Transmits sensor data to the backend
 * @param packet Data packet to send
 */
void transmit_sensor_data(SensorDataPacket *packet) {
    if (!program_config.network_enabled || !transmission_init_done) {
        return; // Can't transmit, network not enabled
    }
    Watchdog_clear(watchdogHandle);
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
    Event_post(transmissionEventHandle, EVT_TX_DATA_AVAIL);
}

/**
 * Requests for the transmission task to update the RTC
 */
void request_rtc_update() {
    struct timespec ts;
    if (!program_config.network_enabled || !transmission_init_done) {
        // Can't update rtc without network
        cli_log("Network not enabled, setting RTC directly");
        System_printf("Cannot update RTC without network being enabled, using "
                      "fallback timestamp");
        System_flush();
        ts.tv_nsec = 0;
        ts.tv_sec = FALLBACK_TIMESTAMP;
        clock_settime(CLOCK_REALTIME, &ts);
    } else {
        Event_post(transmissionEventHandle, EVT_UPDATE_CLK);
    }
}

/**
 * This function updates the real time clock with a timestamp from a remote
 * server.
 */
static void update_rtc() {
    struct timespec ts;
    bool sim_powermanage = false;
    bool sim_booted = false;
    if (program_config.network_enabled && transmission_init_done) {
        // Attempt to boot the SIM
        sim_booted = SIM7000_running(&sim_config);
        if (!sim_booted) {
            /*
             * Assume that we should manage the power state of the sim, by
             * booting it up and turning it off in this function
             */
            sim_powermanage = true;
            sim_booted = SIM7000_poweron(&sim_config);
        }
        if (sim_booted) {
            if (SIM7000_ntp_time(&sim_config, &ts) >= 0) {
                // Everything worked, shut down the SIM and set the time
                if (sim_powermanage && !SIM7000_poweroff(&sim_config)) {
                    System_printf("SIM failed to shutdown, in unknown state\n");
                }
                clock_settime(CLOCK_REALTIME, &ts);
                cli_log("network time sync completed\n");
                return;
            }
        }
        // We failed to boot the SIM, warn the user
        System_printf("Failed to start SIM7000\n");
        System_printf("Failed to set timestamp from SIM module\n"
                      "Using fallback timestamp\n");
        System_flush();
        cli_log("Failed to sync with network time\n");
    }
    ts.tv_nsec = 0;
    ts.tv_sec = FALLBACK_TIMESTAMP;
    clock_settime(CLOCK_REALTIME, &ts);
}
