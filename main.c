/*
 * Copyright (c) 2015-2019, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ======== main.c ========
 */

/* Register Level Includes */
#include "ti/devices/msp432p4xx/inc/msp.h"
/* XDC Module Headers */
#include <xdc/runtime/System.h>
#include <xdc/std.h>

/* BIOS Module Headers */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Task.h>

/* TI Drivers */
#include "ti_drivers_config.h"
#include <ti/drivers/Board.h>
#include <ti/drivers/GPIO.h>
#include <ti/drivers/SDFatFS.h>
#include <ti/drivers/UART.h>
#include <ti/drivers/Watchdog.h>
/* Standard libs */
#include <stdlib.h>

#include "cli.h"
#include "common.h"
#include "radar.h"
#include "storage.h"
#include "transmission.h"

// Delay to wait before updating the clock using network time (roughly in ms)
#define NETWORK_TIME_DELAY 259200000LL

/*
 * Global configuration structure
 * These are the default values
 * If an SD card is inserted, the config.txt file can override them.
 */
ProgramConfiguration program_config = {
    "HW_ID",       // Device hardware ID
    1,             // Device synthetic ID
    true,          // Radar module enabled?
    true,          // Camera module enabled?
    true,         // network module enabled?
    "3.21.41.182", // remote server for network module to talk to
    "f94ed0427c1d5d54b4308fe8c1aa7e03703d4bbd", // Auth token for server
    15000,                                      // radar sample interval in ms
    55, // number of radar samples to take every time the interval fires
    0.0 // Distance to offset radar samples by (subtracts)
};
// Watchdog handle implemenation, used across code for watchdog timer
Watchdog_Handle watchdogHandle;
// Interval counting system ticks since last clock update
static long long clockupdate_interal;
static Semaphore_Handle mainTaskSem;

static void main_task(UArg arg0, UArg arg1);
static void clock_handle(UArg arg1);
// static void init_wait();
/*
 *  ======== main ========
 */
int main() {
    Task_Params taskParams;
    // Call init guard (remove this once board is functional
    // init_wait();
    /* Call driver init functions */
    Board_init();
    UART_init();
    SDFatFS_init();
    GPIO_init();

    Watchdog_init();
    Watchdog_Params params;

    Watchdog_Params_init(&params);
    params.resetMode = Watchdog_RESET_ON;
    watchdogHandle = Watchdog_open(CONFIG_WATCHDOG_0, &params);
    // Watchdog_clear(watchdogHandle);

    // Create the initialization task
    Task_Params_init(&taskParams);
    taskParams.stackSize = 2048;
    taskParams.priority = 1;
    if (!Task_create(main_task, &taskParams, NULL)) {
        System_abort("Could not create init task");
    }
    // Start the BIOS Kernel.
    BIOS_start();
    return 0;
}

/**
 * Simple init routine that flashes the D1 LED and waits for GPIO0 to be
 * pressed. Written at the register level to ensure this code is not likely to
 * crash CPU.
 */
// static void init_wait() {
//    volatile uint32_t i;
//
//    WDT_A->CTL = WDT_A_CTL_PW |             // Stop WDT
//                 WDT_A_CTL_HOLD;
//
//    P9->DIR |= BIT1;                        // P9.1 set as output (Connected
//    to D2) P4->DIR &= ~BIT5;                       // P4.5 set as input
//    (connected to GPIO0)
//    // Enable P4 pulldown resistor
//    P4->REN |= BIT5;
//    P4->OUT &= ~BIT5;
//    // Disable alternate functions for P4.5
//    P4->SEL0 &= ~BIT5;
//    P4->SEL1 &= ~BIT5;
//
//    /*
//     * While P4.5 is low, wait for it to be pulled high
//     */
//    while ((P4->IN & BIT5) == 0)
//    {
//        P9->OUT ^= BIT1;                    // Blink P9.1 LED
//        for (i = 2000000; i > 0; i--);        // Delay
//
//    }
//}

/**
 * Main Task. Handles starting all program tasks, and then enters program run
 * loop. Run loop reads radar data packets at a defined interval, and
 * periodically updates the rtc to match network time
 * @param arg0 unused
 * @param arg1 unused
 */
static void main_task(UArg arg0, UArg arg1) {
    Task_Params taskParams;
    Clock_Params clockParams;
    Clock_Handle clock;
    /* Call task init functions */
    cli_init();
    storage_init();
    // Read configuration file
    read_configuration();
    if (program_config.radar_module_enabled) {
        radar_init();
    } else {
        // Ensure radar board is powered down
        GPIO_setConfig(CONFIG_GPIO_RADAR_PMIC_EN,
                       GPIO_CFG_OUTPUT | GPIO_CFG_OUT_LOW);
    }
    if (program_config.network_enabled) {
        transmission_init();
    } else {
        // Ensure transmission module will remain off
        GPIO_setConfig(CONFIG_GPIO_SIM_PWRKEY,
                       GPIO_CFG_OUTPUT | GPIO_CFG_OUT_HIGH);
    }
    /*
     * Start all required tasks
     */
    if (program_config.radar_module_enabled) {
        Task_Params_init(&taskParams);
        taskParams.stackSize = RADAR_TASK_STACK_MEM;
        taskParams.priority = RADAR_TASK_PRIORITY;
        if (Task_create(radar_run, &taskParams, NULL) == NULL) {
            System_abort("Sensor task creation failed\n");
        }
    }
    Task_Params_init(&taskParams);
    taskParams.stackSize = TRANSMISSION_TASK_STACK_MEM;
    taskParams.priority = TRANSMISSION_TASK_PRIORITY;
    if (Task_create(storage_run, &taskParams, NULL) == NULL) {
        System_abort("Storage task creation failed\n");
    }
    if (program_config.network_enabled) {
        Task_Params_init(&taskParams);
        taskParams.stackSize = STORAGE_TASK_STACK_MEM;
        taskParams.priority = STORAGE_TASK_PRIORITY;
        if (Task_create(transmission_run, &taskParams, NULL) == NULL) {
            System_abort("Transmission task creation failed\n");
        }
    }
    Task_Params_init(&taskParams);
    taskParams.stackSize = CLI_TASK_STACK_MEM;
    taskParams.priority = CLI_TASK_PRIORITY;
    if (Task_create(cli_run, &taskParams, NULL) == NULL) {
        System_abort("CLI task creation failed\n");
    }
    System_printf("Flood of ELECs Firmware Starting, Build time %s %s\n",
                  __DATE__, __TIME__);
    // Force all data in the Sysmin buffer to print
    System_flush();
    cli_log("Flood of ELECs Firmware Starting, Build time %s %s\n", __DATE__,
            __TIME__);
    // Set the clock
    request_rtc_update();
    /*
     * Make semaphore to wait for clock to expire
     */
    mainTaskSem = Semaphore_create(0, NULL, NULL);
    if (!mainTaskSem) {
        System_abort("Failed to create main task semaphore");
    }
    /*
     * Create a clock here, that periodically calls the handler function
     * to sample data from the radar board and update the network clock
     */
    Clock_Params_init(&clockParams);
    clockParams.period = program_config.radar_sample_interval;
    clockParams.startFlag = true;
    clock = Clock_create(clock_handle, program_config.radar_sample_interval,
                         &clockParams, NULL);
    if (!clock) {
        System_abort("Could not create main task clock\n");
    }
    while (1) {
        /*
         * Main program run loop. Sample data from the radar board, and
         * periodically update the clock.
         */
        // Get a sample from the radar board.
        sample_radar();
        // Delay until radar sample interval expires.
        Semaphore_pend(mainTaskSem, BIOS_WAIT_FOREVER);
        // Add sample interval to our counter for the clock update
        clockupdate_interal += program_config.radar_sample_interval;
        if (clockupdate_interal >= NETWORK_TIME_DELAY) {
            clockupdate_interal = 0;
            request_rtc_update();
        }
        // Force log files to flush to disk
        sync_to_disk();
    }
}

/**
 * Handler for clock function expiry
 * @param arg1: Unused
 */
static void clock_handle(UArg arg1) { Semaphore_post(mainTaskSem); }
