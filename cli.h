/**
 * @file cli.h
 * CLI implementation for the MSP432. Allows for basic commands
 * to be run for debugging.
 * Created on: Jan 23, 2021
 * Author: danieldegrasse
 */

#ifndef CLI_H_
#define CLI_H_

#define CLI_BAUDRATE 115200     /**< UART baudrate of CLI */
#define CLI_TASK_STACK_MEM 4096 /**< size of the task stack */
#define CLI_TASK_PRIORITY 2     /**< priority of task */

/**
 * Performs required initialization for the CLI, including setting up the UART.
 */
void cli_init();

/**
 * Runs the CLI.
 * @param arg0: Unused
 * @param arg1: Unused
 */
void cli_run(UArg arg0, UArg arg1);

/**
 * Register a function with the CLI
 * @param command name of command when called from the CLI
 * @param help help string for the command
 * @param function function to run when command is called. First argument to
 *  function will be the number of arguments, second will be an array holding
 *  the arguments.
 */
void register_cli_function(char *command, char *help,
                           int (*function)(int, char *[]));

/**
 * Prints data to the CLI and write to the log file. Takes printf style
 * arguments.
 * @param format: printf format string
 */
void cli_log(const char *format, ...);

/**
 * Prints data to the CLI. Takes printf style arguments.
 * @param format: printf format string
 */
void cli_write(const char *format, ...);

#endif /* CLI_H_ */
