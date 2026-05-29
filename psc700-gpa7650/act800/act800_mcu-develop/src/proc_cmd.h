/**
 * @file    proc_cmd.h
 * @brief   Command parser interface definitions.
 * @details This file declares the data structures and function interfaces
 *          required for command parsing and execution. It defines a
 *          table-driven command dispatch mechanism that maps input
 *          command strings to corresponding handler functions.
 *
 *          The command parser is responsible for interpreting commands
 *          received from communication interfaces such as USB CDC or UART.
 * @ingroup command_parser
 */
/* ************************************************************************** */

#ifndef _PROC_CMD_H    /* Guard against multiple inclusion */
#define _PROC_CMD_H


/* ************************************************************************** */
/* ************************************************************************** */
/* Section: Included Files                                                    */
/* ************************************************************************** */
/* ************************************************************************** */


/* Provide C++ Compatibility */
#ifdef __cplusplus
extern "C" {
#endif

    typedef void (*OPERATION_FUNC)(const char params[][50], uint8_t size);
/**
 * @brief   Command operation mapping structure.
 * @details This structure defines the mapping between a command string
 *          and its corresponding handler function. It is typically used
 *          in a command table for command parsing and execution.
 *
 * @note    Each entry associates a command keyword with a specific
 *          operation function.
 */
    typedef struct {
        const char * cmdstr;
        const OPERATION_FUNC function;
    } operation_t;

    typedef struct {
        char command[50];
        char params[10][50];
        uint8_t paramCount;
    } parsed_data_t;

    void process_cmd(char cmdstr[], size_t size);
    extern void ack_with_info(const char* seq, const char* result);

/* Provide C++ Compatibility */
#ifdef __cplusplus
}
#endif

#endif /* _PROC_CMD_H */

/* *****************************************************************************
 End of File
 */
