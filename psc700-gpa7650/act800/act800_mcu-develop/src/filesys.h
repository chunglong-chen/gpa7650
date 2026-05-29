/* 
 * File:   filesys.h
 * Author: A00361
 *
 * Created on 2024.7.17,  10:16
 */
/**
 * @file    filesys.h
 * @brief   SST26 file system management interface definitions.
 * @details This file defines data structures, state definitions, and
 *          public APIs for managing SST26 flash storage through the
 *          file system layer.
 *
 *          It provides interfaces for:
 *          - Disk mount and format control
 *          - File read, write, and removal operations
 *          - Directory access
 *          - Waveform data storage and retrieval
 *          - Checksum calculation and verification
 *          - Persistent storage of system parameters
 * @ingroup data_storage
 */
#ifndef FILESYS_H
#define	FILESYS_H
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <stdio.h>
#include "configuration.h"
#include "system/fs/sys_fs.h"
#include "config/default/peripheral/wdt/plib_wdt.h"
#include "mems.h"
#include "config/default/peripheral/port/plib_port.h"
#include "alg.h"
#include "sld.h"
#ifdef	__cplusplus
extern "C" {
#endif
#define TESTBUFFER_SIZE                (4096U)

    // *****************************************************************************

    /* Application states

      Summary:
        Application states enumeration

      Description:
        This enumeration defines the valid application states.  These states
        determine the behavior of the application at various times.
     */

    typedef enum {
#if SYS_FS_AUTOMOUNT_ENABLE
        /* Wait for disk Mount */
        APP_MOUNT_WAIT = 0,
#else
        /* Mount the disk */
        SST26_MOUNT_DISK = 0,
#endif
        SST26_FORMAT_DISK,

        SST26_IDLE,

        SST26_ERROR,

        SST26_STOP

    } SST26_STATES;

    // *****************************************************************************

    /* Application Data

      Summary:
        Holds application data

      Description:
        This structure holds the application's data.

      Remarks:
        Application strings and buffers are be defined outside this structure.
     */

    typedef struct {
        /* SYS_FS File handle */
        SYS_FS_HANDLE fileHandle;

        /* Application's current state */
        SST26_STATES state;

        CACHE_ALIGN unsigned short checksum[1];

        SYS_FS_FSTAT fileStatus;

        long fileSize;

        bool diskMounted;

        bool diskFormatRequired;

    } SST26_DATA;

    void SST26_Initialize(void);
    void SST26_MountDisk(void);
    bool SST26_ReadDir(void);
    bool SST26_WriteFile(const char* fileChar, WAVEFORM_DATA *waveformData);
    bool SST26_ReadFile(const char* fileChar, WAVEFORM_DATA *waveformData);
    bool SST26_RemoveFile(const char* fileChar);
    bool SST26_FormatDisk();
    unsigned short calculate_checksum(WAVEFORM_DATA *checkdata);
    bool SST26_WriteSLD(const char* fileChar, uint16_t sld_threshold);
    bool SST26_ReadSLD(const char* fileChar, uint16_t* out_threshold);
#ifdef	__cplusplus
}
#endif

#endif	/* FILESYS_H */

