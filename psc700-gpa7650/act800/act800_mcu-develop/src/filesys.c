/**
 * @file    filesys.c
 * @brief   SST26 file system management module.
 * @details This module provides file system operations on SST26 flash
 *          storage using FAT file system. It supports mounting, formatting,
 *          file read/write, directory listing, and file removal.
 *
 *          It also provides data integrity mechanisms such as checksum
 *          verification for waveform data and parameter storage.
 *
 *          Key features:
 *          - Disk mount and format management
 *          - File read/write for waveform data
 *          - Directory listing and file removal
 *          - Checksum verification for data integrity
 *          - Persistent storage for system parameters (e.g. SLD threshold)
 * @ingroup data_storage
 */
#include "filesys.h"

#define SST26_MOUNT_NAME          "/mnt/myDrive1"
#define SST26_MOUNT_MAIN          "/mnt"
#define SST26_DEVICE_NAME         "/dev/mtda1"
#define SST26_FS_TYPE             FAT

static SST26_DATA CACHE_ALIGN sst26Data;

static uint8_t CACHE_ALIGN sst26work[SYS_FS_FAT_MAX_SS];

static SYS_FS_FORMAT_PARAM sst26formatOpt;
/**
 * @brief   Initialize SST26 file system module.
 * @details Initializes file system state and prepares for disk mount.
 *          If auto-mount is enabled, registers file system event handler.
 */
void SST26_Initialize(void) {
    /* Initialize the app state to wait for media attach. */
#if SYS_FS_AUTOMOUNT_ENABLE
    appData.state = APP_MOUNT_WAIT;

    SYS_FS_EventHandlerSet(APP_SysFSEventHandler, (uintptr_t) NULL);
#else
    sst26Data.state = SST26_MOUNT_DISK;

#endif
}
SYS_FS_FSTAT sst26fileStat;
/**
 * @brief   Mount and initialize SST26 disk.
 * @details Attempts to mount the file system. If no valid file system
 *          is detected, formats the disk. After successful mount,
 *          initializes required system files (e.g. SLD flag).
 */
void SST26_MountDisk(void) {
    switch (sst26Data.state) {
        case SST26_MOUNT_DISK:
        {
            /* Mount the disk */
            if (SYS_FS_Mount(SST26_DEVICE_NAME, SST26_MOUNT_NAME, SST26_FS_TYPE, 0, NULL) != 0) {
                /* The disk could not be mounted. Try mounting again until
                 * the operation succeeds. */
                sst26Data.state = SST26_MOUNT_DISK;
            } else {
                /* Check If Mount was successful with no file system on media */
                if (SYS_FS_Error() == SYS_FS_ERROR_NO_FILESYSTEM) {
                    /* Format the disk. */
                    sst26Data.state = SST26_FORMAT_DISK;
                    printf("SST26 FORMAT DISK START \r\n");
                } else {
                    printf("SST26 DISK MOUNTED SUCCEEDED!!\r\n");
                     /* === Check and initialize sldflg === */
                    uint16_t flag = 0;
                    if (!SST26_ReadSLD("sldflg", &flag)) {
                        /* File not exist or read failed -> create one and write default 0 */
                        flag = 0;
                        SST26_WriteSLD("sldflg", flag);
                        sld_flag = flag;
                        printf("sldflg not found, create default 0\r\n");
                    } else {
                        /* File exists -> load value */
                        sld_flag = flag;
                        printf("sldflg loaded: %u\r\n", sld_flag);
                    }
                    sst26Data.state = SST26_IDLE;
                }
            }
            break;
        }
        case SST26_FORMAT_DISK:
        {
            sst26formatOpt.fmt = SYS_FS_FORMAT_FAT;
            sst26formatOpt.au_size = 0;

            if (SYS_FS_DriveFormat(SST26_MOUNT_NAME, &sst26formatOpt, (void *) sst26work, SYS_FS_FAT_MAX_SS) != SYS_FS_RES_SUCCESS) {
                /* Format of the disk failed. */
                sst26Data.state = SST26_ERROR;
                printf("SST26 FORMAT DISK FAILED!!\r\n");

            } else {
                /* Format succeeded. Mount disk. */
                sst26Data.state = SST26_MOUNT_DISK;
                printf("SST26 FORMAT DISK SUCCEEDED!!\r\n");
            }
            break;
        }
        case SST26_IDLE:
        {
            sst26Data.state = SST26_STOP;
            break;
        }
        case SST26_ERROR:
        {
            /* The application comes here when the demo has failed.*/
            printf("SST26 ERROR\r\n");
            break;
        }
        case SST26_STOP:
        {

            break;
        }
    }
}
/**
 * @brief   Read and list directory contents.
 * @details Opens the root directory and prints all file names
 *          and sizes.
 *
 * @return true if directory read succeeds, otherwise false.
 */
bool SST26_ReadDir(void) {
    sst26Data.fileHandle = SYS_FS_DirOpen(SST26_MOUNT_NAME);
    if (sst26Data.fileHandle == SYS_FS_HANDLE_INVALID) {
        printf("SST26 READ DIR ERROR\r\n");
        return false;
    } else {
        /* Directory open was successful. Read directory entries */
        while (SYS_FS_DirRead(sst26Data.fileHandle, &sst26fileStat) == SYS_FS_RES_SUCCESS) {

            if (sst26fileStat.fname[0] == '\0') {
                break;
            } else {
                printf("File Name: %s\r\n", sst26fileStat.fname);
                printf("File Size: %lu Bytes\r\n", (unsigned long) sst26fileStat.fsize);
            }
        }
        SYS_FS_DirClose(sst26Data.fileHandle);
        return true;
    }
}
/**
 * @brief   Write waveform data to file.
 * @details Writes waveform structure and associated data arrays
 *          (trigger, x, y) to file, followed by checksum for
 *          data integrity verification.
 *
 * @param[in] fileChar      File name identifier.
 * @param[in] waveformData  Pointer to waveform data structure.
 *
 * @return true if write succeeds, otherwise false.
 */
bool SST26_WriteFile(const char* fileChar, WAVEFORM_DATA *waveformData) {
    //// Open file 
    bool ret = true;
    char fileName[150];
    sst26Data.checksum[0] = 0;
    snprintf(fileName, sizeof (fileName), "%s/%s.txt", SST26_MOUNT_NAME, fileChar);
    sst26Data.fileHandle = SYS_FS_FileOpen(fileName, (SYS_FS_FILE_OPEN_WRITE_PLUS));
    size_t bytesWritten;
    if (sst26Data.fileHandle == SYS_FS_HANDLE_INVALID) {
        /* File open unsuccessful */
        printf("SST26 OPEN FILE FAILED\r\n");
        ret = false;
        return ret;
    } else {
        WDT_Disable();
        sst26Data.checksum[0] = calculate_checksum(waveformData);
        ////Write file
        bytesWritten = SYS_FS_FileWrite(sst26Data.fileHandle, waveformData, sizeof (WAVEFORM_DATA) - (sizeof (uint32_t *) + 2 * sizeof (unsigned short *)));
        if (bytesWritten != sizeof (WAVEFORM_DATA) - (sizeof (uint32_t *) + 2 * sizeof (unsigned short *))) {
            printf("WAVEFORM STRUCT DATA SIZE ERROR\r\n");
            ret = false;
        }

        bytesWritten = SYS_FS_FileWrite(sst26Data.fileHandle, waveformData->ptr_trigger, waveformData->trigger_length * sizeof (uint32_t));

        if (bytesWritten != waveformData->trigger_length * sizeof (uint32_t)) {
            ret = false;
        }

        bytesWritten = SYS_FS_FileWrite(sst26Data.fileHandle, waveformData->ptr_x, waveformData->length * sizeof (unsigned short));

        if (bytesWritten != waveformData->length * sizeof (unsigned short)) {
            ret = false;
        }

        bytesWritten = SYS_FS_FileWrite(sst26Data.fileHandle, waveformData->ptr_y, waveformData->length * sizeof (unsigned short));

        if (bytesWritten != waveformData->length * sizeof (unsigned short)) {
            ret = false;
        }

        bytesWritten = SYS_FS_FileWrite(sst26Data.fileHandle, (void *) sst26Data.checksum, sizeof (unsigned short));

        if (SYS_FS_FileClose(sst26Data.fileHandle) != 0) {
            printf("SST26 CLOSE FILE FAILED\r\n");
            ret = false;
        }
        WDT_Enable();
        return ret;
    }
}
/**
 * @brief   Read waveform data from file.
 * @details Reads waveform structure and associated data arrays
 *          from file, verifies file size and checksum to ensure
 *          data integrity.
 *
 * @param[in]  fileChar          File name identifier.
 * @param[out] waveformReadData  Pointer to waveform data structure.
 *
 * @return true if read and verification succeed, otherwise false.
 */
bool SST26_ReadFile(const char* fileChar, WAVEFORM_DATA *waveformReadData) {
    bool ret = true;
    size_t bytesRead;
    sst26Data.checksum[0] = 0;
    unsigned short checksum = 0;
    char fileName[150];
    snprintf(fileName, sizeof (fileName), "%s/%s.txt", SST26_MOUNT_NAME, fileChar);
    sst26Data.fileHandle = SYS_FS_FileOpen(fileName, (SYS_FS_FILE_OPEN_READ));
    if (sst26Data.fileHandle == SYS_FS_HANDLE_INVALID) {
        /* File open unsuccessful */
        printf("SST26 OPEN FILE FAILED\r\n");
        ret = false;
        return ret;
    } else {
        if (SYS_FS_FileStat(fileName, &sst26Data.fileStatus) == SYS_FS_RES_FAILURE) {
            /* Reading file status was a failure */
            printf("SST26 reading file status was a failure\r\n");
            ret = false;
        } else {
            /* Read file size */
            sst26Data.fileSize = SYS_FS_FileSize(sst26Data.fileHandle);
            if (sst26Data.fileSize == -1) {
                /* Reading file size was a failure */
                printf("SST26 reading file size was a failure\r\n");
                ret = false;
            } else {
                if (sst26Data.fileSize == sst26Data.fileStatus.fsize) {
                    if (SYS_FS_FileSeek(sst26Data.fileHandle, sst26Data.fileSize, SYS_FS_SEEK_SET) == -1) {
                        /* File seek caused an error */
                        printf("SST26 file seek caused an error\r\n");
                        ret = false;
                    } else {
                        /* Check for End of file */
                        if (SYS_FS_FileEOF(sst26Data.fileHandle) == false) {
                            /* Either, EOF is not reached or there was an error
                               In any case, for the application, its an error condition */
                            printf("SST26 EOF is not reached or there was an error\r\n");
                            ret = false;
                        } else {
                            if (SYS_FS_FileSeek(sst26Data.fileHandle, 0, SYS_FS_SEEK_SET) == -1) {
                                /* File seek caused an error */
                                printf("SST26 file seek caused an error\r\n");
                                ret = false;
                            } else {

                                bytesRead = SYS_FS_FileRead(sst26Data.fileHandle, waveformReadData, sizeof (WAVEFORM_DATA) - (sizeof (uint32_t *) + 2 * sizeof (short *)));
                                if (bytesRead != sizeof (WAVEFORM_DATA) - (sizeof (uint32_t *) + 2 * sizeof (short *))) {
                                    ret = false;
                                }

                                bytesRead = SYS_FS_FileRead(sst26Data.fileHandle, waveformReadData->ptr_trigger, waveformReadData->trigger_length * sizeof (uint32_t *));
                                if (bytesRead != waveformReadData->trigger_length * sizeof (uint32_t *)) {
                                    ret = false;
                                }

                                bytesRead = SYS_FS_FileRead(sst26Data.fileHandle, waveformReadData->ptr_x, waveformReadData->length * sizeof (unsigned short));
                                if (bytesRead != waveformReadData->length * sizeof (unsigned short)) {
                                    ret = false;
                                }

                                bytesRead = SYS_FS_FileRead(sst26Data.fileHandle, waveformReadData->ptr_y, waveformReadData->length * sizeof (unsigned short));
                                if (bytesRead != waveformReadData->length * sizeof (unsigned short)) {
                                    ret = false;
                                }

                                bytesRead = SYS_FS_FileRead(sst26Data.fileHandle, (void *) sst26Data.checksum, sizeof (unsigned short));

                                checksum = calculate_checksum(waveformReadData);

                                if (sst26Data.checksum[0] != checksum) {
                                    ret = false;
                                    printf("Checksum ERROR %d:%d\r\n", sst26Data.checksum[0], checksum);
                                }

                                if (SYS_FS_FileClose(sst26Data.fileHandle) != 0) {
                                    printf("SST26 CLOSE FILE FAILED\r\n");
                                    ret = false;
                                }
                            }
                        }
                    }
                } else {
                    printf("ERROR!! Size not match\r\n");
                    ret = false;
                }
            }
        }
        return ret;
    }

}
/**
 * @brief   Remove specified file.
 *
 * @param[in] fileChar  File name identifier.
 *
 * @return true if removal succeeds, otherwise false.
 */
bool SST26_RemoveFile(const char* fileChar) {
    bool ret = true;
    char fileName[150];
    snprintf(fileName, sizeof (fileName), "%s/%s.txt", SST26_MOUNT_NAME, fileChar);

    if (SYS_FS_FileDirectoryRemove(fileName) == SYS_FS_RES_FAILURE) {
        // Directory remove operation failed
        printf("SST26 REMOVE %s FAILED\r\n", fileChar);
        ret = false;
    }
    return ret;
}
/**
 * @brief   Format SST26 disk.
 * @details Formats the mounted storage using FAT file system.
 *
 * @return true if format succeeds, otherwise false.
 */
bool SST26_FormatDisk() {
    bool ret = true;
    sst26formatOpt.fmt = SYS_FS_FORMAT_FAT;
    sst26formatOpt.au_size = 0;

    if (SYS_FS_DriveFormat(SST26_MOUNT_NAME, &sst26formatOpt, (void *) sst26work, SYS_FS_FAT_MAX_SS) != SYS_FS_RES_SUCCESS) {
        /* Format of the disk failed. */
        printf("SST26 FORMAT DISK FAILED!!\r\n");
        ret = false;
        return ret;
    } else {
        /* Format succeeded. Open a file. */
        //printf("SST26 FORMAT DISK SUCCEEDED!!\r\n");
        return ret;

    }
}
/**
 * @brief   Calculate checksum for waveform data.
 * @details Computes 16-bit checksum based on waveform parameters
 *          and associated data arrays to ensure data integrity.
 *
 * @param[in] checkdata  Pointer to waveform data.
 *
 * @return Calculated checksum value.
 */
// Caculate all 32-bit data
#define CHECKSUM_32BIT_TO_16BIT(I)  ((uint16_t)(((I)>>16)&0xFFFF)+(uint16_t)((I)&0xFFFF))

unsigned short calculate_checksum(WAVEFORM_DATA *checkdata) {
    unsigned short checksum = 0;

    checksum += CHECKSUM_32BIT_TO_16BIT(checkdata->length);
    checksum += CHECKSUM_32BIT_TO_16BIT(checkdata->scancnt);
    checksum += CHECKSUM_32BIT_TO_16BIT(checkdata->trigger_length);
    checksum += CHECKSUM_32BIT_TO_16BIT(checkdata->trigger_start);
    checksum += CHECKSUM_32BIT_TO_16BIT(checkdata->trigger_period);
    checksum += CHECKSUM_32BIT_TO_16BIT(checkdata->x_start_volt);
    checksum += CHECKSUM_32BIT_TO_16BIT(checkdata->x_stop_volt);
    checksum += CHECKSUM_32BIT_TO_16BIT(checkdata->y_start_volt);
    checksum += CHECKSUM_32BIT_TO_16BIT(checkdata->y_stop_volt);
    checksum += CHECKSUM_32BIT_TO_16BIT(checkdata->lowpass_filter_freq);

    if (checkdata->ptr_trigger != NULL) {
        for (int i = 0; i < checkdata->trigger_length; i++) {
            checksum += (unsigned short) checkdata->ptr_trigger[i] & 0xFFFF;
        }
    }

    if (checkdata->ptr_x != NULL) {
        for (int i = 0; i < checkdata->length; i++) {
            checksum += (unsigned short) checkdata->ptr_x[i] & 0xFFFF;
        }
    }

    if (checkdata->ptr_y != NULL) {
        for (int i = 0; i < checkdata->length; i++) {
            checksum += (unsigned short) checkdata->ptr_y[i] & 0xFFFF;
        }
    }

    checksum &= 0xFFFF;

    return checksum;
}

// ======================== SLD Threshold R/W ========================

// ===================================================================

static inline uint16_t SLD_CalcChecksum(uint16_t v)
{
    return (uint16_t)(v ^ 0xAAAA);
}

// ===================================================================
// Write SLD threshold value to file
// ===================================================================
/**
 * @brief   Write SLD threshold value to file.
 * @details Stores SLD threshold and corresponding checksum
 *          into file for persistent storage.
 *
 * @param[in] fileChar       File name identifier.
 * @param[in] sld_threshold  Threshold value.
 *
 * @return true if write succeeds, otherwise false.
 */
bool SST26_WriteSLD(const char* fileChar, uint16_t sld_threshold){
    bool ret = true;
    char fileName[150];
    size_t bytesWritten;
    uint16_t checksum = SLD_CalcChecksum(sld_threshold);
    // Compose the full file path: <mount>/<name>.txt
    snprintf(fileName, sizeof(fileName), "%s/%s.txt", SST26_MOUNT_NAME, fileChar);
    // Open file (create if not exist, overwrite if exist)
    sst26Data.fileHandle = SYS_FS_FileOpen(fileName, SYS_FS_FILE_OPEN_WRITE_PLUS);
    if (sst26Data.fileHandle == SYS_FS_HANDLE_INVALID) {
        printf("SST26 OPEN FILE FAILED\r\n");
        return false;
    }
    // Write threshold value
    bytesWritten = SYS_FS_FileWrite(sst26Data.fileHandle, &sld_threshold, sizeof(uint16_t));
    if (bytesWritten != sizeof(uint16_t)) {
        printf("WRITE SLD VALUE FAILED\r\n");
        ret = false;
    }
    // Write checksum
    if (ret) {
        bytesWritten = SYS_FS_FileWrite(sst26Data.fileHandle, &checksum, sizeof(uint16_t));
        if (bytesWritten != sizeof(uint16_t)) {
            printf("WRITE CHECKSUM FAILED\r\n");
            ret = false;
        }
    }

    // Close file
    if (SYS_FS_FileClose(sst26Data.fileHandle) != 0) {
        printf("SST26 CLOSE FILE FAILED\r\n");
        ret = false;
    }

    return ret;
}

// ===================================================================
// Read SLD threshold value from file (with checksum verification)
// ===================================================================
/**
 * @brief   Read SLD threshold value from file.
 * @details Reads stored threshold and verifies checksum
 *          before returning the value.
 *
 * @param[in]  fileChar       File name identifier.
 * @param[out] out_threshold  Pointer to output threshold value.
 *
 * @return true if read and verification succeed, otherwise false.
 */
bool SST26_ReadSLD(const char* fileChar, uint16_t* out_threshold){
    if (!out_threshold) return false;
    bool ret = true;
    char fileName[150];
    size_t bytesRead;
    uint16_t read_threshold = 0;
    uint16_t read_checksum  = 0;
    uint16_t calc_checksum  = 0;
    // Compose the full file path: <mount>/<name>.txt
    snprintf(fileName, sizeof(fileName), "%s/%s.txt", SST26_MOUNT_NAME, fileChar);
    // Open file
    sst26Data.fileHandle = SYS_FS_FileOpen(fileName, SYS_FS_FILE_OPEN_READ);
    if (sst26Data.fileHandle == SYS_FS_HANDLE_INVALID) {
        printf("SST26 OPEN FILE FAILED\r\n");
        return false;
    }
    // Read threshold value
    bytesRead = SYS_FS_FileRead(sst26Data.fileHandle, &read_threshold, sizeof(uint16_t));
    if (bytesRead != sizeof(uint16_t)) {
        printf("READ SLD VALUE FAILED\r\n");
        ret = false;
    }
    // Read checksum
    if (ret) {
        bytesRead = SYS_FS_FileRead(sst26Data.fileHandle, &read_checksum, sizeof(uint16_t));
        if (bytesRead != sizeof(uint16_t)) {
            printf("READ CHECKSUM FAILED\r\n");
            ret = false;
        }
    }
    // Verify checksum
    if (ret) {
        calc_checksum = SLD_CalcChecksum(read_threshold);
        if (read_checksum != calc_checksum) {
            printf("Checksum ERROR %u : %u\r\n", read_checksum, calc_checksum);
            ret = false;
        }
    }
    // Close file
    if (SYS_FS_FileClose(sst26Data.fileHandle) != 0) {
        printf("SST26 CLOSE FILE FAILED\r\n");
        ret = false;
    }
    // Output threshold only if read/verify succeeded
    if (ret) {
        *out_threshold = read_threshold;
    }
    return ret;
}

