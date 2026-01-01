#include "sd_card.h"

bool SDCard::mount() {
    Ctrl_status status;

    sd_card_debug("Waiting for card...");
    do {
        status = sd_mmc_test_unit_ready(0);
        if (CTRL_FAIL == status) {
            sd_card_debug("[FAIL] Please unplug and re-plug the card.\r\n");
            return false;
        }
    } while (CTRL_GOOD != status);

    sd_card_debug("[OK]\r\n");

    sd_card_debug("Mounting card...");
    memset(&fs, 0, sizeof(FATFS));
    FRESULT res = f_mount(LUN_ID_SD_MMC_0_MEM, &fs);

    if (res != FR_OK) {
        sd_card_debug("[FAIL] f_mount res=%d\r\n", res);
        return false;
    }
    sd_card_debug("[OK]\r\n");

    DWORD  free_clusters;
    FATFS *fsp;

    sd_card_debug("Checking SD card type...");
    res = f_getfree("0:", &free_clusters, &fsp);

    if (res != FR_OK) {
        sd_card_debug("[FAIL] Not valid FAT filesystem (res=%d)\r\n", res);
        return false;
    }

    if (fsp->fs_type != FS_FAT32) {
        sd_card_debug("[FAIL] Not FAT32 (detected type %d)\r\n", fsp->fs_type);
        return false;
    }

    sd_card_debug("[OK]\r\n");

    m_card_mounted = true;
    return true;
}

/* ---------------------------------------------------------------------------------- */
void SDCard::unmount() {
    sd_card_debug("Unmounting card...");
    if (!m_card_mounted) {
        sd_card_debug("[FAILED] Card not mounted\r\n");
        return;
    }
    f_mount(LUN_ID_SD_MMC_0_MEM, NULL);
    sd_card_debug("[OK]\r\n");

    m_card_mounted = false;
}

/* ---------------------------------------------------------------------------------- */
bool SDCard::writeFile(const char *file_name, const char *data) {
    sd_card_debug("Create/open file (f_open)...");

    // Sanity check
    if (!m_card_mounted) {
        sd_card_debug("[FAIL] Card not mounted\n");
        return false;
    }

    FIL     file_object;
    FRESULT res;
    UINT    bytes_written;

    // Dynamically allocate space for "0:" + file name
    size_t path_len = strlen(file_name) + 3; // 2 for "0:" + 1 for null
    char  *path     = (char *)malloc(path_len);
    if (!path) {
        sd_card_debug("[FAIL] malloc path\n");
        return false;
    }
    snprintf(path, path_len, "0:%s", file_name);

    res = f_open(&file_object, path, FA_WRITE | FA_CREATE_ALWAYS | FA_READ);
    free(path);

    if (res != FR_OK) {
        sd_card_debug("[FAIL] f_open=%d\r\n", res);
        return false;
    }

    sd_card_debug("[OK]\r\n");
    sd_card_debug("Write to file (f_write)...");

    res = f_write(&file_object, data, strlen(data), &bytes_written);

    if (res != FR_OK || bytes_written != strlen(data)) {
        sd_card_debug("[FAIL] f_write res=%d, bw=%u\r\n", res, bytes_written);
        f_close(&file_object);
        return false;
    }

    sd_card_debug("[OK]\r\n");

    sd_card_debug("Sync file...");
    res = f_sync(&file_object);

    if (res != FR_OK) { sd_card_debug("[FAIL] f_sync res=%d\r\n", res); }

    f_close(&file_object);
    sd_card_debug("[OK]\r\n");
    return true;
}

/* ---------------------------------------------------------------------------------- */
bool SDCard::readFile(const char *file_name, char **out_buffer) {
    sd_card_debug("Reading file (%s)...", file_name);

    // Sanity check
    if (!m_card_mounted) {
        sd_card_debug("[FAIL] Card not mounted\n");
        return false;
    }

    FIL     file;
    FRESULT res;
    UINT    bytes_read;

    *out_buffer = NULL;

    // Dynamically allocate space for "0:" + file name
    size_t path_len = strlen(file_name) + 3; // 2 for "0:" + 1 for null
    char  *path     = (char *)malloc(path_len);
    if (!path) {
        sd_card_debug("[FAIL] malloc path\n");
        return false;
    }
    snprintf(path, path_len, "0:%s", file_name);

    // Open file
    res = f_open(&file, path, FA_READ);
    free(path);
    if (res != FR_OK) {
        sd_card_debug("[FAIL] f_open=%d\n", res);
        return false;
    }

    // Get file size
    DWORD size = f_size(&file);
    if (size == 0) {
        sd_card_debug("[WARN] File is empty\n");
        f_close(&file);
        return false;
    }

    // Allocate buffer (+1 for null terminator)
    char *buffer = (char *)malloc(size + 1);
    if (!buffer) {
        sd_card_debug("[FAIL] malloc\n");
        f_close(&file);
        return false;
    }

    // Read entire file
    res = f_read(&file, buffer, size, &bytes_read);
    if (res != FR_OK || bytes_read != size) {
        sd_card_debug("[FAIL] f_read\n");
        free(buffer);
        f_close(&file);
        return false;
    }

    sd_card_debug("[OK]\r\n");
    buffer[size] = '\0'; // null terminate

    f_close(&file);

    // Optional: replace \r\n or \r with \n
    // for (DWORD i = 0; i < size; i++) {
    //   if (buffer[i] == '\r') buffer[i] = '\n';
    // }

    *out_buffer = buffer;
    return true;
}

/* ---------------------------------------------------------------------------------- */
bool SDCard::deleteFile(const char *file_name) {
    sd_card_debug("Delete file (f_unlink)...");

    // Sanity check
    if (!m_card_mounted) {
        sd_card_debug("[FAIL] Card not mounted\n");
        return false;
    }

    // Build "0:filename"
    size_t path_len = strlen(file_name) + 3; // "0:" + null
    char  *path     = (char *)malloc(path_len);
    if (!path) {
        sd_card_debug("[FAIL] malloc path\n");
        return false;
    }
    snprintf(path, path_len, "0:%s", file_name);

    // Delete the file
    FRESULT res = f_unlink(path);
    free(path);

    if (res != FR_OK) {
        sd_card_debug("[FAIL] f_unlink res=%d\r\n", res);
        return false;
    }

    sd_card_debug("[OK]\r\n");
    return true;
}

/* ---------------------------------------------------------------------------------- */
bool SDCard::wipe() {
    if (!m_card_mounted) {
        sd_card_debug("[FAIL] Card not mounted\n");
        return false;
    }

    DIR     dir;
    FILINFO fno;
    FRESULT res;

    res = f_opendir(&dir, "0:"); // open root directory
    if (res != FR_OK) {
        sd_card_debug("[FAIL] f_opendir=%d\n", res);
        return false;
    }

    while (true) {
        res = f_readdir(&dir, &fno);
        if (res != FR_OK || fno.fname[0] == 0) break; // end of dir or error

        // Skip "." and ".."
        if (strcmp(fno.fname, ".") == 0 || strcmp(fno.fname, "..") == 0) continue;

        // Build full path
        char path[64];
        snprintf(path, sizeof(path), "0:%s", fno.fname);

        // Delete file or directory
        res = f_unlink(path);
        if (res != FR_OK) {
            sd_card_debug("[WARN] Could not delete %s (err=%d)\n", path, res);
        } else {
            sd_card_debug("[OK] Deleted %s\n", path);
        }
    }

    return true;
}

/* ---------------------------------------------------------------------------------- */