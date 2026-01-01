/*
 * sd_card.h
 *
 * Created: 29/10/2025 14:29:25
 *  Author: Tan
 */

#ifndef SD_CARD_H_
#define SD_CARD_H_

extern "C" {
#include <asf.h>
#include <string.h>
}

/**
 * @brief Debug prints
 *
 */
#ifdef SD_CARD_DEBUG
#    include <stdio.h>
#    define sd_card_debug(...) printf(__VA_ARGS__)
#else
#    define sd_card_debug(...)
#endif

class SDCard {
  public:
    bool mount();
    void unmount();

    /**
     * @brief Write contents to file
     *
     * @param file_name
     * @param data
     * @return true
     * @return false
     */
    bool writeFile(const char *file_name, const char *data);

    /**
     * @brief Read a text file and return its contents as a dynamically allocated string.
     * @brief The caller must free the buffer.
     *
     * @param file_name  Path to the file (e.g., "0:test.txt")
     * @param out_buffer Pointer to char* that will be allocated
     * @return true on success, false on failure
     */
    bool readFile(const char *file_name, char **out_buffer);

    bool deleteFile(const char *file_name);

    bool wipe();

  private:
    FATFS fs;
    FIL   file;

    bool m_card_mounted = false;
};

#endif /* SD_CARD_H_ */