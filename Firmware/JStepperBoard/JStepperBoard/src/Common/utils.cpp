#include "utils.h"

void print_task_memory(void) {
    size_t        freeHeap = xPortGetFreeHeapSize();
    unsigned long stackRem = uxTaskGetStackHighWaterMark(NULL);

    printf("Free heap: %u bytes, Task remaining stack: %u bytes\n", (unsigned int)freeHeap,
           (unsigned int)(stackRem * 4));
}
