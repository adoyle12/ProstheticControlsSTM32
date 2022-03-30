#ifndef PROSTHETICCONTROLSSTM32_DATA_PROCESSOR_H
#define PROSTHETICCONTROLSSTM32_DATA_PROCESSOR_H

#include <stdint.h>

#define RELEASE 0
#define CLENCH 1
#define LOWER_THRESHOLD 186
#define UPPER_THRESHOLD 744
#define MAX_THRESHOLD 3800 // TODO: Update this value
#define BASE_VELOCITY_DELAY 35 // TODO: Set this to 15 again

int DataProcessor_CheckThreshold(uint16_t half_buffer[4096], int startIndex, int stopIndex);
#endif //PROSTHETICCONTROLSSTM32_DATA_PROCESSOR_H
