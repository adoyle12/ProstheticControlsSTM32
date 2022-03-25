#ifndef PROSTHETICCONTROLSSTM32_DATA_PROCESSOR_H
#define PROSTHETICCONTROLSSTM32_DATA_PROCESSOR_H

#include <stdint.h>

#define RELEASE 0
#define CLENCH 1
#define LOWER_THRESHOLD 620
#define UPPER_THRESHOLD 1241
#define BASE_VELOCITY_DELAY 15

int DataProcessor_CheckThreshold(uint16_t half_buffer[4096], int startIndex, int stopIndex);

#endif //PROSTHETICCONTROLSSTM32_DATA_PROCESSOR_H
