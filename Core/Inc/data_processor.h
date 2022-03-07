#ifndef PROSTHETICCONTROLSSTM32_DATA_PROCESSOR_H
#define PROSTHETICCONTROLSSTM32_DATA_PROCESSOR_H

#include <stdint.h>

#define RELEASE 0
#define CLENCH 1
#define LOWER_THRESHOLD 186
#define UPPER_THRESHOLD 744
#define BASE_VELOCITY_DELAY 15

int DataProcessor_CheckThreshold(uint16_t half_buffer[4096], int startIndex, int stopIndex);
void delay(float number_of_seconds);


#endif //PROSTHETICCONTROLSSTM32_DATA_PROCESSOR_H
