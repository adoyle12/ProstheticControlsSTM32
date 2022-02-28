//
// Created by parallels on 7/9/21.
//

#ifndef PROSTHETICCONTROLSSTM32_DATA_PROCESSOR_H
#define PROSTHETICCONTROLSSTM32_DATA_PROCESSOR_H

#include <stdint.h>

#define RELEASE 0
#define CLENCH 1
#define LOWER_THRESHOLD 186
#define UPPER_THRESHOLD 744

//int DataProcessor_Initialize();
//int DataProcessor_ReadData(uint16_t adc_buf_copy[4096], int startIndex, int stopIndex);
//int DataProcessor_ProcessData();
int DataProcessor_CheckThreshold(uint16_t half_buffer[4096], int startIndex, int stopIndex);
//int DataProcessor_CompleteAction(int action);

#endif //PROSTHETICCONTROLSSTM32_DATA_PROCESSOR_H
