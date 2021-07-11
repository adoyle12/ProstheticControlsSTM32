//
// Created by parallels on 7/9/21.
//

#ifndef PROSTHETICCONTROLSSTM32_DATA_PROCESSOR_H
#define PROSTHETICCONTROLSSTM32_DATA_PROCESSOR_H

#define RELEASE 0
#define CLENCH 1
#define THRESHOLD 0.15

int DataProcessor_Initialize();
int DataProcessor_ReadData();
int DataProcessor_ProcessData();
int DataProcessor_CheckThreshold();
int DataProcessor_CompleteAction(int action);

#endif PROSTHETICCONTROLSSTM32_DATA_PROCESSOR_H
