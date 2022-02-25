#include "data_processor.h"
#include "sll.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "stm32g4xx.h"
#include "stm32g4xx_hal.h"
#include <time.h>
#include "hand.h"

// TODO: ADD TIME DELAY
int DataProcessor_CheckThreshold(uint16_t half_buffer[4096], int startIndex, int stopIndex){
//    printf("** Checking threshold..\r\n");
    int allFingers[] = {0, 1, 2, 3, 4};

    while (startIndex < stopIndex){
        if(half_buffer[startIndex] >= UPPER_THRESHOLD && GetFingerPositions()[0] > MIN_FINGER_POSITION){ // TODO: Update to check more than thumb
            Hand_Move(GetFingerPositions()[0] + 1, allFingers);
            printf("%i passed upper threshold of %i. Clenching... \r\n", half_buffer[startIndex], UPPER_THRESHOLD);
        } else if(half_buffer[startIndex] < LOWER_THRESHOLD && GetFingerPositions()[0] < MAX_FINGER_POSITION){
            Hand_Move(GetFingerPositions()[0] - 1, allFingers);
            printf("%i passed lower threshold of %i. Releasing... \r\n", half_buffer[startIndex], LOWER_THRESHOLD);
        }
        HAL_Delay(5);
        startIndex++;
    }
}