#include "data_processor.h"
#include "sll.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "stm32g4xx.h"
#include "stm32g4xx_hal.h"
#include <time.h>
#include "hand.h"

int DataProcessor_CheckThreshold(uint16_t half_buffer[4096], int startIndex, int stopIndex){
    int allFingers[] = {0, 1, 2, 3, 4};

    while (startIndex < stopIndex){
        if(half_buffer[startIndex] >= UPPER_THRESHOLD && FingerPositions[0] > CLENCHED_FINGER_POSITION){ // TODO: Update to check more than thumb
            Hand_Move(FingerPositions[0] - 5, allFingers, 5);
            printf("%i passed upper threshold of %i. Clenching... \r\n", half_buffer[startIndex], UPPER_THRESHOLD);
        } else if(half_buffer[startIndex] < LOWER_THRESHOLD && FingerPositions[0] < RELEASED_FINGER_POSITION){
            Hand_Move(FingerPositions[0] + 5, allFingers, 5);
            printf("%i passed lower threshold of %i. Releasing... \r\n", half_buffer[startIndex], LOWER_THRESHOLD);
        }
        startIndex++;
    }
}