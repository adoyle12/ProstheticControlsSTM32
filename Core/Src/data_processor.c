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
    //int allFingers[] = {0, 1, 2, 3, 4};

    while (startIndex < stopIndex){ // Iterate through half of buffer
        if(half_buffer[startIndex] >= UPPER_THRESHOLD){ // Clench
            for(int i = 0; i<5; i++){
                if(FingerPositions[i] > CLENCHED_FINGER_POSITION){ // If this finger is not clenched, clench it
                    Hand_Move(FingerPositions[0] - 5, (int[1]) {i}, 1);
                    printf("%i passed upper threshold of %i. Clenching... \r\n", half_buffer[startIndex], UPPER_THRESHOLD);
                }
            }
        } else if(half_buffer[startIndex] < LOWER_THRESHOLD){ // Release
            for(int i = 0; i<5; i++){
                if(FingerPositions[i] < RELEASED_FINGER_POSITION){ // If this finger is not released, release it
                    Hand_Move(FingerPositions[0] + 5, (int[1]) {i}, 1);
                    printf("%i passed lower threshold of %i. Releasing... \r\n", half_buffer[startIndex], LOWER_THRESHOLD);
                }
            }
        }
        startIndex++;
    }
}