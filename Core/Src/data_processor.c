#include "data_processor.h"
#include <stm32g4xx_hal.h>
#include <stdio.h>
#include "hand.h"

int DataProcessor_CheckThreshold(uint16_t half_buffer[4096], int startIndex, int stopIndex){
    while (startIndex < stopIndex){ // Iterate through half of buffer
        int hasMoved = 0; // Used to delay only after a movement
        if(half_buffer[startIndex] >= UPPER_THRESHOLD){ // Clench
            for(int i = 0; i<5; i++){
                if(FingerPositions[i] > CLENCHED_FINGER_POSITION){ // If this finger is not clenched, clench it
                    hasMoved = 1;
                    Hand_Move(FingerPositions[0] - 1, (int[1]) {i}, 1);
//                    printf("%i passed upper threshold of %i. Clenching... \r\n", half_buffer[startIndex], UPPER_THRESHOLD);
                }
            }
            if(hasMoved == 1){ // Only delay if a finger has moved
                HAL_Delay(BASE_VELOCITY_DELAY);
            }
        } else if(half_buffer[startIndex] < LOWER_THRESHOLD){ // Release
            for(int i = 0; i<5; i++){
                if(FingerPositions[i] < RELEASED_FINGER_POSITION){ // If this finger is not released, release it
                    hasMoved = 1;
                    Hand_Move(FingerPositions[0] + 1, (int[1]) {i}, 1);
//                    printf("%i passed lower threshold of %i. Releasing... \r\n", half_buffer[startIndex], LOWER_THRESHOLD);
                }
            }
            if(hasMoved == 1){ // Only delay if a finger has moved
                HAL_Delay(BASE_VELOCITY_DELAY);
            }
        }
        startIndex++;
    }
}