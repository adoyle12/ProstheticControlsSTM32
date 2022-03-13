#include "data_processor.h"
#include <stm32g4xx_hal.h>
#include <stdio.h>
#include "hand.h"

int DataProcessor_CheckThreshold(uint16_t half_buffer[4096], int startIndex, int stopIndex){
    while (startIndex < stopIndex){ // Iterate through half of buffer
        DAC1->DHR12R1 = half_buffer[startIndex]; // Enable averaging output

        int hasMoved = 0; // Used to delay only after a movement
        if(half_buffer[startIndex] >= MAX_THRESHOLD){ // Clench
            for(int i = 0; i<5; i++){
                if(FingerPositions[i] > CLENCHED_FINGER_POSITION){ // If this finger is not clenched, clench it
                    hasMoved = 1;
                    Hand_Move(FingerPositions[0] - 1, (int[1]) {i}, 1);
//                    printf("%i passed upper threshold of %i. Clenching... \r\n", half_buffer[startIndex], UPPER_THRESHOLD);
                }
            }

            if(hasMoved == 1){ // Only delay if a finger has moved
                HAL_Delay(5);
            }
        } else if(half_buffer[startIndex] >= UPPER_THRESHOLD){ // Clench
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
                HAL_Delay(8); // TODO: Set to BASE_VELOCITY_DELAY
            }
        }
        startIndex++;
    }
}

int DataProcessor_AverageData(uint16_t half_buffer[4096], int startIndex, int stopIndex) {
    printf("Processing data...\r\n");
    float currentSum = 0;

    // Average first length - 40 values
    while (startIndex <= stopIndex - 40){
        for (int i = 0; i < 40; i++) {
            currentSum += half_buffer[startIndex+i];
        }
        half_buffer[startIndex] = currentSum / 40;
        startIndex++;
        currentSum = 0;
    }

    // Average last 40 values
    for(int i=0; i<40; i++){
        currentSum += half_buffer[startIndex+i];
    }
    currentSum /= 40;
    // Add value of average of last 40 elements to each of the last 40 elements
    while(startIndex <= stopIndex){
        half_buffer[startIndex] = currentSum;
    }

    printf("Done processing data...\r\n");

    DataProcessor_CheckThreshold(half_buffer[4096], startIndex, stopIndex);
}