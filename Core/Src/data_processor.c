#include "data_processor.h"
#include <stm32g4xx_hal.h>
#include <stdio.h>
#include "hand.h"
#include "SERVO.h"
#include "config.h"

int DataProcessor_CheckThreshold(uint16_t* half_buffer, int startIndex, int stopIndex){
    int MaxHandPulse = CalculateMaxServoPulse();

    while (startIndex < stopIndex){ // Iterate through half of buffer
        int hasMoved = 0; // Used to delay only after a movement
        if(half_buffer[startIndex] >= maxThreshold){ // Clench
            if(FingerPositions[2] < MaxHandPulse){ // Finger that clenches the farthest
                Hand_Move(FingerPositions[2] + 1, (int[5]) {0, 1, 2, 3, 4}, 5);
                HAL_Delay(5);
//                    /printf("%i passed max threshold of %f. Clenching... \r\n", half_buffer[startIndex], maxThreshold);
            }
        } else if(half_buffer[startIndex] >= middleThreshold){ // Clench
            if(FingerPositions[2] < MaxHandPulse){ // Finger that clenches the farthest
                Hand_Move(FingerPositions[2] + 1, (int[5]) {0, 1, 2, 3, 4}, 5);
                HAL_Delay(BASE_VELOCITY_DELAY);
//                    printf("%i passed upper threshold of %f. Clenching... \r\n", half_buffer[startIndex], middleThreshold);
            }
        } else if(half_buffer[startIndex] < minThreshold){ // Release
            if(FingerPositions[0] > SERVO_Get_MinPulse(2)){ // If this finger is not released, release it
                hasMoved = 1;
                Hand_Move(FingerPositions[0] - 1, (int[5]) {0, 1, 2, 3, 4}, 5);
//                    printf("%i passed lower threshold of %f. Releasing... \r\n", half_buffer[startIndex], minThreshold);
            }
            HAL_Delay(8); // TODO: Set to BASE_VELOCITY_DELAY
        }
    }
    startIndex++;
}