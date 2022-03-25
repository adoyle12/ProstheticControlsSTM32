#include "data_processor.h"
#include <stm32g4xx_hal.h>
#include <stdio.h>
#include "hand.h"
#include <SERVO.h>

int DataProcessor_CheckThreshold(uint16_t half_buffer[4096], int startIndex, int stopIndex){
    while (startIndex < stopIndex){ // Iterate through half of buffer
        //int hasMoved = 0; // Used to delay only after a movement
        if(half_buffer[startIndex] >= UPPER_THRESHOLD)
        { // Clench
            uint16_t SERVO_0_MAX_PULSE = SERVO_Get_MaxPulse(0);
            uint16_t SERVO_1_MAX_PULSE = SERVO_Get_MaxPulse(1);
            uint16_t SERVO_2_MAX_PULSE = SERVO_Get_MaxPulse(2);

            SERVO_RawMove(0, SERVO_0_MAX_PULSE);
            SERVO_RawMove(1, SERVO_1_MAX_PULSE);
            SERVO_RawMove(2, SERVO_2_MAX_PULSE);



            /*
            for(int i = 0; i<5; i++)
            {
                if(FingerPositions[i] > CLENCHED_FINGER_POSITION){ // If this finger is not clenched, clench it
                    hasMoved = 1;
                    Hand_Move(FingerPositions[0] - 1, (int[1]) {i}, 1);
//                    printf("%i passed upper threshold of %i. Clenching... \r\n", half_buffer[startIndex], UPPER_THRESHOLD);
                }
            }
            if(hasMoved == 1)
            { // Only delay if a finger has moved
                HAL_Delay(BASE_VELOCITY_DELAY);
            }
             */
        }
        else if(half_buffer[startIndex] < LOWER_THRESHOLD)
        { // Release
            uint16_t SERVO_0_MIN_PULSE = SERVO_Get_MinPulse(0);
            uint16_t SERVO_1_MIN_PULSE = SERVO_Get_MinPulse(1);
            uint16_t SERVO_2_MIN_PULSE = SERVO_Get_MinPulse(2);


            SERVO_RawMove(0, SERVO_0_MIN_PULSE);
            SERVO_RawMove(1, SERVO_1_MIN_PULSE);
            SERVO_RawMove(2, SERVO_2_MIN_PULSE);

            /*
            for(int i = 0; i<5; i++)
            {
                if(FingerPositions[i] < RELEASED_FINGER_POSITION){ // If this finger is not released, release it
                    hasMoved = 1;
                    Hand_Move(FingerPositions[0] + 1, (int[1]) {i}, 1);
//                    printf("%i passed lower threshold of %i. Releasing... \r\n", half_buffer[startIndex], LOWER_THRESHOLD);
                }
            }
            if(hasMoved == 1)
            { // Only delay if a finger has moved
                HAL_Delay(BASE_VELOCITY_DELAY);
            }
             */
        }
        startIndex++;
    }
}