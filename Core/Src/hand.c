#include "hand.h"
#include <SERVO.h>

int FingerPositions[5] = {0,0,0,0,0};

// Move fingers
void Hand_Move(int destination, int *fingers, int fingerArraySize){
    for(int i = 0; i < fingerArraySize; i++){
        SERVO_RawMove(fingers[i],destination);
        FingerPositions[fingers[i]] = destination;

    }

    printf("Moving Servos to %i \r\n", destination);
}