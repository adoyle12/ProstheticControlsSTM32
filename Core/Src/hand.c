#include "hand.h"
#include <SERVO.h>

int FingerPositions[5] = {RELEASED_FINGER_POSITION, RELEASED_FINGER_POSITION, RELEASED_FINGER_POSITION, RELEASED_FINGER_POSITION, RELEASED_FINGER_POSITION};

// Move fingers
void Hand_Move(int destination, int *fingers, int fingerArraySize){
    for(int i = 0; i < fingerArraySize; i++){
        SERVO_RawMove(fingers[i],destination);
        FingerPositions[fingers[i]] = destination;
//        printf("Moving Servo %i to %i \r\n", fingers[i], destination);
    }
}