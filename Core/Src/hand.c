#include "hand.h"
#include <SERVO.h>

// Move fingers
void Hand_Move(int destination, int *fingers, int fingerArraySize){
    for(int i = 0; i < fingerArraySize; i++){
        SERVO_RawMove(fingers[i],destination);
        FingerPositions[fingers[i]] = destination;
    }
    printf("Moving Servo %i to %i \r\n", fingers[2], destination);
}