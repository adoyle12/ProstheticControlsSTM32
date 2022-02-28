#include "hand.h"
#include <SERVO.h>

int FingerPositions[5] = {RELEASED_FINGER_POSITION, RELEASED_FINGER_POSITION, RELEASED_FINGER_POSITION, RELEASED_FINGER_POSITION, RELEASED_FINGER_POSITION};

void Hand_Move(int destination, int *fingers, int fingerArraySize){
//    int nowClenched = 1; // Used to check if all fingers are at max position

    // Move fingers
    for(int i = 0; i < fingerArraySize; i++){
        printf("Moving Servo %i to %i \r\n", i, destination);
        SERVO_RawMove(fingers[i],destination);
        FingerPositions[fingers[i]] = destination;
    }

    // Check if clenched
//    for(int i=0; i<5; i++){
//        if(HandFingerPositions[i] > MAX_FINGER_POSITION){
//            nowClenched = 0;
//        }
//    }
}