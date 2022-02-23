#include <hand.h>
#include <SERVO.h>

void Hand_Move(int destination, int fingers[]){
    int nowClenched = 1; // Used to check if all fingers are at max position

    // Move fingers
    for(int i=0; i<(sizeof fingers / sizeof fingers[0]); i++){
        SERVO_RawMove(fingers[i],destination);
        FingerPositions[fingers[i]] = destination;
    }

    // Check if clenched
    for(int i=0; i<5; i++){
        if(FingerPositions[i] > MAX_FINGER_POSITION){
            nowClenched = 0;
        }
    }

    isClenched = nowClenched;
}