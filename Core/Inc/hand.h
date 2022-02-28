#ifndef PROSTHETICCONTROLSSTM32_HAND_H
#define PROSTHETICCONTROLSSTM32_HAND_H

#include <stdint.h>

#define THUMB 0
#define INDEX 1
#define MIDDLE 2
#define RING 3
#define LITTLE 4
#define RELEASED_FINGER_POSITION 100
#define CLENCHED_FINGER_POSITION 10

extern int FingerPositions[5];

// Functions
void Hand_Move(int destination, int *fingers, int fingerArraySize);

#endif //PROSTHETICCONTROLSSTM32_HAND_H
