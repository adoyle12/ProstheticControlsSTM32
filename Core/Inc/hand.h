#ifndef PROSTHETICCONTROLSSTM32_HAND_H
#define PROSTHETICCONTROLSSTM32_HAND_H

#include <stdint.h>

#define THUMB 0
#define INDEX 1
#define MIDDLE 2
#define RING 3
#define LITTLE 4
#define MAX_FINGER_POSITION 100
#define MIN_FINGER_POSITION 10

// Properties
int FingerPositions[5];
int isClenched = 0;

// Functions
void Hand_Move(int destination, int fingers[]);

#endif PROSTHETICCONTROLSSTM32_HAND_H
