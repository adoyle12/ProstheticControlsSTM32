#ifndef PROSTHETICCONTROLSSTM32_HAND_H
#define PROSTHETICCONTROLSSTM32_HAND_H

#include <stdint.h>
#include "config.h"

extern int FingerPositions[5];
extern int FingerClosedPositions[5];

void Hand_Move(int destination, int *fingers, int fingerArraySize);

#endif //PROSTHETICCONTROLSSTM32_HAND_H