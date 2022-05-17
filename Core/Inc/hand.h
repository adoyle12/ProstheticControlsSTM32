#ifndef PROSTHETICCONTROLSSTM32_HAND_H
#define PROSTHETICCONTROLSSTM32_HAND_H

#include <stdint.h>
#include "config.h"

void Hand_Move(int destination, int *fingers, int fingerArraySize);

#endif //PROSTHETICCONTROLSSTM32_HAND_H