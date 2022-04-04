#ifndef PROSTHETICCONTROLSSTM32_CONFIG_H
#define PROSTHETICCONTROLSSTM32_CONFIG_H

#define RELEASED_FINGER_POSITION 100
#define CLENCHED_FINGER_POSITION 10

#define CLENCHED_VOLTAGE 2.6
#define RESTING_VOLTAGE 0.3

#define BASE_VELOCITY_DELAY 35

int minThreshold;
int middleThreshold;
int maxThreshold;

void CalculateThresholds(int clenchedVoltage, int restingVoltage);
void CalculateServoPulses();

#endif //PROSTHETICCONTROLSSTM32_CONFIG_H
