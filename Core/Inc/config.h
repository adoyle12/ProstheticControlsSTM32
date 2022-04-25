#ifndef PROSTHETICCONTROLSSTM32_CONFIG_H
#define PROSTHETICCONTROLSSTM32_CONFIG_H

#define PWM_PERIOD 3.0                        // PWM Signal Period in ms
#define WIDTH_PER_DEGREE 11.1e-3             // Width Per Degree in ms (i.e. the amount to change the PWM signal by to move one degree, this is specific to the servo model)
#define STARTING_PULSE 0.7                  // Starting position for each servo in ms, set at 0.7 for a small buffer before end range

#define CLENCHED_VOLTAGE 2.6
#define RESTING_VOLTAGE 2

#define BASE_VELOCITY_DELAY 35

extern int FingerPositions[5];

extern int minThreshold;
extern int middleThreshold;
extern int maxThreshold;
extern int StartPosition;

void CalculateThresholds(int clenchedVoltage, int restingVoltage);
int CalculateMaxServoPulse();

#endif //PROSTHETICCONTROLSSTM32_CONFIG_H
