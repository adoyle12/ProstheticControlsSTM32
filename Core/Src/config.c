#include "config.h"
#include "SERVO.h"
#include "hand.h"

int minThreshold = 0;
int middleThreshold = 0;
int maxThreshold = 0;
int StartPosition = 0;

//Calculates the 3 thresholds based on peak clenched voltage and average resting voltage
void CalculateThresholds(int clenchedVoltage, int restingVoltage){
    minThreshold = restingVoltage * 4096 / 3.3 + 100;
    middleThreshold = (clenchedVoltage-restingVoltage) * 4096 / 3.3 * 0.6;
    maxThreshold = (clenchedVoltage-restingVoltage) * 4096 / 3.3 * 0.8;
}

int CalculateMaxServoPulse()
{
    int MaxPulse = 0;
    for (int i = 0; i < SERVO_NUM; i++)
    {
        if (SERVO_Get_MaxPulse(i) > MaxPulse)
        {
            MaxPulse = SERVO_Get_MaxPulse(i);
        }
    }
    StartPosition = SERVO_Get_MinPulse(0);

    for (int i = 0; i < 5; ++i)
    {
        FingerPositions[i] = StartPosition;
    }

    return MaxPulse;
}