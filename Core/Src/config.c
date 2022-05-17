#include "config.h"
#include "SERVO.h"
#include "hand.h"

float minThreshold = 0;
float middleThreshold = 0;
float maxThreshold = 0;
float StartPosition = 0;

//Calculates the 3 thresholds based on peak clenched voltage and average resting voltage
void CalculateThresholds(float clenchedVoltage, float restingVoltage){
    minThreshold = restingVoltage * 4096 / 3.3 + 200;
    middleThreshold = (clenchedVoltage-restingVoltage) * 4096 / 3.3 * 0.45;
    maxThreshold = (clenchedVoltage-restingVoltage) * 4096 / 3.3 * 0.9;
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

    return MaxPulse;
}

void SetFingersStartPosition()
{
    // All min pulses are the same so pick any
    StartPosition = SERVO_Get_MinPulse(0);

    for (int i = 0; i < 5; ++i)
    {
        FingerPositions[i] = StartPosition;
    }

}