#include "config.h"

//Calculates the 3 thresholds based on peak clenched voltage and average resting voltage
void CalculateThresholds(int clenchedVoltage, int restingVoltage){
    minThreshold = restingVoltage * 4096 / 3.3 + 100;
    middleThreshold = (clenchedVoltage-restingVoltage) * 4096 / 3.3 * 0.6;
    maxThreshold = (clenchedVoltage-restingVoltage) * 4096 / 3.3 * 0.8;
}

void CalculateServoPulses(){

}