/*
 * File: SERVO_cfg.c
 * Driver Name: [[ SERVO Motor ]]
 * SW Layer:   ECUAL
 * Created on: Jun 28, 2020
 * Author:     Khaled Magdy
 * -------------------------------------------
 * For More Information, Tutorials, etc.
 * Visit Website: www.DeepBlueMbedded.com
 *
 */

#include "SERVO.h"

const SERVO_CfgType SERVO_CfgParam[SERVO_NUM] =
{
	// Servo Motor 1 Configurations - Pinky - Move 100 degrees
    {
		&htim1,
		&TIM1->CCR1,
		TIM_CHANNEL_1,
		100
	},

    // Servo Motor 2 Configurations - Ring - Move 118 degrees
    {
        &htim1,
        &TIM1->CCR2,
        TIM_CHANNEL_2,
        118

    },

    // Servo Motor 3 Configurations - Middle - Move 141 degrees
    {
        &htim1,
        &TIM1->CCR3,
        TIM_CHANNEL_3,
        141
    },

    // Servo Motor 4 Configurations - Index - Move 118 degrees
    {
        &htim1,
        &TIM1->CCR4,
        TIM_CHANNEL_4,
        118
    },

    // Servo Motor 5 Configurations - Thumb - Move 72 degrees
    {
        &htim2,
        &TIM2->CCR1,
        TIM_CHANNEL_1,
        72
    }

};
