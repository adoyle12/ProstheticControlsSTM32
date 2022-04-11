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
	// Servo Motor 1 Configurations - Thumb - Move 54 degrees
    {
		&htim1,
		&TIM1->CCR1,
		TIM_CHANNEL_1,
		54
	},

    // Servo Motor 2 Configurations - Index - Move 72 degrees
    {
        &htim1,
        &TIM1->CCR2,
        TIM_CHANNEL_2,
        72

    },

    // Servo Motor 3 Configurations - Middle - Move 117 degrees
    {
        &htim1,
        &TIM1->CCR3,
        TIM_CHANNEL_3,
        117
    },

    // Servo Motor 4 Configurations - Ring - Move 72 degrees
    {
        &htim1,
        &TIM1->CCR4,
        TIM_CHANNEL_1,
        72
    },

    // Servo Motor 5 Configurations - Pinky - Move 54 degrees
    {
        &htim2,
        &TIM2->CCR1,
        TIM_CHANNEL_2,
        54
    }

};
