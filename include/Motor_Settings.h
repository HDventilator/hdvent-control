//
// Created by david on 09.05.20.
//

#ifndef HDVENT_CONTROL_MOTOR_SETTINGS_H
#define HDVENT_CONTROL_MOTOR_SETTINGS_H

// Motor settings
// motor curve for volume control operation
uint8_t const RUN_KVAL_IN = 100; // 80
uint8_t const ACC_KVAL_IN = 100;
uint8_t const DEC_KVAL_IN = 100;
uint8_t const HOLD_KVAL_IN = 100;
uint8_t const RUN_KVAL_EX = 80;
uint8_t const ACC_KVAL_EX = 80;
uint8_t const DEC_KVAL_EX = 80;
uint8_t const HOLD_KVAL_EX = 80;


float const ACC_IN = 1000; // steps/s/s
float const DEC_IN =1000; // steps/s/s
//float const SPEED_IN = 800; //steps/s
float const ACC_EX = 1000; // steps/s/s
float const SPEED_EX = 1000; //steps/s
float const DEC_EX = 1000; // steps/s/s

float const SPEED_CORRECTION = 1.23;

bool const DIR_IN = 0;
bool const DIR_EX = abs(DIR_IN - 1);
int const STEP_DIVIDER_REGISTER = STEP_FS_128;
int const STEP_DIVIDER = 128;

unsigned int const STEPS_FULL_TURN = 200;
unsigned int const STEPS_FULL_RANGE = 55;

#endif //HDVENT_CONTROL_MOTOR_SETTINGS_H
