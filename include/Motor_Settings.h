//
// Created by david on 09.05.20.
//

#ifndef HDVENT_CONTROL_MOTOR_SETTINGS_H
#define HDVENT_CONTROL_MOTOR_SETTINGS_H

// Motor settings
// motor curve for volume control operation
uint8_t const RUN_KVAL_IN = 80;
uint8_t const ACC_KVAL_IN = 80;
uint8_t const DEC_KVAL_IN = 80;
uint8_t const HOLD_KVAL_IN = 80;
uint8_t const RUN_KVAL_EX = 70;
uint8_t const ACC_KVAL_EX = 70;
uint8_t const DEC_KVAL_EX = 70;
uint8_t const HOLD_KVAL_EX = 10;


float const ACC_IN = 300; // steps/s/s
float const DEC_IN = 300; // steps/s/s
//float const SPEED_IN = 800; //steps/s
float const ACC_EX = 500; // steps/s/s
float const SPEED_EX = 600; //steps/s
float const DEC_EX = 500; // steps/s/s
bool const DIR_IN = 1;
bool const DIR_EX = abs(DIR_IN - 1);
int const STEP_DIVIDER_REGISTER = STEP_FS_128;
int const STEP_DIVIDER = 128;

unsigned int const STEPS_FULL_TURN = 200;
unsigned int const STEPS_FULL_RANGE = 80;

#endif //HDVENT_CONTROL_MOTOR_SETTINGS_H
