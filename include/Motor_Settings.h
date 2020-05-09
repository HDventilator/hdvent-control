//
// Created by david on 09.05.20.
//

#ifndef HDVENT_CONTROL_MOTOR_SETTINGS_H
#define HDVENT_CONTROL_MOTOR_SETTINGS_H

// Motor settings
// motor curve for volume control operation
uint8_t const RUN_KVAL_IN = 200;
uint8_t const ACC_KVAL_IN = 200;
uint8_t const DEC_KVAL_IN = 200;
uint8_t const HOLD_KVAL_IN = 200;
uint8_t const RUN_KVAL_EX = 200;
uint8_t const ACC_KVAL_EX = 200;
uint8_t const DEC_KVAL_EX = 200;
uint8_t const HOLD_KVAL_EX = 200;


float const ACC_IN = 300; // steps/s/s
float const DEC_IN = 300; // steps/s/s
float const ACC_EX = 300; // steps/s/s
float const SPEED_EX = 100; //steps/s
float const DEC_EX = 300; // steps/s/s
bool const DIR_IN = 1;
bool const DIR_EX = abs(DIR_IN - 1);
int const STEP_DIVIDER_REGISTER = STEP_FS_128;
int const STEP_DIVIDER = 128;


#endif //HDVENT_CONTROL_MOTOR_SETTINGS_H
