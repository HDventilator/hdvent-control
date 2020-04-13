//
// Created by david on 13.04.20.
//

#ifndef HDVENT_CONTROL_PIN_DEFINITIONS_H
#define HDVENT_CONTROL_PIN_DEFINITIONS_H

// Pin definitions
int const PIN_OPTICAL_END = 9;
int const MANUAL_CYCLE_SWITCH_PIN = 8;

// Pin definitions for the X-NUCLEO-IHM03A1 connected to an Uno-compatible board
int const nCS_PIN = 10;
int const STCK_PIN = 9;
int const nSTBY_nRESET_PIN = 8;
int const nBUSY_PIN = 4;

int const PIN_MON_12V_IN1 = A0;
int const PIN_MON_12V_IN2 = A1;
int const PIN_IMON = A2;
int const PIN_RPS_OUT = A3;
int const PIN_TEMP_SEN_1 = A4;
int const PIN_TEMP_SEN_2 = A5;
int const PIN_HEAT_SENSE = A6;
int const PIN_POTI_RR = A7;
int const PIN_POTI_TV = A8;
int const PIN_POTI_IE = A9;
int const PIN_AD_SENSI = A10;

int const PIN_STEPPER_FLAG = 2;
int const PIN_STEPPER_BUSY = 4;
int const PIN_STATUS1_FP_R = 5;
int const PIN_STATUS1_FP_G = 6;
int const PIN_STATUS2_FP_R = 7;

int const PIN_STEPPER_RESET = 8;
int const PIN_STEPPER_STEP_CLK = 9;
int const PIN_STEPPER_SPI_SS = 10;

int const PIN_STATUS2_FP_G = 16;
int const PIN_I2C_nRES = 17;
int const PIN_OPTICAL_SWITCH_END = 18;
int const PIN_OPTICAL_SWITCH_HOME = 19;
int const PIN_I2C_SDA = 20;
int const PIN_I2C_SCL = 21;

int const PIN_STATUS3_FP_R = 23;
int const PIN_STATUS3_FP_G = 25;
int const PIN_ALARM_ENABLE = 27;
int const PIN_ALARM_MUTE = 29;
int const PIN_SD_VENTI = 31;
int const PIN_VENTI_MODE = 35;
int const PIN_ENCO_A = 37;
int const PIN_ENCO_B = 39;
int const PIN_ENCO_BTN = 41;
int const PIN_HEAT_ENABLE = 36;



#endif //HDVENT_CONTROL_PIN_DEFINITIONS_H
