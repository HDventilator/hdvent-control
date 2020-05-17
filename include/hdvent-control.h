//
// Created by david on 17.05.20.
//

#ifndef HDVENT_CONTROL_HDVENT_CONTROL_H
#define HDVENT_CONTROL_HDVENT_CONTROL_H
#include <Arduino.h> // main arduino library
#include <SPI.h> // arduino spi library
#include <avr/wdt.h> // watchdog library
#include <powerSTEP01ArduinoLibrary.h>
#include <Honeywell_SSC.h>
#include "Pin_Definitions_Mega.h"
#include <angleSensor.h>
#include "User_Parameter.h"
#include "Diagnostic_Parameter.h"
#include "Ventilation_Modes.h"
#include "Stopwatch.h"
#include "Trigger.h"
#include "PID_v1.h"
#include "Ventilation_Controller.h"
#include <Display.h>
#include <LiquidCrystal.h>
#include <Motor_Settings.h>
#include <Optical_Sensor.h>
#include <Stepper_Monitor.h>

/* ***********************
 * Constant definitions
 * ***********************
 */

powerSTEP Stepper(0, nCS_PIN, nSTBY_nRESET_PIN);;  // Nr, CS, Reset => 0 , D16/A2 (PA4), D4 (PB5) for IHM02A1 board

// Clinical settings
int const PRESSURE_MAX = 0; // mm H20
float const TIME_HOLD_PLATEAU = 0.200; // seconds, hold time after inspiratory phase
float const TEMPERATURE_INFLUX_THRESHOLD = 20;// °C, start heating when influx temperature falls below this value

int const STEPS_EX_HOMING = 80; // steps to move out when trying to find home
int const STEPS_IN_HOMING = 80; // steps to move in when trying to find home

int const STEPS_FS_FULL_TURN = 200; // how many full steps for one full turn of the motor

// state flags
enum VentilationState {START_IN, MOVING_IN, HOLDING_IN, START_EX, MOVING_EX, HOLDING_EX, STARTUP, END_IN, END_EX, HOMING_EX, HOMING_IN, IDLE};
enum DisplayState {HOME, };


/* **********************
 * Function declarations
 * **********************
 */

byte spi_test();
void ConfigureStepperDriver();
int getBoardStatus();
void UpdateMotorCurveParameters(float respiratoryRate, float pathRatio, float IERatio);
void PrintMotorCurveParameters();
bool isBusy();

void moveStepper(int steps, int speed, int acc, int dec, int dir);
VentilationState ventilationStateMachine(VentilationState state);
int readStatusRegister();
bool getStatusFlag(int r, int n);

void checkHomeSensors();



bool tripleVoteHome(bool optical, bool angle, bool stepper);
int getPosition(int tolerance=1*STEP_DIVIDER);
float motorPositionToVolume(uint16_t position);
void toggleIsHome();
void toggleEnableEncoder();


/* *****************************
 * Global Variables
 * *****************************
 */

/*
 * Sensors
 */
Honeywell_SSC pressureSensor(0x48,0,-150,150,0.1*16383,0.9*16383);
Honeywell_SSC flowSensor(0x68,0,0,4000,-1,1);
Angle_Sensor angleSensor(PIN_RPS_OUT, STEPS_FS_FULL_TURN*STEP_DIVIDER, 360, 0, 1024, 10);
Optical_Sensor opticalHomeSensor(PIN_OPTICAL_SWITCH_HOME);
Stepper_Monitor stepperMonitor(&Stepper);

float timeEx=1;
float timeIn=1;
float speedIn = 50; // steps/s
unsigned int stepsInterval = 300;

// user-set parameters
User_Parameter allUserParams[(int) UP::LAST_PARAM_LABEL];

struct diagnosticParameters_t {
    Diagnostic_Parameter peep;
    Diagnostic_Parameter tidalVolume=Diagnostic_Parameter(0,0,0,"TVOL");
    Diagnostic_Parameter Volume=Diagnostic_Parameter(0,0,0,"VOLU");
    Diagnostic_Parameter flow=Diagnostic_Parameter(0,0,0,"FLOW");
    Diagnostic_Parameter airwayPressure = Diagnostic_Parameter(0,0,0,"P_AW");;
    Diagnostic_Parameter respiratoryRate;
    Diagnostic_Parameter plateauPressure;
    Diagnostic_Parameter meanPressure;
    Diagnostic_Parameter minuteVolume;
    Diagnostic_Parameter pressureChange; //millibar per second
} diagnosticParameters;

struct stopwatches_t{
    Stopwatch holdingIn;
    Stopwatch inspiration;
    Stopwatch expiration;
    Stopwatch mainLoop;
    Stopwatch pressureRate;
} stopwatch;

float pathRatio=1;

VentilationMode mode=VC_CMV ;
VentilationState State = STARTUP;

float oldPressure;
unsigned long cycleTime =0;


Sensor::SensorState stepperPositionState = Sensor::OK;
Sensor::SensorState anglePositionState = Sensor::OK;
float motorSpeed;
PID pressureControlPID();

VentilationController controller(OL_CMV, diagnosticParameters.airwayPressure,diagnosticParameters.flow);

LiquidCrystal lcd(12, 11, 10, 9, 8, 7);

#endif //HDVENT_CONTROL_HDVENT_CONTROL_H
