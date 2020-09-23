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
#include <User_Input.h>
#include <Wire.h>
#include <Serial_Protocol.h>
#include <Aggregated_Parameter.h>
#include <Warning.h>
#include <Encoder.h>

extern "C" {
#include "utility/twi.h"  // from Wire library, so we can do bus scanning
}


/* ***********************
 * Constant definitions
 * ***********************
 */

powerSTEP Stepper(0, nCS_PIN, nSTBY_nRESET_PIN);;  // Nr, CS, Reset => 0 , D16/A2 (PA4), D4 (PB5) for IHM02A1 board

// Clinical settings
int const PRESSURE_MAX = 0; // mm H20
float const TIME_HOLD_PLATEAU = 0.050; // seconds, hold time after inspiratory phase
float const TEMPERATURE_INFLUX_THRESHOLD = 20;// °C, start heating when influx temperature falls below this value

int const STEPS_EX_HOMING = 80; // steps to move out when trying to find home
int const STEPS_IN_HOMING = 80; // steps to move in when trying to find home

int const STEPS_FS_FULL_TURN = 200; // how many full steps for one full turn of the motor

float calculateSpeed(int acc, int dec, float t, int steps);
// state flags
enum VentilationState {START_IN=0, MOVING_IN=1, HOLDING_IN=2, START_EX=3, MOVING_EX=4, HOLDING_EX=5, STARTUP=6, END_IN=7, END_EX=8, HOMING_EX=9, IDLE=10, START_HOMING=11};


float const PRESSURE_FLOW_CONVERSION = 771.49; // ml*s^-1 / mbar
float const PRESSURE_FLOW_CONVERSION_OFFSET = 0.017;
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
void runStepper(int speed, int acc, int dec, int dir);
void writeDiagnosticParameters();
VentilationState ventilationStateMachine(VentilationState &state);
int readStatusRegister();
bool getStatusFlag(int r, int n);

void checkHomeSensors(bool& isHome);
void readSensors();
void scan_i2c();

bool tripleVoteHome(bool optical, bool angle, bool stepper);
int getPosition(int tolerance=1*STEP_DIVIDER);
float motorPositionToVolume(uint16_t position);
void toggleIsHome();
void toggleEnableEncoder();
void readUserInput();
void serialDebug();
void setKVals(uint8_t dir);
void moveStepper(int steps,  int speed, int dir);
void runMachineDiagnostics();

/* *****************************
 * Global Variables
 * *****************************
 */

/*
 * Sensors
 */
SSC_100MD4A3 pressureSensor(1);
SSC_016MD2A5 flowSensor(0);
Angle_Sensor angleSensor(PIN_RPS_OUT, STEPS_FS_FULL_TURN*STEP_DIVIDER, 360, 0, 1024, 10);
Optical_Sensor opticalHomeSensor(PIN_OPTICAL_SWITCH_HOME);
Stepper_Monitor stepperMonitor(&Stepper, STEP_DIVIDER*5);

float timeEx=1;
float timeIn=1;
float speedIn = 50; // steps/s
unsigned int stepsInterval = 300;

// user-set parameters
User_Parameter allUserParams[(int) UP::LAST_PARAM_LABEL];

struct diagnosticParameters_t {
    Diagnostic_Parameter peep=Diagnostic_Parameter(0,0,0,"peep");
    Diagnostic_Parameter tidalVolume=Diagnostic_Parameter(0,0,0,"tvol");
    Diagnostic_Parameter volume=Diagnostic_Parameter(0,0,0,"cvol");
    Diagnostic_Parameter flow=Diagnostic_Parameter(0,0,0,"flow");
    Diagnostic_Parameter airwayPressure = Diagnostic_Parameter(0,0,0,"pins");;
    Diagnostic_Parameter respiratoryRate;
    Diagnostic_Parameter plateauPressure;
    Diagnostic_Parameter meanPressure;
    Diagnostic_Parameter pressureChange; //millibar per second
    Aggregated_Parameter minuteVolume= Aggregated_Parameter(0, 0, 0, "mvol", 60);
} diagnosticParameters;

struct machineDiagnostics_t {
    Diagnostic_Parameter cycle_time=Diagnostic_Parameter(0,0,0,"tcyc");
    Diagnostic_Parameter ventilationState=Diagnostic_Parameter(0,0,0,"svent");
} machineDiagnostics;

struct stopwatches_t{
    Stopwatch holdingIn;
    Stopwatch inspiration;
    Stopwatch expiration;
    Stopwatch mainLoop;
    Stopwatch pressureRate;
    Stopwatch homing;
    Stopwatch sinceIdle;
} stopwatch;



float pathRatio=1;

VentilationMode mode= OL_CMV ;
VentilationState ventilationState = STARTUP;

float oldPressure;
unsigned long cycleTime =0;
bool saveUserParams = false;
bool isHome=false;
bool runVentilation=true;

bool debuggingOn = true;

Sensor::state_t stepperPositionState = Sensor::OK;
Sensor::state_t anglePositionState = Sensor::OK;
float motorSpeed;
PID pressureControlPID();

VentilationController controller(OL_CMV, diagnosticParameters.airwayPressure, diagnosticParameters.flow,
                                 allUserParams);
LiquidCrystal lcd(PIN_LCD_RS, PIN_LCD_RW, PIN_LCD_EN, PIN_LCD_D4, PIN_LCD_D5, PIN_LCD_D6, PIN_LCD_D7);
PacketSerial cobsSerial;
uint8_t potiPins[4] = {PIN_POTI_AD, PIN_POTI_IE, PIN_POTI_TV, PIN_POTI_RR};
User_Input userInput(allUserParams, &mode, &saveUserParams);
Encoder encoder;
Display display(lcd, allUserParams, &VC_CMV, &encoder._increment, &encoder._increment, &encoder.shortPressDetected, &encoder.longPressDetected, &userInput);
#endif //HDVENT_CONTROL_HDVENT_CONTROL_H
