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
#include <Wire.h>
#include <Serial_Protocol.h>
#include <Aggregated_Parameter.h>
#include <Warning.h>
#include <Encoder.h>
#include <Buzzer.h>
#include <Pushbutton.h>
#include <StopwatchEvent.h>
#include <StopwatchMus.h>
#include <EEPROM.h>
#include <Parameter_Container.h>
#include <Clinical_Settings.h>
#include <Sensor_Settings.h>
#include <Debug_Settings.h>

extern "C" {
#include "utility/twi.h"  // from Wire library, so we can do bus scanning
}


powerSTEP Stepper(0, nCS_PIN, nSTBY_nRESET_PIN);;  // Nr, CS, Reset => 0 , D16/A2 (PA4), D4 (PB5) for IHM02A1 board



float calculateSpeed(int acc, int dec, float t, int steps);
// state flags
enum VentilationState {START_IN=0, MOVING_IN=1, HOLDING_IN=2, START_EX=3, MOVING_EX=4, HOLDING_EX=5, STARTUP=6,
        END_IN=7, END_EX=8, HOMING_EX=9, IDLE=10, START_HOMING=11};

/* **********************
 * Function declarations
 * **********************
 */
void checkAlarms();
void safeToEEPROM();
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
void writeUserInput();
void serialDebug();
void setKVals(uint8_t dir);
void moveStepper(int steps, float speed, int dir);
void runMachineDiagnostics();
void rescaleParameterLimits();

/* *****************************
 * Global Variables
 * *****************************
 */

/*
 * Sensors
 */
SSC_100MD4A3 pressureSensor(1);
SSC_016MD2A5 flowSensor(0);
Angle_Sensor angleSensor(PIN_RPS_OUT, STEPS_FS_FULL_TURN*STEP_DIVIDER, 360, 1024*0.05, 1024*0.95, 50);
Optical_Sensor opticalHomeSensor(PIN_OPTICAL_SWITCH_HOME);
Stepper_Monitor stepperMonitor(&Stepper, STEP_DIVIDER * 5, DIR_IN);


// user-set parameters
//User_Parameter allUserParams[(int) UP::LAST_LABEL];
Parameter_Container<(int) UP::LAST_LABEL, UP> allUserParams;
VentiModeContainer<(int) VentiModes::LAST_LABEL, VentiModes> allVentiModes;


diagnosticParameters_t diagnosticParameters =
        { Diagnostic_Parameter("tvol","Tvol",  0,800),
        Diagnostic_Parameter("peep", "PEEP", 0,30),
       Diagnostic_Parameter("cvol","Cvol"),
        Diagnostic_Parameter("flow","Flow"),
         Diagnostic_Parameter("pins","Pins", 0, 60),
                Diagnostic_Parameter("plat","Plat", 0, 60),
                Diagnostic_Parameter("mvol", "mVol")
        };
Stopwatch motorTestTimer;

Aggregated_Parameter minuteVolume(60);

struct machineDiagnostics_t {
    Diagnostic_Parameter cycle_time=Diagnostic_Parameter(0,0,0,"tcyc");
    Diagnostic_Parameter ventilationState=Diagnostic_Parameter(0,0,0,"svent");
    Diagnostic_Parameter stepperPosition=Diagnostic_Parameter(0,0,0,"step");
    Diagnostic_Parameter rotationSensor=Diagnostic_Parameter(0,0,0,"rots");
    Diagnostic_Parameter absolutePosition=Diagnostic_Parameter(0,0,0,"pabs");
    Diagnostic_Parameter calculatedSpeed=Diagnostic_Parameter(0,0,0,"vclc");
    Diagnostic_Parameter setpointPID=Diagnostic_Parameter(0,0,0,"setp");
} machineDiagnostics;

struct stopwatches_t{

    Stopwatch holdingIn;
    Stopwatch inspiration;
    Stopwatch expiration;
    Stopwatch mainLoop;
    Stopwatch pressureRate;
    Stopwatch homing;
    Stopwatch sinceIdle;
    Stopwatch movingIn;
} stopwatch;

StopwatchMus mainLoopMus;

void writeDiagnosticAlarms();

VentilationMode mode= VC_CMV ;
VentilationState ventilationState = STARTUP;


unsigned long cycleTimeMus =0;
bool isHome=false;
bool runVentilation=true;
void ledService();
bool DO_SERIAL_DEBUG = true;
void readFromEEPROM();
void enumerateEEPROM();

Sensor::state_t stepperPositionState = Sensor::OK;
Sensor::state_t anglePositionState = Sensor::OK;
PID pressureControlPID();
bool alarmIsTriggered=false;
VentilationController controller(VC_CMV, diagnosticParameters.s.airwayPressure, diagnosticParameters.s.flow,
                                 allUserParams.params);
LiquidCrystal lcd(PIN_LCD_RS, PIN_LCD_RW, PIN_LCD_EN, PIN_LCD_D4, PIN_LCD_D5, PIN_LCD_D6, PIN_LCD_D7);
PacketSerial cobsSerial;

Encoder encoder(PIN_ENCO_A, PIN_ENCO_B);

Pushbutton confirmButton(PIN_EDIT_MODE);
Pushbutton cancelButton(PIN_OPTICAL_SWITCH_END);
Pushbutton alarmOverwrite(PIN_ALARM_MUTE);
Pushbutton encoderButton(PIN_ENCO_BTN);
Pushbutton enableVentilation(PIN_SD_VENTI);

TimedToggler greenLEDBlink(500);
TimedToggler orangeLEDBlink(300);

Display<(int)UP::LAST_LABEL, UP, (int)VentiModes::LAST_LABEL, VentiModes>
        display(lcd, allUserParams, allVentiModes, &diagnosticParameters);

Buzzer buzzer(PIN_ALARM_ENABLE, 500, 1000);

#endif //HDVENT_CONTROL_HDVENT_CONTROL_H
