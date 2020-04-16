//
// Created by david on 04.04.20.
//
#include <Arduino.h> // main arduino library
#include <SPI.h> // arduino spi library
#include <avr/wdt.h> // watchdog library
#include <powerSTEP01ArduinoLibrary.h>
#include <Honeywell_SSC.h>
#include "Pin_Definitions_Uno.h"
#include <Wire.h>
#include <angleSensor.h>
#include "User_Parameter.h"
/* ***********************
 * Constant definitions
 * ***********************
 */

powerSTEP Stepper(0, nCS_PIN, nSTBY_nRESET_PIN);;  // Nr, CS, Reset => 0 , D16/A2 (PA4), D4 (PB5) for IHM02A1 board

// Clinical settings
int const PRESSURE_MAX = 0; // mm H20
float const TIME_HOLD_PLATEAU = 0.200; // seconds, hold time after inspiratory phase
float const TEMPERATURE_INFLUX_THRESHOLD = 20;// °C, start heating when influx temperature falls below this value

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
int const STEP_DIVIDER_REGISTER = STEP_FS_64;
int const STEP_DIVIDER = 64;

int const STEPS_EX_HOMING = 80; // steps to move out when trying to find home
int const STEPS_IN_HOMING = 80; // steps to move in when trying to find home

int const STEPS_FS_FULL_TURN = 200; // how many full steps for one full turn of the motor

// state flags
enum PumpingState {START_IN, MOVING_IN, HOLDING_IN, START_EX, MOVING_EX, HOLDING_EX, STARTUP, HOMING_EX, HOMING_IN, IDLE};
enum ControlMode {VOLUME_CONTROLLED, PRESSURE_CONTROLLED, BIPAP, VOLUME_OPEN_LOOP, MANUAL, CALIBRATION};
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
PumpingState openLoopVolumeControl(PumpingState state);
int motorStatusRegister;
void writeToLCD(float respiratoryRate, float tidalVolume, float ratioInEx, float PEEP, float peak, float plat);
void read_potis();
int readStatusRegister();
void printUserValues();
bool getStatusFlag(int r, int n);
void updateDisplay(float respiratoryRate, float pathRatio, float IERatio,
                   float pressurePeak, float plateauPressure, float peePressure);

void startInfluxHeating();
void stopInfluxHeating();
void manualControl();
void enableEncoder();
bool tripleVoteHome(bool optical, bool angle, bool stepper, bool &isHome);
int votePosition(int stepper, int angle);
float motorPositionToVolume(uint16_t position);
bool isHome();
void toggleIsHome();
void toggleEnableEncoder();

/* *****************************
 * Global Variables
 * *****************************
 */

/*
 * Sensors
 */
Honeywell_SSC pressureSensor = Honeywell_SSC(0x48,0,-150,150,0.1*16383,0.9*16383);
Honeywell_SSC flowSensor = Honeywell_SSC(0x68,0,0,4000,-1,1);
Angle_Sensor angleSensor = Angle_Sensor(PIN_RPS_OUT, STEPS_FS_FULL_TURN*STEP_DIVIDER, 360, 0, 1024, 10);



float timeEx=1;
float timeIn=1;
float speedIn = 50; // steps/s

unsigned int stepsFullRange = 300;
unsigned int stepsInterval = 300;

// user-set parameters
float respiratoryRate = 15;
float pathRatio = 1;
float IERatio = 0.5;

bool startCyclingSwitch = 0;

unsigned long timerStartMovingIn = 0;
unsigned long timerStartMovingEx = 0;
unsigned long timerStartHoldingIn = 0;
unsigned long timerStartHoldingEx = 0;
float peakPressure=0;
float pressurePlateau=0;
float pressurePEEP=0;
float maxPressure=0;
float pressure=0;
float temperatureInflux=0;
ControlMode mode = VOLUME_OPEN_LOOP;
PumpingState currentState = STARTUP;
//manual control
int stepCounter=0;
int stepperPosition=0;
float anglePosition=0; // TODO map angle position on motor steps
Sensor::SensorState stepperPositionState = Sensor::OK;
Sensor::SensorState anglePositionState = Sensor::OK;

void setup()
{
    Serial.begin(115200);

    // Prepare pins
    pinMode(nSTBY_nRESET_PIN, OUTPUT);
    pinMode(nCS_PIN, OUTPUT);

    // SPI pins
    pinMode(MOSI, OUTPUT);
    pinMode(MISO, OUTPUT);
    pinMode(SCK, OUTPUT);

    // Reset powerSTEP and set CS
    digitalWrite(nSTBY_nRESET_PIN, HIGH);
    digitalWrite(nSTBY_nRESET_PIN, LOW);
    digitalWrite(nSTBY_nRESET_PIN, HIGH);
    digitalWrite(nCS_PIN, HIGH);

    // Start SPI
    SPI.begin();
    SPI.setDataMode(SPI_MODE3);

    pinMode(PIN_OPTICAL_SWITCH_HOME, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PIN_OPTICAL_SWITCH_HOME), toggleIsHome, RISING);
    attachInterrupt(digitalPinToInterrupt(PIN_ENCO_BTN), toggleEnableEncoder, RISING);
    ConfigureStepperDriver();
    pressureSensor.begin();
}

void loop(){
    stepperPosition = Stepper.getPos();
    anglePosition = analogRead(PIN_RPS_OUT);

    switch (mode) {
        case MANUAL:
            manualControl();
            break;
        case VOLUME_OPEN_LOOP:
            UpdateMotorCurveParameters(respiratoryRate, pathRatio, IERatio);
            currentState = openLoopVolumeControl(currentState);
            break;
        default:
            break;
    };
    updateDisplay(respiratoryRate, pathRatio, IERatio, peakPressure, pressurePlateau, pressurePEEP);

}

void toggleEnableEncoder(){
    void(0);
    //TODO toggle global var, when encoder button is pressed
}

void manualControl(){
    char rxChar = 0;
    if (Serial.available()) {
        rxChar = Serial.read();
        switch (rxChar) {
            case 'F':  // ten steps forward
                //Stepper.move(1, 200 * STEP_DIVIDER);
                moveStepper(10*STEP_DIVIDER, 100, 400, 400, DIR_IN);
                stepCounter = stepCounter + 10;
                break;
            case 'B':  // ten steps back
                moveStepper(10 * STEP_DIVIDER, 100, 400, 400, DIR_EX);
                stepCounter = stepCounter - 10;
                break;
            case 'f':  // single step forward
                moveStepper(1 * STEP_DIVIDER, 100, 400, 400, DIR_IN);
                stepCounter = stepCounter + 1;
                break;
            case 'b':  // single step back
                moveStepper(1 * STEP_DIVIDER, 100, 400, 400, DIR_EX);
                stepCounter = stepCounter - 1;
                break;
            case 's':  // save end position and go home
                if (stepCounter>0){
                    stepsFullRange = stepCounter;
                    moveStepper(stepsFullRange * STEP_DIVIDER, 100, 400, 400, DIR_EX);
                }
                break;
            case '0': // save home position
                stepCounter = 0;
                break;
            case 'x':
                Stepper.hardStop();
                break;

            default:
                Serial.println(".");
                break;
        }
    }
}

PumpingState openLoopVolumeControl(PumpingState state)
{
    switch(state)
    {
        case STARTUP:
            if (isHome){
                state = START_IN;
            }
            else {
                // try to find home in expiration direction
                moveStepper(STEPS_EX_HOMING*STEP_DIVIDER, SPEED_EX, ACC_EX, DEC_EX, DIR_EX);
                state = HOMING_EX;
            }
            break;

        case HOMING_EX:
            if (isHome) {
                // motor is at home position, start cycle
                state = START_IN;
            }
            else if (isBusy()){
                // motor still moving, continue homing
                state = HOMING_EX;
            }
            // TODO: Something goes wrong...... stop and go to error!!!
            else{
                // motor couldn't find home position in EX direction
                // try in IN direction
                moveStepper((STEPS_IN_HOMING+STEPS_EX_HOMING)*STEP_DIVIDER, SPEED_EX, ACC_EX, DEC_EX, DIR_IN);
                state = HOMING_IN;
            }
            break;

        case START_IN:
            UpdateMotorCurveParameters( respiratoryRate, pathRatio, IERatio);
            moveStepper(stepsInterval, speedIn, ACC_IN, DEC_IN, DIR_IN);
            timerStartMovingIn = millis();
            state = MOVING_IN;
            break;

        case MOVING_IN:
            // save pressure if new maximum
            if (pressure > maxPressure){
                maxPressure = pressure;
            }
            if (millis() - timerStartMovingIn < timeIn*1000) {
                //TODO
                (void)0;
            }
            else if (isBusy()) {
                // time is up, but motor still busy
                //TODO
                (void)0;
            }
            state = HOLDING_IN;
            timerStartHoldingIn = millis();
            break;

        case HOLDING_IN:
            // pressure might rise to peak just after compressing phase
            if (pressure > maxPressure){
                maxPressure = pressure;
            }

            if ((millis() - timerStartHoldingIn) < TIME_HOLD_PLATEAU*1000) {
                state=HOLDING_IN;

            }
            else {
                peakPressure = maxPressure;
                state = START_EX;
                timerStartMovingEx = millis();
            }
            break;

        case START_EX:
            // take pressure just before moving out as plateau pressure
            pressurePlateau = pressure;
            // move out
            moveStepper(stepsInterval, SPEED_EX, ACC_EX, DEC_EX, DIR_EX);
            timerStartMovingEx = millis();
            state = MOVING_EX;
            break;

        case MOVING_EX:
            if (isHome){
                Stepper.hardStop();
                if (temperatureInflux<TEMPERATURE_INFLUX_THRESHOLD){
                    //startInfluxHeating();
                    //TODO
                    (void)0;
                state = HOLDING_EX;
            }
            else if ((millis() - timerStartMovingEx) < timeEx*1000) {
                state=MOVING_EX;
            }
            else if (isBusy()) {
                state=MOVING_EX;
                // time is up, but motor still busy
            }
            else {
                // moving out is finished but position not reached
                Stepper.move(DIR_EX, 10);
                state = MOVING_EX;
                }
            }
            break;

        case HOLDING_EX:
            if ((millis() - timerStartMovingEx) < timeEx*1000) {
                // record PEEP pressure
                pressurePEEP = pressure;
            }
            else {
                // time is up, stop heating, move in
                state = START_IN;
                stopInfluxHeating();
            }
            break;

        case IDLE:
            break;


        default:
            break;
    }
    return(state);
}

int votePosition(int stepper, int angle, Sensor::SensorState stepperState, Sensor::SensorState angleState, int tolerance=1*STEP_DIVIDER){
    // TODO handle deviating position readings
    if ((stepper-angle) < tolerance){
        return stepper;
    }
    else if (angleState==Sensor::OK){
        return angle;
    }
    else {
        return stepper;
    }
}

float motorPositionToVolume(uint16_t position){
    // TODO LUT or linear scaling to convert Stepper position to volume
    return position;
}

bool tripleVoteHome(bool optical, bool angle, bool stepper, bool &isHome){
    // optical=notHome more reliable than optical=isHome
    uint8_t state = ((optical<<2)+(angle<<1)+stepper);
    switch (state) {
        case 0B000:
            // not Home
            break;
        case 0B001:
            // TODO optical: not home, angle: not home, stepper: home
            // probable causes: step loss moving in
            // actions: set isHome=false,
            break;
        case 0B010:
            // TODO optical: not home, angle: home, stepper: not home
            // probable causes: angle sensor broken/disconnected, angle sensor wrong calibration
            // actions: set isHome=false
            break;
        case 0B100:
            // TODO optical: home, angle: not home, stepper: not home
            // probable causes: optical sensor faulty, e.g. LED broken
            // actions: set isHome=false
            break;
        case 0B011:
            // TODO optical: not home, angle: home, stepper: home
            // probable causes: optical sensor faulty, e.g. LED broken
            // actions: set isHome=true
            break;
        case 0B110:
            // TODO optical: home, angle: home, stepper: not home
            // probable causes: step loss moving out
            // actions: set isHome=true, null Stepper
            Stepper.hardStop();
            Stepper.resetPos();
            break;
        case 0B101:
            // TODO optical: home, angle: not home, stepper: home
            // probable causes: angle sensor broken/disconnected, angle sensor wrong calibration
            // actions: set isHome=true, null angle
            break;
        case 0B111:
            // TODO optical: home, angle: home, stepper: home
            // actions: set isHome=true
        default:
            break;
    }
    return (optical&&angle)||(angle&&stepper)||(stepper&optical);
}

//! \brief interrupt handling routine, stop the motor by rising edge on the
//! light barrier
//!
void toggleIsHome(){
    Stepper.hardStop();
}

void stopInfluxHeating(){
    // TODO
}

void startInfluxHeating(){
    // TODO
}

void updateDisplay(float respiratoryRate, float pathRatio, float IERatio,
                   float pressurePeak, float pressurePlateau, float pressurePEEP){
    delay(10);
}



bool getStatusFlag(int r, int n){
    int flag = ((r >> (n-1)) & 0x01);
    return(flag);
}

//! \brief move the step motor using the given parameters.
//!
//! The motor driver will accelerate with a constant acceleration
//! until the maximal speed is achieved. For deceleration the inverse
//! process is used.
//!
//! \param steps : number of steps to move
//! \param speed : maximal speed in steps/s
//! \param acc : acceleration in steps/s^2
//! \param acc : deceleration in steps/s^2
//! \param dir : indicate the rotate direction
void moveStepper(int steps, int speed, int acc, int dec, int dir) {
    if (dir==DIR_EX){
        Stepper.setRunKVAL(RUN_KVAL_EX);
        Stepper.setAccKVAL(ACC_KVAL_EX);
        Stepper.setDecKVAL(DEC_KVAL_EX);
        Stepper.setHoldKVAL(HOLD_KVAL_EX);
    }
    else {
        Stepper.setRunKVAL(RUN_KVAL_IN);
        Stepper.setAccKVAL(ACC_KVAL_IN);
        Stepper.setDecKVAL(DEC_KVAL_IN);
        Stepper.setHoldKVAL(HOLD_KVAL_IN);
    }
    Stepper.setMaxSpeed(speed);
    Stepper.setAcc(acc);
    Stepper.setDec(dec);
    Stepper.move(dir, steps);
}

void readPotis(){/*
    respiratoryRate = ((float) analogRead(PIN_POTI_RR)) / 1023 * 35 + 5;
    IERatio = ((float) analogRead(PIN_POTI_IE)) / 1023 * 0.9 + 0.2; //
    pathRatio = ((float) analogRead(PIN_POTI_TV)) / 1023;
    */
void(0); //TODO
}

void PrintMotorCurveParameters(){
    Serial.print("RR= ");Serial.println(respiratoryRate);
    Serial.print("TV= ");Serial.println(pathRatio);
    Serial.print("IE= ");Serial.println(IERatio);
    Serial.print("speedIn: "); Serial.print(speedIn);
    Serial.print("timeEx: "); Serial.print(timeEx);
    Serial.print("steps "); Serial.println(stepsInterval);
    Serial.print("\t stepsInterval: "); Serial.println(stepsInterval);
}
void printUserValues(){
    Serial.print("RR= \t");Serial.print(respiratoryRate);
    Serial.print("\tTV= \t");Serial.print(pathRatio);
    Serial.print("\tIE= \t1:");Serial.println(1./IERatio);
}


//! take user-set parameters and calculate the parameters for the motor curve,
//!
//! \param respiratoryRate - respiratory rate in breaths per minute
//! \param pathRatio - float between 0-1, setting the portion of the total path that will be driven
//! \param IERatio - (inhalation time)/(exhalation time)
void UpdateMotorCurveParameters(float respiratoryRate, float pathRatio, float IERatio) {
    float t_total = 60 / respiratoryRate;
    timeEx = t_total / (IERatio + 1);
    timeIn = t_total - timeEx ;
    int steps = stepsFullRange * pathRatio;

    float discriminant = sq(timeIn) - 2 * (1. / ACC_IN + 1. / DEC_IN) * steps;
    if (discriminant > 0) {
        speedIn = (-timeIn + sqrtf(discriminant)) / -(1. / ACC_IN + 1. / DEC_IN);
        stepsInterval = steps * STEP_DIVIDER;
    }
    stepsInterval = steps * STEP_DIVIDER;
}


//! Read the motor driver status. If the motor is moving
//! the isBusy function return true.
//!
//! \return true if the motor is moving
bool isBusy()
{
    byte status=0;
    bool busy=0;
    status = Stepper.getStatus();
    busy = !((status>>1)&0x01);
    return(busy);
}

// Test Serial chain: shift out ox55 pattern and read back : should be 2 for  X-NUCLEO-IHM02A1 BOARD
// Important: L6470 Serial SPI only works if Vcc (Chip voltage) AND Vss (motor Voltage) is applied!
byte spi_test()
{
    byte val,t,u=0;
    digitalWrite(A2, LOW);
    SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE3));
    Serial.print("\n SPI test: ");
    for(t=0x55;t<0x6D;++t)
    {
        val=SPI.transfer(t);
        Serial.print(" Tx:"); Serial.print(t,HEX);Serial.print("/Rx:"); Serial.print(val,HEX);
        if (val==0x55) {u=t-0x55;t=0x6D;}
        delay(100);
    }
    Serial.print("\n Chain="); Serial.println(u);Serial.print("\n");
    return(u);
}

//  For ease of reading, we're just going to configure all the boards to the same settings.
void ConfigureStepperDriver()
{
    Serial.println("Initialize stepper...");

    // Before we do anything, we need to tell each device which SPI port we're using.
    Stepper.SPIPortConnect(&SPI);

    // reset device //
    Stepper.resetDev();

    Stepper.configSyncPin(BUSY_PIN, 0); // use SYNC/nBUSY pin as nBUSY,
    // thus syncSteps (2nd paramater) does nothing

    Stepper.configStepMode(STEP_FS_128); // 1/128 microstepping, full steps = STEP_FS,
    // options: 1, 1/2, 1/4, 1/8, 1/16, 1/32, 1/64, 1/128

    Stepper.setMaxSpeed(100); // max speed in units of full steps/s
    Stepper.setFullSpeed(1000); // full steps/s threshold for disabling microstepping
    Stepper.setAcc(200); // full steps/s^2 acceleration
    Stepper.setDec(200); // full steps/s^2 deceleration

    Stepper.setSlewRate(SR_520V_us); // faster may give more torque (but also EM noise),
    // options are: 114, 220, 400, 520, 790, 980(V/us)

    Stepper.setOCThreshold(8); // over-current threshold for the 2.8A NEMA23 motor
    // used in testing. If your motor stops working for
    // no apparent reason, it's probably this. Start low
    // and increase until it doesn't trip, then maybe
    // add one to avoid misfires. Can prevent catastrophic
    // failures caused by shorts
    Stepper.setOCShutdown(OC_SD_ENABLE); // shutdown motor bridge on over-current event
    // to protect against permanent damage

    Stepper.setPWMFreq(PWM_DIV_1, PWM_MUL_0_75); // 16MHz*0.75/(512*1) = 23.4375kHz
    // power is supplied to stepper phases as a sin wave,
    // frequency is set by two PWM modulators,
    // Fpwm = Fosc*m/(512*N), N and m are set by DIV and MUL,
    // options: DIV: 1, 2, 3, 4, 5, 6, 7,
    // MUL: 0.625, 0.75, 0.875, 1, 1.25, 1.5, 1.75, 2

    Stepper.setVoltageComp(VS_COMP_DISABLE); // no compensation for variation in Vs as
    // ADC voltage divider is not populated

    Stepper.setSwitchMode(SW_USER); // switch doesn't trigger stop, status can be read.
    // SW_HARD_STOP: TP1 causes hard stop on connection
    // to GND, you get stuck on switch after homing

    Stepper.setOscMode(INT_16MHZ); // 16MHz internal oscillator as clock source

    // KVAL registers set the power to the motor by adjusting the PWM duty cycle,
    // use a value between 0-255 where 0 = no power, 255 = full power.
    // Start low and monitor the motor temperature until you find a safe balance
    // between power and temperature. Only use what you need
    Stepper.setRunKVAL(64);
    Stepper.setAccKVAL(64);
    Stepper.setDecKVAL(64);
    Stepper.setHoldKVAL(32);

    Stepper.setParam(ALARM_EN, 0x8F); // disable ADC UVLO (divider not populated),
    // disable stall detection (not configured),
    // disable switch (not using as hard stop)

    Stepper.getStatus(); // clears error flags

    Serial.println(F("Initialisation complete"));


}

int readStatusRegister(){
    int paramValue;
    paramValue = Stepper.getStatus();
    return(paramValue);
}
