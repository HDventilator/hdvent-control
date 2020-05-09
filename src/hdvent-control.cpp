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
#include "Diagnostic_Parameter.h"
#include "Ventilation_Modes.h"
#include "Stopwatch.h"
#include "Trigger.h"
#include "PID_v1.h"
#include "Ventilation_Controller.h"
#include <Display.h>
#include <LiquidCrystal.h>
#include <Motor_Settings.h>

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

bool tripleVoteHome(bool optical, bool angle, bool stepper, bool &isHome);
int getPosition(int tolerance);
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
Honeywell_SSC pressureSensor(0x48,0,-150,150,0.1*16383,0.9*16383);
Honeywell_SSC flowSensor(0x68,0,0,4000,-1,1);
Angle_Sensor angleSensor(PIN_RPS_OUT, STEPS_FS_FULL_TURN*STEP_DIVIDER, 360, 0, 1024, 10);

float timeEx=1;
float timeIn=1;
float speedIn = 50; // steps/s

unsigned int stepsFullRange = 300;
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
bool startCyclingSwitch = 0;

VentilationMode mode=VC_CMV ;
VentilationState State = STARTUP;

float oldPressure;
unsigned long cycleTime =0;


//manual control
int stepCounter=0;
int stepperPosition=0;
float anglePosition=0; // TODO map angle position on motor steps
Sensor::SensorState stepperPositionState = Sensor::OK;
Sensor::SensorState anglePositionState = Sensor::OK;
float motorSpeed;
PID pressureControlPID();

VentilationController controller(VC_CMV, diagnosticParameters.airwayPressure,diagnosticParameters.flow);

LiquidCrystal lcd(12, 11, 10, 9, 8, 7);

void setup()
{
    Serial.begin(115200);
/*
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

    pinMode(PIN_OPTICAL_SWITCH_HOME, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PIN_OPTICAL_SWITCH_HOME), toggleIsHome, RISING);
    attachInterrupt(digitalPinToInterrupt(PIN_ENCO_BTN), toggleEnableEncoder, RISING);*/
    ConfigureStepperDriver();
    pressureSensor.begin();

    // Start SPI
    SPI.begin();
    SPI.setDataMode(SPI_MODE3);

    allUserParams[(int) UP::RESPIRATORY_RATE] = User_Parameter(15, 5, 35, "freq"); //  breaths per minute
    allUserParams[(int) UP::TIDAL_VOLUME] = User_Parameter(250, 0, 650, "VTid"); // milliliters
    allUserParams[(int) UP::T_IN] = User_Parameter(2, 0.6, 4,"T_in"); // Inspiration time
    allUserParams[(int) UP::INSPIRATORY_PRESSURE] = User_Parameter(20, 5, 50, "P_aw"); //  millibar
    allUserParams[(int) UP::FLOW] = User_Parameter(20, 5, 50,"Flow"); //  milliliters per second
    allUserParams[(int) UP::D_PRESSURE_SUPP] = User_Parameter(20, 5, 50, "Psup"); //  millibar
    allUserParams[(int) UP::PRESSURE_TRIGGER_THRESHOLD] = User_Parameter(5, 5, 50, "Pthr"); //  millibar per second
    allUserParams[(int) UP::FLOW_TRIGGER_THRESHOLD] = User_Parameter(20, 5, 50, "Fthr"); //  milliliters per second

    lcd.begin(20,4);
}


void loop(){
    cycleTime = stopwatch.mainLoop.getElapsedTime();
    stopwatch.mainLoop.start();

    diagnosticParameters.airwayPressure.setValue(pressureSensor.getData().pressure);
    diagnosticParameters.flow.setValue(flowSensor.getData().pressure);

    float pressureChange = (pressureSensor.getData().pressure - oldPressure)/cycleTime*1000;
    diagnosticParameters.pressureChange.setValue(pressureChange);
    oldPressure = pressureSensor.getData().pressure;

}

VentilationState ventilationStateMachine( VentilationState state){
    switch (state){
        case HOLDING_EX:
            if (controller.inspirationTrigger()) {
                state = MOVING_IN;
            }
            break;

        case START_IN:
            stopwatch.inspiration.start();
            break;

        case MOVING_IN:
            // set stepper speed to calculated value
            Stepper.run(DIR_IN, controller.calcSpeed());

            // start expiration on trigger
            if (controller.expirationTrigger()) {
            state = END_IN;
            }
            break;

        case END_IN:
            Stepper.hardStop();
            stopwatch.holdingIn.start();
            state=HOLDING_IN;

        case HOLDING_IN:
            state = MOVING_EX;

            if (stopwatch.holdingIn.getElapsedTime() > TIME_HOLD_PLATEAU){
                state = START_EX;
            }
            break;

        case START_EX:
            stopwatch.expiration.start();
            state=MOVING_EX;
            break;

        case MOVING_EX:
            // record PEEP pressure shortly after start of expiration phase
            if (stopwatch.expiration.getElapsedTime() > 50){
                diagnosticParameters.peep.setValue(pressureSensor.getData().pressure);
            }
            state = END_EX;
            break;

        case END_EX:
            state = HOLDING_EX;
    }
}

bool Triggers::respiratoryRate() {
    return (float)stopwatch.inspiration.getElapsedTime() > 1 / allUserParams[(int) UP::RESPIRATORY_RATE].getValue();
}

bool Triggers::inspirationTime() {
    return (float)stopwatch.inspiration.getElapsedTime() > allUserParams[(int)UP::T_IN].getValue();
}

bool Triggers::pressureDrop() {
    return (- diagnosticParameters.pressureChange.getValue() > allUserParams[(int)UP::PRESSURE_TRIGGER_THRESHOLD].getValue());
}

bool Triggers::flowIncrease() {
    return diagnosticParameters.flow.getValue() > allUserParams[(int)UP::FLOW_TRIGGER_THRESHOLD].getValue();
}

void toggleEnableEncoder(){
    void(0);
    //TODO toggle global var, when encoder button is pressed
}


int getPosition(int tolerance=1*STEP_DIVIDER){
    int stepper = Stepper.getPos();
    int angle = angleSensor.readSensor();

    // TODO handle deviating position readings
    if ((stepper-angle) < tolerance){
        return stepper;
    }
    else if (angleSensor.getState()==Sensor::OK){
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

    Stepper.setMaxSpeed(500); // max speed in units of full steps/s
    Stepper.setFullSpeed(1000); // full steps/s threshold for disabling microstepping
    Stepper.setAcc(400); // full steps/s^2 acceleration
    Stepper.setDec(400); // full steps/s^2 deceleration

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
    Stepper.setRunKVAL(180);
    Stepper.setAccKVAL(180);
    Stepper.setDecKVAL(180);
    Stepper.setHoldKVAL(62);

    Stepper.setParam(ALARM_EN, 0x8F); // disable ADC UVLO (divider not populated),
    // disable stall detection (not configured),
    // disable switch (not using as hard stop)

    Stepper.getStatus(); // clears error flags

    Serial.println(F("Initialisation complete"));

}