//
// Created by david on 04.04.20.
//
#include <Arduino.h> // main arduino library
#include <SparkFunAutoDriver.h> // stepper driver library
#include <SPI.h> // arduino spi library
#include <EEPROM.h> // eeprom library
#include <avr/wdt.h> // watchdog library

// Need this line for some reason?!
AutoDriver Stepperdummy(1, A2, 4);  // Nr, CS, Reset => 0 , D16/A2 (PA4), D4 (PB5) for IHM02A1 board
AutoDriver Stepper(0, A2, 4);  // Nr, CS, Reset => 0 , D16/A2 (PA4), D4 (PB5) for IHM02A1 board
AutoDriver *boardIndex ;

/* ***********************
 * Constant definitions
 * ***********************
 */
// Pin definitions
int const POTI_PIN_RR = A0;
int const POTI_PIN_TV = A1;
int const POTI_PIN_IE = A3;

int const MANUAL_CYCLE_SWITCH_PIN = 8;

// Clinical settings
int const PRESSURE_MAX = 0; // mm H20
int const PEEP_PRESSURE = 0; // mm H2O
// hold time after inspiratory phase
float const TIME_HOLD_PLATEAU = 0.200; // seconds

// Motor settings
// motor curve for volume control operation

float const ACC_IN = 300; // steps/s/s
float const DEC_IN = 300; // steps/s/s
float const ACC_EX = 300; // steps/s/s
float const SPEED_EX = 100; //steps/s
float const DEC_EX = 300; // steps/s/s
bool const IN_DIR = 1;
bool const EX_DIR = abs(IN_DIR-1);
int const STEP_DIVIDER_REGISTER = STEP_FS_128;
int const STEP_DIVIDER = 128;

// state flags

enum PumpingState {START_IN, MOVING_IN, HOLDING_IN, START_EX, MOVING_EX, HOLDING_EX, IDLE};
enum UserSetState {READ_POTIS,};

/* *****************
 * Function declarations
 * ********************
 */

byte spi_test();
void ConfigureStepperDriver();
int getBoardStatus();
void UpdateMotorCurveParameters(float respiratoryRate, float pathRatio, float IERatio);
void PrintMotorCurveParameters();
bool isBusy();

void moveStepperIn();
void moveStepperEx();
uint8_t runPumpingStateMachine( uint8_t state );
int motorStatusRegister;
void writeToLCD(float respiratoryRate, float tidalVolume, float ratioInEx, float PEEP, float peak, float plat);
void read_potis();
int readStatusRegister();
void readPotis();
void printUserValues();
bool getStatusFlag(int r, int n);
void updateDisplay(float respiratoryRate, float pathRatio, float IERatio,
                   float peakPressure, float plateauPressure, float peePressure);
void readPressureSensors();
void writeToEEPROM();

/* *****************************
 * Global Variables
 * *****************************
 */
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
float plateauPressure=0;
float PEEPressure=0;
uint8_t state = IDLE;

void setup()
{
    Serial.begin(115200);

    pinMode(13, INPUT);     // D3 = SPI SCK, wired-routed to D13(SCK), (D3 setup as input - tri-state)
    pinMode(4, OUTPUT);    // D4 = nReset
    pinMode(MOSI, OUTPUT); // SPI IN
    pinMode(MISO, INPUT);  // SPI EX
    pinMode(13, OUTPUT);   // SCK
    pinMode(A2, OUTPUT);   // CS signal
    pinMode(MANUAL_CYCLE_SWITCH_PIN, INPUT_PULLUP);
    digitalWrite(A2, HIGH);   // nCS set High
    digitalWrite(4, LOW);     // toggle nReset
    digitalWrite(4, HIGH);
    SPI.begin();             // start

    boardIndex = &Stepper;
    ConfigureStepperDriver();
}

void loop(){
    readPotis();
    readPressureSensors();
    updateDisplay(respiratoryRate, pathRatio, IERatio, peakPressure, plateauPressure, PEEPressure);
    motorStatusRegister = readStatusRegister();
    //PrintMotorCurveParameters();
    UpdateMotorCurveParameters(respiratoryRate, pathRatio, IERatio);
    //startCyclingSwitch = digitalRead(MANUAL_CYCLE_SWITCH_PIN);
    state = runPumpingStateMachine(state);
    //Serial.println(state);
}

void updateDisplay(float respiratoryRate, float pathRatio, float IERatio,
                   float peakPressure, float plateauPressure, float peePressure){
    delay(50);
}

void readPressureSensors(){
    peakPressure=0;
    plateauPressure=0;
    PEEPressure=0;
    delay(10);
}

bool getStatusFlag(int r, int n){
    int flag = ((r >> (n-1)) & 0x01);
    return(flag);
}

void moveStepperIn() {
    boardIndex->setMaxSpeed(speedIn);
    boardIndex->setAcc(ACC_IN);
    boardIndex->setDec(DEC_IN);
    Stepper.move(IN_DIR, stepsInterval);
}

void moveStepperEx() {
    boardIndex->setMaxSpeed(SPEED_EX);
    boardIndex->setAcc(ACC_EX);
    boardIndex->setDec(DEC_EX);
    Stepper.move(EX_DIR, stepsInterval);
}

void readPotis(){
    respiratoryRate = ((float) analogRead(POTI_PIN_RR))/1023*35 + 5;
    IERatio = ((float) analogRead(POTI_PIN_IE))/1023*0.9 + 0.2; //
    pathRatio = ((float) analogRead(POTI_PIN_TV))/1023;
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


uint8_t runPumpingStateMachine(uint8_t state)
{
    switch(state)
    {
        case IDLE:
            if (startCyclingSwitch){
                state = START_IN;
            }
            break;

        case START_IN:
            UpdateMotorCurveParameters( respiratoryRate, pathRatio, IERatio);
            moveStepperIn();
            timerStartMovingIn = millis();
            state = MOVING_IN;
            break;

        case MOVING_IN:
            if (millis() - timerStartMovingIn < timeIn*1000) {
                break;
            }
            else if (isBusy()) {
                // time is up, but motor still busy
                break;
            }
            state = HOLDING_IN;
            timerStartHoldingIn = millis();
            break;

        case HOLDING_IN:
            if ((millis() - timerStartHoldingIn) < TIME_HOLD_PLATEAU*1000) {
                break;
            }
            else {
                state = START_EX;
                timerStartMovingEx = millis();
            }
            break;

        case START_EX:
            moveStepperEx();
            timerStartMovingEx = millis();
            state = MOVING_EX;
            break;

        case MOVING_EX:
            if ((millis() - timerStartMovingEx) < timeEx*1000) {
                break;
            }
            else if (isBusy()) {
                // time is up, but motor still busy
                break;
            }
            else {
                state = HOLDING_EX;
                timerStartHoldingEx = millis();
                break;
            }

        case HOLDING_EX:
            if ((millis() - timerStartMovingEx) < timeEx*1000) {
                break;
            } else {
                state = START_IN;
            }
            break;
    }
    return(state);
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
    //Serial.print("timeIn: "); Serial.println(timeIn);
    //Serial.print("timeEx: "); Serial.println(timeEx);

    float discriminant = sq(timeIn) - 2 * (1. / ACC_IN + 1. / DEC_IN) * steps;
    //Serial.print("discriminant: "); Serial.println(discriminant);
    if (discriminant > 0) {
        speedIn = (-timeIn + sqrtf(discriminant)) / -(1. / ACC_IN + 1. / DEC_IN);
        //time_hold_ex =  timeEx - ((1. / ACC_EX + 1. / DEC_EX)*sq(SPEED_EX)/2 + steps)/SPEED_EX;
        stepsInterval = steps * STEP_DIVIDER;
    }
    stepsInterval = steps * STEP_DIVIDER;
}


//! Read the
//! \return
bool isBusy()
{
    byte status=0;
    bool busy=0;
    status = boardIndex->getStatus();
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
    int paramValue;
    Serial.println("Configuring boards...");

    // Before we do anything, we need to tell each device which SPI port we're using.
    boardIndex->SPIPortConnect(&SPI);

    // reset device //
    boardIndex->resetDev();

    // Set the Overcurrent Threshold to 6A(max). The OC detect circuit is quite sensitive; even if the current is only momentarily
    //  exceeded during acceleration or deceleration, the driver will shutdown. This is a per channel value; it's useful to
    //  consider worst case, which is startup.
    boardIndex->setOCThreshold(OCD_TH_6000mA);

    // KVAL is a modifier that sets the effective voltage applied to the motor. KVAL/255 * Vsupply = effective motor voltage.
    //  This lets us hammer the motor harder during some phases  than others, and to use a higher voltage to achieve better
    //  torqure performance even if a motor isn't rated for such a high current.
    // This IHM02A1 BOARD has 12V motors and a 12V supply.
    boardIndex->setRunKVAL(200);  // 220/255 * 12V = 6V
    boardIndex->setAccKVAL(200);  // 220/255 * 12V = 6V
    boardIndex->setDecKVAL(200);  // /255 * 12V = 3V
    boardIndex->setHoldKVAL(150);  // 132/255 * 12V = 1.5V  // low voltage, almost free turn

    // The dSPIN chip supports microstepping for a smoother ride. This function provides an easy front end for changing the microstepping mode.
    // once in full speed, it will step up to half-step
    boardIndex->configStepMode(STEP_DIVIDER_REGISTER); // Full step
}

int readStatusRegister(){
    int paramValue;
    paramValue = boardIndex->getStatus();
    return(paramValue);
}