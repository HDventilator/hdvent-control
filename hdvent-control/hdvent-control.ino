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


/* ***********************
 * Constant definitions
 * ***********************
 */
// Pin definitions
int const POTI_PIN_RR = A0;
int const POTI_PIN_TV = A1;
int const POTI_PIN_IE = A3;
int const PIN_HOME_SENSOR = 9;
int const MANUAL_CYCLE_SWITCH_PIN = 8;

// Clinical settings
int const PRESSURE_MAX = 0; // mm H20
int const PEEP_PRESSURE = 0; // mm H2O
float const TIME_HOLD_PLATEAU = 0.200; // seconds, hold time after inspiratory phase
float const TEMPERATURE_INFLUX_THRESHOLD = 20;// °C, start heating when influx temperature falls below this value

// Motor settings
// motor curve for volume control operation

float const ACC_IN = 300; // steps/s/s
float const DEC_IN = 300; // steps/s/s
float const ACC_EX = 300; // steps/s/s
float const SPEED_EX = 100; //steps/s
float const DEC_EX = 300; // steps/s/s
bool const DIR_IN = 1;
bool const DIR_EX = abs(DIR_IN - 1);
int const STEP_DIVIDER_REGISTER = STEP_FS_128;
int const STEP_DIVIDER = 128;

int const STEPS_EX_HOMING = 80; // steps to move out when trying to find home
int const STEPS_IN_HOMING = 80; // steps to move in when trying to find home

// state flags

enum PumpingState {START_IN, MOVING_IN, HOLDING_IN, START_EX, MOVING_EX, HOLDING_EX, IDLE, STARTUP, HOMING_EX, HOMING_IN};
enum SensorState {SENSOR_DISCONNECTED, SENSOR_CONNECTED, SENSOR_FAULTY, SENSOR_OK};
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
PumpingState runPumpingStateMachine();
int motorStatusRegister;
void writeToLCD(float respiratoryRate, float tidalVolume, float ratioInEx, float PEEP, float peak, float plat);
void read_potis();
int readStatusRegister();
void readPotis();
void printUserValues();
bool getStatusFlag(int r, int n);
void updateDisplay(float respiratoryRate, float pathRatio, float IERatio,
                   float pressurePeak, float plateauPressure, float peePressure);
void readPressureSensor(float &pressure, SensorState &state);
void startInfluxHeating();
void stopInfluxHeating();

bool isHome();
void toggleIsHome();


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
float pressurePlateau=0;
float pressurePEEP=0;
float maxPressure=0;
float pressureBag=0;
SensorState statePressureSensorBag=SENSOR_DISCONNECTED;
float temperatureInflux=0;

PumpingState currentState;

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

    pinMode(PIN_HOME_SENSOR, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PIN_HOME_SENSOR), toggleIsHome, RISING);

    SPI.begin();             // start

    ConfigureStepperDriver();
}

void loop(){
    readPotis();
    readPressureSensor(pressureBag, statePressureSensorBag);
    updateDisplay(respiratoryRate, pathRatio, IERatio, peakPressure, pressurePlateau, pressurePEEP);
    motorStatusRegister = readStatusRegister();
    //PrintMotorCurveParameters();
    UpdateMotorCurveParameters(respiratoryRate, pathRatio, IERatio);
    //startCyclingSwitch = digitalRead(MANUAL_CYCLE_SWITCH_PIN);
    currentState = runPumpingStateMachine();
    //Serial.println(state);
}

PumpingState runPumpingStateMachine()
{

    static PumpingState state = STARTUP;

    switch(state)
    {
        case STARTUP:
            if (isHome()){
                state = START_IN;
            }
            else {
                moveStepper(STEPS_EX_HOMING*STEP_DIVIDER, SPEED_EX, ACC_EX, DEC_EX, DIR_EX);
                state = HOMING_EX;
            }
            break;
        case HOMING_EX:
            if (isHome()) {
                // motor is at home position, stop it and start cycle
                
                state = START_IN;
            }
            else if (isBusy){
                // motor still moving, continue homing
                state = HOMING_EX;
            }
            else{
                // motor couldn't find home position in EX direction
                // try in IN direction
                moveStepper((STEPS_IN_HOMING+STEPS_EX_HOMING)*STEP_DIVIDER, SPEED_EX, ACC_EX, DEC_EX, DIR_IN);
                state = HOMING_IN;

            }
            break;

        case HOMING_IN:
            if (isHome()) {
                // motor is at home position, stop it and start cycle
                Stepper.hardStop();
                state = START_IN;
            }
            else if (isBusy){
                // motor still moving, continue homing
                state = HOMING_IN;
            }
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
            if (pressureBag> maxPressure){
                maxPressure = pressureBag;
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
            if (pressureBag> maxPressure){
                maxPressure = pressureBag;
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
            pressurePlateau = pressureBag;
            // move out
            moveStepper(stepsInterval, SPEED_EX, ACC_EX, DEC_EX, DIR_EX);
            timerStartMovingEx = millis();
            state = MOVING_EX;
            break;

        case MOVING_EX:
            if ((millis() - timerStartMovingEx) < timeEx*1000) {
                state=MOVING_EX;
            }
            else if (isBusy()) {
                state=MOVING_EX;
                // time is up, but motor still busy
            }
            else {
                // moving out is finished
                // start heating and enter hold phase
                state = HOLDING_EX;
                timerStartHoldingEx = millis();
                if (temperatureInflux<TEMPERATURE_INFLUX_THRESHOLD){
                    //startInfluxHeating();
                    //TODO
                    (void)0;
                }

            }
            break;
        case HOLDING_EX:
            if ((millis() - timerStartMovingEx) < timeEx*1000) {
                // record PEEP pressure
                pressurePEEP = pressureBag;
            }
            else {
                // time is up, stop heating, move in
                state = START_IN;
                stopInfluxHeating();
            }
            break;

        default:
            break;
    }


    return(state);
}

//! \brief return true if the motor is at home position. 
//! Read the value of the IO pin connected to the home light barrier sensor
//!
//! \return true if motor is at home position
bool isHome()
{
   uint8_t val;
   val = digitalRead(PIN_HOME_SENSOR);
   
   return val;
}

//! \brief interrupt handling routine, stop the motor by rising edge on the 
//! light barrier
//!
void toggleIsHome(){
    Stepper.hardStop();
}

void updateDisplay(float respiratoryRate, float pathRatio, float IERatio,
                   float pressurePeak, float pressurePlateau, float pressurePEEP){
    delay(10);
}

void readPressureSensor(float &pressure, SensorState &state){
    // TODO read pressure sensor function
    pressure = 0;
    state = SENSOR_DISCONNECTED;
    delay(10);
}

bool getStatusFlag(int r, int n){
    int flag = ((r >> (n-1)) & 0x01);
    return(flag);
}

void moveStepper(int steps, int speed, int acc, int dec, int dir) {
    Stepper.setMaxSpeed(speed);
    Stepper.setAcc(acc);
    Stepper.setDec(dec);
    Stepper.move(dir, steps);
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


//! Read the
//! \return
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
    int paramValue;
    Serial.println("Configuring boards...");

    // Before we do anything, we need to tell each device which SPI port we're using.
    Stepper.SPIPortConnect(&SPI);

    // reset device //
    Stepper.resetDev();

    // Set the Overcurrent Threshold to 6A(max). The OC detect circuit is quite sensitive; even if the current is only momentarily
    //  exceeded during acceleration or deceleration, the driver will shutdown. This is a per channel value; it's useful to
    //  consider worst case, which is startup.
    Stepper.setOCThreshold(OCD_TH_6000mA);

    // KVAL is a modifier that sets the effective voltage applied to the motor. KVAL/255 * Vsupply = effective motor voltage.
    //  This lets us hammer the motor harder during some phases  than others, and to use a higher voltage to achieve better
    //  torque performance even if a motor isn't rated for such a high current.
    // This IHM02A1 BOARD has 12V motors and a 12V supply.
    Stepper.setRunKVAL(200);  // 220/255 * 12V = 6V
    Stepper.setAccKVAL(200);  // 220/255 * 12V = 6V
    Stepper.setDecKVAL(200);  // /255 * 12V = 3V
    Stepper.setHoldKVAL(150);  // 132/255 * 12V = 1.5V  // low voltage, almost free turn

    // The dSPIN chip supports microstepping for a smoother ride. This function provides an easy front end for changing the microstepping mode.
    // once in full speed, it will step up to half-step
    Stepper.configStepMode(STEP_DIVIDER_REGISTER); // Full step
}

int readStatusRegister(){
    int paramValue;
    paramValue = Stepper.getStatus();
    return(paramValue);
}
