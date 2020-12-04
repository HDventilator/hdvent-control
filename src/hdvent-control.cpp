//
// Created by david on 04.04.20.
//
#include <hdvent-control.h>


void buttonEncoderInterruptRoutine(){
    encoder.shortPressDetected = digitalRead(PIN_ENCO_BTN);
}

void setup()
{
    Serial.begin(115200);

    // Prepare pins
    pinMode(PIN_ENCO_A,INPUT);
    pinMode(PIN_ENCO_B,INPUT);
    pinMode(PIN_ENCO_BTN,INPUT);


    pinMode(nSTBY_nRESET_PIN, OUTPUT);
    pinMode(nCS_PIN, OUTPUT);


    attachInterrupt(digitalPinToInterrupt(PIN_ENCO_BTN), buttonEncoderInterruptRoutine, CHANGE);
// LED Pins

    pinMode(PIN_LED_ORANGE, OUTPUT);
    pinMode(PIN_LED_GREEN, OUTPUT);

    // SPI pins
    pinMode(MOSI, OUTPUT);
    pinMode(MISO, OUTPUT);
    pinMode(SCK, OUTPUT);

    //
    pinMode(10, INPUT);
    pinMode(11, INPUT);
    pinMode(12, INPUT);
    pinMode(13, INPUT);

    // Reset powerSTEP and set CS
    digitalWrite(nSTBY_nRESET_PIN, HIGH);
    digitalWrite(nSTBY_nRESET_PIN, LOW);
    digitalWrite(nSTBY_nRESET_PIN, HIGH);
    digitalWrite(nCS_PIN, HIGH);



    pinMode(PIN_LCD_RW, INPUT);

    pinMode(PIN_OPTICAL_SWITCH_HOME, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PIN_OPTICAL_SWITCH_HOME), toggleIsHome, RISING);

    // Start SPI
    SPI.begin();
    SPI.setDataMode(SPI_MODE3);




    // fastest time to cover full range with chosen motor paramaters, accelerating and then immediately breaking
    float t_in_min = sqrtf(2*(1/(float)ACC_IN+ 1/(float)DEC_IN)*(float) STEPS_FULL_RANGE);
    float t_ex_min = sqrtf(2*(1/(float)ACC_EX+ 1/(float)DEC_EX)*(float) STEPS_FULL_RANGE);

    // these define also an upper boundary for the respiratory rate:
    float respiratory_rate = 6;
    float respiratory_rate_max = 60/(t_in_min+t_ex_min);
    float respiratory_rate_min = 5;
    // slowest in movement is bounded by fastest possible out movement and fastest respiratory rate
    float t_in_max = 60/respiratory_rate - t_ex_min;


    allUserParams[(int) UP::RESPIRATORY_RATE] = User_Parameter(respiratory_rate, respiratory_rate_min, respiratory_rate_max, "freq", 0, 1024, true); //  breaths per minute
    allUserParams[(int) UP::T_IN] = User_Parameter(4, t_in_min, t_in_max,"T_in", 0, 1024, true); // Inspiration time
    allUserParams[(int) UP::TIDAL_VOLUME] = User_Parameter(250, 0, 650, "VTid", 0, 1024, true); // milliliters
    allUserParams[(int) UP::INSPIRATORY_PRESSURE] = User_Parameter(20, 5, 50, "P_aw", 0, 1024, true); //  millibar
    allUserParams[(int) UP::FLOW] = User_Parameter(20, 5, 50,"Flow", 0, 1024, true); //  milliliters per second
    allUserParams[(int) UP::D_PRESSURE_SUPP] = User_Parameter(20, 5, 50, "Psup", 0, 1024, true); //  millibar
    allUserParams[(int) UP::PRESSURE_TRIGGER_THRESHOLD] = User_Parameter(5, 5, 50, "Pthr", 0, 1024, true); //  millibar per second
    allUserParams[(int) UP::FLOW_TRIGGER_THRESHOLD] = User_Parameter(20, 5, 50, "Fthr", 0, 1024, true); //  milliliters per second
    allUserParams[(int) UP::ANGLE] = User_Parameter(0, 0, 1, "angl", 0, 1024, true); //  milliliters per second
    allUserParams[(int) UP::COMPRESSED_VOLUME_RATIO] = User_Parameter(100, 0, 100, "Volu", 0, 1024, true); //  milliliters per second


    ConfigureStepperDriver();
    pressureSensor.begin();

    lcd.begin(20,4);
    lcd.flush();
    lcd.home();

    // adjust sensor state if connected/disconnected
    opticalHomeSensor.setState(Sensor::OK);
    angleSensor.setState(Sensor::DISCONNECTED);

    // stepper cannot know absolute position on startup
    stepperMonitor.setState(Sensor::FAULTY);

    // initial state for state machine
    ventilationState = IDLE;
    //flowSensor.begin();
    Wire.begin();
    //Serial3.begin(115200);
    cobsSerial.setStream(&Serial3);
    //cobsSerial.begin(115200);
    Serial3.begin(115200);

    stopwatch.sinceIdle.start();
    allUserParams.update(mode.parameters, mode.nParams);
    display.printStaticText();
    for (Diagnostic_Parameter &param : diagnosticParameters.arr){
        param.setValue(10);
    }



}


// standard Arduino setup()
void scan_i2c()
{
    Serial.println("\nTCAScanner ready!");

    for (uint8_t t=0; t<8; t++) {
        tcaselect(t);
        Serial.print("TCA Port #"); Serial.println(t);

        for (uint8_t addr = 0; addr<=127; addr++) {
            if (addr == TCAADDR) continue;

            uint8_t data;
            if (! twi_writeTo(addr, &data, 0, 1, 1)) {
                Serial.print("Found I2C 0x");  Serial.println(addr,HEX);
            }
        }
    }
    Serial.println("\ndone");
}
bool buzzerState;

void checkBuzzer(){
    if (alarmOverwrite.getSingleDebouncedPress()){
        buzzerState = !buzzerState;
        if (buzzerState){
            buzzer.saveTurnOn();
        }
        else {
            buzzer.turnOff();
        }
    }
}

void loop(){
    digitalWrite(PIN_LED_GREEN,HIGH);
    digitalWrite(PIN_LED_ORANGE,HIGH);
    //scan_i2c();
    writeDiagnosticParameters();

    // record cycle time
    cycleTime = stopwatch.mainLoop.getElapsedTime();
    stopwatch.mainLoop.start();

    buzzer.service();
    rescaleParameterLimits();

/*
    if (diagnosticParameters.s.volume.getPersistentState() != Diagnostic_Parameter::OK) {
        buzzer.saveTurnOn();
    }
    if (alarmOverwrite.getPush() && (diagnosticParameters.s.volume.getState())){
        buzzer.turnOff();
        diagnosticParameters.s.volume.resetPersistentAlarm();
        Serial.println(diagnosticParameters.s.volume.getState());
    }
*/

    checkAlarms();
    readSensors();
    checkHomeSensors(isHome);

    runVentilation = digitalRead(PIN_SD_VENTI);

    ventilationStateMachine(ventilationState);
    display.update(confirmButton.getSingleDebouncedPress(), cancelButton.getSingleDebouncedPress(), encoderButton.getSingleDebouncedPress(), encoder.getDelta());

    serialDebug();
}
void debugMotor(){
    if (!isHome){
        Serial.println("not Home");
        if (!Stepper.busyCheck()) {
            moveStepper(200 * STEP_DIVIDER, 800, DIR_EX);
        }
        //Stepper.move(DIR_EX, 40*STEP_DIVIDER);
        //Stepper.hardStop();
    }
}
void checkAlarms() {
    //Serial.println(diagnosticParameters.s.volume.getState());

    for (Diagnostic_Parameter &param : diagnosticParameters.arr){

        if (param.getState() != Diagnostic_Parameter::OK) {
            buzzer.saveTurnOn();
        } else if (alarmOverwrite.getSingleDebouncedPress() && (param.getState() == Diagnostic_Parameter::OK)) {
            buzzer.turnOff();
            param.resetPersistentAlarm();
        }
}}

void writeDiagnosticParameters(){
    serialWritePackage(&cobsSerial, diagnosticParameters.s.flow.getPackageStruct());
    serialWritePackage(&cobsSerial, diagnosticParameters.s.airwayPressure.getPackageStruct());
    serialWritePackage(&cobsSerial, diagnosticParameters.s.volume.getPackageStruct());
    serialWritePackage(&cobsSerial, diagnosticParameters.s.tidalVolume.getPackageStruct());
    serialWritePackage(&cobsSerial, diagnosticParameters.s.peep.getPackageStruct());
}

void runMachineDiagnostics(){
    // program cycle time
    machineDiagnostics.cycle_time.setValue(cycleTime);
    serialWritePackage(&cobsSerial, machineDiagnostics.cycle_time.getPackageStruct());
    delay(1);
    // ventilation state
    machineDiagnostics.ventilationState.setValue((int)ventilationState);
    serialWritePackage(&cobsSerial, machineDiagnostics.ventilationState.getPackageStruct());

}

void serialDebug(){
    /*
int8_t delta = encoder.getDelta();
if (delta) {
    Serial.println(delta);
}
 */

    /*
    int i=0;
    for (Diagnostic_Parameter p: diagnosticParameters.arr){
        i++;
        if (i>10){break;}
        Serial.print(p.lcdString);Serial.print("\tLo: ");Serial.print(p.getLoAlarm());
        Serial.print("\tHi: ");Serial.println(p.getHiAlarm());
    }*/
    //Serial.print("nParams:");Serial.println(display.nActiveDiagnosticParameters);


//Serial.println(cycleTime);
    /*for (int i=0; i < (display._mode->nParams); i++){
    lcd.setCursor(1,i);
    lcd.print(display._allUserParameters[(int)display._mode->parameters[i]].lcdString);
}*/
    //display.printStaticText();
    /*
     * if (!digitalRead(PIN_EDIT_MODE)){    Serial.println("Button1 pressed");}
     * if (!digitalRead(PIN_ALARM_MUTE)){    Serial.println("Button2 pressed");}
     * if (!digitalRead(PIN_OPTICAL_SWITCH_END)){    Serial.println("Button3 pressed");}
     */

    //Serial.println(encoder.getPosition());
    /*
    if (encoder.shortPressDetected){
        Serial.println("short Press");
        encoder.shortPressDetected=false;
    }
    if (encoder.wasTurned){
        Serial.println("was turned");
        encoder.wasTurned=false;
    }
     */
    //Serial.println(diagnosticParameters.arr[0].lcdString);
    //Serial.print(digitalRead(PIN_ENCO_B));
    /*
    if (encoder.longPressDetected){
        Serial.println("long Press");
        encoder.longPressDetected=false;
    }
*/
    /*
    if (*display._toggleEditState){
        Serial.println("short Press");
        //*display._toggleEditState=false;
    }
    if (*display._toggleMenuState){
        Serial.println("long Press");
        //*display._toggleMenuState=false;
    }


    display.update();
    Serial.println(display._editState);
*/
    /*Serial.print("T inspiration:   ");Serial.println(allUserParams[(int)UP::T_IN].getValue());
    Serial.print("Volume:   ");Serial.println(allUserParams[(int)UP::COMPRESSED_VOLUME_RATIO].getValue());
    Serial.print("Frequency:   ");Serial.println(allUserParams[(int)UP::RESPIRATORY_RATE].getValue());*/
    Serial.print("T inspiration:   ");Serial.println(allUserParams[(int)UP::T_IN].isGettingEdited);
    Serial.print("Volume:   ");Serial.println(allUserParams[(int)UP::COMPRESSED_VOLUME_RATIO].isGettingEdited);
    Serial.print("Frequency:   ");Serial.println(allUserParams[(int)UP::RESPIRATORY_RATE].isGettingEdited);
   // Serial.println(diagnosticParameters.s.flow.getValue());
    //Serial.print("Stepper pos   ");Serial.println(stepperMonitor.getData().relativePosition);
    //Serial.print("home?   "); Serial.println(isHome);
    //Serial.print("Inspiration time"); Serial.println(stopwatch.inspiration.getElapsedTime());
   // Serial.print("ventilationState:  ");Serial.println(ventilationState);
    //Serial.print("busy?   ");            Serial.println(Stepper.busyCheck());
    //Serial.print("stopwatch inspiration:");Serial.println(stopwatch.inspiration.getElapsedTime());
    //Serial.print("User Input state:");Serial.println(userInput.getInputState());
    //Serial.print("do save?  ");Serial.println(saveUserParams);
    //Serial.print("User Input stopwatch"); Serial.println(userInput._stopwatch.getElapsedTime());
    //Serial.print("StepperState    "); Serial.println(Stepper.getStatus());
    //Serial.print("runVentilation   ");Serial.println(runVentilation);
    //Serial.print("StepperState    "); Serial.println(Stepper.getStatus());
    //Serial.println((int)Stepper.getStatus(), HEX); // print STATUS register
    //Serial.print("opticalHomeSensor:  ");Serial.println(opticalHomeSensor.getData().isBlocked);
    //Serial.print("Button:   "); Serial.println(digitalRead(PIN_ALARM_MUTE));

}
void rescaleParameterLimits(){
    float T = 60/allUserParams[(int)UP::RESPIRATORY_RATE].getDialValue();
    float t_in = allUserParams[(int)UP::T_IN].getDialValue();
    float x = allUserParams[(int)UP::COMPRESSED_VOLUME_RATIO].getDialValue()/100*STEPS_FULL_RANGE*STEP_DIVIDER;
    float a = ACC_EX *STEP_DIVIDER;

    if (allUserParams[(int)UP::RESPIRATORY_RATE].isGettingEdited){
        float f_max = 60/(t_in + sqrtf(4*x/a));
        allUserParams[(int)UP::RESPIRATORY_RATE].setMax(f_max);
    }

    if (allUserParams[(int)UP::T_IN].isGettingEdited){
        float t_in_min = sqrtf(4*x/a);
        float t_in_max = T-sqrtf(4*x/a);
        Serial.println(t_in_max);
        t_in_min = max (0, t_in_min);
        allUserParams[(int)UP::T_IN].setMin(t_in_min);
        allUserParams[(int)UP::T_IN].setMax(t_in_max);
    }

    if (allUserParams[(int)UP::COMPRESSED_VOLUME_RATIO].isGettingEdited){
        float x_max = (T - t_in)*(T - t_in) *a/4;
        x_max = min (x_max, STEPS_FULL_RANGE*STEP_DIVIDER);
        allUserParams[(int)UP::COMPRESSED_VOLUME_RATIO].setMax(x_max*100/STEP_DIVIDER/STEPS_FULL_RANGE);
    }
}

void readUserInput(){

    // dynamically scale allowed range of T_IN when Respiratory Rate Changes
    if (allUserParams[(int)UP::RESPIRATORY_RATE].hasChanged()){
        float rr = allUserParams[(int)UP::RESPIRATORY_RATE].getDialValue();
        float t_in_max = 60/rr - sqrtf(2*(1/ACC_EX+ 1/DEC_EX)* STEPS_FULL_RANGE);
        allUserParams[(int)UP::T_IN].setMax((t_in_max));
    }



    //if (userInput.getInputState() == User_Input::VIEW){
    if (true){
        for ( int i=0;  i<(mode.nParams); i++)
        {
            serialWritePackage(&cobsSerial, controller.userParams[i].getValuePackage());
            serialWritePackage(&cobsSerial, controller.userParams[i].getMinPackage());
            serialWritePackage(&cobsSerial, controller.userParams[i].getMaxPackage());
        }
    }
}

void readSensors(){
    // read pressure Sensors
    pressureSensor.readSensor();
    diagnosticParameters.s.airwayPressure.setValue(pressureSensor.getData().pressure);
    flowSensor.readSensor();

    diagnosticParameters.s.flow.setValue((flowSensor.getData().pressure-PRESSURE_FLOW_CONVERSION_OFFSET) * PRESSURE_FLOW_CONVERSION);

    // integrate flow for volume
    if ((ventilationState == START_IN)||(ventilationState == IDLE)){
        diagnosticParameters.s.volume.setValue(0);
    }
    else {
        float newVolume = diagnosticParameters.s.volume.getValue() + diagnosticParameters.s.flow.getValue()*(float)cycleTime/1000;
        diagnosticParameters.s.volume.setValue(newVolume);
    }

    // calculate tidal volume and minute volume
    if (ventilationState == END_IN) {
        unsigned long timestamp=stopwatch.sinceIdle.getElapsedTime();
        diagnosticParameters.s.tidalVolume.setValue(diagnosticParameters.s.volume.getValue());
        //diagnosticParameters.s.minuteVolume.enqueue(diagnosticParameters.s.volume.getValue()/1000, timestamp);

        if (timestamp>60000){
            timestamp = timestamp-60000;
        }
        else {
            timestamp =0;
        }
        //diagnosticParameters.s.minuteVolume.sumFromTimestamp(timestamp);
        Serial.print("elapsed time:"); Serial.println(timestamp);
    }

    if (ventilationState == HOLDING_IN){
        if (stopwatch.holdingIn.getElapsedTime()>50){
            diagnosticParameters.s.peep.setValue(pressureSensor.getData().pressure);
        }
    }

/*
    float pressureChange = (pressureSensor.getData().pressure - oldPressure)/cycleTime*1000;
    diagnosticParameters.s.pressureChange.setValue(pressureChange);
    oldPressure = pressureSensor.getData().pressure;
*/
    // read position sensors
    stepperMonitor.readSensor();
    angleSensor.readSensor();
    opticalHomeSensor.readSensor();
}

VentilationState ventilationStateMachine( VentilationState &state){
    switch (state){
        case START_HOMING: // executed when transitioning to HOMING_EX
            // stepper driver doesn't know absolute home position
            stepperMonitor.setState(Sensor::FAULTY);

            // start full revolution of stepper in ex direction to find home position
            moveStepper(STEPS_FS_FULL_TURN*STEP_DIVIDER, 200, 400, 400, DIR_EX);

            state = HOMING_EX;
            break;

        case HOMING_EX:

            // home position is recognized
            if (isHome){
                Stepper.hardStop();

                // store home position in stepper driver memory
                Stepper.resetPos();
                state=IDLE;
            }

            // home not found but stepper still moving
            else if (Stepper.busyCheck()){
                state=HOMING_EX;
            }

            // stepper finished full revolution, but home not found
            else {
                // store home position in stepper driver memory and switch sensor state to ok
                Stepper.resetPos();
                stepperMonitor.setState(Sensor::OK);

                // home not found, thus angle sensor and optical sensor are faulty
                angleSensor.setState(Sensor::FAULTY);
                opticalHomeSensor.setState(Sensor::FAULTY);
                state=IDLE;
            }
            break;

        case IDLE: // executed at startup
            stopwatch.sinceIdle.reset();
            if (runVentilation){
                if (isHome){
                    stopwatch.sinceIdle.start();
                    state = START_IN;
                }
                else {
                    state = START_HOMING;
                }
            }
            else{
                state = IDLE;
            }
            break;



        case HOLDING_EX:

            if (controller.inspirationTrigger()) {
                state = START_IN;
            }

            break;

        case START_IN:
            // record time after start of inspiration
            stopwatch.inspiration.start();
            setKVals(DIR_IN);

            // In open loop mode, issue single Motor command to move to specified position
            if (mode.controlMode==ControlMode::VN){
                float steps= allUserParams[(int)UP::COMPRESSED_VOLUME_RATIO].getValue() / 100 * STEPS_FULL_RANGE;
                Serial.print("Move steps:"); Serial.println(steps);
                //steps=40;
                int speed = calculateSpeed(ACC_IN, DEC_IN, allUserParams[(int)UP::T_IN].getValue(), steps);
                Stepper.setMaxSpeed(speed);
                //delay(100);
                //Serial.print("speed:  "); Serial.println(speed);

                //Stepper.move(DIR_IN, 4000);
                moveStepper(steps* STEP_DIVIDER, speed, DIR_IN);
            }
            // start the setpoint generation for the controller
            else{
                controller.startRamp(1000, 100);
            }
            state =MOVING_IN;
            break;

        case MOVING_IN:
            //Serial.print("controlMode: ");Serial.println((int)mode.controlMode);
            if (mode.controlMode!=ControlMode::VN) {
                Serial.println('controlled mode');
                // set stepper speed to calculated value
                Serial.println(controller.calcSpeed());
                Stepper.run(DIR_IN, controller.calcSpeed());
            }
            // start expiration on trigger
            if (controller.expirationTrigger()) {
            state = END_IN;
            }
            break;

        case END_IN:
            Stepper.hardStop();
            stopwatch.holdingIn.start();
            state = HOLDING_IN;

        case HOLDING_IN:

            if (stopwatch.holdingIn.getElapsedTime() > TIME_HOLD_PLATEAU){
                state = START_EX;
            }
            break;

        case START_EX:
            stopwatch.expiration.start();
            setKVals(DIR_EX);
            Stepper.goToDir(DIR_EX, 0);
            state=MOVING_EX;
            break;

        case MOVING_EX:
            // record PEEP pressure shortly after start of expiration phase
            if (stopwatch.expiration.getElapsedTime() > 50){
                diagnosticParameters.s.peep.setValue(pressureSensor.getData().pressure);
            }
            // Stepper has reached home position
            if (isHome) {
                state = END_EX;
            }
            // Stepper not busy anymore but hasn't reached home (step loss)
            else if (!Stepper.busyCheck()){
                state = START_HOMING;
            }

            break;

        case END_EX:
            Stepper.hardStop();
            if (runVentilation){
                state = HOLDING_EX;
            }
            else {
                state = IDLE;
            }
    }
}

bool Triggers::respiratoryRate() {
    return (float)stopwatch.inspiration.getElapsedTime() > 1 / allUserParams[(int) UP::RESPIRATORY_RATE].getValue()*60*1000;
}

bool Triggers::inspirationTime() {
    return (float)stopwatch.inspiration.getElapsedTime() > allUserParams[(int)UP::T_IN].getValue()*1000;
}

bool Triggers::pressureDrop() {
    return (- diagnosticParameters.s.pressureChange.getValue() > allUserParams[(int)UP::PRESSURE_TRIGGER_THRESHOLD].getValue());
}

bool Triggers::flowIncrease() {
    return diagnosticParameters.s.flow.getValue() > allUserParams[(int)UP::FLOW_TRIGGER_THRESHOLD].getValue();
}

bool Triggers::angleReached() {
    return (float) getPosition() > allUserParams[(int)UP::ANGLE].getValue() * STEPS_FULL_TURN * STEP_DIVIDER;
}



int getPosition(int tolerance){
    int stepper = Stepper.getPos();
    int angle = angleSensor.getData().relativePosition;

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

//! Use the optical switch the angle Sensor and the stepper driver data to determine if the stepper is at home position
//! \param isHome
void checkHomeSensors(bool &isHome) {
    bool optical = opticalHomeSensor.getState()==Sensor::OK;
    bool angle = angleSensor.getState()==Sensor::OK;
    bool stepper = stepperMonitor.getState()==Sensor::OK;
    uint8_t state = ((optical<<2)+(angle<<1)+stepper);
    switch (state) {
        // 0: sensor not ok
        // 1: sensor ok
        case 0B000:
            // all faulty: use Stepper data
            isHome = stepperMonitor.getData().isHome;
            break;
        case 0B001:
            // only stepper good: use stepper data
            isHome = stepperMonitor.getData().isHome;
            break;
        case 0B010:
            // only angle good: use angle data
            isHome = angleSensor.getData().isHome;
            break;
        case 0B100:
            // only optical good: use optical data
            isHome = opticalHomeSensor.getData().isBlocked;
            break;
        case 0B011:
            // both angle and stepper good: use angle
            isHome = angleSensor.getData().isHome;
            break;
        case 0B110:
            // both optical and angle good: use optical
            isHome = opticalHomeSensor.getData().isBlocked;
            break;
        case 0B101:
            // both optical and stepper good: use optical
            isHome = opticalHomeSensor.getData().isBlocked;
            break;
        case 0B111:
            // all good: do triple voting
            isHome = tripleVoteHome(opticalHomeSensor.getData().isBlocked,angleSensor.getData().isHome, stepperMonitor.getData().isHome);
        default:
            break;
    }
}


//! Voting logic for three boolean inputs
//! \param optical
//! \param angle
//! \param stepper
//! \return
bool tripleVoteHome(bool optical, bool angle, bool stepper) {
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
    //Stepper.hardStop();
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

void moveStepper(int steps,  int speed, int dir) {
    if (dir==DIR_EX){
        Stepper.setRunKVAL(RUN_KVAL_EX);
        Stepper.setAccKVAL(ACC_KVAL_EX);
        Stepper.setDecKVAL(DEC_KVAL_EX);
        Stepper.setHoldKVAL(HOLD_KVAL_EX);
        Stepper.setMaxSpeed(speed);
        Stepper.setAcc(ACC_EX);
        Stepper.setDec(DEC_EX);
    }
    else {
        Stepper.setRunKVAL(RUN_KVAL_IN);
        Stepper.setAccKVAL(ACC_KVAL_IN);
        Stepper.setDecKVAL(DEC_KVAL_IN);
        Stepper.setHoldKVAL(HOLD_KVAL_IN);
        Stepper.setMaxSpeed(speed);
        Stepper.setAcc(ACC_IN);
        Stepper.setDec(DEC_IN);
    }

    Stepper.move(dir, steps);
}



float calculateSpeed(int acc, int dec, float t, int steps) {
    float discriminant = sq(t) - 2 * (1. / acc + 1. / dec) * steps;
    float speed = steps/t;
    if (discriminant > 0) {
        speed = (-t + sqrtf(discriminant)) / -(1. / acc + 1. / dec);
    }
    else{
        Serial.println("discriminant <0 ");
    }
    return speed;
}

//! Read the motor driver status. If the motor is moving
//! the isBusy function returns true.
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
void setKVals(uint8_t dir) {
    if (dir==DIR_EX){
        Stepper.setRunKVAL(RUN_KVAL_EX);
        Stepper.setAccKVAL(ACC_KVAL_EX);
        Stepper.setDecKVAL(DEC_KVAL_EX);
        Stepper.setHoldKVAL(HOLD_KVAL_EX);

        Stepper.setMaxSpeed(SPEED_EX);
        Stepper.setAcc(ACC_EX);
        Stepper.setDec(DEC_EX);
    }
    else {
        Stepper.setRunKVAL(RUN_KVAL_IN);
        Stepper.setAccKVAL(ACC_KVAL_IN);
        Stepper.setDecKVAL(DEC_KVAL_IN);
        Stepper.setHoldKVAL(HOLD_KVAL_IN);
        Stepper.setAcc(ACC_IN);
        Stepper.setDec(DEC_IN);
    }

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
    Stepper.setRunKVAL(RUN_KVAL_EX);
    Stepper.setAccKVAL(ACC_KVAL_EX);
    Stepper.setDecKVAL(DEC_KVAL_EX);
    Stepper.setHoldKVAL(HOLD_KVAL_EX);

    Stepper.setParam(ALARM_EN, 0x8F); // disable ADC UVLO (divider not populated),
    // disable stall detection (not configured),
    // disable switch (not using as hard stop)

    Stepper.getStatus(); // clears error flags

    Serial.println(F("Initialisation complete"));

}




