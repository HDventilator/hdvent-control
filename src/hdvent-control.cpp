//
// Created by david on 04.04.20.
//
#include <hdvent-control.h>
#include <HardwareSerial.h>


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


    allUserParams[(int) UP::RESPIRATORY_RATE] = User_Parameter(10, 5,30, "freq"); //  breaths per minute
    allUserParams[(int) UP::T_IN] = User_Parameter(1.5, t_in_min, t_in_max,"T_in"); // Inspiration time
    allUserParams[(int) UP::TIDAL_VOLUME] = User_Parameter(350, 0, 650, "VTid"); // milliliters
    allUserParams[(int) UP::INSPIRATORY_PRESSURE] = User_Parameter(20, 5, 50, "Pins"); //  millibar
    allUserParams[(int) UP::FLOW] = User_Parameter(200, 5, 500,"Flow"); //  milliliters per second
    allUserParams[(int) UP::D_PRESSURE_SUPP] = User_Parameter(20, 5, 50, "Psup"); //  millibar
    allUserParams[(int) UP::PRESSURE_TRIGGER_THRESHOLD] = User_Parameter(5, 5, 50, "Pthr"); //  millibar per second
    allUserParams[(int) UP::FLOW_TRIGGER_THRESHOLD] = User_Parameter(20, 5, 50, "Fthr"); //  milliliters per second
    allUserParams[(int) UP::ANGLE] = User_Parameter(0, 0, 1, "angl"); //  milliliters per second
    allUserParams[(int) UP::COMPRESSED_VOLUME_RATIO] = User_Parameter(100, 0, 100, "Volu"); //  milliliters per second
    allUserParams[(int) UP::SLOPE_P] = User_Parameter(0.2, 0, 2, "slop"); //  milliliters per second
    allUserParams[(int) UP::KP] = User_Parameter(1, 0, 1, "kp  "); //  milliliters per second
    allUserParams[(int) UP::KI] = User_Parameter(1, 0, 3, "ki  "); //  milliliters per second

    allVentiModes[(int) VentiModes::OL_CMV] = OL_CMV;
    allVentiModes[(int) VentiModes::PC_CMV] = PC_CMV;
    allVentiModes[(int) VentiModes::VC_CMV] = VC_CMV;

    ConfigureStepperDriver();
    pressureSensor.begin();

    lcd.begin(20,4);
    lcd.flush();
    lcd.home();

    // adjust sensor state if connected/disconnected
    opticalHomeSensor.setState(Sensor::OK);
    angleSensor.setState(Sensor::OK);

    // stepper cannot know absolute position on startup
    stepperMonitor.setState(Sensor::FAULTY);

    // initial state for state machine
    ventilationState = STARTUP;
    //flowSensor.begin();
    Wire.begin();
    //Serial3.begin(115200);
    cobsSerial.setStream(&Serial3);
    //cobsSerial.begin(115200);
    Serial3.begin(115200);

    stopwatch.sinceIdle.start();

    allUserParams.update(mode.parameters, mode.nParams);

    for (Diagnostic_Parameter &param : diagnosticParameters.arr){
        param.setValue(10);
    }

   // controller._pid.SetTunings(allUserParams[(int)UP::KP].getValue(), allUserParams[(int)UP::KI].getValue(),0);

    enumerateEEPROM();
    readFromEEPROM();
    display.printStaticText();
    allVentiModes.setActive((int) VentiModes::VC_CMV);
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
void ledService(){
    if (runVentilation){
        if (greenLEDBlink.getEvent()) {
            digitalWrite(PIN_LED_GREEN, greenLEDBlink.getState());
        }
    }
    else {
        digitalWrite(PIN_LED_GREEN, LOW);
    }

//if (display.getMenuState()==Display::UNSAVED_SETTINGS){
    if (alarmIsTriggered){
        if (orangeLEDBlink.getEvent()){
            digitalWrite(PIN_LED_ORANGE, orangeLEDBlink.getState());
        }
    }
    else {
        digitalWrite(PIN_LED_ORANGE, LOW);
    }

}

void loop(){
    if ((ventilationState==HOLDING_EX )|| (ventilationState==IDLE)){
        mode = allVentiModes.getActiveMode();
    };
    ledService();

    stopwatch.mainLoop.start();
    cycleTimeMus = mainLoopMus.getElapsedTime();
    mainLoopMus.start();

    buzzer.service();
    rescaleParameterLimits();

    runMachineDiagnostics();

    checkAlarms();
    readSensors();
    checkHomeSensors(isHome);

    runVentilation = !digitalRead(PIN_SD_VENTI);

   display.update(confirmButton.getSingleDebouncedPress(),
            cancelButton.getSingleDebouncedPress(),
            encoderButton.getSingleDebouncedPress(),
            encoder.getDelta());

    writeDiagnosticParameters();
    //writeUserInput();

    //Serial.println(angleSensor.getHome());
    //serialDebug();

    if (display.getSavingEvent()){
        safeToEEPROM();
    }
    ventilationStateMachine(ventilationState);
    writeDiagnosticAlarms();

    // check if any user parameter was edited
    bool wasEdited=false;
    for (int i=0; !wasEdited && i<mode.nParams; i++){
        wasEdited = wasEdited || allUserParams.getActive(i).wasEdited;
    }
    if (wasEdited){
        rescaleParameterLimits();
        writeUserInput();
    }

}

void debugEEPROM(){
    for (int i=0; i<(mode.nParams); i++){
        Serial.print(allUserParams.getActive(i).getEeAddress());
        Serial.print("\t");
    }
    Serial.println("");
    for (int i=0; i<(mode.nParams); i++){
        Serial.print(allUserParams.getActive(i).getValue());
        Serial.print("\t");
    }
    Serial.println("");
}

void safeToEEPROM(){
    for (int i=0; i<(mode.nParams); i++){
        int address = allUserParams.getActive(i).getEeAddress();
        float value=allUserParams.getActive(i).getValue();
        EEPROM.put(address, value);
    }
    int address = angleSensor.getEeAddress();
    float value=angleSensor.getHome();
    EEPROM.put(address, value);
}

void enumerateEEPROM(){
    int index=0;
    for (int i=0; i<(int)UP::LAST_PARAM_LABEL; i++){
        allUserParams[i].setEeAddress(index);
        index += sizeof(float);
        //Serial.print(index);
    }
    angleSensor.setEeAddress(index);
    index+=sizeof(float);

}

void readFromEEPROM(){
    for (int i=0; i<(mode.nParams); i++){
        float value;
        int address = allUserParams.getActive(i).getEeAddress();
        EEPROM.get(address, value);
        allUserParams.getActive(i).setValue(value);
        //Serial.print("address: ");Serial.println(address);
    }
    float value;
    int address = angleSensor.getEeAddress();
    EEPROM.get(address, value);
    angleSensor.setHome(value);
   // Serial.print("address: ");Serial.println(address);
}


void debugMotor(){
    Stepper.resetPos();
    if (confirmButton.getSingleDebouncedPress()){
        float speed=0;
        while (speed<800) {
            speed = speed +100;
            float delayTime = 10000;
            Stepper.resetPos();
            Stepper.run(DIR_EX, speed);
            delay(delayTime);
            Stepper.hardStop();
            long d = Stepper.getPos();
            //delay((float)(200.0*1000) / 80 - delayTime);
            Serial.println(d);
            machineDiagnostics.absolutePosition.setValue(d);
            serialWritePackage(&cobsSerial, machineDiagnostics.absolutePosition.getPackageStruct());
        }
    }
    Stepper.hardStop();
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
    bool isAlarmOverwrite = alarmOverwrite.getSingleDebouncedPress();

    for (int i=0; i<mode.nParams;i++){
            Diagnostic_Parameter & param = diagnosticParameters.arr[i];

        if (param.getState() != Diagnostic_Parameter::OK) {
            Serial.print("alarm param");Serial.println(param.getIdentifier());
            buzzer.saveTurnOn();
            Serial.print("alarm state: ");Serial.println(param.getState());
            serialWritePackage(&cobsSerial, param.getAlarmTriggeredPackage());
            alarmIsTriggered=true;
        } else if (isAlarmOverwrite && (param.getState() == Diagnostic_Parameter::OK)) {
            buzzer.turnOff();
            param.resetPersistentAlarm();
            serialWritePackage(&cobsSerial, param.getAlarmTriggeredPackage());
            alarmIsTriggered=false;
        }
    }

}

void writeDiagnosticParameters(){
    serialWritePackage(&cobsSerial, diagnosticParameters.s.flow.getPackageStruct());
    serialWritePackage(&cobsSerial, diagnosticParameters.s.airwayPressure.getPackageStruct());
    serialWritePackage(&cobsSerial, diagnosticParameters.s.plateauPressure.getPackageStruct());
    serialWritePackage(&cobsSerial, diagnosticParameters.s.volume.getPackageStruct());
    serialWritePackage(&cobsSerial, diagnosticParameters.s.tidalVolume.getPackageStruct());
    serialWritePackage(&cobsSerial, diagnosticParameters.s.peep.getPackageStruct());
    serialWritePackage(&cobsSerial, diagnosticParameters.s.minuteVolume.getPackageStruct());
}

void writeDiagnosticAlarms() {
    for (int i = 0; i < nDiagnosticParameters; i++) {
        if (diagnosticParameters.arr[i].isAlarmSettingChanged()) {
            Serial.println("alarm settings changed!");
            serialWritePackage(&cobsSerial, diagnosticParameters.arr[i].getSettingsAlarmPackage());
            if (diagnosticParameters.arr[i].getLoAlarmSet()==Diagnostic_Parameter::ACTIVE){
                serialWritePackage(&cobsSerial, diagnosticParameters.arr[i].getLoAlarmThresholdPackage());
            }
            if (diagnosticParameters.arr[i].getHiAlarmSet()==Diagnostic_Parameter::ACTIVE){
                serialWritePackage(&cobsSerial, diagnosticParameters.arr[i].getHiAlarmThresholdPackage());
            }
        }
    }
}

void runMachineDiagnostics(){
    // program cycle time
   machineDiagnostics.cycle_time.setValue(cycleTimeMus);
    serialWritePackage(&cobsSerial, machineDiagnostics.cycle_time.getPackageStruct());

    // ventilation state
    /*
    machineDiagnostics.ventilationState.setValue((int)ventilationState);
    serialWritePackage(&cobsSerial, machineDiagnostics.ventilationState.getPackageStruct());

    machineDiagnostics.stepperPosition.setValue(stepperMonitor.getData().relativePosition);
   serialWritePackage(&cobsSerial, machineDiagnostics.stepperPosition.getPackageStruct());

    machineDiagnostics.rotationSensor.setValue(angleSensor.getData().relativePosition);
    serialWritePackage(&cobsSerial, machineDiagnostics.rotationSensor.getPackageStruct());

    machineDiagnostics.absolutePosition.setValue(Stepper.getParam(ABS_POS));
    serialWritePackage(&cobsSerial, machineDiagnostics.absolutePosition.getPackageStruct());

*/
    serialWritePackage(&cobsSerial, machineDiagnostics.calculatedSpeed.getPackageStruct());
    serialWritePackage(&cobsSerial, machineDiagnostics.setpointPID.getPackageStruct());

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


//Serial.println(cycleTimeMus);
    /*for (int i=0; i < (display._mode->nParams); i++){
    lcd.setCursor(1,i);
    lcd.print(display._allUserParams[(int)display._mode->parameters[i]].lcdString);
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
    //Serial.print("T inspiration:   ");Serial.println(allUserParams[(int)UP::T_IN].getValue());
   // Serial.print("Volume:   ");Serial.println(allUserParams[(int)UP::COMPRESSED_VOLUME_RATIO].getValue());
   // Serial.print("Frequency:   ");Serial.println(allUserParams[(int)UP::RESPIRATORY_RATE].getValue());
    //Serial.print("Angle:"); Serial.println(angleSensor.getData().absolutePosition);
    //Serial.print("Angle:"); Serial.println(angleSensor.getData().relativePosition);
    //Serial.print("Angle/MotorMonitor:"); Serial.println(stepperMonitor.getData().relativePosition-angleSensor.getData().relativePosition);
    //Serial.print("T inspiration:   ");Serial.println(display._allUserParams[(int)UP::T_IN].getValue());
    //Serial.print("Volume:   ");Serial.println(display._allUserParams[(int)UP::COMPRESSED_VOLUME_RATIO].getValue());
    //Serial.print("Frequency:   ");Serial.println(display._allUserParams[(int)UP::RESPIRATORY_RATE].getValue());
    /*Serial.print("T inspiration:   ");Serial.println(allUserParams[(int)UP::T_IN].isGettingEdited);
    Serial.print("Volume:   ");Serial.println(allUserParams[(int)UP::COMPRESSED_VOLUME_RATIO].isGettingEdited);
    Serial.print("Frequency:   ");Serial.println(allUserParams[(int)UP::RESPIRATORY_RATE].isGettingEdited);*/
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
    //Serial.print("Time: "); Serial.println(cycleTimeMus);
}
void rescaleParameterLimits(){
    float T = 60/allUserParams[(int)UP::RESPIRATORY_RATE].getDialValue();
    float t_in = allUserParams[(int)UP::T_IN].getDialValue();
    float x = allUserParams[(int)UP::COMPRESSED_VOLUME_RATIO].getDialValue()/100*STEPS_FULL_RANGE*STEP_DIVIDER;
    float a = ACC_EX *STEP_DIVIDER;

    float f_max = 60/(t_in + sqrtf(4*x/a) + TIME_HOLD_PLATEAU);
    allUserParams[(int)UP::RESPIRATORY_RATE].setMax(f_max);

    //float t_in_min = sqrtf(4*x/a);
    // using max speed
    float vMax = 600*STEP_DIVIDER;
    float t_in_min = (x-sq(vMax)/a)/vMax +2*vMax/a;
    float t_in_max = T-sqrtf(4*x/a) - TIME_HOLD_PLATEAU;
    t_in_min = max (0, t_in_min);
    allUserParams[(int)UP::T_IN].setMin(t_in_min);
    allUserParams[(int)UP::T_IN].setMax(t_in_max);

    float x_max = sq(t_in) *a/4;
    x_max = min (x_max, STEPS_FULL_RANGE*STEP_DIVIDER);
    allUserParams[(int)UP::COMPRESSED_VOLUME_RATIO].setMax(x_max*100/STEP_DIVIDER/STEPS_FULL_RANGE);

}

void writeUserInput(){
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
    float pressure=0;
    pressureSensor.readSensor();
    pressure=pressureSensor.getData().pressure;
    /*for (int i=0; i<5; i++){
        pressureSensor.readSensor();
        pressure+=pressureSensor.getData().pressure;
    }
    pressure = pressure/5;
*/
    float flow=0;
    for (int i=0; i<15; i++){
        flowSensor.readSensor();
        flow += (flowSensor.getData().pressure-PRESSURE_FLOW_CONVERSION_OFFSET) * PRESSURE_FLOW_CONVERSION;
    }
    flow = flow/15;
    /*flowSensor.readSensor();
    flow = (flowSensor.getData().pressure-PRESSURE_FLOW_CONVERSION_OFFSET) * PRESSURE_FLOW_CONVERSION;*/
    diagnosticParameters.s.flow.setValue(flow);
    diagnosticParameters.s.airwayPressure.setValue(pressure);
    //flowSensor.readSensor();



    // integrate flow for volume
    if ((ventilationState == START_IN)||(ventilationState == IDLE)){
        diagnosticParameters.s.volume.setValue(0);
    }
    else {
        float newVolume = diagnosticParameters.s.volume.getValue() + diagnosticParameters.s.flow.getValue() * (float)cycleTimeMus / 1000000;
        diagnosticParameters.s.volume.setValue(newVolume);
    }

    // calculate tidal volume and minute volume
    if (ventilationState == END_IN) {
        unsigned long timestamp=stopwatch.sinceIdle.getElapsedTime();
        diagnosticParameters.s.tidalVolume.setValue(diagnosticParameters.s.volume.getValue());

        minuteVolume.enqueue(diagnosticParameters.s.tidalVolume.getValue(), timestamp);
        diagnosticParameters.s.minuteVolume.setValue(minuteVolume.sumFromInterval(60000)/1000);


        if (timestamp>60000){
            timestamp = timestamp-60000;
        }
        else {
            timestamp =0;
        }
        //diagnosticParameters.s.minuteVolume.sumFromTimestamp(timestamp);
        //Serial.print("elapsed time:"); Serial.println(timestamp);
    }

    if (ventilationState == HOLDING_IN){
        if (stopwatch.holdingIn.getElapsedTime()>50){
            diagnosticParameters.s.plateauPressure.setValue(pressureSensor.getData().pressure);
        }
    }

    if (ventilationState == START_IN){
        //if (stopwatch.expiration.getElapsedTime()>50){
            diagnosticParameters.s.peep.setValue(pressureSensor.getData().pressure);
        //}
    }
/*
    float pressureChange = (pressureSensor.getData().pressure - oldPressure)/cycleTimeMus*1000;
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
        case STARTUP:
            if (isHome){
                Stepper.resetPos();
                state=IDLE;
            }
            else {
                state=START_HOMING;
            }


        case START_HOMING: // executed when transitioning to HOMING_EX
            // stepper driver doesn't know absolute home position
            stepperMonitor.setState(Sensor::FAULTY);

            // start full revolution of stepper in ex direction to find home position
            moveStepper(STEPS_FS_FULL_TURN*STEP_DIVIDER, 100, 400, 400, DIR_EX);

            state = HOMING_EX;
            break;

        case HOMING_EX:

            // home position is recognized
            if (isHome){
                Stepper.hardStop();

                // store home position in stepper driver memory
                Stepper.resetPos();
                angleSensor.resetPos();
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
            if (! runVentilation){
                state = IDLE;
            }
            else if (controller.inspirationTrigger()) {
                state = START_IN;
            }
            break;

        case START_IN: {
            // record time after start of inspiration
            stopwatch.inspiration.start();
            setKVals(DIR_IN);
            // start the setpoint generation for the controller



            switch (mode.controlMode){
                case ControlMode::PC:{
                    float slopeTime = allUserParams[(int)UP::SLOPE_P].getValue();
                    float level = allUserParams[(int)UP::INSPIRATORY_PRESSURE].getValue();
                    float offset = diagnosticParameters.s.peep.getValue();
                    offset = min(offset, 40);
                    offset = max(offset, 0);
                    controller.startRamp(slopeTime, level, offset,
                            mode.pidParameters.k_p,
                                         mode.pidParameters.k_i,
                                         mode.pidParameters.k_d);
                    Stepper.setMaxSpeed(1000);
                    Stepper.setAcc(3000);
                    break;}
                case ControlMode::VC:{
                    float slopeTime = 0.01;//allUserParams[(int)UP::SLOPE_P].getValue();
                    float level = allUserParams[(int)UP::FLOW].getValue();
                    float offset = 0;//diagnosticParameters.s.peep.getValue();
                    offset = min(offset, 40);
                    offset = max(offset, 0);
                    controller.startRamp(slopeTime, level, offset,
                            0.25,//allUserParams[(int)UP::KP].getValue(),
                            0.03,//allUserParams[(int)UP::KP].getValue(),
                           0);
                    Stepper.setMaxSpeed(1000);
                    Stepper.setAcc(3000);
                    break;}
                case ControlMode::OL: {
                    float steps = allUserParams[(int) UP::COMPRESSED_VOLUME_RATIO].getValue() / 100 * STEPS_FULL_RANGE;
                    float time = allUserParams[(int) UP::T_IN].getValue();
                    float speed = calculateSpeed(ACC_IN, DEC_IN, time, steps);
                    //Serial.print("Move steps: ");
                    //Serial.print(steps);
                    //Serial.print("Absolute Position Start: ");
                    //Serial.println(machineDiagnostics.absolutePosition.getValue());
                    controller.startTrapezoid(ACC_IN, speed, time);
                }

                default:
                    break;

            }


            state = MOVING_IN;
            integratedPosition = 0;
            stopwatch.movingIn.start();
            break;
        }

        case MOVING_IN:{
            //Serial.print("controlMode: ");Serial.println((int)mode.controlMode);
            if (stepperMonitor.getData().relativePosition>STEPS_FULL_RANGE*0.9){
                Stepper.hardStop();
            }
            float speed=0;
            switch (mode.controlMode){
                case ControlMode::OL:
                    // bypass setpoint
                    speed = (float)controller.calcSpeed();
                    break;

                case ControlMode::VC:
                    speed = controller.calcSetPoint()*0.095+(float)controller.calcSpeed(diagnosticParameters.s.flow.getValue());
                    break;
                case ControlMode::PC:
                    speed = (float)controller.calcSpeed(diagnosticParameters.s.airwayPressure.getValue());
                    //speed = controller.calcSetPoint();
                    //Serial.print("speed:");Serial.println(speed);
                    //Serial.print("kp:");Serial.println(controller._pid.GetKp());
                    break;
                default:
                    break;
                }

            //speed = max(speed,(int) 0) * SPEED_CORRECTION;
            speed = speed*SPEED_CORRECTION;
            machineDiagnostics.calculatedSpeed.setValue(speed);
            machineDiagnostics.setpointPID.setValue(controller.calcSetPoint());
            if (!Stepper.busyCheck()) {
                Stepper.run(DIR_IN, speed);

            }
            else {Serial.println("not changed");
            }

            // start expiration on trigger
            if (controller.expirationTrigger()) {
                controller.stopControl();
            state = END_IN;}
            break;}

        case END_IN:

            Stepper.hardStop();
            stopwatch.holdingIn.start();
            state = HOLDING_IN;

        case HOLDING_IN:
            if (!Stepper.busyCheck()){
                //stopwatch.movingIn.stop();
                //Serial.println(stopwatch.movingIn.getElapsedTime());
            }
            if (stopwatch.holdingIn.getElapsedTime() > TIME_HOLD_PLATEAU*1000){
                Stepper.hardStop(); // this ensures moving out command is received
                state = START_EX;

            }
            break;

        case START_EX:
            machineDiagnostics.setpointPID.setValue(0);
            stopwatch.expiration.start();
            setKVals(DIR_EX);
            Stepper.goToDir(DIR_EX, 0);
            state=MOVING_EX;
            break;

        case MOVING_EX:
            // record PEEP pressure shortly after start of expiration phase
            /*if (stopwatch.expiration.getElapsedTime() > 50){
                diagnosticParameters.s.peep.setValue(pressureSensor.getData().pressure);
            }*/
            // Stepper has reached home position
            if (isHome) {
                if (Stepper.busyCheck()){
                    Stepper.resetPos();
                }
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

/*bool Triggers::pressureDrop() {
    return (- diagnosticParameters.s.pressureChange.getValue() > allUserParams[(int)UP::PRESSURE_TRIGGER_THRESHOLD].getValue());
}*/

bool Triggers::flowIncrease() {
    return diagnosticParameters.s.flow.getValue() > allUserParams[(int)UP::FLOW_TRIGGER_THRESHOLD].getValue();
}

bool Triggers::angleReached() {
    return (float) getPosition() > allUserParams[(int)UP::ANGLE].getValue() * STEPS_FULL_TURN * STEP_DIVIDER;
}

bool Triggers::tidalVolume() {
    return diagnosticParameters.s.tidalVolume.getValue() > allUserParams[(int)UP::TIDAL_VOLUME].getValue();
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
            if (opticalHomeSensor.getData().isBlocked && !angleSensor.getData().isHome){
                angleSensor.resetPos();
            }
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

void moveStepper(int steps, float speed, int dir) {
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
        speed = (t - sqrtf(discriminant)) / (1. / acc + 1. / dec);
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

    Stepper.configStepMode(STEP_DIVIDER_REGISTER); // 1/128 microstepping, full steps = STEP_FS,
    // options: 1, 1/2, 1/4, 1/8, 1/16, 1/32, 1/64, 1/128

    Stepper.setMaxSpeed(SPEED_EX); // max speed in units of full steps/s
    Stepper.setFullSpeed(2000); // full steps/s threshold for disabling microstepping
    Stepper.setAcc(ACC_EX); // full steps/s^2 acceleration
    Stepper.setDec(ACC_IN); // full steps/s^2 deceleration

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

    Stepper.setLoSpdOpt(HIGH);
    Stepper.setMinSpeed(0);
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


