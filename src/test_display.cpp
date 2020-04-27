#include <Arduino.h> // main arduino library
#include <LiquidCrystal.h>
#include <Stopwatch.h>
#include <User_Parameter.h>
#include <PID_v1.h>
#include <Ventilation_Modes.h>
#include <Display.h>
LiquidCrystal lcd(12, 11, 10, 9, 8, 7);
// user-set parameters
User_Parameter allUserParams[(int) UP::LAST_PARAM_LABEL];
//Display(lcd, allUserParams, VC_CMV, );

int const PIN_ENCO_BTN=3;
int const PIN_ENCO_A = 2;
int const PIN_ENCO_B = 4;

 bool Triggers::inspirationAttempt(){

 };
bool Triggers::pressureDrop() {

};
bool Triggers::flowIncrease() {

};
bool Triggers::respiratoryRate() {

};
bool Triggers::inspirationTime(){

};


struct{
    bool wasTurned=false;
    bool wasPressed=false;
    bool sense=false;
    int _increment=0;
    Stopwatch pressingTime;
    bool longPressDetected=false;
    void incrementEncoder(){
        if (wasTurned){
            if (sense){
                _increment++;
            }
            if (!sense){
                _increment--;
            }
        }
            wasTurned = false;
    }
} encoder;

void turnEncoderInterruptRoutine(){
    encoder.sense = digitalRead(PIN_ENCO_A) == HIGH == digitalRead(PIN_ENCO_B);
    encoder.wasTurned=true;
    }


void buttonEncoderInterruptRoutine(){
    if (digitalRead(PIN_ENCO_BTN)==HIGH){
        encoder.wasPressed = true;
        encoder.pressingTime.start();
    }
    if (digitalRead(PIN_ENCO_BTN)==LOW){
        encoder.pressingTime.stop();
    }
}


void detectButtonLongPress(){
    if (encoder.pressingTime.getElapsedTime() > 1000){
        encoder.longPressDetected=true;
    }
}


void setup(){
    pinMode(PIN_ENCO_A,INPUT);
    pinMode(PIN_ENCO_B,INPUT);
    pinMode(PIN_ENCO_BTN,INPUT);

    allUserParams[(int) UP::RESPIRATORY_RATE] = User_Parameter(15, 5, 35, "f"); //  breaths per minute
    allUserParams[(int) UP::TIDAL_VOLUME] = User_Parameter(250, 0, 650, "VT"); // milliliters
    allUserParams[(int) UP::T_IN] = User_Parameter(2, 0.6, 4,"Ti"); // Inspiration time
    allUserParams[(int) UP::INSPIRATORY_PRESSURE] = User_Parameter(20, 5, 50, "Paw"); //  millibar
    allUserParams[(int) UP::FLOW] = User_Parameter(20, 5, 50,"Flo"); //  milliliters per second
    allUserParams[(int) UP::D_PRESSURE_SUPP] = User_Parameter(20, 5, 50, "dPs"); //  millibar
    allUserParams[(int) UP::PRESSURE_TRIGGER_THRESHOLD] = User_Parameter(5, 5, 50, "Ptr"); //  millibar per second
    allUserParams[(int) UP::FLOW_TRIGGER_THRESHOLD] = User_Parameter(20, 5, 50, "Ftr"); //  milliliters per second

    attachInterrupt(digitalPinToInterrupt(PIN_ENCO_BTN), buttonEncoderInterruptRoutine, CHANGE);
    attachInterrupt (digitalPinToInterrupt(PIN_ENCO_A),turnEncoderInterruptRoutine,FALLING);

    lcd.begin(20, 4);

    Serial.begin(115200);
}
/*
int buttonCounter=0;
int doubleCounter=0;
int _incrementer;
*/
Display display=Display(lcd, allUserParams, &VC_CMV, &encoder._increment, &encoder._increment, &encoder.wasPressed, &encoder.longPressDetected);

const VentilationMode * pointer=&VC_CMV;
void loop(){
    /*for (int i=0; i < (display._mode->nParams); i++){
        lcd.setCursor(1,i);
        lcd.print(display._allUserParameters[(int)display._mode->parameters[i]].lcdString);
    }*/
    //display.printStaticText();
    display.moveMarker();
    //display._lcd.setCursor(1,1);
    //display._lcd.write(1);
    //display.updateDisplay();
    //Serial.println(display._editState);
}
/*
void loop(){

    lcd.home();
    lcd.clear();
    encoder.incrementEncoder();
    detectButtonLongPress();

    if (encoder.wasPressed) {
        buttonCounter++;
    }
    if (encoder.longPressDetected) {
        doubleCounter++;
        encoder.longPressDetected=false;
    }
    encoder.wasPressed=false;
    lcd.setCursor(1,0);
    lcd.print(buttonCounter);
    lcd.setCursor(1,2);
    lcd.print(doubleCounter);
    lcd.setCursor(1,4);
    lcd.print(encoder._increment);
    delay(20);
}*/
