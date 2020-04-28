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
    bool shortPressDetected=false;
    bool sense=false;
    bool wasPressed=true;
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
    delay(1);
    if (digitalRead(PIN_ENCO_BTN)==HIGH){
        encoder.pressingTime.start();
    }
    if (digitalRead(PIN_ENCO_BTN)==LOW) {
        unsigned long elapsed = encoder.pressingTime.stop();
        if (elapsed > 2000) {
            encoder.longPressDetected = true;
        } else if (elapsed > 30) {
            encoder.shortPressDetected = true;
        }
    }
}


void detectButtonLongPress(){
    if (encoder.pressingTime.getElapsedTime() > 2000){
        encoder.longPressDetected=true;
        encoder.pressingTime.reset();
        Serial.println("longPress");
    }
    else if (encoder.pressingTime.getElapsedTime()>30){
        encoder.shortPressDetected=true;
        encoder.pressingTime.reset();
    }
}
/*
int buttonCounter=0;
int doubleCounter=0;
int _incrementer;
*/
Display display=Display(lcd, allUserParams, &VC_CMV, &encoder._increment, &encoder._increment, &encoder.shortPressDetected, &encoder.longPressDetected);

const VentilationMode * pointer=&VC_CMV;


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


    Serial.begin(115200);

    display.printStaticText();

}

void loop(){
    delay(100);
    /*for (int i=0; i < (display._mode->nParams); i++){
        lcd.setCursor(1,i);
        lcd.print(display._allUserParameters[(int)display._mode->parameters[i]].lcdString);
    }*/
    //display.printStaticText();
    /*
    if (encoder.shortPressDetected){
        Serial.println("short Press");
        encoder.shortPressDetected=false;
    }
    if (encoder.longPressDetected){
        Serial.println("long Press");
        encoder.longPressDetected=false;
    }
*/
    if (*display._toggleEditState){
        Serial.println("short Press");
        //*display._toggleEditState=false;
    }
    if (*display._toggleMenuState){
        Serial.println("long Press");
        //*display._toggleMenuState=false;
    }

    encoder.incrementEncoder();
    display.updateDisplay();
    Serial.println(display._editState);


}
/*
void loop(){

    lcd.home();
    lcd.clear();
    encoder.incrementEncoder();
    detectButtonLongPress();

    if (encoder.shortPressDetected) {
        buttonCounter++;
    }
    if (encoder.longPressDetected) {
        doubleCounter++;
        encoder.longPressDetected=false;
    }
    encoder.shortPressDetected=false;
    lcd.setCursor(1,0);
    lcd.print(buttonCounter);
    lcd.setCursor(1,2);
    lcd.print(doubleCounter);
    lcd.setCursor(1,4);
    lcd.print(encoder._increment);
    delay(20);
}*/
