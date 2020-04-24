//
// Created by david on 22.04.20.
//

#ifndef HDVENT_CONTROL_LCD_DISPLAY_H
#define HDVENT_CONTROL_LCD_DISPLAY_H


#include <LiquidCrystal.h>
#include <Arduino.h>

byte cursorSymbol[] = {
        B00000,
        B00100,
        B00110,
        B11111,
        B00110,
        B00100,
        B00000,
        B00000
};

#define A 2 // output A of the encoder // For use Iterrupts just PIN 2 and 3 with Arduino Uno
#define B 4 // output B of the encoder
#define BTN 3 // button of the encoder // For use Iterrupts just PIN 2 and 3 with Arduino Uno
#define SWT 5 // switch

LiquidCrystal lcd(12, 11, 10, 9, 8, 7);

volatile boolean rotation;
volatile boolean sens;
volatile boolean buttonPressed = LOW;

float validation = 0;
float rounds = 0;
int ArrowRow = 1;
int access = 0;


void Interruption(){
    if (digitalRead(A)==HIGH){
        sens = digitalRead(B); // Clockwise
    }
    else{
        sens = !digitalRead(B); // anticlockwise
    }
    rotation = HIGH;
}

void buttonInterrupt(){
    if (digitalRead(BTN)==HIGH){ // if the button have been activate
        buttonPressed = HIGH;
    }
}

void setup(){
    lcd.begin(20, 4);
    lcd.createChar(0, cursorSymbol);
    attachInterrupt (digitalPinToInterrupt(2),Interruption,FALLING); // interruption on falling edge of output A
    //0 is the direct interrupt numbers for digital pin 2
    //Interruptions is our Interrupt Service Routine, a function that must take no parameters and return nothing
    //FALLING for when the pin goes from high to low (is the mode and defines when the interrupt should be triggered)
    attachInterrupt (digitalPinToInterrupt(3),buttonInterrupt,RISING);
    pinMode(A,INPUT);
    pinMode(B,INPUT);
    pinMode(BTN,INPUT);
    pinMode(SWT,INPUT);
}

void loop(){
    if (digitalRead(SWT)==HIGH){ // if the switch is on
        lcd.home();
        staticMenu();
        encoderChooseRow();

        switch (ArrowRow) {
            case 1:
                lcd.clear();
                staticMenu();
                lcd.setCursor (0, 1);   // the cursorSymbol is on the first line
                lcd.write(0.);
                if (buttonPressed == HIGH){
                    encoderSelectValue();
                }
                lcd.setCursor (3, 1);
                lcd.print(rounds);
                break;

            case 2:
                lcd.clear();
                staticMenu();
                lcd.setCursor (0, 2);   // the cursorSymbol is on the second line
                lcd.write(0.);
                break;

            case 3:
                lcd.clear();
                staticMenu();
                lcd.setCursor (0, 3);    // the cursorSymbol is on the third line
                lcd.write(0.);
                break;

            default:
                staticMenu();
                if(ArrowRow > 3){
                    ArrowRow = 3;
                }
                if(ArrowRow < 1){
                    ArrowRow = 1;
                }
                break;
        }

    }else{
        lcd.clear();
    }
}

void staticMenu(){
    lcd.setCursor(0, 0);
    lcd.print("Set:");
    lcd.setCursor (11, 0);
    lcd.print("P(cmH20):");
    lcd.setCursor (1, 1);
    lcd.print("V=");
    lcd.setCursor (1, 2);
    lcd.print("RR=");
    lcd.setCursor (13, 1);
    lcd.print("peak=");
    lcd.setCursor (18, 1);
    //lcd.print(peak);
    lcd.setCursor (13, 2);
    lcd.print("plat=");
    lcd.setCursor (18, 2);
    //lcd.print(plateu);
    lcd.setCursor (1, 3);
    lcd.print("I:E=1:");
    lcd.setCursor (13, 3);
    lcd.print("PEEP=");
    lcd.setCursor (18, 3);
    //lcd.print(PEEP);
}

void encoderChooseRow(){ // for the choosing the Row
    if (rotation == HIGH){ // If detection of rotation
        if (sens == HIGH){ // if clockwise
            ArrowRow++; // increase counter
        }
        else{
            ArrowRow--; // decrease counter
        }
        rotation = LOW;
    }
}

void encoderSelectValue(){ // for the selecting a value for the Tidal volume
    if (rotation == HIGH){ // If detection of rotation
        if (sens == HIGH){ // if clockwise
            rounds++; // increase counter
        }
        else{
            rounds--; // decrease counter
        }
        rotation = LOW;
    }
}

#endif //HDVENT_CONTROL_LCD_DISPLAY_H
