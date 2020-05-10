//
// Created by david on 24.04.20.
//

#include "Display.h"
#include <Arduino.h>
#include <LiquidCrystal.h>

Display::Display(LiquidCrystal &lcd, User_Parameter* allUserParameters, const VentilationMode* mode,int *cursorIncrementer, int *valueIncrementer, bool *toggleEditState, bool *toggleMenuState): _lcd(12, 11, 10, 9, 8, 7){
    _lcd = lcd;
    _lcd.begin(20, 4);
    byte CURSOR_SYMBOL[8] = {
            B00000,
            B00100,
            B00110,
            B11111,
            B00110,
            B00100,
            B00000,
            B00000
    };
    _mode = mode;
    _allUserParameters = allUserParameters;
    _valueIncrementer = valueIncrementer;
    _toggleMenuState = toggleMenuState;
    _toggleEditState = toggleEditState;
    markerIncrementer = cursorIncrementer;
    _valueIncrementer= valueIncrementer;
    _editState = VIEW_ONLY;
    _markerPositionMax =3;
    _markerPositionMin = 0;
    printStaticText();
    lcd.cursor();
    for (int8_t i=0; i<_mode->nParams;i++){
        printParameterValue(_allUserParameters[(int) _mode->parameters[i]].getValue(),5,i);
    }

}

void Display::updateDisplay() {
    switch(_editState){
        case VIEW_ONLY:{
            moveMarker();
            if (*_toggleEditState){
                _editState=EDIT_ENTRY;
                *_toggleEditState=false;
                _lcd.cursor();

            }
            break;
        }
        case NAVIGATE:
            moveMarker();
            if (*_toggleEditState){
                loadParams();
                _editState=EDIT_ENTRY;
                *_toggleEditState=false;
            }
            else if (*_toggleMenuState){
                _editState=VIEW_ONLY;
                *_toggleMenuState=false;
            }
            break;

        case EDIT_ENTRY: {

            _activeParamIndex = _markerPosition;
            _parametersMemory[_activeParamIndex]+=
                    (float) *_valueIncrementer * _allUserParameters[(int)_mode->parameters[_activeParamIndex]].step;
            Serial.println(_parametersMemory[_activeParamIndex]);
            printParameterValue(_parametersMemory[_activeParamIndex], _valueColumnPos, _markerPosition);
            *_valueIncrementer=0;

            if (*_toggleEditState) {
                _editState = NAVIGATE;
                *_toggleEditState=false;

            }
            else if (*_toggleMenuState) {
                Serial.println("from edit to view_only");
                _editState = VIEW_ONLY;
                *_toggleMenuState=false;
                _lcd.noCursor();
            }
            break;
        }
    }

    *_toggleEditState=false;
}

void Display::moveMarker() {
// calc new position
    _lcd.setCursor(0, _markerPosition);
    //_lcd.print(" ");
    int newPos = _markerPosition + *markerIncrementer;
    *markerIncrementer=0;
    if (newPos>_markerPositionMax){
        _markerPosition=_markerPositionMax;
    }
    else if (newPos<_markerPositionMin){
        _markerPosition=_markerPositionMin;
    }
    else{
        _markerPosition = newPos;
    }
    _lcd.setCursor(0, _markerPosition);
    //_lcd.write(byte(0));

}

void Display::printParameterValue(float value, uint8_t col, uint8_t row) {
    _lcd.setCursor(col, row);
    uint8_t len=0;
    uint8_t blank=1;
    if (value<10){
        len=2;
        blank=0;
    }
    else if (value<100){
        len=1;
        blank=0;
    }
    if (blank){
        _lcd.print(" ");
    }
     _lcd.print(value, len );
}

void Display::safeParams() {
    // save user parameters momentarily
    for (int i; i<(_mode->nParams); i++){
        _allUserParameters[(int)_mode->parameters[i]].setValue(_parametersMemory[i]);
    }
}

void Display::loadParams() {
    // save user parameters momentarily
    for (int i; i<(_mode->nParams); i++){
        _parametersMemory[i]=_allUserParameters[(int)_mode->parameters[i]].getValue();
    }
}

void Display::printStaticText() {
    for (int i=0; i < (_mode->nParams); i++){
        _lcd.setCursor(0, i);
        _lcd.print(_allUserParameters[(int)_mode->parameters[i]].lcdString);
    }
}
