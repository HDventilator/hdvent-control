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

}

void Display::updateDisplay() {
    switch(_editState){
        case VIEW_ONLY:{
            moveMarker();
            if (*_toggleEditState){
                _editState=EDIT_ENTRY;

            }
            break;
        }
        case NAVIGATE:
            moveMarker();
            if (*_toggleEditState){
                loadParams();
                _editState=EDIT_ENTRY;
            }
            else if (*_toggleMenuState){
                _editState=VIEW_ONLY;
            }
            break;

        case EDIT_ENTRY: {

            _activeParamIndex = _markerPosition;
            _parametersMemory[_activeParamIndex]+=
                    (float) *_valueIncrementer * _allUserParameters[(int)_mode->parameters[_activeParamIndex]].step;

            printParameterValue();

            if (*_toggleEditState) {
                _editState = NAVIGATE;

            } else if (*_toggleMenuState) {
                _editState = VIEW_ONLY;
            }
            break;
        }
    }

    *_toggleEditState=false;
}




void Display::moveMarker() {
// calc new position
    _lcd.setCursor(0, _markerPosition);
    _lcd.print(" ");
    int newPos = _markerPosition + *markerIncrementer;
    _markerPosition = min((newPos), _markerPositionMax);
    _markerPosition = max((newPos), _markerPositionMin);
    _lcd.setCursor(0, _markerPosition);
    _lcd.write(byte(0));
}

void Display::printParameterValue() {
    _lcd.setCursor(_markerPosition, _valueColumnPos);
    _lcd.print(_parametersMemory[_activeParamIndex]);
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
        _lcd.setCursor(1, i);
        _lcd.print(_allUserParameters[(int)_mode->parameters[i]].lcdString);
    }
}
