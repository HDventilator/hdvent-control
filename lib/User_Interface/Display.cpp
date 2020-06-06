//
// Created by david on 24.04.20.
//

#include "Display.h"
#include <Arduino.h>
#include <LiquidCrystal.h>
#include <User_Input.h>

Display::Display(LiquidCrystal &lcd, User_Parameter *allUserParameters, const VentilationMode *mode,
                 int *cursorIncrementer, int *valueIncrementer, bool *toggleEditState, bool *toggleMenuState,
                 User_Input* userInput) : _lcd(12, 11, 10, 9, 8, 7){
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
    _userInput = userInput;
    lcd.clear();
    printStaticText();
    lcd.cursor();
    for (int8_t i=0; i<_mode->nParams;i++){
        printValue(_allUserParameters[(int) _mode->parameters[i]].getValue());
    }

}

Display::Display(LiquidCrystal &lcd, User_Parameter *allUserParameters, VentilationMode *mode,
                 User_Input *userInput) : _lcd(12, 11, 10, 9, 8, 7) {
    _lcd = lcd;
    _lcd.begin(20, 4);
    _mode = mode;
    _allUserParameters = allUserParameters;
    _userInput = userInput;
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

            _activeParamIndex = _index;
            _parametersMemory[_activeParamIndex]+=
                    (float) *_valueIncrementer * _allUserParameters[(int)_mode->parameters[_activeParamIndex]].increment;
            Serial.println(_parametersMemory[_activeParamIndex]);
            printValue(_parametersMemory[_activeParamIndex]);
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
    _index += *markerIncrementer;
    _index = _index%_mode->nParams;
    *markerIncrementer=0;
    incrementToPos(_index);
    _lcd.setCursor(_cursorCol, _cursorRow);
}

void Display::printValue(float value) {

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

void Display::printParameterValue(float value, uint8_t n) {

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
        incrementToPos(i);
        _lcd.setCursor(_cursorCol, _cursorRow);
        _lcd.print(_allUserParameters[(int)_mode->parameters[i]].lcdString);
        printValue(_allUserParameters[(int) _mode->parameters[i]].getValue());
    }
}

void Display::printUserParamValues() {
    for (int i=0; i < (_mode->nParams); i++){
        incrementToPos(i);
        _lcd.setCursor(_cursorCol, _cursorRow);
        _lcd.print(_allUserParameters[(int)_mode->parameters[i]].lcdString);
        printValue(_allUserParameters[(int) _mode->parameters[i]].getDialValue());
    }
}

void Display::incrementToPos(uint8_t i) {
    uint8_t nRows=3;

    _cursorRow = i%nRows + 1;
    _cursorCol = i/nRows * 10;
}

void Display::setMode(const VentilationMode *mode) {
    _mode = mode;
}

void Display::refreshDisplay() {
    switch (_userInput->getInputState()){
        case User_Input::EDIT_VENTILATION_MODE:
            break;

        case User_Input::VIEW_SETTINGS:
            //printAllViewMode();
            break;

        case User_Input::ENTER_EDIT:
            _lcd.clear();
            _lcd.setCursor(0,0);
            break;

        case User_Input::EDIT_SETTINGS:
            printAllEditMode();
            break;

        case User_Input::SAVE_SETTINGS:
            _lcd.clear();
            break;

        case User_Input::ENTER_VIEW_SETTINGS:
            _lcd.clear();
            printAllViewMode();
    }

}



void Display::printAllEditMode() {
    for (int i=0; i < (_mode->nParams); i++){
        uint8_t nCols=4;
        _cursorCol = i*5;
        _cursorRow = 1;
        _lcd.setCursor(_cursorCol, _cursorRow);
        _lcd.print(_allUserParameters[(int)_mode->parameters[i]].lcdString);
        _lcd.setCursor(_cursorCol, _cursorRow+1);
        printValue(_allUserParameters[(int) _mode->parameters[i]].getValue());
        _lcd.setCursor(_cursorCol, _cursorRow+2);
        //_lcd.print("(");
        printValue(_allUserParameters[(int) _mode->parameters[i]].getDialValue());
    }
}

void Display::printAllViewMode() {
    for (int i=0; i < (_mode->nParams); i++){
        uint8_t nCols=4;
        _cursorCol = i*5;
        _cursorRow = 1;
        _lcd.setCursor(_cursorCol, _cursorRow+1);
        _lcd.print(_allUserParameters[(int)_mode->parameters[i]].lcdString);
        _lcd.setCursor(_cursorCol, _cursorRow+2);
        printValue(_allUserParameters[(int) _mode->parameters[i]].getValue());
    }
}
