//
// Created by david on 24.04.20.
//

#include "Display.h"
#include <Arduino.h>
#include <LiquidCrystal.h>
#include <User_Input.h>

Display::Display(LiquidCrystal &lcd, User_Parameter *allUserParameters, const VentilationMode *mode,
                 int *cursorIncrementer,
                 int *valueIncrementer, bool *toggleEditState, bool *toggleMenuState, User_Input *userInput,
                 diagnosticParameters_t *diagnosticParameters) : _lcd(12, 11, 10, 9, 8, 7){
    _lcd = lcd;
    _lcd.begin(20, 4);
    byte FULL[8] = {0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff};
    /*byte CURSOR_SYMBOL[8] = {
            B00000,
            B00100,
            B00110,
            B11111,
            B00110,
            B00100,
            B00000,
            B00000
    };*/
    lcd.createChar(0, FULL);

    _mode = mode;
    _topRowIndex =0;
    _allUserParameters = allUserParameters;
    _diagnosticParameters = diagnosticParameters;
    _valueIncrementer = valueIncrementer;
    _toggleMenuState = toggleMenuState;
    _toggleEditState = toggleEditState;
    _markerIncrementer = cursorIncrementer;
    _valueIncrementer= valueIncrementer;
    _editState = NAVIGATE;
    _markerPositionMax =3;
    _markerPositionMin = 0;
    _scrollingOffset =0;
    _userInput = userInput;

    lcd.clear();
    printStaticText();
    lcd.home();
    lcd.cursor();
    for (int8_t i=0; i<_mode->nParams;i++){
        printValue(_allUserParameters[(int) _mode->parameters[i]].getValue());
    }
}


void Display::update() {
    _lcd.cursor();
    switch(_editState){
        case VIEW_ONLY:{
            moveMarker();
            if (*_toggleEditState){
                _editState=EDIT_ENTRY;
                *_toggleEditState=false;
            }

            break;
        }
        case NAVIGATE:
            //Serial.println(*_markerIncrementer);
            moveMarker();
            if (*_toggleEditState){
                loadParams();
                loadThresholds();
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
            //Serial.println(_parametersMemory[_activeParamIndex]);
            indexToParamValuePosition(_index, _cursorRow, _cursorCol);
            setCursor(_cursorCol, _cursorRow);
            printValue(_parametersMemory[_activeParamIndex]);
            *_valueIncrementer=0;

            if (*_toggleEditState) {
                _editState = NAVIGATE;
                *_toggleEditState=false;

            }
            else if (*_toggleMenuState) {
                //Serial.println("from edit to view_only");
                _editState = VIEW_ONLY;
                *_toggleMenuState=false;
                //_lcd.noCursor();
            }
            break;
        }
    }
    *_toggleEditState=false;
}


void Display::moveMarker() {
// calc new position

    //_index = _index%(_mode->nParams+N_DIAGNOSTIC_PARAMETERS);
    *_markerIncrementer = min(*_markerIncrementer, (_mode->nParams+N_DIAGNOSTIC_PARAMETERS*2 -1));
    *_markerIncrementer = max(*_markerIncrementer, 0);
    _index = *_markerIncrementer;
    //Serial.println(_index);
    //Serial.print("topRowIndex:\t");Serial.println(_topRowIndex);
    indexToCursorPosition(_index, _cursorCol, _cursorRow);

    if (_cursorRow > _topRowIndex+ 3){
        _topRowIndex = _cursorRow - 3;
        _lcd.clear();
        printStaticText();
    }
    if (_cursorRow < _topRowIndex){
        _topRowIndex = _cursorRow;
        _lcd.clear();
        printStaticText();
    }
    setCursor(_cursorCol, _cursorRow-_topRowIndex);
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

void Display::loadThresholds() {
    // save user parameters momentarily
    for (int i; i<N_DIAGNOSTIC_PARAMETERS; i++){
        _thresholdsMemoryLower[i]=_diagnosticParameters->arr[i].getMinAlarm();
        _thresholdsMemoryUpper[i]=_diagnosticParameters->arr[i].getMaxAlarm();
    }
}

void Display::setCursor(uint8_t col, uint8_t row) {
    //row = row - _topRowIndex;
    _lcd.setCursor(col, row);
}


void Display::printStaticText() {
    for (int i=0; i < (_mode->nParams); i++){
        indexToTextPosition(i, _cursorCol, _cursorRow);
        if (_topRowIndex <= _cursorRow && _cursorRow< _topRowIndex+4) {
            setCursor(_cursorCol, _cursorRow-_topRowIndex);
            _lcd.print(_allUserParameters[(int)_mode->parameters[i]].lcdString);
        }
        indexToParamValuePosition(i, _cursorRow, _cursorCol);
        if (_topRowIndex <= _cursorRow && _cursorRow< _topRowIndex+4) {
            setCursor(_cursorCol, _cursorRow-_topRowIndex);
            printValue(_allUserParameters[(int) _mode->parameters[i]].getValue());
        }
    }

    for (int i=0; i < N_DIAGNOSTIC_PARAMETERS; i++){
        //Serial.print(_diagnosticParameters->arr[i].lcdString);
        indexToTextPosition(i, _cursorCol, _cursorRow);
        _cursorRow = _cursorRow + _mode->nParams;
        if (_topRowIndex <= _cursorRow && _cursorRow< _topRowIndex+4) {
            Diagnostic_Parameter diagnosticParameter = _diagnosticParameters->arr[i];
            setCursor(_cursorCol, _cursorRow-_topRowIndex);
            Serial.println(diagnosticParameter.getLoAlarmSet());
            _lcd.print(diagnosticParameter.lcdString);
            _lcd.print(" Lo ");
            switch (diagnosticParameter.getLoAlarmSet()){
                case Diagnostic_Parameter::ACTIVE:
                    printValue(diagnosticParameter.getMinAlarm());
                    break;
                case Diagnostic_Parameter::INACTIVE:
                    for (int i=0; i<4; i++){
                        _lcd.print((char)0);
                    }
                    break;
                default:
                    break;
            }

            _lcd.print(" Hi ");

            switch (diagnosticParameter.getHiAlarmSet()){
                case Diagnostic_Parameter::ACTIVE:
                    printValue(diagnosticParameter.getMaxAlarm());
                    break;
                case Diagnostic_Parameter::INACTIVE:
                    for (int i=0; i<4; i++){
                        _lcd.print((char)0);
                    }
                    break;
                default:
                    break;
            }
        }
    }
}



void Display::indexToTextPosition(uint8_t i, uint8_t &col, uint8_t &row) {
    row =i;
    col = 0;// i/nRows * 10;
}

void Display::indexToParamValuePosition(uint8_t i, uint8_t &row, uint8_t &col) {
    row =i;
    col = 5;// i/nRows * 10;

}

void Display::indexToAlarmValuePosition(uint8_t i, uint8_t &row, uint8_t &col) {
    row =i;
    if (i % 2) {
        /* i is odd */
        col=5;
    }
    else {
        col=14;
    }
    col = 5;// i/nRows * 10;
}


void Display::indexToCursorPosition(uint8_t i, uint8_t &col, uint8_t &row) {
    uint8_t offset = _mode->nParams;
    if (_index < offset) {
        row = i;
        col = 0;
    }
    else {
        row = (i-offset)/2+offset;
        if (i % 2) {
            /* i is odd */
            col=14;
        }
        else {
            col=5;
        }
    }
}

