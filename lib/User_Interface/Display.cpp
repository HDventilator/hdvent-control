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
                 diagnosticParameters_t *diagnosticParameters) : _lcd(12, 11, 10, 9, 8, 7)                                                                 {
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
    uint8_t j =0;
    for (int i=0; i<nDiagnosticParameters; i++) {
        if ((diagnosticParameters->arr[i].getHiAlarmSet() != Diagnostic_Parameter::DISABLED) ||
                (diagnosticParameters->arr[i].getLoAlarmSet() != Diagnostic_Parameter::DISABLED)){
            _allowedAlarmIndexes[j] = i;
            Serial.println(diagnosticParameters->arr[i].lcdString);
            j++;
        }
    }
    nActiveDiagnosticParameters =j;

}


void Display::update() {
    _lcd.cursor();
    updateIndexes();
    switch(_editState){
        case VIEW_ONLY:{
            moveMarker();
            if (*_toggleEditState){
                _editState=EDIT_PARAMETER;
                *_toggleEditState=false;
            }

            break;
        }
        case NAVIGATE:
            moveMarker();
            if (*_toggleEditState) {
                if (_navigationIndex < _mode->nParams) {
                    loadParams();
                    loadThresholds();
                    _editState = EDIT_PARAMETER;
                    *_toggleEditState = false;
                }
                else {
                    *_valueIncrementer =0;
                    _editState = EDIT_ALARM;
                    _alarmIndex = (_navigationIndex - _mode->nParams ) / 2;
                    Diagnostic_Parameter&param = _diagnosticParameters->arr[_alarmIndex] ;

                    if (!((_navigationIndex - _mode->nParams) % 2)) {
                        _alarmValue = param.getLoAlarm();
                    } else{
                        _alarmValue = param.getHiAlarm();
                    }
                    *_toggleEditState = false;
                }
            }
            else if (*_toggleMenuState){
                _editState=VIEW_ONLY;
                *_toggleMenuState=false;
            }
            break;

        case EDIT_PARAMETER: {

            _parametersMemory[_paramIndex]+=
                    (float) *_valueIncrementer * _allUserParameters[(int)_mode->parameters[_paramIndex]].increment;

            indexToParamValuePosition(_navigationIndex, _cursorRow, _cursorCol);
            setCursor(_cursorCol, _cursorRow);
            printValue(_parametersMemory[_paramIndex]);
            *_valueIncrementer=0;

            if (*_toggleEditState) {
                _editState = NAVIGATE;
                *_markerIncrementer = _navigationIndex;
                *_toggleEditState=false;

            }
            else if (*_toggleMenuState) {
                _editState = VIEW_ONLY;
                *_toggleMenuState=false;
                //_lcd.noCursor();
            }
            break;

        case EDIT_ALARM:
            Diagnostic_Parameter&param = _diagnosticParameters->arr[_alarmIndex];
            _alarmValue = _alarmValue + (float) *_valueIncrementer * param.getIncrement();
            indexToAlarmValuePosition(_navigationIndex, _cursorRow, _cursorCol);
            setCursor(_cursorCol, _cursorRow);
            *_valueIncrementer =0;
            /*
            Serial.print("alarm Index:\t"); Serial.println(_alarmIndex);
            Serial.print("index:\t"); Serial.println(_navigationIndex);
*/
            if (!((_navigationIndex - _mode->nParams) % 2)) {
                if (_alarmValue < param.getMinAlarm()){
                    param.setLoAlarmSet(Diagnostic_Parameter::INACTIVE);
                    _alarmValue = param.getMinAlarm() - param.getIncrement();
                    printInactiveAlarm();
                }
                else {
                    param.setLoAlarmSet(Diagnostic_Parameter::ACTIVE);
                    _alarmValue = min(_alarmValue, param.getMaxAlarm());
                    printValue(_alarmValue);
                }

            }
            else {
                if (_alarmValue > param.getMaxAlarm()){
                    param.setHiAlarmSet(Diagnostic_Parameter::INACTIVE);
                    _alarmValue = param.getMaxAlarm() + param.getIncrement();
                    printInactiveAlarm();
                }
                else {
                    param.setHiAlarmSet(Diagnostic_Parameter::ACTIVE);
                    _alarmValue = max(_alarmValue, param.getMinAlarm());
                    printValue(_alarmValue);
                }
            }


            if (*_toggleEditState) {
                if (!((_navigationIndex - _mode->nParams) % 2)) {
                    param.setLoAlarm(_alarmValue);
                    //Serial.println("set lo alarm");
                } else{
                    param.setHiAlarm(_alarmValue);
                    //Serial.println("set hi alarm");
                }
                _editState = NAVIGATE;
                _alarmValue =0;
                *_markerIncrementer = _navigationIndex;
                *_toggleEditState=false;
            }
            break;
        }
    }
    *_toggleEditState=false;
}


void Display::moveMarker() {
// calc new position

    //_navigationIndex = _navigationIndex%(_mode->nParams+nDiagnosticParameters);
    *_markerIncrementer = min(*_markerIncrementer, (_mode->nParams + nActiveDiagnosticParameters * 2 - 1));
    *_markerIncrementer = max(*_markerIncrementer, 0);
    _navigationIndex = *_markerIncrementer;

    indexToCursorPosition(_navigationIndex, _cursorCol, _cursorRow);

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
    setCursor(_cursorCol, _cursorRow);
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
    for (int i; i < nActiveDiagnosticParameters; i++){
        _thresholdsMemoryLower[i]=_diagnosticParameters->arr[i].getMinAlarm();
        _thresholdsMemoryUpper[i]=_diagnosticParameters->arr[i].getMaxAlarm();
    }
}

void Display::setCursor(uint8_t col, uint8_t row) {
    row = row - _topRowIndex;
    _lcd.setCursor(col, row);
}


void Display::printStaticText() {
    for (int i=0; i < (_mode->nParams); i++){
        indexToTextPosition(i, _cursorCol, _cursorRow);
        if (_topRowIndex <= _cursorRow && _cursorRow< _topRowIndex+4) {
            setCursor(_cursorCol, _cursorRow);
            _lcd.print(_allUserParameters[(int)_mode->parameters[i]].lcdString);
        }
        indexToParamValuePosition(i, _cursorRow, _cursorCol);
        if (_topRowIndex <= _cursorRow && _cursorRow< _topRowIndex+4) {
            setCursor(_cursorCol, _cursorRow);
            printValue(_allUserParameters[(int) _mode->parameters[i]].getValue());
        }
    }

    for (int j=0; j < nActiveDiagnosticParameters; j++){
        uint8_t i = _allowedAlarmIndexes[j];
        indexToTextPosition(j, _cursorCol, _cursorRow);
        _cursorRow = _cursorRow + _mode->nParams;
        //Serial.print(i); Serial.print(":\t");
        if (_topRowIndex <= _cursorRow && _cursorRow< _topRowIndex+4) {
            Diagnostic_Parameter diagnosticParameter = _diagnosticParameters->arr[i];
            setCursor(_cursorCol, _cursorRow);
            //Serial.println(_cursorRow);
            _lcd.print(diagnosticParameter.lcdString);
            _lcd.print(" Lo ");
            switch (diagnosticParameter.getLoAlarmSet()){
                case Diagnostic_Parameter::ACTIVE:
                    printValue(diagnosticParameter.getLoAlarm());
                    break;
                case Diagnostic_Parameter::INACTIVE:
                    printInactiveAlarm();
                    break;
                default:
                    break;
            }

            _lcd.print(" Hi ");
            switch (diagnosticParameter.getHiAlarmSet()){
                case Diagnostic_Parameter::ACTIVE:
                    printValue(diagnosticParameter.getHiAlarm());
                    break;
                case Diagnostic_Parameter::INACTIVE:
                    printInactiveAlarm();
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
    uint8_t offset = _mode->nParams;
    row = (i-offset)/2+offset;
    if (i % 2) {
        /* i is odd */
        col=8;
    }
    else {
        col=16;
    }
}


void Display::indexToCursorPosition(uint8_t i, uint8_t &col, uint8_t &row) {
    uint8_t offset = _mode->nParams;
    if (_navigationIndex < offset) {
        row = i;
        col = 0;
    }
    else {
        row = (i-offset)/2+offset;
        if (i % 2) {
            /* i is odd */
            col=5;
        }
        else {
            col=13;
        }
    }
}

void Display::printInactiveAlarm() {
    for (int i=0; i<4; i++){
        _lcd.print((char)0);
    }
}

void Display::updateIndexes() {
    _alarmIndex = (_navigationIndex - _mode->nParams ) / 2;
    _alarmIndex = _allowedAlarmIndexes[_alarmIndex];
    _paramIndex = _navigationIndex;
}



