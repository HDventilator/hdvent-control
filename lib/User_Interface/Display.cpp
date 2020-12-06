//
// Created by david on 24.04.20.
//

#include "Display.h"
#include <Arduino.h>
#include <LiquidCrystal.h>
#include <User_Input.h>
byte SYMBOL_SCROLL_DOWN[8] = {
        B00000,
        B00000,
        B00000,
        B00000,
        B00000,
        B10001,
        B01010,
        B00100
};
byte SYMBOL_SCROLL_UPDOWN[8] = {
        B00100,
        B01010,
        B10001,
        B00000,
        B00000,
        B10001,
        B01010,
        B00100
};

Display::Display(LiquidCrystal &lcd, Parameter_Container<N_USER_PARAMETERS> & allUserParameters, const VentilationMode *mode,
                 diagnosticParameters_t *diagnosticParameters)
                 : _lcd(lcd)
                 , _allUserParams(allUserParameters)
                 , _mode(mode)
                 , _diagnosticParameters(diagnosticParameters)
{
    _lcd.begin(20, 4);
    lcd.createChar(1, SYMBOL_SCROLL_DOWN);
    lcd.createChar(2, SYMBOL_SCROLL_UPDOWN);
    _header =1;
    _topRowIndex =0;
    _menuState = VIEW;
    _editState = NAVIGATE;
    _nRows=4-_header;
    _timedToggler.set(700);
    lcd.clear();
    printStaticText();
    lcd.home();
    lcd.cursor();

    uint8_t j =0;
    for (int i=0; i<nDiagnosticParameters; i++) {
        if ((diagnosticParameters->arr[i].getHiAlarmSet() != Diagnostic_Parameter::DISABLED) ||
                (diagnosticParameters->arr[i].getLoAlarmSet() != Diagnostic_Parameter::DISABLED)){
            _allowedAlarmIndexes[j] = i;
            //Serial.println(diagnosticParameters->arr[i].lcdString);
            j++;
        }
    }
    nActiveDiagnosticParameters =j;
}



void Display::blinkText(char* text, uint8_t col, uint8_t row){
    if (_timedToggler.getEvent()){
        _lcd.setCursor(col, row);
        if (_timedToggler.getState()) {
            _lcd.print(text);
        }
        else {
            for (uint8_t i=0; i<strlen_P(text); i++) {
                _lcd.print(" ");
            }
        }
    }
}

void Display::deleteText(char* text, uint8_t col, uint8_t row){
    _lcd.setCursor(col, row);
    for (uint8_t i=0; i<strlen_P(text); i++) {
        _lcd.print(" ");
    }
}
void Display::update(bool confirm, bool cancel, bool toggle, int8_t delta) {
    int8_t navigationDelta = delta;
    _lcd.cursor();
    updateIndexes();
    switch(_menuState){
        case UNSAVED_SETTINGS:
            blinkText("OK/CANCEL", 10,0);

            if (cancel){
                Serial.println("cancel");
                resetParams();
            }
            else if (confirm){
                Serial.println("confirm");
                safeParams();
                _settingsWereSaved=true;
            }
            if (confirm||cancel){
                deleteText("OK/CANCEL", 10,0);
                _menuState = VIEW;
                _editState = NAVIGATE;
                _markerIncrementer=0;
                printStaticText();
            }
            break;

        case VIEW:
            _settingsWereSaved=false;
            break;
        default:
            break;
    }

    switch(_editState){
        case NAVIGATE:
            // calc new position
            _markerIncrementer = _markerIncrementer + navigationDelta;
           // Serial.println(_markerIncrementer);
            _markerIncrementer = min(_markerIncrementer, (_mode->nParams + nActiveDiagnosticParameters * 2 - 1));
            _markerIncrementer = max(_markerIncrementer, 0);
            _navigationIndex = _markerIncrementer;

            moveMarker();
            if (toggle) {
                if (_navigationIndex < _mode->nParams) {
                    _menuState = UNSAVED_SETTINGS;
                    _editState = EDIT_PARAMETER;
                    _allUserParams.getActive(_paramIndex).isGettingEdited = true;
                }
                else {
                    _editState = EDIT_ALARM;
                    Diagnostic_Parameter & param = _diagnosticParameters->arr[_alarmIndex] ;

                    if (!((_navigationIndex - _mode->nParams) % 2)) {
                        _alarmValue = param.getLoAlarm();
                    } else{
                        _alarmValue = param.getHiAlarm();
                    }
                }
            }
            break;

        case EDIT_PARAMETER:
        {
            float _newValue = _allUserParams.getActive(_paramIndex).getDialValue() + (float)delta * _allUserParams.getActive(_paramIndex).increment;
            _newValue = min(_newValue, _allUserParams.getActive(_paramIndex).getMax());
            _newValue = max(_newValue, _allUserParams.getActive(_paramIndex).getMin());
            _allUserParams.getActive(_paramIndex).setDialValue(_newValue);
            indexToParamValuePosition(_navigationIndex, _cursorRow, _cursorCol);
            setCursor(_cursorCol, _cursorRow);
            printValue(_allUserParams.getActive(_paramIndex).getDialValue());

            if (toggle) {
                _allUserParams.getActive(_paramIndex).isGettingEdited = false;
                _editState = ENTER_NAVIGATE;
            }
            break;}

        case EDIT_ALARM: {
            Diagnostic_Parameter & param = _diagnosticParameters->arr[_alarmIndex];
            _alarmValue = _alarmValue + (float) delta * param.getIncrement();
            indexToAlarmValuePosition(_navigationIndex, _cursorRow, _cursorCol);
            setCursor(_cursorCol, _cursorRow);


            Diagnostic_Parameter::AlarmSetting alarmSetting;
            if (!((_navigationIndex - _mode->nParams) % 2)) {
                if (_alarmValue < param.getMinAlarm()){
                    alarmSetting = Diagnostic_Parameter::INACTIVE;
                    _alarmValue = param.getMinAlarm() - param.getIncrement();
                    printInactiveAlarm();
                }
                else {
                    alarmSetting = Diagnostic_Parameter::ACTIVE;
                    _alarmValue = min(_alarmValue, param.getMaxAlarm());
                    printValue(_alarmValue);
                }

            }
            else {
                if (_alarmValue > param.getMaxAlarm()){
                    alarmSetting = Diagnostic_Parameter::INACTIVE;
                    _alarmValue = param.getMaxAlarm() + param.getIncrement();
                    printInactiveAlarm();
                }
                else {
                    alarmSetting = Diagnostic_Parameter::ACTIVE;
                    _alarmValue = max(_alarmValue, param.getMinAlarm());
                    printValue(_alarmValue);
                }
            }

            if (toggle) {
                if (!((_navigationIndex - _mode->nParams) % 2)) {
                    param.setLoAlarm(_alarmValue);
                    param.setLoAlarmSet(alarmSetting);
                    //Serial.println("set lo alarm");
                } else{
                    param.setHiAlarmSet(alarmSetting);
                    param.setHiAlarm(_alarmValue);
                    //Serial.println("set hi alarm");
                }
                _editState = ENTER_NAVIGATE;
            }
            break;}

        case ENTER_NAVIGATE:
            _editState = NAVIGATE;
            _allUserParams.getActive(_paramIndex).isGettingEdited = false;
            _alarmValue =0;
            _markerIncrementer = _navigationIndex;
            break;

        default:
            break;
    }
}


void Display::moveMarker() {


    indexToCursorPosition(_navigationIndex, _cursorCol, _cursorRow);
        if (_cursorRow > _topRowIndex+ _nRows - 1){
        _topRowIndex = _cursorRow - _nRows +1 ;
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
    for (int i=0; i<(_mode->nParams); i++){
        _allUserParams.getActive(i).saveValue();
    }
}

void Display::loadParams() {
    for (int i; i<(_mode->nParams); i++){
        _allUserParams.getActive(i).resetDialValue();
    }
}

void Display::setCursor(uint8_t col, uint8_t row) {
    row = row - _topRowIndex+_header;
    _lcd.setCursor(col, row);
}


void Display::printStaticText() {
    printScrollIndicator();
    _lcd.setCursor(0,0);
    _lcd.print(_mode->lcdString);

    for (int i=0; i < (_mode->nParams); i++){
        indexToTextPosition(i, _cursorCol, _cursorRow);
        if (_topRowIndex <= _cursorRow && _cursorRow< _topRowIndex+4) {
            setCursor(_cursorCol, _cursorRow);
            _lcd.print(_allUserParams.getActive(i).lcdString);
        }
        indexToParamValuePosition(i, _cursorRow, _cursorCol);
        if (_topRowIndex <= _cursorRow && _cursorRow< _topRowIndex+4) {
            setCursor(_cursorCol, _cursorRow);
            printValue(_allUserParams[(int) _mode->parameters[i]].getDialValue());
        }
    }

    for (int j=0; j < nActiveDiagnosticParameters; j++){
        uint8_t i = _allowedAlarmIndexes[j];
        indexToTextPosition(j, _cursorCol, _cursorRow);
        _cursorRow = _cursorRow + _mode->nParams;
        if (_topRowIndex-_header <= _cursorRow && _cursorRow< _topRowIndex+_nRows) {
            Diagnostic_Parameter & diagnosticParameter = _diagnosticParameters->arr[i];
            setCursor(_cursorCol, _cursorRow);
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
        //_lcd.print((char)0);
        _lcd.print((char)B11111111);
    }
}

void Display::updateIndexes() {
    _alarmIndex = (_navigationIndex - _mode->nParams ) / 2;
    _alarmIndex = _allowedAlarmIndexes[_alarmIndex];
    _paramIndex = _navigationIndex;
}

void Display::printScrollIndicator() {
    if (_topRowIndex>0) {
        if (_topRowIndex < nActiveDiagnosticParameters + _mode->nParams - _nRows) {
            _lcd.setCursor(19, 0);
            _lcd.write(2);
        } else {
            _lcd.setCursor(19, 0);
            _lcd.write(B1011110);
        }
    } else if (_topRowIndex < nActiveDiagnosticParameters + _mode->nParams - _nRows){
        _lcd.setCursor(19, 0);
        _lcd.write(1);
    }
}

void Display::resetParams() {
    for (int i; i<(_mode->nParams); i++){
        _allUserParams.getActive(i).resetDialValue();
    }
}

void Display::resetCursor() {
    _lcd.setCursor(_cursorCol,_cursorRow);
}

Display::menuState_t Display::getMenuState() const {
    return _menuState;
}

bool Display::getSavingEvent() {
    return _settingsWereSaved;
    }
