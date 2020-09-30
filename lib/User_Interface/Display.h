//
// Created by david on 24.04.20.
//

#ifndef HDVENT_CONTROL_DISPLAY_H
#define HDVENT_CONTROL_DISPLAY_H

#include <LiquidCrystal.h>
#include <User_Parameter.h>
#include <Ventilation_Modes.h>
#include "User_Input.h"
#include <TimedToggler.h>

const uint8_t nDiagnosticParameters=4;
class Display {
public:
    Display(LiquidCrystal &lcd, User_Parameter *allUserParameters, const VentilationMode *mode, int *cursorIncrementer,
            int *valueIncrementer, bool *toggleEditState, diagnosticParameters_t *diagnosticParameters);

    enum editState_t {EDIT_PARAMETER, EDIT_ALARM, NAVIGATE};
    enum menuState_t {UNSAVED_SETTINGS, VIEW};

    void setCursor(uint8_t col, uint8_t row);
    void update(bool confirm, bool cancel);
    void indexToCursorPosition(uint8_t i, uint8_t &col, uint8_t &row);

    void printValue(float value);

    void printInactiveAlarm();
    void safeParams();
    void resetParams();
    void loadParams();
    void printStaticText();
    void resetCursor();
    void indexToTextPosition(uint8_t i, uint8_t &col, uint8_t &row);
    void indexToParamValuePosition(uint8_t i, uint8_t &row, uint8_t &col);
    float _parametersMemory[4]{};

    void indexToAlarmValuePosition(uint8_t i, uint8_t &row, uint8_t &col);

    void updateIndexes();
    uint8_t nActiveDiagnosticParameters;

    void printScrollIndicator();

private:

    uint8_t  _header;
    uint8_t _nRows;
    uint8_t _topRowIndex;

    uint8_t _paramIndex;
    float _alarmValue;
    editState_t _editState;
    menuState_t _menuState;
     uint8_t  _navigationIndex;
     uint8_t _alarmIndex;
     uint8_t _cursorRow;
     uint8_t _cursorCol;
    TimedToggler _timedToggler;
   uint8_t _allowedAlarmIndexes[nDiagnosticParameters];
   void moveMarker();
   LiquidCrystal &_lcd;
   User_Parameter* _allUserParameters;
    const VentilationMode *_mode;
    int *_markerIncrementer;
    int *_valueIncrementer;
    bool *_toggleEditState;
   diagnosticParameters_t* _diagnosticParameters;
   Stopwatch stopwatch;

    void printOKCancel(bool doShow);
};


#endif //HDVENT_CONTROL_DISPLAY_H
