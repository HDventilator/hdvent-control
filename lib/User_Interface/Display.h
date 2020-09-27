//
// Created by david on 24.04.20.
//

#ifndef HDVENT_CONTROL_DISPLAY_H
#define HDVENT_CONTROL_DISPLAY_H

#include <LiquidCrystal.h>
#include <User_Parameter.h>
#include <Ventilation_Modes.h>
#include "User_Input.h"

/*
float roundNumber(float a, uint8_t n){
    static int pow10[5] = {1, 10, 100, 1000, 10000};
    a = a*pow10[n];
    a = round(a);
    a = a/pow10[n];
    return a;
}
*/
const uint8_t N_DIAGNOSTIC_PARAMETERS=4;
class Display {
public:
    Display(LiquidCrystal &lcd, User_Parameter *allUserParameters, const VentilationMode *mode, int *cursorIncrementer,
            int *valueIncrementer, bool *toggleEditState, bool *toggleMenuState, User_Input *userInput,
            diagnosticParameters_t *diagnosticParameters);

    enum editState_t {EDIT_PARAMETER, EDIT_ALARM, NAVIGATE, VIEW_ONLY};
    enum menuState_t {EDIT_SETTINGS, VIEW};

    void setCursor(uint8_t col, uint8_t row);
    void update();
    void indexToCursorPosition(uint8_t i, uint8_t &col, uint8_t &row);
    void refreshDisplay();
    void printValue(float value);
    void displayParameterValue();
    void printParameterValue(float value);
    void printInactiveAlarm();
    uint8_t getParameterIndex();
    void safeParams();
    void loadParams();
    void printStaticText();
    void indexToTextPosition(uint8_t i, uint8_t &col, uint8_t &row);
    void printUserParamValues();
    void printAllEditMode();
    void printAllViewMode();
    void indexToParamValuePosition(uint8_t i, uint8_t &row, uint8_t &col);
    void setMode(const VentilationMode *mode);
    void printAlarmThreshold(Diagnostic_Parameter p);
    float _parametersMemory[4];
    void loadThresholds();
    void indexToAlarmValuePosition(uint8_t i, uint8_t &row, uint8_t &col);
    void indexToAlarmTextPosition(uint8_t i, uint8_t &row, uint8_t &col);
    void updateIndexes();
private:
    uint8_t _valueColumnPos=5;
    const VentilationMode *_mode;
    uint8_t _topRowIndex;
    uint8_t columnValues;
    uint8_t _paramIndex;
    //Diagnostic_Parameter &_currentAlarm;
    uint8_t _scrollingOffset;
    float _alarmValue;
    editState_t _editState;
    menuState_t _menuState;
     uint8_t  _navigationIndex;
     uint8_t _alarmIndex;
     uint8_t _cursorRow;
     uint8_t _cursorCol;
    int *_markerIncrementer;
    int *_valueIncrementer;
    bool *_toggleEditState;
    bool *_toggleMenuState;
    User_Input *_userInput;
    uint8_t  _markerPositionMax;
   uint8_t _markerPositionMin;
   uint8_t _allowedAlarmIndexes[N_DIAGNOSTIC_PARAMETERS];
   void moveMarker();
   LiquidCrystal _lcd;
   User_Parameter* _allUserParameters;
    diagnosticParameters_t* _diagnosticParameters;
   float _thresholdsMemoryUpper[N_DIAGNOSTIC_PARAMETERS];
   float _thresholdsMemoryLower[N_DIAGNOSTIC_PARAMETERS];

};


#endif //HDVENT_CONTROL_DISPLAY_H
