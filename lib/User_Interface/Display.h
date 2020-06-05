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

class Display {
public:
    Display(LiquidCrystal &lcd, User_Parameter *allUserParameters, const VentilationMode *mode,
            int *cursorIncrementer, int *valueIncrementer, bool *toggleEditState, bool *toggleMenuState,
            User_Input* userInput);
    Display(LiquidCrystal &lcd, User_Parameter *allUserParameters, VentilationMode *mode,
            User_Input *userInput);
    enum editState_t {EDIT_ENTRY, NAVIGATE, VIEW_ONLY};
    enum menuState_t {EDIT_SETTINGS, VIEW};

    void updateDisplay();
    void refreshDisplay();
    void printParameterValue(float value);
    void displayParameterValue();

    uint8_t getParameterIndex();
    void safeParams();
    void loadParams();
    void printStaticText();
    void incrementToPos(uint8_t i);
    void printUserParamValues();
    void printAllEditMode();

    void setMode(const VentilationMode *mode);

    float _parametersMemory[4];
private:
    uint8_t _valueColumnPos=5;
    const VentilationMode *_mode;
    uint8_t columnValues;
    uint8_t _activeParamIndex;
    editState_t _editState;
    menuState_t _menuState;
     uint8_t  _index;
     uint8_t _cursorRow;
     uint8_t _cursorCol;
    int *markerIncrementer;
    int *_valueIncrementer;
    bool *_toggleEditState;
    bool *_toggleMenuState;
    User_Input *_userInput;
    uint8_t  _markerPositionMax;
   uint8_t _markerPositionMin;
   void moveMarker();
   LiquidCrystal _lcd;
   User_Parameter* _allUserParameters;



};


#endif //HDVENT_CONTROL_DISPLAY_H
