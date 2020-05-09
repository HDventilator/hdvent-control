
#include "Wire.h"
#include <Arduino.h>
#include "Honeywell_SSC.h"

#define HSCDRRN400MD2A3_I2C 0x48 // each I2C object has a unique bus address, the DS1307 is 0x68
#define OUTPUT_MIN 1638.4        // 1638 counts (10% of 2^14 counts or 0x0666)
#define OUTPUT_MAX 14745.6       // 14745 counts (90% of 2^14 counts or 0x3999)
#define PRESSURE_MIN -100        // min is 0 for sensors that give absolute values
#define PRESSURE_MAX 100

Honeywell_SSC hps = Honeywell_SSC(HSCDRRN400MD2A3_I2C, 0, PRESSURE_MIN, PRESSURE_MAX, OUTPUT_MIN, OUTPUT_MAX);

void setup()
{
    Wire.begin(); // wake up I2C bus
    delay (500);
    Serial.begin(115200);
}

void loop()
{
    sensors_event_t event;
    hps.readSensor();
    Serial.print("pressure    (BAR) ");
    Serial.println(event.pt.pressure);

    Serial.print("temperature (C) ");
    Serial.println(event.pt.temperature);
    Serial.println("");
    delay (500);
}