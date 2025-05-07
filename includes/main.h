#pragma once
// Triac
#define ZC_PIN 8
#define HEATER_PIN 9 //Triac output for heater
#define FAN_PIN 10 //Triac output for fan

#define NCHAN 2
#define CAL_GAIN 1.0
#define UV_OFFSET ( 0 )
#define MIN_DELAY 300   // ms between ADC samples (tested OK at 270)
#define D_MULT 0.001 // multiplier to convert temperatures from int to float

#define ONE_WIRE_BUS 2

