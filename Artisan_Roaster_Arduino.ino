#include <Arduino.h>
#include "includes/main.h"
#include "SerialCommand.h"
#include "TriacDimmer.h"
#include "cADC.h"
#include "TypeK.h"
#include "DS18B20.h"

float ET, BT, AT, tempC; // current temps in float
int16_t fan = 0;
int16_t pwr = 0;

int32_t v;

OneWire oneWire(ONE_WIRE_BUS);
DS18B20 sensor(&oneWire); // DS18B20 sensor(ONE_WIRE_BUS); // DS18B20 on pin 2

cADC adc(A_ADC); // MCP3424 using the default of A4(SDA), A5(SCL)

TypeK tc[NCHAN]; // array of pointers to thermocouples

SerialCommand myCMD; // The  SerialCommand object
// The SerialCommand object is used to parse the incoming serial data

void unrecognized(const char *command)
{
	Serial.print("Unrecognized argument: ");
	Serial.println(command);
}

void process_FAN()
{
	char *arg;
	float d_fl;
	uint8_t d_int;
	int32_t d = 0;		// between 0 and 100%
	arg = myCMD.next(); // Get the next argument from the SerialCommand object buffer
	if (arg != NULL)	// As long as it exists, take it
	{
		d = atol(arg);
		d_fl = constrain(d, 0, 100);
		d_int = constrain(d, 0, 100);
		// TriacDimmer::setBrightness(FAN_PIN, d_fl/100);
		TriacDimmer::setDuty(FAN_PIN, d_int);
		fan = d_int;
	}
}

void process_PWR()
{
	char *arg;
	uint8_t d_in;
	int32_t d = 0;		// between 0 and 100%
	arg = myCMD.next(); // Get the next argument from the SerialCommand object buffer
	if (arg != NULL)	// As long as it exists, take it
	{
		d = atol(arg);
		d_in = constrain(d, 0, 100);
		// TriacDimmer::setBrightness(HEATER_PIN, d_fl/100);
		TriacDimmer::setICC(d_in);
		pwr = d_in;
	}
}

void process_READ()
{
	// printf("0.0,%1.1f,%1.1f,%d,%d,55,%1.1f\n", ET, BT, pwr, fan, current_temp_ds_C);
	Serial.print("0.0,");
	Serial.print(ET, 1);
	Serial.print(",");
	Serial.print(BT, 1);
	Serial.print(",");
	Serial.print(pwr);
	Serial.print(",");
	Serial.print(fan);
	Serial.print(",55,");
	Serial.println(AT, 1);

	// Artisan says: response: list ["t0","t1","t2","t3","t4"]  with t0 = internal temp; t1 = ET; t2 = BT, t3 = chan3, t4 = chan4 on "CHAN;1234" if ArduinoTC4_34 is configured
	// after PID_ON: + [,"Heater", "Fan", "SV"]
}

void process_CHAN()
{
	char *arg;
	arg = myCMD.next(); // Get the next argument from
	if (arg != NULL)	// As long as it existed, take it
	{
		Serial.print("# Active channels set to ");
		Serial.println(arg);
		// printf("# Active channels set to %s\n", arg);
	}
}

void setup()
{
	Serial.begin(115200);
	Serial.println("Starting Artisan Roaster Arduino");
	myCMD.setDefaultHandler(unrecognized);	//
	myCMD.addCommand("PWR", process_PWR);	//
	myCMD.addCommand("FAN", process_FAN);	//
	myCMD.addCommand("READ", process_READ); //
	myCMD.addCommand("CHAN", process_CHAN); // register parse handlers
	myCMD.clearBuffer();

	TriacDimmer::begin(400, 2000, 0.81, 0.15); //>81 full ON or <1 full OFF
	// default is uint16_t pulse_length = 20, uint16_t min_trigger = 2000, float on_thresh = 2.0, float off_thresh = 0.01
	// 400 is 200us
	delay(500); // let's get a few cycles before we read the period
	// Serial.print("TriacDimmer period: ");
	// Serial.println(TriacDimmer::getPeriod());
	//  all Triacs are off

	TriacDimmer::setBrightness(FAN_PIN, 0);
	TriacDimmer::setBrightness(HEATER_PIN, 0);

	sensor.begin();
	sensor.setResolution(12);

	Wire.begin();
	adc.setCal(CAL_GAIN, UV_OFFSET);

	// setup fake ZC
	// WGM22/WGM21/WGM20 all set -> Mode 7, fast PWM
	/*TCCR2A = 0;
	TCCR2A = (1 << COM2B1) + (1 << WGM21) + (1 << WGM20); // Set OC2B at bottom, clear OC2B at compare match
	TCCR2B = 0;
	TCCR2B = (1 << CS22) + (1 << CS21) + (1 << CS20) + (1 << WGM22); // prescaler = 1024;

	OCR2A = 155; // period 10ms
	OCR2B = 137; // 1 count = 64us we want logic H for 8.8ms --> 137

	DDRD |= (1 << PD3); // signal is on PIN PD3 (OC2B)
	*/
}

void loop()
{
	// delay(1000);
	// Serial.println(TriacDimmer::getPeriod());
	myCMD.readSerial();			  //  parse commands
	sensor.requestTemperatures(); // get DS18B20 temperature

	for (int j = 0; j < NCHAN; j++)
	{										 // one-shot conversions on both chips
		adc.nextConversion(j);				 // start ADC conversion on channel j
		delay(MIN_DELAY);					 // wait for conversion to finish
		v = adc.readuV();					 // retrieve microvolt sample from MCP3424
		tempC = tc[j].Temp_C(0.001 * v, AT); //--> crashes and reboots
		if (j == 0)
		{
			ET = tempC;
			// Serial.print("ET: ");
			// Serial.println(ET, 1);
		}
		else if (j == 1)
		{
			BT = tempC;
			// Serial.print("BT: ");
			// Serial.println(BT, 1);
		}
	}
	AT = sensor.getTempC();
	// Serial.print("AT: ");
	// Serial.println(AT, 1);
}
