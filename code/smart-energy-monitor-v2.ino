// Include libraries for connecting to I2C using Arduino Wire (for the onboard OLED display)
#include <Wire.h>               
#include "HT_SSD1306Wire.h"

// Display setup (manufacturer provided): addr , freq , i2c group , resolution , rst
SSD1306Wire  display(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED);

// Define variables
int counter = 1;
int button_state = 0;
double results = 0;
int mVperAmp = 67;
int RawValue = 0;
int ACSoffset = 2500;
double Voltage = 0;
double Amps = 0;
double Watts = 0;

// Pin mapping
const int sensor_pin = 6;        // Analog Input, baseline (zero power consumption) = ~0.66V
const int relay_pin = 4;         // Digital output, pull-up resistor, active LOW
const int button_pin = 47;       // Digital Input, active LOW
const int onboard_led_pin = 35;  // White LED built-in to the dev board

void setup() {
  Serial.begin(115200);
  Serial.println("Starting up...");

  // Turn on the "Vext" transistor-controlled pins (controls the onboard display)
  pinMode(Vext,OUTPUT);
  digitalWrite(Vext, LOW);
  delay(100);

  // Set pin modes
  pinMode(sensor_pin, INPUT)
  pinMode(relay_pin, OUTPUT);
  pinMode(button_pin, INPUT_PULLUP);
  pinMode(onboard_led_pin, OUTPUT);

  // Set the initial relay state (this will be the "default" state after a power failure, so it's worth considering which default we want)
  digitalWrite(relay_pin, HIGH);

  // Initialize display and set parameters
  display.init();
  display.setFont(ArialMT_Plain_16);
  display.setTextAlignment(TEXT_ALIGN_LEFT);
}

void sensor_read()
{
  RawValue = analogRead(sensor_pin);
  Voltage = (RawValue / 1024.0) * 3300; // Gets you mV
  Amps = ((Voltage - ACSoffset) / mVperAmp);
  Watts = (Amps*120/1.2);
  delay(2000);
}

void loop() {
  display.clear();
  sensor_read();

  // Print results to serial
  // Serial.print("mV: " + String(Voltage) + "mV");
  // Serial.print("Amps: " + String(Amps) + "A");
  // Serial.print("Watts: " + String(Watts) + "W");
  // Serial.println("Raw: " + String(analogRead(sensor_pin)));

  // // Print results to OLED
  display.drawString(0, 0, ("Raw: " + String(analogRead(sensor_pin))));
  display.drawString(0, 20, ("mV: " + String(Voltage)));

  // display.drawString(0, 0, ("Raw: " + String(Voltage) + "mV"));
  // display.drawString(0, 20, ("Amps: " + String(Amps) + "A"));
  // display.drawString(0, 40, ("Watts: " + String(Watts) + "W"));

  display.display();

  button_state = digitalRead(button_pin);
  if (button_state == LOW) {
      // turn LED on:
      digitalWrite(onboard_led_pin, HIGH);
    } else {
      // turn LED off:
      digitalWrite(onboard_led_pin, LOW);
    }

}
