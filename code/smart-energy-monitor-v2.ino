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

  // Initialize display and set text parameters
  display.init();
  display.setFont(ArialMT_Plain_16);
  display.setTextAlignment(TEXT_ALIGN_LEFT);
}

// This function will read the analog input from the sensor and use it to calculate the electrical properties
void sensor_read()
{
  RawValue = analogRead(sensor_pin);
  Voltage = (RawValue / 1024.0) * 3300; // Gets you mV
  Amps = ((Voltage - ACSoffset) / mVperAmp);
  Watts = (Amps*120/1.2);
  delay(2000);
}

void loop() {
  // Prepare the display
  display.clear();
  
  // Call the sensor reading function to collect a fresh reading from the current sensor
  sensor_read();

  // Print results to serial
  Serial.print("Raw: " + String(analogRead(sensor_pin)));
  Serial.print("Volts: " + String(Voltage) + "mV");
  Serial.println("Amps: " + String(Amps) + "A");
  //Serial.print("Watts: " + String(Watts) + "W");

  // Print results to OLED
  display.drawString(0, 0, ("Raw: " + String(analogRead(sensor_pin))));
  display.drawString(0, 20, ("Volts: " + String(Voltage) + "mV"));
  display.drawString(0, 40, ("Amps: " + String(Amps) + "A"));
  //display.drawString(0, 40, ("Watts: " + String(Watts) + "W"));
  
  // Simplified output for testing
//   display.drawString(0, 0, ("Raw: " + String(analogRead(sensor_pin))));
//   display.drawString(0, 20, ("mV: " + String(Voltage)));

  // Send everything to the display
  display.display();
}
