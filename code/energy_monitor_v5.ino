// Include libraries for the onboard display and current sensor
#include <Wire.h>               
#include "HT_SSD1306Wire.h"

// Display setup (manufacturer provided): addr , freq , i2c group , resolution , rst
SSD1306Wire  display(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED);

// Define variables
//int counter = 1;
int button_state = 1;
int prev_button_state = 1;
int relay_state = HIGH;
//double results = 0;
//float sensor_raw = 0;
//float sensor_raw_prev = 0;
//float sensor_ma = 0;
//float sensor_ma_prev = 0;

// Sensor reading and calculation variables
double sensor_mv_raw = 0;
//double sensor_mv_prev = 0;
double sensor_mv = 0;
double amps = 0;
double watts = 0;
double amps_for_printing = 0;
double watts_for_printing = 0;
// Update these values as needed/measured
int volts = 120;
double mv_baseline = 2055;
int mv_per_amp = 80;

// Timing variables - sensor_interval determines how often the sensor is read
unsigned long prev_sensor_millis = 0;
unsigned long prev_debounce_millis = 0;
long sensor_interval = 2500;
long debounce_interval = 500;

// int mVperAmp = 100;
// int RawValue = 0;
// int ACSoffset = 2500;


// Pin mapping
const int sensor_pin = 6;        // Signal from current sensor: pin 6, analog input
const int relay_pin = 4;         // Relay control signal: pin 4, digital output, active HIGH
const int button_pin = 47;       // External button: pin 47 (pin 0 for built-in button), digital input, pull-up resistor, active LOW
const int onboard_led_pin = 35;  // White LED built-in to the dev board: pin 35, active LOW

void setup() {

  // Enable serial monitor
  Serial.begin(115200);

  // Set up pins
  pinMode(sensor_pin, INPUT);
  pinMode(relay_pin, OUTPUT);
  pinMode(button_pin, INPUT_PULLUP);
  pinMode(onboard_led_pin, OUTPUT);

  // Set initial relay state
  digitalWrite(relay_pin, HIGH);

  // Turn on the "Vext" transistor-controlled pins (controls the onboard display)
  pinMode(Vext,OUTPUT);
  digitalWrite(Vext, LOW);
  delay(50);

  // Initialize display and set text parameters
  display.init();
  display.setFont(ArialMT_Plain_16);
  display.setTextAlignment(TEXT_ALIGN_LEFT);
}

void loop() {
  
  unsigned long current_millis = millis();

  // Read button state with debounce
  int new_button_state = digitalRead(button_pin);
  if (new_button_state == LOW && ((current_millis - prev_debounce_millis) >= debounce_interval)) {
    relay_state = !relay_state;
    digitalWrite(relay_pin, relay_state);
    prev_debounce_millis = millis();
  }



  // Take a new sensor reading if relay_state is HIGH and sensor_interval has elapsed
  if (current_millis - prev_sensor_millis >= sensor_interval) {
    if (relay_state == HIGH) {
      Serial.println("interval elapsed-taking sensor reading...");
      sensor_mv_raw = analogReadMilliVolts(sensor_pin);
      Serial.print("Raw mV: ");
      Serial.println(sensor_mv_raw);
      // Calibrating with roughly-approximated best-fit values: baseline (no current) = 2055mV; mV change in reading per 1 Amp change in current = 80mV/A
      // Calibration points (for reference): baseline=~2040-2080mV (2055); 1.1A=~2142mV; 2.2A=~2247mV; 8.5A=2747mV
      if ((sensor_mv_raw - mv_baseline) <= 30) {
        sensor_mv = 0;
        // The hardware shouldn't be used for >10A, so the maximum mV value (over baseline) should be approximately 10A * 83mV/A = 830mV
      } else if ((sensor_mv_raw - mv_baseline) >= 830) {
        sensor_mv = -1;
      } else {
        sensor_mv = sensor_mv_raw - mv_baseline;
      }
      Serial.print("sensor_mv ");
      Serial.println(sensor_mv);
      // Use the sensor_mv to calculate the current; use the current and measured voltage to calculate power
      if (sensor_mv == -1) {
        amps_for_printing = 999;
        watts_for_printing = 999;
      } else if (sensor_mv == 0) {
        amps = 0;
        watts = 0;
        amps_for_printing = 0;
        watts_for_printing = 0;
      } else {
        amps = sensor_mv / mv_per_amp;
        watts = amps * volts;
        amps_for_printing = ((round(amps * 10)) / 10);
        watts_for_printing = int(round(watts));
      }
      
      Serial.print("amps: ");
      Serial.println(amps);
      Serial.print("amps_for_printing: ");
      Serial.println(amps_for_printing);
      Serial.print("watts: ");
      Serial.println(watts);
      Serial.print("watts_for_printing: ");
      Serial.println(watts_for_printing);
      
      // Setup built-in OLED
      display.clear();
      display.drawString(0, 0, ("Power: On"));
      //display.drawString(0, 20, ("Raw mv: " + String(sensor_mv_raw)));
      //display.drawString(0, 20, ("New mV: " + String(new_mv)));
      display.drawString(0, 20, ("Current: " + String(amps_for_printing) + " A"));
      display.drawString(0, 40, ("Power: " + String(watts_for_printing) + " W"));
      display.display();
    } else if (relay_state == LOW) {
      display.clear();
      display.drawString(0, 0, ("Power: Off"));
      display.drawString(0, 20, ("Sensor Inactive"));
      display.display();
    }
  // Reset sensor timer
  prev_sensor_millis = current_millis;    
  }
}
