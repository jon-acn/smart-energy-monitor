// Include libraries for the onboard display and current sensor
#include <Wire.h>               
#include "HT_SSD1306Wire.h"
#include "ACS712.h"

// Display setup (manufacturer provided): addr , freq , i2c group , resolution , rst
SSD1306Wire  display(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED);

// Define variables
int counter = 1;
int button_state = 0;
double results = 0;
int mVperAmp = 200;
int RawValue = 0;
int ACSoffset = 2500;
double Voltage = 0;
double Amps = 0;
double Watts = 0;

// Pin mapping
const int sensor_pin = 6;        // Signal from current sensor: pin 6, analog input
const int relay_pin = 4;         // Relay control signal: pin 4, digital output, active LOW
const int button_pin = 0;       // External button: pin 47 (pin 0 for built-in button), digital input, pull-up resistor, active LOW
const int onboard_led_pin = 35;  // White LED built-in to the dev board: pin 35, active LOW

//ACS712  ACS(A0, 5.0, 1023, 100);
//  ESP 32 example (might requires resistors to step down the logic voltage)
ACS712  ACS(6, 3.3, 4095, 100);

void setup()
{
  Serial.begin(115200);
  Serial.println("Starting up...");
  // ACS712 code
  while (!Serial);
  Serial.println(__FILE__);
  Serial.print("ACS712_LIB_VERSION: ");
  Serial.println(ACS712_LIB_VERSION);
  
  ACS.autoMidPoint();
  Serial.print("MidPoint: ");
  Serial.println(ACS.getMidPoint());
  Serial.print("Noise mV: ");
  Serial.println(ACS.getNoisemV());

  // Turn on the "Vext" transistor-controlled pins (controls the onboard display)
  pinMode(Vext,OUTPUT);
  digitalWrite(Vext, LOW);
  delay(100);

  // Set pin modes
  pinMode(sensor_pin, INPUT);
  pinMode(relay_pin, OUTPUT);
  pinMode(button_pin, INPUT_PULLUP);
  pinMode(onboard_led_pin, OUTPUT);

  // Set the initial relay state (this will be the "default" state after a power failure, so it's worth considering which default we want)
  digitalWrite(relay_pin, LOW);

  // Initialize display and set text parameters
  display.init();
  display.setFont(ArialMT_Plain_16);
  display.setTextAlignment(TEXT_ALIGN_LEFT);
}






void loop()
{
  // Prepare the display
  display.clear();

  
  // ACS712 code
  float average = 0;
  uint32_t start = millis();
  for (int i = 0; i < 100; i++)
  {
    //  select sppropriate function
    average += ACS.mA_AC_sampling(60, 2);
    //average += ACS.mA_AC();
  }
  float mA = average / 100.0;
  uint32_t duration = millis() - start;
  Serial.print("Time: ");
  Serial.print(duration);
  Serial.print("  mA: ");
  Serial.println(mA);

  display.drawString(0, 0, ("Raw: " + String(analogRead(sensor_pin))));
  display.drawString(0, 20, ("Time: " + String(duration)));
  display.drawString(0, 40, ("Amps: " + String(mA) + "mA"));
  
  // Send everything to the display
  display.display();
  
  
  delay(500);


  //button_state = digitalRead(button_pin);
  // if (button_state == LOW) {
  //   digitalWrite(relay_pin, LOW);
  //   digitalWrite(onboard_led_pin, HIGH);
  //   delay(5000);
  //   digitalWrite(relay_pin, HIGH);
  //   digitalWrite(onboard_led_pin, LOW);
  // }
  
}
