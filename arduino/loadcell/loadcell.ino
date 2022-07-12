
#include "HX711.h"


// HX711 circuit wiring
const int LOADCELL_DOUT_PIN = 2;
const int LOADCELL_SCK_PIN = 3;

const float calibration_weight = 4900.0;
float zero_reading, load_reading;

const float calibration_factor = -99.52;


HX711 scale;

void setup() {
  Serial.begin(38400);

  // Initialize library with data output pin, clock input pin and gain factor.
  // Channel selection is made by passing the appropriate gain:
  // - With a gain factor of 64 or 128, channel A is selected
  // - With a gain factor of 32, channel B is selected
  // By omitting the gain factor parameter, the library
  // default "128" (Channel A) is used here.
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);

  delay(3);

  // determine the offset - setup: no load
  scale.tare(10);  // sets the offset value within scale to current value of reading
  zero_reading = scale.  //average over 10 vales


  //delay(2);
  //Serial.println("mount the known weight to the load cell and press ENTER to proceed with calibration");
  //int incomingByte = 0;
  // Calibration of the load cell
  //while(Serial.available() == 0) {
    //}
    
  //load_reading = scale.read_average(10);

  //scale.set_scale((load_reading - zero_reading)/calibration_weight);  // this value is obtained by calibrating the scale with known weights; see the README for details
  //incomingByte = Serial.read();   // clear the receive buffer by assigning value to var

  //Serial.print("the calibration factor is:  ");
  //Serial.println(load_reading - zero_reading/calibration_weight);
  
  scale.set_scale(calibration_factor);
  
  //Serial.println("load cell is setup. Start reading now...");
  delay(1); // delay that python code sees the correct starting byte
}

void loop() {
  //Serial.print("get value:\t");
  Serial.println(scale.get_units());

  //scale.power_down();			        // put the ADC in sleep mode
  delay(1/80);
  //scale.power_up();
}
