
#include "HX711.h"


// HX711 circuit wiring
const int LOADCELL_DOUT_PIN = 2;
const int LOADCELL_SCK_PIN = 3;

const float calibration_weight = 500.0;
float zero_reading, load_reading;


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

  //Serial.print("offset: ");
  //Serial.print(scale.get_offset(), DEC);
  //Serial.print("\t scale: ");
  //Serial.println(scale.get_scale(), DEC);

  Serial.println("tare the scale. in 3 sec. Make sure it is unloaded");

  delay(3);


  // determine the offset - setup: no load
  scale.tare();  // sets the offset value within scale to current value of reading
  zero_reading = scale.read_average(10); //average over 10 vales
  //scale.set_scale(); 
  //scale.get_units(10);

  //Serial.print("offset: ");
  //Serial.print(scale.get_offset(), DEC);
  //Serial.print("\t scale: ");
  //Serial.println(scale.get_scale(), DEC);


  //delay(2);
  Serial.println("mount the known weight to the load cell and press ENTER to proceed with calibration");
  int incomingByte = 0;
  // Calibration of the load cell
  while(Serial.available() == 0) {
    }
    
  load_reading = scale.read_average(10);
  Serial.print("zero and load reading: ");
  Serial.print(zero_reading, DEC);
  Serial.print("  ");
  Serial.println(load_reading, DEC);
  scale.set_scale((load_reading - zero_reading)/calibration_weight);  // this value is obtained by calibrating the scale with known weights; see the README for details
  incomingByte = Serial.read();   // clear the receive buffer by assigning value to var

  Serial.println("set scale....");
  Serial.print("offset: ");
  Serial.print(scale.get_offset(), DEC);
  Serial.print("\t scale: ");
  Serial.println(scale.get_scale(), DEC);

  Serial.print("get_value: ");
  Serial.print(scale.get_value());
  Serial.print("\t get_units: ");
  Serial.println(scale.get_units());
    

  
  Serial.println("load cell is setup. Start reading now...");
  delay(1); // delay that python code sees the correct starting byte
}

void loop() {
  //Serial.print("get value:\t");
  Serial.println(scale.get_units());

  //scale.power_down();			        // put the ADC in sleep mode
  //delay(5);
  //scale.power_up();
}
