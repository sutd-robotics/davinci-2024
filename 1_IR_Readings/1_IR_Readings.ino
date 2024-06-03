/*
This file contains the code to print out the IR sensor value to the serial monitor

Debugging:
1. Ensure wires are from IR1 to IR5 are connect to pins A0 to A4
2. Ensure that the bottom of the IR sensor is close to but not scraping the ground
3. Ideally the value for a line (black surface) should be below 100
   if it isn't then washers can be added between the brass stand-offs to lower the sensor
*/

// Create a constant for the number of IR sensors
#define SENSOR_NUM 5

// Create array for the IR sensor Pins and sensor values
int sensor[SENSOR_NUM] = {A0, A1, A2, A3, A4};
int sensorReadings[SENSOR_NUM];

void setup() {
  // Allows for the serial monitor to be used
  Serial.begin(115200);
  
  // Set the pins to be treated as an input
  for(int i = 0; i < SENSOR_NUM; i++) {
    pinMode(sensor[i], INPUT);
  }
}

void loop() {
  int value;
  
  // Read the values of all of the IR sensors
  for(int i = 0; i < SENSOR_NUM; i++) {
    value = analogRead(sensor[i]);
    
    // Store the value in an array
    sensorReadings[i] = value;

    // Print out the value to the serial monitor
    Serial.print(value);
    Serial.print(" | ");
  }
  Serial.println("");
  delay(200);
}
