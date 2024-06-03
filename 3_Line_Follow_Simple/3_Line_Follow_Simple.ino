/*
This file contains the code for pure line following with and without PID

Debugging: 
1. Use the IR_Readings.ino file to manually check the readings for the black and white surfaces
   Remember to get the readings at various positions and orientations
2. Adjust the thresholds array below such that the value is higher than the readings for black surface
   but lower than the readings for white surface and ensure enough margin is given.
   This is crucial in ensuring the line is properly detected and no false detection occurs
3. Adjusting KP:
      If the robot turns too slowly when following the line, increase the value of KP
      If the robot turns too sharply, then reduce the value of KP
      NOTE: Better too high KP is better than too low as at least the robot will follow the line
*/

#include <AFMotor.h>

AF_DCMotor lmotor(1);
AF_DCMotor rmotor(4);

#define SENSOR_NUM 5
#define WEIGHT_UNIT 10
#define BASE_SPEED 80

// Adjust this number such that robot is moving straight at this speed
#define MAX_SPEED_LEFT 110
#define MAX_SPEED_RIGHT 130

#define KP 8
#define SP 3.5*WEIGHT_UNIT

// Uncomment the below for PID and change the values of KP, KI, and KI
// until robot follows the line smoothly
/*
#define KI 0.5
#define KD 0.2
int lastError = 0;
*/

const int MID_POSITION = WEIGHT_UNIT * (SENSOR_NUM + 1) / 2;

int sensor[SENSOR_NUM] = {A0, A1, A2, A3, A4};

// Adjust such that the value is between the readings for black and white surfaces
int thresholds[SENSOR_NUM] = {150, 150, 150, 150, 150};

int getPosition() {
  // Explanation of the logic behind this can be found in the slides
  int detectedCount = 0;
  int position = 0;
  
  // Iterate through each sensor
  for(int i = 0; i < SENSOR_NUM; i++) {
    int deviation = thresholds[i] - analogRead(sensor[i]);
    if (deviation > 0) {
      // Line is detected
      detectedCount++;

      // Add the weighted value to position
      position += WEIGHT_UNIT * (i + 1);
      position += (WEIGHT_UNIT * deviation)/thresholds[i];
    }
  }

  if (detectedCount == 0) {
    return SP;
  }

  // Calculate the average to get the postion
  return position / detectedCount;
}

void lineFollowing(int position) {
  // Calculate the line error/offset from middle position
  int error = position - MID_POSITION;

  // Multiply error by a constant to adjust the speed correction amount
  float correction = error * KP;

  // Alternatively, replace the above with the above with the commented
  // lines below for PID
  /*
  int P = error;
  int I += error;
  int D = error - lastError;
  lastError = error;
  float correction = (P * KP) + (I * KI) + (D * KD);
  */

  // Calculate the corrected left and right motor speeds
  float lspeed = BASE_SPEED + correction;
  float rspeed = BASE_SPEED - correction;

  // Limit speed to between 0% and 100%
  // Check out ternary operator for more information of the below syntax
  lspeed = lspeed > 100 ? 100 : lspeed;
  lspeed = lspeed < 0 ? 0 : lspeed;
  rspeed = rspeed > 100 ? 100 : rspeed;
  rspeed = rspeed < 0 ? 0 : rspeed;

  // Set the motor speed to the corrected speed
  lmotor.setSpeed(lspeed * MAX_SPEED_LEFT / 100);
  rmotor.setSpeed(rspeed * MAX_SPEED_RIGHT / 100);   

}

void setup() {
  Serial.begin(115200);
  for(int i = 0; i < SENSOR_NUM; i++) {
    pinMode(sensor[i], INPUT);
  }
  
  lmotor.run(FORWARD);
  rmotor.run(FORWARD);
  lmotor.setSpeed(0);
  rmotor.setSpeed(0);

  delay(3000);
}

void loop() {
  int position = getPosition();  
  lineFollowing(position);
}
