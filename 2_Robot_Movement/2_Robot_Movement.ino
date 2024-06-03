/* 
This file contains the code for basic robot movements and can
be used to check if the motors are working properly.
If wired correctly and fully functional, the robot should:
1. Move Forwards
2. Move Backwards
3. Turn Left (Only Right motor moving forwards)
4. Turn Right (Left forwards)
5. Rotate Left (Right forwards, Left reverse)
6. Rotate Right (Left forwards, Right reverse)
7. Stop

DEBUGGING:
If the above does not happen:
1. Ensure that the wires are connect to both the Motor Shield and the motors
2. If any of the motors are going in the reverse direction of the above
   then swap the connection of the wires on the Motor Shield
3. If the robot suddenly stops and repeats the sequence above or repeatedly jerks in place
   then either reduce the max speed below, or change the batteries
*/

#include <AFMotor.h>

AF_DCMotor lmotor(1);
AF_DCMotor rmotor(4);

// Adjust the following values such that the robot
// moves in a straight line when moving forwards
#define MAX_SPEED_LEFT 110
#define MAX_SPEED_RIGHT 140

void stop(int speed = 100) {
  lmotor.run(RELEASE);
  rmotor.run(RELEASE);
}

void forwards(int speed = 100) {
  lmotor.setSpeed(speed / 100 * MAX_SPEED_LEFT);
  rmotor.setSpeed(speed / 100 * MAX_SPEED_RIGHT);
  lmotor.run(FORWARD);
  rmotor.run(FORWARD);
}

void backwards(int speed = 100) {
  lmotor.setSpeed(speed / 100 * MAX_SPEED_LEFT);
  rmotor.setSpeed(speed / 100 * MAX_SPEED_RIGHT);
  lmotor.run(BACKWARD);
  rmotor.run(BACKWARD);
}

void turnLeft(int lspeed = 0, int rspeed = 100) {
  lmotor.setSpeed(lspeed / 100 * MAX_SPEED_LEFT);
  rmotor.setSpeed(rspeed / 100 * MAX_SPEED_RIGHT);
  lmotor.run(FORWARD);
  rmotor.run(FORWARD);
}

void turnRight(int lspeed = 100, int rspeed = 0) {
  lmotor.setSpeed(lspeed / 100 * MAX_SPEED_LEFT);
  rmotor.setSpeed(rspeed / 100 * MAX_SPEED_RIGHT);
  lmotor.run(FORWARD);
  rmotor.run(FORWARD);
}

void rotateLeft(int lspeed = 100, int rspeed = 100) {
  lmotor.setSpeed(lspeed / 100 * MAX_SPEED_LEFT);
  rmotor.setSpeed(rspeed / 100 * MAX_SPEED_RIGHT);
  lmotor.run(BACKWARD);
  rmotor.run(FORWARD);
}

void rotateRight(int lspeed = 100, int rspeed = 100) {
  lmotor.setSpeed(lspeed / 100 * MAX_SPEED_LEFT);
  rmotor.setSpeed(rspeed / 100 * MAX_SPEED_RIGHT);
  lmotor.run(FORWARD);
  rmotor.run(BACKWARD);
}

void setup() {
  forwards();
  delay(1000);
  stop();
  delay(1000);
  backwards();
  delay(1000);
  stop();
  delay(1000);
  turnLeft();
  delay(1000);
  stop();
  delay(1000);
  turnRight();
  delay(1000);
  stop();
  delay(1000);
  rotateLeft();
  delay(1000);
  stop();
  delay(1000);
  rotateRight();
  delay(1000);
  stop();
}

void loop() {
  
}
