/*
This file contains the code for maze solving by left hand rule with path optimisation
for the second run.

Note:
  Remember to test and correctly set the sensor thresholds and the movement durations.
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

// Adjust the following durations such that the line is close to the centre 
// of the IR sensors by the end of the movement
#define TURN_AROUND_DURATION 500
#define FORWARD_STEP_DURATION 300
#define ROTATE_LEFT_DURATION 300
#define ROTATE_RIGHT_DURATION 300
#define TURN_LEFT_DURATION 500
#define TURN_RIGHT_DURATION 500

#define PATH_MAX 20
bool mazeEnd = false;
char path[PATH_MAX];
int actionCount = 0;
int runNum = 1;

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
    if (analogRead(sensor[i]) < thresholds[i]) {
      // Line is detected
      detectedCount++;

      // Add the weighted value to position
      position += WEIGHT_UNIT * (i + 1);
    }
  }

  if (detectedCount == 0) {
    return position;
  }

  // Calculate the average to get the postion
  return position / detectedCount;
}

void lineFollowing(int position) {
  // Calculate the line error/offset from middle position
  int error = position - MID_POSITION;

  // Multiply error by a constant to adjust the speed correction amount
  float correction = error * KP;

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

int getLine() {
  // Returns the IR sensor values in the format of 11000, 101 for 00101, etc.
  // to check the type/state of the current junction
  int line = 0;
  
  // Iterate through each sensor
  for(int i = 0; i < SENSOR_NUM; i++) {
    if (analogRead(sensor[i]) < thresholds[i]) {

      // Format the line data into the form of XXXXX
      // E.g. 11100 or 11 for 00011
      int weight = 1;
      for(int j = 0; j < SENSOR_NUM - i - 1; j++) {
        weight *= 10;
      }
      line += 1 * weight;
    }
  }
  return line;
}

void junctionAction(int line) {
  // Actions to take when specific junctions are encountered
  // Explanation with diagrams available in the slides
  // This function has been modified from the previous file
  // to record the action taken as well

  /*
    Note: The difference in rotating and turning is that in rotating,
    one motor is set to forwards and the other in reverse, causing the robot
    to rotate in place. While when turning left, only the right motor is moving
    forwards while the left motor stops causing the robot to turn left with
    the left wheel as a pivot.
    Distinguishing between these 2 types of movement will account for
    whether the robot has taken a step forwards or not to ideally make the maze
    line in the centre of the sensors by the end of the movement.
  */
  switch (line)
  {
    case 0:
      turnAround();
      path[actionCount] = 'B';
      actionCount++;
      break;
    case 11111:
      stepForward();
      if (getLine() == 11111) {
        stop();
        mazeEnd = true;
        path[actionCount] = 'E';
      } else {
        rotateLeft();
        path[actionCount] = 'L';
        actionCount++;
      }
      break;
    case 111:
      // i.e. when line is 00111
      stepForward();
      if (getLine() == 0) {
        rotateRight();
        path[actionCount] = 'R';
      } else {
        stepForward();
        path[actionCount] = 'S';
      }
      actionCount++;
      break;
    case 11100:
      turnLeft();
      path[actionCount] = 'L';
      actionCount++;
      break;
  }
}

void stop() {
  // Reset the motors back to forwards and stop
  lmotor.run(FORWARD);
  rmotor.run(FORWARD);
  lmotor.setSpeed(0);
  rmotor.setSpeed(0);
}

void turnAround() {
  // Make the robot rotate to the left in place
  lmotor.run(BACKWARD);
  rmotor.run(FORWARD);
  lmotor.setSpeed(BASE_SPEED * MAX_SPEED_LEFT / 100);
  rmotor.setSpeed(BASE_SPEED * MAX_SPEED_RIGHT / 100);
  
  // Keep rotating for a duration of TURN_AROUND_DURATION
  // Alternatively, you can create a condition of when to stop 
  // such as when a line is detected  
  delay(TURN_AROUND_DURATION);
  stop();
}

void stepForward() {
  lmotor.setSpeed(BASE_SPEED * MAX_SPEED_LEFT / 100);
  rmotor.setSpeed(BASE_SPEED * MAX_SPEED_RIGHT / 100);
  delay(FORWARD_STEP_DURATION);
  stop();
}

void rotateLeft() {
  lmotor.run(BACKWARD);
  rmotor.run(FORWARD);
  lmotor.setSpeed(BASE_SPEED * MAX_SPEED_LEFT / 100);
  rmotor.setSpeed(BASE_SPEED * MAX_SPEED_RIGHT / 100);
  delay(ROTATE_LEFT_DURATION);
  stop();
}

void rotateRight() {
  lmotor.run(FORWARD);
  rmotor.run(BACKWARD);
  lmotor.setSpeed(BASE_SPEED * MAX_SPEED_LEFT / 100);
  rmotor.setSpeed(BASE_SPEED * MAX_SPEED_RIGHT / 100);
  delay(ROTATE_RIGHT_DURATION);
  stop();
}

void turnLeft() {
  lmotor.setSpeed(0);
  rmotor.setSpeed(BASE_SPEED * MAX_SPEED_LEFT / 100);
  delay(TURN_LEFT_DURATION);
  stop();
}

void turnRight() {
  lmotor.setSpeed(BASE_SPEED * MAX_SPEED_LEFT / 100);
  rmotor.setSpeed(0);
  delay(TURN_RIGHT_DURATION);
  stop();
}

void optimisePath() {
  // Note: it is necessary to use single quotes otherwise it will be treated as a string (char *)
  for(int i = 1; i < PATH_MAX - 1; i++) {
    if(path[i] == 'B') {
      // If there is a back action, determine the optimised action based on the previous and next action
      char optimisedAction = ' ';
      if (path[i-1] == 'L' && path[i+1] == 'L') {
        optimisedAction = 'S';
      } else if (path[i-1] == 'L' && path[i+1] == 'S') {
        optimisedAction = 'R';
      } else if (path[i-1] == 'L' && path[i+1] == 'R') {
        optimisedAction = 'B';
      } else if (path[i-1] == 'S' && path[i+1] == 'L') {
        optimisedAction = 'R';
      } else if (path[i-1] == 'S' && path[i+1] == 'S') {
        optimisedAction = 'B';
      } else if (path[i-1] == 'R' && path[i+1] == 'L') {
        optimisedAction = 'B';
      }
      
      // Replace the 3 actions with just the optimised action and
      // shift the remaining actions to not leave empty actions
      path[i-1] = optimisedAction;
      for(int j = i; j < PATH_MAX - 2; j++) {
        path[j] = path[j+2];
      }
      
      // Further optimise the path using recursion
      optimisePath();
    }
  }
}

void shortestPath() {
  char action = path[actionCount];
  switch(action) 
  {
    case 'L':
      turnLeft();
      break;
    case 'R':
      turnRight();
      break;
    case 'S':
      stepForward();
      break;
    case 'E':
      mazeEnd = true;
      stepForward();
      stop();
      break;  
  }
  actionCount++;
}

void setup() {
  Serial.begin(115200);
  for(int i = 0; i < SENSOR_NUM; i++) {
    pinMode(sensor[i], INPUT);
  }
  
  stop();
  
  // Reset the path
  for(int i = 0; i < PATH_MAX; i++) {
    path[i] = ' ';
  }
  delay(3000);
}

void loop() {
  if (runNum == 1) {
    if (!mazeEnd) {
      int line = getLine();
      junctionAction(line);
      int position = getPosition();  
      lineFollowing(position);
    } else {
      runNum++;
      optimisePath();
      mazeEnd = false;
      actionCount = 0;
      // Delay to allow for repositioning of robot back to "START"
      delay(7000);
    }
  } else if (runNum == 2) {
    if (!mazeEnd) {
      int line = getLine();
      // Follow path action if junction is detected
      if (line == 11111 || line == 11100 || line == 111) {
        shortestPath();
      }
      int position = getPosition();  
      lineFollowing(position);
    }
  }
}
