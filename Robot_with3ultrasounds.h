#ifndef ROBOT_WITH3ULTRASOUNDS_H
#define ROBOT_WITH3ULTRASOUNDS_H

#include <NewPing.h>
#include <SimpleKalmanFilter.h>

#define SONAR_NUM 3          //The number of sensors. 
#define MAX_DISTANCE 200     //Mad distance to detect obstacles.
#define PING_INTERVAL 33     //Looping the pings after 33 microseconds.
 
int LOOPING              = 10; //Loop for every 10 milliseconds.
int DECREESE_SPEED_LOOP  = 70;//Give some time to sensors for few more readings.
int MOVE_TO_NEW_POSITION = 100;//Wait for the new position.
 
unsigned long _timerStart         = 0;
unsigned long _timerStartReady    = 0;
unsigned long _timerStartPosition = 0;
 
uint8_t MIN_RANGE_OBSTACLE = 2; //Between 0 and 5 cm is the blind zone of the sensor.
uint8_t MAX_RANGE_OBSTACLE = 70; //The maximum range to check if obstacle exists.
 
uint8_t oldSensorReading[3];    //Store last valid value of the sensors.
 
uint8_t leftSensor;             //Store the sensor's value.
uint8_t centerSensor;
uint8_t rightSensor;
 
bool isObstacleLeft;           //If obstacle detected or not.
bool isObstacleCenter;
bool isObstacleRight;
 
uint8_t maximumSpeed = 140;//120; //PWM value for maximum speed.
uint8_t minSpeed = 100; //PWM value for minimum speed.
 
unsigned long pingTimer[SONAR_NUM]; // Holds the times when the next ping should happen for each sensor.
unsigned int cm[SONAR_NUM];         // Where the ping distances are stored.
uint8_t currentSensor = 0;          // Keeps track of which sensor is active.
 
NewPing sonar[SONAR_NUM] = {
  NewPing(26, 27, MAX_DISTANCE), //left sensor// Trigger pin, echo pin, and max distance to ping.
  NewPing(28, 29, MAX_DISTANCE),//center sensor
  NewPing(30, 31, MAX_DISTANCE)//right sensor
};
 
SimpleKalmanFilter KF_Left(2, 2, 0.01);
SimpleKalmanFilter KF_Center(2, 2, 0.01);
SimpleKalmanFilter KF_Right(2, 2, 0.01);
 
enum NavigationStates {
  CHECK_ALL,
  MAX_SPEED,
  DISABLE_MOTORS,
  CHECK_OBSTACLE_POSITION,
  FWRD_LEFT,
  CENTER,
  FWRD_RIGHT,
  BWRD_LEFT,
  BACK,
  BWRD_RIGHT,
};
NavigationStates _navState = CHECK_ALL;
 
//L298N motor driver pins
byte enA = 4;
byte in1 = 22;
byte in2 = 23;
byte enB = 5;
byte in3 = 24;
byte in4 = 25;



void LOOP();
void startTimer();
void startTimerReady();
void startTimerPosition();
bool isTimeForLoop(int _mSec);
bool isTimerReady(int _mSec);
bool isTimerPosition(int _mSec);
void sensorCycle();
void echoCheck();
void oneSensorCycle();
int returnLastValidRead(uint8_t sensorArray, uint8_t cm);
void applyKF();
bool obstacleDetection(int sensorRange);
void obstacleAvoidance();
void stopMotors();
void moveForward(uint8_t pwmValue);
void moveBackward(uint8_t pwmValue1, uint8_t pwmValue2);
void moveLeft(uint8_t pwmValue);
void moveRight(uint8_t pwmValue);
int randomMove();
void Init(void);



void LOOP(){
  if (isTimeForLoop(LOOPING)) {
    sensorCycle();
    applyKF();
    obstacleAvoidance();
    startTimer();
  }
  }


/*******************************Functions for time*********************************************************/
void startTimer() {
  _timerStart = millis();
}
 
void startTimerReady() {
  _timerStartReady = millis();
}
void startTimerPosition() {
  _timerStartPosition = millis();
}
bool isTimeForLoop(int _mSec) {
  return (millis() - _timerStart) > _mSec;
}
bool isTimerReady(int _mSec) {
  return (millis() - _timerStartReady) > _mSec;
}
bool isTimerPosition(int _mSec) {
  return (millis() - _timerStartPosition) > _mSec;
}
/*********************************End Functions for time*******************************************************/
//looping the sensors
void sensorCycle() {
  for (uint8_t i = 0; i < SONAR_NUM; i++) {
    if (millis() >= pingTimer[i]) {
      pingTimer[i] += PING_INTERVAL * SONAR_NUM;
      if (i == 0 && currentSensor == SONAR_NUM - 1) oneSensorCycle();
      sonar[currentSensor].timer_stop();
      currentSensor = i;
      cm[currentSensor] = 0;
      sonar[currentSensor].ping_timer(echoCheck);
    }
  }
}

// If ping received, set the sensor distance to array.
void echoCheck() {
  if (sonar[currentSensor].check_timer())
    cm[currentSensor] = sonar[currentSensor].ping_result / US_ROUNDTRIP_CM;
}

//Return the last valid value from the sensor.
void oneSensorCycle() {
  leftSensor   = returnLastValidRead(0, cm[0]);
  centerSensor = returnLastValidRead(1, cm[1]);
  rightSensor  = returnLastValidRead(2, cm[2]);
}

//If sensor value is 0, then return the last stored value different than 0.
int returnLastValidRead(uint8_t sensorArray, uint8_t cm) {
  if (cm != 0) {
    return oldSensorReading[sensorArray] = cm;
  } else {
    return oldSensorReading[sensorArray];
  }
}

//Apply Kalman Filter to sensor reading.
void applyKF() {
  isObstacleLeft = obstacleDetection(KF_Left.updateEstimate(leftSensor));
  isObstacleCenter = obstacleDetection(KF_Center.updateEstimate(centerSensor));
  isObstacleRight = obstacleDetection(KF_Right.updateEstimate(rightSensor));
}

//Define the minimum and maximum range of the sensors, and return true if an obstacle is in range.
bool obstacleDetection(int sensorRange) {
  if ((MIN_RANGE_OBSTACLE <= sensorRange) && (sensorRange <= MAX_RANGE_OBSTACLE)) return true; else return false;
}
  
//Obstacle avoidance algorithm.
void obstacleAvoidance()
{
  switch (_navState) {
    case CHECK_ALL: { //if no obstacle, go forward at maximum speed
        if (isObstacleLeft == 0 && isObstacleCenter == 0 && isObstacleRight == 0) {
          _navState = MAX_SPEED;
        } else {
          _navState = DISABLE_MOTORS;
          startTimerReady();
        }
      } break;
 
    case MAX_SPEED: {
        moveForward(maximumSpeed);
        _navState = CHECK_ALL;
      } break;
 
    case DISABLE_MOTORS: {
        stopMotors();
        //Wait for few more readings at speed = 0 and then go to check the obstacle position
        if (isTimerReady(DECREESE_SPEED_LOOP)) _navState = CHECK_OBSTACLE_POSITION;
      } break;
 
    case CHECK_OBSTACLE_POSITION: {
/*
L  C  R
0  0  0
0  0  1
0  1  0
0  1  1
1  0  0
1  0  1
1  1  0
1  1  1
*/
        //If the path is free, go again to MAX_SPEED else check the obstacle position
        if (isObstacleLeft == 0 && isObstacleCenter == 0 && isObstacleRight == 0) {
          _navState = MAX_SPEED;
        }
        else if (isObstacleLeft == 0 && isObstacleCenter == 0 && isObstacleRight  == 1) {
          startTimerPosition();
          _navState = FWRD_RIGHT;
        }
        else if (isObstacleLeft == 0 && isObstacleCenter == 1 && isObstacleRight == 0) {
          startTimerPosition();
          _navState = CENTER;
        }
        else if (isObstacleLeft == 0 && isObstacleCenter == 1 && isObstacleRight  == 1){
           startTimerPosition(); 
           _navState = FWRD_RIGHT; 
        }
        else if (isObstacleLeft == 1 && isObstacleCenter == 0 && isObstacleRight == 0) {
          startTimerPosition();
          _navState = FWRD_LEFT;
        }
        else if (isObstacleLeft == 1 && isObstacleCenter == 0 && isObstacleRight  == 1){
           startTimerPosition();
           _navState = BACK;
        }
        else if (isObstacleLeft == 1 && isObstacleCenter == 1 && isObstacleRight  == 0){
            startTimerPosition();  
            _navState = FWRD_LEFT;
        }
        else if (isObstacleLeft == 1 && isObstacleCenter == 1 && isObstacleRight == 1) {
          startTimerPosition();
          _navState = BACK;
        }
      } break;
  
    case FWRD_LEFT: { //Move left and check obstacle. If obstacle exists, go again to left, else exit
        moveLeft(minSpeed);
        if (isTimerPosition(MOVE_TO_NEW_POSITION)) {
          if (isObstacleLeft == 1) _navState = FWRD_LEFT;
          else _navState = CHECK_ALL;
        }
      } break;
 
    case CENTER: { //If obstacle exists, go left or right
        if (randomMove() == 1)  _navState = FWRD_LEFT; else  _navState = FWRD_RIGHT;
      } break;
 
    case FWRD_RIGHT: {
        moveRight(minSpeed);
        if (isTimerPosition(MOVE_TO_NEW_POSITION)) {
          if (isObstacleRight == 1) _navState = FWRD_RIGHT;
          else _navState = CHECK_ALL;
        }
      } break;
 
    case BACK: {
          if (randomMove() == 1)  _navState = BWRD_LEFT; else  _navState = BWRD_RIGHT;     
      } break;

    case BWRD_LEFT: {
       moveBackward(0,minSpeed);
        if (isTimerPosition(MOVE_TO_NEW_POSITION)) {
          if( (isObstacleLeft == 1 && isObstacleCenter == 1 && isObstacleRight == 1)|| (isObstacleLeft == 1 && isObstacleCenter == 0 && isObstacleRight == 1)) _navState = BWRD_LEFT;
          else _navState = CHECK_ALL;
          }
      } break;
    case BWRD_RIGHT: {
       moveBackward(minSpeed,0);
        if (isTimerPosition(MOVE_TO_NEW_POSITION)) {
          if( (isObstacleLeft == 1 && isObstacleCenter == 1 && isObstacleRight == 1)|| (isObstacleLeft == 1 && isObstacleCenter == 0 && isObstacleRight == 1)) _navState = BWRD_RIGHT;
          else _navState = CHECK_ALL;
          }     
      } break;
  }
}


//L298N Motor Driver.
void stopMotors() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}
 
void moveForward(uint8_t pwmValue) {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(enA, pwmValue);
  analogWrite(enB, pwmValue);
}
 
void moveBackward(uint8_t pwmValue1, uint8_t pwmValue2) {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  analogWrite(enA, pwmValue1);
  analogWrite(enB, pwmValue2);
}
 
void moveLeft(uint8_t pwmValue) {
  digitalWrite(in1, LOW); //Left wheel backward.
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);//Right wheel forward.
  digitalWrite(in4, LOW);
  analogWrite(enA, pwmValue);
  analogWrite(enB, pwmValue);
}
 
void moveRight(uint8_t pwmValue) {
  digitalWrite(in1, HIGH); //Left wheel forward.
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);//Right wheel backward.
  digitalWrite(in4, HIGH);
  analogWrite(enA, pwmValue);
  analogWrite(enB, pwmValue);
}
 
//Return 1 or 0 from a random number. 
int randomMove() {
  uint8_t rnd_number = random(1, 100);
  return rnd_number % 2;
 
}

void Init(){
  Serial.begin(9600);
 
  pingTimer[0] = millis() + 75;
  for (uint8_t i = 1; i < SONAR_NUM; i++)
    pingTimer[i] = pingTimer[i - 1] + PING_INTERVAL;
 
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
 
  stopMotors();
  }

#endif
