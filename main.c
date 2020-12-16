/**************************************************************
* Class: CSC-615-01 Spring 2020
* Authors: George Freedland, George Shen, Cameron Cirini, John Freirez
*
* Project: Term Project
*
* File: main.c
*
* Description: Main driver file to start robot car interface.
**************************************************************/

#include <stdio.h>
#include <unistd.h>
#include <time.h>
#include <stdbool.h>
#include <pthread.h>
#include <wiringPi.h>
#include <softPwm.h>

//TEMPORARY DEFITIONS: Change as pins are chosen
//Motor Voltage Control and Direction
// #define MOTOR_1_ENABLE 0    // 11 physical pin for motor 1
// #define MOTOR_VOLT_FL 0;
// #define MOTOR_VOLT_FR 0;
// #define MOTOR_VOLT_RL 0;
// #define MOTOR_VOLT_RR 0;
// #define MOTOR_DIR_FL 0;
// #define MOTOR_DIR_FR 0;
// #define MOTOR_DIR_RL 0;
// #define MOTOR_DIR_RR 0;

// //Line Sensor Voltage Control and Digital Output
#define LINE_SENSOR_L_VCC 0;
#define LINE_SENSOR_R_VCC 0;
#define LINE_SENSOR_C_VCC 0;
#define LINE_SENSOR_L_DO 0;
#define LINE_SENSOR_R_DO 0;
#define LINE_SENSOR_C_DO 0;
//TODO define remaining sensors
#define ECHO_VCC 0;
#define ECHO_TRIG 0;
#define ECHO_ECHO 0;
//update existing defitions with corresponding pin numbers

//front right
#define MOTOR_1_ENABLE 0    // 11 physical pin for motor 1
#define MOTOR_1_CONTROL_1 3 // 15 physical pin for motor 1
#define MOTOR_1_CONTROL_2 2 // 13 physical pin for motor 1

//front left
#define MOTOR_2_ENABLE 6    // 11 physical pin for motor 2
#define MOTOR_2_CONTROL_1 4 // 13 physical pin for motor 2
#define MOTOR_2_CONTROL_2 5 // 15 physical pin for motor 2

//back left
#define MOTOR_3_ENABLE 12    // 11 physical pin for motor 3
#define MOTOR_3_CONTROL_1 14 // 15 physical pin for motor 3
#define MOTOR_3_CONTROL_2 13 // 13 physical pin for motor 3

//back right
#define MOTOR_4_ENABLE 26    // 11 physical pin for motor 4
#define MOTOR_4_CONTROL_1 10 // 13 physical pin for motor 4
#define MOTOR_4_CONTROL_2 11 // 15 physical pin for motor 4

const int MILLISEC = 1000;
const int OBSTACLE_WAIT_TIME = 5;
const int OBSTACLE_DISTANCE = 50;
const int MAX_RUN_TIME = 90;
const int OPTIMAL_SPEED = 50;
const int MIN_SPEED = 10;
const int ADJUST = 5;
//const int HARD_ADJUST = 10;

typedef struct args
{
  int runFlag;
  int obstacleDetected;
  int motor1_c1;
  int motor1_c2;
  int motor2_c1;
  int motor2_c2;
  int motor3_c1;
  int motor3_c2;
  int motor4_c1;
  int motor4_c2;

  //add a past adjustments stack if necessary
} args;

void initilalizePins()
{
  // Inits motor 1
  softPwmCreate(MOTOR_1_ENABLE, 100, 100);
  softPwmWrite(MOTOR_1_ENABLE, 100);
  softPwmCreate(MOTOR_1_CONTROL_1, 0, 100);
  softPwmWrite(MOTOR_1_CONTROL_1, 0);
  softPwmCreate(MOTOR_1_CONTROL_2, 0, 100);
  softPwmWrite(MOTOR_1_CONTROL_2, 0);
  // Inits motor 2
  softPwmCreate(MOTOR_2_ENABLE, 100, 100);
  softPwmWrite(MOTOR_2_ENABLE, 100);
  softPwmCreate(MOTOR_2_CONTROL_1, 0, 100);
  softPwmWrite(MOTOR_2_CONTROL_1, 0);
  softPwmCreate(MOTOR_2_CONTROL_2, 0, 100);
  softPwmWrite(MOTOR_2_CONTROL_2, 0);
  // Inits motor 3
  softPwmCreate(MOTOR_3_ENABLE, 100, 100);
  softPwmWrite(MOTOR_3_ENABLE, 100);
  softPwmCreate(MOTOR_3_CONTROL_1, 0, 100);
  softPwmWrite(MOTOR_3_CONTROL_1, 0);
  softPwmCreate(MOTOR_3_CONTROL_2, 0, 100);
  softPwmWrite(MOTOR_3_CONTROL_2, 0);
  // Inits motor 4
  softPwmCreate(MOTOR_4_ENABLE, 100, 100);
  softPwmWrite(MOTOR_4_ENABLE, 100);
  softPwmCreate(MOTOR_4_CONTROL_1, 0, 100);
  softPwmWrite(MOTOR_4_CONTROL_1, 0);
  softPwmCreate(MOTOR_4_CONTROL_2, 0, 100);
  softPwmWrite(MOTOR_4_CONTROL_2, 0);

  //Init Echo
  pinMode(ECHO_TRIG, OUTPUT);
  pinMode(ECHO_ECHO, INPUT);
  digitalWrite(ECHO_TRIG, 0);
  digitalWrite(ECHO_ECHO, 0);
  //Init Line Sensor L
  pinMode(LINE_SENSOR_L_VCC, OUTPUT);
  pinMode(LINE_SENSOR_L_DO, INPUT);
  digitalWrite(LINE_SENSOR_L_VCC, 1);
  //Init Line Sensor C
  pinMode(LINE_SENSOR_C_VCC, OUTPUT);
  pinMode(LINE_SENSOR_C_DO, INPUT);
  digitalWrite(LINE_SENSOR_C_VCC, 1);
  //Init Line Sensor R
  pinMode(LINE_SENSOR_R_VCC, OUTPUT);
  pinMode(LINE_SENSOR_R_DO, INPUT);
  digitalWrite(LINE_SENSOR_R_VCC, 1);

  delay(100);
}

void clearPins()
{
  delay(100);
  softPwmCreate(MOTOR_1_ENABLE, 0, 100);
  softPwmWrite(MOTOR_1_ENABLE, 0);
  softPwmCreate(MOTOR_1_CONTROL_1, 0, 100);
  softPwmWrite(MOTOR_1_CONTROL_1, 0);
  softPwmCreate(MOTOR_1_CONTROL_2, 0, 100);
  softPwmWrite(MOTOR_1_CONTROL_2, 0);

  softPwmCreate(MOTOR_2_ENABLE, 0, 100);
  softPwmWrite(MOTOR_2_ENABLE, 0);
  softPwmCreate(MOTOR_2_CONTROL_1, 0, 100);
  softPwmWrite(MOTOR_2_CONTROL_1, 0);
  softPwmCreate(MOTOR_2_CONTROL_2, 0, 100);
  softPwmWrite(MOTOR_2_CONTROL_2, 0);

  softPwmCreate(MOTOR_3_ENABLE, 0, 100);
  softPwmWrite(MOTOR_3_ENABLE, 0);
  softPwmCreate(MOTOR_3_CONTROL_1, 0, 100);
  softPwmWrite(MOTOR_3_CONTROL_1, 0);
  softPwmCreate(MOTOR_3_CONTROL_2, 0, 100);
  softPwmWrite(MOTOR_3_CONTROL_2, 0);

  softPwmCreate(MOTOR_4_ENABLE, 0, 100);
  softPwmWrite(MOTOR_4_ENABLE, 0);
  softPwmCreate(MOTOR_4_CONTROL_1, 0, 100);
  softPwmWrite(MOTOR_4_CONTROL_1, 0);
  softPwmCreate(MOTOR_4_CONTROL_2, 0, 100);
  softPwmWrite(MOTOR_4_CONTROL_2, 0);
}

void stopCar()
{
  softPwmWrite(MOTOR_1_CONTROL_1, 0);
  softPwmWrite(MOTOR_2_CONTROL_1, 0);
  softPwmWrite(MOTOR_3_CONTROL_1, 0);
  softPwmWrite(MOTOR_4_CONTROL_1, 0);

  softPwmWrite(MOTOR_1_CONTROL_2, 0);
  softPwmWrite(MOTOR_2_CONTROL_2, 0);
  softPwmWrite(MOTOR_3_CONTROL_2, 0);
  softPwmWrite(MOTOR_4_CONTROL_2, 0);
}

void moveCarForward()
{
  // softPwmWrite(MOTOR_1_CONTROL_2, 0);
  // softPwmWrite(MOTOR_2_CONTROL_2, 0);
  // softPwmWrite(MOTOR_3_CONTROL_2, 0);
  // softPwmWrite(MOTOR_4_CONTROL_2, 0);
  stopCar();
  softPwmWrite(MOTOR_1_CONTROL_1, OPTIMAL_SPEED);
  softPwmWrite(MOTOR_2_CONTROL_1, OPTIMAL_SPEED);
  softPwmWrite(MOTOR_3_CONTROL_1, OPTIMAL_SPEED);
  softPwmWrite(MOTOR_4_CONTROL_1, OPTIMAL_SPEED);
}

void moveCarBackward()
{
  // softPwmWrite(MOTOR_1_CONTROL_1, 0);
  // softPwmWrite(MOTOR_2_CONTROL_1, 0);
  // softPwmWrite(MOTOR_3_CONTROL_1, 0);
  // softPwmWrite(MOTOR_4_CONTROL_1, 0);
  stopCar();
  softPwmWrite(MOTOR_1_CONTROL_2, 50);
  softPwmWrite(MOTOR_2_CONTROL_2, 50);
  softPwmWrite(MOTOR_3_CONTROL_2, 50);
  softPwmWrite(MOTOR_4_CONTROL_2, 50);
}

void spinCarRight()
{
  stopCar();
  // 2 and 3 forward, 1 and 4 backwards
  softPwmWrite(MOTOR_2_CONTROL_1, 50);
  softPwmWrite(MOTOR_3_CONTROL_1, 50);
  softPwmWrite(MOTOR_1_CONTROL_2, 50);
  softPwmWrite(MOTOR_4_CONTROL_2, 50);
}

void spinCarLeft()
{
  stopCar();
  // 2 and 3 forward, 1 and 4 backwards
  softPwmWrite(MOTOR_2_CONTROL_2, 50);
  softPwmWrite(MOTOR_3_CONTROL_2, 50);
  softPwmWrite(MOTOR_1_CONTROL_1, 50);
  softPwmWrite(MOTOR_4_CONTROL_1, 50);
}

int main(void)
{
  //printf("Welcome to the Raspberry Pi Self Driving \n");
  wiringPiSetup();
  initilalizePins();

  pthread_t ir;
  pthread_t line;
  args arguments;
  arguments.runFlag = 1;
  arguments.obstacleDetected = 0;

  arguments.motor1_c1 = OPTIMAL_SPEED;
  arguments.motor1_c2 = OPTIMAL_SPEED;
  arguments.motor2_c1 = OPTIMAL_SPEED;
  arguments.motor2_c2 = OPTIMAL_SPEED;
  arguments.motor3_c1 = OPTIMAL_SPEED;
  arguments.motor3_c2 = OPTIMAL_SPEED;
  arguments.motor4_c1 = OPTIMAL_SPEED;
  arguments.motor4_c2 = OPTIMAL_SPEED;

  bool condition = true;
  while (condition)
  {
    int input;
    printf("Enter 1 to start all motors forward, Enter 2 to stop all motors, and 3 to exit the program");
    printf("Enter 4 to go backwards, Enter 5 to turn right, Enter 6 to turn left, Enter 7 to test Program");

    scanf("%d", &input);
    switch (input)
    {
    case 1:
      printf("Move all motors forward\n");
      initilalizePins();
      moveCarForward();
      break;
    case 2:
      printf("Stop all motors\n");
      stopCar();
      break;
    case 3:
      printf("Exitting program\n");
      clearPins();
      condition = false;
      break;
    case 4:
      printf("Move all motors backwards\n");
      moveCarBackward();
      break;
    case 5:
      printf("Spin car Right In Place\n");
      spinCarRight();
      break;
    case 6:
      printf("Spin car Left In Place\n");
      spinCarLeft();
      break;
    case 7:
      printf("Spin car Left In Place\n");
      moveCarForward();
      //Run program until threads finish
      pthread_create(&ir, NULL, irSensor, &arguments);
      pthread_create(&line, NULL, lineSensor, &arguments);
      /*Join/Wait for threads to finish
      pthread_join(ir, NULL);
      pthread_join(line, NULL);*/
      break;
    default:
      printf("not a valid input");
      break;
    }
  }
  printf("Program Ended");
  return 0;
}

// initializePins();

// //initialize arguments
// pthread_t ir;
// pthread_t line;

// args arguments;
// arguments.runFlag = 1;
// arguments.obstacleDetected = 0;

// // Run program until threads finish
/*
    pthread_create(&ir, NULL, irSensor, &arguments);
    pthread_create(&line, NULL, lineSensor, &arguments);

    // // Join/Wait for threads to finish
    pthread_join(ir, NULL);
    pthread_join(line, NULL);
    */

// digitalWrite(MOTOR_DIR_FR, 1);
// digitalWrite(MOTOR_DIR_FL, 1);
// digitalWrite(MOTOR_DIR_RR, 1);
// digitalWrite(MOTOR_DIR_RL, 1);

// startColdMotors();
// while (1)
// {
// }
// // Clear Pins

// void clearPins()
// {
//   pinMode(MOTOR_VOLT_FL, INPUT)
//       pinMode(MOTOR_VOLT_FR, INPUT)
//           pinMode(MOTOR_VOLT_RL, INPUT)
//               pinMode(MOTOR_VOLT_RR, INPUT)
//                   pinMode(MOTOR_DIR_FL, INPUT);
//   pinMode(MOTOR_DIR_FR, INPUT);
//   pinMode(MOTOR_DIR_RL, INPUT);
//   pinMode(MOTOR_DIR_RR, INPUT);

//   pinMode(LINE_SENSOR_L_VCC, INPUT);
//   pinMode(LINE_SENSOR_R_VCC, INPUT);
//   pinMode(LINE_SENSOR_C_VCC, INPUT);

//   digitalWrite(LINE_SENSOR_L_VCC, LOW);
//   digitalWrite(LINE_SENSOR_R_VCC, LOW);
//   digitalWrite(LINE_SENSOR_C_VCC, LOW);
//   //TODO clean newly added pins as they come online
// }

// void startColdMotors()
// {
//   softPwmWrite(MOTOR_VOLT_FL, 100);
//   softPwmWrite(MOTOR_VOLT_FR, 100);
//   softPwmWrite(MOTOR_VOLT_RL, 100);
//   softPwmWrite(MOTOR_VOLT_RR, 100);
// }

//temp thread, taken from assignment 5
//checks for obstacles
void *irSensor(void *value)
{
  args *arguments = (struct args *)value;
  unsigned int start = micros(), run = micros(), total = micros();
  double tot;
  double cm = 1000;

  void updateDistance(unsigned int *start, double *total, double *cm)
  {
    digitalWrite(ECHO_TRIG, 0);
    delayMicroseconds(10);
    digitalWrite(ECHO_TRIG, 1);
    delayMicroseconds(10);
    digitalWrite(ECHO_TRIG, 0);
    *start = micros();
    while (digitalRead(ECHO_ECHO) == 0)
    {
      *start = micros();
    }
    while (digitalRead(ECHO_ECHO) == 1)
    {
      *total = micros();
    }
    *total = total - start;
    *cm = (total / 2.0) * .0340;
  }

  while (arguments->runFlag == 1)
  {

    updateDistance(&start, &total, &cm);
    if (cm < OBSTACLE_DISTANCE)
    {
      arguments->obstacleDetected = 1;
      stopCar();
      delay(OBSTACLE_WAIT_TIME * MILLISEC);
      updateDistance(&start, &total, &cm);
      if (cm > OBSTACLE_DISTANCE)
      {
        arguments->obstacleDetected = 0;
      }
      else
      {
        /** TODO */
        //evade
      }
    }
    else
  }
}

//temp thread, taken from assignment 5
//checks for line, if not avoiding obstacle this adjusts heading
void *lineSensor(void *value)
{
  args *arguments = (struct args *)value;
  while (arguments->runFlag == 1)
  {
    while (arguments->obstacleDetected == 0)
    {
      int n = 0;
      if (digitalRead(LINE_SENSOR_R_DO) == 1)
      {
        n += 1;
      }
      if (digitalRead(LINE_SENSOR_C_DO) == 1)
      {
        n += 2;
      }
      if (digitalRead(LINE_SENSOR_L_DO) == 1)
      {
        n += 4;
      }
      //may slow front motors to a stop
      //add conditions to maintain minimum speed
      switch (n)
      {
      case 0:
        //line lost
        break;
      case 1:
        //adjust hard right
        if (arguments->motor1_c1 > MIN_SPEED && arguments->motor2_c1 > MIN_SPEED)
        {
          arguments->motor1_c1 -= (2 * ADJUST);
          digitalWrite(MOTOR_1_CONTROL_1, arguments->motor1_c1);
        }
        //
        else
        {
          arguments->motor2_c1 += (2 * ADJUST);
          arguments->motor1_c1 = MIN_SPEED;
          digitalWrite(MOTOR_2_CONTROL_1, arguments->motor2_c1);
          digitalWrite(MOTOR_1_CONTROL_1, arguments->motor2_c1);
        }
        break;
      case 3:
        //adjust soft right
        if (arguments->motor1_c1 > MIN_SPEED && arguments->motor2_c1 > MIN_SPEED)
        {
          arguments->motor1_c1 -= (ADJUST);
          digitalWrite(MOTOR_1_CONTROL_1, arguments->motor1_c1);
        }
        else
        {
          arguments->motor2_c1 += (ADJUST);
          arguments->motor1_c1 = MIN_SPEED;
          digitalWrite(MOTOR_2_CONTROL_1, arguments->motor2_c1);
          digitalWrite(MOTOR_1_CONTROL_1, arguments->motor2_c1);
        }
        break;
      case 4:
        //adjust hard left
        if (arguments->motor2_c1 > MIN_SPEED && arguments->motor1_c1 > MIN_SPEED)
        {
          arguments->motor2_c1 -= (2 * ADJUST);
          digitalWrite(MOTOR_2_CONTROL_1, arguments->motor2_c1);
        }
        else
        {
          arguments->motor1_c1 += (2 * ADJUST);
          arguments->motor2_c1 = MIN_SPEED;
          digitalWrite(MOTOR_2_CONTROL_1, arguments->motor2_c1);
          digitalWrite(MOTOR_1_CONTROL_1, arguments->motor2_c1);
        }
        break;
      case 6:
        //adjust soft left
        if (arguments->motor2_c1 > MIN_SPEED && arguments->motor1_c1 > MIN_SPEED)
        {
          arguments->motor2_c1 -= (ADJUST);
          digitalWrite(MOTOR_2_CONTROL_1, arguments->motor2_c1);
        }
        else
        {
          arguments->motor1_c1 += (ADJUST);
          arguments->motor2_c1 = MIN_SPEED;
          digitalWrite(MOTOR_2_CONTROL_1, arguments->motor2_c1);
          digitalWrite(MOTOR_1_CONTROL_1, arguments->motor2_c1);
        }
        break;
      default:
        //on line or edgecase
        break;
      }
    }
  }
}

//init Motors
// pinMode(MOTOR_VOLT_FL, OUTPUT);
// pinMode(MOTOR_VOLT_FR, OUTPUT);
// pinMode(MOTOR_VOLT_RL, OUTPUT);
// pinMode(MOTOR_VOLT_RR, OUTPUT);
// pinMode(MOTOR_DIR_FL, OUTPUT);
// pinMode(MOTOR_DIR_FR, OUTPUT);
// pinMode(MOTOR_DIR_RL, OUTPUT);
// pinMode(MOTOR_DIR_RR, OUTPUT);

// softPwmCreate(MOTOR_VOLT_FL, 0, 100);
// softPwmCreate(MOTOR_VOLT_FR, 0, 100);
// softPwmCreate(MOTOR_VOLT_RL, 0, 100);
// softPwmCreate(MOTOR_VOLT_RR, 0, 100);

// digitalWrite(MOTOR_DIR_FL, HIGH);
// digitalWrite(MOTOR_DIR_FR, HIGH);
// digitalWrite(MOTOR_DIR_RL, HIGH);
// digitalWrite(MOTOR_DIR_RR, HIGH);

// //init Line Sensors
// pinMode(LINE_SENSOR_L_VCC, OUTPUT);
// pinMode(LINE_SENSOR_R_VCC, OUTPUT);
// pinMode(LINE_SENSOR_C_VCC, OUTPUT);

// pinMode(LINE_SENSOR_L_DO, INPUT);
// pinMode(LINE_SENSOR_R_DO, INPUT);
// pinMode(LINE_SENSOR_C_DO, INPUT);

// digitalWrite(LINE_SENSOR_L_VCC, HIGH);
// digitalWrite(LINE_SENSOR_R_VCC, HIGH);
// digitalWrite(LINE_SENSOR_C_VCC, HIGH);

// TODO
//init Servo
//init Collision Detection