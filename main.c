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
#include <pthread.h>
#include <wiringPi.h>

//TEMPORARY DEFITIONS: Change as pins are chosen
//Motor Voltage Control and Direction
#define MOTOR_VOLT_FL 0;
#define MOTOR_VOLT_FR 0;
#define MOTOR_VOLT_RL 0;
#define MOTOR_VOLT_RR 0;
#define MOTOR_DIR_FL 0;
#define MOTOR_DIR_FR 0;
#define MOTOR_DIR_RL 0;
#define MOTOR_DIR_RR 0;
//Line Sensor Voltage Control and Digital Output
#define LINE_SENSOR_L_VCC 0;
#define LINE_SENSOR_R_VCC 0;
#define LINE_SENSOR_C_VCC 0;
#define LINE_SENSOR_L_DO 0;
#define LINE_SENSOR_R_DO 0;
#define LINE_SENSOR_C_DO 0;

const int MILLISEC = 1000;
const int OBSTACLE_WAIT_TIME = 5;
const int MAX_RUN_TIME = 90;

typedef struct args{
    int runFlag;
    int obstacleDetected;
    //
} args;

void initilalizePins(){
  //init Motors
  pinMode(MOTOR_VOLT_FL, OUTPUT);
  pinMode(MOTOR_VOLT_FR, OUTPUT);
  pinMode(MOTOR_VOLT_RL, OUTPUT);
  pinMode(MOTOR_VOLT_RR, OUTPUT);
  pinMode(MOTOR_DIR_FL, OUTPUT);
  pinMode(MOTOR_DIR_FR, OUTPUT);
  pinMode(MOTOR_DIR_RL, OUTPUT);
  pinMode(MOTOR_DIR_RR, OUTPUT);

  softPwmCreate(MOTOR_VOLT_FL,0,100);
  softPwmCreate(MOTOR_VOLT_FR,0,100);
  softPwmCreate(MOTOR_VOLT_RL,0,100);
  softPwmCreate(MOTOR_VOLT_RR,0,100); 

  digitalWrite(MOTOR_DIR_FL,HIGH);
  digitalWrite(MOTOR_DIR_FR,HIGH);
  digitalWrite(MOTOR_DIR_RL,HIGH);
  digitalWrite(MOTOR_DIR_RR,HIGH);

  //init Line Sensors
  pinMode(LINE_SENSOR_L_VCC, OUTPUT);
  pinMode(LINE_SENSOR_R_VCC, OUTPUT);
  pinMode(LINE_SENSOR_C_VCC, OUTPUT);

  pinMode(LINE_SENSOR_L_DO, INPUT);
  pinMode(LINE_SENSOR_R_DO, INPUT);
  pinMode(LINE_SENSOR_C_DO, INPUT); 

  digitalWrite(LINE_SENSOR_L_VCC,HIGH);
  digitalWrite(LINE_SENSOR_R_VCC,HIGH);
  digitalWrite(LINE_SENSOR_C_VCC,HIGH);

  // TODO
  //init Servo
  //init Collision Detection

}

//temp thread, taken from assignment 5
void * irSensor(void *value){
  /*
    args *arguments = (struct args*)value;
    while(arguments->runFlag == 1){
        if(digitalRead(IR_OUT) == 1){
            printf("Obstacle Detected\n");
        }
    }
    */
}

//temp thread, taken from assignment 5
void * lineSensor(void *value){
  /*
    args *arguments = (struct args*)value;
    while(arguments->runFlag == 1){
        if(digitalRead(LINE_SENSOR_DO) == 0){
            printf("No Longer on the Line\n");
        }
    }
    */
}

int main(void)
{
  wiringPiSetup();
  initializePins();

  // //initialize arguments
  pthread_t ir;
  pthread_t line;
  
  args arguments;
  arguments.runFlag = 1;
  arguments.obstacleDetected = 0;

  // // Run program until threads finish
  pthread_create(&ir, NULL, irSensor, &arguments);
  pthread_create(&line, NULL, lineSensor, &arguments);

  // // Join/Wait for threads to finish
  pthread_join(ir, NULL);
  pthread_join(line, NULL);

  // // Clear Pins
  clearPins();
  printf("Program Ended");
}

void clearPins(){
  pinMode(MOTOR_VOLT_FL, INPUT)
  pinMode(MOTOR_VOLT_FR, INPUT)
  pinMode(MOTOR_VOLT_RL, INPUT)
  pinMode(MOTOR_VOLT_RR, INPUT)
  pinMode(MOTOR_DIR_FL, INPUT);
  pinMode(MOTOR_DIR_FR, INPUT);
  pinMode(MOTOR_DIR_RL, INPUT);
  pinMode(MOTOR_DIR_RR, INPUT);

  pinMode(LINE_SENSOR_L_VCC, INPUT);
  pinMode(LINE_SENSOR_R_VCC, INPUT);
  pinMode(LINE_SENSOR_C_VCC, INPUT);

  digitalWrite(LINE_SENSOR_L_VCC,LOW);
  digitalWrite(LINE_SENSOR_R_VCC,LOW);
  digitalWrite(LINE_SENSOR_C_VCC,LOW);

}