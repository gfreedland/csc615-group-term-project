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
    args *arguments = (struct args*)value;
    while(arguments->runFlag == 1){
        if(digitalRead(IR_OUT) == 1){
            printf("Obstacle Detected\n");
        }
    }
}

//temp thread, taken from assignment 5
void * lineSensor(void *value){
    args *arguments = (struct args*)value;
    while(arguments->runFlag == 1){
        if(digitalRead(LINE_SENSOR_DO) == 0){
            printf("No Longer on the Line\n");
        }
    }
}

int main(void)
{
  wiringPiSetup();
  initializePins();

  // //initialize arguments
  pthread_t ir;
  pthread_t line;
  
  // // Run program until threads finish
  // pthread_create(&ir, NULL, irSensor, NULL);
  // pthread_create(&line, NULL, lineSensor, NULL);

  // // Join/Wait for threads to finish
  // pthread_join(ir, NULL);
  // pthread_join(line, NULL);

  // // Clear Pins
  // clearPins();
  printf("Program Ended");
}



void clearPins(){


}