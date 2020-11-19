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

int main(void)
{
  // wiringPiSetup();
  // initializePins();

  // //initialize arguments
  // pthread_t ir;
  // pthread_t line;

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