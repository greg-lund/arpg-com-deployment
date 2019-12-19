#include <Arduino.h>
#include "Wiring.h"
#include "AccelStepper.h"
#include <ros.h>
#include <beacon/Empty.h>
#include <beacon/Beacon.h>

#define MAX_HOME -10000000
#define MAX_SPEED 4200
#define ACCEL 1600
#define MICROSTEPS 8
#define NUM_TEETH 14
#define BELT_PITCH 2
#define DEPLOY_DIST 400 //Distance to deploy in mm
#define DEPLOY_LOC (200 * MICROSTEPS / NUM_TEETH / BELT_PITCH * DEPLOY_DIST)

/* STEPPER MOTOR SETUP */
AccelStepper stepperX = AccelStepper(1,X_STEP, X_DIR);
AccelStepper stepperY = AccelStepper(1,Y_STEP, Y_DIR);
AccelStepper stepperZ = AccelStepper(1,Z_STEP, Z_DIR);
AccelStepper steppers[3] = {AccelStepper(1,X_STEP,X_DIR), AccelStepper(1,Y_STEP,Y_DIR), AccelStepper(1,Z_STEP,Z_DIR)};

/* ROS SETUP */
ros::NodeHandle nh;

void home_all(const beacon::Empty::Request &req, beacon::Empty::Response &res) {
  stepperX.moveTo(MAX_HOME);
  stepperY.moveTo(MAX_HOME);
  stepperZ.moveTo(MAX_HOME);
  while(digitalRead(X_LIM) == HIGH || digitalRead(Y_LIM) == HIGH || digitalRead(Z_LIM) == HIGH) {
    if(digitalRead(X_LIM) == HIGH){
      stepperX.run();
    } else {
      //stepperX.stop();
    }
    if(digitalRead(Y_LIM) == HIGH){
      stepperY.run();
    } else {
      //stepperY.stop();
    }
    if(digitalRead(Z_LIM) == HIGH) {
      stepperZ.run();
    } else {
      //stepperZ.stop();
    }
  }
  stepperX.setCurrentPosition(0);
  stepperY.setCurrentPosition(0);
  stepperZ.setCurrentPosition(0);
}

ros::ServiceServer<beacon::Empty::Request, beacon::Empty::Response> home_all_service("home_all", &home_all);

void home(const beacon::Beacon::Request &req,beacon::Beacon::Response &res) {
  if(req.beacon_id == 0) {
    stepperX.moveTo(MAX_HOME);
    while(digitalRead(X_LIM) == HIGH) {
      stepperX.run();
    }
    stepperX.setCurrentPosition(0);
  }else if(req.beacon_id == 1) {
    stepperY.moveTo(MAX_HOME);
    while(digitalRead(Y_LIM) == HIGH) {
      stepperY.run();
    }
    stepperY.setCurrentPosition(0);
  }else if(req.beacon_id == 2) {
    stepperZ.moveTo(MAX_HOME);
    while(digitalRead(Z_LIM) == HIGH) {
      stepperZ.run();
    }
    stepperZ.setCurrentPosition(0);
  }
  return;
}

ros::ServiceServer<beacon::Beacon::Request, beacon::Beacon::Response> home_service("home", &home);

void deploy(const beacon::Beacon::Request &req,beacon::Beacon::Response &res) {
  if(req.beacon_id == 0) {
    stepperX.moveTo(DEPLOY_LOC);
    while(stepperX.currentPosition() != DEPLOY_LOC) {
      stepperX.run();
    }
  }else if(req.beacon_id == 1) {
    stepperY.moveTo(DEPLOY_LOC);
    while(stepperY.currentPosition() != DEPLOY_LOC) {
      stepperY.run();
    }
  }else if(req.beacon_id == 2) {
    stepperZ.moveTo(DEPLOY_LOC);
    while(stepperZ.currentPosition() != DEPLOY_LOC) {
      stepperZ.run();
    }
  }
  return;
}

ros::ServiceServer<beacon::Beacon::Request, beacon::Beacon::Response> deploy_service("deploy", &deploy);

void setup() {
  //Set pullup resistors for limit switches
  pinMode(X_LIM, INPUT_PULLUP);
  pinMode(Y_LIM, INPUT_PULLUP);
  pinMode(Z_LIM, INPUT_PULLUP);

  //Set acceleration and speed 
  stepperX.setMaxSpeed(MAX_SPEED);
  stepperX.setAcceleration(ACCEL);
  stepperY.setMaxSpeed(MAX_SPEED);
  stepperY.setAcceleration(ACCEL);
  stepperZ.setMaxSpeed(MAX_SPEED / 2);
  stepperZ.setAcceleration(ACCEL / 2);


  //Setup ros
  nh.initNode();
  nh.advertiseService(home_service);
  nh.advertiseService(deploy_service);
  nh.advertiseService(home_all_service);
}

void loop() {
  nh.spinOnce();
  delay(1);
}
