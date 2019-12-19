#include <Arduino.h>
#include "Wiring.h"
#include "AccelStepper.h"
#include <ros.h>
#include <beacon/Empty.h>
#include <beacon/Beacon.h>

#define MAX_HOME -10000000
#define MAX_SPEED 1200
#define ACCEL 1200
#define MICROSTEPS 8
#define NUM_TEETH 14
#define BELT_PITCH 2
#define DEPLOY_DIST 100 //Distance to deploy in mm
#define DEPLOY_LOC (200 * MICROSTEPS / NUM_TEETH / BELT_PITCH * DEPLOY_DIST)

/* STEPPER MOTOR SETUP */
AccelStepper steppers[3] = {AccelStepper(1,X_STEP,X_DIR), AccelStepper(1,Y_STEP,Y_DIR), AccelStepper(1,Z_STEP,Z_DIR)};

/* ROS SETUP */
ros::NodeHandle nh;

void home_all(const beacon::Empty::Request &req, beacon::Empty::Response &res) {
  for(int i = 0; i < 3; i++) {
    steppers[i].moveTo(MAX_HOME);
  }

  while(digitalRead(lim[0]) == HIGH || digitalRead(lim[1]) == HIGH || digitalRead(lim[2]) == HIGH) {
    for(int i = 0; i < 3; i++) {
      if(digitalRead(lim[i]) == HIGH) {
        steppers[i].run();
      }
    }
  }

  for(int i = 0; i < 3; i++) {
    steppers[i].setCurrentPosition(0);
  }
}

ros::ServiceServer<beacon::Empty::Request, beacon::Empty::Response> home_all_service("home_all", &home_all);

void home(const beacon::Beacon::Request &req,beacon::Beacon::Response &res) {
  if(req.beacon_id < 0 || req.beacon_id > 2) {
    nh.loginfo("Invalid beacon_id");
    return;
  }
  int id = req.beacon_id;
  steppers[id].moveTo(MAX_HOME);
  while(digitalRead(lim[id]) == HIGH) {
    steppers[id].run();
  }
  steppers[id].setCurrentPosition(0);
  return;
}

ros::ServiceServer<beacon::Beacon::Request, beacon::Beacon::Response> home_service("home", &home);

void deploy(const beacon::Beacon::Request &req,beacon::Beacon::Response &res) {
  if(req.beacon_id < 0 || req.beacon_id > 2) {
    nh.loginfo("Invalid beacon_id");
    return;
  }
  int id = req.beacon_id;
  steppers[id].moveTo(DEPLOY_LOC);
  while(steppers[id].currentPosition() != DEPLOY_LOC) {
    steppers[id].run();
  }
  return;
}

ros::ServiceServer<beacon::Beacon::Request, beacon::Beacon::Response> deploy_service("deploy", &deploy);

void setup() {
  //Set pullup resistors for limit switches
  //Set acceleration and speed 
  for(int i = 0; i < 3; i++) {
    pinMode(lim[i], INPUT_PULLUP);
    steppers[i].setMaxSpeed(MAX_SPEED);
    steppers[i].setAcceleration(ACCEL);
  }

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
