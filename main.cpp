#include "BlueMotor.h"
#include <Rangefinder.h>
#include <Arduino.h>
#include <Romi32U4.h>
#include "Timer.h"
#include <servo32u4.h>
#include <IRdecoder.h>
#include <Chassis.h>
#include <ir_codes.h>

Rangefinder rangefinder(17, 12);
BlueMotor motor;
Romi32U4ButtonB buttonB;
Romi32U4ButtonA buttonA;
Romi32U4ButtonC buttonC;
const uint8_t IRPIN = 14;
IRDecoder decoder(IRPIN);
Chassis chassis;

#define PLAY_PAUSE 1
#define NUM_1 16
#define NUM_2 17
#define NUM_3 18
#define NUM_4 19
#define NUM_5 20
#define NUM_6 21
#define NUM_7 22
#define NUM_8 23
#define NUM_9 24
#define LEFT_ARROW 8
#define RIGHT_ARROW 10

#define DRIVE_TO_HOUSE 1
#define GRIP_PANEL 2
#define ROTATE 3
#define DRIVE_TO_PLATFORM 4
#define RELEASE_PANEL 5
#define STOP 6 
#define prep 7

long timeToPrint = 0;
long now = 0;
long newPosition = 0;
long oldPosition = 0;
long sampleTime = 10;
long speedInRPM = 0;
long CPR = 270;
int motorEffort = 0;
long positionDifference = 0;
long leftsensor = 0;
long rightsensor = 0;
int task = 0;
int state = prep;
long error;
int gainlinefollow = 0.06;
int drivespeed = 0;
int leftEffort = 0;
int rightEffort = 0;
int armState = 0;
int distanceTo = 100;
bool secondTime = false;
int effort = 0; 
int laststate = prep;
long long drivetime = 0;



void setup()
{
    Serial.begin(9600);
    decoder.init();
    chassis.init();
    rangefinder.init();
    motor.setup();
    motor.reset();
}
void loop()
{
  int16_t keyPress = decoder.getKeyCode();

  if(keyPress == NUM_3){
    task = 4;
    state = prep;
  }
  if(keyPress == RIGHT_ARROW){
    task = 2;
    state = GRIP_PANEL;
  } 
  if(keyPress == NUM_1){
    task = 3;
    
  }
  if(keyPress == LEFT_ARROW){
    task = 1;
    state = prep;
  } 
  if(keyPress == NUM_2){
    task = 5;
    state = GRIP_PANEL;
  } 
  if(keyPress == PLAY_PAUSE){
    task = 6;
  }

  if(task == 1){
     // task 1
     // 37 away from house, 10 away from plate
    if(state == prep){
      motor.gripperOpen();
      motor.moveTo45DegreeRoof();
      state = DRIVE_TO_HOUSE;
      Serial.println("done");
    }
    if(state == DRIVE_TO_HOUSE){
      Serial.println("going");
      distanceTo = rangefinder.getDistance();
      leftsensor = motor.getLeftLineSensor();
      rightsensor = motor.getRightLineSensor();
      error = leftsensor - rightsensor;
      effort = error * 0.02;
      Serial.println(distanceTo);
      if(distanceTo > 10){
        chassis.setMotorEfforts(-60 - effort,-60 + effort ); // line following
      }else{
        chassis.setMotorEfforts(0,0);
        state = GRIP_PANEL;
      }
    }
    if(state == GRIP_PANEL){
      motor.gripperClose();
      motor.moveTo(-4500);
      chassis.driveFor(5, -15, true);
      state = ROTATE;
    }
    if(state == ROTATE){
      //chassis.turnFor(175, 80, true);
      chassis.turnFor(180, 60, true);
      state = DRIVE_TO_PLATFORM;
    }
    if(state == DRIVE_TO_PLATFORM){
      leftsensor = motor.getLeftLineSensor();
      rightsensor = motor.getRightLineSensor();
      chassis.setMotorEfforts(-70, -70);
      if(leftsensor > 500 && rightsensor > 500){
        chassis.setMotorEfforts(0,0);
        state = RELEASE_PANEL; 
      }
    }
    if(state == RELEASE_PANEL){
      motor.moveToStagingPlatform();
      motor.gripperOpen();
      task = 7;
    }
  }

  if(task == 2){
    if(state == GRIP_PANEL){
      motor.gripperClose();
      motor.moveTo(-4000);
      state = ROTATE;
    }
    if(state == ROTATE){
      chassis.driveFor(5, -15, true);
      chassis.turnFor(175, 80, true);
      state = DRIVE_TO_HOUSE;
    }
    if(state == DRIVE_TO_HOUSE){ 
      leftsensor = motor.getLeftLineSensor();
      rightsensor = motor.getRightLineSensor();
      error = leftsensor - rightsensor;
      effort = error * 0.02;
      Serial.println(distanceTo);
      if(rangefinder.getDistance() > 10){
        chassis.setMotorEfforts(-60 - effort,-60 + effort );
         // line following 
      }else{
        chassis.setMotorEfforts(0,0);
        state = RELEASE_PANEL;
      }
    }
    if(state == RELEASE_PANEL){
      motor.moveTo45DegreeRoof();
      motor.gripperOpen();
      state = prep;
      task = 7;
    }
  }

  if(task == 3){
    Serial.println(state);
    if(state == prep){
      chassis.driveFor(15, 15, true);
      chassis.turnFor(120, 80, true);
      state = DRIVE_TO_HOUSE;
    }
    if(state == DRIVE_TO_HOUSE){
      laststate = DRIVE_TO_HOUSE;
      leftsensor = motor.getLeftLineSensor();
      rightsensor = motor.getRightLineSensor();
      if(leftsensor < 400 && rightsensor < 400){
        chassis.setMotorEfforts(-60,-60);
      }else{
        state = DRIVE_TO_PLATFORM; //line following
      }
    }
    if(state == DRIVE_TO_PLATFORM){ // line following state
      leftsensor = motor.getLeftLineSensor();
      rightsensor = motor.getRightLineSensor();
      error = leftsensor - rightsensor;
      effort = error * 0.02;
      if(leftsensor > 400 && rightsensor > 400){
        chassis.driveFor(-8, -20, true);
        chassis.turnFor(-90,-60, true);
        task = 7;
      }else{
        chassis.setMotorEfforts(-60 - effort,-60 + effort ); // line following
      }
    }
  }

  if(task == 4){
    if(state == prep){
      motor.gripperOpen();
      motor.moveTo25DegreeRoof();
      state = DRIVE_TO_HOUSE;
      Serial.println("done");
    }
    if(state == DRIVE_TO_HOUSE){
      Serial.println("going");
      distanceTo = rangefinder.getDistance();
      leftsensor = motor.getLeftLineSensor();
      rightsensor = motor.getRightLineSensor();
      error = leftsensor - rightsensor;
      effort = error * 0.02;
      Serial.println(distanceTo);
      if(distanceTo > 5){
        chassis.setMotorEfforts(-60 - effort,-60 + effort ); // line following
      }else{
        chassis.driveFor(-5, 5,true);
        chassis.setMotorEfforts(0,0); 
        state = GRIP_PANEL;
      }
    }
    if(state == GRIP_PANEL){
      motor.gripperClose();
      motor.moveTo(-8500);
      chassis.driveFor(10, -15, true);
      state = ROTATE;
      
    }
    if(state == ROTATE){
      //chassis.turnFor(175, 80, true);
      chassis.turnFor(180, 60, true);
      state = DRIVE_TO_PLATFORM;
    }
    if(state == DRIVE_TO_PLATFORM){
      leftsensor = motor.getLeftLineSensor();
      rightsensor = motor.getRightLineSensor();
      chassis.setMotorEfforts(-50, -50);
      if(leftsensor > 500 && rightsensor > 500){
        chassis.setMotorEfforts(0,0);
        state = RELEASE_PANEL; 
      }
    }
    if(state == RELEASE_PANEL){
      motor.moveToStagingPlatform();
      motor.gripperOpen();
      task = 7;
    }
  }
  
  if(task == 5) {
    if(state == GRIP_PANEL){
      motor.gripperClose();
      motor.moveTo(-8500);
      state = ROTATE;
    }
    if(state == ROTATE){
      chassis.driveFor(5, -15, true);
      chassis.turnFor(180, 80, true);
      state = DRIVE_TO_HOUSE;
    }
    if(state == DRIVE_TO_HOUSE){ 
      leftsensor = motor.getLeftLineSensor();
      rightsensor = motor.getRightLineSensor();
      error = leftsensor - rightsensor;
      effort = error * 0.02;
       if(rangefinder.getDistance() > 5){
        chassis.setMotorEfforts(-60 - effort,-60 + effort ); // line following
      }else{
        chassis.driveFor(-5, 5,true);
        chassis.setMotorEfforts(0,0);
        state = RELEASE_PANEL;
      }
    }
    if(state == RELEASE_PANEL){
      motor.moveTo25DegreeRoof();
      motor.gripperOpen();
      state = prep;
      task = 7;
    }
  }

  if(task == 6){ //estop
    chassis.setMotorEfforts(0,0);
    Serial.println(rangefinder.getDistance());
    state = DRIVE_TO_HOUSE;
   
  }
  if(task == 7){
    chassis.setMotorEfforts(0,0);
  }

}

