/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       C:\Users\Admin                                            */
/*    Created:      Fri Nov 01 2024                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// Drivetrain           drivetrain    1, 2, 3, 4      
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;

// User defined function
void myblockfunction_Menu_Button();

int Brain_precision = 0, Console_precision = 0;

float myVariable;

// User defined function
void myblockfunction_Menu_Button() {
  Brain.Screen.setFillColor(red);
  Brain.Screen.drawRectangle(0, 0, 240, 120);
  Brain.Screen.setFillColor(red);
  Brain.Screen.drawRectangle(240, 0, 240, 120);
  Brain.Screen.setFillColor(blue);
  Brain.Screen.drawRectangle(0, 120, 240, 120);
  Brain.Screen.setFillColor(blue);
  Brain.Screen.drawRectangle(240, 120, 240, 120);
}

// "when started" hat block
int whenStarted1() {
  while (true) {
    myblockfunction_Menu_Button();
    waitUntil(Brain.Screen.pressing());
    if (Brain.Screen.xPosition() < 240.0) {
      if (Brain.Screen.yPosition() < 120.0) {
        Brain.Screen.setFillColor(red);
        Brain.Screen.drawRectangle(0, 0, 480, 240);
        Brain.Screen.setCursor(6, 16);
        Brain.Screen.print("LEFT AUTON");
      }
      else {
        Brain.Screen.setFillColor(red);
        Brain.Screen.drawRectangle(0, 0, 480, 240);
        Brain.Screen.setCursor(6, 16);
        Brain.Screen.print("RIGHT AUTON");
      }
    }
    else {
      if (Brain.Screen.xPosition() < 240.0) {
        Brain.Screen.setFillColor(blue);
        Brain.Screen.drawRectangle(0, 0, 480, 240);
        Brain.Screen.setCursor(6, 16);
        Brain.Screen.print("LEFT AUTON");
      }
      else {
        Brain.Screen.setFillColor(blue);
        Brain.Screen.drawRectangle(0, 0, 480, 240);
        Brain.Screen.setCursor(6, 16);
        Brain.Screen.print("RIGHT AUTON");
      }
    }
    wait(2.0, seconds);
  wait(5, msec);
  }
  return 0;
}
//___________________________________________________________________________________//

int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

}
