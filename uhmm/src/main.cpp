/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       Admin                                                     */
/*    Created:      11/27/2024, 11:18:17 AM                                    */
/*    Description:  Copius vex project                                         */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"

using namespace vex;


// A global instance of competition
competition Competition;

// Global defined for autonomous program

// define your global instances of motors and other devices here

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {
      Brain.Screen.printAt( 10, 50, "Hello V5" );
        while (true) {
          Brain.Screen.setFillColor(red);
          Brain.Screen.drawRectangle(0, 0, 240, 120);
          Brain.Screen.setFillColor(red);
          Brain.Screen.drawRectangle(240, 0, 240, 120);
          Brain.Screen.setFillColor(blue);
          Brain.Screen.drawRectangle(0, 120, 240, 120);
          Brain.Screen.setFillColor(blue);
          Brain.Screen.drawRectangle(240, 120, 240, 120);


          waitUntil(Brain.Screen.pressing());
          if (Brain.Screen.yPosition() < 120.0) {
          if (Brain.Screen.xPosition() < 240.0) {
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
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void autonomous(void) {
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
  // User control code here, inside the loop
  while (1) {
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.

    // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.
    // ........................................................................

    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
