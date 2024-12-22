/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       Admin                                                     */
/*    Created:      11/20/2024, 3:52:10 PM                                    */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include "vex.h"

using namespace vex;

// A global instance of vex::brain used for printing to the V5 brain screen
vex::brain       Brain;

// define your global instances of motors and other devices here


int main() {

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
   
    while(1) {
        
        // Allow other tasks to run
        this_thread::sleep_for(10);
    }
}
