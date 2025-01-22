#include "vex.h"

using namespace vex;

vex::brain  Brain;

competition Competition;

// Global variable for autonomous selection
int autonomousSelection = 0;

// Define autonomous routine functions
void leftAutonRed() {
    Brain.Screen.clearScreen();
    Brain.Screen.setFillColor(red);
    Brain.Screen.drawRectangle(0, 0, 480, 240);
    Brain.Screen.setCursor(6, 16);
    Brain.Screen.print("Executing LEFT AUTON - RED");
    // Add your autonomous code for left red here
}

void rightAutonRed() {
    Brain.Screen.clearScreen();
    Brain.Screen.setFillColor(red);
    Brain.Screen.drawRectangle(0, 0, 480, 240);
    Brain.Screen.setCursor(6, 16);
    Brain.Screen.print("Executing RIGHT AUTON - RED");
    // Add your autonomous code for right red here
}

void leftAutonBlue() {
    Brain.Screen.clearScreen();
    Brain.Screen.setFillColor(blue);
    Brain.Screen.drawRectangle(0, 0, 480, 240);
    Brain.Screen.setCursor(6, 16);
    Brain.Screen.print("Executing LEFT AUTON - BLUE");
    // Add your autonomous code for left blue here
}

void rightAutonBlue() {
    Brain.Screen.clearScreen();
    Brain.Screen.setFillColor(blue);
    Brain.Screen.drawRectangle(0, 0, 480, 240);
    Brain.Screen.setCursor(6, 16);
    Brain.Screen.print("Executing RIGHT AUTON - BLUE");
    // Add your autonomous code for right blue here
}

// Autonomous program selector
void pre_auton() {
    while (true) {
        // Draw the selection screen
        Brain.Screen.clearScreen();
        Brain.Screen.setFillColor(red);
        Brain.Screen.drawRectangle(0, 0, 240, 120); // Top-left
        Brain.Screen.setFillColor(red);
        Brain.Screen.drawRectangle(240, 0, 240, 120); // Top-right
        Brain.Screen.setFillColor(blue);
        Brain.Screen.drawRectangle(0, 120, 240, 120); // Bottom-left
        Brain.Screen.setFillColor(blue);
        Brain.Screen.drawRectangle(240, 120, 240, 120); // Bottom-right

        // Wait for user input
        waitUntil(Brain.Screen.pressing());
        int x = Brain.Screen.xPosition();
        int y = Brain.Screen.yPosition();

        // Determine the selected option
        if (y < 120) { // Top row
            if (x < 240) {
                autonomousSelection = 0; // Left Red
                Brain.Screen.clearScreen();
                Brain.Screen.print("Selected: LEFT AUTON - RED");
            } else {
                autonomousSelection = 1; // Right Red
                Brain.Screen.clearScreen();
                Brain.Screen.print("Selected: RIGHT AUTON - RED");
            }
        } else { // Bottom row
            if (x < 240) {
                autonomousSelection = 2; // Left Blue
                Brain.Screen.clearScreen();
                Brain.Screen.print("Selected: LEFT AUTON - BLUE");
            } else {
                autonomousSelection = 3; // Right Blue
                Brain.Screen.clearScreen();
                Brain.Screen.print("Selected: RIGHT AUTON - BLUE");
            }
        }

        // Wait a bit for screen feedback
        wait(2, seconds);
        return; // Exit the selection loop once a choice is made
    }
}

// Autonomous function
void autonomous() {
    switch (autonomousSelection) {
        case 0:
            leftAutonRed();
            break;
        case 1:
            rightAutonRed();
            break;
        case 2:
            leftAutonBlue();
            break;
        case 3:
            rightAutonBlue();
            break;
        default:
            Brain.Screen.print("No autonomous selected!");
            break;
    }
}

// User control function
void usercontrol() {
    while (true) {
        wait(20, msec); // Sleep to prevent wasted resources
    }
}

// Main function
int main() {
    // Set up callbacks for autonomous and driver control periods
    Competition.autonomous(autonomous);
    Competition.drivercontrol(usercontrol);

    // Run pre-autonomous function for setup
    pre_auton();

    // Keep the program running
    while (true) {
        wait(100, msec);
    }
}
