#include "main.h"
#include "pros/apix.h"
#include "pros/colors.h"
#include <llemu.h>
#include <llemu.hpp>
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib-tarball/api.hpp" // IWYU pragma: keep

/// @brief Position value of Potentiometer
const double POTENTIOMETER_POSITION = 1050;

/// @brief Drivetrain RPM
const double DRIVETRAIN_RPM = 200 * GEAR_RATIO;

/// @brief The distance between the IMU and horizontal wall (in Inch)
const double HORRIZONTAL_OFFSET = 0; //not set yet

/// @brief The distance between the IMU and vertical wall (in Inch)
const double VERTICAL_OFFSET = 0; //not set yet

/// @brief Track width in inch.
const double TRACK_WIDTH = 14.05;

/// @brief Wheel circumference in mm.
const double WHEEL_CIRC = 262.0;

/// @brief Number of output gear teeth divided by number of input gear teeth.
const double GEAR_RATIO = 36.0 / 60.0;

/// @brief The speed of the conveyor as a percentage of its max speed (200 rpm).
const double CONVEYOR_SPEED_PERCENT = 100;

/// @brief The speed of the ladybrown as a percentage of its max speed (200 rpm).
const double LADYBROWN_SPEED_PERCENT = 75;

/// @brief Vision sensor signature ID for the red donut.
const uint32_t RED_SIG_ID = 1;

/// @brief Vision sensor signature ID for the blue donut.
const uint32_t BLUE_SIG_ID = 2;

/// @brief Delay in ms for the conveyor to stop after a donut of the wrong color
/// is detected.
const uint32_t CONVEYOR_DELAY = 350;

/// @brief Delay in ms for the conveyor to start after ejecting a donut of the
/// wrong color.
const uint16_t CONVEYOR_HALT = 200;

/// @brief Distance in degrees for the conveyor to move after a donut of the
/// wrong color is detected.
const double CONVEYOR_STOP_DISTANCE = 770;

/// @brief Minimum screen coverage for the donut to be detected using the vision
/// sensor.
const int MIN_SCREEN_COVERAGE = 100;

/// @brief Gear ratio for the conveyor motor.
const double CONVEYOR_GEAR_RATIO = 12.0 / 18.0;

/// @brief Number of teeth in the conveyor output sprocket.
const int CONVEYOR_OUT_TEETH = 12;

/// @brief Number of links in the conveyor.
const int CONVEYOR_LINKS = 74;

/// @brief Offset in links between the first and second conveyor hooks.
const int CONVEYOR_HOOK_2_OFFSET = 31;

/// @brief Number of motor degrees for the conveyor to make a full revolution.
const double CONVEYOR_REVOLUTION_DEGREES = double(CONVEYOR_LINKS) /
                                           double(CONVEYOR_OUT_TEETH) * 360.0 *
                                           CONVEYOR_GEAR_RATIO;

/// @brief Offset in degrees between the first and second conveyor hook.
const double CONVEYOR_HOOK_2_OFFSET_DEGREES = double(CONVEYOR_HOOK_2_OFFSET) /
                                              double(CONVEYOR_OUT_TEETH) *
                                              360.0 * CONVEYOR_GEAR_RATIO;

/// @brief Distance in conveyor motor degrees between the vision sensor and the
/// top of the conveyor.
const double DISTANCE_VISION_TO_TOP = 520;

/// @brief Enum for the colors of the donuts.
enum class DONUT_COLOR { RED, BLUE };

/// @brief Enum for motor spinning states.
enum class SPIN_STATE { STOP, FORWARD, REVERSE };

/// @brief Enum for donut color sorting states.
enum class SORTING_STATE { NOT_DETECTED, STANDBY, STOP };

// Define constants for screen dimensions and colors (do not fucking change this!!!) (in Pixel)
const int SCREEN_WIDTH = 480;
const int SCREEN_HEIGHT = 240;
const int REGION_WIDTH = SCREEN_WIDTH / 2;
const int REGION_HEIGHT = SCREEN_HEIGHT / 2;

// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// motor groups
pros::Motor ladyBrown(9, pros::MotorGearset::green); // Left front motor
pros::Motor intake(-6, pros::MotorGearset::green); // Intake motor
pros::Motor conveyor(-7, pros::MotorGearset::green); // Conveyor motor
pros::MotorGroup leftMotors({-1, -2}, pros::MotorGearset::green); // Left motor group
pros::MotorGroup rightMotors({3, 4}, pros::MotorGearset::green); // Right motor group
pros::ADIAnalogIn potentiometer('a');// Potentiometer on port A
pros::adi::DigitalOut piston('b'); // Line sensor on port B
pros::Vision vision_sensor(8); // Vision sensor on port 8  

// Inertial Sensor on port 13
pros::Imu imu(13);

/// Default autonomous selection
int autonomousSelection = 0; 

void recalibratePosition() {
    double x, y, theta; // Position variables
    if (autonomousSelection == 1) { //left auton red
        x = 70 - HORRIZONTAL_OFFSET;
        y = -23.6;
        theta = 270;
    } else if (autonomousSelection == 2) { //right auton red
        x = 70- HORRIZONTAL_OFFSET;
        y = 23.6;
        theta = 0;
    } else if (autonomousSelection == 3) { //left auton blue
        x = HORRIZONTAL_OFFSET - 70;
        y = 23.6;
        theta = 0;
    } else if (autonomousSelection = 4) { //right auton blue
        x = HORRIZONTAL_OFFSET - 70;
        y = -23.6;
        theta = 90;
    } else {
        pros::lcd::set_text(1, "No default position found!");
    }
    chassis.setPose({x, y, theta});
}

// Utility function to draw a rectangle with a color
void drawAutonRegion(int x, int y, int width, int height) {
    pros::screen::fill_rect(x, y, x + width, y + height);
}

// Utility function to clear screen
void clearScreen() {
    pros::screen::set_pen(pros::c::COLOR_BLACK);
    pros::screen::fill_rect(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT);
}

// Utility function to draw the autonomous menu
void drawAutonMenu() {
    clearScreen();
    pros::screen::set_pen(pros::c::COLOR_RED);
    drawAutonRegion(0, 0, REGION_WIDTH, REGION_HEIGHT);          // Top-left
    drawAutonRegion(REGION_WIDTH, 0, REGION_WIDTH, REGION_HEIGHT);  // Top-right
    pros::screen::set_pen(pros::c::COLOR_BLUE);
    drawAutonRegion(0, REGION_HEIGHT, REGION_WIDTH, REGION_HEIGHT); // Bottom-left
    drawAutonRegion(REGION_WIDTH, REGION_HEIGHT, REGION_WIDTH, REGION_HEIGHT); // Bottom-right
}

// Utility function to select an autonomous 
void selectAutonRegion(int x, int y) {
    clearScreen();
    if (y <= 120 ) {
        pros::screen::set_pen(pros::c::COLOR_RED);
    } else {
        pros::screen::set_pen(pros::c::COLOR_BLUE);
    }

    drawAutonRegion(x, y, REGION_WIDTH, REGION_HEIGHT);

    // Wait for another screen press to return to the menu
    pros::screen::touch_callback(drawAutonMenu, TOUCH_PRESSED);
}

// Function to proccess the touch screen input
void autonSelection() {
    drawAutonMenu();
    while (true) {
        pros::screen_touch_status_s_t status = pros::screen::touch_status();
        auto touch = pros::screen::touch_status();
        int x = touch.x;
        int y = touch.y;

        // Determine which region was pressed and highlight it
        if (x < REGION_WIDTH && y < REGION_HEIGHT) {
            selectAutonRegion(0, 0); // Top-left
            autonomousSelection = 1;
            recalibratePosition();
        } else if (x >= REGION_WIDTH && y < REGION_HEIGHT) {
            selectAutonRegion(REGION_WIDTH, 0); // Top-right
            autonomousSelection = 2;
            recalibratePosition();
        } else if (x < REGION_WIDTH && y >= REGION_HEIGHT) {
            selectAutonRegion(0, REGION_HEIGHT); // Bottom-left
            autonomousSelection = 4;
            recalibratePosition();
        } else {
            selectAutonRegion(REGION_WIDTH, REGION_HEIGHT); // Bottom-right 
            autonomousSelection = 3;
            recalibratePosition();  
        }
    }
}

// Define autonomous routine functions
void leftAutonRed() {
    pros::lcd::clear();
    pros::lcd::set_text(3, "Executing LEFT AUTON - RED");
    // Add your autonomous code for left red here
}

void rightAutonRed() {
    pros::lcd::clear();
    pros::lcd::set_text(3, "Executing RIGHT AUTON - RED");
    //Follow set path
    chassis.follow(decoder["path1"],15,2000,false);
    chassis.follow(decoder["path2"],15,2000);
    chassis.follow(decoder["path3"],15,2000);
    chassis.follow(decoder["path4"],15,2000,false);

    //Spin the intake/conveyor motors
    motorSpin(200, 2000);

    // Execute the rest of the autonomous routine
    while (true) {
        lemlib::Pose pose = chassis.getPose();
        if (pose.x >= -30 && pose.y >= -18 ) {
            moveToAngle(23.94);
            intake.move_velocity(200)&&conveyor.move_velocity(200);
            pros::delay(2000);
            intake.move_velocity(0)&&conveyor.move_velocity(0);
        }
        // Delay to save resources
        pros::delay(10);
    }
}

void leftAutonBlue() {
    pros::lcd::clear();
    pros::lcd::set_text(3, "Executing LEFT AUTON - BLUE");
    // Add your autonomous code for left blue here
}

void rightAutonBlue() {
    pros::lcd::clear();
    pros::lcd::set_text(3, "Executing RIGHT AUTON - BLUE");
    // Add your autonomous code for right blue here
}

void moveToAngle(int targetAngle) { 
    int targetValue = targetAngle * 4095 / 265; // Map 0-300 degrees to 0-4095 range
    int currentValue = potentiometer.get_value();

    // Move until the potentiometer value is close to the target
    while (abs(currentValue - targetValue) > 10) { // 10 is a tolerance value
        if (currentValue < targetValue) {
            ladyBrown.move_velocity(50); // Forward
        } else {
            ladyBrown.move_velocity(-50); // Reverse
        }
        // Update the current value
        currentValue = potentiometer.get_value();
    }

    // Stop the motor once the target is reached
    ladyBrown.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
}

// Function to spin the both intake and conveyor motor
void motorSpin(int speed, int duration) {
    conveyor.move_velocity(speed) && intake.move_velocity(speed);
    pros::delay(duration);
    conveyor.move_velocity(0) && intake.move_velocity(0);
}



// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              TRACK_WIDTH, // 10 inch track width
                              lemlib::Omniwheel::OLD_325, // using old 3.25" omnis
                              DRIVETRAIN_RPM, // drivetrain rpm is 360
                              2 // horizontal drift is 2. If we had traction wheels, it would have been 8
);

// lateral motion controller
lemlib::ControllerSettings linearController(10, // proportional gain (kP)
                                            0, // integral gain (kI)
                                            3, // derivative gain (kD)
                                            3, // anti windup
                                            1, // small error range, in inches
                                            100, // small error range timeout, in milliseconds
                                            3, // large error range, in inches
                                            500, // large error range timeout, in milliseconds
                                            20 // maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angularController(2, // proportional gain (kP)
                                             0, // integral gain (kI)
                                             10, // derivative gain (kD)
                                             3, // anti windup
                                             1, // small error range, in degrees
                                             100, // small error range timeout, in milliseconds
                                             3, // large error range, in degrees
                                             500, // large error range timeout, in milliseconds
                                             0 // maximum acceleration (slew)
);

// sensors for odometry
lemlib::OdomSensors sensors(nullptr, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            nullptr, // horizontal tracking wheel
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttleCurve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steerCurve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.019 // expo curve gain
);

// create the chassis
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);


void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
    while (imu.is_calibrating()) {
        pros::delay(10);
    }
    //Print message to brain screen
    pros::lcd::set_text(2, "Inertial sensor calibrated!");

    //Record the lastest pressed event
    pros::screen::touch_callback(autonSelection, TOUCH_PRESSED);
}

void pre_auton() {
    // initialize the inertial sensor
    imu.reset();
}

/**
 * Runs while the robot is disabled
 */
void disabled() {}

/**
 * runs after initialize if the robot is connected to field control
 */
void competition_initialize() {}
// this needs to be put outside a function
ASSET(test_txt); // '.' replaced with "_" to make c++ happy
lemlib_tarball::Decoder decoder(test_txt);
/**
 * Runs during auto
 *
 * This is an example autonomous routine which demonstrates a lot of the features LemLib has to offer
 */
void autonomous() {
    //Stop Lady Brown at the starting point
    while (potentiometer.get_value() > 700) {
        ladyBrown.move_velocity(0);
    }
    //Set the brake mode for Lady Brown
    ladyBrown.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

    // Select autonomous routine
    switch (autonomousSelection) {
        case 1:
            leftAutonRed();
            break;
        case 2:
            rightAutonRed();
            break;
        case 3:
            leftAutonBlue();
            break;
        case 4:
            rightAutonBlue();
            break;
        default:
            pros::lcd::set_text(2, "No autonomous selected!");
            break;
    }
}

/**
 * Runs in driver control
 */
void opcontrol() {

    //Stop Lady Brown at the starting point
    ladyBrown.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

    int frame_counter = 0;

    bool retracted = false;
    SORTING_STATE sorting_state = SORTING_STATE::NOT_DETECTED;
    SPIN_STATE conveyor_state = SPIN_STATE::STOP;

    auto RED_SIG = pros::Vision::signature_from_utility(
    RED_SIG_ID, 3797, 11657, 7727, -2281, -1609, -1945, 1.5, 0);
    auto BLUE_SIG = pros::Vision::signature_from_utility(
    BLUE_SIG_ID, -4477, -3539, -4008, 51, 4449, 2250, 2.6, 0);

    vision_sensor.set_signature(RED_SIG_ID, &RED_SIG);
    vision_sensor.set_signature(BLUE_SIG_ID, &BLUE_SIG);

    DONUT_COLOR scoring_color = DONUT_COLOR::BLUE;
    pros::vision_object_s_t visible_donut;
    double conveyor_stop_target = 0;

    conveyor.tare_position();

        //Controller (tank drive)
    while (true) {
        // Get joystick positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightY = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
        // move the chassis with curvature drive
        chassis.tank(leftY, rightY);

        // lady brown control
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            ladyBrown.move_velocity(2 * LADYBROWN_SPEED_PERCENT);
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            ladyBrown.move_velocity(-2 * LADYBROWN_SPEED_PERCENT);
        } else {
            ladyBrown.move_velocity(0);
        }

        // Intake and conveyor control
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            intake.move_velocity(2 * CONVEYOR_SPEED_PERCENT);
            conveyor.move_velocity(2 * CONVEYOR_SPEED_PERCENT);

            
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            intake.move_velocity(-2 * CONVEYOR_SPEED_PERCENT);
            conveyor.move_velocity(-2 * CONVEYOR_SPEED_PERCENT);
        } else {
            conveyor.move_velocity(0);
            intake.move_velocity(0);
        }

        // Set lady brown to a specific position (intake)
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_X)) {
            while (potentiometer.get_value() < POTENTIOMETER_POSITION) {
                ladyBrown.move_velocity(100);
            }
            if (potentiometer.get_value() > POTENTIOMETER_POSITION) {
                ladyBrown.move_velocity(0);
            }
        } else {
           ladyBrown.move_velocity(0);
        }

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
            if (retracted) {
                piston.set_value(0);
                retracted = false;
                pros::delay(200);
            } else {
                piston.set_value(1);
                retracted = true;
                pros::delay(200);
            }
        }

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
            if (scoring_color == DONUT_COLOR::RED) {
                scoring_color = DONUT_COLOR::BLUE;
                pros::delay(200);
            } else {
                scoring_color = DONUT_COLOR::RED;
                pros::delay(200);
            }
        }

        if (conveyor_state == SPIN_STATE::FORWARD) {
            switch (sorting_state) {
                case SORTING_STATE::NOT_DETECTED:
                visible_donut = vision_sensor.get_by_sig(
                    0, scoring_color == DONUT_COLOR::RED ? BLUE_SIG_ID : RED_SIG_ID);

                if (visible_donut.height >= MIN_SCREEN_COVERAGE) {
                    sorting_state = SORTING_STATE::STANDBY;
                    double conveyor_position = conveyor.get_position();
                    int conveyor_revs =
                        int(conveyor_position / CONVEYOR_REVOLUTION_DEGREES);
                    double conveyor_offset =
                        conveyor_position - conveyor_revs * CONVEYOR_REVOLUTION_DEGREES;
                    double hook_offset = conveyor_offset + DISTANCE_VISION_TO_TOP;
                    double closest_offset = std::abs(hook_offset);

                    int detected_hook = 1;

                    if (std::abs(hook_offset + CONVEYOR_HOOK_2_OFFSET_DEGREES) >
                        closest_offset) {
                    detected_hook = 2;
                    }

                    conveyor_stop_target =
                        (conveyor_revs + 1) * CONVEYOR_REVOLUTION_DEGREES;

                    if (detected_hook == 2) {
                    conveyor_stop_target += CONVEYOR_HOOK_2_OFFSET_DEGREES;
                    }
                }
                break;

                case SORTING_STATE::STANDBY:
                if (conveyor.get_position() >= conveyor_stop_target) {
                    conveyor.move_velocity(0);
                    sorting_state = SORTING_STATE::STOP;
                    pros::delay(CONVEYOR_HALT);
                }
                break;

                default:
                conveyor.move_velocity(2 * CONVEYOR_SPEED_PERCENT);
            }
            } else {
            sorting_state = SORTING_STATE::NOT_DETECTED;
            conveyor_stop_target = 0;
            }

        if (!(frame_counter % 10)) {
            controller.print(0, 0, "Score: %s",
                        scoring_color == DONUT_COLOR::RED ? "RED " : "BLUE");
        }

            frame_counter++;        

        // delay to save resources (he ran 20ms why not)
        pros::delay(20);
    }
}