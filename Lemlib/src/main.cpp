#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#define POTENTIOMETER_PORT 'A'

// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// motor groups
pros::Motor ladyBrown(9, pros::MotorGearset::green); // left front motor
pros::Motor intake(-6, pros::MotorGearset::green); // intake motor
pros::Motor conveyor(-7, pros::MotorGearset::green); // conveyor motor
pros::MotorGroup leftMotors({-1, -2}, pros::MotorGearset::green); // left motor group
pros::MotorGroup rightMotors({3, 4}, pros::MotorGearset::green); // right motor group
pros::ADIAnalogIn potentiometer(POTENTIOMETER_PORT);// line sensor on port 1

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

// Inertial Sensor on port 13
pros::Imu imu(13);

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              14.05, // 10 inch track width
                              lemlib::Omniwheel::OLD_325, // using old 3.25" omnis
                              340, // drivetrain rpm is 360
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

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors

    // the default rate is 50. however, if you need to change the rate, you
    // can do the following.
    // lemlib::bufferedStdout().setRate(...);
    // If you use bluetooth or a wired connection, you will want to have a rate of 10ms

    // for more information on how the formatting for the loggers
    // works, refer to the fmtlib docs

    // thread to for brain screen and position logging
    pros::Task screenTask([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // log position telemetry
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            // delay to save resources
            pros::delay(50);
        }
    });
}

/**
 * Runs while the robot is disabled
 */
void disabled() {}

/**
 * runs after initialize if the robot is connected to field control
 */
void competition_initialize() {}

// get a path used for pure pursuit
// this needs to be put outside a function
ASSET(test_txt); // '.' replaced with "_" to make c++ happy

/**
 * Runs during auto
 *
 * This is an example autonomous routine which demonstrates a lot of the features LemLib has to offer
 */
void autonomous() {
    //Set the stopping point for Lady Brown
    while (potentiometer.get_value() > 700) {
        ladyBrown.move_velocity(0);
    }
    //Set the brake mode for Lady Brown
    ladyBrown.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

    //Setting pose manually
    chassis.setPose({-60, -18, 332});

    //Alliance wall stake
    conveyor.move_velocity(200);
    pros::delay(2000);
    conveyor.move_velocity(0);

    chassis.follow(test_txt, 15, 4000);
    // Follow set path
    while (true) {
        lemlib::Pose pose = chassis.getPose();
        if (pose.x >= -30 && pose.y >= -18 ) {
            moveToAngle(23.94);
            intake.move_velocity(200)&&conveyor.move_velocity(200);
            pros::delay(2000);
            intake.move_velocity(0)&&conveyor.move_velocity(0);
        }
        // Update motors
        // Delay to save resources
        pros::delay(10);
    }
}

/**
 * Runs in driver control
 */
void opcontrol() {
    ladyBrown.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

        //Controller (arcade drive)
    while (true) {
        // get joystick positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        // move the chassis with curvature drive
        chassis.arcade(leftY, rightX);
    }

    while (true) {
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            ladyBrown.move_velocity(100);
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            ladyBrown.move_velocity(-100);
        } else {
            ladyBrown.move_velocity(0);
        }
    }

    while (true) {
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            intake.move_velocity(200);
            conveyor.move_velocity(200);
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            intake.move_velocity(-200);
            conveyor.move_velocity(-200);
        } else {
            conveyor.move_velocity(0);
            intake.move_velocity(0);
        }
    }

    while (true) {
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
            while (potentiometer.get_value() < 2060) {
                ladyBrown.move_velocity(100);
            }
            if (potentiometer.get_value() > 2060) {
                ladyBrown.move_velocity(0);
            }
        } else {
           ladyBrown.move_velocity(0);
        }
    }

    while (true) {
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            ladyBrown.move_velocity(100);
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            ladyBrown.move_velocity(-100);
        } else {
            ladyBrown.move_velocity(0);
        }
    }
        // delay to save resources
        pros::delay(10);
}