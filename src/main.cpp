#include "main.h"
#include "lemlib/api.hpp"
#include "drivercontrol.hpp"
#include "devices.hpp"
#include "autons.hpp"
#include "gif-pros/gifclass.hpp"
#include "pros/misc.h"
#include "screen.hpp"
#include "display/lvgl.h" 

// Enter your autons here!
AutonFunction autonFunctions[] = {

};

// this is needed for LVGL displaying! Do not touch!
size_t autonCount = sizeof(autonFunctions) / sizeof(autonFunctions[0]);

// these are the gifs that will display randomly for the calibration screen
std::string calibrationGifs[] = {
                "/usd/jq-griddy.gif", 
                "/usd/dog-angry.gif", 
                "/usd/did-you-pray-today.gif", 
                "/usd/beast-boy.gif", 
            };

// these are the gifs that will display randomly during the match
std::string matchGifs[] = {
                "/usd/getreal.gif", 
                "/usd/minecraft-sky-parkour.gif", 
            };

void initialize() {
    ez::ez_template_print();

    pros::delay(500); // Stop the user from doing anything while legacy ports configure.
 
	// screen init
    calibrationScreenInit();

    // calibrate the lem lib x ez temp chassis
    // not doing lem anymore because bad
    calibrateBothChassis(/*init lem lib chassis: */ false);

    // auton selector
    autonSelectorScreenInit(
        // your auton funcitons
        autonFunctions, 
        // auton count needed for LVGL displaying
        autonCount, 
        // customizable color scheme, play around with it!
        LV_COLOR_MAKE(0xFF, 0x00, 0x00)
    );

    // pid and curve inits
    ezTempChassisInits();
    intake.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    intake2.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {} 

/**
 * Runs after initialize(), and before autonomous when connected to the Field

 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 */
void autonomous() {
    EzTempChassis.pid_targets_reset(); // Resets PID targets to 0
    EzTempChassis.drive_imu_reset(); // Reset gyro position to 0
    EzTempChassis.drive_sensor_reset(); // Reset drive sensors to 0
    EzTempChassis.drive_brake_set(pros::E_MOTOR_BRAKE_HOLD); // Set motors to hold.

    runSelectedAuton(autonFunctions, autonCount);
}

/**
 * Runs the operator control code. This function will be started in its own task
 */
bool wasR1PressedLast = false;
bool wasR2PressedLast = false;
void opcontrol() {
    EzTempChassis.drive_brake_set(pros::E_MOTOR_BRAKE_COAST);

    // task to make sure all motors are plugged in and check the temperature of the drivetrain
    pros::Task motorCheckDT(checkDTMotorsAndReturnTemperature);
    pros::Task motorCheckOther(checkOtherMotorsAndReturnTemperature);


    // Driver Skills Code (COMMENT OUT when not doing skills)
    // EzTempChassis.set_drive_brake(pros::E_MOTOR_BRAKE_HOLD);
    // pros::Task skillsTask([]() {
    //     skills();
    // });

    // leftFront.move_velocity(0);
    // leftBack.move_velocity(0);
    // leftTop.move_velocity(0);
    // rightFront.move_velocity(0);
    // rightBack.move_velocity(0);
    // rightTop.move_velocity(0);
    // Wait for any button press to interrupt skills
    // bool interrupted = false;
    // while (!interrupted) {
    //     for (int button = pros::E_CONTROLLER_DIGITAL_L1; button <= pros::E_CONTROLLER_DIGITAL_Y; button++) {
    //         if (master.get_digital_new_press(static_cast<pros::controller_digital_e_t>(button))) {
    //             interrupted = true;
    //             skillsTask.remove(); // Stop the skills task
                
    //             break;
    //         }
    //     }
    //     pros::delay(20); // Small delay to prevent hogging CPU
    // }
    // END OF SKILLS AUTO CODE
    // ZACH YOU NEED TO LOCK IN ON DRiVING AND CODING AND LOCK ThE FUCK IN WAKEY WAKEY

	while (true) {
        // tank_drive( 
            
        //     // OPTIONAL:
        //     // adjust curve (DEFAULT IS 7.0):
        //     // <curve_value_here>
        // );

        EzTempChassis.opcontrol_tank();
        
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_B)) { 
            motorCheckDT.suspend(); 
            motorCheckOther.suspend(); 
        }
        run_intake();
        hold_clamp();

		pros::delay(ez::util::DELAY_TIME);
	}
}

int matchGifsSize = sizeof(matchGifs) / sizeof(matchGifs[0]);
int calibrationGifsSize = sizeof(calibrationGifs) / sizeof(calibrationGifs[0]);
