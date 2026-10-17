#include "main.h"

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	chassis.calibrate();

	arm.set_brake_mode(pros::v5::MotorBrake::hold);
	arm.tare_position();
	
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
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
    // set position to x:0, y:0, heading:0
    //chassis.setPose(0, 0, 0);
    // turn to face heading 90 with a very long timeout
	//chassis.turnToHeading(180, 1000);
	//chassis.waitUntilDone();
	//chassis.turnToHeading(0, 1000);
    //chassis.moveToPoint(0, 20, 100000);
	//auton();
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
	// R2:Intake
	// L2:outtake
	// R1:raise cascade
	// L1:lower cascade
	// Y: arm up / 90 then 45 
	// right arrow: arm down

	bool macro = true;

	while (true) {

		// drivetrain
		int left = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y); // Y axis of the left joystick
        int right = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y); // Y axis of the right joystick
		chassis.tank(left, right); // tank drive

		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
			intake.move(127);
		} else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
			intake.move(-127);
		} else {
			intake.move(0);
		}

		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
			cascade.move(127);
		} else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
			cascade.move(-127);
		} else {
			cascade.move(0);
		}
		
		if (macro) {
			if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
				if (arm.get_position() <= 576) {
					arm.move_absolute(576, 100);
				} else if (arm.get_position() > 576) {
					arm.move_absolute(864, 100);
				}
			}
			if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
				if (arm.get_position() <= 576) {
					arm.move_absolute(0, 100);
				} else if (arm.get_position() > 576) {
					arm.move_absolute(576, 100);
				}
			}
		} else {
			if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y)) {
				arm.move(75);
			} else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
				arm.move(-50);
			} else {
				arm.brake();
			}
		}

		if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
			macro = !macro;
		}
		

		pros::delay(20); // Run for 20 ms then update
	}
}