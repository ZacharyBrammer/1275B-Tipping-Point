#include "main.h"

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");

	pros::lcd::register_btn1_cb(on_center_button);
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
void autonomous() {}

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
	pros::Controller master(pros::E_CONTROLLER_MASTER);

	// Setup motors. All drive motors are standard green cartridges, fork motors use red torque cartridges
	pros::Motor front_left_drive(1);
	pros::Motor back_left_drive(2);
	pros::Motor front_lift(3, true);
	pros::Motor back_forklift(4);
	pros::Motor claw(7, true);
	pros::Motor back_right_drive(8, true);
	pros::Motor front_right_drive(9, true);
	back_forklift.set_gearing(pros::E_MOTOR_GEARSET_36);
	front_lift.set_gearing(pros::E_MOTOR_GEARSET_36);
	claw.set_gearing(pros::E_MOTOR_GEARSET_36);

	// Set forklift breaking mode to hold so that the weight of the goals doesn't make the forks drop
	back_forklift.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
	front_lift.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
	claw.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

	while (true) {
		// Power is how fast to drive, turn is what angle to drive at
		int power = master.get_analog(ANALOG_LEFT_Y);
    int turn = master.get_analog(ANALOG_LEFT_X);

		int left = power + turn;
		int right = power - turn;

		// Move motors according to joystick input
		front_left_drive.move(left);
		back_left_drive.move(left);
		front_right_drive.move(right);
		back_right_drive.move(right);

		// Move front lift with the right bumpers
		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
		{
			front_lift.move(100);
		}
		else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
		{
			front_lift.move(-100);
		}
		else
		{
			front_lift.move(0);
		}

		// Move rear forklift with the left bumpers
		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1))
		{
			back_forklift.move(75);
		}
		else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2))
		{
			back_forklift.move(-75);
		}
		else
		{
			back_forklift.move(0);
		}

		// Move claw with A and Y
		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_Y))
		{
			claw.move(50);
		}
		else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_A))
		{
			claw.move(-50);
		}
		else
		{
			claw.move(0);
		}

		pros::delay(2);
	}
}
