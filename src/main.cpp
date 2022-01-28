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
void autonomous() {
	std::shared_ptr<ChassisController> chassis =
		ChassisControllerBuilder()
			.withMotors({1, 2}, {-9, -8})
			// Green gearset, 4 in wheel diam, 11.5 in wheel track
			.withDimensions(AbstractMotor::gearset::green, {{4_in, 7_in}, imev5GreenTPR})
			//.withGains(
			//	{0.001, 0, 0.0001},
			//	{0.001, 0, 0.0001},
			//	{0.001, 0, 0.0001})
			.build();

	std::shared_ptr<AsyncMotionProfileController> profileController =
	  AsyncMotionProfileControllerBuilder()
	    .withLimits({
	      1.0, // Maximum linear velocity of the Chassis in m/s
	      2.0, // Maximum linear acceleration of the Chassis in m/s/s
	      10.0 // Maximum linear jerk of the Chassis in m/s/s/s
	    })
	    .withOutput(chassis)
	    .buildMotionProfileController();

		std::shared_ptr<AsyncPositionController<double, double>> lift =
			AsyncPosControllerBuilder()
				.withMotor(3)
				//.withGains({0.001, 0, 0.0001})
				.build();

		pros::ADIDigitalOut claw('A');
		claw.set_value(false);

		// Gets first goal
		profileController->generatePath({
		  {0_ft, 0_ft, 0_deg},  // Starting position
		  {48_in, 0_ft, 0_deg}}, // Position of goal
		  "Goal 1" // Profile name
		);

		// Set max veolicty and tare positions of the lift
		lift->setMaxVelocity(100);
		lift->tarePosition();

		// Raise the lift to not drag on floor
		lift->setTarget(350);

		profileController->setTarget("Goal 1");

		pros::delay(2000);

		claw.set_value(true);

		chassis->moveDistance(-32_in);

		claw.set_value(false);

		chassis->moveDistance(-3_in);

		chassis->turnAngle(95_deg);

		chassis->moveDistance(20_in);
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
	pros::Controller master(pros::E_CONTROLLER_MASTER);

	// Setup motors. All drive motors are standard green cartridges, fork motors use red torque cartridges
	pros::Motor front_left_drive(1);
	pros::Motor back_left_drive(2);
	pros::Motor front_lift(3);
	pros::Motor back_forklift(4);
	pros::Motor intake(5);
	//pros::Motor claw(12, true);
	pros::Motor back_right_drive(8, true);
	pros::Motor front_right_drive(9, true);
	pros::ADIDigitalOut claw('A');
	back_forklift.set_gearing(pros::E_MOTOR_GEARSET_36);
	front_lift.set_gearing(pros::E_MOTOR_GEARSET_36);
	//claw.set_gearing(pros::E_MOTOR_GEARSET_36);

	// Set forklift breaking mode to hold so that the weight of the goals doesn't make the forks drop
	back_forklift.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	front_lift.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
	//claw.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

	int intake_velocity = 0;
	claw.set_value(false);

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
			back_forklift.move(125);
		}
		else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2))
		{
			back_forklift.move(-125);
		}
		else if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B))
		{
			back_forklift.move_absolute(1520, 125); // Move to position for intake
		}
		else
		{
			back_forklift.move(0);
		}

		// Move claw with A and Y
		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_Y))
		{
			claw.set_value(true);
		}
		else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_A))
		{
			claw.set_value(false);
		}

		// Toggle ring intake with X
		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X))
		{
			if(intake_velocity == 0)
			{
				intake_velocity = 200;
			}
			else
			{
				intake_velocity = 0;
			}
		}

		// Move intake according to velocity set above
		intake.move(intake_velocity);

		pros::delay(2);
	}
}
