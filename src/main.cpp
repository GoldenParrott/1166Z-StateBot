#include "init.h"

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	
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

	// Front, Middle, Rear
	leftDrivetrain.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	rightDrivetrain.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	intake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	arm.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	bool clampOn = true;

	// Drving variables
	int drvfb;
	int drvlr;
	int drvtrdz = 10;

	while (true) {
		
	//Drivetrain Control 
		drvfb = master.get_analog(ANALOG_LEFT_Y);
		drvlr = master.get_analog(ANALOG_RIGHT_X);

		if ((abs(drvfb) > drvtrdz) || (abs(drvlr) > drvtrdz)) {
      		// ^^ Checks to see if either joystick has moved out of the deadzone
			rightDrivetrain.move((drvfb-(drvlr)));
      		leftDrivetrain.move((drvfb+(drvlr)));	
    	} else {
			rightDrivetrain.brake();
      		leftDrivetrain.brake();
    	} 
	
	//Intake Control
	//Arm up Y
	//Arm down B
		if(master.get_digital(DIGITAL_RIGHT))
		{
			intake.move(127);
		}
		else if(master.get_digital(DIGITAL_DOWN))
		{
			intake.move(-127);
		}
		else if(master.get_digital(DIGITAL_R2))
		{
			preRoller.move(127);
		}
		else
		{
			intake.brake();
		}

	//Arm Control
		if(master.get_digital(DIGITAL_Y))
		{
			arm.move(127);
		}
		else if(master.get_digital(DIGITAL_B))
		{
			arm.move(-127);
		}
		else
		{
			arm.brake();
		}			

		//Mogo
		if(Master.get_digital_new_press(DIGITAL_R1)){

			// ↓↓ If the manipulator is open, activate this code
			if (clamp.get_value() == false) {

				// ↓↓ Closes the manipulator to grab an object
				clamp.set_value(true);

			// ↓↓ If the manipulator is closed, activate this code
			} else if (clamp.get_value() == true){

				// ↓↓ Opens the manipulator to grab an object
				clamp.set_value(false);
			}
		}
		
		
		pros::delay(20);
	}
}