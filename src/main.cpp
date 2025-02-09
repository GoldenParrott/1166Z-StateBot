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
void competition_initialize() {


	autonnumber = 1; 
	globalAuton = false;

	autoSelector_task_ptr = new pros::Task(drawBasicSelector);

	while (true) {
		
		if (globalAuton == true) {
			switch (autonnumber) {
				case 1: //Blue Mogo
					initializeRobotOnCoordinate(&Rotational, &Inertial1, &Inertial2, {50, -36}, 251);
					break;
				case 2:
					initializeRobotOnCoordinate(&Rotational, &Inertial1, &Inertial2, {54.5, 13.125}, 208);
					break;
				case -1:
					initializeRobotOnCoordinate(&Rotational, &Inertial1, &Inertial2, {-49.25, -60.325}, 69);
					break;
				case -2: //Red Ring
					initializeRobotOnCoordinate(&Rotational, &Inertial1, &Inertial2, {-55, 12}, 148);
					break;
				case 3:
				case -3: //Test (?)
					initializeRobotOnCoordinate(&Rotational, &Inertial1, &Inertial2, {-48, -48}, 225);
					break;
			}
		} else {
			switch (autonnumber) {
				case 1://Blue Mogo
					initializeRobotOnCoordinate(&Rotational, &Inertial1, &Inertial2, {50.5, -35.5}, 245);
					break;
				case -1:
					initializeRobotOnCoordinate(&Rotational, &Inertial1, &Inertial2, {-50.5, -60.5}, 65);
					break;
				case 2:
					initializeRobotOnCoordinate(&Rotational, &Inertial1, &Inertial2, {0, 0}, 0);
					break;
				case -2:
					initializeRobotOnCoordinate(&Rotational, &Inertial1, &Inertial2, {-54, 12.5}, 232);
					break;
				case -5:
					initializeRobotOnCoordinate(&Rotational, &Inertial1, &Inertial2, {-60.75, 0}, 90);
					break;
			}
		}
		status = pros::screen::touch_status();
		// initializeRobotOnCoordinate(&Rotational, &Inertial1, &Inertial2, {55, 10}, 140);
		pros::delay(10);

	}
	
}

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
	// disables the auto selector
	if (autoSelector_task_ptr != NULL) {
		autoSelector_task_ptr->remove();
	}
/*
	Bozo.clean();
	Logo.clean();
	RMA.clean();
	BMA.clean();
	RRA.clean();
	BRA.clean();
	RME.clean();
	BME.clean();
	RRE.clean();
	BRE.clean();
*/
	// starts the system that fixes the turning tracking wheel's heading
	pros::Task updateRotational = pros::Task(bindTurnTrackingWheelHeading);

	// starts the coordinate updating system
	coordinateUpdater_task_ptr = new pros::Task(updateCoordinateLoop);

	// autonomous setup
	colorSense.set_led_pwm(100);

	intake.tare_position();

	arm.set_encoder_units(MOTOR_ENCODER_DEGREES);
	arm.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	arm.tare_position();

	Kalman1.startFilter();
	Kalman2.startFilter();

	leftDrivetrain.set_brake_mode(pros::MotorBrake::hold);
	rightDrivetrain.set_brake_mode(pros::MotorBrake::hold);

	blueGoalside();
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

	master.rumble("-.-");
	//std::cout << logfile.readFile();

	// Front, Middle, Rear
	leftDrivetrain.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	rightDrivetrain.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	intake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	arm.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	bool clampOn = true;

	// Driving variables
	int drvfb;
	int drvlr;
	int drvtrdz = 10;
	
	coordinateUpdater_task_ptr = new pros::Task(updateCoordinateLoop);
	eject_task_ptr = new pros::Task(eject);
	macros_task_ptr = new pros::Task(ArmMacros);

	while (true) {
	
	//Drivetrain Control 
		drvfb = master.get_analog(ANALOG_LEFT_Y);
		drvlr = master.get_analog(ANALOG_RIGHT_X);

		if ((abs(drvfb) > drvtrdz) || (abs(drvlr) > drvtrdz)) {
      		// ^^ Checks to see if either joystick has moved out of the deadzMasterone
			rightDrivetrain.move((drvfb-(drvlr)));
      		leftDrivetrain.move((drvfb+(drvlr)));	
    	} else {
			rightDrivetrain.brake();
      		leftDrivetrain.brake();
    	} 
	
	//Intake Control
	//Arm up Y
	//Arm down B
		
	//INTAKE CODE IS NOW IN THE TASK

		if(master.get_digital(DIGITAL_R2))
		{
			preRoller.move(-127);
		}
		else
		{
			preRoller.brake();
		}

	//Arm Control
		if(master.get_digital(DIGITAL_UP))
		{
			arm.move(127);
		}
		else if(master.get_digital(DIGITAL_LEFT))
		{
			arm.move(-127);
		}
		else if(armMoving == false)
		{
			arm.brake();
		}			

		//Mogo
		if(master.get_digital_new_press(DIGITAL_R1)){

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

		// Yoinker
		if (master.get_digital_new_press(DIGITAL_L1)) {
			if (!yoin.get_value()) {
				yoin.set_value(true);
			} else {
				yoin.set_value(false);
			}
		}

		// Finger
		if (master.get_digital_new_press(DIGITAL_L2)) {
			if (!ker.get_value()) {
				ker.set_value(true);
			} else {
				ker.set_value(false);
			}
		}

		// Input Raiser
		if (master.get_digital_new_press(DIGITAL_A)) {
			if (!inPutston.get_value()) {
				inPutston.set_value(true);
			} else {
				inPutston.set_value(false);
			}
		}
		
		pros::delay(20);
	}
}