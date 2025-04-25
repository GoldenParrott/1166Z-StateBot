#include "init.h"

void ArmMacros() {
    while (true) {
		/*
		Scoring 65 - 550
		Passive 0 - 45
		*/
		// Puts arm in scroing position
		if (master.get_digital(DIGITAL_UP)) {
			armMoving = true;
			double startTime = pros::millis();
			
            while (((arm.get_position() > 45)||(arm.get_position() < 25)) && ((pros::millis() - startTime) < 2000)) {
                arm.move_absolute(55,96);
            }
			arm.brake();
			armMoving = false;
		}
		// Puts arm in passive position
		else if (master.get_digital_new_press(DIGITAL_LEFT)) {
			armMoving = true;
			double startTime = pros::millis();
			
            while ((arm.get_position() > 5) && ((pros::millis() - startTime) < 2000)) {
                arm.move_absolute(0,96);
            }
			arm.brake();
			armMoving = false;
        }
	pros::delay(10);
    }
}

void eject() {
	colorSense.set_led_pwm(100);
	int ejectStartPoint = 0;
	bool ejectOn = false;

	if(autonnumber < 0){
		master.print(0,0,"Scoring Red\n",NULL);
	}else if(autonnumber > 0){
		master.print(0,0,"Scoring Blue\n",NULL);
	}

	while (true) {
		// master.print(0,0,"%d\n",colorSense.get_proximity());
		// distance sensor (eject)
		// Changes the eject to be for the opposite color when the button is pressed
		if (master.get_digital_new_press(DIGITAL_X)){
			autonnumber *= -1;
			if(autonnumber < 0){
				master.print(0,0,"Scoring Red\n",NULL);
			}else if(autonnumber > 0){
				master.print(0,0,"Scoring Blue\n",NULL);
			}
		}
		// handles the cases for if the eject is in the enabled state
		if(master.get_digital(DIGITAL_RIGHT)){
			// case 1: redirect is currently on
			if (ejectOn == true) {
				// case 1a: if the difference between the starting point and the current point
				// 			is greater than 700 (meaning that it has gone all the way),
				//			turn off the 
				if (abs(transport.get_position() - ejectStartPoint) >= 1000) {
					ejectOn = false;
					ejectStartPoint = 0;
					transport.brake();
				// case 1b: if case 1a is not true, then continue moving the intake down
				} else {
					transport.move(-128);
				}
			}
			//case 2: eject is not on, but the distance sensor is at the proper distance and the color sensor has found a correct color
			else if ((((colorSense.get_hue() > 100) && (autonnumber < 0)) || // blue
				      ((colorSense.get_hue() < 30)  && (autonnumber > 0)))   // red
					&& (Distance.get() < 100)
					)
			{
				// in this case, the redirect is started and the starting point is stored for later
				if (autonnumber < 0) {
					pros::delay(90);
				} else if (autonnumber > 0) {
					pros::delay(10);
				}
				transport.move(-128);
				ejectOn = true;
				ejectStartPoint = transport.get_position();
			}
			// case 3: if the redirect is not on and should not be on, 
			//		   then L2 moves the robot forward as normal
			else {
				intake.move(128);
			}
		} else if(master.get_digital(DIGITAL_DOWN)){
			intake.move(-128);
		} else if(master.get_digital(DIGITAL_R2)){
			preRoller.move(128);
		} else{
			intake.brake();
			ejectOn = false;
		}
		pros::delay(10);
	}
}

void autoEject() {
	bool ejectOn = false;
	int ejectStartPoint = 0;

	////////////////////////////////////////////
	//										  //
	//	ERROR: Code is SOMETIMES not reaching //
	//	inside the color sensor statement 	  //
	//	on lines 128-132. Also changed  	  //
	//	red to <25 as it detects better    	  //
	//									      //
	//////////////////////////////////////////// 

	while (true) {
    // distance sensor (eject)
		// handles the cases for if B is being held down
			// case 1: redirect is currently on
			if (ejectOn == true) {
				// case 1a: if the difference between the starting point and the current point
				// 			is greater than 700 (meaning that it has gone all the way),
				//			turn off the 
				if (abs(transport.get_position() - ejectStartPoint) >= 1000) {
					ejectOn = false;
					ejectStartPoint = 0;
					transport.move(128);
				// case 1b: if case 1a is not true, then continue moving the intake down
				} else {
					transport.move(-128);
				}
			}
			//case 2: eject is not on, but the distance sensor is at the proper distance and the color sensor has found a correct color
			else if ((((colorSense.get_hue() > 100) && (autonnumber < 0)) || // blue
				      ((colorSense.get_hue() < 30)  && (autonnumber > 0)))   // red
					&& (Distance.get() < 100)
					&& (clamp.get_value())
					)
			{
				// in this case, the redirect is started and the starting point is stored for later
				if (autonnumber < 0) {
					pros::delay(90);
				} else if (autonnumber > 0) {
					pros::delay(70);
				}
				transport.move(-128);
				ejectOn = true;
				ejectStartPoint = transport.get_position();
			}
	}
}

void coords() {
	
	while(1){
		pros::screen::print(TEXT_LARGE, 0, "x = %f",universalCurrentLocation.x);
		pros::screen::print(TEXT_LARGE, 2, "y = %f",universalCurrentLocation.y);
		pros::screen::print(TEXT_LARGE, 4, "θ = %f",getAggregatedHeading(Kalman1, Kalman2));
		pros::delay(100);
	}
	
}


void CutoffPID(Point goalPoint, bool reverse, double maxAllowableTime) {
	endCoords = goalPoint;
	endReverse = reverse;
	pros::Task movement = pros::Task(PIDMoverBasic);
	
	
}

void CutoffTurnPID(Point goalPoint, bool reverse, double maxAllowableTime, int direction) {
	// the turn movement
	auto movement = [goalPoint, direction] () {PIDTurner(findHeadingOfLine({universalCurrentLocation.x, universalCurrentLocation.y}, goalPoint), direction);};
	// finds the heading's inverse when it is positive
	auto determineInverse = [] (double angle) -> double {
		return angle > 180 ? angle - 180 : angle + 180;
	};
	// the reversed form of the turn movement
	auto revMovement = [goalPoint, direction, determineInverse] () {PIDTurner(determineInverse(findHeadingOfLine({universalCurrentLocation.x, universalCurrentLocation.y}, goalPoint)), direction);};
	pros::Task* movement_task = NULL;
	if (!reverse) {
		movement_task = new pros::Task(movement);
	} else {
		movement_task = new pros::Task(revMovement);
	}
	pros::delay(maxAllowableTime);
	movement_task->remove();
	drivetrain.brake();
}

void CutoffTurnHeadingPID(int goalHeading, bool reverse, double maxAllowableTime, int direction) {
	// the turn movement
	auto movement = [goalHeading, direction] () {PIDTurner(goalHeading, direction);};
	// finds the heading's inverse when it is positive
	auto determineInverse = [] (double angle) -> double {
		return angle > 180 ? angle - 180 : angle + 180;
	};
	// the reversed form of the turn movement
	auto revMovement = [goalHeading, direction, determineInverse] () {PIDTurner(determineInverse(goalHeading), direction);};
	pros::Task* movement_task = NULL;
	if (!reverse) {
		movement_task = new pros::Task(movement);
	} else {
		movement_task = new pros::Task(revMovement);
	}
	pros::delay(maxAllowableTime);
	movement_task->remove();
	drivetrain.brake();
}