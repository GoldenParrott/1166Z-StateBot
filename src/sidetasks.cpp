#include "init.h"

void ArmMacros() {
    while (true) {
		
	
		// scores the arm on a Wall Stake and then retracts to its original position
		if (master.get_digital(DIGITAL_Y)) {
			armMoving = true;
			while (ArmRotational.get_position() > -14000) {
                arm.move(127);
            }
            while (ArmRotational.get_position() < -3000) {
                arm.move(-32);
            }
			arm.brake();
			armMoving = false;
		}
		// puts the arm in a scoring position
		else if (master.get_digital_new_press(DIGITAL_B)) {
			armMoving = true;
			if(ArmRotational.get_position() > -1100){
				while (ArmRotational.get_position() > -2000) {
					arm.move(32);
				}
			}else{
				while (ArmRotational.get_position() < 0) {
					arm.move(-32);
				}
			}
			arm.brake();
			armMoving = false;
        }
	pros::delay(10);
    }
}

void eject() {
	int ejectStartPoint = 0;
	bool ejectOn = false;

	if(autonnumber < 0){
		master.print(0,0,"Scoring Red\n",NULL);
	}else if(autonnumber > 0){
		master.print(0,0,"Scoring Blue\n",NULL);
	}

	while (true) {
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
				if (abs(transport.get_position() - ejectStartPoint) >= 300) {
					ejectOn = false;
					ejectStartPoint = 0;
					transport.brake();
				// case 1b: if case 1a is not true, then continue moving the intake down
				} else {
					intake.move(-128);
				}
			}
			//case 2: eject is not on, but the distance sensor is at the proper distance and the color sensor has found a correct color
			else if ((((colorSense.get_hue() > 180) && (autonnumber < 0)) || // blue
				      ((colorSense.get_hue() < 35)  && (autonnumber > 0))    // red
					 )
					&& (Distance.get() < 50)
					)
			{
				// in this case, the redirect is started and the starting point is stored for later
				pros::delay(10); // the robot waits for the Ring to reach the proper point before starting the eject
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
			preRoller.move(-127);
		} else{
			intake.brake();
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
				if (abs(transport.get_position() - ejectStartPoint) >= 200) {
					ejectOn = false;
					ejectStartPoint = 0;
					transport.move(-128);
				// case 1b: if case 1a is not true, then continue moving the intake down
				} else {
					transport.move(128);
				}
			}
			// case 2: eject is not on, but the distance sensor is at the proper distance and the color sensor has found the right color
			else if ((((colorSense.get_hue() > 180) && (autonnumber < 0)) || // blue
				      ((colorSense.get_hue() < 35)  && (autonnumber > 0)) // red
					 )
					&& (Distance.get() < 50)
					)
			{
				// in this case, the redirect is started and the starting point is stored for later
				pros::delay(65); // the robot waits for the Ring to reach the proper point before starting the eject
				transport.move(128);
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
