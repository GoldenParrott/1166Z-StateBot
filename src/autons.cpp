#include "init.h"
#include "profiling.h"

void BlueAWP() {
    VelocityController follower = VelocityController();

    // spline setup
    // CubicHermiteSpline goalSpline = CubicHermiteSpline({-54.75, 13.25}, {28.7, 45}, {35.14, 29.54}, {11.6, 17.9});
    CubicHermiteSpline goalSpline = CubicHermiteSpline({54.75, 13.25}, {-10, 36}, {20, 25}, {-10, 36});
    CubicHermiteSpline innerRingSpline = CubicHermiteSpline({20, 25}, {25, 51}, {23, 42}, {29, 83});
    CubicHermiteSpline outerRingSpline = CubicHermiteSpline({23, 42}, {34, 77}, {50, 39}, {49, 20});
    CubicHermiteSpline crossSpline = CubicHermiteSpline({49.5, 39}, {50, 20}, {50, 12}, {43, 41.5});
    CubicHermiteSpline southernRingSpline = CubicHermiteSpline({50, -12}, {38, -125.5}, {22, -56}, {13.5, -83});
    CubicHermiteSpline ladderSpline = CubicHermiteSpline({26, -56}, {24.5, -22.5}, {32.5, -23}, {32, 5});
    CubicHermiteSpline ladder2Spline = CubicHermiteSpline({32.5, -23}, {32, 5}, {11, -15}, {-2, -13});

    // profile setup
    MotionProfile* goalProfile = new MotionProfile(&goalSpline, RPMtoIPS(600));
    MotionProfile* innerRingProfile = new MotionProfile(&innerRingSpline, RPMtoIPS(600), 
        {
            {{0, 0.1}, {0.05, 1}}, 
            {{0.05, 1}, {1, 1}}
        }
    );
    MotionProfile* outerRingProfile = new MotionProfile(&outerRingSpline, RPMtoIPS(600),
        {
            {{0, 0.1}, {0.1, 1}}, 
            {{0.1, 1}, {1, 1}}
        }
    );
    MotionProfile* crossProfile = new MotionProfile(&crossSpline, RPMtoIPS(600),
        {
            {{0, 1}, {0.9, 1}},
            {{0.5, 1}, {0.6, 0.4}},
            {{0.9, 0.4}, {1, 0.4}}
        }
    );
    MotionProfile* southernRingProfile = new MotionProfile(&southernRingSpline, RPMtoIPS(600), {
        {
            {{0, 0.4}, {0.4, 0.4}},
            {{0.4, 0.4}, {0.5, 1}},
            {{0.5, 1}, {0.9, 1}},
            {{0.9, 1}, {1, 0}}
        }
    });
    MotionProfile* ladderProfile = new MotionProfile(&ladderSpline, RPMtoIPS(600));
    MotionProfile* ladder2Profile = new MotionProfile(&ladder2Spline, RPMtoIPS(600));

    // scores on Alliance Stake
    drivetrain.move_relative(200, 150);
    arm.move(70);
    waitUntil(arm.get_position() > 450);
    pros::delay(250);
    drivetrain.move_relative(-220, 150);
    pros::delay(200);
    arm.move(-70);
    waitUntil(arm.get_position() < 230);
    arm.brake();

    // moves to goal and grabs it
    PIDTurner(107, 1);
    follower.addAction([](){clamp.set_value(true);}, 0.98);
    follower.startProfile(goalProfile, true);
    follower.clearActions();

    // moves to inner Ring stack and takes the correct Ring
    PIDTurner(findHeadingOfLine({universalCurrentLocation.x, universalCurrentLocation.y}, {24, 48}), 1);
    intake.move(128);
    transport.move(96);
    follower.startProfile(innerRingProfile);

    // moves to middle Ring stack and takes those Rings
    follower.startProfile(outerRingProfile);
    transport.move(128);

    // moves all the way to the other side of the field, grabbing the middle Ring and dropping the first MoGo
    follower.addAction([](){inPutston.set_value(true);}, 0.6);
    follower.startProfile(crossProfile);
    follower.clearActions();

    // moves to the southern Ring stack (on the Goal side)
    follower.addAction([](){inPutston.set_value(false);}, 0.05);
    follower.addAction([](){clamp.set_value(false); transport.brake();}, 0.6);
    follower.startProfile(southernRingProfile);
    follower.clearActions();

    // grabs the third MoGo and touches the Ladder with it
    follower.addAction([](){clamp.set_value(true);}, 0.98);
    follower.startProfile(ladderProfile, true);
    follower.clearActions();
    pros::delay(2000);
    follower.startProfile(ladder2Profile, true);
    arm.move_relative(-180, 200);
}

void RedAWP() {
    VelocityController follower = VelocityController();

    // spline setup
    // CubicHermiteSpline goalSpline = CubicHermiteSpline({-54.75, 13.25}, {28.7, 45}, {35.14, 29.54}, {11.6, 17.9});
    CubicHermiteSpline goalSpline = CubicHermiteSpline({-54.75, 13.25}, {10, 36}, {-20, 25}, {10, 36});
    CubicHermiteSpline innerRingSpline = CubicHermiteSpline({-20, 25}, {-25, 51}, {-23, 42}, {-29, 83});
    CubicHermiteSpline outerRingSpline = CubicHermiteSpline({-23, 42}, {-34, 77}, {-50, 39}, {-49, 20});
    CubicHermiteSpline crossSpline = CubicHermiteSpline({-49.5, 39}, {-50, 20}, {-50, -12}, {-43, -41.5});
    CubicHermiteSpline southernRingSpline = CubicHermiteSpline({-50, -12}, {-38, -125.5}, {-22, -56}, {-13.5, -83});
    CubicHermiteSpline ladderSpline = CubicHermiteSpline({-26, -56}, {-24.5, -22.5}, {-28.3, -20}, {-32, 5});
    CubicHermiteSpline ladder2Spline = CubicHermiteSpline({-28.25, -20}, {-32, 5}, {-11, -15}, {-2, -13});

    // profile setup
    MotionProfile* goalProfile = new MotionProfile(&goalSpline, RPMtoIPS(600));
    MotionProfile* innerRingProfile = new MotionProfile(&innerRingSpline, RPMtoIPS(600), 
        {
            {{0, 0.1}, {0.05, 1}}, 
            {{0.05, 1}, {1, 1}}
        }
    );
    MotionProfile* outerRingProfile = new MotionProfile(&outerRingSpline, RPMtoIPS(600),
        {
            {{0, 0.1}, {0.1, 1}}, 
            {{0.1, 1}, {1, 1}}
        }
    );
    MotionProfile* crossProfile = new MotionProfile(&crossSpline, RPMtoIPS(600),
        {
            {{0, 1}, {0.9, 1}},
            {{0.5, 1}, {0.6, 0.3}},
            {{0.9, 0.3}, {1, 0.3}}
        }
    );
    MotionProfile* southernRingProfile = new MotionProfile(&southernRingSpline, RPMtoIPS(600), {
        {
            {{0, 0.3}, {0.4, 0.3}},
            {{0.4, 0.3}, {0.5, 1}},
            {{0.5, 1}, {0.9, 1}},
            {{0.9, 1}, {1, 0}}
        }
    });
    MotionProfile* ladderProfile = new MotionProfile(&ladderSpline, RPMtoIPS(600));
    MotionProfile* ladder2Profile = new MotionProfile(&ladder2Spline, RPMtoIPS(600));

    // scores on Alliance Stake
    drivetrain.move_relative(220, 150);
    arm.move(70);
    waitUntil(arm.get_position() > 450);
    pros::delay(250);
    drivetrain.move_relative(-220, 150);
    pros::delay(200);
    arm.move(-70);
    waitUntil(arm.get_position() < 230);
    arm.brake();

    // moves to goal and grabs it
    PIDTurner(255, 2);
    follower.addAction([](){clamp.set_value(true);}, 0.98);
    follower.startProfile(goalProfile, true);
    follower.clearActions();

    // moves to inner Ring stack and takes the correct Ring
    PIDTurner(findHeadingOfLine({universalCurrentLocation.x, universalCurrentLocation.y}, {-24, 48}), 2);
    intake.move(128);
    transport.move(96);
    follower.startProfile(innerRingProfile);

    // moves to middle Ring stack and takes those Rings
    follower.startProfile(outerRingProfile);
    transport.move(128);

    // moves all the way to the other side of the field, grabbing the middle Ring and dropping the first MoGo
    follower.addAction([](){inPutston.set_value(true);}, 0.6);
    follower.startProfile(crossProfile);
    follower.clearActions();

    // moves to the southern Ring stack (on the Goal side)
    follower.addAction([](){inPutston.set_value(false);}, 0.05);
    follower.addAction([](){clamp.set_value(false); transport.brake();}, 0.4);
    follower.startProfile(southernRingProfile);
    follower.clearActions();

    // grabs the third MoGo and touches the Ladder with it
    follower.addAction([](){clamp.set_value(true); transport.move(128);}, 0.98);
    follower.startProfile(ladderProfile, true);
    follower.clearActions();
    pros::delay(750);
    follower.startProfile(ladder2Profile, true);
    arm.move_relative(-180, 200);
}

void RedGoalRush() {

    VelocityController follower = VelocityController();

    // spline setup
    CubicHermiteSpline rushSpline = CubicHermiteSpline({-52.8, -58.6}, {22.4, -36.5}, {-16.8, -48}, {22.37, -36.3});
    CubicHermiteSpline secondGoalSpline = CubicHermiteSpline({-26.98, -53.06}, {-56.3, -44}, {-32.66, -28.5}, {2.6, -10.6});
    CubicHermiteSpline cornerSpline = CubicHermiteSpline({-21.9, -22.03}, {-73, -55.3}, {-62.2, -52.4}, {-69.1, -84.4});
    CubicHermiteSpline ladderSpline = CubicHermiteSpline({-40.33, -55.9}, {-15.65, -59.76}, {-9.1, -28.1}, {-17.4, -8.6});
    // profile setup
    MotionProfile* rushProfile = new MotionProfile(&rushSpline, 600);
    MotionProfile* secondGoalProfile = new MotionProfile(&secondGoalSpline, 400);
    MotionProfile* cornerProfile = new MotionProfile(&cornerSpline, 600);
    MotionProfile* ladderProfile = new MotionProfile(&ladderSpline, 500);

    // rushes MoGo in middle
    intake.move(128);
    yoin.set_value(true);
    follower.addAction([](){ker.set_value(true);}, 0.95);
    pros::Task armMovement = pros::Task([](){
        arm.move(128);
        waitUntil(arm.get_position() < 120);
        arm.brake();
    });
    follower.startProfile(rushProfile);
    follower.clearActions();
    armMovement.remove();

    // backs up from the line, then turns and moves forward to score the preload on the MoGo with the arm
    PIDMover({-30, -52}, true);
    PIDTurner((universalCurrentLocation.heading + 25), 2);
    arm.move(128);
    drivetrain.move(80);
    pros::delay(350);
    arm.brake();
    drivetrain.brake();

    // aligns with second MoGo via a curve and grabs that MoGo
    follower.addAction([](){ker.set_value(false);}, 0.65);
    follower.startProfile(secondGoalProfile, true);
    follower.clearActions();
    PIDMover({-22, -22}, true, {[](){clamp.set_value(true);}}, {0.9});

    // moves into Corner and gets its Rings
    follower.addAction([](){yoin.set_value(true);}, 0.8);
    follower.startProfile(cornerProfile);
    follower.clearActions();
    PIDTurner(findHeadingOfLine({universalCurrentLocation.x, universalCurrentLocation.y}, {-40.33, -55.9}), 1);
    intake.move(-128);
    PIDMover({-40.33, -55.9});

    // raises arm and moves to ladder to contact it
    armMovement = pros::Task([](){
        arm.move(128);
        waitUntil(arm.get_position() < -50);
        arm.brake();
    });
    follower.startProfile(ladderProfile);
    armMovement.remove();
}

void BlueGoalRush() {

    VelocityController follower = VelocityController();

    // spline setup
    CubicHermiteSpline rushSpline = CubicHermiteSpline({52.3, -32.8}, {-17.2, -54.1}, {16.89, -43.68}, {-17.2, -54.2});
    CubicHermiteSpline secondGoalSpline = CubicHermiteSpline({27.03, -42}, {54.95, -37.2}, {32.66, -28.5}, {-2.6, -10.6});
    CubicHermiteSpline cornerSpline = CubicHermiteSpline({21.9, -22.03}, {73, -55.3}, {53.3, -62.2}, {82.1, -72.3});
    CubicHermiteSpline ladderSpline = CubicHermiteSpline({60, -46.9}, {45, -94.6}, {9.1, -28.1}, {17.4, -8.6});
    // profile setup
    MotionProfile* rushProfile = new MotionProfile(&rushSpline, RPMtoIPS(600));
    MotionProfile* secondGoalProfile = new MotionProfile(&secondGoalSpline, RPMtoIPS(400));
    MotionProfile* cornerProfile = new MotionProfile(&cornerSpline, RPMtoIPS(600));
    MotionProfile* ladderProfile = new MotionProfile(&ladderSpline, RPMtoIPS(500));

    // rushes MoGo in middle
    intake.move(128);
    yoin.set_value(true);
    follower.addAction([](){ker.set_value(true);}, 0.95);
    pros::Task armMovement = pros::Task([](){
        arm.move(128);
        waitUntil(arm.get_position() < 120);
        arm.brake();
    });
    follower.startProfile(rushProfile);
    follower.clearActions();
    armMovement.remove();

    // backs up from the line, then turns and moves forward to score the preload on the MoGo with the arm
    PIDMover({30, -39.65}, true);
    PIDTurner((universalCurrentLocation.heading + 25), 2);
    arm.move(128);
    drivetrain.move(80);
    pros::delay(350);
    arm.brake();
    drivetrain.brake();

    // aligns with second MoGo via a curve and grabs that MoGo
    follower.addAction([](){ker.set_value(false);}, 0.65);
    follower.startProfile(secondGoalProfile, true);
    follower.clearActions();
    PIDMover({-22, -22}, true, {[](){clamp.set_value(true);}}, {0.9});

    // moves into Corner and gets its Rings
    follower.addAction([](){yoin.set_value(true);}, 0.8);
    follower.startProfile(cornerProfile);
    follower.clearActions();
    PIDTurner(findHeadingOfLine({universalCurrentLocation.x, universalCurrentLocation.y}, {-40.33, -55.9}), 1);
    intake.move(-128);
    PIDMover({40.33, -55.9});

    // raises arm and moves to ladder to contact it
    armMovement = pros::Task([](){
        arm.move(128);
        waitUntil(arm.get_position() < -50);
        arm.brake();
    });
    follower.startProfile(ladderProfile);
    armMovement.remove();
}

void redRingside() {}

void blueRingside() {}
/*
void autoSkills() {
    /*
    VelocityController follower = VelocityController();

    // spline setup
    CubicHermiteSpline Q3GoalSpline = CubicHermiteSpline({-52, 0}, {-45.5, -40}, {-49, -28}, {-45.5, -40});
    CubicHermiteSpline southWallSpline = CubicHermiteSpline({-48, -29}, {32, 53.5}, {8, -65}, {8, -100});
    CubicHermiteSpline Q3Ring1Spline = CubicHermiteSpline({-0.1, -62}, {-8, -31.5}, {-47.5, -47}, {-40, -89.5});
    CubicHermiteSpline Q3Ring2Spline = CubicHermiteSpline({-47.5, -47}, {-31, -123}, {-59, -46.5}, {-60, -31.6});
    // profile setup
    MotionProfile* Q3GoalProfile = new MotionProfile(&Q3GoalSpline, RPMtoIPS(600));
    MotionProfile* southWallProfile = new MotionProfile(&southWallSpline, RPMtoIPS(600));
    MotionProfile* Q3Ring1Profile = new MotionProfile(&Q3Ring1Spline, RPMtoIPS(600));
    MotionProfile* Q3Ring2Profile = new MotionProfile(&Q3Ring2Spline, RPMtoIPS(600));

    // scores on Alliance Stake
    arm.move(70);
    waitUntil(arm.get_position() > 450);
    pros::delay(250);
    drivetrain.move_relative(-220, 600);
    pros::delay(200);
    arm.move(-70);
    waitUntil(arm.get_position() < -5);
    arm.brake();

    // grabs first MoGo
    PIDTurner(findHeadingOfLine({universalCurrentLocation.x, universalCurrentLocation.y}, {-49, -29}) - 180, 2);
    follower.addAction([](){clamp.set_value(true);}, 0.9);
    follower.startProfile(Q3GoalProfile, true);
    follower.clearActions();

    // moves to Wall Stake
    PIDTurner(45, 2);
    intake.move(128);
    pros::Task* loadArm = new pros::Task([](){
        arm.move(128);
        waitUntil(arm.get_position() > 10);
        arm.brake();
    });
    pros::Task* raiseArm = new pros::Task([](){
        arm.move(128);
        waitUntil(arm.get_position() > 180);
        arm.brake();
    });
    raiseArm->suspend();
    follower.addAction([raiseArm](){raiseArm->resume();}, 0.6);
    follower.startProfile(southWallProfile);
    raiseArm->remove();
    */
   
}
*/

void autoSkills() {
    // QUADRANT 1
	// sets the time from initialization that the code starts at
	int startTime = pros::millis();

	// mid-PID actions
	auto gripMoGoM = []() {clamp.set_value(true);};
	
	// scores on the Alliance Stake
	double initialPos = transport.get_position();
	transport.move_relative(340, 200);
	waitUntil((transport.get_position() >= initialPos - 340) || ((pros::millis() - startTime) / 1000 >= 0.7));
	transport.move_relative(-100, 200);
	pros::delay(300);
	
	// moves forward from the Alliance Stake
	auto posFN = []() {return (rightRear.get_position() + leftRear.get_position() + rightFront.get_position() + leftFront.get_position()) / 4;};
	initialPos = posFN();
	drivetrain.move_relative(530, 200);
	waitUntil(posFN() >= initialPos + 530);

	// turns to a MoGo and moves to it, then grabs it
	CutoffTurnPID({-49, -19}, true, 1000, 1);
	PIDMover({-49, -19.5}, true, {gripMoGoM}, {18});

	// turns and moves to a Ring, then grabs it
	CutoffTurnPID({-26, -24}, false, 800, 2);
	intake.move(-128);
	arm.move_relative(380, 200);
	CutoffPID({-26, -24}, false, 900);

	// turns and moves to the Ring on the line, then grabs it
	PIDTurner(findHeadingOfLine({universalCurrentLocation.x, universalCurrentLocation.y}, {-8, -48}), 2);
	CutoffPID({-8, -48}, false, 2200);

	// turns and moves to the next three Rings in a line, automatically grabbing them along the way
	transport.brake();
	PIDTurner(findHeadingOfLine({universalCurrentLocation.x, universalCurrentLocation.y}, {-57, -45}), 2);
	transport.move(128);
	CutoffPID({-36, -49}, false, 1800);
	CutoffPID({-57, -43}, false, 1850);
	pros::delay(500);

	// turns and moves to the final Ring in this quadrant, then grabs it automatically
	transport.brake();
	CutoffTurnPID({-49.5, -55.5}, false, 1000, 1);
	transport.move(128);
	CutoffPID({-49.5, -53.5}, false, 1500);

	// turns to the Corner and places the Mobile Goal there
	CutoffTurnPID({-58, -57.5}, true, 1300, 1);
	CutoffPID({-58, -57.5}, true, 500);
	clamp.set_value(false);
	transport.move_relative(-600, 200);
	pros::delay(200);



// QUADRANT 2
		// moves forward from the Corner
		initialPos = posFN();
		drivetrain.move_relative(200, 200);
		waitUntil(posFN() >= initialPos + 200);

		// turns to face the MoGo on the opposite quadrant, then moves to it and grabs it
		PIDTurner(findHeadingOfLine({universalCurrentLocation.x, universalCurrentLocation.y}, {-44, 10}) - 180, 2);
		transport.move(-128);
		PIDMover({-44, 10}, true);
		transport.brake();
		drivetrain.move(-80);
		pros::delay(500);
		clamp.set_value(true);
		pros::delay(20);
		drivetrain.brake();

		// turns and moves to a Ring, then grabs it
		PIDTurner(findHeadingOfLine({universalCurrentLocation.x, universalCurrentLocation.y}, {-26, 16}), 1);
		intake.move(-128);
		CutoffPID({-26, 24}, false, 1500);


		// turns and moves to the Ring on the line, then grabs it
		PIDTurner(findHeadingOfLine({universalCurrentLocation.x, universalCurrentLocation.y}, {-5, 48}), 1);
		CutoffPID({-5, 48}, false, 1800);
		

		// turns and moves to the next three Rings in a line, automatically grabbing them along the way
		transport.brake();
		PIDTurner(findHeadingOfLine({universalCurrentLocation.x, universalCurrentLocation.y}, {-57, 48}), 1);
		transport.move(128);
		CutoffPID({-38, 58}, false, 1750);
		CutoffPID({-57, 48}, false, 1800);
		pros::delay(500);

		// turns and moves to the final Ring in this quadrant, then grabs it automatically
		CutoffTurnPID({-49.5, 55.5}, false, 1000, 2);
		CutoffPID({-49.5, 55.5}, false, 1000);

		// turns to the Corner and places the Mobile Goal there
		CutoffTurnPID({-54, 59}, true, 1000, 2);
		CutoffPID({-54, 59}, true, 1000);
		transport.move_relative(-600, 200);
		clamp.set_value(false);
		pros::delay(200);

		// moves forward from the Corner
		initialPos = posFN();
		drivetrain.move_relative(200, 200);
		waitUntil(posFN() >= initialPos + 200);

// QUADRANT 3
			// hi
			// hello
			// sup
			// Drives to quadrant 3 to get the Ring at (24,48)
			preRoller.move(128);
			PIDTurner(findHeadingOfLine({universalCurrentLocation.x, universalCurrentLocation.y}, {13.5, 45}), 1);
			//PIDMover({13.5, 45});
			CutoffPID({13.5, 45}, false, 2250);

			// Drives to get the Ring at (24,24)
			transport.move_relative(480,100);
			PIDTurner(findHeadingOfLine({universalCurrentLocation.x, universalCurrentLocation.y}, {19, 19}), 2);
			CutoffPID({19, 19}, false, 2000);
			transport.move_relative(300,100);

			// Move and grab the Goal at (48,0)
			PIDTurner(findHeadingOfLine({universalCurrentLocation.x, universalCurrentLocation.y}, {30, 4}) - 180, 1);
			//PIDMover({30, 4}, true);
			initialPos = posFN();
			drivetrain.move_relative(-1740,150);
			waitUntil(posFN() <= initialPos - 1740);
			clamp.set_value(true);
			drivetrain.brake();
			transport.move(128);

			// Move to grab the middle Ring in the corner
			CutoffTurnPID({38, 36}, false, 1000, 2);
			CutoffPID({38, 36}, false, 1250);

			CutoffTurnHeadingPID(90, false, 1000, 2);
			initialPos = posFN();
			drivetrain.move_relative(450, 600);
			waitUntil(posFN() >= initialPos + 450);

			// turns and moves to Corner
			//CutoffTurnHeadingPID(220, false, 1500, 2);
			PIDTurner(220, 2);
			clamp.set_value(false);
			drivetrain.move(-64);
			pros::delay(600);
			drivetrain.brake();
			pros::delay(300);

			// moves forward from the Corner
			CutoffPID({37, 35}, false, 1000);
			PIDTurner(351, 2);
			drivetrain.move(-128);
			pros::delay(3000);
			drivetrain.move(70);
			pros::delay(750);
			drivetrain.brake();
}

void autoTest() {

}