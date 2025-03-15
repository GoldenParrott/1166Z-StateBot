#include "init.h"
#include "profiling.h"

void BlueAWP() {
    VelocityController follower = VelocityController();

    // profile setup
    MotionProfile* goalProfile = path[0];
    MotionProfile* innerRingProfile = path[1];
    MotionProfile* outerRingProfile = path[2];
    MotionProfile* crossProfile = path[3];
    MotionProfile* southernRingProfile = path[4];
    MotionProfile* ladderProfile = path[5];
    MotionProfile* ladder2Profile = path[6];

    // scores on Alliance Stake
    drivetrain.move_relative(220, 150);
    arm.move(70);
    waitUntil(arm.get_position() > 450);
    pros::delay(250);
    drivetrain.move_relative(-220, 150);
    pros::delay(200);
    arm.move(-70);
    waitUntil(arm.get_position() < 270);
    arm.brake();

    // moves to goal and grabs it
    PIDTurner(110, 1);
    follower.addAction([](){clamp.set_value(true);}, 0.98);
    follower.startProfile(goalProfile, true);
    follower.clearActions();

    // moves to inner Ring stack and takes the correct Ring
    PIDTurner(findHeadingOfLine({universalCurrentLocation.x, universalCurrentLocation.y}, {24, 48}), 1);
    intake.move(128);
    follower.startProfile(innerRingProfile);

    // moves to middle Ring stack and takes those Rings
    transport.move(128);

    // moves all the way to the other side of the field, grabbing the middle Ring and dropping the first MoGo
    follower.addAction([](){inPutston.set_value(true);}, 0.6);
    follower.startProfile(crossProfile);
    follower.clearActions();

    // moves to the southern Ring stack (on the Goal side)
    follower.addAction([](){inPutston.set_value(false);}, 0.05);
    follower.addAction([](){clamp.set_value(false); transport.brake();}, 0.4);
    follower.addAction([](){transport.move(128);}, 0.5);
    follower.addAction([](){transport.brake();}, 0.8);
    follower.startProfile(southernRingProfile);
    follower.clearActions();

    // grabs the third MoGo and touches the Ladder with it
    follower.addAction([](){clamp.set_value(true); transport.move(128);}, 0.98);
    follower.startProfile(ladderProfile, true);
    follower.clearActions();
    pros::delay(750);
    arm.set_brake_mode(pros::MotorBrake::coast);
    follower.addAction([](){arm.move_relative(-180, 100);}, 0.5);
    follower.startProfile(ladder2Profile, true);
}

void RedAWP() {
    VelocityController follower = VelocityController();

    // profile setup
    MotionProfile* goalProfile = path[0];
    MotionProfile* innerRingProfile = path[1];
    MotionProfile* outerRingProfile = path[2];
    MotionProfile* crossProfile = path[3];
    MotionProfile* southernRingProfile = path[4];
    MotionProfile* ladderProfile = path[5];
    MotionProfile* ladder2Profile = path[6];

    // scores on Alliance Stake
    drivetrain.move_relative(220, 150);
    arm.move(70);
    waitUntil(arm.get_position() > 450);
    pros::delay(250);
    drivetrain.move_relative(-220, 150);
    pros::delay(200);
    arm.move(-70);
    waitUntil(arm.get_position() < 270);
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
    follower.addAction([](){transport.move(128);}, 0.5);
    follower.addAction([](){transport.brake();}, 0.8);
    follower.startProfile(southernRingProfile);
    follower.clearActions();

    // grabs the third MoGo and touches the Ladder with it
    follower.addAction([](){clamp.set_value(true); transport.move(128);}, 0.98);
    follower.startProfile(ladderProfile, true);
    follower.clearActions();
    pros::delay(750);
    arm.set_brake_mode(pros::MotorBrake::coast);
    follower.addAction([](){arm.move_relative(-180, 100);}, 0.5);
    follower.startProfile(ladder2Profile, true);
}

void RedGoalRush() {

    VelocityController follower = VelocityController();

    MotionProfile* rushProfile = path[0];
    MotionProfile* secondGoalProfile = path[1];
    MotionProfile* cornerProfile = path[2];
    MotionProfile* ladderProfile = path[3];

    // rushes MoGo in middle
    preRoller.move(128);
    yoin.set_value(true);
    follower.addAction([](){ker.set_value(true);}, 0.98);
    follower.startProfile(rushProfile);
    follower.clearActions();
    drivetrain.brake();

    // backs up from the line, then turns and moves forward to score the preload on the MoGo with the arm
    PIDMover({-30, -52}, true);
    ker.set_value(false);
    CutoffTurnHeadingPID((universalCurrentLocation.heading + 19), false, 500, 2);
    yoin.set_value(false);
    arm.move(96);
    drivetrain.move(80);
    pros::delay(200);
    drivetrain.brake();
    pros::delay(800);
    arm.brake();

    // aligns with second MoGo via a curve and grabs that MoGo
    follower.addAction([](){clamp.set_value(true);}, 0.95);
    follower.startProfile(secondGoalProfile, true);
    follower.clearActions();
    transport.move(128);

    // moves into Corner and sweeps it
    follower.addAction([](){yoin.set_value(true);}, 0.6);
    follower.startProfile(cornerProfile);
    follower.clearActions();
    CutoffTurnPID({-40.33, -55.9}, false, 1000, 1);
    intake.move(128);

    // raises arm and moves to ladder to contact it
    follower.addAction([](){inPutston.set_value(true);}, 0.4);
    follower.addAction([](){yoin.set_value(false);}, 0.4);
    follower.addAction([](){inPutston.set_value(false);}, 0.8);
    follower.startProfile(ladderProfile);
    drivetrain.brake();
    follower.clearActions();
    pros::Task armMovement = pros::Task([](){
        arm.move(-128);
        waitUntil(arm.get_position() < -50);
        arm.brake();
    });
    pros::delay(1000);
    armMovement.remove();

}

void BlueGoalRush() {

    VelocityController follower = VelocityController();

    MotionProfile* rushProfile = path[0];
    MotionProfile* secondGoalProfile = path[1];
    MotionProfile* cornerProfile = path[2];
    MotionProfile* ladderProfile = path[3];

    // rushes MoGo in middle
    preRoller.move(128);
    yoin.set_value(true);
    follower.addAction([](){ker.set_value(true);}, 0.98);
    follower.startProfile(rushProfile);
    // PIDMover({13.5, -44.7}, false, {[](){ker.set_value(true);}}, {18});
    follower.clearActions();
    drivetrain.brake();

    // backs up from the line, then turns and moves forward to score the preload on the MoGo with the arm
    PIDMover({30, -39.65}, true);
    ker.set_value(false);
    CutoffTurnHeadingPID((universalCurrentLocation.heading + 19), false, 500, 2);
    yoin.set_value(false);
    arm.move(96);
    drivetrain.move(80);
    pros::delay(200);
    drivetrain.brake();
    pros::delay(1000);
    arm.brake();

    // aligns with second MoGo via a curve and grabs that MoGo
    follower.addAction([](){clamp.set_value(true);}, 0.9);
    follower.startProfile(secondGoalProfile, true);
    follower.clearActions();
    transport.move(128);

    // moves into Corner and gets its Rings
    follower.addAction([](){yoin.set_value(true);}, 0.6);
    follower.startProfile(cornerProfile);
    follower.clearActions();
    CutoffTurnPID({60, -47}, false, 1000, 1);
    intake.move(128);

    // raises arm and moves to ladder to contact it
    follower.addAction([](){inPutston.set_value(true);}, 0.4);
    follower.addAction([](){yoin.set_value(false);}, 0.4);
    follower.addAction([](){inPutston.set_value(false);}, 0.8);
    follower.startProfile(ladderProfile);
    drivetrain.brake();
    follower.clearActions();
    pros::Task armMovement = pros::Task([](){
        arm.move(-128);
        waitUntil(arm.get_position() < -50);
        arm.brake();
    });
    pros::delay(1000);
    armMovement.remove();

}

void redRingside() {
    VelocityController follower = VelocityController();

    // profile setup
    MotionProfile* goalProfile = path[0];
    MotionProfile* centerRingProfile = path[1];
    MotionProfile* centerStakeProfile = path[2];
    MotionProfile* ladderProfile = path[3];

    // scores on Alliance Stake
    drivetrain.move_relative(220, 150);
    arm.move(70);
    waitUntil(arm.get_position() > 450);
    pros::delay(250);
    drivetrain.move_relative(-220, 150);
    pros::delay(200);
    arm.move(-70);
    waitUntil(arm.get_position() < 270);
    arm.brake();

    // moves to goal and grabs it
    PIDTurner(255, 2);
    follower.addAction([](){clamp.set_value(true);}, 0.98);
    follower.startProfile(goalProfile, true);
    follower.clearActions();

    // moves to inner Ring stack and takes the correct Ring
    PIDTurner(universalCurrentLocation.heading - 180, 2);
    intake.move(128);
    follower.startProfile(centerRingProfile);
    pros::delay(1000);

    // moves all the way to the other side of the field, grabbing the middle Ring and dropping the first MoGo
    PIDTurner(225, 1);
    follower.addAction([](){inPutston.set_value(true);}, 0.6);
    follower.startProfile(centerStakeProfile);
    follower.clearActions();

    // touches Ladder
    follower.startProfile(ladderProfile);
    drivetrain.brake();
    follower.clearActions();
    pros::Task armMovement = pros::Task([](){
        arm.move(-128);
        waitUntil(arm.get_position() < -50);
        arm.brake();
    });
    pros::delay(1000);
    armMovement.remove();
}

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

/*
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
*/

void Skills() {

}

void autoTest() {

}