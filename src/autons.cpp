#include "init.h"
#include "profiling.h"

void AWP(int color) {
    VelocityController follower = VelocityController();

    // profile setup
    MotionProfile* goalProfile = path[0];
    MotionProfile* innerRingProfile = path[1];
    MotionProfile* outerRingProfile = path[2];
    MotionProfile* crossProfile = path[3];
    MotionProfile* southernRingProfile = path[4];
    MotionProfile* ladderProfile = path[5];
    MotionProfile* ladder2Profile = path[6];

    // determines turn direction based on color
    auto dirSet = [color](bool useRightOnRed) -> int {
        if ((useRightOnRed == true && color == -1) || (useRightOnRed == false && color == 1)) {
            return 2;
        }
        else {
            return 1;
        }
    };

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
    if (color == -1) {
        CutoffTurnHeadingPID(255, false, 1000, dirSet(true));
    } else {
        CutoffTurnHeadingPID(105, false, 1000, dirSet(true));
    }
    follower.addAction([](){clamp.set_value(true);}, 0.98);
    follower.startProfile(goalProfile, true);
    follower.clearActions();

    // moves to inner Ring stack and takes the correct Ring
    CutoffTurnPID({double (color) * 24, 48}, false, 1000, dirSet(true));
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

void GoalRush(int color) {

    VelocityController follower = VelocityController();

    MotionProfile* rushProfile = path[0];
    MotionProfile* secondGoalProfile = path[1];
    MotionProfile* cornerProfile = path[2];
    MotionProfile* cornerRingProfile = path[3];
    MotionProfile* fetchProfile = path[4];
    MotionProfile* ladderProfile = path[5];

    // functions for determining which yoinker to use
    auto yoinSet = [color](bool setTo, bool useRightOnRed){
        if ((useRightOnRed == true && color == -1) || (useRightOnRed == false && color == 1)) {
            rightYoin.set_value(setTo);
        }
        else {
            leftYoin.set_value(setTo);
        }
    };
    auto kerSet = [color](bool setTo, bool useRightOnRed){
        if ((useRightOnRed == true && color == -1) || (useRightOnRed == false && color == 1)) {
            rightKer.set_value(setTo);
        }
        else {
            leftKer.set_value(setTo);
        }
    };
    auto dirSet = [color](bool useRightOnRed) -> int {
        if ((useRightOnRed == true && color == -1) || (useRightOnRed == false && color == 1)) {
            return 2;
        }
        else {
            return 1;
        }
    };

    // rushes MoGo in middle
    preRoller.move(128);
    yoinSet(true, true);
    follower.addAction([kerSet](){kerSet(true, true);}, 0.98);
    follower.startProfile(rushProfile);
    follower.clearActions();
    drivetrain.brake();

    // backs up from the line, then turns and moves forward to score the preload on the MoGo with the arm
    PIDMover({double (color) * 30, -52}, true);
    kerSet(true, true);
    CutoffTurnHeadingPID((universalCurrentLocation.heading + (color * 19)), false, 500, dirSet(true));
    yoinSet(false, true);
    arm.move(96);
    drivetrain.move(80);
    pros::delay(200);
    drivetrain.brake();
    pros::delay(800);
    arm.brake();

    // aligns with second MoGo via a curve and grabs that MoGo
    follower.addAction([](){clamp.set_value(true);transport.move_relative(-500, 600);}, 0.95);
    follower.startProfile(secondGoalProfile, true);
    follower.clearActions();
    transport.move(128);

    // moves into Corner and sweeps it
    follower.addAction([](){transport.move(128);}, 0.3);
    follower.addAction([yoinSet](){yoinSet(true, false);}, 0.6);
    follower.startProfile(cornerProfile);
    follower.clearActions();
    CutoffTurnPID({double (color) * 59, -50}, false, 1000, dirSet(true));
    intake.move(128);

    // moves up, intaking and scoring Rings from the Corner
    follower.addAction([](){inPutston.set_value(true);}, 0.4);
    follower.addAction([yoinSet](){yoinSet(false, false);}, 0.4);
    follower.addAction([](){inPutston.set_value(false);}, 0.8);
    follower.addAction([](){clamp.set_value(false);}, 0.9);
    follower.startProfile(cornerRingProfile);
    follower.clearActions();

    // fetches the contested Goal again
    follower.addAction([](){clamp.set_value(true);}, 0.95);
    follower.startProfile(fetchProfile, true);
    follower.clearActions();

    // touches the Ladder
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

void RingSide(int color) {
    VelocityController follower = VelocityController();

    // profile setup
    MotionProfile* centerProfile = path[0];
    MotionProfile* goalProfile = path[1];
    MotionProfile* cornerProfile = path[2];
    MotionProfile* sweepProfile = path[3];
    MotionProfile* ladderProfile = path[4];

    // functions for determining which yoinker to use
    auto yoinSet = [color](bool setTo, bool useRightOnRed){
        if ((useRightOnRed == true && color == -1) || (useRightOnRed == false && color == 1)) {
            rightYoin.set_value(setTo);
        }
        else {
            leftYoin.set_value(setTo);
        }
    };
    auto kerSet = [color](bool setTo, bool useRightOnRed){
        if ((useRightOnRed == true && color == -1) || (useRightOnRed == false && color == 1)) {
            rightKer.set_value(setTo);
        }
        else {
            leftKer.set_value(setTo);
        }
    };
    auto dirSet = [color](bool useRightOnRed) -> int {
        if ((useRightOnRed == true && color == -1) || (useRightOnRed == false && color == 1)) {
            return 2;
        }
        else {
            return 1;
        }
    };

    // rushes the Rings in the center of the field
    intake.move(128);
    follower.startProfile(centerProfile);

    // grabs the goal with the two intaked Rings
    follower.addAction([](){clamp.set_value(true);}, 0.95);
    follower.startProfile(goalProfile, true);
    follower.clearActions();

    // moves to the Corner in order to sweep Rings
    follower.addAction([yoinSet](){yoinSet(true, true);}, 0.6);
    follower.startProfile(cornerProfile);
    follower.clearActions();

    // sweeps the Corner Rings and intakes them
    PIDTurner(findHeadingOfLine({universalCurrentLocation.x, universalCurrentLocation.y}, {double (color) * 55, 50}), dirSet(false));
    follower.addAction([yoinSet](){yoinSet(false, true);}, 0.5);
    follower.addAction([](){inPutston.set_value(true);}, 0.7);
    follower.startProfile(sweepProfile, false);
    follower.clearActions();

    // moves back into the Ladder
    follower.startProfile(ladderProfile, true);
}

void autoSkills() {
    VelocityController follower = VelocityController();

    // profile setup
    MotionProfile* goalQ3Profile = path[0];
    MotionProfile* ringMidQ3Profile = path[1];
    MotionProfile* wallQ3Profile = path[2];
    MotionProfile* ring3Q3Profile = path[3];
    MotionProfile* ringFarQ3Profile = path[4];
    MotionProfile* cornerQ3Profile = path[5];

    MotionProfile* crossQ34Profile = path[6];

    MotionProfile* goalQ4Profile = path[7];
    MotionProfile* wallQ4Profile = path[8];
    MotionProfile* ring3Q4Profile = path[9];
    MotionProfile* ringFarQ4Profile = path[10];
    MotionProfile* cornerQ4Profile = path[11];

    MotionProfile* crossQ41Profile = path[12];

    MotionProfile* sweepGoalQ12Profile = path[13];
    MotionProfile* goalQ12Profile = path[14];

    // QUADRANT 1
    // scores on Alliance Stake
    arm.move(70);
    waitUntil(arm.get_position() > 450);
    pros::delay(250);
    drivetrain.move_relative(-220, 150);
    pros::delay(200);
    arm.move(-70);
    waitUntil(arm.get_position() < 270);
    arm.brake();

	// turns to a MoGo and moves to it, then grabs it
	CutoffTurnPID({-50, -30}, true, 1000, 2);
    follower.addAction([](){clamp.set_value(true);}, 0.98);
    follower.startProfile(goalQ3Profile, true);
    follower.clearActions();

    // ring grab
    CutoffTurnPID({-22, -23.5}, false, 1000, 2);
    intake.move(128);
    follower.startProfile(ringMidQ3Profile);

    // midring grab
    CutoffTurnPID({0, -45.7}, false, 1000, 2);
    follower.startProfile(wallQ3Profile);

    // moves backward from the Wall Stake
	auto posFN = []() {return (rightRear.get_position() + leftRear.get_position() + rightFront.get_position() + leftFront.get_position()) / 4;};
	double initialPos = posFN();
	drivetrain.move_relative(-530, 200);
	waitUntil(posFN() >= initialPos - 530);

    // ring line
    CutoffTurnPID({-56, -47}, false, 1000, 2);
    follower.startProfile(ring3Q3Profile);

    // far ring
    CutoffTurnHeadingPID(180, false, 1000, 1);
    follower.startProfile(ringFarQ3Profile);

    // corner
    drivetrain.move_relative(-700, 300);
    pros::delay(500);
    clamp.set_value(false);
    pros::delay(200);

    // cross
    drivetrain.move_relative(400, 300);
    pros::delay(500);
    CutoffTurnHeadingPID(0, false, 1000, 1);
    follower.startProfile(crossQ34Profile);

}

void autoTest() {

}