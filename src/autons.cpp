#include "init.h"
#include "profiling.h"

void AWP(int color) {
    VelocityController follower = VelocityController();

    // profile setup
    MotionProfile* goalProfile = path[0];
    MotionProfile* innerRingProfile = path[1];
    MotionProfile* outerRingProfile = path[2];
    MotionProfile* southernRingProfile = path[3];
    MotionProfile* ladderProfile = path[4];
    MotionProfile* ladder2Profile = path[5];

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
    drivetrain.move_relative(230, 150);
    arm.move(-70);
    waitUntil(((ArmRotational.get_position() / 100) > 130) && ((ArmRotational.get_position() / 100) < 150));
    pros::delay(250);
    drivetrain.move_relative(-230, 150);
    pros::delay(200);
    arm.move(70);
    waitUntil((ArmRotational.get_position() / 100) < -30);
    arm.brake();

    // moves to goal and grabs it
    if (color == -1) {
         PIDTurner(250, dirSet(true));
    } else {
         PIDTurner(120, dirSet(true));
    }
    follower.addAction([](){clamp.set_value(true);}, 0.98);
    follower.startProfile(goalProfile, true);
    follower.clearActions();

    // moves to inner Ring stack and takes the correct Ring
    pros::delay(250);
    CutoffTurnPID({double (color) * 24, 48}, false, 1000, dirSet(true));
    intake.move(128);
    follower.addAction([](){transport.brake();}, 0.7);
    follower.startProfile(innerRingProfile);
    follower.clearActions();

    // moves all the way to the other side of the field, grabbing the middle Ring and dropping the first MoGo
    follower.addAction([](){transport.move(128);}, 0.2);
    follower.addAction([](){inPutston.set_value(true);}, 0.7);
    follower.addAction([](){transport.move_relative(-250, 200);}, 0.8);
    follower.addAction([](){transport.move(128);}, 0.9);
    follower.addAction([](){inPutston.set_value(false);}, 0.95);
    transport.move(128);
    follower.startProfile(outerRingProfile);
    follower.clearActions();

    // moves to the southern Ring stack (on the Goal side)
    follower.addAction([](){inPutston.set_value(false);}, 0.05);
    follower.addAction([](){clamp.set_value(false); std::cout << "borb";}, 0.5);
    follower.startProfile(southernRingProfile);
    follower.clearActions();

    // grabs the third MoGo and touches the Ladder with it
    follower.addAction([](){transport.brake();}, 0.15);
    follower.addAction([](){clamp.set_value(true);}, 0.93);
    follower.startProfile(ladderProfile, true);
    follower.clearActions();
    transport.move(128);
    
    pros::delay(750);
    arm.set_brake_mode(pros::MotorBrake::coast);
    arm.move_relative(-18000, 200);
    PIDTurner(45, 1);
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
    yoinSet(true, false);
    follower.addAction([kerSet](){kerSet(true, false);}, 0.98);
    follower.startProfile(rushProfile);
    follower.clearActions();
    drivetrain.brake();

    // backs up from the line, then turns and moves forward to score the preload on the MoGo with the arm
    PIDMover({double (color) * 30, -52}, true);
    kerSet(false, false);
    CutoffTurnHeadingPID((universalCurrentLocation.heading + (color * 19)), false, 500, dirSet(false));
    yoinSet(false, false);
    arm.move(-128);
    drivetrain.move(80);
    pros::delay(200);
    drivetrain.brake();
    pros::delay(800);
    arm.brake();

    // aligns with second MoGo via a curve and grabs that MoGo
    follower.addAction([](){arm.move(128);}, 0.75);
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
    follower.addAction([yoinSet](){yoinSet(false, false);}, 0.05);
    follower.addAction([](){inPutston.set_value(false);}, 0.8);
    follower.addAction([](){clamp.set_value(false);}, 0.9);
    follower.startProfile(cornerRingProfile);
    follower.clearActions();

    // fetches the contested Goal again
    follower.addAction([](){clamp.set_value(true);}, 0.95);
    follower.startProfile(fetchProfile, true);
    follower.clearActions();
/*
    // touches the Ladder
    follower.startProfile(ladderProfile);
    drivetrain.brake();
    follower.clearActions();
    pros::Task armMovement = pros::Task([](){
        arm.move(128);
        waitUntil(ArmRotational.get_position() > 20);
        arm.brake();
    });
    pros::delay(1000);
    armMovement.remove();
*/
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
    follower.addAction([](){preRoller.move(128);}, 0.3);
    follower.addAction([](){transport.move(128); preRoller.move(128);}, 0.65);
    follower.addAction([](){transport.brake(); preRoller.move(128);}, 0.75);
    follower.startProfile(centerProfile);
    follower.clearActions();

    // grabs the goal with the two intaked Rings
    follower.addAction([](){clamp.set_value(true);}, 0.95);
    follower.startProfile(goalProfile, true);
    follower.clearActions();
    transport.move(128); preRoller.move(128);

    // moves to the Corner in order to sweep Rings
    follower.addAction([yoinSet](){yoinSet(true, true); preRoller.move(128);}, 0.6);
    follower.startProfile(cornerProfile);
    follower.clearActions(); preRoller.move(128);

    // sweeps the Corner Rings and intakes them
    PIDTurner(findHeadingOfLine({universalCurrentLocation.x, universalCurrentLocation.y}, {double (color) * 72, 36}), dirSet(false));
    follower.addAction([yoinSet](){yoinSet(false, true); preRoller.move(128);}, 0.5);
    follower.addAction([](){inPutston.set_value(true); preRoller.move(128);}, 0.7);
    follower.addAction([](){inPutston.set_value(false); preRoller.move(128);}, 0.9);
    follower.startProfile(sweepProfile, false);
    follower.clearActions(); preRoller.move(128);
    pros::delay(250);

    // moves back into the Ladder
    arm.move_relative(-750, 200);
    PIDTurner(findHeadingOfLine({universalCurrentLocation.x, universalCurrentLocation.y}, {double (color) * 24, 0}), dirSet(false));
    drivetrain.set_brake_mode(pros::MotorBrake::coast);
    drivetrain.move(128);
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