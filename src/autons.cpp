#include "init.h"
#include "profiling.h"

void globalBlueRing() {
    /*
    // velocity controller class
    VelocityController follower = VelocityController();

    // profile setup with pregeneration
    MotionProfile* goalProfile = new MotionProfile(&BlueSoloAWP::goalProfile, BlueSoloAWP::goalSpeed);
    MotionProfile* tempProfile = new MotionProfile(&BlueSoloAWP::tempProfile, BlueSoloAWP::tempSpeed);
    */
    /*MotionProfile* midringProfile1 = new MotionProfile(&SoloAWP::midring1Profile, SoloAWP::midring1Speed);
    MotionProfile* midringProfile2 = new MotionProfile(&SoloAWP::midring2Profile, SoloAWP::midring2Speed);
    MotionProfile* crossP1Profile = new MotionProfile(&SoloAWP::crossP1Profile, SoloAWP::crossP1Speed);
    MotionProfile* crossP2Profile = new MotionProfile(&SoloAWP::crossP2Profile, SoloAWP::crossP2Speed);
    MotionProfile* goal2Profile = new MotionProfile(&SoloAWP::goal2Profile, SoloAWP::goal2Speed);
    MotionProfile* ring2Profile = new MotionProfile(&SoloAWP::ring2Profile, SoloAWP::ring2Speed);
    MotionProfile* ladderProfile = new MotionProfile(&SoloAWP::ladderProfile, SoloAWP::ladderSpeed);


    // score on Alliance Stake
    drivetrain.move_relative(270, 300);
    pros::delay(200);
    arm.move(128);
    waitUntil(ArmRotational.get_position() < -13000);
    arm.brake();
    follower.clearActions();
    follower.addAction([](){arm.move(-128);}, 0.2);
    follower.addAction([](){clamp.set_value(true);}, 0.725);
    follower.addAction([](){arm.brake();}, 0.8);
    follower.startProfile(goalProfile, true);
    intake.move(-128);
    follower.clearActions();
    follower.startProfile(tempProfile, false);
    pros::delay(14000);
    */
    /*
    follower.startProfile(midringProfile1, false);
    follower.startProfile(midringProfile2, false);
    drivetrain.brake();
    pros::delay(1000);
    follower.startProfile(crossP1Profile, false);
    */
    /*
    pros::delay(500);
    follower.startProfile(crossP2Profile, false);
    pros::delay(500);
    follower.startProfile(goal2Profile, true);
    pros::delay(500);
    follower.startProfile(ring2Profile, false);
    pros::delay(500);
    follower.startProfile(ladderProfile, true);
    */

}

void globalRedRing() {
    /*
    // velocity controller class
    VelocityController follower = VelocityController();

    // profile setup with pregeneration
    MotionProfile* goalProfile = new MotionProfile(&RedSoloAWP::goalProfile, RedSoloAWP::goalSpeed);
    MotionProfile* tempProfile = new MotionProfile(&RedSoloAWP::tempProfile, RedSoloAWP::tempSpeed);
    */
    /*MotionProfile* midringProfile1 = new MotionProfile(&SoloAWP::midring1Profile, SoloAWP::midring1Speed);
    MotionProfile* midringProfile2 = new MotionProfile(&SoloAWP::midring2Profile, SoloAWP::midring2Speed);
    MotionProfile* crossP1Profile = new MotionProfile(&SoloAWP::crossP1Profile, SoloAWP::crossP1Speed);
    MotionProfile* crossP2Profile = new MotionProfile(&SoloAWP::crossP2Profile, SoloAWP::crossP2Speed);
    MotionProfile* goal2Profile = new MotionProfile(&SoloAWP::goal2Profile, SoloAWP::goal2Speed);
    MotionProfile* ring2Profile = new MotionProfile(&SoloAWP::ring2Profile, SoloAWP::ring2Speed);
    MotionProfile* ladderProfile = new MotionProfile(&SoloAWP::ladderProfile, SoloAWP::ladderSpeed);*/

/*
    // score on Alliance Stake
    drivetrain.move_relative(270, 100);
    pros::delay(200);
    arm.move(128);
    waitUntil(ArmRotational.get_position() < -13000);
    arm.brake();
    follower.clearActions();
    follower.addAction([](){arm.move(-128);}, 0.2);
    follower.addAction([](){clamp.set_value(true);}, 0.725);
    follower.addAction([](){arm.brake();}, 0.8);
    follower.startProfile(goalProfile, true);
    intake.move(-128);
    follower.clearActions();
    follower.startProfile(tempProfile, false);
    pros::delay(14000);
*/
    /*
    follower.startProfile(midringProfile1, false);
    follower.startProfile(midringProfile2, false);
    drivetrain.brake();
    pros::delay(1000);
    follower.startProfile(crossP1Profile, false);
    */
    /*
    pros::delay(500);
    follower.startProfile(crossP2Profile, false);
    pros::delay(500);
    follower.startProfile(goal2Profile, true);
    pros::delay(500);
    follower.startProfile(ring2Profile, false);
    pros::delay(500);
    follower.startProfile(ladderProfile, true);
    */
    
}

void globalBlueGoal() {
    globalRedRing();
}

void globalRedGoal() {
    globalBlueRing();
}


void redGoalside() {
/*
    // velocity controller class
    VelocityController follower = VelocityController();

    // profile setup with pregeneration
    MotionProfile* rushProfile = new MotionProfile(&RedMoGoRush::rushProfile, RedMoGoRush::rushSpeed);
    MotionProfile* yoinkProfile = new MotionProfile(&RedMoGoRush::yoinkProfile, RedMoGoRush::yoinkSpeed);
    MotionProfile* backProfile = new MotionProfile(&RedMoGoRush::backProfile, RedMoGoRush::backSpeed);
    MotionProfile* dropProfile = new MotionProfile(&RedMoGoRush::dropProfile, RedMoGoRush::dropSpeed);
    MotionProfile* centerProfile = new MotionProfile(&RedMoGoRush::centerProfile, RedMoGoRush::centerSpeed);
    MotionProfile* ladderProfile = new MotionProfile(&RedMoGoRush::ladderProfile, RedMoGoRush::ladderSpeed);

    // score on Alliance Stake
    yoin.set_value(true);
    preRoller.move(-128);
    follower.clearActions();
    follower.addAction([](){ker.set_value(true);}, 0.95);
    follower.startProfile(rushProfile, false);
    follower.clearActions();
    // follower.addAction([](){transport.move_relative(-360, 600);}, 0.3);
    follower.addAction([](){ker.set_value(false);}, 0.7);
    follower.addAction([](){clamp.set_value(true);}, 0.8);
    follower.startProfile(yoinkProfile, true);
    follower.clearActions();
    //follower.addAction([](){clamp.set_value(true);}, 0.08);
    //follower.startProfile(backProfile, true);
    drivetrain.brake();
    pros::delay(200);
    transport.move_relative(-300, 600);
    pros::delay(550);
    follower.clearActions();
    clamp.set_value(false);
    follower.addAction([](){yoin.set_value(false);}, 0.7);
    // follower.addAction([](){clamp.set_value(false);}, 0.5);
    follower.startProfile(dropProfile, false);
    drivetrain.brake();
    follower.clearActions();
    drivetrain.move_relative(-1500, 200);
    pros::delay(1300);
    clamp.set_value(true);
    transport.move_relative(-300, 600);
    pros::delay(14000);
    /*
    pros::delay(600);
    follower.clearActions();
    inPutston.set_value(true);
    follower.addAction([](){inPutston.set_value(false);}, 0.65);
    follower.addAction([](){transport.brake();}, 0.85);
    follower.startProfile(centerProfile, false);
    drivetrain.move_relative(270, 100);
    pros::delay(600);
    rightDrivetrain.move_relative(120, 100);
    pros::delay(300);
    leftDrivetrain.move_relative(90, 100);
    arm.move(128);
    waitUntil(ArmRotational.get_position() < -13000);
    drivetrain.move_relative(-120, 100);
    arm.move(-128);
    waitUntil(ArmRotational.get_position() > -1000);
    arm.brake();
    follower.clearActions();
    // follower.startProfile(ladderProfile, true);
    pros::delay(14000);
    */
}

void blueGoalside() {
    std::cout << "START!\n";
    // velocity controller class
    VelocityController follower = VelocityController();
    double start = pros::millis();
    // spline paths
    CubicHermiteSpline rushSpline = CubicHermiteSpline({50.5, -35.5}, {-40.5, -19.5}, {17, -49}, {-40.5, -19.5});
    //CubicHermiteSpline yoinkSpline = CubicHermiteSpline({-26.5, -49.5}, {-24, -11}, {-16, -17}, {100, 50});
    double dilation = 2.2;
    CubicHermiteSpline yoinkSpline = CubicHermiteSpline({12, -51}, {dilation * 40.5, dilation * 19.5}, {10 + (dilation * (25 - 10)), -52 + (dilation * (-28 - -52))}, {dilation * -20, dilation * -6});
    CubicHermiteSpline backSpline = CubicHermiteSpline({25, -28}, {-12, 8}, {19, -20}, {-12, 8});
    CubicHermiteSpline dropSpline = CubicHermiteSpline({19, -20}, {12, -8}, {55, -25}, {-15, 40});
    /*CubicHermiteSpline grabSpline = CubicHermiteSpline({-55, -25}, {5, -40}, {-10.5, -52.5}, {0, -20});
    CubicHermiteSpline cornerSpline = CubicHermiteSpline({-21, -47}, {-10, -20}, {-57, -57.5}, {-15, -15});
    CubicHermiteSpline ladderSpline = CubicHermiteSpline({-57, -57.5}, {15, 15}, {-11, -30}, {10, 10});*/
    CubicHermiteSpline centerSpline = CubicHermiteSpline({16.5, -53.25}, {120, 20}, {60, 5}, {100, -40});
    CubicHermiteSpline ladderSpline = CubicHermiteSpline({63, -3}, {-15, 10}, {22, -9}, {-22, 9});

    // profile generation
    MotionProfile* rushProfile = new MotionProfile(
        &rushSpline,
        RPMtoIPS(600),
        {{{0, 0.1}, {0.2, 1}}, {{0.2, 1}, {0.6, 1}}, {{0.6, 1}, {1, 0.25}}}
    );
    MotionProfile* yoinkProfile = new MotionProfile(
        &yoinkSpline, 
        RPMtoIPS(400),
        {{{0, 1}, {0.9, 1}}, {{0.5, 1}, {1, 0.25}}}
    );
    MotionProfile* backProfile = new MotionProfile(
        &backSpline,
        RPMtoIPS(200),
        {{{0, 1}, {0.8, 1}}, {{0.25, 1}, {1, 0}}});
    MotionProfile* dropProfile = new MotionProfile(&dropSpline, RPMtoIPS(600));
    //MotionProfile* grabProfile = new MotionProfile(&grabSpline, RPMtoIPS(300));
    MotionProfile* centerProfile = new MotionProfile(&centerSpline, RPMtoIPS(300));
    MotionProfile* ladderProfile = new MotionProfile(&ladderSpline, RPMtoIPS(300));
    std::cout << "time taken = " << (pros::millis() - start) << "\n";
    return;
    // score on Alliance Stake
    yoin.set_value(true);
    preRoller.move(-128);
    follower.clearActions();
    // follower.addAction([](){ker.set_value(true);}, 0.99);
    follower.startProfile(rushProfile, false);
    ker.set_value(true);
    follower.clearActions();
    // follower.addAction([](){transport.move_relative(-360, 600);}, 0.3);
    follower.addAction([](){ker.set_value(false);}, 0.8);
    //follower.addAction([](){clamp.set_value(true);}, 0.98);
    follower.startProfile(yoinkProfile, true);
    follower.clearActions();
    //follower.addAction([](){clamp.set_value(true);}, 0.08);
    //follower.startProfile(backProfile, true);
    pros::delay(100);
    clamp.set_value(true);
    transport.move_relative(-600, 600);
    pros::delay(150);
    drivetrain.brake();
    pros::delay(750);
    follower.clearActions();
    clamp.set_value(false);
    follower.addAction([](){yoin.set_value(false);}, 0.6);
    // follower.addAction([](){clamp.set_value(false);}, 0.5);
    follower.startProfile(dropProfile, false);
    drivetrain.brake();
    follower.clearActions();
    drivetrain.move_relative(-1250, 200);
    pros::delay(1200);
    clamp.set_value(true);
    transport.move(-128);
    pros::delay(14000);
}

void redRingside() {}

void blueRingside() {}

void autoSkills() {}

void autoTest() {
    intake.move(128);
}