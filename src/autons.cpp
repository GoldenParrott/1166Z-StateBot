#include "init.h"
#include "paths.h"
#include "profiling.h"

void globalBlueRing() {
/*
    // velocity controller class
    VelocityController follower = VelocityController();

    // profile setup with pregeneration
    MotionProfile* goalProfile = new MotionProfile(&SoloAWP::goalProfile, SoloAWP::goalSpeed);
    MotionProfile* midringProfile1 = new MotionProfile(&SoloAWP::midring1Profile, SoloAWP::midring1Speed);
    MotionProfile* midringProfile2 = new MotionProfile(&SoloAWP::midring2Profile, SoloAWP::midring2Speed);
    MotionProfile* crossP1Profile = new MotionProfile(&SoloAWP::crossP1Profile, SoloAWP::crossP1Speed);
    MotionProfile* crossP2Profile = new MotionProfile(&SoloAWP::crossP2Profile, SoloAWP::crossP2Speed);
    MotionProfile* goal2Profile = new MotionProfile(&SoloAWP::goal2Profile, SoloAWP::goal2Speed);
    MotionProfile* ring2Profile = new MotionProfile(&SoloAWP::ring2Profile, SoloAWP::ring2Speed);
    MotionProfile* ladderProfile = new MotionProfile(&SoloAWP::ladderProfile, SoloAWP::ladderSpeed);


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
    intake.move(128);
    follower.clearActions();
    follower.startProfile(midringProfile1, false);
    follower.startProfile(midringProfile2, false);
    drivetrain.brake();
    pros::delay(1000);
    follower.startProfile(crossP1Profile, false);
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

void globalBlueGoal() {}

void globalRedGoal() {}

void globalRedRing() {}

void redGoalside() {
/*
    // velocity controller class
    VelocityController follower = VelocityController();

    // profile setup with pregeneration
    MotionProfile* rushProfile = new MotionProfile(&RedMoGoRush::rushProfile, RedMoGoRush::rushSpeed);
    MotionProfile* yoinkProfile = new MotionProfile(&RedMoGoRush::yoinkProfile, RedMoGoRush::yoinkSpeed);
    MotionProfile* backProfile = new MotionProfile(&RedMoGoRush::backProfile, RedMoGoRush::backSpeed);
    MotionProfile* dropProfile = new MotionProfile(&RedMoGoRush::dropProfile, RedMoGoRush::dropSpeed);
    MotionProfile* grabProfile = new MotionProfile(&RedMoGoRush::grabProfile, RedMoGoRush::grabSpeed);
    MotionProfile* cornerProfile = new MotionProfile(&RedMoGoRush::cornerProfile, RedMoGoRush::cornerSpeed);
    MotionProfile* ladderProfile = new MotionProfile(&RedMoGoRush::ladderProfile, RedMoGoRush::ladderSpeed);


    // score on Alliance Stake
    yoin.set_value(true);
    preRoller.move(-128);
    follower.clearActions();
    follower.addAction([](){ker.set_value(true);}, 0.62);
    follower.startProfile(rushProfile, false);
    follower.clearActions();
    // follower.addAction([](){transport.move_relative(-360, 600);}, 0.3);
    follower.addAction([](){ker.set_value(false);}, 0.7);
    // follower.addAction([](){clamp.set_value(true);}, 0.99);
    follower.startProfile(yoinkProfile, true);
    follower.clearActions();
    follower.startProfile(backProfile, true);
    drivetrain.move_relative(-500, 150);
    pros::delay(550);
    clamp.set_value(true);
    pros::delay(200);
    yoin.set_value(false);
    transport.move(-128);
    /*
    follower.startProfile(midringProfile2, false);
    drivetrain.brake();
    pros::delay(1000);
    follower.startProfile(crossP1Profile, false);
    
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

void blueGoalside() {
    // velocity controller class
    VelocityController follower = VelocityController();

    // profile setup with pregeneration
    MotionProfile* rushProfile = new MotionProfile(&BlueMoGoRush::rushProfile, BlueMoGoRush::rushSpeed);
    MotionProfile* yoinkProfile = new MotionProfile(&BlueMoGoRush::yoinkProfile, BlueMoGoRush::yoinkSpeed);
    MotionProfile* backProfile = new MotionProfile(&BlueMoGoRush::backProfile, BlueMoGoRush::backSpeed);
    /*MotionProfile* dropProfile = new MotionProfile(&RedMoGoRush::dropProfile, RedMoGoRush::dropSpeed);
    MotionProfile* grabProfile = new MotionProfile(&RedMoGoRush::grabProfile, RedMoGoRush::grabSpeed);
    MotionProfile* cornerProfile = new MotionProfile(&RedMoGoRush::cornerProfile, RedMoGoRush::cornerSpeed);
    MotionProfile* ladderProfile = new MotionProfile(&RedMoGoRush::ladderProfile, RedMoGoRush::ladderSpeed);*/


    // score on Alliance Stake
    yoin.set_value(true);
    preRoller.move(-128);
    follower.clearActions();
    follower.addAction([](){ker.set_value(true);}, 0.64);
    follower.startProfile(rushProfile, false);
    follower.clearActions();
    // follower.addAction([](){transport.move_relative(-360, 600);}, 0.3);
    follower.addAction([](){ker.set_value(false);}, 0.7);
    // follower.addAction([](){clamp.set_value(true);}, 0.99);
    follower.startProfile(yoinkProfile, true);
    follower.clearActions();
    follower.startProfile(backProfile, true);
    drivetrain.move_relative(-600, 150);
    pros::delay(650);
    clamp.set_value(true);
    pros::delay(200);
    yoin.set_value(false);
    transport.move(-128);
    /*
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

void redRingside() {}

void blueRingside() {}

void autoSkills() {}

void autoTest() {}