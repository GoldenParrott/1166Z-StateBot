#include "init.h"
#include "paths.h"
#include "profiling.h"

void globalBlueRing() {}

void globalBlueGoal() {}

void globalRedGoal() {}

void globalRedRing() {}

void redGoalside() {

    // velocity controller class
    VelocityController follower = VelocityController();

    // profile setup with pregeneration
    /*
    MotionProfile* rushProfile = new MotionProfile(&RedMoGoPath::rushProfile, RedMoGoPath::rushSpeed);
    MotionProfile* yoinkProfile = new MotionProfile(&RedMoGoPath::yoinkProfile, RedMoGoPath::yoinkSpeed);
    MotionProfile* dropProfile = new MotionProfile(&RedMoGoPath::dropProfile, RedMoGoPath::dropSpeed);
    MotionProfile* grabProfile = new MotionProfile(&RedMoGoPath::grabProfile, RedMoGoPath::grabSpeed);
    MotionProfile* cornerProfile = new MotionProfile(&RedMoGoPath::cornerProfile, RedMoGoPath::cornerSpeed);
    MotionProfile* ladderProfile = new MotionProfile(&RedMoGoPath::ladderProfile, RedMoGoPath::ladderSpeed);
    */

    // profile following
    //yoin.set_value(true);
    //follower.startProfile(rushProfile);
    //follower.startProfile(yoinkProfile);
    //follower.startProfile(dropProfile);
    //follower.startProfile(grabProfile);
    //follower.startProfile(cornerProfile);
    //follower.startProfile(ladderProfile);
}

void blueGoalside() {}

void redRingside() {}

void blueRingside() {}

void autoSkills() {}

/*
void autoTest() {

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
    

}
*/

void autoTest() {

        // velocity controller class
    VelocityController follower = VelocityController();

    // profile setup with pregeneration
    MotionProfile* rushProfile = new MotionProfile(&RedMoGoRush::rushProfile, RedMoGoRush::rushSpeed);
    MotionProfile* yoinkProfile = new MotionProfile(&RedMoGoRush::yoinkProfile, RedMoGoRush::yoinkSpeed);
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
    follower.addAction([](){transport.move_relative(-270, 600);}, 0.3);
    follower.addAction([](){ker.set_value(false);}, 0.5);
    follower.addAction([](){clamp.set_value(true);}, 0.99);
    follower.startProfile(yoinkProfile, true);
    follower.clearActions();
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