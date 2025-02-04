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

void autoTest() {

    // velocity controller class
    VelocityController follower = VelocityController();

    // profile setup with pregeneration
    MotionProfile* goalProfile = new MotionProfile(&SoloAWP::goalProfile, SoloAWP::goalSpeed);
    MotionProfile* midringProfile = new MotionProfile(&SoloAWP::midringProfile, SoloAWP::midringSpeed);
    MotionProfile* crossP1Profile = new MotionProfile(&SoloAWP::crossP1Profile, SoloAWP::crossP1Speed);
    MotionProfile* crossP2Profile = new MotionProfile(&SoloAWP::crossP2Profile, SoloAWP::crossP2Speed);
    MotionProfile* goal2Profile = new MotionProfile(&SoloAWP::goal2Profile, SoloAWP::goal2Speed);
    MotionProfile* ring2Profile = new MotionProfile(&SoloAWP::ring2Profile, SoloAWP::ring2Speed);
    MotionProfile* ladderProfile = new MotionProfile(&SoloAWP::ladderProfile, SoloAWP::ladderSpeed);

    // profile following
    follower.startProfile(goalProfile, true);
    pros::delay(500);
    follower.startProfile(midringProfile, false);
    pros::delay(500);
    follower.startProfile(crossP1Profile, false);
    pros::delay(500);
    follower.startProfile(crossP2Profile, false);
    pros::delay(500);
    follower.startProfile(goal2Profile, true);
    pros::delay(500);
    follower.startProfile(ring2Profile, false);
    pros::delay(500);
    follower.startProfile(ladderProfile, true);

}