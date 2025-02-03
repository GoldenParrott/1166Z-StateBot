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
    MotionProfile* rushProfile = new MotionProfile(&RedMoGoPath::rushProfile, RedMoGoPath::rushSpeed);
    MotionProfile* yoinkProfile = new MotionProfile(&RedMoGoPath::yoinkProfile, RedMoGoPath::yoinkSpeed);
    MotionProfile* dropProfile = new MotionProfile(&RedMoGoPath::dropProfile, RedMoGoPath::dropSpeed);
    MotionProfile* grabProfile = new MotionProfile(&RedMoGoPath::grabProfile, RedMoGoPath::grabSpeed);
    MotionProfile* cornerProfile = new MotionProfile(&RedMoGoPath::cornerProfile, RedMoGoPath::cornerSpeed);
    MotionProfile* ladderProfile = new MotionProfile(&RedMoGoPath::ladderProfile, RedMoGoPath::ladderSpeed);

    // profile following
    //yoin.set_value(true);
    //follower.startProfile(rushProfile);
    //follower.startProfile(yoinkProfile);
    follower.startProfile(dropProfile);
    //follower.startProfile(grabProfile);
    //follower.startProfile(cornerProfile);
    //follower.startProfile(ladderProfile);
}

void blueGoalside() {}

void redRingside() {}

void blueRingside() {}

void autoSkills() {}

void autoTest() {}