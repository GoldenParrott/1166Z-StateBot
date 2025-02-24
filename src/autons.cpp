#include "init.h"
#include "profiling.h"

void BlueAWP() {

    VelocityController follower = VelocityController();

    // spline setup
    CubicHermiteSpline goalSpline = CubicHermiteSpline({-57.2, 10.6}, {-28.7, 45}, {-35.14, 29.54}, {-11.6, 17.9});
    CubicHermiteSpline innerRingSpline = CubicHermiteSpline({-20.54, 22}, {-35.07, 29.36}, {-23.71, 44.84}, {-22.6, 91.2});
    CubicHermiteSpline outerRingSpline = CubicHermiteSpline({-23.7, 44.85}, {-22.6, 91.4}, {-8.42, 40.2}, {-17.8, -41});
    CubicHermiteSpline crossSpline = CubicHermiteSpline({-8.421, 40.198}, {-14.68, 21.02}, {-59.2, -7.8}, {-81.8, 6.5});
    // profile setup
    MotionProfile* goalProfile = new MotionProfile(&goalSpline, 400);
    MotionProfile* innerRingProfile = new MotionProfile(&innerRingSpline, 600);
    MotionProfile* outerRingProfile = new MotionProfile(&outerRingSpline, 400);
    MotionProfile* crossProfile = new MotionProfile(&crossSpline, 600);

    // scores on Alliance Stake
    arm.move(128);
    waitUntil(ArmRotational.get_position() < -18000);
    arm.move(-128);
    waitUntil(ArmRotational.get_position() > 1000);
    arm.brake();

    // moves to goal and grabs it
    follower.startProfile(goalProfile, true);
    PIDMover({-23.7, 44.85}, true, {[](){clamp.set_value(true);}}, {14});

    // moves to inner Ring stack and takes the correct Ring
    intake.move(-128);
    follower.startProfile(innerRingProfile);

    // moves to middle Ring stack and takes those Rings
    follower.startProfile(outerRingProfile);

    // moves all the way to the other side of the field, grabbing the middle Ring and dropping the first MoGo
    follower.addAction([](){clamp.set_value(false);}, 0.7);
    follower.startProfile(crossProfile);
    follower.clearActions();

    // moves to the other MoGo and grabs it
    PIDMover({-20, -25}, true, {[](){clamp.set_value(true);}}, {38});

    // moves to the other inner Ring and grabs it
    PIDTurner(190, 2);
    intake.move(128);
    PIDMover({-24, -48});

    // raises arm and moves to ladder to contact it
    arm.move(128);
    waitUntil(ArmRotational.get_position() < -5000);
    arm.brake();
    PIDMover({-18, -13.5}, true);
}

void RedAWP() {

    VelocityController follower = VelocityController();

    // spline setup
    CubicHermiteSpline goalSpline = CubicHermiteSpline({57.2, 10.6}, {28.7, 45}, {35.14, 29.54}, {11.6, 17.9});
    CubicHermiteSpline innerRingSpline = CubicHermiteSpline({20.54, 22}, {35.07, 29.36}, {23.71, 44.84}, {22.6, 91.2});
    CubicHermiteSpline outerRingSpline = CubicHermiteSpline({23.7, 44.85}, {22.6, 91.4}, {8.42, 40.2}, {17.8, -41});
    CubicHermiteSpline crossSpline = CubicHermiteSpline({8.421, 40.198}, {14.68, 21.02}, {59.2, -7.8}, {81.8, 6.5});
    // profile setup
    MotionProfile* goalProfile = new MotionProfile(&goalSpline, 400);
    MotionProfile* innerRingProfile = new MotionProfile(&innerRingSpline, 600);
    MotionProfile* outerRingProfile = new MotionProfile(&outerRingSpline, 400);
    MotionProfile* crossProfile = new MotionProfile(&crossSpline, 600);

    // scores on Alliance Stake
    arm.move(128);
    waitUntil(ArmRotational.get_position() < -18000);
    arm.move(-128);
    waitUntil(ArmRotational.get_position() > 1000);
    arm.brake();

    // moves to goal and grabs it
    follower.startProfile(goalProfile, true);
    PIDMover({-23.7, 44.85}, true, {[](){clamp.set_value(true);}}, {14});

    // moves to inner Ring stack and takes the correct Ring
    intake.move(-128);
    follower.startProfile(innerRingProfile);

    // moves to middle Ring stack and takes those Rings
    follower.startProfile(outerRingProfile);

    // moves all the way to the other side of the field, grabbing the middle Ring and dropping the first MoGo
    follower.addAction([](){clamp.set_value(false);}, 0.7);
    follower.startProfile(crossProfile);
    follower.clearActions();

    // moves to the other MoGo and grabs it
    PIDMover({20, -25}, true, {[](){clamp.set_value(true);}}, {38});

    // moves to the other inner Ring and grabs it
    PIDTurner(170, 1);
    intake.move(128);
    PIDMover({24, -48});

    // raises arm and moves to ladder to contact it
    arm.move(128);
    waitUntil(ArmRotational.get_position() < -5000);
    arm.brake();
    PIDMover({18, -13.5}, true);
}

void RedGoalRush() {
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

void BlueGoalRush() {
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

    arm.move(128);
    waitUntil(ArmRotational.get_position() < -5000);
	arm.brake();
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

}