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
    waitUntil(arm.get_position() < -180);
    arm.move(-128);
    waitUntil(arm.get_position() > 10);
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
    waitUntil(arm.get_position() < -50);
    arm.brake();
    PIDMover({-18, -13.5}, true);
}

void RedAWP() {

    VelocityController follower = VelocityController();

    // spline setup
    // CubicHermiteSpline goalSpline = CubicHermiteSpline({-54.75, 13.25}, {28.7, 45}, {35.14, 29.54}, {11.6, 17.9});
    CubicHermiteSpline goalSpline = CubicHermiteSpline({-54.75, 13.25}, {10, 36}, {-20, 25}, {10, 36});
    CubicHermiteSpline innerRingSpline = CubicHermiteSpline({-20, 25}, {-25, 51}, {-23, 42}, {-29, 83});
    CubicHermiteSpline outerRingSpline = CubicHermiteSpline({-23, 42}, {-34, 77}, {-50, 39}, {-49, 20});
    CubicHermiteSpline crossSpline = CubicHermiteSpline({-49.5, 39}, {-50, 20}, {-50, -12}, {-43, -41.5});
    CubicHermiteSpline southernRingSpline = CubicHermiteSpline({-50, -12}, {-43, -81.5}, {-23, -51}, {-23.5, -90});
    CubicHermiteSpline ladderSpline = CubicHermiteSpline({-23, -51}, {-24, -32.5}, {-23.5, -25.2}, {-21, 6.5});
    CubicHermiteSpline ladder2Spline = CubicHermiteSpline({-23.5, -25.2}, {-22.06, -5.33}, {-11, -15}, {-2, -13});

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
            {{0.5, 1}, {0.6, 0.8}},
            {{0.9, 0.8}, {1, 0}}
        }
    );
    MotionProfile* southernRingProfile = new MotionProfile(&southernRingSpline, RPMtoIPS(600));
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
    waitUntil(arm.get_position() < 180);
    arm.brake();

    // moves to goal and grabs it
    PIDTurner(253, 2);
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
    follower.addAction([](){clamp.set_value(false); transport.brake();}, 0.6);
    follower.startProfile(southernRingProfile);
    follower.clearActions();

    // grabs the third MoGo and touches the Ladder with it
    follower.addAction([](){clamp.set_value(true); intake.move(128);}, 0.98);
    follower.startProfile(ladderProfile, true);
    follower.clearActions();
    pros::delay(1000);
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

void autoSkills() {
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
}

void autoTest() {

}