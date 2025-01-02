#include "init.h"
#include "profiling.h"

void globalBlueRing() {}

void globalBlueGoal() {}

void globalRedGoal() {}

void globalRedRing() {}

void redGoalside() {}

void blueGoalside() {}

void redRingside() {}

void blueRingside() {}

void autoSkills() {}

void autoTest() {
    auto RPMtoMPS = [] (double gearset, double gearRatio, double diameter) {
        return (gearset * gearRatio * (3.14 * diameter)) / 60;
    };

    // ROBOT CONFIG
    double gearRatio = 0.75;
    double maxRPM = 600;
    double diameter = 3.25;
    double distBetweenWheels = 10.5;


    double numPoints = 1000;

    double maxSpeed = RPMtoMPS(maxRPM, gearRatio, diameter); // in meters per second


    
    CubicHermiteSpline mySpline = CubicHermiteSpline({0, 0}, {0, -1}, {0.5, -0.5}, {1, -0.5});
    MotionProfile* myProfile = new MotionProfile(mySpline.entirePath(numPoints), maxSpeed);
    VelocityController myController = VelocityController(diameter, distBetweenWheels, gearRatio, maxRPM);
    myController.queueProfile(myProfile);
    myController.startQueuedProfile(false);
}