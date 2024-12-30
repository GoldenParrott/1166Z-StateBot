#include "profiling.h"
#include "init.h"

VelocityController::VelocityController(double wheelDiameter, double distBetweenWheels, double gearRatio, double maxRPM) {
    this->wheelDiameter = wheelDiameter;
    this->distBetweenWheels = distBetweenWheels;
    this->gearRatio = gearRatio;
    this->maxRPM = maxRPM;
}

// uses the kinematic equations of a differential chassis and unit conversions to convert a linear and angular velocity to something that can be used
// by each side of the drivetrain
std::vector<double> VelocityController::calculateOutputOfSides(double linearVelocityMPS, double angularVelocityRADPS, Direction direction) {
    
    double leftVelocityMPS = linearVelocityMPS - ((angularVelocityRADPS * (this->distBetweenWheels * 0.0254)) / 2); // lv = v - ((w * L) / 2)
    double rightVelocityMPS = linearVelocityMPS + ((angularVelocityRADPS * (this->distBetweenWheels * 0.0254)) / 2); // rv = v + ((w * L) / 2)

    double leftVelocityRPM = (leftVelocityMPS * 60 / (3.141592 * this->wheelDiameter)) / this->gearRatio; // rpm = m/s * (60 s / min) * (1 rotation / (single degree travel * 360))
    double rightVelocityRPM = (rightVelocityMPS * 60 / (3.141592 * this->wheelDiameter)) / this->gearRatio; // rpm = m/s * (60 s / min) * (1 rotation / (single degree travel * 360))

    if (leftVelocityRPM > this->maxRPM) {
        double scaling = maxRPM / leftVelocityRPM;
        leftVelocityRPM *= scaling;
        rightVelocityRPM *= scaling;
    }

    if (rightVelocityRPM > this->maxRPM) {
        double scaling = maxRPM / rightVelocityRPM;
        leftVelocityRPM *= scaling;
        rightVelocityRPM *= scaling;
    }

    if (((leftVelocityRPM > rightVelocityRPM) && (direction == RIGHT)) ||
    ((rightVelocityRPM > leftVelocityRPM) && (direction == LEFT)))
    {
        double rvcache = rightVelocityRPM;
        rightVelocityRPM = leftVelocityRPM;
        leftVelocityRPM = rvcache;
    }

    return {leftVelocityRPM, rightVelocityRPM};
}

// calculates the linear travel of a single degree of movement for a wheel of a given diameter
double VelocityController::calculateSingleDegree(double wheelDiameter) {
    // sets up the odometry to convert angle readings to cm
    double wheelCircumference = 3.141592 * wheelDiameter; // 2 is the pre-measured wheel diameter in inches
	long double singleDegree = wheelCircumference / 360; // the distance that the robot moves in one degree of rotation of its wheels

    return singleDegree;
}

// (private)
void VelocityController::followProfile(MotionProfile* profile)
{
    double currentStep = 0;
    double len = this->queuedProfile->profile.size();
    double step = 1 / len;
    int x = 0;
    // control loop
    while (true) {
        MPPoint currentPoint = profile->findNearestPoint(currentStep);
        std::vector<double> velocitiesRPM = this->calculateOutputOfSides(currentPoint.linVel, currentPoint.angVel, this->queuedProfile->findCurveDirectionOfPoint(currentPoint));

        auto RPMtoMPS = [] (double gearset, double gearRatio, double diameter) {
            return (gearset * gearRatio * (3.14 * diameter)) / 60;
        };
        MPPoint nextPoint = this->queuedProfile->profile[(currentStep + step) * 512];
        double timeAtCurrentVelocity = (calculateDistance({currentPoint.x, currentPoint.y}, {nextPoint.x, nextPoint.y}) / 100) / ((RPMtoMPS(velocitiesRPM[0], gearRatio, wheelDiameter) + (RPMtoMPS(velocitiesRPM[1], gearRatio, wheelDiameter))) / 2);

        for (int i = 0; i < actions.size(); i++) {
            if (currentPoint.t == actionTs[i]) {
                actions[i]();
            }
        }

        leftDrivetrain.move_velocity(velocitiesRPM[0]);
        rightDrivetrain.move_velocity(velocitiesRPM[1]);


        std::cout << "lrpm = " << velocitiesRPM[0] << ", rrpm = " << velocitiesRPM[1] << "\n";
        std::cout << "x = " << currentPoint.x << ", y = " << currentPoint.y << "\n";
        std::cout << "step = " << currentStep << "\n";
        std::cout << " lvel = " << currentPoint.linVel << ", avel = " << currentPoint.angVel << "\n\n";
        
        pros::delay(timeAtCurrentVelocity / 1000);

        currentStep += step;

        if (currentPoint.t == 1) {
            drivetrain.brake();
            return;
        }
    }
}

// starts the filter loop if it is not already active (public)
void VelocityController::startQueuedProfile() {

    auto filterLoopFunction = [this] () {return this->followProfile(this->queuedProfile);};

    if (controlLoop_task_ptr == NULL) {
        controlLoop_task_ptr = new pros::Task(filterLoopFunction);
    }
}

// completely ends the filter loop if it is active (public)
void VelocityController::endProfile() {
    if (controlLoop_task_ptr != NULL) {
        controlLoop_task_ptr->remove();
        controlLoop_task_ptr = NULL;
    }
}

void VelocityController::queueProfile(MotionProfile* profile) {
    this->queuedProfile = profile;
}

void VelocityController::addAction(std::function<void(void)> action, double time) {
    double t = time / this->timeToRun;
    double actionT = this->queuedProfile->findNearestPoint(t).t;
    this->actions.push_back(action);
    this->actionTs.push_back(actionT);
}