#include "init.h"

VelocityController::VelocityController(double wheelDiameter, double distBetweenWheels, double gearRatio, double maxRPM) {
    this->wheelDiameter = wheelDiameter;
    this->distBetweenWheels = distBetweenWheels;
    this->gearRatio = gearRatio;
    this->maxRPM = maxRPM;
    this->actions = {[](){master.rumble(".");pros::lcd::print(0, "0.25");}, [](){master.rumble(".");pros::lcd::print(1, "0,5");}, [](){master.rumble(".");pros::lcd::print(2, "0.9");}};
    this->actionTs = {0.25, 0.5, 0.9};
}

// uses the kinematic equations of a differential chassis and unit conversions to convert a linear and angular velocity to something that can be used
// by each side of the drivetrain
std::vector<double> VelocityController::calculateOutputOfSides(double linearVelocityIPS, double angularVelocityRADPS, Direction direction) {
    
    double leftVelocityIPS = linearVelocityIPS - ((angularVelocityRADPS * this->distBetweenWheels) / 2); // lv = v - ((w * L) / 2)
    double rightVelocityIPS = linearVelocityIPS + ((angularVelocityRADPS * this->distBetweenWheels) / 2); // rv = v + ((w * L) / 2)

    double leftVelocityRPM = (leftVelocityIPS * 60 / (M_PI * this->wheelDiameter)) / this->gearRatio; // rpm = m/s * (60 s / min) * (1 rotation / (single degree travel * 360))
    double rightVelocityRPM = (rightVelocityIPS * 60 / (M_PI * this->wheelDiameter)) / this->gearRatio; // rpm = m/s * (60 s / min) * (1 rotation / (single degree travel * 360))
    
    double profileMax = IPStoRPM(this->queuedProfile->maxSpeed, gearRatio, wheelDiameter);

    if (leftVelocityRPM > profileMax) {
        double scaling = profileMax / leftVelocityRPM;
        leftVelocityRPM *= scaling;
        rightVelocityRPM *= scaling;
    }

    if (rightVelocityRPM > profileMax) {
        double scaling = profileMax / rightVelocityRPM;
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
    double wheelCircumference = M_PI * wheelDiameter; // 2 is the pre-measured wheel diameter in inches
	long double singleDegree = wheelCircumference / 360; // the distance that the robot moves in one degree of rotation of its wheels

    return singleDegree;
}

// (private)
void VelocityController::followProfile(MotionProfile currentlyFollowing, CubicHermiteSpline path, bool RAMSETE)
{
    // step-related variables
    double currentStep = 0;
    double len = currentlyFollowing.profile.size();
    double step = 1 / len;
    double distStep = 0.001;
    double pointDist = 0;
    int currentFrac = 0;
    int x = 0;
    double sum = 0;
    int count = 0;
    // point variables
    MPPoint currentPoint = {0, 0, 0, 0, 0, 0};
    MPPoint nextPoint = {0, 0, 0, 0, 0, 0};
    // speed variables
    std::vector<double> velocitiesRPM = {0, 0};
    uint32_t timeSinceStartOfLoop;
    uint32_t startTime = pros::millis();

    // control loop
    while (true) {
    
        timeSinceStartOfLoop = pros::micros();

        currentPoint = currentlyFollowing.findNearestPoint((((double) pros::millis() - (double) startTime) / 1000) / currentlyFollowing.totalTime);
        std::cout << currentPoint.t << "\n";
        currentStep = currentPoint.t;

        // standard calculation of output of each side based on specifications of the motion profile
        velocitiesRPM = this->calculateOutputOfSides(currentPoint.linVel, currentPoint.angVel, currentlyFollowing.findCurveDirectionOfPoint(currentPoint));

        // calculation of output of each side with error corrections from RAMSETE
        if (RAMSETE) {
            fixAngle(currentPoint.heading);
        // rotation of the x, y, and heading errors to fit the local frame
            Pose error;
            error.x = (std::cos(currentPoint.heading) * (currentPoint.x - universalCurrentLocation.x)) + (std::sin(currentPoint.heading) * (currentPoint.y - universalCurrentLocation.y));
            error.y = (-1 * std::sin(currentPoint.heading) * (currentPoint.x - universalCurrentLocation.x) + (std::cos(currentPoint.heading) * (currentPoint.y - universalCurrentLocation.y)));
            error.heading = currentPoint.heading - getAggregatedHeading(Kalman1, Kalman2);

        // tuning constants (current values are from the widely accepted defaults from FTCLib)
            double b = 2.0; // this is a proportional gain for each of the different error elements of the controller (put into the gain value calculations)
            double zeta = 0.6; // this is a dampener for the direct movements (k1 and k3/x-value and heading-value)

        // gain values that serve as scaling multipliers for the outputs based on the profile's velocity and pre-set constants
            // k1/k3: the proportional gain value for both the local frame of x and heading
            double k = 2 * zeta * std::sqrt(std::pow(currentPoint.angVel, 2) + (b * std::pow(currentPoint.linVel, 2)));
            // k2: the gain value for the y-value, which the robot cannot move directly on and is thus handled differently 
            double k2 = b * currentPoint.linVel;

        // control inputs that determine the controller's influence on the velocity based on the error
            // simply the normal gain value multiplied by the x-error because that can be directly moved upon (input for linear velocity)
            double u1 = -1 * k * error.x;
            // the special gain value is used for the y-value, and it is also scaled with the part in purple parenthesis to let it switch directions smoothly if it is past
            // its goal point; the second part is the same simple direct angular movement for the error in heading that is used in the same way with the x-value for linear
            // movement
            double u2 = (-1 * k2 * (std::sin(error.heading) / error.heading) * error.y) + (-1 * k * error.heading);

        // actual calculations of modified linear and angular velocities as additions/subtractions to the profile's original values
            // the original linear velocity is also transformed to follow the robot's error in heading before having the control input subtracted from it
            double linVel = (currentPoint.linVel * std::cos(error.heading)) - u1;
            // the angular velocity does not need to be transformed in the same way that the linear velocity needs to because it is already angular in reference to the robot
            double angVel = currentPoint.angVel - u2;
        }
        for (int i = 0; i < actions.size(); i++) {
            if (currentPoint.t == actionTs[i]) {
                actions[i]();
            }
        }
        MPPoint nextPoint = this->queuedProfile->findNearestPoint(currentStep + step);
        if (currentPoint.t == (1 - step)) {
            nextPoint = {path.findPoint(1).x, path.findPoint(1).y, 0, 0, 0, 1};
        }
        pointDist = 0;
        for (double t = currentPoint.t; (nextPoint.t - t) > 0.000001; t += distStep) {
            double nextT = t + distStep;
            pointDist += calculateDistance(path.findPoint(t), path.findPoint(nextT));
            // std::cout << "dist = " << calculateDistance(path.findPoint(t), path.findPoint(nextT)) << ", cx = " << path.findPoint(t).x << ", cy = " << path.findPoint(t).y << ", ct = " << t << ", nx = " << path.findPoint(nextT).x << ", ny = " << path.findPoint(nextT).y << ", nt = " << (nextT) << "\n";
        }
        sum += pointDist;
        double timeAtCurrentVelocity = pointDist / currentPoint.linVel;

        int maxVoltage = 12000; // innate max voltage of motors (in mV)
        double rpmToV = maxVoltage / this->maxRPM; // multiplier to convert rpm to voltage (in units of voltage / rpm so multiplying it by rpm cancels to voltage)
        leftDrivetrain.move_voltage(velocitiesRPM[0] * rpmToV);
        rightDrivetrain.move_voltage(velocitiesRPM[1] * rpmToV);

        //std::cout << timeAtCurrentVelocity << "\n";
        //std::cout << "lvol = " << velocitiesRPM[0] * rpmToV << ", rvol = " << velocitiesRPM[1] * rpmToV << "\n";
        //std::cout << "lrpm = " << velocitiesRPM[0] << ", rrpm = " << velocitiesRPM[1] << "\n";
        //std::cout << "x = " << currentPoint.x << ", y = " << currentPoint.y << "\n";
        //std::cout << "step = " << currentStep << "\n";
        //Sstd::cout << " lvel = " << currentPoint.linVel << ", avel = " << currentPoint.angVel << "\n\n";
        //std::cout << "ct = " << currentPoint.t << ", nt = " << nextPoint.t << "\n";
        //logfile.appendFile("x = " + std::to_string(universalCurrentLocation.x) + ", should be " + std::to_string(currentPoint.x) + "\n");
        //logfile.appendFile("y = " + std::to_string(universalCurrentLocation.y) + ", should be " + std::to_string(currentPoint.y) + "\n");
        //logfile.appendFile("h = " + std::to_string(getAggregatedHeading(Kalman1, Kalman2)) + ", should be " + std::to_string(currentPoint.heading));
        //logfile.appendFile("lvol = " + std::to_string(leftDrivetrain.get_voltage()) + ", should be " + std::to_string(velocitiesRPM[0] * rpmToV) + "\n" + "rvol = " + std::to_string(rightDrivetrain.get_voltage()) + ", should be " + std::to_string(velocitiesRPM[1] * rpmToV) + "\n\n");
        //std::cout << (pros::micros() - timeTrack) / 1000 << " is calc\n";
        //std::cout << timeAtCurrentVelocity * 1000 << " is delay\n";
        int calcTime = pros::micros() - timeSinceStartOfLoop;
        double totalDelay = ((timeAtCurrentVelocity * 1000) - (calcTime / 1000)) * 1.25;
        int delay = std::round(totalDelay);
        if (delay < 5) {delay = 5;}

        //std::cout << delay << "\n";

        pros::delay(delay);

        if (currentStep >= 1 - step) {
            drivetrain.brake();
            std::cout << sum << "\n";
            return;
        }

        currentStep += step;
    }
}

// starts the filter loop if it is not already active (public)
void VelocityController::startQueuedProfile(CubicHermiteSpline path, bool RAMSETE) {

    // auto controlLoopFunction = [this, path, RAMSETE] () {return this->followProfile(*this->queuedProfile, path, RAMSETE);};

    if (controlLoop_task_ptr == NULL) {
        // pros::Task* controlLoop_task_ptr = new pros::Task(controlLoopFunction);
        this->followProfile(*this->queuedProfile, path, RAMSETE);
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