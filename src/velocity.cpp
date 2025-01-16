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

    std::cout << "lv = " << leftVelocityRPM << ", rv = " << rightVelocityRPM << "\n";

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

    std::cout << "alv = " << leftVelocityRPM << ", arv = " << rightVelocityRPM << "\n";

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
void VelocityController::followProfile(MotionProfile currentlyFollowing, bool RAMSETE)
{
    // step-related variables
    double currentStep = 0;
    double step = 1 / (double) currentlyFollowing.profile.size();
    double sum = 0;
    // point variables
    MPPoint currentPoint = {0, 0, 0, 0, 0, 0};
    MPPoint nextPoint = {0, 0, 0, 0, 0, 0};
    // speed variables
    std::vector<double> velocitiesRPM = {0, 0};
    // clock variables
    uint32_t timeSinceStartOfLoop;
    uint32_t startTime = pros::millis();
    // action variables
    std::vector<bool> actionCompleteds(3, false);

    // control loop
    while (true) {
    
        // begins a timer to ensure that the calculation time is subtracted from the delay
        timeSinceStartOfLoop = pros::micros();

        // calculates the current point as the nearest point to the current step
        currentPoint = currentlyFollowing.findNearestPoint(currentStep);
        //std::cout << currentPoint.t << "\n";

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
        // executes custom actions if the profile has reached or passed their t-point and have not yet been activated
        for (int i = 0; i < actions.size(); i++) {
            if ((currentPoint.t >= actionTs[i]) && actionCompleteds[i]) {
                actions[i]();
            }
        }

        // converts the velocity in rpm to velocity in millivolts
        int maxVoltage = 12000; // innate max voltage of motors (in mV)
        double rpmToV = maxVoltage / this->maxRPM; // multiplier to convert rpm to voltage (in units of millivoltage / rpm so multiplying it by rpm cancels to millivoltage)

        // sends the output voltage to the motors
        leftDrivetrain.move_voltage(velocitiesRPM[0] * rpmToV);
        rightDrivetrain.move_voltage(velocitiesRPM[1] * rpmToV);

        // LOGGING FOR TEST PURPOSES
        //std::cout << timeAtCurrentVelocity << "\n";
        //std::cout << "lvol = " << velocitiesRPM[0] * rpmToV << ", rvol = " << velocitiesRPM[1] * rpmToV << "\n";
        //std::cout << "lrpm = " << velocitiesRPM[0] << ", rrpm = " << velocitiesRPM[1] << "\n";
        //std::cout << "x = " << currentPoint.x << ", y = " << currentPoint.y << "\n";
        //std::cout << "step = " << currentStep << "\n";
        //std::cout << " lvel = " << currentPoint.linVel << ", avel = " << currentPoint.angVel << "\n\n";
        // std::cout << "ct = " << currentPoint.t << ", nt = " << nextPoint.t << "\n";
        //logfile.appendFile("x = " + std::to_string(universalCurrentLocation.x) + ", should be " + std::to_string(currentPoint.x) + "\n");
        //logfile.appendFile("y = " + std::to_string(universalCurrentLocation.y) + ", should be " + std::to_string(currentPoint.y) + "\n");
        //logfile.appendFile("h = " + std::to_string(getAggregatedHeading(Kalman1, Kalman2)) + ", should be " + std::to_string(currentPoint.heading));
        //logfile.appendFile("lvol = " + std::to_string(leftDrivetrain.get_voltage()) + ", should be " + std::to_string(velocitiesRPM[0] * rpmToV) + "\n" + "rvol = " + std::to_string(rightDrivetrain.get_voltage()) + ", should be " + std::to_string(velocitiesRPM[1] * rpmToV) + "\n\n");
        //std::cout << "t = " << std::to_string(currentPoint.t) << ", time = " << std::to_string(((double) pros::millis() - (double) startTime) / 1000) << "\n";
        //std::cout << (pros::micros() - timeTrack) / 1000 << " is calc\n";
        //std::cout << timeAtCurrentVelocity * 1000 << " is delay\n";
        
        // computes the calculation time using the timer and converts it to ms
        double calcTime = (pros::micros() - timeSinceStartOfLoop) / 1000;

        // 5 ms delay (- the time taken to calculate)
        pros::delay(5 - calcTime);

        // if the current step is the final point (t = 1 - step), 
        // then the drivetrain is stopped and the function ends
        if (currentStep >= 1 - step) {
            drivetrain.brake();
            //std::cout << (pros::millis() - startTime) / (double) 1000 << "\n";
            //std::cout << currentlyFollowing.totalTime << "\n";
            return;
        }

        // goes to the next step of the function if it did not end
        currentStep += step;
    }
}

// starts the filter loop if it is not already active (public)
void VelocityController::startQueuedProfile(bool RAMSETE) {

    // auto controlLoopFunction = [this, path, RAMSETE] () {return this->followProfile(*this->queuedProfile, path, RAMSETE);};

    if (controlLoop_task_ptr == NULL) {
        // pros::Task* controlLoop_task_ptr = new pros::Task(controlLoopFunction);
        this->followProfile(*this->queuedProfile, RAMSETE);
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