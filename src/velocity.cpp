#include "init.h"

VelocityController::VelocityController(std::vector<std::function<void(void)>> actions, std::vector<double> actionTs) {
    //this->actions = {[](){master.rumble(".");pros::lcd::print(0, "0.25");}, [](){master.rumble(".");pros::lcd::print(1, "0,5");intake.move(128);}, [](){master.rumble(".");pros::lcd::print(2, "0.9");}};
    this->actions = actions;
    this->actionTs = actionTs;
}

// uses the kinematic equations of a differential chassis and unit conversions to convert a linear and angular velocity to something that can be used
// by each side of the drivetrain
std::vector<double> VelocityController::calculateOutputOfSides(double linearVelocityIPS, double angularVelocityRADPS, double profileMaxIPS) {
    
    double leftVelocityIPS = linearVelocityIPS - ((angularVelocityRADPS * g_distBetweenWheels) / 2); // lv = v - ((w * L) / 2)
    double rightVelocityIPS = linearVelocityIPS + ((angularVelocityRADPS * g_distBetweenWheels) / 2); // rv = v + ((w * L) / 2)

    double leftVelocityRPM = (leftVelocityIPS * 60 / (M_PI * g_diameter)) / g_gearRatio; // rpm = m/s * (60 s / min) * (1 rotation / (single degree travel * 360))
    double rightVelocityRPM = (rightVelocityIPS * 60 / (M_PI * g_diameter)) / g_gearRatio; // rpm = m/s * (60 s / min) * (1 rotation / (single degree travel * 360))
    
    double profileMax = IPStoRPM(profileMaxIPS);

    if (leftVelocityRPM > profileMax) {
        double scaling = profileMax / leftVelocityRPM;
        leftVelocityRPM *= scaling;
        rightVelocityRPM *= scaling;
    } else if (leftVelocityRPM < -profileMax) {
        double scaling = -profileMax / leftVelocityRPM;
        leftVelocityRPM *= scaling;
        rightVelocityRPM *= scaling;
    }

    if (rightVelocityRPM > profileMax) {
        double scaling = profileMax / rightVelocityRPM;
        leftVelocityRPM *= scaling;
        rightVelocityRPM *= scaling;
    } else if (rightVelocityRPM < -profileMax) {
        double scaling = -profileMax / rightVelocityRPM;
        leftVelocityRPM *= scaling;
        rightVelocityRPM *= scaling;
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
void VelocityController::followProfile(MotionProfile* currentlyFollowing, bool RAMSETE, bool reverse)
{
    // step-related variables
    double currentStep = 0;
    double step = 1 / (double) currentlyFollowing->profile.size();
    double sum = 0;
    // distance calculation variables

    // point variables
    MPPoint currentPoint = {0, 0, 0, 0, 0, 0};
    MPPoint nextPoint = {0, 0, 0, 0, 0, 0};
    // speed variables
    std::vector<double> velocitiesRPM = {0, 0};
    // clock variables
    uint32_t timeSinceStartOfLoop;
    uint32_t startTime = pros::millis();
    double delay;
    // action variables
    std::vector<bool> actionCompleteds = {false, false, false, false, false, false};

    // control loop
    while (true) {
        sum++;
        // begins a timer to ensure that the calculation time is subtracted from the delay
        timeSinceStartOfLoop = pros::micros();

        // while RAMSETE is not on, the delay is always 5 ms
        delay = 5;

        // calculates the current point as the nearest point to the current step
        currentPoint = currentlyFollowing->findNearestPoint(currentStep);
        //std::cout << currentPoint.t << "\n";

        // sets linear and angular velocities to that of the current point - these are changed by RAMSETE if it is on
        double linVel = currentPoint.linVel;
        double angVel = currentPoint.angVel;

        // std::cout << "lv = " << linVel << ", rv = " << angVel << "\n";

        // calculation of output of each side with error corrections from RAMSETE
        if (RAMSETE) {
            Pose location = {universalCurrentLocation.x * 0.0254, universalCurrentLocation.y * 0.0254, universalCurrentLocation.heading};
            nextPoint = currentlyFollowing->findNearestPoint(currentStep + step);
            nextPoint = {nextPoint.x * 0.0254, nextPoint.y * 0.0254, nextPoint.heading};

            if (reverse) {
                linVel *= -1;
                            
            std::cout << "odom = " << fixAngle(location.heading) << "\n";
            std::cout << "next = " << fixAngle(nextPoint.heading) << "\n";
            std::cout << "diff = " << fixAngle(nextPoint.heading) - fixAngle(location.heading) << "\n\n";

                if (location.heading > 180) {
                    location.heading -= 180;
                } else {
                    location.heading += 180;
                }
       
            std::cout << "fodom = " << fixAngle(location.heading) << "\n";
            std::cout << "fnext = " << fixAngle(nextPoint.heading) << "\n";
            std::cout << "fdiff = " << fixAngle(nextPoint.heading) - fixAngle(location.heading) << "\n\n";

            }

            double fixedOdomAngle = fixAngle(location.heading) * (M_PI / 180);
            double fixedNextAngle = fixAngle(nextPoint.heading) * (M_PI / 180);



        //textToWrite.push_back("odom = " + std::to_string(location.heading) + ", fodom = " + std::to_string(fixedOdomAngle) + "\n");
        //textToWrite.push_back("next = " + std::to_string(nextPoint.heading) + ", fnext = " + std::to_string(fixedNextAngle) + "\n\n");
        // rotation of the x, y, and heading errors to fit the local frame
            Pose error;
            error.x = (std::cos(fixedOdomAngle) * (nextPoint.x - location.x)) + (std::sin(fixedOdomAngle) * (nextPoint.y - location.y));
            if (reverse) {error.x *= -1;}
            error.y = (std::cos(fixedOdomAngle) * (nextPoint.y - location.y)) - (std::sin(fixedOdomAngle) * (nextPoint.x - location.x));
            error.heading = fixedNextAngle - fixedOdomAngle;
            // std::cout << error.heading << "\n";

            linVel *= 0.0254;

            // bounds the error from 0-180 to prevent the correction from being an un-optimal turn direction (where going the other way would be faster)
            if (error.heading < -M_PI) {
                error.heading = error.heading + (2 * M_PI);
            } else if (error.heading > M_PI) {
                error.heading = (2 * M_PI) - error.heading;
            }

            //std::cout << "prp = " << fixedNextAngle << ", ap = " << fixedOdomAngle << ", c = " << error.heading << "\n";
            //std::cout << "ucl = " << location.heading << ", actual = " << getAggregatedHeading(Kalman1, Kalman2) << ", used = " << fixedOdomAngle << "\n";

        // tuning constants (current values are from the widely accepted defaults from FTCLib)
            double b = 2.0; // this is a proportional gain for each of the different error elements of the controller (put into the gain value calculations)
            double zeta = 0.7; // this is a dampener for the direct movements (k1 and k3/x-value and heading-value)

        // gain values that serve as scaling multipliers for the outputs based on the profile's velocity and pre-set constants
            // k1/k3: the proportional gain value for both the local frame of x and heading
            double k = 2 * zeta * std::sqrt(std::pow(angVel, 2) + (b * std::pow(linVel, 2)));
            // k2: the gain value for the y-value, which the robot cannot move directly on and is thus handled differently 
            double k2 = b * currentPoint.linVel;

        // control inputs that determine the controller's influence on the velocity based on the error
            // simply the normal gain value multiplied by the x-error because that can be directly moved upon (input for linear velocity)
            double u1 = k * error.x;
            // the special gain value is used for the y-value, and it is also scaled with the part in purple parenthesis to let it switch directions smoothly if it is past
            // its goal point; the second part is the same simple direct angular movement for the error in heading that is used in the same way with the x-value for linear
            // movement
            double u2 = (k2 * (std::sin(error.heading) / error.heading) * error.y) + (k * error.heading);

        // actual calculations of modified linear and angular velocities as additions/subtractions to the profile's original values
            // the original linear velocity is also transformed to follow the robot's error in heading before having the control input subtracted from it
            linVel = ((currentPoint.linVel * 0.0254) * std::cos(error.heading)) + u1;
            // the angular velocity does not need to be transformed in the same way that the linear velocity needs to because it is already angular in reference to the robot
            angVel = currentPoint.angVel + u2;

            linVel *= 39.37008;

            linVel *= currentPoint.linVel / currentlyFollowing->maxSpeed;

        // recalculates the delay, as it may change due to the RAMSETE controller modifying the robot's movement
            double newDistance = std::fabs(error.heading * (linVel / angVel));
            double newDelay = newDistance / std::fabs(linVel);
            delay = newDelay * 1000;

            // textToWrite.push_back("ex = " + std::to_string(error.x) + ", ey = " + std::to_string(error.y) + ", eh = " + std::to_string(error.heading) + "\n");
            //std::cout << ("ex = " + std::to_string(error.x) + ", ey = " + std::to_string(error.y) + ", eh = " + std::to_string(error.heading) + "\n");
            /*
            if (delay > 100) {
                std::cout << "PSYCHOPATH VALUES AHEAD: \n";
                std::cout << "nd = " << delay << "\n";
                std::cout << "t = " << sum << "\n";
                std::cout << ("ex = " + std::to_string(error.x) + ", ey = " + std::to_string(error.y) + ", eh = " + std::to_string(error.heading) + "\n");
                std::cout << "ucl = " << location.heading << ", actual = " << getAggregatedHeading(Kalman1, Kalman2) << ", used = " << fixedOdomAngle << ", desired = " << fixedNextAngle << "\n\n";
            } else {
                std::cout << "nd = " << delay << "\n";
            }
            */
            
                delay = 5;
            /*

            std::cout << "ex = " << error.x << "\ney = " << error.y << "\neh = " << error.heading << "\n\n";

            std::cout << "k = " << k << "\nk2 = " << k2 << "\n\n";

            std::cout << "linv = " << linVel << ", prolinv = " << currentPoint.linVel << "\n";
            std::cout << "angv = " << angVel << ", proangv = " << currentPoint.angVel << "\n\n";
            
            */
        }

        // standard calculation of output of each side based on specifications of the motion profile
        velocitiesRPM = this->calculateOutputOfSides(linVel, angVel, currentlyFollowing->maxSpeed);

        // executes custom actions if the profile has reached or passed their t-point and have not yet been activated
        for (int i = 0; i < actions.size(); i++) {
            if (((currentPoint.t >= actionTs[i])) && !actionCompleteds[i]) {
                actions[i]();
                actionCompleteds[i] = true;
            }
        }

        // converts the velocity in rpm to velocity in millivolts
        int maxVoltage = 12000; // innate max voltage of motors (in mV)
        double rpmToV = maxVoltage / g_maxRPM; // multiplier to convert rpm to voltage (in units of millivoltage / rpm so multiplying it by rpm cancels to millivoltage)

        // sends the output voltage to the motors
        if (reverse && !RAMSETE) {
            leftDrivetrain.move_voltage(-velocitiesRPM[1] * rpmToV);
            rightDrivetrain.move_voltage(-velocitiesRPM[0] * rpmToV);
        } else {
            leftDrivetrain.move_voltage(velocitiesRPM[0] * rpmToV);
            rightDrivetrain.move_voltage(velocitiesRPM[1] * rpmToV);
        }
        // LOGGING FOR TEST PURPOSES
        //std::cout << timeAtCurrentVelocity << "\n";
        //textToWrite.push_back("ix = " + std::to_string(currentPoint.x) + ", iy = " + std::to_string(currentPoint.y) + ", ihead = " + std::to_string(currentPoint.heading) + "\n" + "ax = " + std::to_string(location.x) + ", ay = " + std::to_string(location.y) + ", ahead = " + std::to_string(location.heading) + "\n" + std::to_string(currentPoint.t) + "\n\n");
        // textToWrite.push_back("il = " + std::to_string(currentPoint.linVel) + ", ia = " + std::to_string(currentPoint.angVel) + "\nal = " + std::to_string(linVel) + ", aa = " + std::to_string(angVel) + "\n\n");
        //std::cout << "lvol = " << velocitiesRPM[0] * rpmToV << ", rvol = " << velocitiesRPM[1] * rpmToV << "\n";
        //std::cout << "lv = " << linVel << ", rv = " << angVel << "\n\n";
        std::cout << "lrpm = " << velocitiesRPM[0] << ", rrpm = " << velocitiesRPM[1] << "\n";
        //std::cout << "x = " << currentPoint.x << ", y = " << currentPoint.y << "\n";
        //std::cout << "step = " << currentStep << "\n";
        //std::cout << " lvel = " << currentPoint.linVel << ", avel = " << currentPoint.angVel << "\n\n";
        // std::cout << "ct = " << currentPoint.t << ", nt = " << nextPoint.t << "\n";
        //logfile.appendFile("x = " + std::to_string(location.x) + ", should be " + std::to_string(currentPoint.x) + "\n");
        //logfile.appendFile("y = " + std::to_string(location.y) + ", should be " + std::to_string(currentPoint.y) + "\n");
        //logfile.appendFile("h = " + std::to_string(location.heading + ", should be " + std::to_string(currentPoint.heading));
        //logfile.appendFile("lvol = " + std::to_string(leftDrivetrain.get_voltage()) + ", should be " + std::to_string(velocitiesRPM[0] * rpmToV) + "\n" + "rvol = " + std::to_string(rightDrivetrain.get_voltage()) + ", should be " + std::to_string(velocitiesRPM[1] * rpmToV) + "\n\n");
        //std::cout << "t = " << std::to_string(currentPoint.t) << ", time = " << std::to_string(((double) pros::millis() - (double) startTime) / 1000) << "\n";
        //std::cout << (pros::micros() - timeTrack) / 1000 << " is calc\n";
        //std::cout << timeAtCurrentVelocity * 1000 << " is delay\n";
        
        // computes the calculation time using the timer and converts it to ms
        double calcTime = (pros::micros() - timeSinceStartOfLoop) / 1000;

        // 5 ms delay (- the time taken to calculate)
        pros::delay(delay - calcTime);

        // if the current step is the final point (t = 1 - step), 
        // then the drivetrain is stopped and the function ends
        if (currentPoint.t == currentlyFollowing->profile[currentlyFollowing->profile.size() - 1].t) {
            if (currentlyFollowing->zones[currentlyFollowing->zones.size() - 1].zoneLine.slope + currentlyFollowing->zones[currentlyFollowing->zones.size() - 1].zoneLine.yIntercept == 0) {
                drivetrain.brake();
            }
            //std::cout << (pros::millis() - startTime) / (double) 1000 << "\n";
            //std::cout << currentlyFollowing->totalTime << "\n";
            return;
        }

        // goes to the next step of the function if it did not end
        currentStep += step;
    }
}

// starts the filter loop if it is not already active (public)
void VelocityController::startProfile(MotionProfile* profile, bool reverse, bool RAMSETE) {

    // auto controlLoopFunction = [this, path, RAMSETE] () {return this->followProfile(*this->queuedProfile, path, RAMSETE);};

    // if (controlLoop_task_ptr == NULL) {
        // pros::Task* controlLoop_task_ptr = new pros::Task(controlLoopFunction);
        this->followProfile(profile, RAMSETE, reverse);
    // }
}

void VelocityController::addAction(std::function<void(void)> action, double time) {
    double actionT = time;
    this->actions.push_back(action);
    this->actionTs.push_back(actionT);
}

void VelocityController::clearActions(void) {
    this->actions.clear();
    this->actionTs.clear();
    this->actionCompleteds = {false, false, false, false, false, false};
}