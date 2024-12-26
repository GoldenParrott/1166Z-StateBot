#ifndef _PROS_INIT_H_
#define _PROS_INIT_H_

#include "main.h"

//Controllers
    extern pros::Controller Master;

//Motors
    extern pros::Motor leftFront;
    extern pros::Motor leftMiddle;
    extern pros::Motor leftRear;

    extern pros::Motor rightFront;
    extern pros::Motor rightMiddle;
    extern pros::Motor rightRear;

    extern pros::MotorGroup leftDrivetrain;
	extern pros::MotorGroup rightDrivetrain;

    extern pros::MotorGroup drivetrain;


// Variables

    extern int num;

#endif