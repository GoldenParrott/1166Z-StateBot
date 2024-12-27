#ifndef _PROS_INIT_H_
#define _PROS_INIT_H_

#include "main.h"

//Controllers
    extern pros::Controller Master;

//Motors
    //Drivetrain 
        extern pros::Motor leftFront;
        extern pros::Motor leftMiddle;
        extern pros::Motor leftRear;

        extern pros::Motor rightFront;
        extern pros::Motor rightMiddle;
        extern pros::Motor rightRear;

        extern pros::MotorGroup leftDrivetrain;
        extern pros::MotorGroup rightDrivetrain;

        extern pros::MotorGroup drivetrain;

    //Intake
        extern pros::Motor preRoller;
        extern pros::Motor transport;

        extern pros::MotorGroup intake;

    //Arm
        extern pros::Motor arm;
    
    //Clamp
        extern pros::ADIDigitalOut clamp;
   


// Variables

    extern int num;
    extern bool clampOn;

// Pointers

    pros::Task* controlLoop_task_ptr;

#endif