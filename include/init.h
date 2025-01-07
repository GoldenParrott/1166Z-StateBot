#ifndef _PROS_INIT_H_
#define _PROS_INIT_H_

#include "main.h"

//Controllers
    extern pros::Controller master;

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
    
    // Pistons
        extern pros::adi::DigitalOut clamp;
        extern pros::adi::DigitalOut yoin;
        extern pros::adi::DigitalOut ker;

    // Rotational Sensors
        extern pros::Rotation Rotational;
        extern pros::Rotation RotationalTurn;

    // Other Sensors
        extern pros::Optical colorSense;
        extern pros::Distance Distance;

    // Inertial Sensors/Kalman Filters
        extern pros::IMU Inertial1;
        extern KalmanFilter Kalman1;
        extern pros::IMU Inertial2;
        extern KalmanFilter Kalman2;

// Task Pointers

    extern pros::Task* controlLoop_task_ptr;
    extern pros::Task* coordinateUpdater_task_ptr;
    extern pros::Task* rotationalBinder_task_ptr ;
    extern pros::Task* autoSelector_task_ptr;

// Variables

    extern int num;
    extern bool clampOn;

    extern int autonnumber;
    extern bool globalAuton;
    extern int logoCount;

    extern int drvtrDZ;
    extern int drvtrFB;
    extern int drvtrLR;
    // upright = 1900, upleft = -1900

    extern bool toggleColorSensor;
    extern int colorDelay;
    extern bool ejectIsEjecting;

    extern bool presettingA;
    extern bool presettingX;

    extern Point endCoords;
    extern bool endReverse;
    extern bool endended;

// declared in initialize(), not init.cpp
    extern Point universalCurrentLocation;

#endif