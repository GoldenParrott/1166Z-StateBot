#ifndef _PROS_INIT_H_
#define _PROS_INIT_H_

#include "main.h"
#include "gif-pros/gifclass.hpp"

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
        extern pros::adi::DigitalOut leftYoin;
        extern pros::adi::DigitalOut leftKer;
        extern pros::adi::DigitalOut rightYoin;
        extern pros::adi::DigitalOut rightKer;
        extern pros::adi::DigitalOut inPutston;

    // Rotational Sensors
        extern pros::Rotation Rotational;
        extern pros::Rotation RotationalTurn;
        extern pros::Rotation ArmRotational;

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
    extern pros::Task* macros_task_ptr;
    extern pros::Task* autoEject_task_ptr;
    extern pros::Task* eject_task_ptr;
    extern pros::Task* rotationalBinder_task_ptr ;
    extern pros::Task* autoSelector_task_ptr;
    extern pros::Task* loggingEnabled_task_ptr;

// Image Definitions
/*
    extern Gif Bozo;
    extern Gif Logo;
    extern Gif RMA;
    extern Gif BMA;
    extern Gif RRA;
    extern Gif BRA;
    extern Gif RME;
    extern Gif BME;
    extern Gif RRE;
    extern Gif BRE;
*/
// Variables

    extern int num;
    extern bool clampOn;

    extern int autonnumber;
    extern bool confirm;
    extern bool globalAuton;
    extern int logoCount;

    extern std::vector<MotionProfile*> path;

    extern int drvtrDZ;
    extern int drvtrFB;
    extern int drvtrLR;
    // upright = 1900, upleft = -1900

    extern bool toggleColorSensor;
    extern int colorDelay;
    extern bool ejectIsEjecting;
    extern bool armMoving;

    extern bool presettingA;
    extern bool presettingX;

    extern Point endCoords;
    extern bool endReverse;
    extern bool endended;
    
    //For Autonomous Selector
    extern pros::screen_touch_status_s_t status;
    
// declared in initialize(), not init.cpp
    extern Pose universalCurrentLocation;

    extern std::vector<std::string> textToWrite;

    extern File logfile;

    // robot config for motion profiling
    extern double g_gearRatio;
    extern double g_maxRPM;
    extern double g_diameter;
    extern double g_distBetweenWheels;

#endif