#include "init.h"

//Controllers
    pros::Controller master(pros::E_CONTROLLER_MASTER);



    //Motors
        //Drivetrain
            pros::Motor leftFront(-8, pros::v5::MotorGears::blue, pros::v5::MotorEncoderUnits::degrees);
            pros::Motor leftMiddle(-9,pros::v5::MotorGears::blue, pros::v5::MotorEncoderUnits::degrees);
            pros::Motor leftRear(-10, pros::v5::MotorGears::blue, pros::v5::MotorEncoderUnits::degrees);

            pros::Motor rightFront(3, pros::v5::MotorGears::blue, pros::v5::MotorEncoderUnits::degrees);
            pros::Motor rightMiddle(4, pros::v5::MotorGears::blue, pros::v5::MotorEncoderUnits::degrees);
            pros::Motor rightRear(5,pros::v5::MotorGears::blue, pros::v5::MotorEncoderUnits::degrees);

            // Front, Middle, Rear
            pros::MotorGroup leftDrivetrain({-8,-9,-10}, pros::v5::MotorGears::blue, pros::v5::MotorEncoderUnits::degrees);
            pros::MotorGroup rightDrivetrain({3,4,5}, pros::v5::MotorGears::blue, pros::v5::MotorEncoderUnits::degrees);

            pros::MotorGroup drivetrain({-8,-9,-10,3,4,5}, pros::v5::MotorGears::blue, pros::v5::MotorEncoderUnits::degrees);

        //Intake
            pros::Motor preRoller(11,pros::v5::MotorGears::green, pros::v5::MotorEncoderUnits::degrees);
            pros::Motor transport(1,pros::v5::MotorGears::green, pros::v5::MotorEncoderUnits::degrees);

            pros::MotorGroup intake({11,1}, pros::v5::MotorGears::blue, pros::v5::MotorEncoderUnits::degrees);
        
        //Arm 
            pros::Motor arm(-2,pros::v5::MotorGears::green, pros::v5::MotorEncoderUnits::degrees);

    // Pistons
         pros::adi::DigitalOut clamp(1);
         pros::adi::DigitalOut inPutston(2);
// 1 2  3 4 5 6 7 8 10 11  20, 19, 15
         // left items
         pros::adi::DigitalOut leftYoin(8);
         pros::adi::DigitalOut leftKer(7);
         pros::adi::DigitalOut leftNoid(4);

         // right items
         pros::adi::DigitalOut rightYoin(6);
         pros::adi::DigitalOut rightKer(3);
         pros::adi::DigitalOut rightNoid(5);

    // Rotational Sensors
        pros::Rotation Rotational(15);
        pros::Rotation RotationalTurn(7);
        pros::Rotation ArmRotational(6);

    // Other Sensors
        pros::Optical colorSense(14);
        pros::Distance Distance(20);

    // Inertial Sensors/Kalman Filters
        pros::IMU Inertial1(18);
        KalmanFilter Kalman1 = KalmanFilter(&Inertial1, &RotationalTurn);
        pros::IMU Inertial2(19);
        KalmanFilter Kalman2 = KalmanFilter(&Inertial2, &RotationalTurn);

// Task Pointers

    pros::Task* controlLoop_task_ptr = NULL;
    pros::Task* coordinateUpdater_task_ptr = NULL;
    pros::Task* macros_task_ptr = NULL;
    pros::Task* autoEject_task_ptr = NULL;
    pros::Task* eject_task_ptr = NULL;
    pros::Task* rotationalBinder_task_ptr = NULL;
    pros::Task* autoSelector_task_ptr = NULL;
    pros::Task* loggingEnabled_task_ptr = NULL;
    pros::Task* cutoffMovement_task_ptr = NULL;

// Image Definitions
/*
    Gif Bozo("/usd/[Bozo].gif", lv_scr_act());
    Gif Logo("/usd/[Logo].gif", lv_scr_act());
    Gif RMA("/usd/[Red_Mogo_AWP].gif", lv_scr_act());
    Gif BMA("/usd/[Blue_Mogo_AWP].gif", lv_scr_act());
    Gif RRA("/usd/[Red_Ring_AWP].gif", lv_scr_act());
    Gif BRA("/usd/[Blue_Ring_AWP].gif", lv_scr_act());
    Gif RME("/usd/[Red_Mogo_ELIM].gif", lv_scr_act());
    Gif BME("/usd/[Blue_Mogo_ELIM].gif", lv_scr_act());
    Gif RRE("/usd/[Red_Ring_ELIM].gif", lv_scr_act());
    Gif BRE("/usd/[Blue_Ring_ELIM].gif", lv_scr_act());
*/
// Variables

    int num;
    bool clampOn;

    int autonnumber = 1;
    bool globalAuton = true;
    bool confirm = false;
    bool was2 = false;
    int logoCount = 0;

    std::vector<MotionProfile*> path = {};

    int drvtrDZ = 10;
    int drvtrFB;
    int drvtrLR;
    // upright = 1900, upleft = -1900

    bool toggleColorSensor = false;
    int colorDelay = 0;
    bool ejectIsEjecting = false;

    bool armMoving = false;
    

    bool presettingA = false;
    bool presettingX = false;

    Point endCoords;
    bool endReverse;
    bool endended;
    
    //For Autonomous Selector
    pros::screen_touch_status_s_t status;

    Pose universalCurrentLocation = {0, 0};

    std::vector<std::string> textToWrite;

    File logfile = File("log");

    // robot config for motion profiling
    double g_gearRatio = 0.75;
    double g_maxRPM = 600;
    double g_diameter = 3.25;
    double g_distBetweenWheels = 10.5;