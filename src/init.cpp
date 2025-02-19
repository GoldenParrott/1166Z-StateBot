#include "init.h"

//Controllers
    pros::Controller master(pros::E_CONTROLLER_MASTER);

//Motors
// Hello, anyone who might be in the vicinity. Ports 2, 3, and 11 are dead, likely afflicted by one of the many ailments that their kind is subject to. Please remove them
// from the brain and replace them with another port, ensuring that they are well-educated in how to connect a device to a V5 brain so the robot will not break mid-match.
    //Drivetrain
        pros::Motor leftFront(-11, pros::v5::MotorGears::blue, pros::v5::MotorEncoderUnits::degrees);
        pros::Motor leftMiddle(-3,pros::v5::MotorGears::blue, pros::v5::MotorEncoderUnits::degrees);
        pros::Motor leftRear(-4, pros::v5::MotorGears::blue, pros::v5::MotorEncoderUnits::degrees);

        pros::Motor rightFront(20, pros::v5::MotorGears::blue, pros::v5::MotorEncoderUnits::degrees);
        pros::Motor rightMiddle(7, pros::v5::MotorGears::blue, pros::v5::MotorEncoderUnits::degrees);
        pros::Motor rightRear(17,pros::v5::MotorGears::blue, pros::v5::MotorEncoderUnits::degrees);

        // Front, Middle, Rear
        pros::MotorGroup leftDrivetrain({-11,-3,-4}, pros::v5::MotorGears::blue, pros::v5::MotorEncoderUnits::degrees);
        pros::MotorGroup rightDrivetrain({20,7,17}, pros::v5::MotorGears::blue, pros::v5::MotorEncoderUnits::degrees);

        pros::MotorGroup drivetrain({-11,-1,-4,20,7,17}, pros::v5::MotorGears::blue, pros::v5::MotorEncoderUnits::degrees);

    //Intake
        pros::Motor preRoller(-19,pros::v5::MotorGears::green, pros::v5::MotorEncoderUnits::degrees);
        pros::Motor transport(8,pros::v5::MotorGears::blue, pros::v5::MotorEncoderUnits::degrees);

        pros::MotorGroup intake({19,8}, pros::v5::MotorGears::blue, pros::v5::MotorEncoderUnits::degrees);
    
    //Arm 
        pros::Motor arm(10,pros::v5::MotorGears::green, pros::v5::MotorEncoderUnits::degrees);

    // Pistons
         pros::adi::DigitalOut clamp(1);
         pros::adi::DigitalOut inPutston(2);
         pros::adi::DigitalOut yoin(5);
         pros::adi::DigitalOut ker(7);

    // Rotational Sensors
        pros::Rotation Rotational(6);
        pros::Rotation RotationalTurn(9);
        pros::Rotation ArmRotational(1);

    // Other Sensors
        pros::Optical colorSense(18);
        pros::Distance Distance(15);

    // Inertial Sensors/Kalman Filters
        pros::IMU Inertial1(5);
        KalmanFilter Kalman1 = KalmanFilter(&Inertial1, &RotationalTurn);
        pros::IMU Inertial2(16);
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
    int logoCount = 0;

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