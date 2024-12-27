#include "init.h"

//Controllers
    pros::Controller Master(pros::E_CONTROLLER_MASTER);

//Motors
    
    //Drivetrain
        pros::Motor leftFront({-18}, pros::v5::MotorGears::blue, pros::v5::MotorEncoderUnits::degrees);
        pros::Motor leftMiddle(-19,pros::v5::MotorGears::blue, pros::v5::MotorEncoderUnits::degrees);
        pros::Motor leftRear(-17, pros::v5::MotorGears::blue, pros::v5::MotorEncoderUnits::degrees);

        pros::Motor rightFront(16, pros::v5::MotorGears::blue, pros::v5::MotorEncoderUnits::degrees);
        pros::Motor rightMiddle(12, pros::v5::MotorGears::blue, pros::v5::MotorEncoderUnits::degrees);
        pros::Motor rightRear(15,pros::v5::MotorGears::blue, pros::v5::MotorEncoderUnits::degrees);

        // Front, Middle, Rear
        pros::MotorGroup leftDrivetrain({-18,-19,-17}, pros::v5::MotorGears::blue, pros::v5::MotorEncoderUnits::degrees);
        pros::MotorGroup rightDrivetrain({16,12,15}, pros::v5::MotorGears::blue, pros::v5::MotorEncoderUnits::degrees);

        pros::MotorGroup drivetrain({-18,-19,-17,16,12,15}, pros::v5::MotorGears::blue, pros::v5::MotorEncoderUnits::degrees);

    //Intake
        pros::Motor preRoller(9,pros::v5::MotorGears::green, pros::v5::MotorEncoderUnits::degrees);
        pros::Motor transport(-10,pros::v5::MotorGears::blue, pros::v5::MotorEncoderUnits::degrees);

        pros::MotorGroup intake({9,-10}, pros::v5::MotorGears::blue, pros::v5::MotorEncoderUnits::degrees);
    
    //Arm 
        pros::Motor arm(-2,pros::v5::MotorGears::green, pros::v5::MotorEncoderUnits::degrees );

    //mogo
         pros::ADIDigitalOut clamp(1);

// Task Pointers

    pros::Task* controlLoop_task_ptr = NULL;
      
// Variables

    int num;
    bool clampOn;