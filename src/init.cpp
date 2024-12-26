#include "init.h"

//Controllers
    pros::Controller Master(pros::E_CONTROLLER_MASTER);

//Motors
    
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

// Variables

    int num;