#include "main.h"
#include "okapi/api.hpp"
#include <vector>

#ifndef AUTON_CPP
#define AUTON_CPP
#include "auton_util.cpp"
#endif

// PID Control
okapi::IterativePosPIDController::Gains ks; 
okapi::IterativePosPIDController::Gains rotator_ks; 


// Drive Motor Initializations
std::shared_ptr<okapi::Motor> frontFrontLft;
std::shared_ptr<okapi::Motor> frontLft;
std::shared_ptr<okapi::Motor> backLft;
std::shared_ptr<okapi::Motor> backBackLft;

std::shared_ptr<okapi::Motor> frontFrontRt;
std::shared_ptr<okapi::Motor> frontRt;
std::shared_ptr<okapi::Motor> backRt;
std::shared_ptr<okapi::Motor> backBackRt;

std::shared_ptr<okapi::MotorGroup> drive_lft;
std::shared_ptr<okapi::MotorGroup> drive_rt;

// Chassis Initialization
std::shared_ptr<okapi::ChassisController> chassis;

// Intake Initializations
std::shared_ptr<okapi::Motor> intakeL;
std::shared_ptr<okapi::Motor> intakeR;
std::shared_ptr<okapi::MotorGroup> intake;

// Flywheel Initializations
std::shared_ptr<okapi::Motor> flywheelL;
std::shared_ptr<okapi::Motor> flywheelR;
std::shared_ptr<okapi::MotorGroup> flywheel;

// Rotator Initializations
std::shared_ptr<okapi::Motor> rotator;
std::shared_ptr<okapi::AsyncPositionController<double, double>> rotator_control;

// Elevator Initializations
std::shared_ptr<okapi::Motor> elevatorL;
std::shared_ptr<okapi::Motor> elevatorR;
std::shared_ptr<okapi::MotorGroup> elevator;
std::shared_ptr<okapi::AsyncPositionController<double, double>> elevator_control;

// Piston Initializations
std::shared_ptr<pros::ADIDigitalOut> piston;

// Controller Initializations
std::shared_ptr<pros::Controller> master;
std::shared_ptr<pros::Controller> partner;

// IMU Initialization
std::shared_ptr<pros::Imu> imu;

// Color Sensor Initialization
std::shared_ptr<pros::Optical> color_sensor;

// Limit Switch Initialization
std::shared_ptr<pros::ADIDigitalIn> limit_switch;

// Ratchet Initialization
//std::shared_ptr<okapi::Motor> ratchet;

