#include "main.h"
#include "okapi/api.hpp"
#include <vector>

#ifndef AUTON_CPP
#define AUTON_CPP
#include "auton_util.cpp"
#endif

// PID Control
okapi::IterativePosPIDController::Gains ks; 

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

// Front Lift Initializations
std::shared_ptr<okapi::Motor> intakeL;
std::shared_ptr<okapi::Motor> intakeR;
std::shared_ptr<okapi::MotorGroup> intake;

// Flywheel Initializations
std::shared_ptr<okapi::Motor> flywheelL;
std::shared_ptr<okapi::Motor> flywheelR;
std::shared_ptr<okapi::MotorGroup> flywheel;

// Indexer Initializations
std::shared_ptr<okapi::Motor> indexer;

// Controller Initializations
std::shared_ptr<pros::Controller> master;

// IMU Initialization
std::shared_ptr<pros::Imu> imu;

// Color Sensor Initialization
std::shared_ptr<pros::Optical> color_sensor;

// Limit Switch Initialization
std::shared_ptr<pros::ADIDigitalIn> limit_switch;

// Ratchet Initialization
std::shared_ptr<okapi::Motor> ratchet;

