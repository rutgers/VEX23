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
std::shared_ptr<okapi::Motor> frontUpperLft;
std::shared_ptr<okapi::Motor> backLft; // top
std::shared_ptr<okapi::Motor> backBackLft;

std::shared_ptr<okapi::Motor> frontFrontRt;
std::shared_ptr<okapi::Motor> frontRt;
std::shared_ptr<okapi::Motor> frontUpperRt;
std::shared_ptr<okapi::Motor> backRt; // top
std::shared_ptr<okapi::Motor> backBackRt;

std::shared_ptr<okapi::MotorGroup> drive_lft;
std::shared_ptr<okapi::MotorGroup> drive_rt;

// Chassis Initialization
std::shared_ptr<okapi::ChassisController> chassis;

// Back Lift Initializations
std::shared_ptr<okapi::Motor> backLeftLift;
std::shared_ptr<okapi::Motor> backRtLift;
std::shared_ptr<okapi::MotorGroup> back_lift;
std::shared_ptr<okapi::AsyncPositionController<double, double>> back_lift_control;

// Front Lift Initializations
std::shared_ptr<okapi::Motor> frontLftLift;
std::shared_ptr<okapi::Motor> frontRtLift;
std::shared_ptr<okapi::MotorGroup> front_lift;
std::shared_ptr<okapi::AsyncPositionController<double, double>> front_lift_control;

// Intake Initializations
std::shared_ptr<okapi::Motor> intake;

// Controller Initializations
std::shared_ptr<pros::Controller> master;

// Piston Initializations
std::shared_ptr<pros::ADIDigitalOut> piston;
std::shared_ptr<pros::ADIDigitalOut> front_piston;

// Camera Initialization
std::shared_ptr<GoalCamera> camera;

// IMU Initialization
std::shared_ptr<pros::Imu> imu;

// Distance Sensor Init
std::shared_ptr<pros::Distance> dist_sensor;

// Button Init
std::shared_ptr<okapi::ADIButton> button;

// Snatcher
std::shared_ptr<okapi::Motor> snatcher;
