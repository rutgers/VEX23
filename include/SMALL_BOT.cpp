#include "SMALL_BOT.hpp"

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button()
{
	static bool pressed = false;
	pressed = !pressed;
	if (pressed)
	{
		pros::lcd::set_text(2, "I was pressed!");
	}
	else
	{
		pros::lcd::clear_line(2);
	}
}

int FRONT_LIFT_GEAR_RATIO = 7;
int BACK_LIFT_GEAR_RATIO = 7;

double BACK_LIFT_DOWN = (1.5 / 5.0) * BACK_LIFT_GEAR_RATIO;
double BACK_LIFT_UP = (0.03) * BACK_LIFT_GEAR_RATIO;
double BACK_LIFT_UP_S = (1.5 / 6.0) * BACK_LIFT_GEAR_RATIO;

double FRONT_LIFT_DOWN = (0.12) * FRONT_LIFT_GEAR_RATIO;
double FRONT_LIFT_DOWN_1 = (0.08) * FRONT_LIFT_GEAR_RATIO;
double FRONT_LIFT_UP = (0.5) * FRONT_LIFT_GEAR_RATIO;
double FRONT_LIFT_UP_S = (0.35) * FRONT_LIFT_GEAR_RATIO;
double FRONT_LIFT_UP_SLI = (1 / 360.0) * FRONT_LIFT_GEAR_RATIO;

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few 1.seconds.
 */
void initialize()
{

	// selector::init();

	ks.kP = 0.0010;
	ks.kI = 0;
	ks.kD = 0;
	ks.kBias = 0;

	// Drive Motors
	frontFrontLft.reset(new okapi::Motor(11, true, okapi::AbstractMotor::gearset::blue, okapi::AbstractMotor::encoderUnits::rotations));
	frontLft.reset(new okapi::Motor(12, false, okapi::AbstractMotor::gearset::blue, okapi::AbstractMotor::encoderUnits::rotations));
	backLft.reset(new okapi::Motor(13, true, okapi::AbstractMotor::gearset::blue, okapi::AbstractMotor::encoderUnits::rotations));
	backBackLft.reset(new okapi::Motor(14, false, okapi::AbstractMotor::gearset::blue, okapi::AbstractMotor::encoderUnits::rotations));

	frontFrontRt.reset(new okapi::Motor(17, true, okapi::AbstractMotor::gearset::blue, okapi::AbstractMotor::encoderUnits::rotations));
	frontRt.reset(new okapi::Motor(18, false, okapi::AbstractMotor::gearset::blue, okapi::AbstractMotor::encoderUnits::rotations));
	backRt.reset(new okapi::Motor(19, true, okapi::AbstractMotor::gearset::blue, okapi::AbstractMotor::encoderUnits::rotations));
	backBackRt.reset(new okapi::Motor(20, false, okapi::AbstractMotor::gearset::blue, okapi::AbstractMotor::encoderUnits::rotations));

	drive_lft.reset(new okapi::MotorGroup({frontFrontLft, frontLft, backLft, backBackLft}));
	drive_rt.reset(new okapi::MotorGroup({frontFrontRt, frontRt, backRt, backBackRt}));

	drive_lft->setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
	drive_rt->setBrakeMode(okapi::AbstractMotor::brakeMode::hold);

	chassis = okapi::ChassisControllerBuilder()
				  .withMotors(drive_lft, drive_rt)
				  // Green gearset, 4 in wheel diam, 11.5 in wheel track
				  .withDimensions(okapi::AbstractMotor::gearset::blue, {{3.25_in, 14.5_in}, okapi::imev5BlueTPR})
				  .withGains(ks, ks)
				  .build();

	// Intake - with gearset
	intakeR.reset(new okapi::Motor(10, false, okapi::AbstractMotor::gearset::green, okapi::AbstractMotor::encoderUnits::rotations));
	intakeL.reset(new okapi::Motor(2, true, okapi::AbstractMotor::gearset::green, okapi::AbstractMotor::encoderUnits::rotations));
	intake.reset(new okapi::MotorGroup({intakeR, intakeL}));
	intake->setBrakeMode(okapi::AbstractMotor::brakeMode::hold);

	// catapult 
	catapult.reset(new okapi::Motor(1, false, okapi::AbstractMotor::gearset::red, okapi::AbstractMotor::encoderUnits::rotations));
	// catapult->setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
	catapult_control = okapi::AsyncPosControllerBuilder().withMotor(catapult).build();

	// Controller Initialization
	master.reset(new pros::Controller(pros::E_CONTROLLER_MASTER));
	partner.reset(new pros::Controller(pros::E_CONTROLLER_PARTNER));

	// Color Sensor
	color_sensor.reset(new pros::Optical(9));
	color_sensor->set_led_pwm(100);
	// Limit Switch B
	limit_switch.reset(new pros::ADIDigitalIn('B'));

	// Pneumatics
	piston.reset(new pros::ADIDigitalOut('A', true));	// back lift piston

	endgame.reset(new pros::ADIDigitalOut('C', true));	// back lift piston

	// IMU
	imu.reset(new pros::Imu(15));

	// Ratchet 
	ratchet.reset(new okapi::Motor(5, false, okapi::AbstractMotor::gearset::red, okapi::AbstractMotor::encoderUnits::rotations));

}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */

void autonomous()
{

	chassis->setMaxVelocity(150);
	pros::Task launching_task{ [] {
        while (pros::Task::notify_take(true, TIMEOUT_MAX)) {
			
		}
    } }; 

	piston->set_value(false);


	chassis->moveDistance(39_in);
	// launching_task.notify();

	catapult->moveVoltage(12000);
	pros::delay(250);
	while (!limit_switch->get_value()) 
	{
		catapult->moveVoltage(12000);
		pros::delay(20);
	}
	catapult->moveVoltage(0);

	pros::delay(2000);

	intake->moveVoltage(12000);
	pros::delay(1000);
	intake->moveVoltage(0);
	catapult->moveVoltage(12000);
	pros::delay(250);
	while (!limit_switch->get_value()) 
	{
		catapult->moveVoltage(12000);
		pros::delay(20);
	}
	catapult->moveVoltage(0);
	pros::delay(2000);

	int init_angle_offset = -25;

	imu_turning(init_angle_offset-40, drive_lft, drive_rt, imu, master);
	chassis->moveDistance(-44_in);
	imu_turning(init_angle_offset, drive_lft, drive_rt, imu, master);
	drive_lft->moveVoltage(-3000);
	drive_rt->moveVoltage(-3000);
	pros::delay(1000);
	move_roller(drive_lft, drive_rt, color_sensor, intake, master, COLOR_SIDE);
	chassis->moveDistance(12_in);
	imu_turning(init_angle_offset-90, drive_lft, drive_rt, imu, master);
	// imu_turning(init_angle_offset+135, drive_lft, drive_rt, imu, master);
	// intake->moveVelocity(12000);
	// chassis->moveDistance(-70_in);
	// imu_turning(init_angle_offset+45, drive_lft, drive_rt, imu, master);

	// intake->moveVelocity(12000);
	// catapult->moveVoltage(12000);
	// pros::delay(250);
	// while (!limit_switch->get_value()) 
	// {
	// 	catapult->moveVoltage(12000);
	// 	pros::delay(20);
	// }
	// catapult->moveVoltage(0);


	// if (selector::auton == 0)
	// {
	// 	// Get the colored mobile goal
	// 	chassis->moveDistance(0.3_ft);
	// 	/* piston->set_value(false);
	// 	pros::delay(200);

	// 	// drive a little forward
	// 	chassis->moveDistance(-0.4_ft);
	// 	pros::delay(200);

	// 	// lift the front lift up
	// 	front_lift_control->setTarget(-FRONT_LIFT_DOWN);
	// 	pros::delay(500);

	// 	// turn around 90 degrees
	// 	imu_turning(270, drive_rt, drive_lft, imu);
	// 	pros::delay(200);
	// 	//pros::delay(200);

	// 	// move to line up with rings line
	// 	chassis->moveDistance(-1_ft);
	// 	pros::delay(200);

	// 	// turn around to line front with rings
	// 	imu_turning(270, drive_rt, drive_lft, imu);
	// 	pros::delay(200); */

	// 	// move to line up with rings line
	// 	/* chassis->moveDistance(-0.4_ft);
	// 	pros::delay(200);

	// 	// turn around to line front with rings
	// 	imu_turning(270, drive_rt, drive_lft, imu);
	// 	pros::delay(200);

	// 	// move front lift up
	// 	front_lift_control->setTarget(-FRONT_LIFT_DOWN_1);
	// 	pros::delay(200); */
	// }
	// else
	// {
		// Move to the Yellow
		// int DIST = 35; // mm distance
		// drive_lft->moveVoltage(12000);
		// drive_rt->moveVoltage(12000);
		// int move_time = 0;

		// drive_lft->setBrakeMode(okapi::AbstractMotor::brakeMode::brake);
		// drive_rt->setBrakeMode(okapi::AbstractMotor::brakeMode::brake);
		// int MAX_TIME = 1400; // seconds want to stop running no matter what (FOR AUTON LINE)
		// while ((dist_sensor->get() > DIST || dist_sensor->get() == 0) && move_time < MAX_TIME)
		// {
			
		// 	if (dist_sensor->get() < 400 && dist_sensor->get() != 0)
		// 	{
		// 		drive_rt->moveVoltage(6000);
		// 		drive_lft->moveVoltage(6000);
		// 		MAX_TIME = MAX_TIME + 3;
		// 	}

		// 	pros::delay(5);
		// 	move_time += 5;
			
		// }
		// drive_rt->moveVoltage(0);
		// drive_lft->moveVoltage(0);

		// // Grab the yellow
		// front_piston->set_value(true);

		// // Go back
		// chassis->moveDistance(-1.61_ft);
		// chassis->setMaxVelocity(100);
		// pros::delay(400);

		// /* // Turn to the red or blue
		// chassis->turnAngle(-60_deg); //imu_turning(90, drive_lft, drive_rt, imu);
		// pros::delay(200);

		// // drop the yellow in prep for the middle
		// //front_piston->set_value(false);

		// // Move toward the blue or red
		// chassis->moveDistance(-0.5_ft);
		// pros::delay(500);
		// piston->set_value(false);
		// pros::delay(1000);

		// // Move back from the blue spot
		// chassis->moveDistance(0.2_ft);
		// pros::delay(200); */

	// }
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol()
{

	chassis->stop();

	piston->set_value(true);


	/* bool chassis_hold = false; */
	int delay = 0;
	int move_volt = 11000;
	int CATAPULT_SPEED = 12000;
	int intake_speed = 0;
	bool first_launch = true;
	int MAX_INTAKE = 12000;
	while (true)
	{

		pros::c::optical_rgb_s_t color = color_sensor->get_rgb();
		master->print(0, 0, "%f %f\n", color.red, color.blue);
		// Drive Mechanics

		// if (selector::auton == 0)
		// {
			double y = master->get_analog(ANALOG_LEFT_Y);
			double x = 0; // master->get_analog(ANALOG_LEFT_X);
			double z = master->get_analog(ANALOG_RIGHT_X);

			if(partner->get_digital(DIGITAL_B)) {
				y = y*.5;
				z = z*.5;
			}

			drive_lft->moveVoltage((y + x + z) / 127 * move_volt);
			drive_rt->moveVoltage((y - x - z) / 127 * move_volt);
		// }
		// else
		// {
		// 	double y_l = -master->get_analog(ANALOG_LEFT_Y);
		// 	double y_r = master->get_analog(ANALOG_RIGHT_Y);
		// 	drive_lft->moveVoltage(y_l / 127 * move_volt);
		// 	drive_rt->moveVoltage(y_r / 127 * move_volt);
		// 	/* double y = master->get_analog(ANALOG_LEFT_Y);
		// 	double x = 0; // master->get_analog(ANALOG_LEFT_X);
		// 	double z = master->get_analog(ANALOG_RIGHT_X);
		// 	drive_lft->moveVoltage((y + x + z) / 127 * move_volt);
		// 	drive_rt->moveVoltage((y - x - z) / 127 * move_volt); */
		// }

		// Catapult 
		// If limit switch isn't pressed, always move forward
		// When pressed, wait for user input
		

		if(master->get_digital(DIGITAL_B)) {
			move_roller(drive_lft, drive_rt, color_sensor, intake, master, COLOR_SIDE);
		}

		// Intake Mechanics
		if (master->get_digital(DIGITAL_UP))
		{ // Intake Up
			intake_speed = MAX_INTAKE;
		}
		else if (master->get_digital(DIGITAL_RIGHT))
		{ // Intake Down
			intake_speed = 0;
		}
		else if (master->get_digital(DIGITAL_DOWN))
		{
			intake_speed = -MAX_INTAKE;
		}
		else if (master->get_digital_new_press(DIGITAL_L2)) 
		{
			if(intake_speed < MAX_INTAKE) 
			{
				intake_speed = MAX_INTAKE;
			}
			else
			{
				intake_speed = 0;
			}
		}

		if (!limit_switch->get_value() && !first_launch) 
		{
			intake_speed = 0;
			catapult->moveVoltage(CATAPULT_SPEED);
		}
		else {
			if(master->get_digital(DIGITAL_R2)) {
				first_launch = false;
				catapult->moveVoltage(CATAPULT_SPEED);
			}
			else {
				catapult->moveVoltage(0);
			}
		}

		intake->moveVoltage(intake_speed);

		if(master->get_digital(DIGITAL_X)) {
			piston->set_value(false);
		}
		else {
			piston->set_value(true);
		}

		if(master->get_digital(DIGITAL_Y))
		{
			catapult->moveVoltage(0);
			ratchet->moveAbsolute(-45.0/360.0, 12000);
		}
		else {
			ratchet->moveAbsolute(2.5/360.0, 1200);
		}
		pros::delay(20);
		if (delay > 0)
		{
			delay = delay - 20;
		}

		if(master->get_digital(DIGITAL_A) && partner->get_digital(DIGITAL_A)) {
			endgame->set_value(true);
		}
	}
}