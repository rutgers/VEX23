#include "SKAR_2.hpp"

void imu_turning_2(double target) {
	imu_turning(target, drive_lft, drive_rt, imu, master);
}

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

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize()
{

	selector::init();

	if (selector::auton == 0)
	{
		ks.kP = 0.002;
		ks.kI = 0;
		ks.kD = 0; //-0.00001;
		ks.kBias = 0;
	}
	else
	{
		ks.kP = 0.00064;
		ks.kI = 0;
		ks.kD = 0; //-0.00001;
		ks.kBias = 0;
	}

	// Drive Motors
	front_rt1.reset(new okapi::Motor(1, false, okapi::AbstractMotor::gearset::green, okapi::AbstractMotor::encoderUnits::rotations));
	front_rt2.reset(new okapi::Motor(2, true, okapi::AbstractMotor::gearset::green, okapi::AbstractMotor::encoderUnits::rotations));
	back_rt1.reset(new okapi::Motor(3, true, okapi::AbstractMotor::gearset::green, okapi::AbstractMotor::encoderUnits::rotations));
	back_rt2.reset(new okapi::Motor(11, false, okapi::AbstractMotor::gearset::green, okapi::AbstractMotor::encoderUnits::rotations));
	front_lft1.reset(new okapi::Motor(10, true, okapi::AbstractMotor::gearset::green, okapi::AbstractMotor::encoderUnits::rotations));
	front_lft2.reset(new okapi::Motor(9, false, okapi::AbstractMotor::gearset::green, okapi::AbstractMotor::encoderUnits::rotations));
	back_lft1.reset(new okapi::Motor(20, true, okapi::AbstractMotor::gearset::green, okapi::AbstractMotor::encoderUnits::rotations));
	back_lft2.reset(new okapi::Motor(8, false, okapi::AbstractMotor::gearset::green, okapi::AbstractMotor::encoderUnits::rotations));

	front_rt.reset(new okapi::MotorGroup({front_rt1, front_rt2}));
	front_lft.reset(new okapi::MotorGroup({front_lft1, front_lft2}));
	back_rt.reset(new okapi::MotorGroup({back_rt1, back_rt2}));
	back_lft.reset(new okapi::MotorGroup({back_lft1, back_lft2}));

	drive_lft.reset(new okapi::MotorGroup({front_lft1, front_lft2, back_lft1, back_lft2}));
	drive_rt.reset(new okapi::MotorGroup({front_rt1, front_rt2, back_rt1, back_rt2}));
	chassis = okapi::ChassisControllerBuilder()\
				  .withMotors(drive_lft, drive_rt)
				  // Green gearset, 4 in wheel diam, 11.5 in wheel track
				  .withDimensions({okapi::AbstractMotor::gearset::green, (3.0 / 5.0)}, {{3.25_in, 12.4375_in}, okapi::imev5GreenTPR})
				  .withGains(ks, ks, ks)
				  .build();

	lift_front_lft.reset(new okapi::Motor(19, true, okapi::AbstractMotor::gearset::red, okapi::AbstractMotor::encoderUnits::rotations));
	lift_front_rt.reset(new okapi::Motor(12, false, okapi::AbstractMotor::gearset::red, okapi::AbstractMotor::encoderUnits::rotations));
	lift_front.reset(new okapi::MotorGroup({*lift_front_lft, *lift_front_rt}));
	lift_front->setBrakeMode(okapi::AbstractMotor::brakeMode::hold);

	lift_front_control = okapi::AsyncPosControllerBuilder().withMotor(*lift_front).build();
	lift_front_control->setTarget(FRONT_LIFT_INIT);

	front_claw_piston.reset(new pros::ADIDigitalOut('A'));
	back_tilter.reset(new pros::ADIDigitalOut('B'));
	back_claw_piston.reset(new pros::ADIDigitalOut('C'));


	intake_lft.reset(new okapi::Motor(14, true, okapi::AbstractMotor::gearset::green, okapi::AbstractMotor::encoderUnits::rotations));
	intake_rt.reset(new okapi::Motor(13, false, okapi::AbstractMotor::gearset::green, okapi::AbstractMotor::encoderUnits::rotations));
	intake.reset(new okapi::MotorGroup({*intake_lft, *intake_rt}));

	intake->setBrakeMode(okapi::AbstractMotor::brakeMode::coast);

	//camera.reset(new GoalCamera(17));

	imu.reset(new pros::Imu(4));

	dist_sensor.reset(new pros::Distance(15));

	master.reset(new pros::Controller(pros::E_CONTROLLER_MASTER));
	partner.reset(new pros::Controller(pros::E_CONTROLLER_PARTNER));
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
// original autonomous code
void autonomous()
{
	lift_front_control->tarePosition();
	chassis->stop();
	chassis->setMaxVelocity(200);
	if (selector::auton == 0)
	{
		int move_vel = 110;
		chassis->setMaxVelocity(move_vel);
		//grabbing the balance goal
		front_claw_piston->set_value(FRONT_CLAW_RELEASE); 

		back_tilter->set_value(BACK_TILTER_DOWN); 
		pros::delay(400);
		back_claw_piston->set_value(BACK_CLAW_RELEASE); 
		chassis->moveDistance(-18_in);
		back_claw_piston->set_value (BACK_CLAW_GRAB);
		pros::delay(400);
		back_tilter->set_value(BACK_TILTER_UP); 
		chassis->moveDistance(1.1_ft);
		imu_turning_2(22);
		chassis->moveDistance(12_in);
		
		//grabbing left goal
		imu_turning_2(103);
		chassis->moveDistance(5.25_ft);
		front_claw_piston->set_value(FRONT_CLAW_GRAB);
		pros::delay(400);

        //grabbing rings
		// chassis->setMaxVelocity(move_vel*0.7);
	    lift_front_control->setTarget(FRONT_LIFT_PLAT);
		chassis->moveDistance(1.35_ft);
		imu_turning_2(180);
		intake->moveVoltage(INTAKE_IN);
		chassis->moveDistance(2.5_ft);
		chassis->setMaxVelocity(move_vel);
		imu_turning_2(90);

		//placing the left goal onto the balance
		chassis->moveDistanceAsync(2.25_ft);
		pros::delay(750);
		chassis->stop();
	    lift_front_control->setTarget(FRONT_LIFT_PLAT_PLACE);
		imu_turning_2(90);
		front_claw_piston->set_value (FRONT_CLAW_RELEASE);

		//place the alliance goal 
		//drop, move back, claw dowm, turn around and lift the goal, and drop
		//Drop alliance
		back_tilter->set_value(BACK_TILTER_DOWN);
		chassis->moveDistance(-1.15_ft);
		imu_turning_2(90);
		back_claw_piston->set_value (BACK_CLAW_RELEASE);
		pros::delay(500);
		lift_front_control->setTarget(FRONT_LIFT_DOWN);
		intake->moveVoltage(0);
		chassis->moveDistance(0.75_ft);
		//Regrab alliance
		imu_turning_2(-90);
		chassis->moveDistance(1.35_ft);
		imu_turning_2(-90);

		front_claw_piston->set_value (FRONT_CLAW_GRAB);
		pros::delay(250);
		lift_front_control->setTarget(FRONT_LIFT_PLAT);
		//Grab big yellow
		imu_turning_2(90);
		chassis->setMaxVelocity(60);
		chassis->moveDistance(-1.5_ft);
		chassis->setMaxVelocity(move_vel);
		back_claw_piston->set_value(BACK_CLAW_GRAB);
		pros::delay(500);
		back_tilter->set_value(BACK_TILTER_UP);
		//Place Yellow
		imu_turning_2(89);
		chassis->moveDistanceAsync(4.25_ft);
		int timer = 0;
		while(!chassis->isSettled() && timer <= 1500) {
			pros::delay(100);
			timer = timer+100;
		}
		chassis->stop();
		lift_front_control->setTarget(FRONT_LIFT_PLAT_PLACE);
		pros::delay(500);
		front_claw_piston->set_value (FRONT_CLAW_RELEASE);
		pros::delay(500);

		//grab Alliance
		chassis->moveDistance(-5_in);
		imu_turning_2(0);
		lift_front_control->setTarget(FRONT_LIFT_DOWN);
		chassis->moveDistanceAsync(5_ft);
		timer = 0;
		while(!chassis->isSettled() && timer <= 2000) {
			pros::delay(100);
			timer = timer+100;
		}
		front_claw_piston->set_value(FRONT_CLAW_GRAB);
		pros::delay(500);
		chassis->moveDistance(-1_ft);
		lift_front_control->setTarget(FRONT_LIFT_PLAT);

		//Go to balance
		imu_turning_2(-87);
		chassis->setMaxVelocity(130);
		intake->moveVoltage(INTAKE_IN);
		chassis->moveDistanceAsync(10_ft);
		timer = 0;
		while(!chassis->isSettled() && timer <= 4000) {
			pros::delay(100);
			timer = timer+100;
		}
		chassis->stop();
		chassis->moveDistance(-.6_ft);
		intake->moveVoltage(0);
		imu_turning_2(-180);
		// chassis->moveDistanceAsync(-4_ft);
		// timer = 0;
		// while(!chassis->isSettled() && timer <= 750) {
		// 	pros::delay(100);
		// 	timer = timer+100;
		// }
		// imu_turning_2(-180);
		chassis->setMaxVelocity(80);		
		chassis->moveDistance(1.2_ft);
		drive_lft->setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
		drive_rt->setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
		lift_front_control->setTarget(FRONT_LIFT_DOWN);
		pros::delay(1500);
		balance(chassis, imu, master);
		drive_lft->moveVelocity(0);
		drive_rt->moveVelocity(0);
		pros::delay(2000);
		back_tilter->set_value(BACK_TILTER_DOWN);
		
	}
	else
	{
		drive_lft->setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
		drive_rt->setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
		int DIST =40;//35
		front_claw_piston->set_value(FRONT_CLAW_RELEASE);
		int starting_angle = 0;
		if (selector::auton == 1) 
		{
			starting_angle = 5;
			//Grab Yellow
			// int timer = 0;
			// int voltage = 0;
			// while(timer < 400) {
			// 	drive_rt->moveVoltage(voltage);
			// 	drive_lft->moveVoltage(voltage);
			// 	voltage = voltage + 100;
			// 	timer = timer + 10;
			// 	pros::delay(10);
			// }
			drive_rt->moveVoltage(12000);
			drive_lft->moveVoltage(12000);
			front_claw_piston->set_value(true);
			int move_time = 100;

			drive_lft->setBrakeMode(okapi::AbstractMotor::brakeMode::brake);
			drive_rt->setBrakeMode(okapi::AbstractMotor::brakeMode::brake);
			int MAX_TIME = 1100;
			int lowest_dist = dist_sensor->get();
			while((dist_sensor->get() > DIST || dist_sensor->get() == 0) && move_time < MAX_TIME) {
				// if(dist_sensor->get() < 350 && dist_sensor->get() != 0) {
				// 	drive_rt->moveVoltage(6000);
				// 	drive_lft->moveVoltage(6000);
				// 	MAX_TIME = MAX_TIME+3;
				// }
				pros::delay(5);
				move_time += 5;
				lowest_dist = dist_sensor->get();
				master->print(0, 0, "Dist: %d\n", lowest_dist);
			}
			front_claw_piston->set_value(false);
			drive_rt->moveVoltage(0);
			drive_lft->moveVoltage(0);
			// if(false && dist_sensor->get() <= DIST) {
			// 	chassis->turnAngleAsync(90_deg);
			// 	pros::delay(3000);
			// 	chassis->stop();
			// 	chassis->moveDistanceAsync(-2_ft);
			// 	pros::delay(3000);
			// 	chassis->stop();
			// 	chassis->turnAngleAsync(45_deg);
			// 	pros::delay(3000);
			// 	chassis->stop();
			// 	chassis->moveDistanceAsync(3_ft);
			// 	pros::delay(3000);
			// 	chassis->stop();
			// }
			// else {
			pros::delay(150);
			lift_front_control->setTarget(FRONT_LIFT_MOVE);
			chassis->moveDistance(-4.5_ft);
			chassis->setMaxVelocity(100);
			chassis->waitUntilSettled();
			chassis->moveDistanceAsync(-3.5_ft);
			pros::delay(1000);
			chassis->stop();
			lift_front_control->setTarget(FRONT_LIFT_PLAT);
			pros::delay(1000);
			chassis->moveDistance(0.05_ft);
			imu_turning_2(-87-starting_angle);
			back_tilter->set_value(BACK_TILTER_DOWN);
			pros::delay(500);
			back_claw_piston->set_value(BACK_CLAW_RELEASE);
			// if (selector::auton < 0)
			// {
			// 	turn_to_goal(camera, drive_lft, drive_rt, BLUE);
			// }
			chassis->moveDistanceAsync(-2.5_ft);
			pros::delay(2000);
			chassis->stop();
			back_claw_piston->set_value(BACK_CLAW_GRAB);
			pros::delay(500);
			back_tilter->set_value(BACK_TILTER_UP);
			chassis->moveDistance(2_ft);

			imu_turning_2(-180-starting_angle);
			chassis->waitUntilSettled();
			
			//}
			chassis->setMaxVelocity(80);
			chassis->moveDistance(-1_ft);
			intake->moveVoltage(INTAKE_IN);
			if(dist_sensor->get() > DIST) {
				front_claw_piston->set_value(false);
			}
			int CYCLES = 2;
			for(int i = 0; i < CYCLES; i++)
			{
				chassis->moveDistance(1.5_ft);
				chassis->moveDistance(-1.5_ft);
			}
			back_tilter->set_value(BACK_TILTER_DOWN);
			
		
		}
		else
		{
			chassis->setMaxVelocity(110);
			//MIDDLE AUTON
			chassis->moveDistance(3.5_ft);
			imu_turning_2(90);
			chassis->moveDistance(2.5_ft);
			imu_turning_2(45);
			chassis->moveDistance(1.5_ft);		
			front_claw_piston->set_value(FRONT_CLAW_GRAB);
			chassis->moveDistance(-1.5_ft);	
			imu_turning_2(90);	
			chassis->moveDistance(-3.5_ft);
			imu_turning_2(0);
			chassis->moveDistance(-3.5_ft);
			imu_turning_2(-90-starting_angle);
			back_tilter->set_value(BACK_TILTER_DOWN);
			pros::delay(500);
			back_claw_piston->set_value(BACK_CLAW_RELEASE);
			intake->moveVoltage(INTAKE_IN);
			chassis->moveDistance(2_ft);
			chassis->setMaxVelocity(80);
			chassis->moveDistance(1.5_ft);
			int CYCLES = 3;
			for(int i = 0; i < CYCLES; i++)
			{
				chassis->moveDistance(1.5_ft);
				chassis->moveDistance(-1.5_ft);
			}
			back_tilter->set_value(BACK_TILTER_DOWN);
		}
		
	}
}



 void autonomous1()
{	
	drive_lft->setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
	drive_rt->setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
	chassis->setMaxVelocity(80);
	lift_front_control->setTarget(FRONT_LIFT_PLAT);
	pros::delay(2000);
	chassis->moveDistance(1.5_ft);
	lift_front_control->setTarget(FRONT_LIFT_DOWN);
	pros::delay(1500);
	balance(chassis, imu, master);
	pros::delay(2000);
	back_tilter->set_value(BACK_TILTER_DOWN);
	drive_lft->moveVoltage(0);
	drive_rt->moveVoltage(0);
	//balance(drive_rt, drive_lft, imu);
	// imu_turning(180, drive_lft, drive_rt, imu, master);
	// front_claw_piston->set_value(true);
	// pros::delay(2000);
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
	chassis->setMaxVelocity(200);
	int intake_flag = 0;
	int chassis_mode_delay = 0;
	int front_claw_timer = 0;
	int back_claw_timer = 0;
	bool front_flag = false;
	bool back_lock = true;
	bool back_tilt = true;
	bool chassis_hold = false;
	int back_timer = 0;
	int front_timer = 0;
	int y_timer = -1000;

	int double_tap = 0;
	int move_volt = 11000;
	while (true)
	{

		// Driving Mechanics
		double y = master->get_analog(ANALOG_LEFT_Y);
		double x = 0; // master->get_analog(ANALOG_LEFT_X);
		double z = -master->get_analog(ANALOG_RIGHT_X);

		front_rt->moveVoltage((y + x + z) / 127 * move_volt);
		back_rt->moveVoltage((y - x + z) / 127 * move_volt);
		front_lft->moveVoltage((y - x - z) / 127 * move_volt);
		back_lft->moveVoltage((y + x - z) / 127 * move_volt);

		if (master->get_digital(DIGITAL_R2) || partner->get_digital(DIGITAL_R2))
		{
			lift_front_control->setTarget(FRONT_LIFT_DOWN);
			intake_flag = 0;
			// front_claw_control->setTarget(0);
		}
		else if (master->get_digital(DIGITAL_R1) || partner->get_digital(DIGITAL_R1))
		{
			lift_front_control->setTarget(FRONT_LIFT_PLAT);
			// front_claw_control->setTarget(1.0/4.0);
		}
		else if (lift_front_control->isSettled())
		{
			lift_front->moveVelocity(0);
		}

		

		if ((master->get_digital(DIGITAL_LEFT) || partner->get_digital(DIGITAL_LEFT)) && chassis_mode_delay <= 0)
		{
			chassis_hold = !chassis_hold;
			chassis_mode_delay = 200;
		}

		if (chassis_hold)
		{
			drive_lft->setBrakeMode(okapi::AbstractMotor::brakeMode::brake);
			drive_rt->setBrakeMode(okapi::AbstractMotor::brakeMode::brake);
			move_volt = 6000;
		}
		else
		{
			drive_lft->setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
			drive_rt->setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
			move_volt = 11000;
		}

		// Intake
		if (master->get_digital(DIGITAL_UP) || partner->get_digital(DIGITAL_UP))
		{
			intake_flag = -1;
		}
		else if (master->get_digital(DIGITAL_DOWN) || partner->get_digital(DIGITAL_DOWN))
		{
			intake_flag = 1;
		}
		else if (master->get_digital(DIGITAL_RIGHT) || partner->get_digital(DIGITAL_RIGHT))
		{
			intake_flag = 0;
			double_tap++;
		}
		if (intake_flag == 1)
		{
			intake->moveVoltage(12000);
		}
		else if (intake_flag == -1)
		{
			intake->moveVoltage(-12000);
		}
		else
		{
			intake->moveVoltage(0);
		}

		
		if (front_claw_timer <= 0 && (master->get_digital(DIGITAL_B) || partner->get_digital(DIGITAL_B)))
		{
			front_claw_timer = 300;
			front_flag = !front_flag;
			front_claw_piston->set_value(front_flag);
		}	

		if (y_timer == -1000 && (master->get_digital(DIGITAL_L1) || partner->get_digital(DIGITAL_L1)))
		{
			if(back_lock) {
				back_tilt = true;	
				back_tilter->set_value(!back_tilt);
			}
			else {
				y_timer = 500;
			}
			back_lock = true;
			back_claw_piston->set_value(!back_lock);
		}
		else if (master->get_digital(DIGITAL_L2) || partner->get_digital(DIGITAL_L2))
		{
			back_tilt = false;
			back_tilter->set_value(!back_tilt);
		}

		if (!back_tilt && back_claw_timer <= 0 && (master->get_digital(DIGITAL_A) || partner->get_digital(DIGITAL_A))) {
			back_lock = !back_lock;
			back_claw_piston->set_value(!back_lock);
			back_claw_timer = 300;
		}

		if(y_timer != -1000 && y_timer <= 0) {
			back_tilt = true;
			back_tilter->set_value(!back_tilt);
			y_timer = -1000;
		}

		pros::delay(20);

		if (chassis_mode_delay > 0)
		{
			chassis_mode_delay = chassis_mode_delay - 20;
		}
		if (front_claw_timer > 0)
		{
			front_claw_timer = front_claw_timer - 20;
		}
		if (back_claw_timer > 0)
		{
			back_claw_timer = back_claw_timer - 20;
		}
		if (y_timer > 0) {
			y_timer = y_timer - 20;
		}
	}
}