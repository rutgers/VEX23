#include "main.h"
#include "okapi/api.hpp"
using namespace okapi;

void on_center_button() {

}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");

	pros::lcd::register_btn1_cb(on_center_button);
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
void autonomous() {
std::shared_ptr<OdomChassisController> chassis =
	ChassisControllerBuilder()
    	.withMotors(
        	{1, 10}, // Left motors are 1 & 2
        	{-2, -20}    // Right motors are 3 & 4 (reversed)
    	)
		.withGains(
        	{0.001, 0, 0.0001}, // distance controller gains
        	{0.001, 0, 0.0001}, // turn controller gains
        	{0.001, 0, 0.0001}  // angle controller gains (helps drive straight)
    	)
    	.withSensors(
        	ADIEncoder{'A', 'B'}, // left encoder in ADI ports A & B
        	ADIEncoder{'C', 'D'},  // right encoder in ADI ports C & D 
        	ADIEncoder{'E', 'F'}  // middle encoder in ADI ports E & F
		)
		.withDimensions(AbstractMotor::gearset::green,{{2.75_in, 2.75_in, 5.5_in, 2.75_in}, quadEncoderTPR})
    	.buildOdometry();


chassis->setState(f


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
void opcontrol() {
    // Chassis Controller - lets us drive the robot around with open- or closed-loop control
std::shared_ptr<ChassisController> drive =
	ChassisControllerBuilder()
    	.withMotors(
        	{1, 10}, // Left motors are 1 & 2 (reversed)
        	{-2, -20}    // Right motors are 3 & 4
    	)
		.withDimensions(AbstractMotor::gearset::green, {{3.25_in, 11.5_in}, imev5GreenTPR})
		.build();
			
		Controller controller;

		while (true){
        drive->getModel()->tank(controller.getAnalog(ControllerAnalog::leftY),
                                controller.getAnalog(ControllerAnalog::rightY));			
			pros::delay(10);
		}

    // int speed = 100;
    // pros::Controller master(pros::E_CONTROLLER_MASTER);
    // pros::Motor leftBack_mtr(1,pros::E_MOTOR_GEAR_GREEN,0,pros:: E_MOTOR_ENCODER_ROTATIONS);
    // pros::Motor rightBack_mtr(2,pros::E_MOTOR_GEAR_GREEN,1,pros:: E_MOTOR_ENCODER_ROTATIONS);
    // pros::Motor leftFront_mtr(10,pros::E_MOTOR_GEAR_GREEN,0,pros:: E_MOTOR_ENCODER_ROTATIONS);
    // pros::Motor rightFront_mtr(20,pros::E_MOTOR_GEAR_GREEN,1,pros:: E_MOTOR_ENCODER_ROTATIONS);

    // while (true) {
    //     pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
    //                      (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
    //                      (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);
    //     int left = (master.get_analog(ANALOG_LEFT_Y)/127)*speed;
    //     int right = (master.get_analog(ANALOG_RIGHT_Y)/127)*speed;

    //     leftBack_mtr = left;
    //     rightBack_mtr = right;
    //     leftFront_mtr = left;
    //     rightFront_mtr = right;
    //     pros::delay(20);

	// 	//speed increase and decrease
	// 	if(master.get_digital(DIGITAL_UP) && speed != 125 ){
    //     speed = speed+5;
	// 	}
	// 	if(master.get_digital(DIGITAL_DOWN) && speed != 0){
	// 		speed = speed-5;
	// 	}

	// 	//move 5 feet
	// 	if(master.get_digital(DIGITAL_LEFT)){
		
	// 		double distanceTraveled = leftBack_mtr.get_position() * 18 * (2*3.14*3.25);
		
	// 		while(distanceTraveled < 60){
	// 		distanceTraveled = leftBack_mtr.get_position() * 18 * (2*3.14*3.25);
	// 		leftBack_mtr = 100;
	// 		rightBack_mtr = 100;
	// 		leftFront_mtr = 100;
	// 		rightFront_mtr = 100;
	// 		master.print(0,0,std::to_string.c_str(distanceTraveled));
	// 		}

	// 		leftBack_mtr = 0;
	// 		rightBack_mtr = 0;
	// 		leftFront_mtr = 0;
	// 		rightFront_mtr = 0;
	// 	}
    // }
}