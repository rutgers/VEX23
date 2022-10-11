#include "main.h"

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
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
	enum ports{motorfrp=1,motorflp=2,motorbrp=10,motorblp=20};
	pros::Motor Mbr(motorbrp,1);
	pros::Motor Mbl(motorblp);
	pros::Motor Mfr(motorfrp,1);
	pros::Motor Mfl(motorflp);

	double inpertick = .0113446401;
	double fttick = inpertick/12;
	fttick = ((1/fttick) *5);
	
	Mbr.tare_position();
	Mbl.tare_position();
	Mfr.tare_position();
	Mfl.tare_position();

	Mbr.move_absolute(fttick,100);
	Mbl.move_absolute(fttick,100);
	Mfr.move_absolute(fttick,100);
	Mfl.move_absolute(fttick,100);

	pros::delay(200000);

	//while(!((Mbr.get_position() < (5288 + 5)) && (Mbr.get_position() > (5288 - 5)))){
	//	pros::delay(2);
	//}
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
	enum ports{motorfrp=1,motorflp=2,motorbrp=10,motorblp=20};
	pros::Controller C1(pros::E_CONTROLLER_MASTER);
	pros::Motor Mbr(motorbrp,1);
	pros::Motor Mbl(motorblp);
	pros::Motor Mfr(motorfrp,1);
	pros::Motor Mfl(motorflp);
	while(true){
		int LX = C1.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
		int LY = C1.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
		if(C1.get_digital(pros::E_CONTROLLER_DIGITAL_B)){
			Mbr.move(127);
			Mbl.move(127);
			Mfr.move(127);
			Mfl.move(127);
		}
		else if(LX or LY){
			Mbr.move(LY-LX);
			Mbl.move(LY+LX);
			Mfr.move(LY-LX);
			Mfl.move(LY+LX);
		}
		else{
			Mbr.move(0);
			Mbl.move(0);
			Mfr.move(0);
			Mfl.move(0);
		}

		pros::delay(2);
	}
	// pros::Controller master(pros::E_CONTROLLER_MASTER);
	// pros::Motor fleft_mtr(1);
	// pros::Motor fright_mtr(2);
	// pros::Motor bleft_mtr(3);
	// pros::Motor bright_mtr(4);

	// while (true) {
	// 	pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
	// 	                 (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
	// 	                 (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);
	// 	int left = master.get_analog(ANALOG_LEFT_Y);
	// 	int right = master.get_analog(ANALOG_RIGHT_Y);

	// 	left_mtr = left;
	// 	right_mtr = right;
	// 	pros::delay(20);
	// }
}
