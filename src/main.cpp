#include "main.h"

/**
pros make to build without upload
pros mu to build and upload

Motor 1 = front right red gear motor
Motor 2 = front right green gear motor. is inverted

Motor 11 = back right red motor
Motor 3 = back right green motor. is inverted

motor 10 = front left red motor. is inverted
motor 9 = front left green motor

motor 20 = back left red motor. is inverted
motor 8 = back left green motor

motor 12 = right elevator motor
motor 19 = left elevator motor. is inverted

motor 13 = right lift motor
motor 14 left lift motor. is inverted
*/
std::shared_ptr<okapi::MotorGroup> rallm; //motor group for all right motors
std::shared_ptr<okapi::MotorGroup> lallm; //motor group for all left motors

std::shared_ptr<okapi::MotorGroup> gele; //motor group for elevator motors
std::shared_ptr<okapi::MotorGroup> glift; //motor group for lift motors

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

	okapi::Motor frredm (1,false,okapi::AbstractMotor::gearset::green,okapi::AbstractMotor::encoderUnits::rotations);
	okapi::Motor frgrem (2,true,okapi::AbstractMotor::gearset::green,okapi::AbstractMotor::encoderUnits::rotations);
	okapi::Motor brredm (11,false,okapi::AbstractMotor::gearset::green,okapi::AbstractMotor::encoderUnits::rotations);
	okapi::Motor brgrem (3,true,okapi::AbstractMotor::gearset::green,okapi::AbstractMotor::encoderUnits::rotations);
	rallm.reset(new okapi::MotorGroup({frredm, frgrem, brredm, brgrem}));

	okapi::Motor flredm (10,true,okapi::AbstractMotor::gearset::green,okapi::AbstractMotor::encoderUnits::rotations);
	okapi::Motor flgrem (9,false,okapi::AbstractMotor::gearset::green,okapi::AbstractMotor::encoderUnits::rotations);
	okapi::Motor blredm (20,true,okapi::AbstractMotor::gearset::green,okapi::AbstractMotor::encoderUnits::rotations);
	okapi::Motor blgrem (8,false,okapi::AbstractMotor::gearset::green,okapi::AbstractMotor::encoderUnits::rotations);
	lallm.reset(new okapi::MotorGroup({flredm, flgrem, blredm, blgrem}));

	okapi::Motor rele (12,false,okapi::AbstractMotor::gearset::green,okapi::AbstractMotor::encoderUnits::rotations);
	okapi::Motor lele (19,true,okapi::AbstractMotor::gearset::green,okapi::AbstractMotor::encoderUnits::rotations);
	gele.reset(new okapi::MotorGroup({rele, lele}));

	okapi::Motor rlift (13,false,okapi::AbstractMotor::gearset::green,okapi::AbstractMotor::encoderUnits::rotations);
	okapi::Motor llift (14,true,okapi::AbstractMotor::gearset::green,okapi::AbstractMotor::encoderUnits::rotations);
	glift.reset(new okapi::MotorGroup({rlift, llift}));
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
	/** green wheels are 2.75in diameter
	white 3.25in diameter
	*/
	enum ports{motorfrp=1,motorflp=2,motorbrp=10,motorblp=20};
	char sforwardbot = 'C';
	char sforwardtop = 'D';
	pros::Controller C1(pros::E_CONTROLLER_MASTER);
	pros::ADIEncoder forwardsensor (sforwardtop,sforwardbot,true);
	pros::Motor Mbr(motorbrp,1);
	pros::Motor Mbl(motorblp);
	pros::Motor Mfr(motorfrp,1);
	pros::Motor Mfl(motorflp);

	/** green wheels are 2.75in diameter
	for green: 500.03589393235480037932935110492 tick/ft
	0.00199985643457683597355839567223 ft/tick
	*/

	/**
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
	*/
	double sensortickval = forwardsensor.get_value();
	C1.print(0,0,"s: %f",sensortickval);
	while (sensortickval < 2500.1794696617740018966467555246){
		Mbr.move(30);
		Mbl.move(30);
		Mfr.move(30);
		Mfl.move(30);
		sensortickval = forwardsensor.get_value();
		C1.print(0,0,"s: %f",sensortickval);
		pros::delay(2);
	}
	C1.print(0,0,"s: %f",sensortickval);
	Mbr.move(0);
	Mbl.move(0);
	Mfr.move(0);
	Mfl.move(0);

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

	//rallm -> moveVoltage(1000);
	//lallm -> moveVoltage(1000);

	gele -> moveVoltage(5000);

	pros::delay(300);
/**	enum ports{motorfrp=1,motorflp=2,motorbrp=10,motorblp=20};
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
*/
}
