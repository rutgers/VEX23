#include "main.h"
#include "okapi/api.hpp"

/**
connect controller to robot by taking one motor connector
see if it shows connecting icon
plug in usb c to controller to upload

open pros menu integrated terminal
pros make to build without upload
pros mu to build and upload

use -> to get the object the pointer is referring to
*/

/*
class PIDController{
	public:
	double Kp;
	double Ki;
	double Kd;
	double dt;
	double prev_error;
	double sum;


PIDController (double pKp,
	double pKi,
	double pKd,
	double pdt){
	Kp = pKp;
	Ki = pKi;
	Kd = pKd;
	dt = pdt;
	prev_error = 0;
	sum = 0;
}

//set target in degrees of rotationn
void reset (double perr){
	prev_error = perr;
	sum = 0;
}

double update (double perr){
	double de_dt = (perr - prev_error)/dt;
	sum = sum + perr*dt;
	double output = Kp*perr + Ki*sum + Kd*de_dt;
	prev_error = perr;
	return output;
}
};

double dclamp(double value, double low, double high){
	if(value < low){
		return low;
	}
	else if(value > high){
		return high;
	}
	else{
		return value;
	}
}

void turn_degrees(pros::Controller pC1, okapi::MotorGroup prallm, okapi::MotorGroup plallm, pros::Imu sensor, double target_deg){
	double current_deg = sensor.get_rotation();
	//pC1.print (0, 0, "error:%d %d %d %d", errno, ENXIO, ENODEV, EAGAIN);
	double err = target_deg - current_deg;
	double vKp = 200;
	double vKi = 0;
	double vKd = 0;
	double vdt = 5;
	double timesincegoal = 0;
	bool run = true;
	double errtolerance = 1;
	PIDController PIDControl(vKp, vKi, vKd, vdt);
	PIDControl.reset(err);
	while(run){
		printf("error: %f\n", err);
		int out = dclamp(PIDControl.update(err), -12000, 12000);
		prallm.moveVoltage(-out);
		plallm.moveVoltage(out);
		current_deg = sensor.get_rotation();
		err = target_deg - current_deg;
		if(abs(err) < errtolerance && (timesincegoal/1000 == 1)){
			run = false;
		}
		else if(abs(err) < errtolerance){
			timesincegoal = timesincegoal + vdt;
		}
		else{
			timesincegoal = 0;
		}
		pros::delay(vdt);
	}	
}
*/

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

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	ks.kP = 0.0010;
	ks.kI = 0;
	ks.kD = 0;
	ks.kBias = 0;

	// Drive Motors (front is shooting direction)
	frontFrontLft.reset(new okapi::Motor(1, true, okapi::AbstractMotor::gearset::blue, okapi::AbstractMotor::encoderUnits::rotations));
	frontLft.reset(new okapi::Motor(2, true, okapi::AbstractMotor::gearset::blue, okapi::AbstractMotor::encoderUnits::rotations));
	backLft.reset(new okapi::Motor(3, true, okapi::AbstractMotor::gearset::blue, okapi::AbstractMotor::encoderUnits::rotations));
	backBackLft.reset(new okapi::Motor(4, true, okapi::AbstractMotor::gearset::blue, okapi::AbstractMotor::encoderUnits::rotations));

	frontFrontRt.reset(new okapi::Motor(11, false, okapi::AbstractMotor::gearset::blue, okapi::AbstractMotor::encoderUnits::rotations));
	frontRt.reset(new okapi::Motor(12, false, okapi::AbstractMotor::gearset::blue, okapi::AbstractMotor::encoderUnits::rotations));
	backRt.reset(new okapi::Motor(13, false, okapi::AbstractMotor::gearset::blue, okapi::AbstractMotor::encoderUnits::rotations));
	backBackRt.reset(new okapi::Motor(14, false, okapi::AbstractMotor::gearset::blue, okapi::AbstractMotor::encoderUnits::rotations));

	drive_lft.reset(new okapi::MotorGroup({frontFrontLft, frontLft, backLft, backBackLft}));
	drive_rt.reset(new okapi::MotorGroup({frontFrontRt, frontRt, backRt, backBackRt}));
	chassis = okapi::ChassisControllerBuilder()
				  .withMotors(drive_lft, drive_rt)
				  // Green gearset, 4 in wheel diam, 11.5 in wheel track
				  .withDimensions(okapi::AbstractMotor::gearset::blue, {{3.25_in, 12_in}, okapi::imev5BlueTPR})
				  .withGains(ks, ks)
				  .build();

	// Intake - with gearset
	intakeR.reset(new okapi::Motor(19, false, okapi::AbstractMotor::gearset::blue, okapi::AbstractMotor::encoderUnits::rotations));
	intakeL.reset(new okapi::Motor(20, true, okapi::AbstractMotor::gearset::blue, okapi::AbstractMotor::encoderUnits::rotations));
	intake.reset(new okapi::MotorGroup({intakeR, intakeL}));
	intake->setBrakeMode(okapi::AbstractMotor::brakeMode::hold);

	// flywheel
	flywheelL.reset(new okapi::Motor(16, true, okapi::AbstractMotor::gearset::blue, okapi::AbstractMotor::encoderUnits::rotations));
	flywheelR.reset(new okapi::Motor(17, false, okapi::AbstractMotor::gearset::blue, okapi::AbstractMotor::encoderUnits::rotations));
	flywheel.reset(new okapi::MotorGroup({flywheelR, flywheelL}));
	flywheel->setBrakeMode(okapi::AbstractMotor::brakeMode::hold);

	// Controller Initialization
	master.reset(new pros::Controller(pros::E_CONTROLLER_MASTER));

	// Color Sensor
	color_sensor.reset(new pros::Optical(5));
	color_sensor->set_led_pwm(100);
	// Limit Switch B
	limit_switch.reset(new pros::ADIDigitalIn('B'));


	// IMU
	imu.reset(new pros::Imu(15));

	// indexer 
	indexer.reset(new okapi::Motor(10, true, okapi::AbstractMotor::gearset::green, okapi::AbstractMotor::encoderUnits::rotations)); 
/** old code below:
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");

	pros::lcd::register_btn1_cb(on_center_button);

	pros::Controller controller(pros::E_CONTROLLER_MASTER);
	C1.reset(&controller);

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

	allm.reset(new okapi::MotorGroup({frredm, frgrem, brredm, brgrem, flredm, flgrem, blredm, blgrem}));

	okapi::Motor rele (12,false,okapi::AbstractMotor::gearset::green,okapi::AbstractMotor::encoderUnits::rotations);
	okapi::Motor lele (19,true,okapi::AbstractMotor::gearset::green,okapi::AbstractMotor::encoderUnits::rotations);
	gele.reset(new okapi::MotorGroup({rele, lele}));

	okapi::Motor rlift (13,false,okapi::AbstractMotor::gearset::green,okapi::AbstractMotor::encoderUnits::rotations);
	okapi::Motor llift (14,true,okapi::AbstractMotor::gearset::green,okapi::AbstractMotor::encoderUnits::rotations);
	glift.reset(new okapi::MotorGroup({rlift, llift}));


	imu.reset(new pros::Imu(4));
	
	int ret = imu->reset(true);
	*/
	// C1->print(0,0,"test: %d", ret);
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
	~8.64in circumference
	for green: 500.03589393235480037932935110492 tick/ft
	0.00199985643457683597355839567223 ft/tick
	white 3.25in diameter
	*/

	/** old code for moving absolute
	double inpertick = .0113446401;
	double fttick = inpertick/12;
	fttick = ((1/fttick) *5);

	/** move elevator
	gele -> moveVoltage(5000);

	pros::delay(300);**/
	// double data = imu->get_rotation();
	// printf("test: %f\n", data);

	/*std::shared_ptr<okapi::AsyncPositionController<double, double>> elepid = okapi::AsyncPosControllerBuilder().withMotor(gele).build();
	std::shared_ptr<okapi::AsyncPositionController<double, double>> allmpid = okapi::AsyncPosControllerBuilder().withMotor(allm).build();
	std::shared_ptr<okapi::AsyncPositionController<double, double>> rallmpid = okapi::AsyncPosControllerBuilder().withMotor(rallm).build();
	std::shared_ptr<okapi::AsyncPositionController<double, double>> lallmpid = okapi::AsyncPosControllerBuilder().withMotor(lallm).build();
	//max rotations is 3.9 for elevator

	/**
	allmpid -> setMaxVelocity(300);
	allmpid->setTarget(5);
	allmpid->waitUntilSettled();
	allm->moveVoltage(0);
	pros::delay(50);
	
	rallm->tarePosition();
	lallm->tarePosition();
	rallmpid -> setMaxVelocity(300);
	lallmpid -> setMaxVelocity(300);
	rallmpid->setTarget(10);
	lallmpid->setTarget(10);
	rallmpid->waitUntilSettled();
	lallmpid->waitUntilSettled();
	rallm->moveVoltage(0);
	lallm->moveVoltage(0);
	pros::delay(500);

	/* old code for moving bot forward
	enum ports{motorfrp=1,motorflp=2,motorbrp=10,motorblp=20};
	char sforwardbot = 'C';
	char sforwardtop = 'D';

	pros::ADIEncoder forwardsensor (sforwardtop,sforwardbot,true);
	pros::Motor Mbr(motorbrp,1);
	pros::Motor Mbl(motorblp);
	pros::Motor Mfr(motorfrp,1);
	pros::Motor Mfl(motorflp);
	*/

	/**	
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

	/* code for moving bot forward with sensory
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
	*/
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

bool slowmode_on = false;

bool intake_on = false;
bool flywheel_on = false;
bool indexer_on = false;
bool indexer_roller = false;

int motor_multiple = 250;

void opcontrol() {

	/* voltage is in millivolts for moveVoltage
	min  -12000 mV to max 12000 mV
	*/

	//std::shared_ptr<okapi::AsyncPositionController<double, double>> elepid = okapi::AsyncPosControllerBuilder().withMotor(gele).build();
	//max rotations is 3.9 for elevator
	while(true)
	{
		printf("hi\n");
		int LX = master -> get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
		int LY = master -> get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
		//drive slowmode toggle
		if(master -> get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)){
			if(slowmode_on){
				motor_multiple = 250;
				slowmode_on = false;
			}
			else{
				motor_multiple = 50;
				slowmode_on = true;
			}
		}
		//drive
		if(LX || LY){
			drive_rt->moveVoltage((LY*motor_multiple)-(LX*motor_multiple));
			drive_lft->moveVoltage((LY*motor_multiple)+(LX*motor_multiple));
		}
		else{
			drive_rt->moveVoltage(0);
			drive_lft->moveVoltage(0);
		}
		//intake
		if(master -> get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)){
			if(intake_on){
				intake->moveVoltage(0);
				intake_on = false;
			}
			else{
				intake->moveVoltage(12000);
				intake_on = true;
			}
		}
		//flywheel
		if(master -> get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)){
			if(flywheel_on){
				flywheel->moveVoltage(0);
				flywheel_on = false;
			}
			else{
				flywheel->moveVoltage(12000);
				flywheel_on = true;
			}
		}
		//indexer
		if(master -> get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)){
			if(indexer_on){
				indexer->moveVoltage(0);
				indexer_on = false;
			}
			else{
				indexer->moveVoltage(12000);
				indexer_on = true;
			}
		}
		//indexer roller mode
		if(master -> get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)){
			if(indexer_roller){
				indexer->moveVoltage(0);
				indexer_roller = false;
			}
			else{
				indexer->moveVoltage(5000);
				indexer_roller = true;
			}
		}
		
		pros::delay(5);
	}
		




  	/* move elevator set amount
	gele -> moveVoltage(5000);

	pros::delay(300);
	*/

}
