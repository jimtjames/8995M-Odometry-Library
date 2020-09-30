#include "main.h"
#include <math.h>
#include "odom/odometry.h"
pros::Controller controller(pros::E_CONTROLLER_MASTER);
pros::Motor left_drive_1(12, pros::E_MOTOR_GEARSET_18);
pros::Motor left_drive_2(13, pros::E_MOTOR_GEARSET_18);
pros::Motor right_drive_1(17, pros::E_MOTOR_GEARSET_18, true);
pros::Motor right_drive_2(18, pros::E_MOTOR_GEARSET_18, true);
pros::Motor left_roller(1);
pros::Motor right_roller(9, true);
pros::Motor lift(14, pros::E_MOTOR_GEARSET_36);
pros::Motor tilter(16, pros::E_MOTOR_GEARSET_36, true);
double max_speed = 200;
double max_voltage = 12000;
double max_integral = 1000000;
pros::ADIPotentiometer tilter_pot (1);
pros::ADIPotentiometer lift_pot (2);

/*
* Motor ports. We don't care about gearing since our algos use
* voltage and not rpm. Negative means reversed
*/
int8_t left_1 = 12;
int8_t left_2 = 13;
int8_t right_1 = -17;
int8_t right_2 = -18;
//IMU port
int8_t inertial = 20;
//Encoder ports, top port, Negative means reversed
int8_t encoder_l = -3;
int8_t encoder_r = -7;
int8_t encoder_s = -5;
//Radius of tracking wheels
double radius = 3.25/2;
//Distance from tracking center to wheels
double s_l = 2.39;
double s_r = 2.39;
double s_s = -5.85;
//Initial values
double x_init = 0;
double y_init = 0;
double theta_init = 0;
//PID u Constants
double kP_u = 20000;
double kI_u = 0;
double kD_u = 450000;
//PID v Constants
double kP_v = 2000;
double kI_v = 0;
double kD_v = 4750000;
//PID turning Constants
double kP_t = 12500;
double kI_t = 0.0;
double kD_t = 500000;
chassis chassis (
	left_1, left_2,
	right_1, right_2,
	encoder_l, encoder_r, encoder_s,
	inertial, radius, s_l, s_r, s_s,
	x_init, y_init, theta_init,
	kP_u, kI_u, kD_u,
	kP_v, kI_v, kD_v,
	kP_t, kI_t, kD_t);


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
	left_roller.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
	left_roller.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
	right_roller.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
	right_roller.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
	lift.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	tilter.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

	left_drive_1.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
	left_drive_2.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

	right_drive_1.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
	right_drive_2.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

	pros::Task odometry (start_odometry, &chassis, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Odometry");
	pros::delay(3000);
	chassis.set_position(0.00,0.00,0.00);
	pros::delay(500);
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
void autonomous() {}

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
	chassis.set_position(21, -21, -1*M_PI/2);
	chassis.set_lookahead_distance(8);
	std::vector<std::tuple<double, double>> path;
	//path.push_back(std::make_tuple(21, -21));
	path.push_back(std::make_tuple(31.4, -23.5));
	path.push_back(std::make_tuple(38.8, -29.4));
	path.push_back(std::make_tuple(45.2,-36.6));
	path.push_back(std::make_tuple(52.6,-42.5));
	path.push_back(std::make_tuple(63, -45));
	chassis.follow_path(path);
	while(true) {
		pros::delay(5);
	}
}
