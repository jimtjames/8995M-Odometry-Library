#include "main.h"
#include <stdint.h>
#include <tuple>
#include <vector>
#include <cmath>
#include "odometry.h"

#define INTEGRAL_MAX 10000
#define TURN_INTEGRAL_MAX 500

/*
* Constructor
* l1, l2, l3, r1, r2, r3 are motor ports, use negative to reverse
* l, r, s are encoder ports
* w_r is wheel radius
* sl, sr, ss are the distances from tracking center to wheel axis
* x_init, y_init, theta_init are initial positions
* PID Constants can also be passed through
*/
chassis::chassis(int8_t l1, int8_t l2, int8_t r1, int8_t r2,
int8_t l, int8_t r, int8_t s, int8_t imu, double w_r, double sl,
double sr, double ss, double x_init, double y_init, double theta_init,
double kPu, double kIu, double kDu, double kPv, double kIv, double kDv,
double kPt, double kIt, double kDt) {
    moving = false;
    left_1 = l1;
    left_2 = l2;
    right_1 = r1;
    right_2 = r2;
    left_enc = l;
    right_enc = r;
    side_enc = s;
    inertial = imu;
    wheel_radius = w_r;
    s_l = sl;
    s_r = sr;
    s_s = ss;
    x_prev = x_init;
    y_prev = y_init;
    theta_reset = theta_init;
    x = x_prev;
    y = y_prev;
    theta = theta_reset;
    kP_u = kPu;
    kI_u = kIu;
    kD_u = kDu;
    kP_v = kPv;
    kI_v = kIv;
    kD_v = kDv;
    kP_t = kPt;
    kI_t = kIt;
    kD_t = kDt;
    voltage_max = 12000;
    lookahead_distance = 6;
}
/*
* Returns a tuple formatted as x, y, theta
*/
std::tuple<double, double, double> chassis::get_position() {
    std::tuple<double, double, double> pos = std::make_tuple(x, y, theta);
    return pos;
}

/*
* Updates the max voltage
*/
void chassis::set_voltage_max(int16_t max) {
    voltage_max = max;
}

/*
* Updates the lookahead distance
*/
void chassis::set_lookahead_distance(uint8_t dist) {
    lookahead_distance = dist;
}
/*
* Task to update the absolute position and orientation
* Uses encoders and tracking wheels on the left drive, right drive
* and back to accurately track
* Theta is in radians, x and y are in inches
*/
void chassis::update_odometry() {
    pros::Imu imu(inertial);
    imu.reset();
    pros::delay(2500);
    int right_port = std::abs(right_enc);
    bool right_rev = right_enc<0;
    int left_port = std::abs(left_enc);
    bool left_rev = left_enc<0;
    int side_port = std::abs(side_enc);
    bool side_rev = side_enc<0;

    //Sensor declarations
    pros::ADIEncoder right (right_port, right_port+1, right_rev);
    pros::ADIEncoder left (left_port, left_port+1, left_rev);
    pros::ADIEncoder side (side_port, side_port+1, side_rev);

    double right_value, left_value, side_value;

    //Store the initial values. For an actual program, this should probably be done in auton
    double right_value_prev = right.get_value();
    double left_value_prev = left.get_value();
    double side_value_prev = side.get_value();
    theta_prev = theta_reset;

    //Values of encoders when odometry first starts
    double theta_r = theta_reset;
    double theta_imu = theta_r;
    double right_value_r = right_value_prev;
    double left_value_r = left_value_prev;
    int i = 0;
    while(true) {
        //pros::lcd::print(0, "(%f %f %f)", x, y, theta * 180/M_PI);
        if(i==5) {
          std::cout << "x:" << x << "\n";
          std::cout << "y:" << y << "\n";
          std::cout << "t:" << theta * 180/M_PI << "\n";
          std::cout << "i:" << theta_imu * 180/M_PI<< "\n";
          i=0;
        }
        else {
          i++;
        }
        //Store the time at the start of the loop
        uint32_t time = pros::millis();

        //Find the new encoder values
        right_value = right.get_value();
        left_value = left.get_value();
        side_value = side.get_value();

        //Calculate the arc lengths, circumference * fraction
        double delta_r = (right_value - right_value_prev) / 360.0 * M_PI * 2 * wheel_radius;
        double delta_l = (left_value - left_value_prev) / 360.0 * M_PI * 2 * wheel_radius;
        double delta_s = (side_value - side_value_prev) / 360.0 * M_PI * 2 * wheel_radius;

        //Calculate the change in left and right distances since last reset
        double delta_r_r  = (right_value - right_value_r) / 360.0 * M_PI * 2 * wheel_radius;
        double delta_l_r  = (left_value - left_value_r) / 360.0 * M_PI * 2 * wheel_radius;

        //Update previous encoder values
        right_value_prev = right_value;
        left_value_prev = left_value;
        side_value_prev = side_value;

        //Calculate orientation
        //theta = theta_r + (delta_l_r - delta_r_r)/(s_l + s_r) + theta_reset;
        theta = theta_reset + -1 * imu.get_rotation()*M_PI/180.0;

        //Calculate change in orientation from last loop
        double delta_theta = theta-theta_prev;

        //Update the position of the robot
        double delta_d_l_x, delta_d_l_y;

        //Make our aligned displacement vector
        if(std::abs(delta_theta) <= 1E-6) {
            delta_d_l_x = delta_s;
            delta_d_l_y = delta_r;
        }
        else {
            delta_d_l_x = 2 * sin(delta_theta/2) * (delta_s/delta_theta + s_s);
            delta_d_l_y = 2 * sin(delta_theta/2) * (delta_r/delta_theta + s_r);
        }

        //Calculate the average oreintation
        double theta_m = theta_prev + delta_theta/2;

        //Hang the vector on the perpframe defined by our robot's axes
        double delta_d_x = delta_d_l_x * cos(-1 * theta_m) - delta_d_l_y * sin(-1 * theta_m);
        double delta_d_y = delta_d_l_y * cos(-1 * theta_m) + delta_d_l_x * sin(-1 * theta_m);

        //Use our change to update the current position
        x = x_prev + delta_d_x;
        y = y_prev - delta_d_y;

        //Update previous value for orientation
        theta_prev = theta;

        //Update the previous values
        x_prev = x;
        y_prev = y;

        //Run for a consistent 10 ms each cycle
        pros::Task::delay_until(&time, 10);
    }
}

/*
* Reset chassis state to new values
*/
void chassis::set_position(double new_x, double new_y, double new_theta) {
  x = new_x;
  x_prev = new_x;
  y = new_y;
  y_prev = new_y;
  theta_reset = new_theta - theta;
  theta_prev = theta_reset;
}
/*
* Use a matrix hit to convert {x,y} coordinates to robot perpframe coords
* via a hit with an aligner frame
*/
std::tuple<double, double> matrix_hit(double x_disp, double y_disp, double theta) {
    double u = std::cos(theta)*(x_disp) + std::sin(theta)*y_disp;
    double v = -1 * std::sin(theta)*(x_disp) + std::cos(theta)*y_disp;
    std::tuple<double, double> pos = std::make_tuple(u, v);
    return pos;
}

/*
* Drives in an arc to the target x and y and continuously readjusts
*/
void chassis::drive_to(double x_t, double y_t) {
    pros::Motor left_drive_1(std::abs(left_1), left_1<0);
    pros::Motor left_drive_2(std::abs(left_2), left_2<0);
    pros::Motor right_drive_1(std::abs(right_1), right_1<0);
    pros::Motor right_drive_2(std::abs(right_2), right_2<0);

    left_drive_1.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    left_drive_2.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    right_drive_1.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    right_drive_2.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

		//Field centric displacement vector
    double x_disp = x_t - x;
    double y_disp = y_t - y;

    //Hit with aligner to convert to robot coordinates
    std::tuple<double, double> perpframe_coords = matrix_hit(x_disp, y_disp, theta);
    //+v is in front of the robot, +u is the right of the robot
    double u = std::get<0>(perpframe_coords);
    double v = std::get<1>(perpframe_coords);
    double u_prev = std::get<0>(perpframe_coords);
    double v_prev = std::get<1>(perpframe_coords);

    //angle change necessary...?
    double t = theta-angle_to(x_t, y_t);
    double t_prev = t;

    //Speeds in direction of u and v
    double speed_t = 12000;
    double speed_v = 12000;
    //t terms
    double t_integral = 0;
    double t_deriv = 0;
    double u_deriv = 0;
    //v terms
    double v_integral = 0;
    double v_deriv = 0;
    //Time interval between iterations
    uint8_t dt = 10;
    int16_t lowcount = 0;
    //Loop as long as error is significant and speed isn't minimal, this condition is probably wrong tho
    while(std::sqrt(u*u+v*v)>1.5) {
				//std::cout << "v: " <<  v << ", " << speed_v << "\n";
        //std::cout << "u: " <<  u << ", " << speed_u << "\n";
        //Calculate field centric displacement vector
        x_disp = x_t - x;
        y_disp = y_t - y;

        //Hit with aligner to convert to robot coordinates
        perpframe_coords = matrix_hit(x_disp, y_disp, theta);
        u = std::get<0>(perpframe_coords);
        v = std::get<1>(perpframe_coords);
        //Calculate the angle change needed
        t = theta-angle_to(x_t, y_t);

        //Integrals: Limit them to the end range
        if(std::abs(t)<M_PI/12) {
          t_integral += t*dt;
        }
        if(std::abs(v)<5) {
          v_integral += v*dt;
        }
        //Disable integral if it is too large
        if(std::abs(t_integral) > INTEGRAL_MAX) {
            t_integral = 0;
        }
        if(std::abs(v_integral) > INTEGRAL_MAX) {
            v_integral = 0;
        }

        //We're stuck, break the loop
        if(std::abs(1000*v_deriv)<0.5 && std::abs(1000*u_deriv)<0.5) {
          lowcount++;
        }
        if(lowcount>7) {
          std::cout << "something is wrong\n";
          break;
        }
        //Derivatives
        t_deriv = (t-t_prev)/dt;
        u_deriv = (u-u_prev)/dt;
        v_deriv = (v-v_prev)/dt;

        //Speeds
        speed_t = kP_u * t + kI_u * t_integral * kD_u * t_deriv;
        speed_v = kP_v * v + kI_v * v_integral * kD_v * v_deriv;

        //Limit the voltage to max_voltage
        speed_t = speed_t < voltage_max ? speed_t:voltage_max;
        speed_v = speed_v < voltage_max ? speed_v:voltage_max;
        speed_t = speed_t > -1 * voltage_max ? speed_t:-1 * voltage_max;
        speed_v = speed_v > -1 * voltage_max ? speed_v:-1 * voltage_max;

        double left_speed = speed_v + speed_t;
        double right_speed = speed_v - speed_t;

				//std::cout << "l: " << left_speed << ", " << "r: " << right_speed << "\n";
        //Set the drives to their speeds
        left_drive_1.move_voltage(left_speed);
        left_drive_2.move_voltage(left_speed);
        right_drive_1.move_voltage(right_speed);
        right_drive_2.move_voltage(right_speed);

        //Update previous values
        u_prev = u;
        v_prev = v;
        t_prev = t;

        //Wait for 5 ms between cycles
        pros::Task::delay(dt);
    }
    left_drive_1.move_voltage(0);
    left_drive_2.move_voltage(0);
    right_drive_1.move_voltage(0);
    right_drive_2.move_voltage(0);
}

/*
* Calculate angle to target
*/
double chassis::angle_to(double x_t, double y_t) {
	double hypotenuse = sqrt((x-x_t)*(x-x_t) + (y-y_t)*(y-y_t));
	double cos = (x_t-x)/hypotenuse;
	double sin = (y_t-y)/hypotenuse;
	double theta = 0;
	if(cos>0 && sin>0) {
		theta = acos(cos);
	}
	else if(cos<0 && sin>0) {
		theta = acos(cos);
	}
	else if(cos<0 && sin<0) {
		theta = M_PI-asin(sin);
	}
	else {
		theta = asin(sin);
	}
	return theta-M_PI/2;
}

/*
* Supposedly follows a path. 0% chance of working.
*/
void chassis::follow_path(std::vector<std::tuple<double, double>> path) {
    int i = 0;
    double x_t = std::get<0>(path[i]);
    double y_t = std::get<1>(path[i]);
    pros::Motor left_drive_1(std::abs(left_1), left_1<0);
    pros::Motor left_drive_2(std::abs(left_2), left_2<0);
    pros::Motor right_drive_1(std::abs(right_1), right_1<0);
    pros::Motor right_drive_2(std::abs(right_2), right_2<0);

    left_drive_1.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    left_drive_2.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    right_drive_1.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    right_drive_2.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

    //Field centric displacement vector
    double x_disp = x_t - x;
    double y_disp = y_t - y;

    //Hit with aligner to convert to robot coordinates
    std::tuple<double, double> perpframe_coords = matrix_hit(x_disp, y_disp, theta);
    //+v is in front of the robot, +u is the right of the robot
    double u = std::get<0>(perpframe_coords);
    double v = std::get<1>(perpframe_coords);
    double u_prev = std::get<0>(perpframe_coords);
    double v_prev = std::get<1>(perpframe_coords);

    //angle change necessary...?
    double t = theta-angle_to(x_t, y_t);
    double t_prev = t;

    //Speeds in direction of u and v
    double speed_t = 12000;
    double speed_v = 12000;
    //t terms
    double t_integral = 0;
    double t_deriv = 0;
    double u_deriv = 0;
    //v terms
    double v_integral = 0;
    double v_deriv = 0;
    //Time interval between iterations
    uint8_t dt = 10;
    int16_t lowcount = 0;
    //std::cout << "(" << x_t <<", " << y_t << ")\n";
    //Loop as long as error is significant and speed isn't minimal, this condition is probably wrong tho
    while(std::sqrt(u*u+v*v)>2) {
        //std::cout << i << ": (" << std::get<0>(path[i])
        //If we're within lookahead_distance to the next point, target the next point
        if(std::sqrt(u*u + v*v) < lookahead_distance && i < path.size()-1) {
            x_t = std::get<0>(path[++i]);
            y_t = std::get<1>(path[i]);
            std::cout << "(" << x_t <<", " << y_t << ")\n";
        }

        x_disp = x_t - x;
        y_disp = y_t - y;

        //Hit with aligner to convert to robot coordinates
        perpframe_coords = matrix_hit(x_disp, y_disp, theta);
        u = std::get<0>(perpframe_coords);
        v = std::get<1>(perpframe_coords);
        //Calculate the angle change needed
        t = theta-angle_to(x_t, y_t);

        //Integrals: Limit them to the end range
        if(std::abs(t)<M_PI/12) {
          t_integral += t*dt;
        }
        if(std::abs(v)<5) {
          v_integral += v*dt;
        }
        //Disable integral if it is too large
        if(std::abs(t_integral) > INTEGRAL_MAX) {
            t_integral = 0;
        }
        if(std::abs(v_integral) > INTEGRAL_MAX) {
            v_integral = 0;
        }

        //We're stuck, break the loop
        if(std::abs(1000*v_deriv)<0.5 && std::abs(1000*u_deriv)<0.5) {
          lowcount++;
        }
        if(lowcount>7) {
          std::cout << "something is wrong\n";
          break;
        }
        //Derivatives
        t_deriv = (t-t_prev)/dt;
        u_deriv = (u-u_prev)/dt;
        v_deriv = (v-v_prev)/dt;

        //Speeds
        speed_t = kP_u * t + kI_u * t_integral * kD_u * t_deriv;
        speed_v = kP_v * v + kI_v * v_integral * kD_v * v_deriv;

        //Limit the voltage to max_voltage
        speed_t = speed_t < voltage_max ? speed_t:voltage_max;
        speed_v = speed_v < voltage_max ? speed_v:voltage_max;
        speed_t = speed_t > -1 * voltage_max ? speed_t:-1 * voltage_max;
        speed_v = speed_v > -1 * voltage_max ? speed_v:-1 * voltage_max;

        double left_speed = speed_v + speed_t;
        double right_speed = speed_v - speed_t;

				//std::cout << "l: " << left_speed << ", " << "r: " << right_speed << "\n";
        //Set the drives to their speeds
        left_drive_1.move_voltage(left_speed);
        left_drive_2.move_voltage(left_speed);
        right_drive_1.move_voltage(right_speed);
        right_drive_2.move_voltage(right_speed);

        //Update previous values
        u_prev = u;
        v_prev = v;
        t_prev = t;

        //Wait for 5 ms between cycles
        pros::Task::delay(dt);
    }
    left_drive_1.move_voltage(0);
    left_drive_2.move_voltage(0);
    right_drive_1.move_voltage(0);
    right_drive_2.move_voltage(0);
}

/*
* Turn to a field centric angle, in degrees
*/
void chassis::turn_to(double theta_deg) {
  pros::Motor left_drive_1(std::abs(left_1), left_1<0);
  pros::Motor left_drive_2(std::abs(left_2), left_2<0);
  pros::Motor right_drive_1(std::abs(right_1), right_1<0);
  pros::Motor right_drive_2(std::abs(right_2), right_2<0);

  left_drive_1.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  left_drive_2.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  right_drive_1.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  right_drive_2.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  theta_deg*=M_PI/180;
  double error = theta_deg-theta;
  if(error>M_PI) {
		theta_deg-= 2*M_PI;
		error = theta_deg - theta;
	}
	if(error<-1 * M_PI) {
		theta_deg+=2*M_PI;
		error = theta_deg - theta;
	}
	double error_prev = error;
	//Terms
	double integral = 0;
	double deriv = 0;
	double speed = 12000;
	//Time interval between iterations
	uint8_t dt = 20;
  int lowcount = 0;
	while(std::abs(error)>M_PI/200 || std::abs(speed)>250) {
    std::cout << 1000*deriv << "\n";
    error = theta_deg-theta;
		integral += error*dt;
		deriv = (error-error_prev)/dt;
    if(std::abs(1000*deriv)<0.1) {
      lowcount++;
    }
    if(lowcount>7) {
      break;
    }
    if(std::abs(error)<M_PI/200) {
      integral = 0;
    }
    //Cap the integral
    if(std::abs(integral)>TURN_INTEGRAL_MAX) {
      integral = 0;
    }
		speed = kP_t * error + kI_t * integral + kD_t * deriv;
		left_drive_1.move_voltage(-speed);
		left_drive_2.move_voltage(-speed);
		right_drive_1.move_voltage(speed);
		right_drive_2.move_voltage(speed);
		error_prev = error;
		pros::delay(dt);
	}
	left_drive_1.move_voltage(0);
	left_drive_2.move_voltage(0);
	right_drive_1.move_voltage(0);
	right_drive_2.move_voltage(0);
}

/*
* Move forward a certain number of inches
*/
void chassis::move_forward(double distance) {
  pros::Motor left_drive_1(std::abs(left_1), left_1<0);
  pros::Motor left_drive_2(std::abs(left_2), left_2<0);
  pros::Motor right_drive_1(std::abs(right_1), right_1<0);
  pros::Motor right_drive_2(std::abs(right_2), right_2<0);

  left_drive_1.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  left_drive_2.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  right_drive_1.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  right_drive_2.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

	//Get the current state
	double x_init = x;
	double y_init = y;

	//Calculate error
	double distance_travelled = std::sqrt((x-x_init)*(x-x_init) + (y-y_init)*(y-y_init));
	distance_travelled *= ((distance>0) - (distance<0));
	double error = distance-distance_travelled;
	double error_prev = error;
	//Terms
	double integral = 0;
	double deriv = 0;
	double speed = 12000;

	//Time interval between iterations
	uint8_t dt = 20;
  int lowcount = 0;
	while(std::abs(error)>.2 && std::abs(speed)>700) {
    std::cout << error << "\n";
		distance_travelled = std::sqrt((x-x_init)*(x-x_init) + (y-y_init)*(y-y_init));
		distance_travelled *= ((distance>0) - (distance<0));
		error = distance-distance_travelled;
		integral += error*dt;
		if(std::abs(error)<0.2) {
			integral = 0;
		}
		//Cap the integral
		if(std::abs(integral)>1000) {
			integral = 0;
		}
		deriv = (error-error_prev)/dt;
    //We're stuck, break the loop
    if(std::abs(1000*deriv)<0.5) {
      lowcount++;
    }
    if(lowcount>10) {
      std::cout << "something is wrong\n";
      break;
    }
		speed = kP_v * error + kI_v * integral + kD_v * deriv;
		if(speed>voltage_max) {
			speed = voltage_max;
		}
		else if(speed<-1*voltage_max) {
			speed = 1*voltage_max;
		}
		left_drive_1.move_voltage(speed);
		left_drive_2.move_voltage(speed);
		right_drive_1.move_voltage(speed);
		right_drive_2.move_voltage(speed);
		//std::cout << error << "\n";
		error_prev = error;
		pros::delay(dt);
	}
	left_drive_1.move_voltage(0);
	left_drive_2.move_voltage(0);
	right_drive_1.move_voltage(0);
	right_drive_2.move_voltage(0);
}

/*
* Turn to a point, then drive to it.
*/
void chassis::drive_to_point(double x_t, double y_t, bool backwards) {
	double target_theta = angle_to(x_t, y_t);
	if(backwards) {
		target_theta+=M_PI;
	}
	turn_to(target_theta*180/M_PI);

	drive_to(x_t,y_t);
}
void drive_to_point_helper(void* params) {
	struct Param* parameters = (struct Param*) params;
  chassis* base = parameters->base;
  double x = parameters->x;
	double y = parameters->y;
	bool backwards = parameters->backwards;
	base->drive_to_point(x, y, backwards);
	base->moving = false;
}
void chassis::wait_until_point(double x_t, double y_t) {
	while(sqrt( (x_t-x)*(x_t-x) + (y_t-y)*(y_t-y) ) > 6) {
		pros::Task::delay(5);
	}
}
void chassis::drive_to_point_async(double x_t, double y_t, bool backwards) {
  moving = true;
	struct Param parameters = {this, x_t, y_t, backwards};
	pros::Task drive (drive_to_point_helper, &parameters, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Drive to point");
}

/*
* Call this task to start tracking
*/
void start_odometry(void* base) {
    pros::lcd::print(1, "odom task started");
    std::cout << "tracking started!" << "\n";
    ((chassis*) (base))->update_odometry();
}
