#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <string>
#include <cstring>
#include <cassert>
#include <unistd.h>

#ifndef MAIN_H
#define MAIN_H
#include "main.h"
#endif
#ifndef OKAPI_H
#define OKAPI_H
#include "okapi/api.hpp"
#endif
#ifndef VISION_CPP
#define VISION_CPP
#include "vision.cpp"
#endif

char data_received[100];
char previous_data_received[100];

char none_string[] = "NONE\n";
bool detected = false;

double xmin = 0;
double ymin = 0;
double xmax = 0;
double ymax = 0;


// int is_ready(int fd) {
//     fd_set fdset;
//     struct timeval timeout;
//     int ret;
//     FD_ZERO(&fdset);
//     FD_SET(fd, &fdset);
//     timeout.tv_sec = 0;
//     timeout.tv_usec = 1;
//     //int select(int nfds, fd_set *readfds, fd_set *writefds, fd_set *exceptfds,struct timeval *timeout);
//     return select(1 , &fdset, NULL, NULL, &timeout) == 1 ? 1 : 0;
// }

// Reads data being sent from the Jetson
// Credited to: https://stackoverflow.com/questions/9210528/split-string-with-delimiters-in-c
char** str_split(char* a_str, const char a_delim)
{
    char** result    = 0;
    size_t count     = 0;
    char* tmp        = a_str;
    char* last_comma = 0;
    char delim[2];
    delim[0] = a_delim;
    delim[1] = 0;
 
    /* Count how many elements will be extracted. */
    while (*tmp)
    {
        if (a_delim == *tmp)
        {
            count++;
            last_comma = tmp;
        }
        tmp++;
    }
 
    /* Add space for trailing token. */
    count += last_comma < (a_str + strlen(a_str) - 1);
 
    /* Add space for terminating null string so caller
       knows where the list of returned strings ends. */
    count++;
 
    result = (char**)malloc(sizeof(char*) * count);
 
    if (result)
    {
        size_t idx  = 0;
        char* token = strtok(a_str, delim);
 
        while (token)
        {
            assert(idx < count);
            *(result + idx++) = strdup(token);
            token = strtok(0, delim);
        }
        assert(idx == count - 1);
        *(result + idx) = 0;
    }
 
    return result;
}
 
bool read_from_jetson()
{
    char** parsed_data_received;


    fgets(data_received, sizeof(data_received), stdin);
 
    if (strcmp(data_received, previous_data_received) != 0)
    {
		//printf("%s\n",data_received);
        parsed_data_received = str_split(data_received, '_');
        //printf(parsed_data_received[1]);
        
		if(strcmp(parsed_data_received[1], none_string) == 0) {
			detected = false;
            // xmin = 0;
			// ymin = 0;
			// xmax = 0;
			// ymax = 0;
		}
		else {
			xmin = strtod(parsed_data_received[2], nullptr);
			ymin = strtod(parsed_data_received[3], nullptr);
			xmax = strtod(parsed_data_received[4], nullptr);
			ymax = strtod(parsed_data_received[5], nullptr);
			detected = true;
		}
        
        strcpy(previous_data_received, data_received);
        return true;
    }
    return false;
}
 

class PID_Controller {
    public:
    double kp = 0;
    double ki = 0;
    double kd = 0;
    double dt = 0;
    double sum = 0;
    double last_err = 0;

    PID_Controller(double kp_, double ki_, double kd_, double dt_) {
        kp = kp_;
        ki = ki_;
        kd = kd_;
        dt = dt_;
    }

    void reset(double err) {
        sum = 0;
        last_err = err;
    }

    double update(double err) {
        sum = sum + err*dt;
        double output = kp*err + ki*sum + kd*(err-last_err)/dt;
        last_err = err;
        return output;
    }
};

void turn_to_goal(std::shared_ptr<GoalCamera> camera,
                  std::shared_ptr<okapi::MotorGroup> lft,
                  std::shared_ptr<okapi::MotorGroup> rt,
                  goal_color c)
{
    double x, y;
    std::tie(x, y) = camera->get_by_sig(c);

    int err_thresh = 20;
    int fov_angle = 60;
    int settled_time = 0;
    int dt = 10;
    int settled_thresh = 0;
    int total_time = 0;
    double kp = 9000;
    double ki = 50;

    while (total_time < 2000 && (abs(x) > err_thresh || settled_thresh <= 150))
    {
        std::tie(x, y) = camera->get_by_sig(c);
        double v_prop = x / VISION_FOV_WIDTH * 2;
        double sign = 1;
        if (v_prop < 0)
        {
            sign = -1;
        }
        lft->moveVoltage(-kp * v_prop - ki * dt * sign);
        rt->moveVoltage(kp * v_prop + ki * dt * sign);

        if (abs(x) < err_thresh)
        {
            settled_time += dt;
        }
        else
        {
            settled_time = 0;
        }
        pros::delay(dt);
        total_time += dt;
    }
    lft->moveVoltage(0);
    rt->moveVoltage(0);
}

void balance(std::shared_ptr<okapi::ChassisController> chassis, std::shared_ptr<pros::Imu> imu, std::shared_ptr<pros::Controller> master)
{
    master->print(1, 1, "roll: %f", imu->get_pitch());
    double orig_velocity = chassis->getMaxVelocity();
    chassis->setMaxVelocity(70);
    double original_pitch = imu->get_pitch();
    chassis->moveDistanceAsync(3.2_ft);

    double pitch_change_thresh = 21;

    while (abs(imu->get_pitch() - original_pitch) < pitch_change_thresh)
    {   
        master->print(1, 1, "pitch: %d vs %d", (int) imu->get_pitch(), (int) original_pitch);
        pros::delay(30);
    }
    chassis->stop();
    chassis->setMaxVelocity(40);
    chassis->moveDistanceAsync(33.5_in);
    pros::delay(500);
    original_pitch = imu->get_pitch();
    pitch_change_thresh = 4;

    while (abs(imu->get_pitch() - original_pitch) < pitch_change_thresh)
    {
        // if(chassis->isSettled()) {
        //     chassis->moveDistanceAsync(1_in);
        // }
        master->print(1, 1, "on_roll: %d vs %d", (int) imu->get_pitch(), (int) original_pitch);
        pros::delay(30);
    }
    chassis->stop();
    chassis->setMaxVelocity(orig_velocity);
}

void imu_turning(double target, std::shared_ptr<okapi::MotorGroup> drive_lft, std::shared_ptr<okapi::MotorGroup> drive_rt, std::shared_ptr<pros::IMU> imu, std::shared_ptr<pros::Controller> master)
{
	double heading = imu->get_rotation(); // initial heading
	double kp = 2.3;
	double ki = 0;
	double kd = .13;

	double dt = 5;
	double tol = 1.5;
	PID_Controller controller = PID_Controller(kp, ki, kd, dt/1000);
	double err = target-heading;
	controller.reset(err);
	double steady_time = 400;
	double below_tol_time = 0;
    while (abs(err) >= tol || below_tol_time < steady_time)
    {	
		heading = imu->get_rotation();
		err = heading - target;
		double output = controller.update(err);
        output = output/abs(output)*std::min(abs(output), (double) 12000);
		drive_lft->moveVelocity(-output);
		drive_rt->moveVelocity(output);
        master->print(1, 1, "pow: %d, rot: %d, ", (int) err, (int) output);
		if(abs(err) < tol) {
			below_tol_time = below_tol_time + dt;
		}
		else {
			below_tol_time = 0;
		}
		pros::delay(dt);
    }
    drive_lft->moveVelocity(0);
    drive_rt->moveVelocity(0);
}

goal_color sensor_rgb_to_enum(std::shared_ptr<pros::Optical> sensor) {
    pros::c::optical_rgb_s_t c_s = sensor->get_rgb();
    goal_color c;
    if(c_s.red > c_s.blue) {
        c = goal_color::RED;
    }
    else {
        c = goal_color::BLUE;
    }
    return c;
}

void move_roller(std::shared_ptr<okapi::MotorGroup> lft,
                 std::shared_ptr<okapi::MotorGroup> rt,
                 std::shared_ptr<pros::Optical> sensor,
                 std::shared_ptr<okapi::MotorGroup> intake,
                 std::shared_ptr<pros::Controller> master,
                 goal_color target_color)
{  

    int intake_speed = -50;
    rt->moveVoltage(-1500);
    lft->moveVoltage(-1500);
    
    goal_color starting_color = sensor_rgb_to_enum(sensor);
    
    if(starting_color != target_color) {
        intake->moveVelocity(-intake_speed);
    }
    else {
        intake->moveVelocity(intake_speed);
    }
    
    goal_color new_color = sensor_rgb_to_enum(sensor);
    
    while(new_color == starting_color && !master->get_digital(DIGITAL_A)) {
        //master->print(0, 0, "%d %d\n",starting_color, new_color);
        new_color = sensor_rgb_to_enum(sensor);
    }
    intake->moveVelocity(0);
    rt->moveVoltage(2500);
    lft->moveVoltage(2500);
    pros::delay(400);
    rt->moveVoltage(0);
    lft->moveVoltage(0);
}