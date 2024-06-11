#include <Arduino.h>
#include <math.h>
#include <esp_timer.h>
#include "vehicle.h"
#include "configuration.h"

void init_vehicle(Vehicle *vehicle, Motor left_front_motor, Motor right_front_motor, Vehicle_PIDs vehicle_pids) {

    vehicle->current_state.time_stamp = 0;

    vehicle->current_state.velocity.x = 0.0;
    vehicle->current_state.velocity.y = 0.0;
    vehicle->current_state.velocity.angular = 0.0;

    vehicle->last_state.velocity.x = 0.0;
    vehicle->last_state.velocity.y = 0.0;
    vehicle->last_state.velocity.angular = 0.0;
    vehicle->last_state.time_stamp = 0;

    vehicle->desired_state.velocity.x = 0.0;
    vehicle->desired_state.velocity.y = 0.0;
    vehicle->desired_state.velocity.angular = 0.0;
    vehicle->desired_state.time_stamp = 0;

    vehicle->signal_state.velocity.x = 0.0;
    vehicle->signal_state.velocity.y = 0.0;
    vehicle->signal_state.velocity.angular = 0.0;
    vehicle->signal_state.time_stamp = 0;

    vehicle->left_front_motor = left_front_motor;   
    vehicle->right_front_motor = right_front_motor;
    
    //MOD vehicle->vehicle_width = VEHICLE_WIDTH;
    //MOD vehicle->vehicle_length = VEHICLE_LENGTH;

    vehicle->wheel_diameter = WHEEL_DIAMETER;
    vehicle->wheel_distance = WHEEL_DISTANCE;
}


void init_vehicle_pids(Vehicle_PIDs *vehicle_pids , VEL_PID velocity_pid_x ,  VEL_PID velocity_pid_y, VEL_PID velocity_pid_angular , POS_PID pos_pid_x , POS_PID pos_pid_y , POS_PID pos_pid_angular) {
    vehicle_pids->velocity_pid_x = velocity_pid_x;
    vehicle_pids->velocity_pid_y = velocity_pid_y;
    vehicle_pids->velocity_pid_angular = velocity_pid_angular;

    vehicle_pids->pos_pid_x = pos_pid_x;
    vehicle_pids->pos_pid_y = pos_pid_y;
    vehicle_pids->pos_pid_angular = pos_pid_angular;
}

void translate_twist_to_motor_commands(Vehicle *vehicle) {
    float r = WHEEL_DIAMETER / 2.0; // Radius of the wheels
    float V = vehicle->desired_state.velocity.x; // Desired linear velocity
    float omega = vehicle->desired_state.velocity.angular; // Desired angular velocity
    float L = WHEEL_DISTANCE; // Distance between the wheels (wheelbase)

    // Calculate wheel velocities
    vehicle->left_front_motor.desired_velocity = (V - (omega * L / 2.0)) / r; // Adjust for wheelbase
    vehicle->right_front_motor.desired_velocity = (V + (omega * L / 2.0)) / r; // Adjust for wheelbase
}



void vehicle_step(Vehicle *vehicle) {
    vehicle->signal_state.velocity.x = vel_pid_step(&vehicle->vehicle_pids.velocity_pid_x, vehicle->desired_state.velocity.x, vehicle->current_state.velocity.x);
    vehicle->signal_state.velocity.angular = vel_pid_step(&vehicle->vehicle_pids.velocity_pid_angular, vehicle->desired_state.velocity.angular, vehicle->current_state.velocity.angular);
    translate_twist_to_motor_commands(vehicle);
}

