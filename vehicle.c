#include <Arduino.h>
#include <math.h>
#include <esp_timer.h>
#include "vehicle.h"
#include "configuration.h"

void init_vehicle(Vehicle *vehicle, Motor left_front_motor, Motor right_front_motor, Vehicle_PIDs vehicle_pids) {
    vehicle->current_state.odometry_variance.static_error = ENCODER_ERROR * ENCODER_ERROR;
    vehicle->current_state.odometry_variance.position_error.x = 0.0;
    vehicle->current_state.odometry_variance.position_error.y = 0.0;
    vehicle->current_state.odometry_variance.position_error.angular = 0.0;
    vehicle->current_state.odometry_variance.velocity_error.x = 0.0;
    vehicle->current_state.odometry_variance.velocity_error.y = 0.0;
    vehicle->current_state.odometry_variance.velocity_error.angular = 0.0;

    vehicle->current_state.time_stamp = 0;

    vehicle->current_state.position.x = 0.0;
    vehicle->current_state.position.y = 0.0;
    vehicle->current_state.position.angular = 0.0;
    vehicle->current_state.velocity.x = 0.0;
    vehicle->current_state.velocity.y = 0.0;
    vehicle->current_state.velocity.angular = 0.0;

    vehicle->last_state.position.x = 0.0;
    vehicle->last_state.position.y = 0.0;
    vehicle->last_state.position.angular = 0.0;
    vehicle->last_state.velocity.x = 0.0;
    vehicle->last_state.velocity.y = 0.0;
    vehicle->last_state.velocity.angular = 0.0;
    vehicle->last_state.time_stamp = 0;

    vehicle->desired_state.position.x = 0.0;
    vehicle->desired_state.position.y = 0.0;
    vehicle->desired_state.position.angular = 0.0;
    vehicle->desired_state.velocity.x = 0.0;
    vehicle->desired_state.velocity.y = 0.0;
    vehicle->desired_state.velocity.angular = 0.0;
    vehicle->desired_state.time_stamp = 0;

    vehicle->signal_state.position.x = 0.0;
    vehicle->signal_state.position.y = 0.0;
    vehicle->signal_state.position.angular = 0.0;
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

void compute_odometry_from_encoders(Vehicle *vehicle) {
    float wheel_radius = vehicle->wheel_diameter / 2.0;
    float left_velocity = vehicle->left_front_motor.current_velocity * wheel_radius; // left_motor -> left_front_motor
    float right_velocity = vehicle->right_front_motor.current_velocity * wheel_radius; // right_motor -> right_front_motor

    // Calculate linear and angular velocity
    vehicle->current_state.velocity.x = (left_velocity + right_velocity) / 2.0;
    vehicle->current_state.velocity.angular = (right_velocity - left_velocity) / vehicle->wheel_distance;

    // Update position based on velocities
    vehicle->current_state.time_stamp = esp_timer_get_time(); // Get the current time in microseconds
    float dt = (vehicle->current_state.time_stamp - vehicle->last_state.time_stamp) / 1000000.0; 
    vehicle->current_state.position.x += vehicle->current_state.velocity.x * dt;
    vehicle->current_state.position.angular += vehicle->current_state.velocity.angular * dt;

    vehicle->last_state = vehicle->current_state;
}

//deprecated -> done iin ros
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
    // Compute current odometry based on encoder readings
    //compute_odometry_from_encoders(vehicle);
    vehicle->signal_state.velocity.x = vel_pid_step(&vehicle->vehicle_pids.velocity_pid_x, vehicle->desired_state.velocity.x, vehicle->current_state.velocity.x);
    vehicle->signal_state.velocity.angular = vel_pid_step(&vehicle->vehicle_pids.velocity_pid_angular, vehicle->desired_state.velocity.angular, vehicle->current_state.velocity.angular);
    //translate_twist_to_motor_commands(vehicle);
}

