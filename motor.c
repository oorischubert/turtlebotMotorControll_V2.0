#include "motor.h"
#include "configuration.h"
#include <esp_timer.h>

#define ALPHA 0.5 // Example value, adjust based on desired responsiveness vs. smoothness

void initMotor(Motor *motor, Encoder enc ,L298N driver, VEL_PID vel_pid ,int direction) {                                  
    motor->desired_position = 0;
    motor->current_position = 0;
    motor->desired_velocity = 0;
    motor->current_velocity = 0;
    motor->last_position = 0;
    motor->controlMode = VELOCITY;
    motor->direction = direction;
    motor->encoder = enc;
    motor->l298n = driver;
    motor->pos_pid = pos_pid;
    motor->vel_pid = vel_pid;
    motor->ticksPerTurn = TICKS_PER_TURN;
    motor->wheelDiameter = WHEEL_DIAMETER;
    motor->distancePerTick = (2.0 * PI) / motor->ticksPerTurn;  // in radians
    motor->lastUpdateTime = esp_timer_get_time();

}

void computeVelocity(Motor *motor) {
    int64_t currentTime = esp_timer_get_time(); // Get the current time in microseconds
    int64_t deltaTime = currentTime - motor->lastUpdateTime; // Time difference in microseconds

    if (deltaTime > TIMEOUT_MICROSECONDS) {
        // Timeout reached, assume motor has stopped
        motor->current_velocity = 0.0;
    } 
    if (motor->current_position != motor->last_position) {
        float deltaPosition = motor->current_position - motor->last_position;
        float dt = deltaTime / 1000000.0; // Convert time difference to seconds
        motor->current_velocity = (deltaPosition / dt) * motor->direction;
        motor->last_position = motor->current_position;
        motor->lastUpdateTime = currentTime; // Update the last update time regardless
    } 

}

void updateMotor(Motor *motor) {  //updates position and velocity
    // motor->current_position = getEncoderCount(&motor->encoder) * motor->distancePerTick;  //update position
    computeVelocity(motor); // update velocity
}


void motor_step(Motor *motor) {
    // updateMotor(motor);
    float control_signal = 0;
    switch(motor->controlMode){
        case VELOCITY:
            control_signal = vel_pid_step(&motor->vel_pid, motor->desired_velocity , motor->current_velocity);
            break;
        }
    if (control_signal > 0) move_forward(&motor->l298n , control_signal);
    if (control_signal < 0) move_backward(&motor->l298n , -control_signal);
    if (control_signal == 0) stop(&motor->l298n);
}


