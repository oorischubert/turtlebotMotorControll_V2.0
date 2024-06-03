#ifndef CONFIGURATION_H
#define CONFIGURATION_H


// Math configuration
#define PI 3.14159265358979323846

// Encoder configuration for 4 motors
#define LF_ENCODER_PIN_A 5
#define LF_ENCODER_PIN_B 4

//#define RR_ENCODER_PIN_A 5
//#define RR_ENCODER_PIN_B 34

//#define LR_ENCODER_PIN_A 15
//#define LR_ENCODER_PIN_B 14

#define RF_ENCODER_PIN_A 19 //19
#define RF_ENCODER_PIN_B 18 //18

#define ENCODER_ERROR 0.05

// Motor configuration
#define TICKS_PER_TURN 660 // was 660
#define WHEEL_DIAMETER 0.068   // 80 mm

//Motor direction configuration
//#define RR_DIRECTION -1
#define RF_DIRECTION 1 // was -1
//#define LR_DIRECTION 1
#define LF_DIRECTION -1 //changed to -1

// L298N configuration for 4 motors
#define RF_L298N_ENA 32 //32
#define RF_L298N_IN1 15 //33
#define RF_L298N_IN2 33 //15

// #define RR_L298N_ENA 32
// #define RR_L298N_IN1 33
// #define RR_L298N_IN2 13

// #define LR_L298N_ENA 12
// #define LR_L298N_IN1 2
// #define LR_L298N_IN2 4

#define LF_L298N_ENA 13 //13
#define LF_L298N_IN1 12 //12
#define LF_L298N_IN2 27 //27

// Pos PID configuration
#define POS_KP 1.0
#define POS_KI 0.01
#define POS_KD 0.1
#define POS_I_WINDUP 1000.0

// Vel PID configuration
#define VEL_KP 150.0
#define VEL_KI 2.0
#define VEL_KD 1.0
#define VEL_I_WINDUP 1000.0

//vehicle dimensions
#define VEHICLE_WIDTH 0.125 // meter
#define VEHICLE_LENGTH 0.17 //meter
#define WHEEL_DISTANCE 0.15 //meter

//timer configurations
#define ODOMETRY_DT 0.005 // 200 Hz
#define MOTOR_CONTROL_DT 0.002        // 500 Hz
#define COMMUNICATION_DT 0.01  // 100 Hz

// communication configuration
#define SERIAL_BAUDRATE 115200
#define SIZE_OF_RX_DATA 17  // 2 headrs +type of command + vector 3 of commands + check sum + tail
#define SIZE_OF_TX_DATA 52  // 2 headers + odometry + variance + check sum + tail
#define HEADER 200
#define TAIL 199


// tasks configuration
#define MOTOR_CONTROL_CORE  1      // Using Core 1 for Motor Control
#define ODOMETRY_CORE  0        // Using Core 0 for Odometry computation
#define COMMUNICATION_CORE  0        // Using Core 0 for communication handling


// imu configs
#define DEG_TO_RAD (PI / 180.0)

// control super important configuration
#define D 0

#define TIMEOUT_MICROSECONDS 300000 // 0.1 second, adjust as needed

#endif // CONFIGURATION_H
