#include <Arduino.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <ESP32Encoder.h>
#include <esp_task_wdt.h>
#include "encoder.h"
#include "motor.h"
#include "vehicle.h"
#include "configuration.h"
#include "pos_pid.h"
#include "velocity_pid.h"
#include "l298n.h"
#include "comm_controller.h"


POS_PID pos_pid_left_front_motor;
POS_PID pos_pid_right_front_motor;
//MOD POS_PID pos_pid_left_rear_motor;
//MOD POS_PID pos_pid_right_rear_motor;

VEL_PID vel_pid_left_front_motor;
VEL_PID vel_pid_right_front_motor;
//MOD VEL_PID vel_pid_left_rear_motor;
//MOD VEL_PID vel_pid_right_rear_motor;

POS_PID pos_pid_x; 
POS_PID pos_pid_y;
POS_PID pos_pid_angular;

VEL_PID vel_pid_x;
VEL_PID vel_pid_y;
VEL_PID vel_pid_angular;

Vehicle_PIDs vehicle_pids;

// Encoder objects
Encoder encoderLF;
//MOD Encoder encoderRR;
//MOD Encoder encoderLR;
Encoder encoderRF;

L298N driverLF;
//MOD L298N driverRR;
//MOD L298N driverLR;
L298N driverRF;

Motor left_front_motor;
Motor right_front_motor;
//MOD Motor left_rear_motor;
//MOD Motor right_rear_motor;

Vehicle vehicle;

CommController comm;


TaskHandle_t MotorControlTaskHandle;
TaskHandle_t vehicleControlTaskHandle;
TaskHandle_t CommunicationTaskHandle;

SemaphoreHandle_t vehicleDesiredStateMutex;
SemaphoreHandle_t MotorVelocityMutex;
SemaphoreHandle_t vehicleCurrentStateMutex;
SemaphoreHandle_t MotorUpdateMutex;

//modified for differential-drive driver. MOD ticker signifies these changes.
//175.3mm wheel distance

void setup() {
    BaseType_t taskCreated;

    Serial.begin(115200);
    ESP32Encoder::useInternalWeakPullResistors = UP;

    // init vehicle PIDs
    initPosPID(&pos_pid_x, POS_KP, POS_KI, POS_KD, POS_I_WINDUP);
    initPosPID(&pos_pid_y, POS_KP, POS_KI, POS_KD, POS_I_WINDUP);
    initPosPID(&pos_pid_angular, POS_KP, POS_KI, POS_KD, POS_I_WINDUP);

    initVelPID(&vel_pid_x, VEL_KP, VEL_KI, VEL_KD, VEL_I_WINDUP);
    initVelPID(&vel_pid_y, VEL_KP, VEL_KI, VEL_KD, VEL_I_WINDUP);
    initVelPID(&vel_pid_angular, VEL_KP, VEL_KI, VEL_KD, VEL_I_WINDUP);

//     // init PIDs for each motor
    initPosPID(&pos_pid_left_front_motor, POS_KP, POS_KI, POS_KD, POS_I_WINDUP);
    initVelPID(&vel_pid_left_front_motor, VEL_KP, VEL_KI, VEL_KD, VEL_I_WINDUP);

    initPosPID(&pos_pid_right_front_motor, POS_KP, POS_KI, POS_KD, POS_I_WINDUP);
    initVelPID(&vel_pid_right_front_motor, VEL_KP, VEL_KI, VEL_KD, VEL_I_WINDUP);

   //MOD initPosPID(&pos_pid_left_rear_motor, POS_KP, POS_KI, POS_KD, POS_I_WINDUP);
   //MOD initVelPID(&vel_pid_left_rear_motor, VEL_KP, VEL_KI, VEL_KD, VEL_I_WINDUP);

   //MOD initPosPID(&pos_pid_right_rear_motor, POS_KP, POS_KI, POS_KD, POS_I_WINDUP);
   //MOD initVelPID(&vel_pid_right_rear_motor, VEL_KP, VEL_KI, VEL_KD, VEL_I_WINDUP); 


    init_vehicle_pids(&vehicle_pids, vel_pid_x, vel_pid_y, vel_pid_angular, pos_pid_x, pos_pid_y, pos_pid_angular);

     // init encoders
    initEncoder(&encoderLF, LF_ENCODER_PIN_A, LF_ENCODER_PIN_B);  
    //MOD initEncoder(&encoderRR, RR_ENCODER_PIN_A, RR_ENCODER_PIN_B);
    //MOD initEncoder(&encoderLR, LR_ENCODER_PIN_A, LR_ENCODER_PIN_B);
    initEncoder(&encoderRF, RF_ENCODER_PIN_A, RF_ENCODER_PIN_B);


     // init motor driver l298n
    initL298N(&driverLF, LF_L298N_ENA, LF_L298N_IN1, LF_L298N_IN2);
    //MOD initL298N(&driverRR, RR_L298N_ENA, RR_L298N_IN1, RR_L298N_IN2);
    //MOD initL298N(&driverLR, LR_L298N_ENA, LR_L298N_IN1, LR_L298N_IN2);
    initL298N(&driverRF, RF_L298N_ENA, RF_L298N_IN1, RF_L298N_IN2);

     //init motors
    initMotor(&left_front_motor, encoderLF, driverLF, pos_pid_left_front_motor, vel_pid_left_front_motor,LF_DIRECTION); 
    initMotor(&right_front_motor, encoderRF, driverRF, pos_pid_right_front_motor, vel_pid_right_front_motor,RF_DIRECTION);
    //MOD initMotor(&left_rear_motor, encoderLR, driverLR, pos_pid_left_rear_motor, vel_pid_left_rear_motor, LR_DIRECTION);
    //MOD initMotor(&right_rear_motor, encoderRR, driverRR, pos_pid_right_rear_motor, vel_pid_right_rear_motor, RR_DIRECTION);
    

//     //init vehicle
    init_vehicle(&vehicle, left_front_motor, right_front_motor, vehicle_pids);


    //init communication
    comm_controller_init(&comm);

    // Create Mutex for Vehicle Data
    vehicleDesiredStateMutex =xSemaphoreCreateMutex();
    MotorVelocityMutex = xSemaphoreCreateMutex();
    vehicleCurrentStateMutex = xSemaphoreCreateMutex();
    MotorUpdateMutex = xSemaphoreCreateMutex();

    // Creating motorControlTask
    Serial.println("create task");
    taskCreated = xTaskCreatePinnedToCore(
        motorControlTask,       // Task function
        "MotorControlTask",     // Name of task
        60000,                  // Stack size of task
        &vehicle,               // Parameter of the task
        3,                      // Priority of the task
        &MotorControlTaskHandle,// Task handle to keep track of created task
        MOTOR_CONTROL_CORE);    // Core where the task should run

    if (taskCreated != pdPASS) {
        Serial.println("MotorControlTask creation failed!");
    }
    else {
      Serial.println("MotorControlTask creation success!");
    }

    // Creating vehicleControlTask
    taskCreated = xTaskCreatePinnedToCore(
      vehicleControlTask,     // Task function
      "VehicleControlTask",   // Name of task
      60000,                  // Stack size of task
      &vehicle,               // Parameter of the task
      2,                      // Priority of the task
      &vehicleControlTaskHandle, // Task handle to keep track of created task
      ODOMETRY_CORE);         // Core where the task should run

    if (taskCreated != pdPASS) {
        Serial.println("VehicleControlTask creation failed!");
    }
    else {
      Serial.println("VehicleControlTask creation success!");
    }

    // Creating communicationTask
    taskCreated = xTaskCreatePinnedToCore(
        communicationTask,          // Task function
        "CommunicationTask",        // Name of task
        30000,                      // Stack size of task
        &vehicle,                   // Parameter of the task
        1,                          // Priority of the task
        &CommunicationTaskHandle,   // Task handle to keep track of created task
        COMMUNICATION_CORE);        // Core where the task should run

    if (taskCreated != pdPASS) {
        Serial.println("CommunicationTask creation failed!");
    }
    else {
      Serial.println("CommunicationTask creation success!");
    }


    //vehicle.left_front_motor.desired_velocity = 0.1; //testing
    //vehicle.right_front_motor.desired_velocity = 0.1; //testing
    // vehicle.left_rear_motor.desired_velocity = 5.0;
    // vehicle.right_rear_motor.desired_velocity = 5.0;
    // vehicle.desired_state.velocity.y = 0.1;    // 1.0 m/s is the max speed of the vehicle 
    // vehicle.desired_state.velocity.x = 0.1;    // 1.0 m/s is the max speed of the vehicle
    
}

void loop() {
}

void motorControlTask(void * parameter) {
    Vehicle *vehicle = (Vehicle *)parameter;
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(MOTOR_CONTROL_DT * 1000);

    // Variables for counting
    int count = 0;
    unsigned long lastPrintTime = 0;

    xLastWakeTime = xTaskGetTickCount();
    
    for(;;) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        if (millis() - lastPrintTime >= 1000) {
            // Serial.print("Motor Control Step Count: ");
            // Serial.println(count);
            count = 0;
            lastPrintTime = millis();
        }

        // Update and compute velocity for the left front motor
        vehicle->left_front_motor.current_position = readEncoder(&vehicle->left_front_motor.encoder) * vehicle->left_front_motor.distancePerTick;
        if(xSemaphoreTake(MotorVelocityMutex, portMAX_DELAY)) {
            computeVelocity(&vehicle->left_front_motor);
            xSemaphoreGive(MotorVelocityMutex);
        }
        if(xSemaphoreTake(MotorUpdateMutex, portMAX_DELAY)) {
            motor_step(&vehicle->left_front_motor);
            xSemaphoreGive(MotorUpdateMutex);
        }
      
        // Update and compute velocity for the right front motor
        vehicle->right_front_motor.current_position = readEncoder(&vehicle->right_front_motor.encoder) * vehicle->right_front_motor.distancePerTick;
        if(xSemaphoreTake(MotorVelocityMutex, portMAX_DELAY)) {
            computeVelocity(&vehicle->right_front_motor);
            xSemaphoreGive(MotorVelocityMutex);
        }
        if(xSemaphoreTake(MotorUpdateMutex, portMAX_DELAY)) {
            motor_step(&vehicle->right_front_motor);
            xSemaphoreGive(MotorUpdateMutex);
        }

        // Update and compute velocity for the left rear motor
        //MOD
        // vehicle->left_rear_motor.current_position = readEncoder(&vehicle->left_rear_motor.encoder) * vehicle->left_rear_motor.distancePerTick;
        // if(xSemaphoreTake(MotorVelocityMutex, portMAX_DELAY)) {
        //     computeVelocity(&vehicle->left_rear_motor);
        //     xSemaphoreGive(MotorVelocityMutex);
        // }
        // if(xSemaphoreTake(MotorUpdateMutex, portMAX_DELAY)) {
        //     motor_step(&vehicle->left_rear_motor);
        //     xSemaphoreGive(MotorUpdateMutex);
        // }
      
        // Update and compute velocity for the right rear motor
        //MOD
        // vehicle->right_rear_motor.current_position = readEncoder(&vehicle->right_rear_motor.encoder) * vehicle->right_rear_motor.distancePerTick;
        // if(xSemaphoreTake(MotorVelocityMutex, portMAX_DELAY)) {
        //     computeVelocity(&vehicle->right_rear_motor);
        //     xSemaphoreGive(MotorVelocityMutex);
        // }
        // if(xSemaphoreTake(MotorUpdateMutex, portMAX_DELAY)) {
        //     motor_step(&vehicle->right_rear_motor);
        //     xSemaphoreGive(MotorUpdateMutex);
        // }
      
        // Print the current velocities
        // Serial.print(vehicle->left_front_motor.current_velocity);
        // Serial.print(",");
        // Serial.print(vehicle->right_front_motor.current_velocity);
        // Serial.print(",");
        // Serial.print(vehicle->left_rear_motor.current_velocity);
        // Serial.print(",");
        // Serial.println(vehicle->right_rear_motor.current_velocity);
        
        count++;
    }
}




void vehicleControlTask(void * parameter) {
    Vehicle *vehicle = (Vehicle *)parameter;
    int count_vehicle = 0;
    unsigned long lastPrintTime = millis(); // Initialize lastPrintTime to current millis

    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(ODOMETRY_DT * 1000);

    xLastWakeTime = xTaskGetTickCount();

    for(;;) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        

        // Check if one second has passed
        if (millis() - lastPrintTime >= 1000) {
            // Serial.print("Vehicle Control Step Count: ");
            // Serial.println(count_vehicle);
            count_vehicle = 0;
            lastPrintTime = millis();
        }
        // if(xSemaphoreTake(MotorVelocityMutex, portMAX_DELAY)) {
        //   compute_odometry_from_encoders(vehicle);
        //   count_vehicle++;    
        //   xSemaphoreGive(MotorVelocityMutex);
        // }
        if(xSemaphoreTake(vehicleCurrentStateMutex, portMAX_DELAY)) {
          ProcessDataToSend(&comm, vehicle); 
          // Serial.write(comm.TxData, SIZE_OF_TX_DATA);  // Transmit data     
          xSemaphoreGive(vehicleCurrentStateMutex);
        } 
        

    } 
}


void communicationTask(void * parameter) {
    Vehicle *vehicle = (Vehicle *)parameter;
    int count_comm = 0;
    unsigned long lastPrintTime = millis();
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(COMMUNICATION_DT * 1000);

    xLastWakeTime = xTaskGetTickCount();

    for(;;) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        // Check if one second has passed
        if (millis() - lastPrintTime >= 1000) {
            // Reset the counter and update lastPrintTime
            count_comm = 0;
            lastPrintTime = millis();
        }
        
        if (xSemaphoreTake(vehicleCurrentStateMutex, portMAX_DELAY)) { 
            Serial.write(comm.TxData, SIZE_OF_TX_DATA);  // Transmit data
            xSemaphoreGive(vehicleCurrentStateMutex);
        }

        if (Serial.available() >=  0) {
            Serial.readBytes(comm.RxData, SIZE_OF_RX_DATA);  // Receive data
            int check = receiveData(&comm, vehicle);  
            if (check == 2) {
              Serial.flush(); 
              if (xSemaphoreTake(MotorUpdateMutex, portMAX_DELAY)) {  
                  //translate_twist_to_motor_commands(vehicle);
                  xSemaphoreGive(MotorUpdateMutex);
              }
            }
        }
        // if (Serial.available() > SIZE_OF_RX_DATA) {
        //   Serial.flush(); //clear buffer
        // } 
        count_comm++;
    }
}




