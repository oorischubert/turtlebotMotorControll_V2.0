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

// PID objects
POS_PID pos_pid_left_front_motor;
POS_PID pos_pid_right_front_motor;

VEL_PID vel_pid_left_front_motor;
VEL_PID vel_pid_right_front_motor;
/////////////////////////

// Encoder objects
Encoder encoderLF;
Encoder encoderRF;
///////////////////

// Motor driver objects
L298N driverLF;
L298N driverRF;
////////////////////////

// Motor objects
Motor left_front_motor;
Motor right_front_motor;
///////////////////////

// Vehicle object
Vehicle vehicle;
/////////////////

// Communication object
CommController comm;
//////////////////////

// FreeRtos objects
TaskHandle_t MotorControlTaskHandle;
TaskHandle_t vehicleControlTaskHandle;
TaskHandle_t CommunicationTaskHandle;

SemaphoreHandle_t vehicleDesiredStateMutex;
SemaphoreHandle_t MotorVelocityMutex;
SemaphoreHandle_t vehicleCurrentStateMutex;
SemaphoreHandle_t MotorUpdateMutex;
SemaphoreHandle_t MotorControlSemaphore;
/////////////////////////////

esp_timer_handle_t motor_timer;

void IRAM_ATTR onMotorTimer(void* arg) {
    xSemaphoreGiveFromISR(MotorControlSemaphore, NULL);
}


void setup() {

  BaseType_t taskCreated;

  // init PIDs for each motor
  initPosPID(&pos_pid_left_front_motor, POS_KP, POS_KI, POS_KD, POS_I_WINDUP);
  initVelPID(&vel_pid_left_front_motor, VEL_KP, VEL_KI, VEL_KD, VEL_I_WINDUP);

  initPosPID(&pos_pid_right_front_motor, POS_KP, POS_KI, POS_KD, POS_I_WINDUP);
  initVelPID(&vel_pid_right_front_motor, VEL_KP, VEL_KI, VEL_KD, VEL_I_WINDUP);

  
  // init encoders
    ESP32Encoder::useInternalWeakPullResistors = UP;
  initEncoder(&encoderLF, LF_ENCODER_PIN_A, LF_ENCODER_PIN_B);  
  initEncoder(&encoderRF, RF_ENCODER_PIN_A, RF_ENCODER_PIN_B);


  // init motor driver l298n
  initL298N(&driverLF, LF_L298N_ENA, LF_L298N_IN1, LF_L298N_IN2);
  initL298N(&driverRF, RF_L298N_ENA, RF_L298N_IN1, RF_L298N_IN2);


  //init motors
  initMotor(&left_front_motor, encoderLF, driverLF, pos_pid_left_front_motor, vel_pid_left_front_motor,LF_DIRECTION); 
  initMotor(&right_front_motor, encoderRF, driverRF, pos_pid_right_front_motor, vel_pid_right_front_motor,RF_DIRECTION);
  

  //init vehicle
  init_vehicle(&vehicle, left_front_motor, right_front_motor);


  //init communication
  comm_controller_init(&comm);
  Serial.begin(SERIAL_BAUDRATE);


  // Create Mutex for Vehicle Data
  vehicleDesiredStateMutex =xSemaphoreCreateMutex();
  MotorVelocityMutex = xSemaphoreCreateMutex();
  vehicleCurrentStateMutex = xSemaphoreCreateMutex();
  MotorUpdateMutex = xSemaphoreCreateMutex();
  MotorControlSemaphore = xSemaphoreCreateBinary();

  // Creating motorControlTask
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

   // Create motor timer for MotorControlTask (500 Hz)
    const esp_timer_create_args_t motor_timer_args = {
        .callback = &onMotorTimer,
        .arg = NULL,
        .name = "motor_timer"
    };
    esp_timer_create(&motor_timer_args, &motor_timer);
    esp_timer_start_periodic(motor_timer, 500); // 500 Hz

    // vehicle.desired_state.velocity.angular = 0.0;
    // translate_twist_to_motor_commands(&vehicle);   
}

void loop() {
}

/////////////////////////// MOTOR CONTROL TASK ////////////////////////////////
void motorControlTask(void * parameter) {
    Vehicle *vehicle = (Vehicle *)parameter;

    for(;;) {
        if (xSemaphoreTake(MotorControlSemaphore, portMAX_DELAY)) {
            vehicle->left_front_motor.current_position = readEncoder(&vehicle->left_front_motor.encoder) * vehicle->left_front_motor.distancePerTick * vehicle->right_front_motor.direction;
    
            if(xSemaphoreTake(MotorVelocityMutex, portMAX_DELAY)) {
                computeVelocity(&vehicle->left_front_motor);
                xSemaphoreGive(MotorVelocityMutex);
            }
            if(xSemaphoreTake(MotorUpdateMutex, portMAX_DELAY)) {
                motor_step(&vehicle->left_front_motor);
                xSemaphoreGive(MotorUpdateMutex);
            }
    
            vehicle->right_front_motor.current_position = readEncoder(&vehicle->right_front_motor.encoder) * vehicle->right_front_motor.distancePerTick * vehicle->right_front_motor.direction;
    
            if(xSemaphoreTake(MotorVelocityMutex, portMAX_DELAY)) {
                computeVelocity(&vehicle->right_front_motor);
                xSemaphoreGive(MotorVelocityMutex);
            }
            if(xSemaphoreTake(MotorUpdateMutex, portMAX_DELAY)) {
                motor_step(&vehicle->right_front_motor);
                xSemaphoreGive(MotorUpdateMutex);
            }
        }
    }
}
////////////////////////// END OF MOTOR CONTROL TASK ////////////////////////////////////////

//////////////////////// VEHICLE CONTROL TASK ///////////////////////////////////////////////
void vehicleControlTask(void * parameter) {
  Vehicle *vehicle = (Vehicle *)parameter;
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = pdMS_TO_TICKS(ODOMETRY_DT * 1000);
  xLastWakeTime = xTaskGetTickCount();

  for(;;) {

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    
    if(xSemaphoreTake(MotorVelocityMutex, portMAX_DELAY)) {
      compute_odometry_from_encoders(vehicle);    
      xSemaphoreGive(MotorVelocityMutex);
    }

    if(xSemaphoreTake(vehicleCurrentStateMutex, portMAX_DELAY)) {
      ProcessDataToSend(&comm, vehicle);      
      xSemaphoreGive(vehicleCurrentStateMutex);
    } 
  } 
}
////////////////////////END OF  VEHICLE CONTROL TASK ///////////////////////////////////////////////

//////////////////////// COMMUNICATION TASK ///////////////////////////////////////////////
void communicationTask(void * parameter) {
  Vehicle *vehicle = (Vehicle *)parameter;
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = pdMS_TO_TICKS(COMMUNICATION_DT * 1000);
  xLastWakeTime = xTaskGetTickCount();

  for(;;) {

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    
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
              translate_twist_to_motor_commands(vehicle);
              xSemaphoreGive(MotorUpdateMutex);
          }
        }

    }
  }
}
////////////////////////END OF COMMUNICATION TASK ///////////////////////////////////////////////



