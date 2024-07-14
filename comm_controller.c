#include <stdint.h>       // Standard library headers first
#include <Arduino.h>
#include "configuration.h"
#include "vehicle.h"
#include "comm_controller.h"



void comm_controller_init(CommController *comm) {
    comm->comm_baud_rate = SERIAL_BAUDRATE;
    memset(comm->RxData, 0, SIZE_OF_RX_DATA);
    memset(comm->TxData, 0, SIZE_OF_TX_DATA);
}


int receiveData(CommController *comm, Vehicle *vehicle) {
  int valid_data = 0;
  if (comm->RxData[0] == HEADER && comm->RxData[1] == HEADER) {
    valid_data = 1;
    uint8_t checksum = 0;
    for (int i = 3; i < SIZE_OF_RX_DATA - 2 ; i++) {
        checksum += comm->RxData[i];
    }
    if (comm->RxData[SIZE_OF_RX_DATA - 2] == checksum) {  //passed all integrity checks
      valid_data = 2;
      if (comm->RxData[2] == RESET_ENCODERS) {
           clearCount(&vehicle->left_front_motor.encoder.instance);
           clearCount(&vehicle->right_front_motor.encoder.instance);
       }
      else if (comm->RxData[2] == VELOCITY_MODE) {
          memcpy(&vehicle->left_front_motor.desired_velocity, &comm->RxData[3], 4);
          memcpy(&vehicle->right_front_motor.desired_velocity, &comm->RxData[7], 4);
      }
      else if (comm->RxData[2] == PID_MODE) {
            // Directly pass values from comm->RxData to initialization functions
            initVelPID(&vehicle->left_front_motor.vel_pid, 
                    *(float*)&comm->RxData[3],  // kp
                    *(float*)&comm->RxData[7],  // ki
                    *(float*)&comm->RxData[11], // kd
                    *(float*)&comm->RxData[15]); // i_windup

            initVelPID(&vehicle->right_front_motor.vel_pid, 
                    *(float*)&comm->RxData[3],  // kp
                    *(float*)&comm->RxData[7],  // ki
                    *(float*)&comm->RxData[11], // kd
                    *(float*)&comm->RxData[15]); // i_windup

            initPosPID(&vehicle->left_front_motor.pos_pid, 
                    *(float*)&comm->RxData[19], // kp
                    *(float*)&comm->RxData[23], // ki
                    *(float*)&comm->RxData[27], // kd
                    *(float*)&comm->RxData[31]); // i_windup

            initPosPID(&vehicle->right_front_motor.pos_pid, 
                    *(float*)&comm->RxData[19], // kp
                    *(float*)&comm->RxData[23], // ki
                    *(float*)&comm->RxData[27], // kd
                    *(float*)&comm->RxData[31]); // i_windup
}
    }
    else {
        memset(comm->RxData, 0, SIZE_OF_RX_DATA);  // Clear the buffer
    }
  }
  else {
      memset(comm->RxData, 0, SIZE_OF_RX_DATA);  // Clear the buffer
  }
  return valid_data;
}


void ProcessDataToSend(CommController *comm, const Vehicle *vehicle) {
    comm->TxData[0] = HEADER;
    comm->TxData[1] = HEADER;

    memcpy(&comm->TxData[2], &vehicle->left_front_motor.current_velocity, 4);
    memcpy(&comm->TxData[6], &vehicle->right_front_motor.current_velocity, 4);
    memcpy(&comm->TxData[10], &vehicle->left_front_motor.encoder_count, 8);
    memcpy(&comm->TxData[18], &vehicle->right_front_motor.encoder_count, 8); 

    // Compute checksum
    uint8_t checksum = 0;
    for (int i = 2; i < 50; i++) { 
        checksum += comm->TxData[i];
    }
    comm->TxData[50] = checksum;
    comm->TxData[51] = TAIL;

    // Send data
}
