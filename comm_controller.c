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
        for (int i = 3; i < SIZE_OF_RX_DATA - 1; i++) {  // Corrected the start index
            checksum += comm->RxData[i];
        }
        if (comm->RxData[SIZE_OF_RX_DATA - 2] == checksum) {  // Passed all integrity checks
            valid_data = 2;
            if (comm->RxData[2] == VELOCITY_MODE) {
                memcpy(&vehicle->temp_left_front_motor_vel, &comm->RxData[3], 4);
                memcpy(&vehicle->temp_left_front_motor_vel, &comm->RxData[7], 4);

            } else if (comm->RxData[2] == ENCODER_RESET) {
                resetEncoder(&vehicle->left_front_motor.encoder);
                resetEncoder(&vehicle->right_front_motor.encoder);
            }
            
        } else {
            memset(comm->RxData, 0, SIZE_OF_RX_DATA);  // Clear the buffer
        }
    } else {
        memset(comm->RxData, 0, SIZE_OF_RX_DATA);  // Clear the buffer
    }
    return valid_data;
}



void ProcessDataToSend(CommController *comm, const Vehicle *vehicle) {
    comm->TxData[0] = HEADER;
    comm->TxData[1] = HEADER;

    memcpy(&comm->TxData[2], readEncoder(&vehicle->left_front_motor.encoder), 4); //was readEncoder(&vehicle->left_front_motor.encoder) using placeholder
    memcpy(&comm->TxData[6], readEncoder(&vehicle->right_front_motor.encoder), 4); //was readEncoder(&vehicle->right_front_motor.encoder) using placeholder
    memcpy(&comm->TxData[10], &vehicle->left_front_motor.current_velocity, 4);
    memcpy(&comm->TxData[14], &vehicle->right_front_motor.current_velocity, 4);

    // Compute checksum
    uint8_t checksum = 0;
    for (int i = 2; i < SIZE_OF_TX_DATA-2; i++) {  
        checksum += comm->TxData[i];
    }
    comm->TxData[18] = checksum;
    comm->TxData[19] = TAIL;

    // Send data
   
}
