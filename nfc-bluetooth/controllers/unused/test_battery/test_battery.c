// e-pucks have a 5Wh battery - 18000 Joules
// USB chargers deliver from 2.5W to 5W per second (= 2.5 to 5 Joules/sec)
// NFC chargers delivel 1W per second (= 1 Joule/sec)

// Red - X - Nord
// Green - Y - Up
// Blue - Z - East
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <webots/distance_sensor.h>
#include <webots/emitter.h>
#include <webots/motor.h>
#include <webots/receiver.h>
#include <webots/robot.h>
#include <webots/led.h>

#define SPEED 6
#define TIME_STEP 64
#define COMMUNICATION_CHANNEL 1

//------------------------------------------------------------------------------
//
//    BEHAVIORAL MODULES
//
//------------------------------------------------------------------------------



////////////////////////////////////////////
// NRM - NFC Range Module
//
// The NRM routine



//------------------------------------------------------------------------------
//
//    CONTROLLER
//
//------------------------------------------------------------------------------

int main() {
  WbDeviceTag emitter, receiver, left_motor, right_motor;
  //int message_printed = 0; /* used to avoid printing continuously the communication state */

  wb_robot_init();

  /* get a handler to the motors and set target position to infinity (speed control). */
  left_motor = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);
  
  emitter = wb_robot_get_device("nfc_e");
  wb_emitter_set_channel(emitter, COMMUNICATION_CHANNEL);
  wb_emitter_set_range(emitter, 0.001);
  
  receiver = wb_robot_get_device("nfc_r");
  wb_receiver_enable(receiver, TIME_STEP);
  
  
  
  while (wb_robot_step(TIME_STEP) != -1) {
    
    ////////////////////////////////////////////
    // Test basic communication between the robots
    //
    // Broadcast your ID as message to all the other robots
    
    // const char *message = wb_robot_get_name();
    // wb_emitter_send(emitter, message, strlen(message) + 1);
    
    // if (wb_receiver_get_queue_length(receiver) > 0) {
       // while (wb_receiver_get_queue_length(receiver) > 0) {
        // /* read current packet's data */
        // const char *buffer = wb_receiver_get_data(receiver);

        // if (message_printed != 1) {
          // /* print null-terminated message */
          // const char *message = wb_robot_get_name();
          // printf("Communicating: I'm robot \"%s\" and I received a message from robot \"%s\"\n", message, buffer);
          
        // }
        // /* fetch next packet */
        // wb_receiver_next_packet(receiver);
      // } 
      
      // message_printed = 1;
    // }
    
    ////////////////////////////////////////////
    // Test leader decision and set up process
    //
    // Here we assume that we meet all at the same time
    //
    // We print a few information to check the correctness of the algorithm:
    // 1. My ID and whether I'm leader or  follower
    // 2. My comms queue if I have inglobated a different robot
    
    // First decide if I'm the leader
    
    
    if (wb_receiver_get_queue_length(receiver) > 0) {
      
      printf("aaaa\n");
        
    }    
    
    ////////////////////////////////////////////
    // Final step of the simulation
    //
    // See that the code is actually running
    
    
      wb_motor_set_velocity(left_motor, 1);
      wb_motor_set_velocity(right_motor, 1);
    
  }

  wb_robot_cleanup();

  return 0;
}
