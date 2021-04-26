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
  wb_emitter_set_range(emitter, 0.04);
  
  receiver = wb_robot_get_device("nfc_r");
  wb_receiver_enable(receiver, TIME_STEP);
  
  
  bool leader = false;
  int is_in_queue = 0;
  bool is_stop = false;
  int comms_queue[3];
  int leader_ID = 0;
  int idx = 0;
  int i = 0;
         
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
    
    int my_ID = atoi(wb_robot_get_name() + 5);
    const char *message = wb_robot_get_name();
    wb_emitter_send(emitter, message, strlen(message) + 1); // send my ID continuously because we have no bumper here
    
    if (wb_receiver_get_queue_length(receiver) > 0) {
      
      is_in_queue = 0;
      const char *buffer = wb_receiver_get_data(receiver); // receive the packet
      
      // Check if it is an ID
      int is_ID = strncmp(buffer, "epuck", 5);
      
      // If it is an ID, we check if we have it saved in our comms_queue
      if (is_ID == 0) {
      
        int ext_ID = atoi(buffer + 5);
        // printf("my idx is = %d\n", idx);
        // printf("got an ID, idx = %d"
        for(i = 0; i < idx; i++) {
            
          if(comms_queue[i] == ext_ID) {
            is_in_queue = 1;
          }
          // printf("ID = %s, idx = %d, i = %d, ext_ID = %d, comms_queue[i] = %d\n", wb_robot_get_name() + 5, idx, i, ext_ID, comms_queue[i]);
          
          
        }
           
        // if (my_ID == 2)
          // for(i = 0; i < idx; i++)
              // printf("I'm %s and %d is \n", wb_robot_get_name() + 5, comms_queue[i]);
            
        if (is_in_queue == 0) {
          // for(i = 0; i < idx; i++)
              // printf("I'm %s and my queue contains %d\n", wb_robot_get_name() + 5, comms_queue[i]);
            
          if (ext_ID > my_ID) {
            leader = true;
            printf("Communicating: I'm robot \"%d\" and I am the leader\n", my_ID);
            // printf("saving at index %d \n", idx);
            comms_queue[idx] = ext_ID;
            // printf("%d\n", idx);
            idx += 1;
            // printf("my new idx is = %d\n", idx);
            // printf("new index is %d \n", idx);
          }
          else {
            leader = false;
            if (leader_ID != ext_ID) {
              printf("Communicating: I'm robot \"%d\" and I am NOT the leader\n", my_ID);
              leader_ID = ext_ID;
              char *messagel = "listc";
              wb_emitter_send(emitter, messagel, strlen(messagel) + 1);
              /* I also need to pass the list of robots in my queue */
              for(i = 0; i < idx; i++) {
                int length = snprintf(NULL, 0, "%d", comms_queue[i]);
                char *messagee = malloc(length + 1);
                snprintf(messagee, length + 1, "%d", comms_queue[i]);
                // char *messagee = "aaa";
                // sprintf(messagee, "%d", comms_queue[i]);
                wb_emitter_send(emitter, messagee, strlen(messagee) + 1);
                // printf("sending queue = %d\n", comms_queue[i]);
                // idx--;
              }
              char *messages = "lists";
              wb_emitter_send(emitter, messages, strlen(messages) + 1);
            }
            
          }
        }
        
      }
      
      int is_list_comms = strncmp(buffer, "listc", 5);
      
      // We have already read the header so we can go to the next one
      wb_receiver_next_packet(receiver);
      
      // If it is not an ID, in this case we know that it contains a list of followers to be transmitted to the new leader
      if (is_list_comms == 0) {
      
        while (wb_receiver_get_queue_length(receiver) > 0) {
          
          // Read new message and save the ID in  the queue
          const char *buffer = wb_receiver_get_data(receiver); // receive the packet
          
          is_stop = strncmp(buffer, "lists", 5);
          
          if (is_stop == 0) {
            if (idx > 0)
              printf("ID: %s, my comms queue contains:", wb_robot_get_name() + 5);
            for(i = 0; i < idx; i++) {
              printf(" %d", comms_queue[i]);
            }
            printf("\n");
            break;
          }
          
          comms_queue[idx] = atoi(buffer);
          idx ++;
          
          wb_receiver_next_packet(receiver);
          
        }
        
        // if (message_printed < 1) {
          // if (leader) {
            // wb_led_set(led, 2); // set led green
            // printf("Communicating: I'm robot \"%d\" and I am the leader\n", my_ID);
            // //If I'm the leader, wait to read the queue
            
          // }
          // else {
            // wb_led_set(led, 1); // set led red
            // printf("Communicating: I'm robot \"%d\" and I am NOT the leader\n", my_ID);
            // //If I have a queue, I'll pass it to the leader
            
          // }
          // message_printed = 1;
        // }
        
      }
        
    }    
    
    ////////////////////////////////////////////
    // Final step of the simulation
    //
    // See that the code is actually running
    
    if (my_ID == 1) {
      wb_motor_set_velocity(left_motor, 0);
      wb_motor_set_velocity(right_motor, 0);
    }
    else {
      wb_motor_set_velocity(left_motor, 2);
      wb_motor_set_velocity(right_motor, 2);
    }
  }

  wb_robot_cleanup();

  return 0;
}
