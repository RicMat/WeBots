#include <stdio.h>
#include <string.h>
#include <webots/distance_sensor.h>
#include <webots/emitter.h>
#include <webots/motor.h>
#include <webots/receiver.h>
#include <webots/robot.h>

#define SPEED 6
#define TIME_STEP 64
#define COMMUNICATION_CHANNEL 1

int main() {
  WbDeviceTag communication, left_motor, right_motor;
  int message_printed = 0; /* used to avoid printing continuously the communication state */
  double left_speed, right_speed;

  wb_robot_init();

  /* get a handler to the motors and set target position to infinity (speed control). */
  left_motor = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);
  
  /*
   * as we are using the same controller for the emitter and the reciever,
   * we need to differentiate them
   */
  
  emitter = wb_robot_get_device("emitter");
  wb_emitter_set_channel(emitter, COMMUNICATION_CHANNEL);
  
  receiver = wb_robot_get_device("receiver");
  wb_receiver_enable(receiver, TIME_STEP);
    
  while (wb_robot_step(TIME_STEP) != -1) {
    /*
     * the emitter simply sends the message but the receiver
     * has to check if there is something before it can reads the buffer.
     */
      /* send null-terminated message */
    const char *message = "Hello!";
    wb_emitter_send(emitter, message, strlen(message) + 1);
    
    if (wb_receiver_get_queue_length(receiver) > 0) {
        /* read current packet's data */
        const char *buffer = wb_receiver_get_data(receiver);

        if (message_printed != 1) {
          /* print null-terminated message */
          printf("Communicating: received \"%s\"\n", buffer);
          message_printed = 1;
        }
        /* fetch next packet */
        wb_receiver_next_packet(receiver);
      } else {
        if (message_printed != 2) {
          printf("Communication broken!\n");
          message_printed = 2;
        }
      }
    }

    

    /* set the motor speeds. */
    wb_motor_set_velocity(left_motor, 1);
    wb_motor_set_velocity(right_motor, 1);
  }

  wb_robot_cleanup();

  return 0;
}
