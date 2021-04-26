// NUE
// Red - X - Nord
// Green - Y - Up
// Blue - Z - East
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <webots/distance_sensor.h>
#include <webots/emitter.h>
#include <webots/motor.h>
#include <webots/receiver.h>
#include <webots/robot.h>
#include <webots/gps.h>
#include <webots/compass.h>

#define SPEED 6
#define TIME_STEP 64
#define COMMUNICATION_CHANNEL 1

double x, y, z, bear, left_speed, right_speed, bearing, rotation;
bool arrived;

double get_bearing_in_degrees(WbDeviceTag compass) {
  const double *north = wb_compass_get_values(compass);
  double rad = atan2(north[0], north[2]);
  double bearing = (rad - 1.5708) / M_PI * 180.0;
  if (bearing < 0.0)
    bearing = bearing + 360.0;
  return bearing;
}

int main() {
  WbDeviceTag emitter, receiver, left_motor, right_motor, gps, compass;
  //int message_printed = 0; /* used to avoid printing continuously the communication state */

  wb_robot_init();

  /* get a handler to the motors and set target position to infinity (speed control). */
  left_motor = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);
  
  emitter = wb_robot_get_device("emitter");
  wb_emitter_set_channel(emitter, COMMUNICATION_CHANNEL);
  wb_emitter_set_range(emitter, 0.15);
  
  receiver = wb_robot_get_device("receiver");
  wb_receiver_enable(receiver, TIME_STEP);
  
  gps = wb_robot_get_device("gps");
  wb_gps_enable(gps, TIME_STEP);
  
  compass = wb_robot_get_device("compass");
  wb_compass_enable(compass, TIME_STEP);
  
  arrived = false;
  
  while (wb_robot_step(TIME_STEP) != -1) {
  
    const double *values = wb_gps_get_values(gps);
    x = values[0];
    y = values[1];
    z = values[2];
    const double *north = wb_compass_get_values(compass);
    
    bearing = get_bearing_in_degrees(compass);
    // bear = (atan2(x, z) / M_PI) * 180.0;
    if (x < 0) {
      bear = 90 - (atan2(x, z) * 180 / M_PI);
      // bear = bear;
    } else {
      bear = 270 - (atan2(x, z) * 180 / M_PI);
      // bear = bear;
    }
     
    // const double *north = wb_compass_get_values(compass);
    
    if (strncmp(wb_robot_get_name(), "epuck1", 6) == 0) {
      printf("I'm %s and my coords are: %f x, %f y, and %f z\n", wb_robot_get_name(), x, y ,z);
      // printf("I'm heading: %f %f\n", 1 - (atan(x / z) * 180.0 / (45 * M_PI)), (atan(x / z) * 180.0 / (45 * M_PI)));
      // printf("Bear: %f\n", 180 + bear);
      printf("Bearing: %f\n", bearing);
      printf("I'm heading: %f x, %f y, and %f z\n", north[0], north[1], north[2]);
    }
    /*
    if (x < 0.1 && z < 0.1) {
      wb_motor_set_velocity(left_motor, 0);
      wb_motor_set_velocity(right_motor, 0);
      break;
    }
    
    if (x > 0) {
    
      if (z > 0) {
        
        bear = 180 + bear;
        rotation = bearing - bear;
        
        if (rotation > 1 || rotation < -1) {
          left_speed = (rotation) / 90;
          right_speed = (rotation) / 90;
        }
        else {
          left_speed = 1;
          right_speed = 1;
        }
        
      }
      else {
      
        bear = 360 + bear;
        rotation = bearing - bear;
        // printf("rotation %f\n", rotation);
        
        if (rotation > 1 || rotation < -1) {
          left_speed = (rotation) / 90;
          right_speed = (rotation) / 90;
        }
        else {
          left_speed = 1;
          right_speed = 1;
        }
        
      }
    
    }
    else {
    
      if (z > 0) {
        
        bear = bear - 270;
        rotation = bearing - bear;
        
        if (rotation > 1 || rotation < -1) {
          left_speed = (rotation) / 90;
          right_speed = (rotation) / 90;
        }
        else {
          left_speed = 1;
          right_speed = 1;
        }
        
      }
      else {
      
        bear = 270 - bear;
        rotation = bearing - bear;
        
        if (rotation > 1 || rotation < -1) {
          left_speed = (rotation) / 90;
          right_speed = (rotation) / 90;
        }
        else {
          left_speed = 1;
          right_speed = 1;
        }
        
      }
      
    }
    */
    
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
    
     
    
    ////////////////////////////////////////////
    // Final step of the simulation
    //
    // See that the code is actually running
    
    // if (bearing - bear > 2 || bearing - bear < -2) {
      // left_speed = (- bearing + bear) / 90;
      // right_speed = (- bear + bearing) / 90;
    // }
    
    // else {
      left_speed = -0.2;
      right_speed = 0.2;
    // }
    wb_motor_set_velocity(left_motor, left_speed);
    wb_motor_set_velocity(right_motor, right_speed);
  
  }

  wb_robot_cleanup();

  return 0;
}
