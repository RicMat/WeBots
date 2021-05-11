// NUE
// Red - X - Nord y 
// Green - Y - Up z
// Blue - Z - East x
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
#define COMMUNICATION_CHANNEL 2

double x, y, z, bear, left_speed, right_speed, bearing, rotation;
bool arrived;

double get_bearing_in_degrees(WbDeviceTag compass) {
  const double *north = wb_compass_get_values(compass);
  double rad = atan2(north[2], north[0]);
  double bearing = (rad) * 180.0 / M_PI;
  if (bearing < 0.0)
    bearing = 360 +  bearing;
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
  
  emitter = wb_robot_get_device("bt_e");
  wb_emitter_set_channel(emitter, COMMUNICATION_CHANNEL);
  wb_emitter_set_range(emitter, 1);
  
  receiver = wb_robot_get_device("bt_r");
  wb_receiver_enable(receiver, TIME_STEP);
  
  gps = wb_robot_get_device("gps");
  wb_gps_enable(gps, TIME_STEP);
  
  compass = wb_robot_get_device("compass");
  wb_compass_enable(compass, TIME_STEP);
  
  arrived = false;
  
  char message[34+1];
  int my_ID = atoi(wb_robot_get_name() + 5);
  // sprintf(message, "%03d", my_ID);
  // wb_emitter_send(emitter, message, strlen(message) + 1);
  
  
  wb_robot_step(TIME_STEP);
    double *values = (double* )wb_gps_get_values(gps);
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
    
    sprintf(message, "%07.3f%07.3f", x, z);
    printf("sending %f %f in message %s\n", x, z, message);
    wb_emitter_send(emitter, message, strlen(message) + 1);
    
    // const double *north = wb_compass_get_values(compass);
    
    // if (strncmp(wb_robot_get_name(), "epuck1", 6) == 0) {
      // printf("I'm %s and my coords are: %f x, %f y, and %f z\n", wb_robot_get_name(), x, y ,z);
      // printf("I'm heading: %f %f\n", 1 - (atan(x / z) * 180.0 / (45 * M_PI)), (atan(x / z) * 180.0 / (45 * M_PI)));
      // printf("Bear: %f\n", 180 + bear);
      // printf("Bearing: %f\n", bearing);
      // printf("I'm heading: %f x, %f y, and %f z\n", north[0], north[1], north[2]);
    // }
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
  float my_x;
  float X;
  float my_z;
  float Z;
  float angle;
  
  while (wb_robot_step(TIME_STEP) != -1) {
    ////////////////////////////////////////////
    // Test basic communication between the robots
    //
    // Broadcast your ID as message to all the other robots
    
    
    if (wb_receiver_get_queue_length(receiver) > 0) {
        /* read current packet's data */
        const char *buffer = wb_receiver_get_data(receiver);
        char tmp[14+1];
        X = atof(strncpy(tmp, buffer, 7));
        Z = atof(strncpy(tmp, buffer+7, 7));
        printf("received %7.3f %7.3f\n", X, Z);
        /* fetch next packet */
        wb_receiver_next_packet(receiver);
      } 
      
    
    if (my_ID == 2) {
      printf("x = %f\n", x);
      printf("z = %f\n", z);
      my_x = x - X;
      my_z = z - Z;
      printf("my_x = %f\n", my_x);
      printf("my_z = %f\n", my_z);
      // rotate
      angle = atan2(my_x, my_z);
      printf("angle rad %f\n", angle);
      
      // We need to map to coord system when 0 degree is at 3 O'clock, 270 at 12 O'clock
      if (my_x < 0) {
        angle = 360 + (angle * 180 / M_PI);
      }
      else {
        angle = angle * 180 / M_PI;
      }
      // angle = angle * 180 / M_PI;
      printf("angle %f\n", angle);
      
      printf("compass %f\n", get_bearing_in_degrees(compass));
    }
    
    values = (double* )wb_gps_get_values(gps);
    x = values[0];
    y = values[1];
    z = values[2];
    
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
      // left_speed = -0.2;
      // right_speed = 0.2;
    // }
    wb_motor_set_velocity(left_motor, left_speed);
    wb_motor_set_velocity(right_motor, right_speed);
  
  }

  // wb_robot_cleanup();

  return 0;
}
