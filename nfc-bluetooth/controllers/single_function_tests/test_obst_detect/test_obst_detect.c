/*
 * Copyright 1996-2020 Cyberbotics Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <webots/distance_sensor.h>
#include <webots/led.h>
#include <webots/light_sensor.h>
#include <webots/motor.h>
#include <webots/robot.h>

// Global defines
#define TRUE 1
#define FALSE 0
#define NO_SIDE -1
#define LEFT 0
#define RIGHT 1
#define WHITE 0
#define BLACK 1
#define TIME_STEP 64  // [ms]

// 8 IR proximity sensors
#define NB_DIST_SENS 8
#define PS_RIGHT_00 0
#define PS_RIGHT_45 1
#define PS_RIGHT_90 2
#define PS_RIGHT_REAR 3
#define PS_LEFT_REAR 4
#define PS_LEFT_90 5
#define PS_LEFT_45 6
#define PS_LEFT_00 7
const int PS_OFFSET_SIMULATION[NB_DIST_SENS] = {80, 80, 80, 80, 80, 80, 80, 80};
WbDeviceTag ps[NB_DIST_SENS]; /* proximity sensors */
int ps_value[NB_DIST_SENS] = {0, 0, 0, 0, 0, 0, 0, 0};

// Motors
WbDeviceTag left_motor, right_motor;

// LEDs
#define NB_LEDS 8
WbDeviceTag led[NB_LEDS];

//------------------------------------------------------------------------------
//
//    BEHAVIORAL MODULES
//
//------------------------------------------------------------------------------



////////////////////////////////////////////
// OAM - Obstacle Avoidance Module
//
// The OAM routine first detects obstacles in front of the robot, then records
// their side in "oam_side" and avoid the detected obstacle by
// turning away according to very simple weighted connections between
// proximity sensors and motors. "oam_active" becomes active when as soon as
// an object is detected and "oam_reset" inactivates the module and set
// "oam_side" to NO_SIDE. Output speeds are in oam_speed[LEFT] and oam_speed[RIGHT].

int oam_active, oam_reset;
int oam_speed[2];
int oam_side = NO_SIDE;

//0.2 0.9 1.2
#define OAM_OBST_THRESHOLD 80
#define OAM_FORWARD_SPEED 150
#define OAM_K_PS_90 0.9 
#define OAM_K_PS_45 1.3 
#define OAM_K_PS_00 1.7 
#define OAM_K_MAX_DELTAS 600

void ObstacleAvoidanceModule(void) {
  int max_ds_value, i;
  int Activation[] = {0, 0};

  // Module RESET
  if (oam_reset) {
    oam_active = FALSE;
    oam_side = NO_SIDE;
  }
  oam_reset = 0;

  // Determine the presence and the side of an obstacle
  max_ds_value = 0;
  for (i = PS_RIGHT_00; i <= PS_RIGHT_45; i++) {
    if (max_ds_value < ps_value[i])
      max_ds_value = ps_value[i];
    Activation[RIGHT] += ps_value[i];
  }
  for (i = PS_LEFT_45; i <= PS_LEFT_00; i++) {
    if (max_ds_value < ps_value[i])
      max_ds_value = ps_value[i];
    Activation[LEFT] += ps_value[i];
  }
  if (max_ds_value > OAM_OBST_THRESHOLD)
    oam_active = TRUE;
  else {
    oam_reset = TRUE;
  }

  if (oam_active && oam_side == NO_SIDE)  // check for side of obstacle only when not already detected
  {
    if (Activation[RIGHT] > Activation[LEFT])
      oam_side = RIGHT;
    else
      oam_side = LEFT;
  }
  
  // for (i = 0; i < NB_DIST_SENS; i++){
    // printf("%d ", ps_value[i]);
  // }
  // printf("\n");

  // Forward speed
  oam_speed[LEFT] = OAM_FORWARD_SPEED;
  oam_speed[RIGHT] = OAM_FORWARD_SPEED;

  // Go away from obstacle
  if (oam_active) {
    int DeltaS = 0;
    // The rotation of the robot is determined by the location and the side of the obstacle
    if (oam_side == LEFT) {
      // printf("left\n");
      //(((ps_value[PS_LEFT_90]-PS_OFFSET)<0)?0:(ps_value[PS_LEFT_90]-PS_OFFSET)));
      DeltaS -= (int)(OAM_K_PS_90 * ps_value[PS_LEFT_90]);
      //(((ps_value[PS_LEFT_45]-PS_OFFSET)<0)?0:(ps_value[PS_LEFT_45]-PS_OFFSET)));
      DeltaS -= (int)(OAM_K_PS_45 * ps_value[PS_LEFT_45]);
      //(((ps_value[PS_LEFT_00]-PS_OFFSET)<0)?0:(ps_value[PS_LEFT_00]-PS_OFFSET)));
      DeltaS -= (int)(OAM_K_PS_00 * ps_value[PS_LEFT_00]);
    } else {  // oam_side == RIGHT
      // printf("right\n");
      //(((ps_value[PS_RIGHT_90]-PS_OFFSET)<0)?0:(ps_value[PS_RIGHT_90]-PS_OFFSET)));
      DeltaS += (int)(OAM_K_PS_90 * ps_value[PS_RIGHT_90]);
      //(((ps_value[PS_RIGHT_45]-PS_OFFSET)<0)?0:(ps_value[PS_RIGHT_45]-PS_OFFSET)));
      DeltaS += (int)(OAM_K_PS_45 * ps_value[PS_RIGHT_45]);
      //(((ps_value[PS_RIGHT_00]-PS_OFFSET)<0)?0:(ps_value[PS_RIGHT_00]-PS_OFFSET)));
      DeltaS += (int)(OAM_K_PS_00 * ps_value[PS_RIGHT_00]);
    }
    if (DeltaS > OAM_K_MAX_DELTAS)
      DeltaS = OAM_K_MAX_DELTAS;
    if (DeltaS < -OAM_K_MAX_DELTAS)
      DeltaS = -OAM_K_MAX_DELTAS;

    // Set speeds
    oam_speed[LEFT] -= DeltaS;
    oam_speed[RIGHT] += DeltaS;
  }
}


//------------------------------------------------------------------------------
//
//    CONTROLLER
//
//------------------------------------------------------------------------------

////////////////////////////////////////////
// Main
int main() {
  int ps_offset[NB_DIST_SENS] = {0, 0, 0, 0, 0, 0, 0, 0}, i, speed[2];

  /* intialize Webots */
  wb_robot_init();

  /* initialization */
  char name[20];
  for (i = 0; i < NB_LEDS; i++) {
    sprintf(name, "led%d", i);
    led[i] = wb_robot_get_device(name); /* get a handler to the sensor */
  }
  for (i = 0; i < NB_DIST_SENS; i++) {
    sprintf(name, "ps%d", i);
    ps[i] = wb_robot_get_device(name); /* proximity sensors */
    wb_distance_sensor_enable(ps[i], TIME_STEP);
  }
  
  // motors
  left_motor = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);
  
  srand(time(NULL) + atoi(wb_robot_get_name() + 5));
  int rand_num = (rand() % 200); // + (10 * atoi(wb_robot_get_name() + 5));
  int num = 0;
  float turn = (1.0 + ((float)(rand() % 6) / 10.0));
  
  // for (i = 0; i < NB_DIST_SENS; i++)
    // ps_offset[i] = PS_OFFSET_SIMULATION[i];
  wb_motor_set_velocity(left_motor, 0);
  wb_motor_set_velocity(right_motor, 0);
  // wb_robot_step(TIME_STEP);  // Just run one step to make sure we get correct sensor values

  
  for (;;) {  // Main loop
    // Run one simulation step
    wb_robot_step(TIME_STEP);

    
    // read sensors value
    for (i = 0; i < NB_DIST_SENS; i++){
      ps_value[i] = (((int)wb_distance_sensor_get_value(ps[i]) - ps_offset[i]) < 0) ?
                      0 :
                      ((int)wb_distance_sensor_get_value(ps[i]) - ps_offset[i]);
    
      // printf("%d ", ps_value[i]);
    }
    // printf("\n");
    // *** START OF SUBSUMPTION ARCHITECTURE ***

    
    speed[LEFT] = 200;
    speed[RIGHT] = 200;

    // OAM - Obstacle Avoidance Module
    ObstacleAvoidanceModule();

    
    // OFM - Obstacle Following Module
    // ObstacleFollowingModule(oam_side);

    

    // Sum A
    speed[LEFT] = oam_speed[LEFT]; // + ofm_speed[LEFT];
    speed[RIGHT] = oam_speed[RIGHT]; // + ofm_speed[RIGHT];

    // Suppression A
    // if (oam_active || ofm_active) {
      // speed[LEFT] = oam_ofm_speed[LEFT];
      // speed[RIGHT] = oam_ofm_speed[RIGHT];
    // }

    // *** END OF SUBSUMPTION ARCHITECTURE ***

    // Randomize turns
    if (num > rand_num) {
      if (num > rand_num * turn) {
        num = 0;
        turn = (1.0 + ((float)(rand() % 6) / 10.0));
        rand_num = (rand() % 200); // + (rand() % 10 * atoi(wb_robot_get_name() + 5) + 1);
        // printf("Rand num: %d\n", rand_num);
      }
      else {
        if (oam_side == LEFT) {
          speed[RIGHT] = 0;
          num += 1;
        }
        else if (oam_side == RIGHT) {
          speed[LEFT] = 0;
          num += 1;
        }
        else {
          if (rand_num % 2 == 0) {
            speed[RIGHT] = 0;
            num += 1;
          }
          else {
            speed[LEFT] = 0;
            num += 1;
          }
        }
      }
    }
    else {
      num += 1;
    }
    
    // printf("left: %f ----- right: %d\n", turn, speed[RIGHT]); 
    // Set wheel speeds
    wb_motor_set_velocity(left_motor, 0.00628 * speed[LEFT]);
    wb_motor_set_velocity(right_motor, 0.00628 * speed[RIGHT]);
  }
  return 0;
}
