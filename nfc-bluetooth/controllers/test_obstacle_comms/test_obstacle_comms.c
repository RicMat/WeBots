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
 
/* To increase the probability:
  - Go only Nort East South West
  - When meet an obstacle, turn towards it
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <math.h>
#include <webots/distance_sensor.h>
#include <webots/led.h>
#include <webots/light_sensor.h>
#include <webots/emitter.h>
#include <webots/motor.h>
#include <webots/receiver.h>
#include <webots/robot.h>
#include <webots/supervisor.h>

// Global defines
#define TRUE 1
#define FALSE 0
#define NO_SIDE -1
#define LEFT 0
#define RIGHT 1
#define WHITE 0
#define BLACK 1
#define TIME_STEP 64  // [ms]
#define COMMUNICATION_CHANNEL 1
#define COMMUNICATION_CHANNEL_BT 2
#define FLOOR_SIZE 10.0

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
// const int PS_OFFSET_SIMULATION[NB_DIST_SENS] = {300, 300, 300, 300, 300, 300, 300, 300};
const int PS_OFFSET_SIMULATION[NB_DIST_SENS] = {80, 80, 80, 80, 80, 80, 80, 80};
WbDeviceTag ps[NB_DIST_SENS]; /* proximity sensors */
int ps_value[NB_DIST_SENS] = {0, 0, 0, 0, 0, 0, 0, 0};

// Motors
WbDeviceTag left_motor, right_motor;

// LEDs
#define NB_LEDS 8
WbDeviceTag led[NB_LEDS];

// Comms
WbDeviceTag emitter, receiver;

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

int oam_active, oam_reset, oam_obst, obst_direction;
int oam_speed[2];
int oam_side = NO_SIDE;
int message_sent = FALSE;
int oam_reset_counter = 0;

//0.2 0.9 1.2
// #define OAM_OBST_THRESHOLD 150
// #define OAM_FORWARD_SPEED 150
#define OAM_OBST_THRESHOLD 30
#define OAM_FORWARD_SPEED 150
#define OAM_K_PS_90 0.7 
#define OAM_K_PS_45 1.0 
#define OAM_K_PS_00 1.4 
#define OAM_K_MAX_DELTAS 600
int kkk = 0;

void ObstacleAvoidanceModule(void) {
  int max_ds_value, i;
  int Activation[] = {0, 0};

  // Module RESET
  if (oam_reset) {
    oam_active = FALSE;
    oam_side = NO_SIDE;
    oam_obst = FALSE;
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
  if (max_ds_value > OAM_OBST_THRESHOLD){
    oam_active = TRUE;
    oam_reset_counter = 0;
  }
  else {
    oam_reset_counter += 1;
    if (oam_reset_counter > 40) {
      oam_reset = TRUE;
      message_sent = FALSE;
    }
  }
  /* Check for obstacles, turn in place to look for connections */
  max_ds_value = 0;
  for (i = PS_RIGHT_00; i <= PS_LEFT_00; i++) {
    if (max_ds_value < ps_value[i]) {
      max_ds_value = ps_value[i];
      obst_direction = i;
    }
  }
  if (max_ds_value > OAM_OBST_THRESHOLD){
    oam_obst = TRUE;
    // printf("%d\n", obst_direction);
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
    // if (strncmp(wb_robot_get_name(), "epuck1", 7) == 0) {
      // printf("test\n"); 
    // }
    int DeltaS = 0;
    // The rotation of the robot is determined by the location and the side of the obstacle
    if (oam_side == LEFT) {
      // printf("left %d %s\n", oam_active, wb_robot_get_name());
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










  // int i;

  // wb_robot_init();

  
  // const WbNodeRef root_node = wb_supervisor_node_get_root();
  // const WbFieldRef root_children_field = wb_supervisor_node_get_field(root_node, "children");
  // const int n = wb_supervisor_field_get_count(root_children_field);
  // printf("This world contains %d nodes:\n", n);

  // for (i = 0; i < n; i++) {
    // node = wb_supervisor_field_get_mf_node(root_children_field, i);
    // printf("-> %s\n", wb_supervisor_node_get_type_name(node));
  // }
  // printf("\n");

  // node = wb_supervisor_field_get_mf_node(root_children_field, 0);
  // field = wb_supervisor_node_get_field(node, "gravity");
  // const double gravity = wb_supervisor_field_get_sf_float(field);
  // printf("WorldInfo.gravity = %g\n\n", gravity);

  // wb_supervisor_set_label(0, "Going to move the location of the PointLight\nin 2 seconds (simulation time)...", 0.0, 0.0, 0.1,
                          // 0x00FF00, 0.1, "Georgia");
  // printf("Going to move the location of the PointLight in 2 seconds (simulation time)...\n");
  // wb_robot_step(2000);                                             // wait for 2 seconds
  // node = wb_supervisor_field_get_mf_node(root_children_field, 3);  // PointLight
  // field = wb_supervisor_node_get_field(node, "location");
  // const double location[3] = {0.5, 0.3, 0.5};
  // wb_supervisor_field_set_sf_vec3f(field, location);





WbNodeRef node;
WbFieldRef field;

void reset_simulation(void){
  double translation[3] = {0.0, 0.0, 0.0};
  field = wb_supervisor_node_get_field(node, "translation");
  translation[0] = (float)rand() / (float)(RAND_MAX / (0.92 * FLOOR_SIZE)) - (0.46 * FLOOR_SIZE);
  translation[1] = 0.0;
  translation[2] = (float)rand() / (float)(RAND_MAX / (0.92 * FLOOR_SIZE)) - (0.46 * FLOOR_SIZE);
  wb_supervisor_field_set_sf_vec3f(field, translation);
  double rotation[4] = {0.0, 1.0, 0.0, 0.0};
  field = wb_supervisor_node_get_field(node, "rotation");
  rotation[3] = (float)rand() / (float)(RAND_MAX / 6.28319);
  wb_supervisor_field_set_sf_rotation(field, rotation);
  //fprintf(fpt,"%f\n", wb_robot_get_time());
        
}








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
  int rand_num = ((float)rand() / (float)(RAND_MAX / (200 * FLOOR_SIZE))); // + (10 * atoi(wb_robot_get_name() + 5));
  int num = 0;
  float turn = (1.0 + ((float)(rand() % 6) / 10.0));
  
  for (i = 0; i < NB_DIST_SENS; i++)
    ps_offset[i] = PS_OFFSET_SIMULATION[i];
  wb_motor_set_velocity(left_motor, 0);
  wb_motor_set_velocity(right_motor, 0);
  // wb_robot_step(TIME_STEP);  // Just run one step to make sure we get correct sensor values
  
  // Start communication
  // emitter = wb_robot_get_device("nfc_e");
  // wb_emitter_set_channel(emitter, COMMUNICATION_CHANNEL);
  // wb_emitter_set_range(emitter, 0.06);
  
  emitter = wb_robot_get_device("bt_e");
  wb_emitter_set_channel(emitter, COMMUNICATION_CHANNEL_BT);
  wb_emitter_set_range(emitter, 1.0);
  
  // receiver = wb_robot_get_device("nfc_r");
  // wb_receiver_enable(receiver, TIME_STEP);
  
  receiver = wb_robot_get_device("bt_r");
  wb_receiver_enable(receiver, TIME_STEP);
  
  int my_ID = atoi(wb_robot_get_name() + 5);
  int counter = 0;
  
  const WbNodeRef root_node = wb_supervisor_node_get_root();
  const WbFieldRef root_children_field = wb_supervisor_node_get_field(root_node, "children");
  const int n = wb_supervisor_field_get_count(root_children_field);
  
  // better to work on DEFs
  const char* namee;
  for (i = 0; i < n; i++) {
    node = wb_supervisor_field_get_mf_node(root_children_field, i);
    namee = wb_supervisor_node_get_type_name(node);
    if (strncmp(namee, "E-puck", 7) == 0){
      node = wb_supervisor_field_get_mf_node(root_children_field, i);
      field = wb_supervisor_node_get_field(node, "name");
      // printf("%s", field);
      const char* idd = wb_supervisor_field_get_sf_string(field);
      // printf("%s %d\n", idd, my_ID);
      if (atoi(idd + 5) == my_ID){ // we found this robot
        reset_simulation();
        break;
      }
    }
  } 
  
  // print to file
  int run = 1;
  FILE *fpt;
  char filename[20];
  sprintf(filename, "Times%d_%d.txt", run, my_ID);
  fpt = fopen(filename, "w");
  // char data[50];
  if (strncmp(wb_robot_get_name(), "epuck1", 7) == 0){
    printf("Run%d Size:%.1f\n", run, FLOOR_SIZE);
    fprintf(fpt, "Run%d Size:%.1f\n", run, FLOOR_SIZE);
  }
    
  while(wb_robot_step(TIME_STEP) != -1 && wb_robot_get_time() < (10800 * 5)) {  // Main loop for max 3 hours
    
    // reset simulation

    if (wb_robot_get_time() >= (10800 * run)){
      reset_simulation(); // small probability of starting on the same point
      run += 1;
      if (strncmp(wb_robot_get_name(), "epuck1", 7) == 0){
        printf("\nRun%d Size:%.1f\n", run, FLOOR_SIZE);
        fprintf(fpt, "\nRun%d Size:%.1f\n", run, FLOOR_SIZE);
      }
      //fpt = fopen(filename, "w+");
    }
    // read sensors value
    for (i = 0; i < NB_DIST_SENS; i++){
      // if (my_ID == 1)
        // printf("%d\n", (int)wb_distance_sensor_get_value(ps[i]));
      
      ps_value[i] = (((int)wb_distance_sensor_get_value(ps[i]) - ps_offset[i]) < 0) ?
                      0 :
                      ((int)wb_distance_sensor_get_value(ps[i]) - ps_offset[i]);
      // if (my_ID == 1)
        // printf("%d\n", (int)(ps_value[i]));
      
      // printf("%d ", ps_value[i]);
    }
    // printf("\n");
    // *** START OF SUBSUMPTION ARCHITECTURE ***

    
    speed[LEFT] = 200;
    speed[RIGHT] = 200;

    // OAM - Obstacle Avoidance Module
    ObstacleAvoidanceModule();
    // if (strncmp(wb_robot_get_name(),"epuck1", 6)){
      // printf("oam %d\n", oam_active);
      // printf("sent %d\n", message_sent);
    // }
    
    // if (oam_obst) {
      // for (i=0; i<obst_direction; i++) {
        // wb_motor_set_velocity(left_motor, -0.00628 * 150);
        // wb_motor_set_velocity(right_motor, 0.00628 * 150);
        // wb_robot_step(TIME_STEP);
        // printf("----\n");
      // }
      // oam_obst = FALSE;
    // }
    // OFM - Obstacle Following Module
    // ObstacleFollowingModule(oam_side);
    
    if (1 > 0) { // message_sent == FALSE && oam_active == TRUE
      // printf("sending\n");
      const char *message = wb_robot_get_name();
      wb_emitter_send(emitter, message, strlen(message) + 1); // send my ID continuously because we have no bumper here
      message_sent = TRUE;
      
    }
    // else {
      // if (strncmp(wb_robot_get_name(),"epuck1", 6)){
        // printf("not colliding\n");
      // }
    // }
    // fprintf(fpt, "%d\n", (int)wb_robot_get_time());
    // fflush(fpt);
    if (wb_receiver_get_queue_length(receiver) > 0) {
      const char *buffer = wb_receiver_get_data(receiver);
      // printf("%s at time %ld\n", buffer, time(NULL));
      if (my_ID < atoi(buffer + 5)){
        // printf("%s at time %f\n", buffer,  wb_robot_get_time());
        // snprintf(data, 50, "%f", wb_robot_get_time());
        // fprintf(fpt,"%s\n", data);
        // printf("%.2f\n", fmod(wb_robot_get_time(),10800));
        fprintf(fpt, "%d\n", (int)round(wb_robot_get_time())%10800);
        // fflush(fpt);
        counter += 1;
      }
      wb_receiver_next_packet(receiver);
      // printf("%s at time %d\n", buffer, wb_receiver_get_queue_length(receiver));
      
    }

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
        rand_num = ((float)rand() / (float)(RAND_MAX / (200 * FLOOR_SIZE))); // + (rand() % 10 * atoi(wb_robot_get_name() + 5) + 1);
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
    if (my_ID != 10) {
      wb_motor_set_velocity(left_motor, 0.00628 * speed[LEFT]); // speed[LEFT]);
      wb_motor_set_velocity(right_motor, 0.00628 * speed[RIGHT]); // speed[RIGHT]);
    }
    else {
      wb_motor_set_velocity(left_motor, 0);
      wb_motor_set_velocity(right_motor, 0);
    }
  }
  // for (i=0; i<=counter; i++) {
    // fprintf(fpt,"%f\n", data[i]);
    // printf("%f\n", data[i]);
  // }
  fclose(fpt);

  return 0;
}
