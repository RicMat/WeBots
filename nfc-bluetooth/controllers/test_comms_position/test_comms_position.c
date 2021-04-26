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

// Custom libraries
#include "OAM.h"


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

// Obstacle defines
#define OAM_OBST_THRESHOLD 30
#define OAM_FORWARD_SPEED 150
#define OAM_K_PS_90 0.7 
#define OAM_K_PS_45 1.0 
#define OAM_K_PS_00 1.4 
#define OAM_K_MAX_DELTAS 600

// Motors
WbDeviceTag left_motor, right_motor;

// Comms
WbDeviceTag emitter_nfc, receiver_nfc, emitter_bt, receiver_bt;

// Prox sensors
WbDeviceTag ps[NB_DIST_SENS];

// Supervisor
WbNodeRef node;
WbFieldRef field;

// Variables
FILE *fpt;
char name[20], filename[20];
int ps_offset[NB_DIST_SENS] = {0, 0, 0, 0, 0, 0, 0, 0}, i, speed[2];
int rand_num, num, my_ID, counter = 0, run = 1;
float turn;  

//------------------------------------------------------------------------------
//
//    BEHAVIORAL MODULES
//
//------------------------------------------------------------------------------

////////////////////////////////////////////
// OAM - Obstacle Avoidance Module
////////////////////////////////////////////

int oam_active, oam_reset, oam_obst, obst_direction;
int oam_speed[2];
int oam_side = NO_SIDE;
int message_sent = FALSE;
int oam_reset_counter = 0;

// const int PS_OFFSET_SIMULATION[NB_DIST_SENS] = {300, 300, 300, 300, 300, 300, 300, 300};
const int PS_OFFSET_SIMULATION[NB_DIST_SENS] = {80, 80, 80, 80, 80, 80, 80, 80};
int ps_value[NB_DIST_SENS] = {0, 0, 0, 0, 0, 0, 0, 0};

////////////////////////////////////////////
// PHM - Position Handling Module
////////////////////////////////////////////



////////////////////////////////////////////
// RSM - Reset Simulation Module
////////////////////////////////////////////

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
        
}

////////////////////////////////////////////
// RMM - Randomize Movement Module
////////////////////////////////////////////

void RandomizeMovementModule(void) {
  if (num > rand_num) {
    if (num > rand_num * turn) {
      num = 0;
      turn = (1.0 + ((float)(rand() % 6) / 10.0));
      rand_num = ((float)rand() / (float)(RAND_MAX / (200 * FLOOR_SIZE))); // + (rand() % 10 * atoi(wb_robot_get_name() + 5) + 1);
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
}

//------------------------------------------------------------------------------
//
//    CONTROLLER
//
//------------------------------------------------------------------------------


////////////////////////////////////////////
// Main
int main() {
  
  /* intialize Webots */
  wb_robot_init();
  
  /* Prox Sensors */
  for (i = 0; i < NB_DIST_SENS; i++) {
    sprintf(name, "ps%d", i);
    ps[i] = wb_robot_get_device(name);
    wb_distance_sensor_enable(ps[i], TIME_STEP);
    ps_offset[i] = PS_OFFSET_SIMULATION[i];
  }
  
  /* Motors */
  left_motor = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);
  
  /* Randomizer */
  srand(time(NULL) + atoi(wb_robot_get_name() + 5));
  rand_num = ((float)rand() / (float)(RAND_MAX / (200 * FLOOR_SIZE))); // + (10 * atoi(wb_robot_get_name() + 5));
  num = 0;
  turn = (1.0 + ((float)(rand() % 6) / 10.0));
  
  /* BT Comms */
  
  emitter_bt = wb_robot_get_device("bt_e");
  wb_emitter_set_channel(emitter_bt, COMMUNICATION_CHANNEL_BT);
  wb_emitter_set_range(emitter_bt, 1.0);
  
  receiver_bt = wb_robot_get_device("bt_r");
  wb_receiver_enable(receiver_bt, TIME_STEP);
  
  /* NFC Comms */
  
  // emitter_nfc = wb_robot_get_device("nfc_e");
  // wb_emitter_set_channel(emitter_nfc, COMMUNICATION_CHANNEL_NFC);
  // wb_emitter_set_range(emitter_nfc,  0.06);
  
  // receiver_nfc = wb_robot_get_device("nfc_r");
  // wb_receiver_enable(receiver_nfc, TIME_STEP);
  
  /* Initialization and initial reset*/
  my_ID = atoi(wb_robot_get_name() + 5);
  
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
      const char* idd = wb_supervisor_field_get_sf_string(field);
      if (atoi(idd + 5) == my_ID){ // we found this robot
        reset_simulation();
        break;
      }
    }
  } 
  
  /* File setup */
  sprintf(filename, "Times%d_%d.txt", run, my_ID);
  fpt = fopen(filename, "w");
  if (strncmp(wb_robot_get_name(), "epuck1", 7) == 0){
    printf("Run%d Size:%.1f\n", run, FLOOR_SIZE);
    fprintf(fpt, "Run%d Size:%.1f\n", run, FLOOR_SIZE);
  }
  
  /* Main Loop */    
  while(wb_robot_step(TIME_STEP) != -1 && wb_robot_get_time() < (10800 * 5)) {  // Main loop - 5 times for max 3 hours each
    
    /* Reset Simulation */
    if (wb_robot_get_time() >= (10800 * run)){
      reset_simulation(); // small probability of starting on the same point
      run += 1;
      if (strncmp(wb_robot_get_name(), "epuck1", 7) == 0){
        printf("\nRun%d Size:%.1f\n", run, FLOOR_SIZE);
        fprintf(fpt, "\nRun%d Size:%.1f\n", run, FLOOR_SIZE);
      }
    }
    
    /* Read Sensors Value */
    for (i = 0; i < NB_DIST_SENS; i++){
      ps_value[i] = (((int)wb_distance_sensor_get_value(ps[i]) - ps_offset[i]) < 0) ?
                      0 :
                      ((int)wb_distance_sensor_get_value(ps[i]) - ps_offset[i]);
    }
    
    /* Initial Speed */
    speed[LEFT] = 200;
    speed[RIGHT] = 200;

    /* Obastacle Avoidance */
    ObstacleAvoidanceModule();
    
    /* Randomize Movement */
    RandomizeMovementModule();
    
    /* Communication */
    /* Communication is execute one step in advance as we need time to */
    /* process the data which will be delivered to the other robot */
    /* one step in the future */
    
    if (1 > 0) { // message_sent == FALSE && oam_active == TRUE
      // printf("sending\n");
      const char *message = wb_robot_get_name();
      wb_emitter_send(emitter_bt, message, strlen(message) + 1); // send my ID continuously because we have no bumper here
      message_sent = TRUE;
      
    }
    // else {
      // if (strncmp(wb_robot_get_name(),"epuck1", 6)){
        // printf("not colliding\n");
      // }
    // }
    // fprintf(fpt, "%d\n", (int)wb_robot_get_time());
    // fflush(fpt);
    if (wb_receiver_get_queue_length(receiver_bt) > 0) {
      const char *buffer = wb_receiver_get_data(receiver_bt);
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
      wb_receiver_next_packet(receiver_bt);
      // printf("%s at time %d\n", buffer, wb_receiver_get_queue_length(receiver));
      
    }    
    
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

  fclose(fpt);

  return 0;
}
