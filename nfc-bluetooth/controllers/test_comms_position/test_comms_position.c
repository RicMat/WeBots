// To Do: 
// - Add a control so there cannot be two team inglobations/joins at the same time
//   e.g. robot_11 of team_3 meets robot_2 of team_1, robot_12 of team_3 meets robot_3 of team_2
//   in this case we'd have two robots trying to move team_3 towards team_1 and team_2
// - 

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
#include <cstring.h>

// Custom libraries
#include "OAM.h"
#include "MHM.h"


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
char name[20], filename[20], message[34+1], tmp_s[3+1];
int ps_offset[NB_DIST_SENS] = {0, 0, 0, 0, 0, 0, 0, 0}, i, speed[2];
int rand_num, num, my_ID, counter = 0, run = 1, team_size = 1;
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

void ObstacleAvoidanceModule (void) {
  obstacle_avoidance();
}

////////////////////////////////////////////
// MHM - Message Handling Module
////////////////////////////////////////////

char code_in[4], ext_ID_s[4], receiver_ID[4], leader_ID_s[4], ext_leader_ID[4], ttl_s[4], ext_team_player, ext_leader, last_queued, ext_team_size;
bool leader = false, team_player = false, in_queue = false;
int leader_ID = 0, ext_leader_ID = 0, idx_comms = 0, idx_team = 0, ext_ID = 0, external_team_connection_ID = 0, tmp = 0, ttl = 0;
int comms_queue[7], team_IDs[7];
// comms queue is useless as we are using mesh flooding so it equals to seending broadcast with ttl = 0







////////
// Important
// To Do: check team id - external messages shall not be broadcasted outside of the team
////////






void join_external_team(char* ext_ID_s, char* ext_leader) {
  /* join other bot in a new team */
  /* sends a series of LJT messages for each bot in the team */
  /* if not the leader, update all the slaves of the change */
  /* if the leader, update the status to slave */
  
  /* message structure: 
    
    SSS - code_in - code of the message
    SSS - my_ID - ID of the sending bot
    SSS - ext_ID_s - ID of the receiving robot (for comms)
    SSS - TTL - TTL flag
    S - last_queued - Y if this is the last bot in queue
    
  */
  if (idx_team > 0) {
    construct_join_team_message(my_ID_s, ext_ID_s, "N");
  }
  else {
    construct_join_team_message(my_ID_s, ext_ID_s, "Y");
  }
  wb_emitter_send(emitter_bt, message, strlen(message) + 1);
  
  for (i=0; i<idx_team; i++) {
    // send message to the bot we are joining
    // sprintf(tmp_message, "LJT%03d%sN", team_IDs[i], ext_ID_s);
    
    if(i == idx_team-1) {
      construct_join_team_message(team_IDs[i], ext_ID_s, "Y");
    }
    else {
      construct_join_team_message(team_IDs[i], ext_ID_s, "N");
    }
    wb_emitter_send(emitter_bt, message, strlen(message) + 1);
    
    // send message to the bot we are transferring
    construct_transfer_team_message(my_ID_s, team_IDs[i], ext_leader_ID);
    wb_emitter_send(emitter_bt, message, strlen(message) + 1);
    ////////
    // To Do: Update Location
    ////////
    
  }
}

void inglobate_external_team(char* ext_ID_s, char last_queued) {
  /*
  
  If we are the leader we should share the message with all our team
  If we are not the leader we should share the info with our leader
  
  */
  if (leader) {
    /* TTL set to 0 as it should reach every other bot in the team in a single hop */
    construct_share_with_team_message("ITM", my_ID_s, "000", "000", ext_ID_s); 
    // To Do: add location info
  }
  else {
    /* TTL set to 3 as it should be enough to travel the whole team */
    construct_share_with_team_message("ITM", my_ID_s, "000", "003", ext_ID_s);
  }
  wb_emitter_send(emitter_bt, message, strlen(message) + 1);
  
  /* Save data of the new bot */
  team_IDs[idx_team] = atoi(ext_ID_s);
  idx_team += 1;
  
  /* Update my location */
  ////////
  // To Do: update my location after the new bot has joined
  ////////
  
}

void handle_message(char* buffer) {
  /*
  
  Fixed:
  
  SSS - code_in - message code as string
  SSS - ext_ID_s - external ID as string (sender)
  SSS - receiver_ID - ID of the receiver as string
  SSS - TTL - TTL
  
  */
  
  /* check if this message was already received */
  // To Do: if message already received discard it. Consider also case where the sender id is different but the message is the same
  
  strncpy(code_in, buffer, 3);
  strncpy(ext_ID_s, buffer+3, 3);
  ext_ID = atoi(ext_ID_s);
  strncpy(receiver_ID, buffer+6, 3);
  strncpy(ttl_s, buffer+9, 3);
  ttl = atoi(ttl_s);
  
  if (atoi(receiver_ID) != my_ID || atoi(receiver_ID) != 0) { // might add relay and or TTL
    /* message is neither for this robot nor for broadcast */
    return;
  }
  
  ///////////
  //
  // To Do: increase team_size
  //
  ///////////
  
  if (ttl > 0) {
    strncpy(message, buffer, 9);
    strcat(message, "%03d", ttl-1);
    strcat(message, "%s", buffer+12);
    wb_emitter_send(emitter_bt, message, strlen(message) + 1);
  }
  
  switch(code_in)
  {
    /* 
    
    Variable:
    
    S   - external_team_player - Y if external bot is in a team
    s   - external_team_size - size of the external team
    S   - ext_leader - Y if the external bot is the leader of its team
    SSS - ext_leader_ID_s - ID of the external team's leader
    
    */
    case "DNB": // Discover New Bot
      strncpy(ext_team_player, buffer+12, 1);
      strncpy(ext_team_size, buffer+13, 1);
      strncpy(ext_leader, buffer+14, 1);
      strncpy(ext_leader_ID_s, buffer+15, 3);
      ext_leader_ID = atoi(ext_leader_ID_s);
      
      tmp = atoi(ext_team_size);
      if (tmp + team_size > 7) {
        // To Do:
        reject_union_team();
        // To Do: add the bot to external queue
        break;
      }
      
      for (i=0; i<idx_team; i++) {
        if(team_IDs[i] == ext_ID) {
          in_queue = true; // discard this communication
        }
      }
      if (! in_queue) {
        /* New robot - process received information */
        // team_IDs[idx] = ext_ID;
        if (ext_team_player) {
          /* external bot has a team */
          if (leader_ID < ext_leader_ID) {
            /* inglobate other team */
            /* to be done at message level - LJT message */
            break;
          }
          else {
            /* join other team */
            /* sends a series of LJT messages for each bot in the team */
            join_external_team(ext_ID_s, ext_leader_ID_s);
          }
        }
        else {
          /* external bot has not a team */
          /* check who has the lowest ID to determine who is the leader */
          if (leader_ID < ext_ID) {
            /* external bot will join my team */
            /* to be done at message level - LJT message */
            break; // To Do: Maybe send location for supplementary bots
          }
          else {
            /* join other bot in a new team */
            /* sends a series of LJT messages for each bot in the team */
            /* since it's only a single bot, we might just swap the ID of the bot 
               and the ID of actual leader so we do not have to transfer the whole data */
            join_external_team(ext_ID_s, ext_leader_ID_s);
          }
        }
      }
      break;
    /*
    
    Variable:
    
    S - last_queued - Y if this is the last bot in queue of the external bot
    
    */ 
    case "LJT": // List of other team-members to Join the Team
      strncpy(last_queued, buffer+12, 1);
      inglobate_external_team(ext_ID_s, last_queued);
      break;
    /*
    
    Variable:
    
    SSS - external_leader_ID - ID of the leader of the external team
    
    */
    case "TTT": // Transfer To the new Team
      strncpy(leader_ID_s, buffer+12, 3);
      leader_ID = atoi(leader_ID_s); // set up new leader
      
      //////////////
      // To Do: set up new location
      //////////////
      break;
    /*
    
    Variable:
    
    SSS - ext_ID_s - ID of the external bot we are inglobating
    
    */
    case "ITM": // Inglobate Team Member
      team_IDs[idx_team] = atoi(ext_ID_s);
      idx_team += 1;
      
  } 
}

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
  const char *my_name = wb_robot_get_name();
  
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
    if (! team_player) {
      RandomizeMovementModule();
    }
    
    
    
    
    
    
    
    
    // Only if leaderrrrrrrrrrr //
    
    
    
    
    
    
    
    
    
    /* Communication */
    /* Communication is execute one step in advance as we need time to */
    /* process the data which will be delivered to the other robot */
    /* one step in the future */
    
    /* Initial message exchange - to be repeated every step to engage new bots */ 
    if (!team_player) {
      construct_discovery_message(my_ID, team_player, leader); // saves the desired string in variable message
    }
    
    // if (! have_leader) { 
      // /* DNB Discover New Bot - ID - No Part of Team - No Leader */
      // snprintf(message, 7, "DNB%dNN", my_ID); 
    // }
    // else {
    
    // }
    wb_emitter_send(emitter_bt, message, strlen(message) + 1);
    
    /* Check for new messages and process them */
    while (wb_receiver_get_queue_length(receiver_bt) > 0) {
      const char *buffer = wb_receiver_get_data(receiver_bt);
      handle_message(buffer);
      wb_receiver_next_packet(receiver_bt);      
    }
    
    /* Move to location */
    if (mtl_active) {
      angle = angle_offset();
      
      if (turn) { /* we are facing the right direction */
        forward_move = false;
        turn_towards_location();
      }
      else { /* turn until we face the right direction */
        forward_move = true;
      }
      
      if (forward_move) {
        distance = calculate_distance_location();
        wb_motor_set_velocity(left_motor, 0.00628 * speed[LEFT]);
        wb_motor_set_velocity(right_motor, 0.00628 * speed[RIGHT]);
      }
      
      /* Arrived at location */
      if (mtl_arrived) {
        mtl_active = false;
        mtl_arrived = false;
      }
    }  
    
    // Set wheel speeds
    // if (my_ID != 10) {
      // wb_motor_set_velocity(left_motor, 0.00628 * speed[LEFT]); // speed[LEFT]);
      // wb_motor_set_velocity(right_motor, 0.00628 * speed[RIGHT]); // speed[RIGHT]);
    // }
    // else {
      // wb_motor_set_velocity(left_motor, 0);
      // wb_motor_set_velocity(right_motor, 0);
    // }
  }

  fclose(fpt);

  return 0;
}
