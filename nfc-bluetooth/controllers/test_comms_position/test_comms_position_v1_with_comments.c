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
#include <webots/led.h>
#include <string.h>

// Custom libraries
#include "OAM.h"
#include "MHM.h"


// Global defines
#define SIZE 100
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
#define FLOOR_SIZE 3.0
#define NB_LEDS 8

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

// LEDs
WbDeviceTag leds[NB_LEDS];
WbDeviceTag leddl;

// Supervisor
WbNodeRef node;
WbFieldRef field;

// Variables
FILE *fpt;
char name[20], filename[20], message[34+1];
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

char code_in[4], ext_ID_s[4], my_ID_s[4], receiver_ID_s[4], leader_ID_s[4], ext_leader_ID_s[4];
char ttl_s[4], extra[22+1], tmp_s[34+1], tmp_ss[4];
char ext_team_player, ext_leader, last_queued, ext_team_size;
bool leader = false, team_player = false, in_queue = false, duplicate_message = false;
int leader_ID = 0, team_ID = 0, ext_leader_ID = 0, idx_comms = 0, idx_team = 0, ext_ID = 0;
int receiver_ID = 0, external_team_connection_ID = 0, tmp = 0, ttl = 0;
int comms_queue[7], team_IDs[7];
// comms queue is useless as we are using mesh flooding so it equals to seending broadcast with ttl = 0

// Stored messages - Could use an Hash Table
char messages_lookback[4][SIZE][34+1];
int messages_lookback_size[4] = { 0 };






////////
// Important
// To Do: check team id - external messages shall not be broadcasted outside of the team
////////






void join_external_team(char* ext_ID_s, char* ext_leader_ID_s) {
  /* join other bot in a new team */
  /* sends a series of LJT messages for each bot in the team */
  /* if not the leader, update all the slaves of the change */
  /* if the leader, update the status to slave */
  
  /* message structure: 
    
    SSS - code_in - code of the message
    SSS - my_ID_s - ID of the sending bot
    SSS - ext_ID_s - ID of the receiving robot (for comms)
    SSS - TTL - TTL flag
    S - last_queued - Y if this is the last bot in queue
    
  */
  // printf("... %d joining external team\n", my_ID);
  if (idx_team > 0) {
    construct_join_team_message(my_ID_s, ext_ID_s, "000", 'N');
  }
  else {
    construct_join_team_message(my_ID_s, ext_ID_s, "000", 'Y');
  }
  wb_emitter_send(emitter_bt, message, strlen(message) + 1);

  /* update my stats - we are for sure not the leader */
  leader = false;
  leader_ID = atoi(ext_leader_ID_s);
  // printf("pree %d\n", leader_ID);
  team_IDs[idx_team] = leader_ID;
  idx_team += 1;
  // printf("postt\n");
  
  for (i=0; i<idx_team-1; i++) {
    // send message to the bot we are joining
    // exclude the new leader
    // sprintf(tmp_message, "LJT%03d%sN", team_IDs[i], ext_ID_s);
    
    snprintf(tmp_ss, 4, "%03d", team_IDs[i]);
    if(i == idx_team-1) {
      construct_join_team_message(tmp_ss, ext_ID_s, "000", 'Y');
    }
    else {
      construct_join_team_message(tmp_ss, ext_ID_s, "000", 'N');
    }
    wb_emitter_send(emitter_bt, message, strlen(message) + 1);
    
    // send message to the bot we are transferring
    construct_transfer_team_message(my_ID_s, tmp_ss, "003", leader_ID_s, ext_leader_ID_s);
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

int hash_codes(char* code_in) {
  if (strcmp(code_in, "DNB") == 0)
    return 0;
  else if (strcmp(code_in, "LJT") == 0)
    return 1;
  else if (strcmp(code_in, "TTT") == 0)
    return 2;
  else if (strcmp(code_in, "ITM") == 0)
    return 3;
  else
    return -1;
}

bool duplicate_message_check(char* code_in, int ext_ID, int receiver_ID, int ttl, char* extra, char* buffer) {
  /*
  
  We first devide the messages into different queues based on the code
  We then check whether anothe message with the same structure exists
  
  Discard messages immediately when:
    - I'm the sender
    - ID not in my team_IDs list and it's not the external_team_connection_ID (we might want to reach a different team but not through this hop)
  
  */
  if (ext_ID == my_ID) { 
    /* message is from myself */
    printf("myself\n");
    return true;
  }
  if (receiver_ID != my_ID && receiver_ID != 0) { //&& atoi(ext_hop_ID) != external_team_connection_ID)
    /////////
    // To Do: Consider hop between teams
    /////////
    printf("not for me\n");
    return true;
  }
  
  switch(hash_codes(code_in)) // we could make this shorter but this way we reduce the computational time up to 4 times
  {
    case 0: //"DNB"
      printf("checking saved messages:\n");
      for (i=0; i<messages_lookback_size[0]; i++) {
        // first check the sender
        printf("checking saved number %i: %s\n", i, messages_lookback[0][i]);
        memset(tmp_s, 0, 34+1);
        strncpy(tmp_s, messages_lookback[0][i]+3, 3);
        printf("tmp_s: %s, atoi(tmp_s): %d, ext_ID: %d\n", tmp_s, atoi(tmp_s), ext_ID);
        if (atoi(tmp_s) == ext_ID) {
          printf("atoi(tmp_s): %d, ext_ID: %d\n", atoi(tmp_s), ext_ID);
          // second compare the receiver
          memset(tmp_s, 0, 34+1);
          strncpy(tmp_s, messages_lookback[0][i]+6, 3);
          printf("atoi(tmp_s): %d, receiver_ID: %d\n", atoi(tmp_s), receiver_ID);
          if (atoi(tmp_s) == receiver_ID) {
            // skip ttl but further check the remaining part of the message
            memset(tmp_s, 0, 34+1);
            strcpy(tmp_s, messages_lookback[0][i]+12);
            printf("tmp_s: %s, extra: %s\n", tmp_s, extra);
            if (strcmp(tmp_s, extra) == 0) {
              return true;
            }
          }
        }
      }
      /* if we have reached this point means it's not a duplicate, save it */
      strcpy(messages_lookback[0][messages_lookback_size[0]], buffer);
      messages_lookback_size[0] += 1;
      return false; 
    case 1: //"LJT"
      for (i=0; i<messages_lookback_size[1]; i++) {
        // first check the sender
        memset(tmp_s, 0, 34+1);
        strncpy(tmp_s, messages_lookback[1][i]+3, 3);
        if (atoi(tmp_s) == ext_ID) {
          // second compare the receiver
          memset(tmp_s, 0, 34+1);
          strncpy(tmp_s, messages_lookback[1][i]+6, 3);
          if (atoi(tmp_s) == receiver_ID) {
            // skip ttl but further check the remaining part of the message
            memset(tmp_s, 0, 34+1);
            strcpy(tmp_s, messages_lookback[1][i]+12);
            if (strcmp(tmp_s, extra) == 0) {
              return true;
            }
          }
        }
      }
      /* if we have reached this point means it's not a duplicate, save it */
      strcpy(messages_lookback[1][messages_lookback_size[1]], buffer);
      messages_lookback_size[1] += 1;
      return false;
    case 2: //"TTT"
      for (i=0; i<messages_lookback_size[2]; i++) {
        // first check the sender
        memset(tmp_s, 0, 34+1);
        strncpy(tmp_s, messages_lookback[2][i]+3, 3);
        if (atoi(tmp_s) == ext_ID) {
          // second compare the receiver
          memset(tmp_s, 0, 34+1);
          strncpy(tmp_s, messages_lookback[2][i]+6, 3);
          if (atoi(tmp_s) == receiver_ID) {
            // skip ttl but further check the remaining part of the message
            memset(tmp_s, 0, 34+1);
            strcpy(tmp_s, messages_lookback[2][i]+12);
            if (strcmp(tmp_s, extra) == 0) {
              return true;
            }
          }
        }
      }
      /* if we have reached this point means it's not a duplicate, save it */
      strcpy(messages_lookback[2][messages_lookback_size[2]], buffer);
      messages_lookback_size[2] += 1;
      return false;
    case 3: //"ITM"
      for (i=0; i<messages_lookback_size[3]; i++) {
        // first check the sender
        memset(tmp_s, 0, 34+1);
        strncpy(tmp_s, messages_lookback[3][i]+3, 3);
        if (atoi(tmp_s) == ext_ID) {
          // second compare the receiver
          memset(tmp_s, 0, 34+1);
          strncpy(tmp_s, messages_lookback[3][i]+6, 3);
          if (atoi(tmp_s) == receiver_ID) {
            // skip ttl but further check the remaining part of the message
            memset(tmp_s, 0, 34+1);
            strcpy(tmp_s, messages_lookback[3][i]+12);
            if (strcmp(tmp_s, extra) == 0) {
              return true;
            }
          }
        }
      }
      /* if we have reached this point means it's not a duplicate, save it */
      strcpy(messages_lookback[3][messages_lookback_size[3]], buffer);
      messages_lookback_size[3] += 1;
      return false;
    default:
      return false;
  }
}

void handle_message(char* buffer) {
  /*
  
  Fixed:
  
  SSS - code_in - message code as string
  SSS - ext_ID_s - external ID as string (sender)
  SSS - receiver_ID_s - ID of the receiver as string
  SSS - TTL - TTL
  
  */
  
  /* check if this message was already received */
  // To Do: if message already received discard it. Consider also case where the sender id is different but the message is the same
  
  
  
  
  
  
  
  
  ////////////////
  //
  //
  //
  // To Do: apart from DNB messages, all others should be discarded if the sending bot is not known
  //
  //
  //
  //
  /////////////////
  
  
  
  
  
  
  
  strncpy(code_in, buffer, 3);
  // printf("%d received %s\n", my_ID, code_in);
  strncpy(ext_ID_s, buffer+3, 3);
  ext_ID = atoi(ext_ID_s);
  strncpy(receiver_ID_s, buffer+6, 3);
  receiver_ID = atoi(receiver_ID_s);
  strncpy(ttl_s, buffer+9, 3);
  ttl = atoi(ttl_s);
  
  printf("%d received message %s\n", my_ID, buffer);
  
  duplicate_message = duplicate_message_check(code_in, ext_ID, receiver_ID, ttl, buffer+12, buffer);

  printf("%d in_queue %d\n", my_ID, duplicate_message);

  if (duplicate_message) {
    return;
  }
  
  if (ttl > 0) {
    strncpy(message, buffer, 9);
    sprintf(tmp_s, "%03d", ttl-1);
    strcat(message, tmp_s);
    strcat(message, buffer+12);
    wb_emitter_send(emitter_bt, message, strlen(message) + 1);
  }
  
  switch(hash_codes(code_in))
  {
    /* 
    
    Variable:
    
    S   - external_team_player - Y if external bot is in a team
    s   - external_team_size - size of the external team
    S   - ext_leader - Y if the external bot is the leader of its team
    SSS - ext_leader_ID_s - ID of the external team's leader
    
    */
    case 0: //"DNB" - Discover New Bot
      ext_team_player = buffer[12];
      ext_team_size = buffer[13];
      ext_leader = buffer[14];
      strncpy(ext_leader_ID_s, buffer+15, 3);
      // printf("%d ext_leader_ID_s %s\n", my_ID, ext_leader_ID_s);
      ext_leader_ID = atoi(ext_leader_ID_s);
      
      tmp = ext_team_size - '0';
      
      // printf("ext_team_size %s\n", &ext_team_size);
      if (tmp + team_size > 7) {
        // To Do:
        // reject_union_team();
        // To Do: add the bot to external queue
        break;
      }
      
      in_queue = false;
      for (i=0; i<idx_team; i++) {
        if(team_IDs[i] == ext_ID) {
          printf("%d bot %d already in queue %d\n", my_ID, ext_ID, team_IDs[i]);
          in_queue = true; // discard this communication
        }
      }
      
      if (!team_player) { // at this point for sure I'll either join or inglobate at least anothe robot
        team_player = true;
      }
      
      // printf("message %s\n", buffer);
      if (!in_queue) {
        // printf("leader_ID %d ext_leader_ID %d\n", leader_ID, ext_leader_ID);
        printf("%d bot not in queue %d\n", my_ID, ext_ID);
        /* New robot - process received information */
        // team_IDs[idx] = ext_ID;
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
      break;
    /*
    
    Variable:
    
    S - last_queued - Y if this is the last bot in queue of the external bot
    
    */ 
    case 1: //"LJT" - List of other team-members to Join my Team
      // printf("received LJT\n");
      // printf("message %s\n", buffer);
      last_queued = buffer[12-1];
      inglobate_external_team(ext_ID_s, last_queued);
      break;
    /*
    
    Variable:
    
    SSS - external_leader_ID - ID of the leader of the external team
    
    */
    case 2: //"TTT" - Transfer To the new Team
      // printf("received TTT\n");
      strncpy(tmp_ss, buffer+12, 3);
      if (strcmp(tmp_ss, leader_ID_s) != 0) { // not for my team
      ////////
      // To Do: change it so that the team_ID (leader_ID) is in every message
      ////////
        break;
      }
      strncpy(leader_ID_s, buffer+15, 3);
      leader_ID = atoi(leader_ID_s); // set up new leader
      leader = false;
      ///////////
      // To Do: check below - it should be automatic through the ITM message
      // check the transfer to team and ITM interactions
      // Should be solved
      ///////////
      team_IDs[idx_team] = leader_ID;
      idx_team += 1;
      //////////////
      // To Do: set up new location
      //////////////
      break;
    /*
    
    Variable:
    
    SSS - ext_ID_s - ID of the external bot we are inglobating
    
    */
    case 3: //"ITM" - Inform Team Member
      // in future we will work with codes - ITM should be a general message
      // printf("received ITM\n");
      strncpy(tmp_ss, buffer+12, 3);
      if (atoi(tmp_ss) != my_ID) {
        team_IDs[idx_team] = atoi(tmp_ss);
        idx_team += 1;
      }
    break; 
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
  leader_ID = my_ID;
  sprintf(leader_ID_s, "%03d", my_ID);
  team_player = false;
  leader = true;
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
  
  /* LEDs */
  for (i = 0; i < NB_LEDS; i++) {
    sprintf(name, "led%d", i);
    leds[i] = wb_robot_get_device(name); /* get a handler to the sensor */
  }
  leddl = wb_robot_get_device("ledd");
  wb_led_set(leddl, 1);
  
  /* Initialization and initial reset*/
  my_ID = atoi(wb_robot_get_name() + 5);
  sprintf(my_ID_s, "%03d", my_ID);
  
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
    
    printf("---\n");
    printf("%d new step\n", my_ID);
    printf("---\n");
    
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
    if (1) {
      // for (i=0; i<NB_LEDS; i++)
        // wb_led_set(leds[1], 1);
      construct_discovery_message(my_ID, team_player, leader, idx_team, leader_ID_s); // saves the desired string in variable message
      wb_emitter_send(emitter_bt, message, strlen(message) + 1);
    }
    if (team_player) {
      if (leader)
        wb_led_set(leddl, 2);
      else
        wb_led_set(leddl, 3);
    }
    
    // if (! have_leader) { 
      // /* DNB Discover New Bot - ID - No Part of Team - No Leader */
      // snprintf(message, 7, "DNB%dNN", my_ID); 
    // }
    // else {
    
    // }
    
    /* Check for new messages and process them */
    while (wb_receiver_get_queue_length(receiver_bt) > 0) {
      const char *buffer = wb_receiver_get_data(receiver_bt);
      handle_message((char*)buffer);
      wb_receiver_next_packet(receiver_bt);      
    }
    
    /* Move to location */
    ///////
    // To Do
    ///////
    
    // if (mtl_active) {
      // angle = angle_offset();
      
      // if (turn) { /* we are facing the right direction */
        // forward_move = false;
        // turn_towards_location();
      // }
      // else { /* turn until we face the right direction */
        // forward_move = true;
      // }
      
      // if (forward_move) {
        // distance = calculate_distance_location();
        // wb_motor_set_velocity(left_motor, 0.00628 * speed[LEFT]);
        // wb_motor_set_velocity(right_motor, 0.00628 * speed[RIGHT]);
      // }
      
      // /* Arrived at location */
      // if (mtl_arrived) {
        // mtl_active = false;
        // mtl_arrived = false;
      // }
    // }  
    
    /* Set wheel speeds */
    printf("%d idx_team %d\n", my_ID, idx_team);
    for (i=0; i<idx_team; i++)
      printf("%d id %d\n", i, team_IDs[i]);
    
    if (leader) {
      // printf("%d robot in %d leader %d\n", my_ID, team_IDs[0], leader);
      wb_motor_set_velocity(left_motor, 0.00628 * speed[LEFT]); // speed[LEFT]);
      wb_motor_set_velocity(right_motor, 0.00628 * speed[RIGHT]); // speed[RIGHT]);
    }
    else {
      // printf("%d robot in %d\n", my_ID, team_IDs[0]);
      wb_motor_set_velocity(left_motor, 0);
      wb_motor_set_velocity(right_motor, 0);
    }
  }

  fclose(fpt);

  return 0;
}