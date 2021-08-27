// To Do: 
// - Add a control so there cannot be two team inglobations/joins at the same time
//   e.g. robot_1 of team_3 meets robot_2 of team_1, robot_12 of team_3 meets robot_3 of team_2
//   in this case we'd have two robots trying to move team_3 towards team_1 and team_2
// - TTT when I join a new team, I receive the list of all the other bots? however, when I do the transfer my other team will discard all the messages containing the new bots
// - If there are obstacles, we try to avoid them. If it's not possible, we redirect the blocked robot somewhere else
//   Consider also narrow passages or stuff like that
// - Work with compass and RSSI (RSSI to understand the distance and compass for inter-team interactions) 
// - Use RSSI and number of collisions to expand in the best direction (lower aggregation)
// - EBR are sent at random by anyone receiving a ILM/LOC? wtf

// NUE
// Red - X - Nord y 
// Green - Y - Up z
// Blue - Z - East x


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
#include <webots/gps.h>
#include <webots/compass.h>
#include <string.h>

// Custom libraries
#include "OAM.h"
#include "MHM.h"
#include "ETM.h"

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
#define COMMUNICATION_CHANNEL_NFC 1
#define COMMUNICATION_CHANNEL_BT 2
#define FLOOR_SIZE 80.0 
// 3.0
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

// Position
WbDeviceTag gps, compass;

// Supervisor
WbNodeRef node;
WbFieldRef field;

/* General */
FILE *fpt;
char name[20], filename[20];
int ps_offset[NB_DIST_SENS] = {0, 0, 0, 0, 0, 0, 0, 0}, i, j, speed[2];
int rand_num, num, my_ID, counter = 0, run = 0;

/* Basic comms */
char code_in[4], ext_ID_s[4], my_ID_s[4], receiver_ID_s[4], leader_ID_s[4], ext_leader_ID_s[4];
char message[37+1], ttl_s[4], extra[22+1], tmp_s[37+1], tmp_s2[37+1], tmp_ss[4], tmp_ss2[4];
char ext_team_size_s[3+1], whitelisted_leader_ID_s[3+1];
char ext_team_player, ext_leader;
bool leader = false, team_player = false, in_queue = false, duplicate_message = false;
bool relayed = false, queue_full = false, whitelisted = false, whitelisting = false;
int leader_ID = 0, ext_leader_ID = 0, idx_comms = 0, idx_team = 0, ext_ID = 0, ext_team_size;
int receiver_ID = 0, tmp = 0, ttl = 0, confirmation = -1, whitelist_idx = 0, force_hop = 0;
int team_IDs[7] = {0}, ID_whitelist[6] = {-1};


/* Location */
char x_ref_s[7+1], y_ref_s[7+1], my_x_s[7+1], my_y_s[7+1], my_team_idx_s[2], team_angle_s[3+1]; 
float x_ref, y_ref, my_x, my_y, x_goal, y_goal, bearing, team_bearing, angle, angle_compass;
float diff_angle = -1.0, dist_to_goal = -1.0, backup = 0.0;
double x, y;
int my_team_idx = 0, turn_counter = 0, dist_counter = 0, arrived_folls = 0, team_angle = 0, printed = 0; // my role in the team
bool location_change = false, new_bot = false, waiting_new_bot = false, in_line = false, angle_update = false;

/* Stored messages - Could use an Hash Table */
char messages_lookback[11][SIZE][37+1];
int messages_lookback_size[11] = {0};

char messages_relayed[SIZE][37+1];
int messages_relayed_size = 0;

/* External connections */
// char ext_connection_ID_s[3+1];
// int ext_connection_ID, ext_connection_leader_ID;
float vals[2];
bool ext_connection_existing = false, global_leader = true, arrival_update = false;
int ext_teams_ready[6] = {0}, arrived_ext_part = 0, arrived_ext_full = 0;
int ete, etr, etrf, global_movement = 0, global_rotation = 0, arr_ext[6] = {0}, arr_ext_full[6] = {0};
int team_collision_count = 0, team_collision_time = 0, team_collision_ID[6] = {0}, team_collision_angle[6] = {0};
char hop_team_idx[3+1];
char hop_ID_s[3+1];
// extern struct LeaderExtConnection ext_connections[6];
// extern struct ExtConnection ext_connection;
conn ext_connection;
leader_conn ext_connections[6];

// int comms_queue[7];

/* Relay */
char ext_team_ID_s[4];
bool relay = false, to_relay = false;
int ext_team_ID;

/* Debug */

float min = 10000;
int count = 0;


//------------------------------------------------------------------------------
//
//    BEHAVIORAL MODULES
//
//------------------------------------------------------------------------------

void inform_new_location(char* ext_ID_s, char* TTL);
void handle_position_f(float x_ref, float y_ref, int my_team_idx, bool update);

////////////////////////////////////////////
// OAM - Obstacle Avoidance Module
////////////////////////////////////////////

int oam_active, oam_reset, oam_obst, obst_direction, oam_reset_counter = 0;
int oam_speed[2];
int oam_side = NO_SIDE, message_sent = FALSE;
float turn; 

// const int PS_OFFSET_SIMULATION[NB_DIST_SENS] = {300, 300, 300, 300, 300, 300, 300, 300};
const int PS_OFFSET_SIMULATION[NB_DIST_SENS] = {80, 80, 80, 80, 80, 80, 80, 80};
int ps_value[NB_DIST_SENS] = {0, 0, 0, 0, 0, 0, 0, 0};

void ObstacleAvoidanceModule (void) {
  obstacle_avoidance();
  if (oam_active && !global_rotation) {
    // printf("%d obstacleee\n", my_ID);
    if (team_player) {
      /* Inform my neighbors */
      double *values = (double* )wb_gps_get_values(gps);
      double x = values[2];
      // double z = values[1];
      double y = values[0];
      memset(tmp_s, 0, 37+1);
      sprintf(tmp_s, "001%07.3f%07.3f", x, y);
      construct_share_with_team_message("MOV", my_ID_s, "000", "000", "007", tmp_s);
      wb_emitter_send(emitter_bt, message, strlen(message) + 1);
      
      if (global_leader) {
        printf("%d global leader checking errors\n", my_ID);
        tmp = -1;
        for (i=0; i<team_collision_count; i++) {
          printf("%d ci sta %d\n", my_ID, team_collision_ID[i]);
          if (abs(team_collision_angle[i]-team_angle) < 2 && my_ID != team_collision_ID[i]) {
            /* Cancel this collision */
            team_collision_angle[i] = 0;
            team_collision_ID[i] = 0;
            printf("%d cancelling one error\n", my_ID);
            break;
          }
          else if (my_ID == team_collision_ID[i]) {
            printf("%d alerady saved myself\n", my_ID);
            break;
          }
          else if (team_collision_ID[i] == 0) {
            tmp = i;
            printf("%d saving empty\n", my_ID);
          }
        }
        if (i == team_collision_count) {
          if (tmp != -1) {
            /* Save in empty spot */
            i = tmp;
          }
          team_collision_count += 1;
          printf("%d count up to %d\n", my_ID, team_collision_count);
          team_collision_angle[i] = team_angle;
          team_collision_ID[i] = my_ID;
          if (team_collision_time == 0) {
            team_collision_time = 1;
            printf("%d setting up time\n", my_ID);
          }
          else {
            printf("%d adding error 1\n", my_ID);
          }
        }
        printf("%d exiting mov aaaa \n", my_ID);
      }
      // printf("%d sending obstacle error\n", my_ID);
    }
  }
}






// need to consider, if there are two messages from two bots near each other then probably they r touching each other

/* If global leader inform everyone, else act like a follower */
      // if (global_leader) {
        // /* Get the angle of the incident */
        // angle = atan2(my_y, my_x);
        // if (my_y < 0) {
          // angle = 360 + (angle * 180 / M_PI);
        // }
        // else {
          // angle = angle * 180 / M_PI;
        // }
        
        // /* Move facing a different direction */
        // memset(tmp_s, 0, 37+1);
        // sprintf(tmp_s, "001%s", tmp_ss);
        // construct_share_with_team_message("MOV", my_ID_s, "000", "000", "007", tmp_s);
        // wb_emitter_send(emitter_bt, message, strlen(message) + 1);
        // waiting_new_bot = false;
        // global_movement = 0;
      // }
      
      
      
      
      
      
////////////////////////////////////////////
// MHM - Message Handling Module
////////////////////////////////////////////

void append_bot(char* bot_ID) {
  /*
  
  Here we add the bot to our team queue
  We first check if the bot is already in the queue and if it's, we discard it
  
  */
  printf("%d the bot we are checking is %d\n", my_ID, atoi(bot_ID));
  in_queue = false;
  
  for (i=0; i<idx_team; i++) {
    if(team_IDs[i] == atoi(bot_ID) || atoi(bot_ID) == my_ID) {
      in_queue = true; /* discard this communication */
      printf("%d have %d already in queue\n", my_ID, atoi(bot_ID));
      printf("  because team_ID %d and my_ID %d\n", team_IDs[i] == atoi(bot_ID), atoi(bot_ID) == my_ID);
      break;
    }
  }
  
  queue_full = false;
  
  if (idx_team == 6 || in_queue) {
    printf("%d queue full\n", my_ID);
    queue_full = true;
    return;
  }
  
  if (!in_queue) {
    if (force_hop > 0) {
      team_IDs[force_hop-1] = atoi(bot_ID);
      printf("%d queue forcing hop\n", my_ID);
    }
    else {
      for (i=0; i<6; i++) {
        if (team_IDs[i] == 0) {
          team_IDs[i] = atoi(bot_ID);                    
          printf("%d storing %s\n", my_ID, bot_ID);
          break;
        }
      }
    }
    // if (my_ID == 1)
      // printf("%d appending %d\n", my_ID, atoi(bot_ID));
      // if (my_ID == 1) {
        // printf("%d appending %d\n", my_ID, atoi(bot_ID));
      // }
    // printf("%d new idx_team: %d\n", my_ID, idx_team + 1);
    
    idx_team += 1;
  }
}

void reset_team(void) {
  
  // printf("%d resetting leader and team\n", my_ID);
  leader_ID = my_ID;
  sprintf(leader_ID_s, "%03d", my_ID);
  team_player = false;
  leader = true;
  for (i=0; i<6; i++) {
    team_IDs[i] = 0;
  }
  idx_team = 0;
  // printf("%d idx_team %d\n", my_ID, idx_team);

}

void join_external_team(char* ext_ID_s, char* ext_leader_ID_s) {
  /* 
  
  Join the other team.
  Here we send a series of LJT messages to the other team with the info about all our current teammates
  At the same time we send a TTT message to the current teammates informing them of the transfer
  
  if not the leader - update all the slaves of the change
  if the leader - update the status to slave 
  
  */
  
  /* message structure: 
    
    SSS - code_in - code of the message
    SSS - my_ID_s - ID of the sending bot
    SSS - ext_ID_s - ID of the receiving robot (for comms)
    SSS - TTL - TTL flag
    S - last_queued - Y if this is the last bot in queue
    
  */
  
  /* update my stats - we are for sure not the leader */
  // printf("%d joining ext team\n", my_ID);
  if (leader) {
    // printf("%d cancel relay 1\n", my_ID);
    relay = false;
    
    for (i=0; i<6; i++) {
      if (ext_connections[i].size > 0) {
        memset(ext_connections[i].ext_ID_s, 0, 3+1);
        memset(ext_connections[i].ext_leader_ID_s, 0, 3+1);
        ext_connections[i].size = 0;
        // To Do: then transfer this knowledge to the other team
      }
    }
    
    memset(ext_connection.ext_ID_s, 0, 3+1);
    memset(ext_connection.ext_leader_ID_s, 0, 3+1);
    ext_connection.size = 0;
  }
  // printf(" -- --- --- --- -- 333\n");
  leader = false;
  global_leader = false;
  
  /* save new leader's info */
  strcpy(leader_ID_s, ext_leader_ID_s);
  leader_ID = atoi(ext_leader_ID_s);
  
  /* store new bot */
  append_bot(leader_ID_s);
  // append_bot(ext_ID_s);
  
  /* we are not the leader, so no reason to wait for others */
  waiting_new_bot = false;
  
  for (i=0; i<idx_team-1; i++) { /* all but the newly saved bot, i.e. leader */
    
    /* store the ID of the bot we are currently transferring in tmp_ss */
    snprintf(tmp_ss, 4, "%03d", team_IDs[i]);
    
    /* send the message out with sender = the bot we are transferring */
    
    ////////
    //
    // To Do: change it so that the sender is me and instead of the useless Y/N we add the id of the bot we are transferring
    //
    ////////
    
    if(i == idx_team-1) {
      construct_join_team_message(my_ID_s, ext_ID_s, leader_ID_s, "000", tmp_ss);
    }
    else {
      construct_join_team_message(my_ID_s, ext_ID_s, leader_ID_s, "000", tmp_ss);
    }
    wb_emitter_send(emitter_bt, message, strlen(message) + 1);
    
    /* send message to the bot we are transferring */
    construct_transfer_team_message(my_ID_s, tmp_ss, leader_ID_s, "003", idx_team);
    wb_emitter_send(emitter_bt, message, strlen(message) + 1);
    
    /* Location for the bot is updated in inglobate_external_team */
    
  }
  
  
  
  
  // To Do: We should reset the team_collision thing because we expect to move to another direction
}

void inglobate_external_team(char* ext_ID_s, char* TTL) {
  /*
  
  We are sharing the info about the newly added robot with our team
  If we are the leader we should share the message with all our team
  If we are not the leader we should share the info with our leader
  
  */
  printf("%d inglobating ext team %s\n", my_ID, ext_ID_s);
  if (leader) {
    /* TTL set to 0 as it should reach every other bot in the team in a single hop */
    construct_share_with_team_message("ITM", my_ID_s, "000", leader_ID_s, "000", ext_ID_s);     
  }
  else {
    /* TTL set to 3 as it should be enough to travel the whole team */
    construct_share_with_team_message("ITM", my_ID_s, "000", leader_ID_s, "003", ext_ID_s);
  }
  printf("%d informing team of inglobation %s\n", my_ID, message);
  wb_emitter_send(emitter_bt, message, strlen(message) + 1);
  
  /* Share the info about our team */
  memset(tmp_ss, 0, 3+1);
  sprintf(tmp_ss, "%03d", my_ID); 
  construct_share_with_team_message("ITM", my_ID_s, "000", leader_ID_s, "007", tmp_ss);
  wb_emitter_send(emitter_bt, message, strlen(message) + 1);
  
  for (i=0; i<6; i++) {
    if (team_IDs[i] != 0) {
      memset(tmp_ss, 0, 3+1);
      sprintf(tmp_ss, "%03d", team_IDs[i]); 
      construct_share_with_team_message("ITM", my_ID_s, "000", leader_ID_s, "007", tmp_ss);
      wb_emitter_send(emitter_bt, message, strlen(message) + 1);
    }
  }
         
  /* Save data of the new bot */
  printf("%d appending bot %d\n", my_ID, atoi(ext_ID_s));
  append_bot(ext_ID_s);
  printf("%d after appendition\n", my_ID);
  /* 
  
  If I'm the leader I can inform the bot of it's location directly
  Otherwise, I simply skip this step because the share_with_team message will eventually
  reach my leader who will in turn share the location to the bot (and we will probably relay it)
  
  */  
  if (leader && !queue_full) {
    printf("%d inglobating %s with queue_full %d\n", my_ID, ext_ID_s, queue_full);
    inform_new_location(ext_ID_s, TTL);
    waiting_new_bot = true;
    printed = 0;
    if (global_movement && leader) {
      construct_share_with_team_message("MOV", my_ID_s, "000", "000", "007", "003");
      wb_emitter_send(emitter_bt, message, strlen(message) + 1);
      global_movement = 0;
    }
  }
  // else {
    // printf("%d cannot tell new location cause 
  // }
  
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
  else if (strcmp(code_in, "ILM") == 0)
    return 4;
  else if (strcmp(code_in, "EBR") == 0)
    return 5;
  else if (strcmp(code_in, "LOC") == 0)
    return 6;
  else if (strcmp(code_in, "ARR") == 0)
    return 7;
  else if (strcmp(code_in, "UPD") == 0)
    return 8;
  else if (strcmp(code_in, "MOV") == 0)
    return 9;
  else if (strcmp(code_in, "IBA") == 0)
    return 10;
  else
    return -1;
}

bool duplicate_message_check(char* code_in, int sender, int receiver, int ext_team_ID, int ttl, char* extra, char* buffer) {
  /*
  
  We first devide the messages into different queues based on the code
  We then check whether anothe message with the same structure exists
  
  Discard messages immediately when:
    - I'm the sender
    - ID not in my team_IDs list and it's not the external_team_connection_ID (we might want to reach a different team but not through this hop)
  
  */
  // if (strcmp(code_in, "LOC") == 0 && my_ID == 3) {
    // printf("my_ID %d sender %d\n", my_ID, sender);
  // }
  if (sender == my_ID) { 
    /* message is from myself */
    // if (strcmp(code_in, "LOC") == 0 && my_ID == 3) {
      // printf("  at point 1\n");
    // }
    // if (my_ID == 2) {
      // printf("%d dupplaaa %s\n", my_ID, buffer);
    // }
    return true;
  }
         
  /* 
  
  A few possible cases:
  - receiver != my_ID --- I am not the receiver
  - receiver != 0 --- The message is not for broadcast
  - receiver != atoi(ext_connection.ext_ID_s) --- The message is not for the bot I'm relaying to
  
  A few special cases:
  - ILM or ITM or TTT
    - ext_team_ID != leader_ID --- Receiver is not in my team
  
  */
  
  /* message not for me nor broadcast - no relay node */
  if (receiver != my_ID && receiver != 0 && receiver != atoi(ext_connection.ext_ID_s) && strcmp(code_in, "LOC") != 0)  { 
  // if (strcmp(code_in, "ILM") != 0 && (ext_team_ID != leader_ID && receiver != atoi(ext_connection.ext_ID_s)))  {    
    /////////
    // To Do: Consider hop between teams
    /////////
    // printf("%d %s duplicate receiver %d hop %d\n", my_ID, buffer, receiver, atoi(ext_connection.ext_ID_s));
    // if (strcmp(code_in, "LOC") == 0 && my_ID == 3) {
      // printf("  at point 2 with receiver %d\n", receiver);
    // }
    if (strcmp(code_in, "UPD") == 0 ) {
      printf("--- %d che cazzzzzz ---- -- -- - - -- - - - -- - \n", my_ID);
    }
    // if (my_ID == 8) {
      // printf("%d duppl %s\n", my_ID, buffer);
      // printf("  ext conn: %d\n", atoi(ext_connection.ext_ID_s));
    // }
    return true;
  }
  
  switch(hash_codes(code_in)) // we could make this shorter but this way we reduce the computational time up to 4 times
  {
    case 0: //"DNB"
      // if (sender == atoi(whitelisted_leader_ID_s)) {
        // printf("%d DNB da whitelisted %d\n", my_ID, sender);
        
        // return true;
      // }
      // if (sender == 9) {
        
      // }
      
      for (i=0; i<messages_lookback_size[0]; i++) {
        memset(tmp_s, 0, 37+1);
        strncpy(tmp_s, messages_lookback[0][i]+3, 3); /* check the sender */
        if (atoi(tmp_s) == sender) {
          if (sender == atoi(whitelisted_leader_ID_s)) {
            whitelisted = true;
            memset(whitelisted_leader_ID_s, 0, 3);
          }
          else {
            for (j=0; j<whitelist_idx; j++) {
              if (sender == ID_whitelist[j]) {
                whitelisted = true;
                ID_whitelist[j] = -1;
                strcpy(messages_lookback[0][messages_lookback_size[0]], buffer);
                messages_lookback_size[0] += 1;
                // printf("%d throwing at 2\n", my_ID);
                return true;
              }
            }
          }
          memset(tmp_s, 0, 37+1);
          strncpy(tmp_s, messages_lookback[0][i]+9, 3); /* check the team_ID */
          if (atoi(tmp_s) == ext_team_ID || ext_team_ID == leader_ID) {
            // printf("%d throwing at 3\n", my_ID);
            return true;
          }
        }
      }
      /* if we have reached this point means it's not a duplicate, save it */
      strcpy(messages_lookback[0][messages_lookback_size[0]], buffer);
      messages_lookback_size[0] += 1;
      return false; 
    case 1: //"LJT"
      for (i=0; i<messages_lookback_size[1]; i++) {
        memset(tmp_s, 0, 37+1);
        strncpy(tmp_s, messages_lookback[1][i]+9, 3); /* check the team_ID */
        if (atoi(tmp_s) == ext_team_ID) {
          return true;
        } 
      }
      /* if we have reached this point means it's not a duplicate, save it */
      strcpy(messages_lookback[1][messages_lookback_size[1]], buffer);
      messages_lookback_size[1] += 1;
      return false;
    case 2: //"TTT"
      for (i=0; i<messages_lookback_size[2]; i++) {
        memset(tmp_s, 0, 37+1);
        strncpy(tmp_s, messages_lookback[2][i]+3, 3); /* check the sender */
        if (atoi(tmp_s) == sender) {
          memset(tmp_s, 0, 37+1);
          strncpy(tmp_s, messages_lookback[2][i]+6, 3); /* check the receiver */
          if (atoi(tmp_s) == receiver) {
            memset(tmp_s, 0, 37+1);
            strcpy(tmp_s, messages_lookback[2][i]+15); /* check the rest */
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
      /*
      
      Here we only check for the team_ID and the transferred bot
      This is done because if we have already shared a similar message
        regarding the inglobation of a bot within the same team, it makes no sense 
        to share the same information again. In case a new bot joins, this task
        is handled by the LJT and TTT messages
        
      */
      if (ext_team_ID != leader_ID) {
        return true;
      }
      
      for (i=0; i<messages_lookback_size[3]; i++) {
        memset(tmp_ss, 0, 3+1);
        // strncpy(tmp_s, messages_lookback[3][i]+3, 3); /* check the sender */
        // if (atoi(tmp_s) == sender) {
          // memset(tmp_s, 0, 37+1);
          strncpy(tmp_ss, messages_lookback[3][i]+9, 3); /* check the team_ID */
          // printf("team_ID %d and %d\n", atoi(tmp_ss), ext_team_ID);
          if (atoi(tmp_ss) == ext_team_ID) {
            memset(tmp_ss, 0, 3+1);
            strncpy(tmp_ss, messages_lookback[3][i]+15, 3); /* check the rest */
            // printf("bot %s and %s\n", tmp_ss, extra);
            if (strcmp(tmp_ss, extra) == 0) {
              printf("%d ITM duplicate %s of %s\n", my_ID, buffer, messages_lookback[3][i]);
              return true;
            }
          }
        // }
      }
      /* if we have reached this point means it's not a duplicate, save it */
      strcpy(messages_lookback[3][messages_lookback_size[3]], buffer);
      messages_lookback_size[3] += 1;
      return false;
    case 4: // "ILM"
      // if (my_ID == 9 || my_ID == 8) {
        // printf("Bot 9 received %s\n", buffer);
        // printf("  sender is %d and the whitelisted bot is %d\n", sender, atoi(whitelisted_leader_ID_s));
      // }
      if (sender != leader_ID && sender != atoi(whitelisted_leader_ID_s)) {
        /*
        
        We do not listen to others but our leader
        This prevents bots from updating their location based on old messages 
        from previous leaders (after team union) and which were stuck in the relay nodes
        
        */
        // printf("  at step 1\n");
        return true;
      }
      
      for (i=0; i<messages_lookback_size[4]; i++) {
        memset(tmp_ss, 0, 3+1);
        strncpy(tmp_ss, messages_lookback[4][i]+3, 3); /* check the sender */
        if (atoi(tmp_ss) == sender) {
          memset(tmp_ss, 0, 3+1);
          strncpy(tmp_ss, messages_lookback[4][i]+6, 3); /* check the receiver */
          if (atoi(tmp_ss) == receiver) {
            memset(tmp_s, 0, 37+1);
            strcpy(tmp_s, messages_lookback[4][i]+15); /* check the sender */
            if (strcmp(tmp_s, extra) == 0) {
              return true;
            }
          }
        }
      }
      /* if we have reached this point means it's not a duplicate, save it */
      strcpy(messages_lookback[4][messages_lookback_size[4]], buffer);
      messages_lookback_size[4] += 1;
      return false;
    case 5: // EBR
    ////////
    //
    // To Do: complete check
    //
    ////////
      // printf("%d supl check received EBR: %s\n", my_ID, buffer);
      // printf("%d EBR check:\n", my_ID);
      // printf("  receiver: %d, ext_comm: %d\n", receiver, atoi(ext_connection.ext_ID_s));
      // printf("  bool: %d\n", (receiver != my_ID && receiver != 0 && receiver != atoi(ext_connection.ext_ID_s) && strcmp(code_in, "LOC") != 0));
   
      for (i=0; i<messages_lookback_size[5]; i++) {
        memset(tmp_ss, 0, 3+1);
        strncpy(tmp_ss, messages_lookback[5][i]+18, 3); /* check the actual sender (+3 would have been the hop) */
        char actual_sender[3+1];
        strncpy(actual_sender, buffer+18, 3);
        // printf("%d checking tmp: %s actual sender: %s\n", my_ID, tmp_ss, actual_sender);
        if (atoi(tmp_ss) == atoi(actual_sender)) {
          memset(tmp_ss, 0, 3+1);
          strncpy(tmp_ss, messages_lookback[5][i]+3, 3);
          if (atoi(tmp_ss) == ext_ID) {
          ///////
          //
          // To Do: Inform the sender that the bot is already known so he doesn't need to be a hop for that bot
          //
          ///////
          // printf("problem 1\n");
            return true;
          }
        }
        memset(tmp_ss, 0, 3+1);
        strncpy(tmp_ss, buffer+21, 3);
        ext_team_ID = atoi(tmp_ss);
        memset(tmp_ss, 0, 3+1);
        strncpy(tmp_ss, messages_lookback[5][i]+21, 3);
        /* check the team_ID of the bot - it might happen that we meet two bots from the same team
        when they r still close to each other, in this case we meet all of them and they all have the same leader ID
        if we wait a few timesteps they'll receive a TTT message, but for the moment we flag the message for discard */
        if (ext_team_ID == atoi(tmp_ss)) {
          // printf("problem 2 with %s\n", messages_lookback[5][i]);
          // printf("%d checking tmp: %s (atoi %d) ext team id: %d\n", my_ID, tmp_ss, atoi(tmp_ss), ext_team_ID);
        
          return true;
        }
        
      }
      // /* if we have reached this point means it's not a duplicate, save it */
      strcpy(messages_lookback[5][messages_lookback_size[5]], buffer);
      messages_lookback_size[5] += 1;
      return false;
    case 6: // LOC
      memset(tmp_ss, 0, 3+1);
      strncpy(tmp_ss, buffer+15, 3);
      if (receiver != my_ID && atoi(tmp_ss) != my_ID) {
        return true;
      }
      // printf("%d checking LOC for relay: tmp_ss %s my_ID %d\n", my_ID, tmp_ss, my_ID);
      to_relay = false;
      if (atoi(tmp_ss) == my_ID) {
        to_relay = true;
      }
      for (i=0; i<messages_lookback_size[6]; i++) {
        // printf("%d comparing:\n", my_ID);
        // printf("  %s\n", messages_lookback[6][i]);
        // printf("  %s\n", buffer);
        memset(tmp_ss, 0, 3+1);
        strncpy(tmp_ss, messages_lookback[6][i]+3, 3); /* check the sender */
        // printf("  step 1 %d %d\n", atoi(tmp_ss), ext_ID);
        if (atoi(tmp_ss) == ext_ID) {
          // printf("  step 1 passed\n");
          memset(tmp_s, 0, 37+1);
          strcpy(tmp_s, messages_lookback[6][i]+15);
          // printf("  step 2 %s %s\n", tmp_s, extra);
          if (strcmp(tmp_s, extra) == 0) {
            // printf("  duplicate\n");
            return true;
          }
        }
      }
      strcpy(messages_lookback[6][messages_lookback_size[6]], buffer);
      messages_lookback_size[6] += 1;
      // printf("  not duplicate\n");
      return false;    
      // if (receiver == my_ID) {
        // if (ext_ID == 
        // return false;
      // }
      // return true;
    case 7: // ARR
      // printf("%d checking ARR\n", my_ID);
      if (receiver != my_ID && receiver != leader_ID && receiver != 0) {
        // printf("%d drop ARR step 1\n", my_ID);
        return true;
      }
      // memset(tmp_ss, 0, 3+1);
      // strncpy(tmp_ss, buffer+3, 3);
      for (i=0; i<messages_lookback_size[7]; i++) {
        memset(tmp_ss, 0, 3+1);
        strncpy(tmp_ss, messages_lookback[7][i]+3, 3); 
        if (atoi(tmp_ss) == ext_ID) {
          memset(tmp_s, 0, 37+1);
          strcpy(tmp_s, messages_lookback[7][i]+15);
          // printf("  checking, for %d, extra %s and tmp %s\n", ext_ID, extra, tmp_s);
          if (strcmp(tmp_s, extra) == 0) {
            // printf("%d stop ARR step 2\n", my_ID);
            // printf("  from %s\n", my_ID);
            return true;
          }
        }
      }
      strcpy(messages_lookback[7][messages_lookback_size[7]], buffer);
      messages_lookback_size[7] += 1;
      return false;  
    case 8: // UPD
      for (i=0; i<messages_lookback_size[8]; i++) {
        memset(tmp_ss, 0, 3+1);
        strncpy(tmp_ss, messages_lookback[8][i]+3, 3); /* check the sender */
        if (atoi(tmp_ss) == ext_ID) {
          memset(tmp_s, 0, 37+1);
          strcpy(tmp_s, messages_lookback[8][i]+15);
          if (strcmp(tmp_s, extra) == 0) {
            return true;
          } 
        }
      }
      strcpy(messages_lookback[8][messages_lookback_size[8]], buffer);
      messages_lookback_size[8] += 1;
      return false;
    case 9: // MOV
      for (i=0; i<messages_lookback_size[9]; i++) {
        memset(tmp_ss, 0, 3+1);
        strncpy(tmp_ss, messages_lookback[9][i]+3, 3); /* check the sender */
        if (atoi(tmp_ss) == ext_ID) {
          memset(tmp_ss, 0, 3+1);
          strncpy(tmp_ss, messages_lookback[9][i]+15, 3);
          memset(tmp_ss2, 0, 3+1);
          strncpy(tmp_ss2, buffer+15, 3);
          if (strcmp(tmp_ss, tmp_ss2) == 0) {
            memset(tmp_s, 0, 37+1);
            strncpy(tmp_s, messages_lookback[9][i]+18, 5);
            memset(tmp_s2, 0, 37+1);
            strncpy(tmp_s2, buffer+18, 5);
            if (strcmp(tmp_ss, tmp_ss2) == 0) {
              memset(tmp_s, 0, 37+1);
              strncpy(tmp_s, messages_lookback[9][i]+25, 5);
              memset(tmp_s2, 0, 37+1);
              strncpy(tmp_s2, buffer+25, 5);
              if (strcmp(tmp_ss, tmp_ss2) == 0) {
                return true;
              }
            }
          } 
        }
      }
      strcpy(messages_lookback[9][messages_lookback_size[9]], buffer);
      messages_lookback_size[9] += 1;
      printf("%d MOV size backlog %d\n", my_ID, messages_lookback_size[9]);
      return false;
    case 10: // IBA
      for (i=0; i<messages_lookback_size[10]; i++) {
        memset(tmp_ss, 0, 3+1);
        strncpy(tmp_ss, messages_lookback[10][i]+3, 3); /* check the sender */
        if (atoi(tmp_ss) == ext_ID) {
          memset(tmp_ss, 0, 3+1);
          strncpy(tmp_ss, messages_lookback[10][i]+15, 3);
          memset(tmp_ss2, 0, 3+1);
          strncpy(tmp_ss2, buffer+15, 3);
          if (atoi(tmp_ss) == atoi(tmp_ss2)) {
            return true; 
          }
        }
      }
      strcpy(messages_lookback[10][messages_lookback_size[10]], buffer);
      messages_lookback_size[10] += 1;
      return false;
    default:
      // printf("%d default %s\n", my_ID, buffer);
      return false;
  }
}

void check_team_arrival(void) {
  // printf("%d leader idx_team %d\n", my_ID, idx_team);
  printf("  ext_full %d idx_team %d\n", arrived_ext_full, idx_team);
  printf("  ext_part %d idx_team %d\n", arrived_ext_part, idx_team);
         
  if (arrived_ext_full == idx_team) {
    /* this means that evvery external team is completely ready */
           
    if (global_leader) {
      /* send out signal to move */
      printf("%d fuck ittttt\n", my_ID);
      construct_share_with_team_message("MOV", my_ID_s, "000", "000", "007", "000");
      wb_emitter_send(emitter_bt, message, strlen(message) + 1);
      waiting_new_bot = false;
      global_movement = 1;
    }
    else {
      for (i=0; i<6; i++) {
        if (ext_connections[i].size > 0) {
          sprintf(tmp_s, "000111");
          construct_share_with_team_message("ARR", my_ID_s, "000", leader_ID_s, "002", tmp_s);
          wb_emitter_send(emitter_bt, message, strlen(message) + 1);
        }
      }
    }
  }
  else if (arrived_ext_part == idx_team) {
    printf("  sending part\n");
    for (i=0; i<6; i++) {
      if (ext_connections[i].size > 0) {
        printf("  sending part ready\n");
        sprintf(tmp_s, "000111"); // maybe 000110
        construct_share_with_team_message("ARR", my_ID_s, "000", leader_ID_s, "002", tmp_s);
        wb_emitter_send(emitter_bt, message, strlen(message) + 1);
      }
    }
  }
  else { // we are partially ready
    for (i=0; i<6; i++) {
      if (ext_connections[i].size > 0) {
        sprintf(tmp_s, "000110"); // maybe 000110
        construct_share_with_team_message("ARR", my_ID_s, "000", leader_ID_s, "002", tmp_s);
        printf("%d sending ARR outside team cause we are partially ready\n", my_ID);
        wb_emitter_send(emitter_bt, message, strlen(message) + 1);
      }
    }
  }
}

void handle_excess_bot(char* hop_ID_s, char* hop_team_idx, char* ext_ID_s, char* ext_leader_ID_s, int ext_team_size) {
  /*
  
  Only the leader will end up here
  
  We are on the side of the managing team which will redirect the other team somewhere
  We first need to check whether on the side from where the new team is incoming there is already a team
  -  If there is already a team, we retrieve the size and check whether the two can unite
  -  If there is no team already we check for the best option available
  
  */
  
  /* Already in queue */
  for (i=0; i<6; i++) {
    if (team_IDs[i] == atoi(ext_ID_s) || atoi(ext_connections[i].ext_ID_s) == atoi(ext_ID_s)) { 
      printf("- EBR overflow\n");
      printf("  team_IDs[i]: %d, atoi(ext_ID_s): %d\n", team_IDs[i], atoi(ext_ID_s));
      printf("  atoi(ext_connections[i].ext_ID_s): %d, atoi(ext_ID_s): %d\n", atoi(ext_connections[i].ext_ID_s), atoi(ext_ID_s));
      return;
    }
  }
  
  
  // while it's moving we should discard it's comms
  
  int out;
  
  ext_connection_existing = false;
  
  printf("%d handling ext bot %s\n", my_ID, ext_ID_s);
  
  if (ext_connections[atoi(hop_team_idx)].size > 0 && ext_connections[atoi(hop_team_idx)].size <= 7) {
    printf("%d step 1 with idx %d\n", my_ID, atoi(hop_team_idx));
    // printf("  size of ext team in %d is: %d\n", atoi(hop_team_idx), ext_connections[atoi(hop_team_idx)].size);
    ext_connection_existing = true; /* we have already a team near that position*/
  }
  
  if (ext_connection_existing) {
  
  
  
  
  
  
  
  
  
  // we inform the leader about the new bot coming in : )
  
  
  
  
  
  
  
    printf("%d step 2\n", my_ID);
    /* Assume that at each new join we update the struct for that hop */
    if (ext_connections[atoi(hop_team_idx)].size + ext_team_size < 7) {
      printf("%d step 3\n", my_ID);
      /*
      We can calculate the position of the known team
      It's basically as dealing with a new bot where I am the leader and it has to go 
      to the position of index opposite to mine
      
        known team           
       ||      incoming bot
       ||       || 
       V        V
      *   *     *   *   *
          
        *    *   *    *    *
    
      *   *         *   *
     
     
       
      it's goal location is to the left of the discovering (right team) bot (whose idx is 5) towards 
      the rightmost bot of the other team (left team) (whose idx is 2)
    
      1 -> 4
      2 -> 5
      3 -> 6
      4 -> 1 
      5 -> 2 
      6 -> 3
    
      initial value -> +2 -> % 6 -> + 1 = goal value
      1 -> 3 -> 3 -> 4
      2 -> 4 -> 4 -> 5
      3 -> 5 -> 5 -> 6
      4 -> 6 -> 0 -> 1
      5 -> 7 -> 1 -> 2
      6 -> 8 -> 2 -> 3
    
      */
    
      out = atoi(hop_team_idx) + 1;// (((atoi(hop_team_idx) + 2) % 6) + 1) + 1; // final + 1 is because idx starts from 0 while the out value is from 1
      // handle_position_f(my_x, my_y, out, false);
    }
    else {
      printf("%d step 4\n", my_ID);
      /* if the shortest path is not possible, we calculate the best route */
    
    ///////
    //
    // To Do: what if we redirect them there anyway and the exceding bots r redirected somewhere else
    //
    ///////
    
      out = retrieve_closest_external_connection(atoi(hop_team_idx), ext_team_size);
      if (out == -1) { // what if there are no otehr options available? tell the bot to ask the other team
        out = atoi(hop_team_idx) + 1; // idx starts from 0 while out starts from 1
        printf("%d step 5n", my_ID);
      }
      // handle_position_f(x_ref, y_ref, out, false);
    }  
  }
  else {
  /* I have the ext_team_size so I can guess the needed location
  However, we can directly send the location of the leader and let the ext team handle it */ 
    
    printf("%d step 6\n", my_ID);
    out = retrieve_closest_external_connection(atoi(hop_team_idx), ext_team_size);
    if (out == -1) {
      printf("%d step 7\n", my_ID);
      out = ((atoi(hop_team_idx) + 2) % 6) + 1 + 1; // final + 1 is because idx starts from 0 while the out value is from 1
    }
    // vals[0] = x_ref;
    // printf("%d testtttt x_goal %f y_goal %f\n", my_ID, x_ref, y_ref);
    // vals[1] = y_ref;
    // for (i=0; i<3; i++) { 
      // handle_position_f(vals[0], vals[1], out, false);
    // }
    // printf("%d vals are: %f %f\n", my_ID, vals[0], vals[1]);
    // handle_position_f(vals[0], vals[1], out, false);
    // printf("%d vals are: %f %f\n", my_ID, vals[0], vals[1]);
  }
  
  vals[0] = x_ref;
  vals[1] = y_ref;
  for (i=0; i<3; i++) { 
    handle_position_f(vals[0], vals[1], out, false);
  }
  
  printf("  out: %d\n", out);
    
  if (ext_connections[(out - 1) % 6].size > 0) {
    /* There is already a team so we inform that team that they should whitelist this bot */
    printf("%d step 8\n", my_ID);
    // ext_connections[out].size += ext_team_size;
    
    // construct_share_with_team_message("UPD", my_ID_s, hop_ID_s, my_ID_s, "000", "001");
    // printf("%d sending UPD %s\n", my_ID, message); 
    // wb_emitter_send(emitter_bt, message, strlen(message) + 1);
    // if (ext_connections[(out - 1) % 6].size == 1) {
      // /* In this case we move can check who will be the leader and decide, based on that, if we need to update the ext_hop_ID or no */
      // if (
    // }
    
    // sprintf(hop_ID_s, "%03d", team_IDs[(out - 1) % 6]);
    // printf("%d updating hop for idx %d to %s\n", my_ID, out-1, hop_ID_s);
    memset(tmp_s, 0, 37+1);
    sprintf(tmp_s, "%s%03d%1d%1d0", ext_ID_s, team_IDs[(out - 1) % 6], idx_team + 1, out);
    construct_share_with_team_message("IBA", my_ID_s, ext_connections[(out - 1) % 6].ext_leader_ID_s, my_ID_s, "002", tmp_s); // to leader
    wb_emitter_send(emitter_bt, message, strlen(message) + 1);
    memset(hop_ID_s, 0, 4);
    sprintf(hop_ID_s, "%03d", team_IDs[(out - 1) % 6]);
    // printf("%d sending IBA %s\n", my_ID, message);  
    memset(tmp_s, 0, 37+1);
    sprintf(tmp_s, "%s%03d%1d%1d1", ext_connections[(out - 1) % 6].ext_leader_ID_s, team_IDs[(out - 1) % 6], idx_team + 1, out);
    construct_share_with_team_message("IBA", my_ID_s, ext_ID_s, my_ID_s, "002", tmp_s); // to follower
    wb_emitter_send(emitter_bt, message, strlen(message) + 1);
    // printf("%d sending IBA %s\n", my_ID, message);  
    // printf("  adding 1 so ext size for %d is %d\n", (out - 1) % 6, ext_connections[(out - 1) % 6].size + 1);
    ext_connections[(out - 1) % 6].size += 1;
  }
  else {
    // printf("%d leader storing ext conn %s\n", my_ID, ext_ID_s);
    printf("%d step 9\n", my_ID);
    // printf("  with size %d\n", ext_team_size);
    memset(hop_ID_s, 0, 4);
    sprintf(hop_ID_s, "%03d", team_IDs[(out - 1) % 6]);
    leader_store_external_connection(out, ext_ID_s, ext_leader_ID_s, ext_team_size);
  }
  
  printf("%d step 10\n", my_ID);
  printf("  size of ext team in %d is: %d\n", out, ext_connections[(out-1)%6].size);
    
  memset(tmp_s, 0, 37+1); 
  
  // hop_team_idx is prolly wrong
  // sprintf(hop_ID_s, "%03d", team_IDs[out-1]);
  
  /* from EBR I retrieve the ext leader ID */
  // sprintf(tmp_s, "%s%03d%07.3f%07.3f%1d", hop_ID_s, my_ID, vals[0], vals[1], out);
  sprintf(tmp_s, "%s%s%07.3f%07.3f%1d", hop_ID_s, ext_leader_ID_s, vals[0], vals[1], out);
  
  // memset(tmp_ss, 0, 3+1);
  // sprintf(tmp_ss, "%1d", out);
  // strcat(tmp_s, tmp_ss);
  
  // We send a LOC message from leader to excess bot with the ID of the hop as well as the location and the 
  // memset(tmp_ss, 0, 3+1);
  // sprintf(tmp_ss, "%03d", team_angle);
  construct_share_with_team_message("LOC", my_ID_s, ext_ID_s, team_angle_s, "001", tmp_s);  // vals[0], vals[1], team_bearing);
  wb_emitter_send(emitter_bt, message, strlen(message) + 1);
  // printf("%d sending LOC to %d\n", my_ID, atoi(ext_ID_s));
  // printf("  the message is: %s\n", message);
  // printf("  hop_ID_s %s hop_team_idx %s team_IDs[hop_team_idx] %d\n", hop_ID_s, hop_team_idx, team_IDs[atoi(hop_team_idx)-1]);
}

void handle_message(char* buffer) {
  /*
  
  Fixed:
  
  SSS - code_in - message code as string
  SSS - ext_ID_s - external ID as string (sender)
  SSS - receiver_ID_s - ID of the receiver as string
  SSS - ext_team_ID_s - ID of the team (= ID of the leader)
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
  strncpy(ext_ID_s, buffer+3, 3);
  ext_ID = atoi(ext_ID_s);
  strncpy(receiver_ID_s, buffer+6, 3);
  receiver_ID = atoi(receiver_ID_s);
  strncpy(ext_team_ID_s, buffer+9, 3);
  ext_team_ID = atoi(ext_team_ID_s);
  strncpy(ttl_s, buffer+12, 3);
  ttl = atoi(ttl_s);
    
  duplicate_message = duplicate_message_check(code_in, ext_ID, receiver_ID, ext_team_ID, ttl, buffer+15, buffer);
  
  // if (strcmp(code_in, "EBR") == 0) {
    // printf("%d received EBR %s\n", my_ID, buffer);
  // }
  
  // if (strcmp(code_in, "MOV") == 0) {
    // printf("%d received MOV %s which duplicate: %d\n", my_ID, buffer, duplicate_message);
  // }
  // if (strcmp(code_in, "ILM") == 0) {
    // printf("%d received %s\n", my_ID, buffer);
    // printf("  duplicate %d\n", duplicate_message);
  // }
  // if (duplicate_message == false) {
    // printf("%d received %s\n", my_ID, buffer);
  // }
  // if (strcmp(code_in, "LOC") == 0 && my_ID == 3) {
    // printf("%d receiver_ID != my_ID %d !duplicate_message %d to_relay %d\n", my_ID, receiver_ID != my_ID, !duplicate_message, to_relay);
    // printf("%d (!duplicate_message || to_relay) %d all %d\n", my_ID, (!duplicate_message || to_relay), ttl > 0 && receiver_ID != my_ID && (!duplicate_message || to_relay));
  // }
  if (ttl > 0 && receiver_ID != my_ID && (!duplicate_message || strcmp(code_in, "LOC") == 0 || strcmp(code_in, "ILM") == 0 || strcmp(code_in, "ARR") == 0)) {
    /* relay the message if needed */
    // if (strcmp(code_in, "LOC") == 0) {// && my_ID == 3) {
      // printf("%d relayed %s\n", my_ID, buffer);
    // }
    
    relayed = false;
    
    for (i=messages_relayed_size; i>0; i--) {
      memset(tmp_s, 0, 37+1);
      strncpy(tmp_s, messages_relayed[i], 12); /* check the sender */
      memset(tmp_s2, 0, 37+1);
      strncpy(tmp_s2, buffer, 12);
      if (strcmp(tmp_s, tmp_s2) == 0) {
        memset(tmp_s, 0, 37+1);
        strcpy(tmp_s, messages_relayed[i]+15);
        memset(tmp_s2, 0, 37+1);
        strcpy(tmp_s2, buffer+15);
        // printf("-- checking messages_relayed: %s and extra %s\n", tmp_s, tmp_s2);
        if (strcmp(tmp_s, tmp_s2) == 0) {
          relayed = true;
          break;
        }
      }
    }
    
    if (!relayed) {
      // printf("%d relaying %s\n", my_ID, buffer);
      memset(message, 0, 37+1);
      strncpy(message, buffer, 12);
      sprintf(tmp_s, "%03d", ttl-1);
      strcat(message, tmp_s);
      strcat(message, buffer+15);
      wb_emitter_send(emitter_bt, message, strlen(message) + 1);
      strcpy(messages_relayed[messages_relayed_size], buffer);
      messages_relayed_size += 1;
    }
  }
  
  // if (strcmp(code_in, "ILM") == 0 && my_ID == 14) {
    // printf("%d received ILM from %d: %s\n", my_ID, ext_ID, buffer);
    // printf("  duplicate: %d\n", duplicate_message);
  // }
  
  // if (strcmp(code_in, "LOC") == 0) {
    // memset(tmp_ss, 0, 3+1);
    // strncpy(tmp_ss, buffer+15, 3); // 
    // if ((atoi(tmp_ss) == my_ID && ext_ID == leader_ID)) { 
      // printf("%d I'm a relay now\n", my_ID);
      // relay = true;
      // store_external_connection(receiver_ID_s, ext_ID_s, ext_team_size); // ext_ID_s and ext_leader_ID_s are the same thing, so we use the slot of the leader ID to send the angle
    // }
    // else if (receiver_ID == my_ID && !leader) {
      // printf("%d I'm a relay now\n", my_ID);
      // relay = true;
      // store_external_connection(receiver_ID_s, ext_ID_s, ext_team_size); // ext_ID_s and ext_leader_ID_s are the same thing, so we use the slot of the leader ID to send the angle
    // }
    // else if (leader && !team_player) {
      // printf("%d I'm a relay now\n", my_ID);
      // relay = true;
      // leader_store_external_connection(receiver_ID_s, ext_ID_s, ext_team_size); // ext_ID_s and ext_leader_ID_s are the same thing, so we use the slot of the leader ID to send the angle
    // }
    // else {
      // relay = false;
    // }
  // }
  
  if (duplicate_message && !whitelisted) {
    // if (strcmp(code_in, "UPD") == 0) {
      // printf("%d discard UPD for %d\n", my_ID, ext_ID);
    // }
    return;
  }
  
  // if (my_ID != 111) {
    // printf("%d received %s\n", my_ID, buffer);
  // }
  
  switch(hash_codes(code_in))
  {
    /* 
    
    Variable:
    
    SSS   - external_team_player - Y if external bot is in a team
    SSS   - external_team_size - size of the external team
    SSS   - ext_leader - Y if the external bot is the leader of its team
    
    To Do: Delete - we have the team_ID - SSS - ext_leader_ID_s - ID of the external team's leader
    
    */
    case 0: //"DNB" - Discover New Bot
      ext_team_player = buffer[15];
      memset(ext_team_size_s, 0, 3+1);
      strncpy(ext_team_size_s, buffer+16, 3);
      ext_team_size = atoi(ext_team_size_s);
      ext_leader = buffer[19];
      strncpy(ext_leader_ID_s, buffer+20, 3);
      ext_leader_ID = atoi(ext_leader_ID_s);
      
      if (whitelisted) {
        printf("%d from whitelisted, processing DNB\n", my_ID);
        whitelisted = false;
        memset(whitelisted_leader_ID_s, 0, 3);
        /* If the bot is whitelisted we simply do the same thing */
        printf("%d whitelist part:\n", my_ID);
        printf("  IDs are: ");
        for (i=0; i<6; i++) {
          printf("%d ", team_IDs[i]);
        }
        printf("\n");
        team_player = true;
        if (leader_ID < atoi(ext_ID_s)) {
          /* external bot will join my team */
        
          printf("%d inglobating %s\n", my_ID, ext_ID_s);
          inglobate_external_team(ext_ID_s, "000");
        }
        else if (leader_ID == ext_leader_ID) {
          break;
        }
        else {
          /* join other bot in a new team */
          /* sends a series of LJT messages for each bot in the team */
          /* since it's only a single bot, we might just swap the ID of the bot 
             and the ID of actual leader so we do not have to transfer the whole data */
          join_external_team(ext_ID_s, ext_ID_s);
          confirmation = 0;
          printf("%d joining team %s from %s\n", my_ID, ext_ID_s, ext_ID_s);
        }
        break;
        // join_external_team(ext_ID_s, ext_leader_ID_s);
        // confirmation = 0;
        // printf("%d joining team %s from %s\n", my_ID, ext_leader_ID_s, ext_ID_s);
      
        // ID_whitelist
        // if (idx_team == 0) {
          /* I'm just a single bot and I'm not the leader */
          // printf(" -- --- --- --- -- 222\n");
          // leader = false;
        // }
        /* treat him as an inglobate */
        // printf("%d team player %d\n", my_ID, team_player);
        // if (team_player == 1) {
          // printf("  1\n");
          // inglobate_external_team(ext_ID_s, "000");
        // }
        // else {
          // printf("%d  2\n", my_ID);
          // inglobate_external_team(ext_ID_s, "000");
          // my_ID = ext_ID;
          // strcpy(my_ID_s, ext_ID_s);
          // leader = false;
          // printf("%d  leader %d leader ID %d\n", my_ID, leader, leader_ID);
          // no need to update leader info, they are already set to my previous data
        // }
        // printf("%d whitelisted %d getting inglobated\n", my_ID, atoi(ext_ID_s));
        // if (my_ID == 8) {
          // printf("%d in pre-wtf\n", my_ID);
        // }
        
        // break;
      }
      
      // if (atoi(whitelisted_leader_ID_s) > 0 && atoi(whitelisted_leader_ID_s) != atoi(ext_ID_s) && atoi(whitelisted_leader_ID_s) != atoi(ext_leader_ID_s)) {
        
        // break;
      // }
      // if (my_ID == 8) {
        // printf("%d in wtf from %s\n", my_ID, buffer);
      // }
      ////
      //
      // To Do: If I'm the leader (e.g. at the real beginning there are 8 bots) I should declare a bot in charge of it
      //
      ////
      printf("%d handling DNB\n", my_ID);
      in_queue = false;
  
      for (i=0; i<idx_team; i++) {
        if(team_IDs[i] == ext_ID || ext_ID == my_ID) {
          in_queue = true; /* discard this communication */
          break;
        }
      }
      
      // if (my_ID == 15)
        // printf("%d received DNB from %d whic his %s\n", my_ID, ext_ID, buffer);
        
      if (ext_team_size + idx_team + 1 > 7 && !in_queue) { // idx_team starts from 0, so the number of bots is actually idx_team + 1
        // if (my_ID == 1) {
          printf("%d size overflow\n", my_ID);
          printf("%d message %s\n", my_ID, buffer);
        // }
        /*
  
          We inform our leader of the new request
          The leader will calculate the best route and respond accordingly
          
          There are a few special cases:
          -  If a full team locates a single bot then the leader of this second 
             small team (the single bot) might end up in this place. In this case, the leader of this
             second small team will only store the info about the other bot
          -  Otherwise we communicate the best route to the best of our knownledge 
             (based on size and location). We do this by constructing an EBR message
             for our leader who will do the calculations and return the info
          In any case we'll save the bot in our external queue if empty.
          If it is not empty, there are two cases:
          -  The new bot is routed towards our pre-existing external bot. In this case
             it makes no sense to save this bot again because it'll move outside of our range
          - The new bot is routed somewhere else. In this case it also makes no sense to save it
          In the first case however, we might end up in a situation where the new bot
          will take the position of the old external bot (which in turn will move further away for some reason.
          In this case we'll receive a different message and update the external bot later 
    
        */
        
        
        //
        // Change: here the non leader sends a EBR message to the leader which will handle the case
        //
        
        snprintf(tmp_ss, 4, "%03d", my_team_idx);
           
        // right now we simply split the teams, we could otherwise inglobate only part of them     
        // store_external_connection(ext_ID_s, ext_leader_ID_s, ext_team_size);

        /* 
        It is impossible that the leader of a full team will meet another bot such that their total size is 7
        -  It is possible that the full team's ID is higher than the one of the newly met team, in this case we do not aggregate to them anyway
        
        // To Do: consider partial aggregation
        
        We only consider when a follower of a full team meets a new bot
        */
        // if (ext_ID == 10)
          // printf("processing 10: %s\n", buffer);
        if (leader_ID < ext_leader_ID) { /* leading team, we'll redirect the newly met team towards their goal position */
          if (!leader) {
          /* If the ext_bot is the leader of it's team, we just save it */
            // if (strcmp(ext_ID_s, ext_leader_ID_s) != 0) { /* ext bot is not leader of it's team */
              // printf("%d step 3\n", my_ID);
              memset(tmp_s, 0, 37+1);
              strncpy(tmp_s, tmp_ss, 3);
              strcat(tmp_s, ext_ID_s);
              strcat(tmp_s, ext_leader_ID_s);
              strcat(tmp_s, ext_team_size_s);
              construct_share_with_team_message("EBR", my_ID_s, leader_ID_s, leader_ID_s, "000", tmp_s);
              // printf("%d sending EBR\n", my_ID);
              wb_emitter_send(emitter_bt, message, strlen(message) + 1);
              // if (ext_ID == 10)
                // printf("sent EBR 10: %s\n", message);
            // }
          }          
        }
        // else { /* follower team, we'll wait to be redirected */
          // store_external_connection(ext_ID_s, ext_leader_ID_s, ext_team_size);
        // }
        
        
        
        
        
        // this cannot be right or for each bot we encounter we overwrite everything 
        
        
        
        
        // store_external_connection(ext_ID_s, ext_leader_ID_s, ext_team_size);
        // relay = true;
        // printf("%d relay 2\n", my_ID);
        break;
      }
      
      if (!team_player) { // at this point for sure I'll either join or inglobate at least anothe robot
        team_player = true;
      }
      
      // new_bot = true; // for debug
      
      /* check who has the lowest ID to determine who is the leader */
      if (leader_ID <= ext_leader_ID) {
        /* external bot will join my team */
        // if (ext_ID == 10)
          // printf("%d inglobating 10\n", my_ID);
        // printf("%d ahia\n", my_ID);
        inglobate_external_team(ext_ID_s, "000");
        
      }
      // else if (leader_ID == ext_leader_ID) {
        // printf("%d same leader\n", my_ID);
        // break;
      // }
      else {
        /* join other bot in a new team */
        /* sends a series of LJT messages for each bot in the team */
        /* since it's only a single bot, we might just swap the ID of the bot 
           and the ID of actual leader so we do not have to transfer the whole data */
        join_external_team(ext_ID_s, ext_leader_ID_s);
        confirmation = 0;
        printf("%d joining team %s from %s\n", my_ID, ext_leader_ID_s, ext_ID_s);
      }
      break;
    /*
    
    Variable:
    
    S - last_queued - Y if this is the last bot in queue of the external bot
    
    */ 
    case 1: // LJT - List of other team-members to Join my Team
      /*
      
        There are two cases here:
        - If we are the leader we should save the info of the new bot
        and, at the same time, transmit a message to this new robot and containing
        his location
        - If we are not the leader we simply save the info about the new bot
      
      */
     
      strncpy(tmp_ss, buffer+15, 3);
      printf("%d received LJT %s\n", my_ID, tmp_ss); 
      inglobate_external_team(tmp_ss, "003"); /* go through hop and reach the other (old) leader */
      
      // char term = buffer[18];
      // if (term == 'Y') {
        // for (i=0, i<
      // }
      break;
    /*
    
    Variable:
    
    Delete SSS - external_leader_ID - ID of the leader of the external team
    Add SSS - team_idx_s - my index in the team
    
    */
    case 2: //"TTT" - Transfer To the new Team
      leader_ID = atoi(ext_team_ID_s); // set up new leader
      strcpy(leader_ID_s, ext_team_ID_s);
      printf("%d TTT to leader %s\n", my_ID, leader_ID_s);
      leader = false;
      
      append_bot(leader_ID_s);
      
      break;
    /*
    
    Variable:
    
    SSS - ext_ID_s - ID of the external bot we are inglobating
    
    */
    case 3: //"ITM" - Inform Team Member      
      /*
      
      There are two cases here:
      - If we are the leader we should save the info of the new bot
      and, at the same time, transmit a message to this new robot and containing
      his location
      - If we are not the leader we simply save the info about the new bot
      
      */ 
      // printf("%d handling ITM\n", my_ID);
      // if (receiver_ID != my_ID)
        // break;
      memset(tmp_ss, 0, 3+1);
      strncpy(tmp_ss, buffer+15, 3);
      printf("%d received ITM for %d: %s\n", my_ID, atoi(tmp_ss), buffer);
      if (atoi(tmp_ss) != my_ID) {
        append_bot(tmp_ss);
      }
      
      if (leader && !in_queue && !queue_full) {
        printf("%d sending location to %d\n", my_ID, atoi(tmp_ss));
        inform_new_location(tmp_ss, "006"); // TTL set to 7 as the worst case
        // if (atoi(tmp_ss) == 10)
          // printf("%d sending location to 10\n", my_ID);
        waiting_new_bot = true;
      }
      
      if (!in_queue && !queue_full && global_movement) {
        global_movement = false;
        waiting_new_bot = true;
        printf("%d new bot so stopping\n", my_ID);
      }
    break;
    /*
    
    Variable:
    
    SSSSSSS - x_ref_s - x coordinate of the leader
    SSSSSSS - y_ref_s - y coordinate of the leader
    S       - my_team_idx_s - ID given to me by the leader; based on this value I'll move to a certain location
    
    */
    case 4: // "ILM" - Inform of new Location Message
      printf("%d received ILM from %d: %s\n", my_ID, ext_ID, buffer);
      printf("%d handling ILM\n", my_ID);
      if (receiver_ID != my_ID)
        break;
      
      confirmation = -1;
      memset(x_ref_s, 0, 7+1);
      strncpy(x_ref_s, buffer+15, 7);
      x_ref = atof(x_ref_s);
      memset(y_ref_s, 0, 7+1);
      strncpy(y_ref_s, buffer+22, 7);
      y_ref = atof(y_ref_s);
      memset(team_angle_s, 0, 3+1);
      strncpy(team_angle_s, buffer+29, 3);
      team_angle = atoi(team_angle_s);
      memset(my_team_idx_s, 0, 1+1);
      strncpy(my_team_idx_s, buffer+32, 1);
      my_team_idx = atoi(my_team_idx_s);
      global_leader = false;
      
      // printf("%d received ILM %s\n", my_ID, buffer);
      
      if (strcmp(whitelisted_leader_ID_s, ext_ID_s) == 0) {
        printf("%d from whitelisted leader\n", my_ID);
        reset_team();
        strcpy(leader_ID_s, ext_ID_s);
        leader_ID = atoi(ext_ID_s);
        memset(whitelisted_leader_ID_s, 0, 3);
        leader = false;
        team_player = true;
      }
      
      if (leader) {

        // If yes we might be in the case "step 7" of handle_excess_bot
        
        break; /* shouldn't be a possibility */
      }
      else {
        handle_position_f(x_ref, y_ref, my_team_idx, true);
        global_rotation = 1;
      }
      break;
    /*
    
    Variable:
    
    SSS - hop_team_idx - 
    SSS - ext_ID_s - 
    SSS - ext_leader_ID_s -
    SSS - ext_team_size - 
        
    */
    case 5: // "EBR" - External Bot Request
      /*
      
      This is a message that a slave sends to the leader
      The leader will therefore check the available external connections and redirect the bot to the closest one
      
      */
      printf("%d handling EBR\n", my_ID);
      memset(hop_team_idx, 0, 3+1);
      strncpy(hop_team_idx, buffer+15, 3);
      strncpy(hop_ID_s, ext_ID_s, 3);
      memset(ext_ID_s, 0, 3+1);
      strncpy(ext_ID_s, buffer+18, 3);
      memset(ext_leader_ID_s, 0, 3+1);
      strncpy(ext_leader_ID_s, buffer+21, 3);
      memset(ext_team_size_s, 0, 3+1);
      strncpy(ext_team_size_s, buffer+24, 3);
      ext_team_size = atoi(ext_team_size_s);
      printf("%d received EBR: %s\n", my_ID, buffer);
      printf("  hop_ID_s %s hop_team_idx %s ext_ID_s %s \n", hop_ID_s, hop_team_idx, ext_ID_s);
      handle_excess_bot(hop_ID_s, hop_team_idx, ext_ID_s, ext_leader_ID_s, ext_team_size);
      break;
    /*
    
    Variable:
    
    SSSSSSS - 
    SSSSSSS - 
    
    */
    case 6: // "LOC" - Localization Bot Request
      // printf("%d received LOC %s\n", my_ID, buffer);
      global_leader = false;
      
      printf("%d handling LOC\n", my_ID);
      //
      //
      // Add hop_ID from idx=15 to idx=17
      //
      //
      memset(hop_ID_s, 0, 3+1);
      strncpy(hop_ID_s, buffer+15, 3);
      memset(tmp_ss, 0, 3+1);
      strncpy(tmp_ss, buffer+18, 3); // actual ext_leader_ID_s
      if (strcmp(ext_ID_s, leader_ID_s) != 0 && atoi(receiver_ID_s) != my_ID) {
        /* Probably relayed from external team, I don't care */
        // we could use this opportunity to update our data
        return;
      }
      
      if (confirmation != -1) {
        /* I'm not actually in a team */
        reset_team();
        printf("%d team_player: %d\n", my_ID, team_player);
      }
      
      if (strcmp(hop_ID_s, my_ID_s) == 0) { // I'm the hop, just store the data and go away
        // store_external_connection(ext_ID_s, ext_leader_ID_s, ext_team_size);
        printf("%d relayyy 1\n", my_ID);
        // if (strcmp(tmp_ss, leader_ID_s) == 0) { 
        if (strcmp(tmp_ss, receiver_ID_s) == 0) { 
          /* we might end up here if we are transferring a bot to an external team */
          if (ext_connection.size == 0) {
            store_external_connection(receiver_ID_s, receiver_ID_s, ext_team_size);
            printf("  - storing %s, with leader %s and size %d from %s\n", receiver_ID_s, receiver_ID_s, ext_team_size, buffer);
          }
          else {
            ext_connection.size += 1;
          }
        }
        else {
          // for (i=messages_lookback_size[0]; i>0; i--) {
            // memset(tmp_ss, 0, 3+1);
            // strncpy(tmp_ss, messages_lookback[0][i]+3, 3); /* check the sender */
            // if (atoi(tmp_ss) == atoi(receiver_ID_s)) {
              // strncpy(tmp_ss, messages_lookback[0][i]+9, 3);
              store_external_connection(receiver_ID_s, tmp_ss, ext_team_size);
              printf("  + storing %s, with leader %s and size %d from %s\n", receiver_ID_s, tmp_ss, ext_team_size, buffer);
            // }
          // }
        }
        printf("%d relay true 1\n", my_ID);
        relay = true;
        
        break;
      }
      printf("  my ref was x %f y %f\n", x_ref, y_ref);
      printf("  my goal was x %f y %f\n", x_goal, y_goal);
      
      memset(x_ref_s, 0, 7+1);
      strncpy(x_ref_s, buffer+21, 7);
      x_ref = atof(x_ref_s);
      memset(y_ref_s, 0, 7+1);
      strncpy(y_ref_s, buffer+28, 7);
      y_ref = atof(y_ref_s);
      memset(my_team_idx_s, 0, 1+1);
      strncpy(my_team_idx_s, buffer+35, 1);
      my_team_idx = atoi(my_team_idx_s);
      strcpy(team_angle_s, ext_team_ID_s);
      team_angle = ext_team_ID;
      
      /* 
      
      If the sender is not in my team, and therefore if it comes from the external team 
      I set myself as relay and update my team's positions
      
      A few scenarios may arise:
      - I am a single robot, therefore I'm leader of my team
        In this case I update only my position
      - I am follower of a team
        I send a LOC message to my leader and update his location
      - I am leader of a team
        This is the result of the previous case. In this case I update all my followers
      
      */
      // if (atoi(hop_ID_s) == my_ID && ext_ID == leader_ID) {
        // printf("%d I'm a relay now\n", my_ID);
        // relay = true;
        // store_external_connection(receiver_ID_s, ext_ID_s, ext_team_size); // ext_ID_s and ext_leader_ID_s are the same thing, so we use the slot of the leader ID to send the angle
      // }
      
      
      team_player = true;
      
      in_queue = false;
      for (i=0; i<idx_team; i++) {
        if (team_IDs[i] == ext_ID) {
          in_queue = true;
        }
      }
      
      if (!in_queue) { /* Coming from outside */
        if (leader) { /* Scenario 1: Update my location */
          printf("%d bis team_player: %d\n", my_ID, team_player);
          if (team_player) {
            printf("%d scenario 1\n", my_ID);
            printf("  storing %s, with leader %s and size %d from %s\n", hop_ID_s, ext_ID_s, ext_team_size, buffer);
            my_team_idx = ((my_team_idx + 2) % 6) + 1;
            handle_position_f(x_ref, y_ref, my_team_idx, true);
            leader_store_external_connection(my_team_idx, hop_ID_s, ext_ID_s, ext_team_size);
          }
          else {
            /* single bot team */
            printf("%d scenario 1 bis\n", my_ID);
            printf("  storing %s, with leader %s and size %d from %s\n", hop_ID_s, ext_ID_s, ext_team_size, buffer);
            my_team_idx = ((my_team_idx + 2) % 6) + 1;
            handle_position_f(x_ref, y_ref, my_team_idx, true);
            store_external_connection(hop_ID_s, ext_ID_s, ext_team_size);
            // printf("  my leader is %s\n", leader_ID_s);
            printf("%d realy true 2\n", my_ID);
            relay = true;
          }
        }
        else { /* Scenario 2: Inform the leader */
          printf("%d scenario 2\n", my_ID);
          handle_position_f(x_ref, y_ref, my_team_idx, true);
          memset(tmp_s, 0, 37+1);
          /* 
          The leader will receive the message and he'll use the idx = 0
          However, we send the idx of our team's hop so the leader can save him there
          */
          printf("  storing %s, with leader %s and size %d from %s\n", hop_ID_s, ext_ID_s, ext_team_size, buffer);
          store_external_connection(hop_ID_s, ext_ID_s, ext_team_size);
          // sprintf(tmp_s, "%s%s%07.3f%07.3f%1d", hop_ID_s, tmp_ss, x_ref, y_ref, my_team_idx);
          sprintf(tmp_s, "%s%s%07.3f%07.3f%1d", hop_ID_s, ext_ID_s, x_ref, y_ref, my_team_idx);
          construct_share_with_team_message("LOC", my_ID_s, leader_ID_s, team_angle_s, "001", tmp_s);  // vals[0], vals[1], team_bearing);
          // construct_share_with_team_message("LOC", my_ID_s, ext_ID_s, team_angle_s, "001", tmp_s);  // vals[0], vals[1], team_bearing);
          wb_emitter_send(emitter_bt, message, strlen(message) + 1);
          relay = true;
          printf("%d relay true 3\n", my_ID);
        } 
      }
      else { /* Scenario 3: Coming from my follower; Update my location and inform my followers */
        printf("%d scenario 3\n", my_ID);
        // printf("  my idx: %d\n", 0);
        handle_position_f(x_ref, y_ref, 0, true);
        printf("  storing %s, with leader %s and size %d from %s\n", hop_ID_s, tmp_ss, ext_team_size, buffer);
        leader_store_external_connection(((my_team_idx + 2) % 6) + 1, hop_ID_s, tmp_ss, ext_team_size);

        /* We have to set the hop idx to the correct place */
        
        
        // We can do like: for i=0, i<7, i++ if team_IDs != -1 do stuff
        
        
        
        for (i=0; i<idx_team; i++) {
          memset(tmp_ss, 0, 3+1);
          sprintf(tmp_ss, "%03d", team_IDs[i]);
          angle_compass = get_bearing_in_degrees(compass);
          printf("%d sending ILM with compass %f\n", my_ID, angle_compass);
          construct_inform_location_message(my_ID_s, tmp_ss, leader_ID_s, "000", x_ref, y_ref, team_angle, i+1);
          printf("%d sending ILM with idx_team %d to %s: %s\n", my_ID, idx_team, tmp_ss, message);
          wb_emitter_send(emitter_bt, message, strlen(message) + 1);
        }
      }
      
      // relay = true;
      
      // handle_position_f(x_ref, y_ref, my_team_idx, true);
      // printf("  my ref became x %f y %f\n", x_ref, y_ref);
      // printf("  my goal became x %f y %f\n", x_goal, y_goal);
      
      /* If I'm in a team, I should update location for all my team */
      // if (team_player && !leader) {
        // construct_share_with_team_message("LOC", my_ID_s, ext_ID_s, leader_ID_s, "001", tmp_s);  // vals[0], vals[1], team_bearing);
        // update_team(x_ref, y_ref);
      // }
      // if (team_player && leader) {
        // for (i=0; i<idx_team; i++) {
          // if (team_IDs[i] == sender) {
            // continue;
          // }
          // construct_share_with_team_message("LOC", my_ID_s, ext_ID_s, leader_ID_s, "001", tmp_s);  // vals[0], vals[1], team_bearing);
        // }
      // }
      // printf("%d my goal %f %f\n", my_ID, x_goal, y_goal);
      // printf("  my ref %f %f\n", x_ref, y_ref);
      // printf("%d my position %d\n", my_ID, location_change);
      break;
    case 7: 
      // printf("%d handling ARR\n", my_ID);
      printf("%d received ARR from %d stating %s\n", my_ID, ext_ID, buffer);
      // printf("  arrived followers: %d, team size: %d\n", arrived_folls + 1, idx_team);
      // arrived_folls += 1;
      
      /* For each message I will flag that position as ready */
                // construct_share_with_team_message("ARR", my_ID_s, leader_ID_s, leader_ID_s, "000", "10");
      // strncpy(tmp_ss, buffer+15, 2);
      // int iid = atoi(tmp_ss);
      ete = buffer[18] - '0'; // ext_team_exists: Check if external team exists
      etr = buffer[19] - '0'; // ext_team_ready: Check if external team is ready
      etrf = buffer[20] - '0'; // ext_team_ready_final: Check if external team is completely ready
      
      /* A team is temporarily ready if it's blocked only by external constraints (other teams)
      If a temporarily ready team received as temporarily ready message from all it's blocking neighbors, 
      then it will change to completely ready */
      
      
      // printf("%d received ete %d, etr %d, etrf %d\n", my_ID, ete, etr, etrf);
      // printf("  char version ete %c, etr %c, etrf %c\n", buffer[18], buffer[19], buffer[20]);
      
      // arrived_folls += 1;
      // break;
      
      // set the flag if need and wait to flag this connection as ok
      // update the sum of missing ext connections +1
      if (!leader) {
        /* here comes the message from outside, we should process it differently */
        if (ext_ID == atoi(ext_connection.ext_leader_ID_s)) {
          /* we rework this message and send it to our leader */
          if (etrf) {
            printf("%d sending 1\n", my_ID);
            sprintf(tmp_s, "000111");
          }
          else {
            printf("%d sending 2\n", my_ID);
            sprintf(tmp_s, "000110");
          }
          printf("%d sending 3\n", my_ID);
          // construct_share_with_team_message("ARR", my_ID_s, leader_ID_s, leader_ID_s, "001", tmp_s);
          construct_share_with_team_message("ARR", ext_ID_s, leader_ID_s, leader_ID_s, "001", tmp_s);
          wb_emitter_send(emitter_bt, message, strlen(message) + 1);
        }
        break;
      }
      
      
      
      
      // missing_ext initialized to idx_team so when it reaches 0 we know that we have traversed every spot
      
      
      
      /* No matter what, we know that a bot has arrived 
      We can therefore increase the counter of arrived bots and then process the external conections*/
      
      arrival_update = true;
      if (!ete) {
        printf("%d received 1\n", my_ID);
        /* If there is no external connection we should wait for, we simply flag this position as ready */
        // arr_folls_r[iid] = 1;
        // missing_ext_r[iid] = 0;
        arrived_folls += 1;
        arrived_ext_part += 1;
        arrived_ext_full += 1;
      }
      else {
       /* If external connection exists, we check whether it says it's done 
         At the same time we collect info about the arrived team
       */
       
       
       if (etr) {
       
         printf("%d received 2\n", my_ID);
         /* External team is informing us that they are done and ready to move 
         We can update the flags and update the sum of missing ext connections*/

         /* If we are all within each other's communication range I might get this message twice
           We need an array wit harrived external teams and check against it */
         
         if (etrf) {
           printf("%d received 3\n", my_ID);
           // missing_ext_r[iid] = 0;
           // missing_ext -= 1;
           // arrived_ext_part -= 1;
           /* We cannot come here and receive twice this message, as we need to be in place before the other teams are fully ready, therefore outside their range */
           for (i=0; i<6; i++) {
             // printf("%d checking %d\n", my_ID, team_IDs[i]);
             // if (team_IDs[i] == ext_ID || atoi(ext_connections[i].ext_leader_ID_s) == ext_ID) 
             if (atoi(ext_connections[i].ext_leader_ID_s) == ext_ID) {
               /* We have found the relay in our team */
               printf("%d found at %d\n", my_ID, i);
               break;
             }
           }
       
           if (arr_ext_full[i] == 1) {
             /* it has already arrived, so break */
             printf("%d skip this\n", my_ID);
             break;
           }
           
           arr_ext_full[i] = 1;
           printf("  saving in place %d\n", i);
           arrived_ext_full += 1;
         }
         else {
           
           printf("%d ext ID is %d\n", my_ID, ext_ID);
           printf("  I know that pos 0 has %d, pos 1 has %d and pos 3 has %d\n", team_IDs[0], team_IDs[1], team_IDs[3]);

           printf("  I know that ext pos 0 has %d, pos 1 has %d, pos 2 has %d, pos 3 has %d, pos 4 has %d and pos 5 has %d\n", atoi(ext_connections[0].ext_leader_ID_s), atoi(ext_connections[1].ext_leader_ID_s), atoi(ext_connections[2].ext_leader_ID_s), atoi(ext_connections[3].ext_leader_ID_s), atoi(ext_connections[4].ext_leader_ID_s), atoi(ext_connections[5].ext_leader_ID_s));
           for (i=0; i<6; i++) {
             // printf("%d checking %d\n", my_ID, team_IDs[i]);
             // if (team_IDs[i] == ext_ID || atoi(ext_connections[i].ext_leader_ID_s) == ext_ID) 
             if (atoi(ext_connections[i].ext_leader_ID_s) == ext_ID) {
               /* We have found the relay in our team */
               printf("%d found at %d\n", my_ID, i);
               break;
             }
           }
       
           if (arr_ext[i] == 1) {
             /* it has already arrived, so break */
             printf("%d skip this\n", my_ID);
             break;
           }
         
         
           printf("%d received 4\n", my_ID);
           /* If the external team is not completely ready, we count it as partial */
           // missing_ext_r[iid] = 1;
           arr_ext[i] = 1;
           printf("  saving in place %d\n", i);
           arrived_ext_part += 1;
         }
       }
       else {
         printf("%d received 5\n", my_ID);
         arrived_folls += 1;
         /*missing_ext_r[iid] is initialized to 1 */
         // missing_ext_r[iid] = 1;
         // missing_ext += 1;
       }
      }
      
      
      
      // if (leader && arr_folls == idx_team) {
        /* Everyone is arrived so we can share with the external teams that we are completely ready */
        // for (i=0; i<idx_team; i++) {
          // if (ext_connections[i].size > 0) {
            /* There is an external team so we communicate that we are ready */
                      //
                      //
                      // message
                      //
                      //            
          // }
        // }                
      // }
      
      printf("%d maybe done\n", my_ID);
      printf("  leader? %d arrived falls %d idx_team %d\n", leader, arrived_folls, idx_team);
       if (leader && arrived_folls == idx_team && !location_change) {
         check_team_arrival();
         arrival_update = false;
       }
      
      
      // missing_ext = 0;
      // for (i=0; i<idx_team; i++) {
        // if (missing_ext_r[i] == 1) {
          // missing_ext = 1;
          // break;
        // }
      // }
      
      // if (!missing_ext) {
      
      // }

      
      // printf("%d arrived folls: %d\n", my_ID, arrived_folls);
      
      
      
      
      // if comin from external team update the connection and the sum
      break;
    case 8: // UPD
      printf("%d handling UPD %s\n", my_ID, buffer);
      
      /* Check if it's a heartbeat or an update request */
      memset(tmp_ss, 0, 3+1);
      strncpy(tmp_ss, buffer+15, 1);
      
      switch (atoi(tmp_ss)) 
      {
        case 0:
          /* In the following, each update we make should be communicate to our leader */
          printf("%d --- doing crazy stuff %s\n", my_ID, buffer);
          memset(tmp_ss, 0, 3+1);
          strncpy(tmp_ss, buffer+16, 1);
          tmp = atoi(tmp_ss); // ext_bot_idx
          memset(tmp_ss, 0, 3+1);
          strncpy(tmp_ss, buffer+17, 1);
          ext_team_size = atoi(tmp_ss); // ext_team_size
          /* If we receive a message from a bot whose idx is opposite of ours, we can add him to our ext_team queue */
          /* For this task we need to know the idx of the bot, it's team and the distance to his goal (if too far we better wait) */
          if (tmp == ((my_team_idx + 2) % 6) + 1) {
            if (ext_ID != atoi(ext_connection.ext_ID_s) && strcmp(ext_team_ID_s, leader_ID_s) != 0) {
              memset(tmp_ss, 0, 3+1);
              strncpy(tmp_ss, buffer+18, 3);
              tmp = atoi(tmp_ss); // ext_distance
              if (tmp < 10 && dist_to_goal < 10) {
                store_external_connection(ext_ID_s, ext_team_ID_s, ext_team_size);
                printf("%d storing %s leader %s size %d\n", my_ID, ext_ID_s, ext_team_ID_s, ext_team_size); 
                printf("  check %s %s %d\n", ext_ID_s, ext_team_ID_s, ext_team_size);
                memset(tmp_s, 0, 37+1);
                sprintf(tmp_s, "2%s%s%03d%03d", ext_ID_s, ext_team_ID_s, ext_team_size, my_team_idx);
                construct_share_with_team_message("UPD", my_ID_s, leader_ID_s, leader_ID_s, "000", tmp_s);
                wb_emitter_send(emitter_bt, message, strlen(message) + 1);
                relay = true;
              }
            }
          }
          break;
        case 1:
          /* If we receive a message from a team that we know and their size is different, we update it */
          printf("%d doing more crazy stuff %s\n", my_ID, buffer);
          if (ext_ID == atoi(ext_connection.ext_ID_s)) {
            if (ext_team_size != ext_connection.size) {
              store_external_connection(ext_ID_s, ext_team_ID_s, ext_team_size - ext_connection.size);
            }
          }
          break;
        case 2:
          memset(hop_team_idx, 0, 3+1);
          strncpy(hop_team_idx, buffer+25, 3);
          memset(ext_ID_s, 0, 3+1);
          strncpy(ext_ID_s, buffer+16, 3);
          memset(ext_leader_ID_s, 0, 3+1);
          strncpy(ext_leader_ID_s, buffer+19, 3);
          memset(tmp_ss, 0, 3+1);
          strncpy(tmp_ss, buffer+22, 3);
          ext_team_size = atoi(tmp_ss); // size
          printf("%d ----- updating at spot %d\n", my_ID, atoi(hop_team_idx));
          leader_store_external_connection(atoi(hop_team_idx), ext_ID_s, ext_leader_ID_s, ext_connections[atoi(hop_team_idx)-1].size - ext_team_size);
          printf(" storing %s leader %s size %d\n", ext_ID_s, ext_leader_ID_s, ext_team_size); 
          break;
        default:
          break;
        /* Since it's syncrhronous, we can keep a table of IDs and if after XXX iterations one bot hasn't contacted us, we consider it as dead */
        /* In this case we only change our led color */
        
      }
      // else {
        // memset(hop_ID_s, 0, 3+1);
        // strncpy(hop_ID_s, buffer+16, 3);
        // memset(ext_leader_ID_s, 0, 3+1);
        // strncpy(ext_leader_ID_s, buffer+19, 3);
        // memset(tmp_ss, 0, 3+1);
        // strncpy(tmp_ss, buffer+22, 1);
        // i = atoi(tmp_ss);
      
        // memset(ext_connection.ext_ID_s, 0, 3+1);
        // strcpy(ext_connection.ext_ID_s, hop_ID_s);
        // memset(ext_connection.ext_leader_ID_s, 0, 3+1);
        // strcpy(ext_connection.ext_ID_s, ext_leader_ID_s);
        // ext_connection.size = i;
        // relay = true;
        // printf("%d received UPD\n", my_ID);
        // printf("  relay true 4\n");
      // }
      break;
    case 9: // MOV
      memset(tmp_ss, 0, 3+1);
      strncpy(tmp_ss, buffer+15, 3);
      switch (atoi(tmp_ss))
      {
        case 0:
          location_change = false;
          waiting_new_bot = false;
          global_movement = 1;
          printf("%d fuck this shit\n", my_ID);
          break;
        case 1:
        {
          double x_t;
          double y_t;
          memset(tmp_s, 0, 37+1);
          strncpy(tmp_s, buffer+18, 7);
          x_t = atof(tmp_s);
          memset(tmp_s, 0, 37+1);
          strncpy(tmp_s, buffer+25, 7);
          y_t = atof(tmp_s);
          double *values = (double* )wb_gps_get_values(gps);
          x = values[2]-x_t;
          y = values[0]-y_t;
          // if (my_ID == 1) {
            printf("%d received MOV and my distance from the bot is %f and my oam is %d\n", my_ID, sqrt(x*x + y*y), oam_active);
            printf("  also received x: %f and y: %f\n", x_t, y_t);
            printf("  the message %s\n", buffer);
            printf("  time is: %d\n", team_collision_time);
          // }
        
          if (sqrt(x*x + y*y) < 0.2 && !oam_active) {
            /* Colliding with me, discard */
            memset(tmp_s, 0, 37+1);
            sprintf(tmp_s, "001%07.3f%07.3f", x+x_t, y+y_t);
            construct_share_with_team_message("MOV", my_ID_s, "000", "000", "007", tmp_s);
            wb_emitter_send(emitter_bt, message, strlen(message) + 1);
          }
          angle = atan2(y, x);
          if (y < 0) {
            angle = 360 + (angle * 180 / M_PI);
          }
          else {
            angle = angle * 180 / M_PI;
          }
          printf("%d asdffff ----- \n", my_ID);
          printf("  from %d \n", ext_ID);
          printf("  direction %f \n", angle); 
          if (global_leader) {
            tmp = -1;
            for (i=0; i<team_collision_count; i++) {
              if (fabs(team_collision_angle[i]-angle) < 2 && ext_ID != team_collision_ID[i]) {
                /* Cancel this collision */
                team_collision_angle[i] = 0;
                team_collision_ID[i] = 0;
                // team_collision_count -= 1;
                printf("%d cancelling one error\n", my_ID);
                // if (team_collision_count == 0) {
                  // /* Fully reset and do not waste time */
                  // team_collision_time = 0;
                // }
                break;
              }
              else if (ext_ID == team_collision_ID[i]) {
                break;
              }
              else if (team_collision_ID[i] == 0) {
                tmp = i;
              }
            }
            if (i == team_collision_count) {
              if (tmp != -1) {
                /* Save in empty spot */
                i = tmp;
              }
              team_collision_count += 1;
              team_collision_angle[i] = angle;
              team_collision_ID[i] = ext_ID;
              if (team_collision_time == 0) {
                team_collision_time = 1;
                printf("%d setting up time\n", my_ID);
              }
              else {
                printf("%d adding error\n", my_ID);
              }
            }
            printf("%d exiting mov\n", my_ID);
          }
          printf("%d exiting mov 2\n", my_ID);
          break;
        }
        case 2:
          /* If I received it and it's me that is stuck, I stop and say that I'm arrived */
          memset(tmp_ss, 0 , 3+1);
          strncpy(tmp_ss, buffer+18, 3);
          team_angle = atoi(tmp_ss);
          printf("%d received MOV 2 angle %d\n", my_ID, team_angle);
          
          // we gotta find a way to then return to our position, when there is enough space */
          if (oam_active) {
            dist_to_goal = 0.05;
            printf("  case 1\n");
          }
          else if (!location_change) {
            location_change = true;
            dist_to_goal = 0.05;
            global_rotation = true;
            printf("  case 2\n");
          }
          
          break;
        case 3:
          printf("%d MOV 3\n", my_ID);
          if (!location_change) {
            printf("%d MOV 3 inn\n", my_ID);
            global_movement = 0;
          }
          break;
        default:
          printf("%d MOV wtf\n", my_ID);
          break; 
      }     
    case 10: // IBA
      if (receiver_ID != my_ID) {
        break;
      }
      printf("%d received IBA %s\n", my_ID, buffer);
      
      memset(hop_ID_s, 0, 3+1);
      strncpy(hop_ID_s, buffer+18, 3);
      memset(hop_team_idx, 0, 3+1);
      memset(ext_team_size_s, 0, 3+1);
      strncpy(ext_team_size_s, buffer+21, 1);
      strncpy(hop_team_idx, buffer+22, 1);
      memset(tmp_ss, 0, 3+1);
      strncpy(tmp_ss, buffer+23, 1);
      
      // team_player = true;
      
      if (atoi(tmp_ss) == 0) {
        /* I'm the leader - store the state of the incoming bot */
        memset(tmp_ss, 0, 3+1);
        strncpy(tmp_ss, buffer+15, 3); // incoming bot ID
        ID_whitelist[whitelist_idx] = atoi(tmp_ss);
        whitelist_idx += 1;
        if (leader) {
          printf("  in position 4 have %d\n", team_IDs[((atoi(hop_team_idx) + 2) % 6) + 1]);
          leader_store_external_connection(((atoi(hop_team_idx) + 2) % 6) + 1, hop_ID_s, ext_ID_s, atoi(ext_team_size_s));
          if (team_IDs[((atoi(hop_team_idx) + 2) % 6) + 1] == 0) {
            /* No bot existing, we have to force the hop */
            // printf(" ////// shitttt /////\n");
            force_hop = ((atoi(hop_team_idx) + 2) % 6) + 1;
            // this needs to be an array if the size grows
            // break;
          }
          // else {
          // if fucking (out - 1 % 6) has nothing, send the bot here and inform him about the external thinghy thing
         
          // }
        
        
        
          handle_position_f(x_ref, y_ref, 0, true);
          // leader_store_external_connection(atoi(hop_team_idx), hop_ID_s, ext_ID_s, atoi(ext_team_size_s));
          printf("%d changed position cause I'm leader\n", my_ID);
          printf("  stored %s from team %s in position %d and size %d\n", hop_ID_s, ext_ID_s, ((atoi(hop_team_idx) + 1) % 6) + 2, atoi(ext_team_size_s));
        }
        printf("%d in 1\n", my_ID);
        printf("  added %d to whitelist\n", atoi(tmp_ss));
      }
      else if (atoi(tmp_ss) == 1) {
        /* I'm the follower - initiate  */
        memset(whitelisted_leader_ID_s, 0, 3+1);
        strncpy(whitelisted_leader_ID_s, buffer+15, 3);
        printf("%d in 2\n", my_ID);
      }  
      printf("%d whitelisting\n", my_ID);
      whitelisting = true;  
      break;
    default:
      break;
  }

}

////////////////////////////////////////////
// PHM - Position Handling Module
////////////////////////////////////////////







void handle_position_f(float x_ref, float y_ref, int my_team_idx, bool update) {
  
  switch(my_team_idx)
  {
    case 0:
      vals[0] = x_ref;
      vals[1] = y_ref; 
      break;
    case 1:
      vals[0] = x_ref + 5;
      vals[1] = y_ref + 8.66; 
      break;
    case 2:
      vals[0] = x_ref + 10; 
      vals[1] = y_ref + 0; 
      break;
    case 3:
      vals[0] = x_ref + 5; 
      vals[1] = y_ref - 8.66; 
      break;
    case 4:
      vals[0] = x_ref - 5; 
      vals[1] = y_ref - 8.66; 
      break;
    case 5:
      vals[0] = x_ref - 10; 
      vals[1] = y_ref + 0; 
      break;
    case 6:
      vals[0] = x_ref - 5; 
      vals[1] = y_ref + 8.66; 
      break;
    default:
      break;
    /*
    case 1:
      x_goal = -10; //x_ref - 10;
      y_goal = 0; //y_ref;
      break;
    case 2:
      x_goal = 10; //x_ref + 10;
      y_goal = 0; //y_ref;
      break;
    case 3:
      x_goal = -5; //x_ref - 5;
      y_goal = 8.66; //y_ref + 8.66;
      break;
    case 4:
      x_goal = 5; //x_ref + 5;
      y_goal = 8.66; //y_ref + 8.66;
      break;
    case 5:
      x_goal = -5; //x_ref - 5;
      y_goal = -8.66; //y_ref - 8.66;
      break;
    case 6:
      x_goal = 5; //x_ref + 5;
      y_goal = -8.66; //y_ref - 8.66;
      break;
    default:
      break;
    */
  }
  
  if (update) {
    x_goal = vals[0];
    y_goal = vals[1];
    location_change = true;
    in_line = false;
    printf("%d updated location\n", my_ID);
    printf("  my ref became x %f y %f\n", x_ref, y_ref);
    printf("  my goal became x %f y %f\n", x_goal, y_goal);
    printf("  leader became: %d\n", leader_ID);
  }
  
  return;
    
}

void recover_ref_pos(float x, float y, int my_team_idx) { // wrongggggg only works if we r still in our place : )
  if (leader) {
    x_ref = x;
    y_ref = y;
  }
  else {
    switch(my_team_idx)
    {
      case 1:
        x_ref = x - 5;
        y_ref = y - 8.66; 
        break;
      case 2:
        x_ref = x - 10; 
        y_ref = y + 0; 
        break;
      case 3:
        x_ref = x - 5; 
        y_ref = y + 8.66; 
        break;
      case 4:
        x_ref = x + 5; 
        y_ref = y + 8.66; 
        break;
      case 5:
        x_ref = x + 10; 
        y_ref = y + 0; 
        break;
      case 6:
        x_ref = x + 5; 
        y_ref = y - 8.66; 
        break;
      default:
        break;
      /*
      case 1:
        x_ref = x + 10;
        y_ref = y;
        break;
      case 2:
        x_ref = x - 10;
        y_ref = y;
        break;
      case 3:
        x_ref = x + 5;
        y_ref = y - 8.66;
        break;
      case 4:
        x_ref = x - 5;
        y_ref = y - 8.66;
        break;
      case 5:
        x_ref = x + 5;
        y_ref = y + 8.66;
        break;
      case 6:
        x_ref = x - 5;
        y_ref = y + 8.66;
        break;
      default:
        break;
      */
    }
  }

}

void inform_new_location(char* ext_ID_s, char* TTL) {
  /*
  
  we inform the new bot of it's new location in the team
  even though we are not the leader, we know the current team_idx so we can share it
  we also know the reference position for the leader and we can share it also
  
  */
  
  double *values = (double* )wb_gps_get_values(gps);
  double x = values[2];
  // double z = values[1];
  double y = values[0];
  
  // bearing = get_bearing_in_degrees(compass);
  
  recover_ref_pos(x, y, my_team_idx);
  // printf("%d reference location x: %f, y: %f\n", my_ID, x_ref, y_ref);
  
  sprintf(x_ref_s, "%07.3f", x_ref);
  sprintf(y_ref_s, "%07.3f", y_ref);
  
  angle_compass = get_bearing_in_degrees(compass);
  
  if (leader) {
    if (global_leader) {
      // printf("%d updating team_angle to %f\n", my_ID, angle_compass);
      team_angle = angle_compass;
      sprintf(team_angle_s, "%03d", team_angle);
    }
  }
  if (force_hop > 0) {
    // construct_inform_location_message(my_ID_s, ext_ID_s, leader_ID_s, TTL, x_ref, y_ref, team_angle, ((idx_team + 2) % 6) + 1);
    /* send LOC instead of ILM so the bot will update external team as well */
    // printf("%d before UPD with force hop %d\n", my_ID, force_hop);
    // printf("  size at hop %d is %d and at 3 is %d\n", force_hop, ext_connections[force_hop].size, ext_connections[3].size);
    if (ext_connections[force_hop-1].size > 0) {
      memset(tmp_s, 0, 37+1);
      sprintf(tmp_s, "1%s%s%1d", ext_connections[force_hop-1].ext_ID_s, ext_connections[force_hop-1].ext_leader_ID_s, ext_connections[force_hop-1].size);
      construct_share_with_team_message("UPD", my_ID_s, ext_ID_s, leader_ID_s, "000", tmp_s);
      wb_emitter_send(emitter_bt, message, strlen(message) + 1);
    
      // printf("%d sending crazy UPD %s\n", my_ID, message);
    }
    
    construct_inform_location_message(my_ID_s, ext_ID_s, leader_ID_s, TTL, x_ref, y_ref, team_angle, force_hop);
    // wb_emitter_send(emitter_bt, message, strlen(message) + 1);
    printf("%d sent ILM first %s\n", my_ID, message);
    
    // memset(tmp_s, 0, 37+1); 
    // sprintf(tmp_s, "%s%s%07.3f%07.3f%1d", ext_ID_s, my_ID_s, x_ref, y_ref, force_hop);
    // construct_share_with_team_message("LOC", ext_connections[((force_hop + 2) % 6)].ext_leader_ID_s, ext_connections[((force_hop + 2) % 6)].ext_ID_s, team_angle_s, "000", tmp_s);  // vals[0], vals[1], team_bearing);
    // printf("%d sending LOC also %s\n", my_ID, message);
    // printf("  roba iniziale %d and %s\n", ((force_hop + 2) % 6) + 1, ext_connections[((force_hop + 2) % 6) + 1].ext_ID_s);
    force_hop = 0;
  }
  else {
    for (i=1; i<=idx_team; i++) {
      // printf("%d looking spot for %s at i = %d which returns %d\n", my_ID, ext_ID_s, i-1, team_IDs[i-1]);
      if (team_IDs[i-1] == 0 || team_IDs[i-1] == atoi(ext_ID_s)) {
        break;
      }
    }
    construct_inform_location_message(my_ID_s, ext_ID_s, leader_ID_s, TTL, x_ref, y_ref, team_angle, i);
  }
  printf("%d sending ILM %s\n", my_ID, message);
  wb_emitter_send(emitter_bt, message, strlen(message) + 1);
}

////////////////////////////////////////////
// RSM - Reset Simulation Module
////////////////////////////////////////////

void reset_simulation(void){
  /* Position stuff */
  double translation[3] = {0.0, 0.0, 0.0};
  field = wb_supervisor_node_get_field(node, "translation");
  translation[0] = (float)rand() / (float)(RAND_MAX / (0.92 * FLOOR_SIZE)) - (0.46 * FLOOR_SIZE);
  translation[1] = 0.0;
  translation[2] = (float)rand() / (float)(RAND_MAX / (0.92 * FLOOR_SIZE)) - (0.46 * FLOOR_SIZE);
  // translation[0] = (float)rand() / (float)(RAND_MAX / (0.92 * 2)) - (0.46 * 2);
  // translation[1] = 0.0;
  // translation[2] = (float)rand() / (float)(RAND_MAX / (0.92 * 2)) - (0.46 * 2);
  wb_supervisor_field_set_sf_vec3f(field, translation);
  double rotation[4] = {0.0, 1.0, 0.0, 0.0};
  field = wb_supervisor_node_get_field(node, "rotation");
  rotation[3] = (float)rand() / (float)(RAND_MAX / 6.28319);
  wb_supervisor_field_set_sf_rotation(field, rotation);
  
  /* General stuff */
  leader_ID = my_ID;
  sprintf(leader_ID_s, "%03d", my_ID);
  team_player = false;
  leader = true;
  // double *values = (double* )wb_gps_get_values(gps);
  // double x = values[2];
  // double y = values[0];
  x_ref = translation[2];//x;
  y_ref = translation[0];//y;
  // if (my_ID == 1)
    // printf("%d x_ref: %f y_ref: %f\n", my_ID, x_ref, y_ref);
}

////////////////////////////////////////////
// RMM - Randomize Movement Module
////////////////////////////////////////////

void RandomizeMovementModule(void) {
  if (num > rand_num) {
    if (num > rand_num + turn) {
      num = 0;
      turn = (1.0 + ((float)(rand() % 300) / 1.0));
      rand_num = ((float)rand() / (float)(RAND_MAX / (100 * FLOOR_SIZE))); // + (rand() % 10 * atoi(wb_robot_get_name() + 5) + 1);
    }
    else {
      // printf("%d num %d \n", my_ID, num);
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
  // int printed = 0;
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
  rand_num = ((float)rand() / (float)(RAND_MAX / (100 * FLOOR_SIZE))); // + (10 * atoi(wb_robot_get_name() + 5));
  num = 0;
  turn = (1.0 + ((float)(rand() % 300) / 1.0));
  
  /* BT Comms */
  emitter_bt = wb_robot_get_device("bt_e");
  wb_emitter_set_channel(emitter_bt, COMMUNICATION_CHANNEL_BT);
  wb_emitter_set_range(emitter_bt, 11);
  
  receiver_bt = wb_robot_get_device("bt_r");
  wb_receiver_enable(receiver_bt, TIME_STEP);
  
  /* NFC Comms */
  emitter_nfc = wb_robot_get_device("nfc_e");
  wb_emitter_set_channel(emitter_nfc, COMMUNICATION_CHANNEL_NFC);
  wb_emitter_set_range(emitter_nfc,  0.06);
  
  receiver_nfc = wb_robot_get_device("nfc_r");
  wb_receiver_enable(receiver_nfc, TIME_STEP);
  
  /* LEDs */
  for (i = 0; i < NB_LEDS; i++) {
    sprintf(name, "led%d", i);
    leds[i] = wb_robot_get_device(name); /* get a handler to the sensor */
  }
  leddl = wb_robot_get_device("ledd");
  wb_led_set(leddl, 1);
  
  /* GPS and compass */
  gps = wb_robot_get_device("gps");
  wb_gps_enable(gps, TIME_STEP);
  
  compass = wb_robot_get_device("compass");
  wb_compass_enable(compass, TIME_STEP);
  
  /* Initialization and initial reset*/
  my_ID = atoi(wb_robot_get_name() + 5);
  sprintf(my_ID_s, "%03d", my_ID);
  
  const WbNodeRef root_node = wb_supervisor_node_get_root();
  const WbFieldRef root_children_field = wb_supervisor_node_get_field(root_node, "children");
  const int n = wb_supervisor_field_get_count(root_children_field);
  
  const char* namee;
  for (i = 0; i < n; i++) {
    node = wb_supervisor_field_get_mf_node(root_children_field, i);
    namee = wb_supervisor_node_get_type_name(node);
    if (strncmp(namee, "E-puck", 7) == 0){
      node = wb_supervisor_field_get_mf_node(root_children_field, i);
      field = wb_supervisor_node_get_field(node, "name");
      const char* idd = wb_supervisor_field_get_sf_string(field);
      if (atoi(idd + 5) == my_ID){ // we found this robot
        // reset_simulation();
        // if (my_ID == 1)
          // printf("%d reset 1\n", my_ID);
        break;
      }
    }
  } 
  
  // printf("%d after reset ref: x %f y %f\n", my_ID, x_ref, y_ref);
  
  /* File setup */
  // sprintf(filename, "Times%d_%d.txt", run+1, my_ID);
  // fpt = fopen(filename, "w");
  // if (strncmp(wb_robot_get_name(), "epuck1", 7) == 0){
    // printf("Run%d Size:%.1f\n", run, FLOOR_SIZE);
    // fprintf(fpt, "Run%d Size:%.1f\n", run, FLOOR_SIZE);
  // }
  
  /* Main Loop */    
  while(wb_robot_step(TIME_STEP) != -1 && wb_robot_get_time() < (10800 * 5)) {  // Main loop - 5 times for max 3 hours each
    
    // if (my_ID == 1)
      // printf("---   ---   %d   ---   ---\n", atoi(wb_robot_get_name() + 5));
    // printf("---\n");
    
    /* Reset Simulation */
    if (wb_robot_get_time() >= (10800 * run)){
      reset_simulation(); // small probability of starting on the same point
      run += 1;
      if (strncmp(wb_robot_get_name(), "epuck1", 7) == 0){
        printf("\nRun%d Size:%.1f\n", run, FLOOR_SIZE);
        // fprintf(fpt, "\nRun%d Size:%.1f\n", run, FLOOR_SIZE);
      }
    }

    /* Read Sensors Value */
    for (i = 0; i < NB_DIST_SENS; i++){
      ps_value[i] = (((int)wb_distance_sensor_get_value(ps[i]) - ps_offset[i]) < 0) ?
                      0 :
                      ((int)wb_distance_sensor_get_value(ps[i]) - ps_offset[i]);
    }
    
    // if (my_ID == 9) 
      // printf("----- relay %d -----\n", relay);
      // printf(" ---  ext %s  --- \n", ext_connection.ext_leader_ID_s);
    
    /* Initial Speed */
    speed[LEFT] = 200;
    speed[RIGHT] = 200;

    /* Obastacle Avoidance */
    ObstacleAvoidanceModule();
    
    /* Randomize Movement */
    if (!team_player) {
      RandomizeMovementModule();
    }
    
    /* Communication */
    /* Communication is execute one step in advance as we need time to */
    /* process the data which will be delivered to the other robot */
    /* one step in the future */
    
    if (confirmation > 4 && !leader) {
      reset_team();
    }
    
    /* Check for new messages and process them */
    while (wb_receiver_get_queue_length(receiver_bt) > 0) {
      const char *buffer = wb_receiver_get_data(receiver_bt);
      // if (my_ID == 2 || my_ID == 12)
        // printf("%d xhandling message %s\n", my_ID, buffer);
      handle_message((char*)buffer);
      wb_receiver_next_packet(receiver_bt);      
    }
    
    /* Initial message exchange - to be repeated every step to engage new bots */ 
    if ((!(leader && idx_team > 3) || (!relay))) {
      if ((!leader && !whitelisting) || leader) {
        construct_discovery_message(my_ID, team_player, leader, idx_team, leader_ID_s); // saves the desired string in variable message      
        wb_emitter_send(emitter_bt, message, strlen(message) + 1);
        // if (my_ID == 9) {
          // printf("9 sending DNB with team_idx: %d\n", idx_team);
        // }
      }
    }
    
    if (team_collision_count > 0 && global_leader && (team_collision_time) > 15 && !global_rotation) {
      // printf("%d team_collision_time %d and by 15 is %d\n", my_ID, team_collision_time, team_collision_time % 15);
      tmp = 0;
      angle = 0;
      for (i=0; i<team_collision_count; i++) {
        if (team_collision_ID[i] > 0) {
          team_collision_ID[i] = 0;
        }
        if (team_collision_angle[i] > 0) {
          angle += team_collision_angle[i];
          tmp += 1;
          team_collision_angle[i] = 0;
        }
      }
      if (angle != 0) {
        printf("  angle: %d, tmp: %d\n", (int)(angle), tmp);
        printf("  calculations: angle/tmp %d, divide by 360: %d\n", (int)(angle / tmp), ((int)(angle / tmp) + 180) % 360);
        angle = ((int)(angle / tmp) + 180) % 360;
        sprintf(team_angle_s, "%03d", (int)angle);
        team_angle = atoi(team_angle_s);
        memset(tmp_s, 0, 37+1);
        sprintf(tmp_s, "002%03d", team_angle);
        construct_share_with_team_message("MOV", my_ID_s, "000", "000", "007", tmp_s);
        wb_emitter_send(emitter_bt, message, strlen(message) + 1);
        printf("%d team collision, updating direction\n", my_ID);
        printf("  new angle %d\n", (int)angle);
        printf("  sent MOV 2 as %s\n", message);
        // maybe send a message to the interested bots so they know
        global_rotation = true;

        // To Do
      }
      team_collision_time = 0;
      team_collision_count = 0;
    }
    
    if (team_collision_time > 0 && global_leader) {
      team_collision_time += 1;
    }
    
    // if (my_ID == 14 && confirmation != -1)
      // printf("%d confirmation %d\n", my_ID, confirmation);
      
    if (confirmation <= 3 && confirmation >= 0) {
      confirmation += 1;
    }
    
    
    // if (my_ID == 14 && confirmation != -1)
      // printf("%d confirmation %d\n", my_ID, confirmation);
    
    // if (ext_connection.size > 0) {
      // relay = true;
    // }
    
    // if (my_ID == 9) {
      // printf("--- %d ext size: %d ext ID: %d\n", my_ID, ext_connection.size, atoi(ext_connection.ext_ID_s));
    // }
    
    if (team_player) {
      if (leader)
        wb_led_set(leddl, 2);
      else if (relay)
        wb_led_set(leddl, 4);
      else
        wb_led_set(leddl, 3);
    } 
    else {
      if (leader)
        wb_led_set(leddl, 2);
      else
        wb_led_set(leddl, 1);
    }        
      
    /*
    
    There are multiple cases and ways we can move:
    - If we are joining a team, we move to that direction. When we 
      are close to our goal, we move following the leader
    - If we are the follower in a simple scenario, we follow the 
      leader and the team
    - If we are the leader, we move following our direction
    
    */
    
    /* First of all we need to check whether we are requested to move somewhere because we are joining a team. */
    if (location_change && !in_line && !oam_active) {
      /* First we rotate */
      handle_rotation(-1.0);
      printf("%d rotating\n", my_ID);
      /* Reset flag */
    }    
    /* Once we are in line for moving to the goal location, we can start move */
    else if (dist_counter < 2 && dist_to_goal > 0.1 && in_line) {
      
      speed[LEFT] = 150;
      speed[RIGHT] = 150;
      dist_to_goal -= 1.0/829;
      if (dist_to_goal < (backup / 3) + 1 && dist_to_goal > (backup / 3) - 1) {
        double *values = (double* )wb_gps_get_values(gps);
        x = values[2];
        y = values[0];
        
        my_x = x - x_goal; 
        my_y = y - y_goal;
        
        dist_to_goal = sqrtf(my_x*my_x + my_y*my_y);
        backup = dist_to_goal;
        
        angle = atan2(my_y, my_x);
        if (my_y < 0) {
          angle = 360 + (angle * 180 / M_PI);
        }
        else {
          angle = angle * 180 / M_PI;
        }
        // printf("%d first time here\n", my_ID);
     
        angle_compass = (get_bearing_in_degrees(compass));
        if (angle_compass > 360)
          angle_compass -= 360;
      
        diff_angle = angle - angle_compass; //fmod(fabs(angle - angle_compass), 360); // we lose the sign
        if (fabs(diff_angle) > 180) {
          diff_angle = sign(diff_angle) * (360.0 - fabs(diff_angle)) * (-1);
        }
      
        if (fabs(diff_angle) > 2) {
          handle_rotation(-1);
          // printf("%d re-rotating\n", my_ID);
          // printf("  diff_angle: %f\n", diff_angle);
        }
        else {
          printf("%d updating dist\n", my_ID);
          dist_counter += 1;
          // send alive message
          memset(tmp_s, 0, 37+1);
          sprintf(tmp_s, "0%1d%1d%03d", my_team_idx, idx_team, (int)round(dist_to_goal));
          construct_share_with_team_message("UPD", my_ID_s, "000", leader_ID_s, "000", tmp_s);
          wb_emitter_send(emitter_bt, message, strlen(message) + 1);
          printf("%d sending out UPD as %s\n", my_ID, message);
          
        }
        // in_line = false;
      }
        
    }
    else if (dist_counter >= 2 && dist_to_goal > 0.1 && in_line) {
      speed[LEFT] = 150;
      speed[RIGHT] = 150;
      /* 53:56 / 16 = 53056 / 64 = 829 steps to move 1 meter */
      dist_to_goal -= 1.0/829; //3317.0;
    }
    else if (location_change && dist_to_goal < 0.1 && dist_to_goal > 0.0) {
      /* arrived */
      speed[LEFT] = 0.0;
      speed[RIGHT] = 0.0;
      
      angle_compass = get_bearing_in_degrees(compass);
      diff_angle = team_angle - angle_compass;
      if (fabs(diff_angle) > 180) {
        diff_angle = sign(diff_angle) * (360.0 - fabs(diff_angle)) * (-1);
      }
      
      if (fabs(diff_angle) > 2 && global_rotation == 1) {
        handle_rotation(team_angle);
        printf("%d rotating 1\n", my_ID);
      }
      // else if (angle_update == true) {
        // angle_update = false;
        // location_change = false;
        // dist_to_goal = -1.0;
      // }
      else {
        global_rotation = 0;
        // printf("%d done rotation 1\n", my_ID);
        /* If external conenction exists */
        memset(tmp_s, 0, 37+1);
        if (ext_connection.size > 0 && team_player) {
          /* send a message that I'm ready but my ext_team might not */
          sprintf(tmp_s, "%03d100", my_team_idx);
          construct_share_with_team_message("ARR", my_ID_s, leader_ID_s, leader_ID_s, "000", tmp_s);
        
        }
        else if (ext_connection.size > 0 && !team_player) {
          /* single bot team */
          sprintf(tmp_s, "000111");
          construct_share_with_team_message("ARR", my_ID_s, "000", leader_ID_s, "001", tmp_s);
                 
        }
        else {
          /* send a message that I'm totally ready */
          sprintf(tmp_s, "%03d011", my_team_idx);
          construct_share_with_team_message("ARR", my_ID_s, leader_ID_s, leader_ID_s, "000", tmp_s);
        }
        
        if (leader) {
          check_team_arrival();
          arrival_update = false;
        }
        
        printf("%d sending ARR %s\n", my_ID, message);
        wb_emitter_send(emitter_bt, message, strlen(message) + 1);
        location_change = false;
        dist_to_goal = -1.0;
        
        if (printed == 0) {
          double *values = (double* )wb_gps_get_values(gps);
          x = values[2];
          y = values[0];
        
          my_x = x - x_goal; 
          my_y = y - y_goal;
        
          printf("%d arrived\n", my_ID);
          printf("  leader %d, my idx %d, leader? %d\n", leader_ID, my_team_idx, leader);
          printf("  fake error cm %f\n", dist_to_goal*100);
          printf("  actual error cm %f\n", sqrtf(my_x*my_x + my_y*my_y)*100);
          printf("  reference x %f y %f\n", x_ref, y_ref);
          printf("  goal x %f y %f\n", x_goal, y_goal);
          printf("  real x %f y %f\n", x, y);
          printf("  my x %f y %f\n", my_x, my_y);
          printf("  ext: %s, ext_leader: %s, ext_size: %d\n", ext_connection.ext_ID_s, ext_connection.ext_leader_ID_s, ext_connection.size);
          if (leader) {
            printf("  leader 0 - ext: %s, ext_leader: %s\n", ext_connections[0].ext_ID_s, ext_connections[0].ext_leader_ID_s);
            printf("  leader 1 - ext: %s, ext_leader: %s\n", ext_connections[1].ext_ID_s, ext_connections[1].ext_leader_ID_s);
            printf("  leader 2 - ext: %s, ext_leader: %s\n", ext_connections[2].ext_ID_s, ext_connections[2].ext_leader_ID_s);
            printf("  leader 3 - ext: %s, ext_leader: %s\n", ext_connections[3].ext_ID_s, ext_connections[3].ext_leader_ID_s);
            printf("  leader 4 - ext: %s, ext_leader: %s\n", ext_connections[4].ext_ID_s, ext_connections[4].ext_leader_ID_s);
            printf("  leader 5 - ext: %s, ext_leader: %s\n", ext_connections[5].ext_ID_s, ext_connections[5].ext_leader_ID_s);
          }
          // printed = 1;
        }
        
      }
    }
    else if (leader && !location_change && waiting_new_bot) {
      speed[LEFT] = 0.0;
      speed[RIGHT] = 0.0;
      if (arrived_folls == idx_team && arrival_update) {
        check_team_arrival();
        arrival_update = false;
      }
      // if (my_ID == 6)
        // printf("%d here 1\n", my_ID);
    }
    /* Last case, we move randomly */
    else if (!team_player && !location_change && !global_movement) {
      RandomizeMovementModule();
      // if (my_ID == 6)
        // printf("%d strange here 2\n", my_ID);
    }
    else if (team_player && !location_change && !leader && global_movement == 0) {
      speed[LEFT] = 0.0;
      speed[RIGHT] = 0.0;
      // if (my_ID == 6)
        // printf("%d here 3\n", my_ID);
    }
    else if (global_movement == 1 && global_rotation == 0) {
      if (printed == 0) {
        printf("%d global movement ---\n", my_ID);
        printed = 1;
      }
      speed[LEFT] = 150.0;
      speed[RIGHT] = 150.0;
    }
    // else if (global_rotation == 1) {
      // printf("%d gloab rotation ---\n", my_ID);
      // angle_compass = get_bearing_in_degrees(compass);
      // diff_angle = team_angle - angle_compass;
      // if (fabs(diff_angle) > 180) {
        // diff_angle = sign(diff_angle) * (360.0 - fabs(diff_angle)) * (-1);
      // }
      // if (fabs(diff_angle) > 2) {
        // handle_rotation(team_angle);
      // }
      // else {
        // global_rotation = 0;
      // }
    // }
    else {
      // printf("%d che cazz %d global_mov %d\n", my_ID, waiting_new_bot, global_movement);
      // if (my_ID == 6)
        // printf("%d here 4\n", my_ID);
      if (global_rotation == 1) {
        // printf("%d gloab rotation ---\n", my_ID);
        angle_compass = get_bearing_in_degrees(compass);
        diff_angle = team_angle - angle_compass;
        if (fabs(diff_angle) > 180) {
          diff_angle = sign(diff_angle) * (360.0 - fabs(diff_angle)) * (-1);
        }
        if (fabs(diff_angle) > 2) {
          handle_rotation(team_angle);
          printf("%d rotating 2 left %f\n", my_ID, fabs(diff_angle));
        }
        else {
          global_rotation = 0;
          printf("%d done rotation 2\n", my_ID);
        }
      }
      else {
        speed[LEFT] = 0.0;
        speed[RIGHT] = 0.0;
      }
    }
    
    wb_motor_set_velocity(left_motor, 0.00628 * speed[LEFT]);
    wb_motor_set_velocity(right_motor, 0.00628 * speed[RIGHT]);
    
    // printf("%d team has %d bots:\n", my_ID, idx_team);
    // for (i=0; i<idx_team-1; i++) {
      // printf("%d, ", team_IDs[i]);
    // }
    // if (idx_team > 0)
      // printf("%d\n", team_IDs[idx_team-1]);
    
  }
  
  fclose(fpt);

  return 0;
}



// LIM - location information message
// needs to be sent to each robot that joins
// it contains leader's coordinates and the number of the idx of the bot
// it also contains the spots left
// - we do a round robin assignment, each bot collects it's spot and the others will follow
// - we can also consider using the leader as the central brain which calculates the best route
// - we add new bots in a certain order