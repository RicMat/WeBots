// To Do: 
// - Add a control so there cannot be two team inglobations/joins at the same time
//   e.g. robot_1 of team_3 meets robot_2 of team_1, robot_12 of team_3 meets robot_3 of team_2
//   in this case we'd have two robots trying to move team_3 towards team_1 and team_2
// - TTT when I join a new team, I receive the list of all the other bots? however, when I do the transfer my other team will discard all the messages containing the new bots
// - If there are obstacles, we try to avoid them. If it's not possible, we redirect the blocked robot somewhere else
//   Consider also narrow passages or stuff like that
// - Work with compass and RSSI (RSSI to understand the distance and compass for inter-team interactions) 
// - Use RSSI and number of collisions to expand in the best direction (lower aggregation)

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
#define FLOOR_SIZE 100.0 
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
int ps_offset[NB_DIST_SENS] = {0, 0, 0, 0, 0, 0, 0, 0}, i, speed[2];
int rand_num, num, my_ID, counter = 0, run = 0;

/* Basic comms */
char code_in[4], ext_ID_s[4], my_ID_s[4], receiver_ID_s[4], leader_ID_s[4], ext_leader_ID_s[4];
char message[34+1], ttl_s[4], extra[22+1], tmp_s[34+1], tmp_ss[4], ext_team_size_s[3+1];
char ext_team_player, ext_leader;
bool leader = false, team_player = false, in_queue = false, duplicate_message = false;
int leader_ID = 0, ext_leader_ID = 0, idx_comms = 0, idx_team = 0, ext_ID = 0, ext_team_size;
int receiver_ID = 0, tmp = 0, ttl = 0;
int team_IDs[7];

/* Location */
char x_ref_s[7+1], y_ref_s[7+1], my_x_s[7+1], my_y_s[7+1], my_team_idx_s[2];
float x_ref, y_ref, my_x, my_y, x_goal, y_goal, bearing, team_bearing, angle, angle_compass, diff_angle;
double x, y;
int my_team_idx = 0; // my role in the team
bool location_change = false, new_bot = false, waiting_new_bot = false;

/* Stored messages - Could use an Hash Table */
char messages_lookback[7][SIZE][34+1];
int messages_lookback_size[7] = { 0 };

/* External connections */
// char ext_connection_ID_s[3+1];
// int ext_connection_ID, ext_connection_leader_ID;
float vals[2];
bool ext_connection_existing = false;
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
}

////////////////////////////////////////////
// MHM - Message Handling Module
////////////////////////////////////////////

void append_bot(char* bot_ID) {
  /*
  
  Here we add the bot to our team queue
  We first check if the bot is already in the queue and if it's, we discard it
  
  */
  in_queue = false;
  
  for (i=0; i<idx_team; i++) {
    if(team_IDs[i] == atoi(bot_ID) || atoi(bot_ID) == my_ID) {
      in_queue = true; /* discard this communication */
      break;
    }
  }
  
  if (!in_queue) {
    team_IDs[idx_team] = atoi(bot_ID);
    idx_team += 1;
  }
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
  leader = false;
  
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
  
}

void inglobate_external_team(char* ext_ID_s, char* TTL) {
  /*
  
  We are sharing the info about the newly added robot with our team
  If we are the leader we should share the message with all our team
  If we are not the leader we should share the info with our leader
  
  */
  if (leader) {
    /* TTL set to 0 as it should reach every other bot in the team in a single hop */
    construct_share_with_team_message("ITM", my_ID_s, "000", leader_ID_s, "000", ext_ID_s);     
  }
  else {
    /* TTL set to 3 as it should be enough to travel the whole team */
    construct_share_with_team_message("ITM", my_ID_s, "000", leader_ID_s, "003", ext_ID_s);
  }
  wb_emitter_send(emitter_bt, message, strlen(message) + 1);
  
  /* Share the info about our team */
  memset(tmp_ss, 0, 3+1);
  sprintf(tmp_ss, "%03d", my_ID); 
  construct_share_with_team_message("ITM", my_ID_s, "000", leader_ID_s, "007", tmp_ss);
  wb_emitter_send(emitter_bt, message, strlen(message) + 1);
  
  for (i=0; i<idx_team; i++) {
    memset(tmp_ss, 0, 3+1);
    sprintf(tmp_ss, "%03d", team_IDs[i]); 
    construct_share_with_team_message("ITM", my_ID_s, "000", leader_ID_s, "007", tmp_ss);
    wb_emitter_send(emitter_bt, message, strlen(message) + 1);
  }
         
  /* Save data of the new bot */
  append_bot(ext_ID_s);
  
  /* 
  
  If I'm the leader I can inform the bot of it's location directly
  Otherwise, I simply skip this step because the share_with_team message will eventually
  reach my leader who will in turn share the location to the bot (and we will probably relay it)
  
  */  
  if (leader) {
    inform_new_location(ext_ID_s, TTL);
    waiting_new_bot = true;
  }
  
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
  if (receiver != my_ID && receiver != 0 && receiver != atoi(ext_connection.ext_ID_s))  { 
  // if (strcmp(code_in, "ILM") != 0 && (ext_team_ID != leader_ID && receiver != atoi(ext_connection.ext_ID_s)))  {    
    /////////
    // To Do: Consider hop between teams
    /////////
    // printf("%d %s duplicate receiver %d hop %d\n", my_ID, buffer, receiver, atoi(ext_connection.ext_ID_s));
    // if (strcmp(code_in, "LOC") == 0 && my_ID == 3) {
      // printf("  at point 2 with receiver %d\n", receiver);
    // }
    // if (strcmp(code_in, "ILM") == 0 && my_ID == 5) {
      // printf("--- %d receiver %d\n", my_ID, receiver);
    // }
    return true;
  }
  
  switch(hash_codes(code_in)) // we could make this shorter but this way we reduce the computational time up to 4 times
  {
    case 0: //"DNB"
      for (i=0; i<messages_lookback_size[0]; i++) {
        memset(tmp_s, 0, 34+1);
        strncpy(tmp_s, messages_lookback[0][i]+3, 3); /* check the sender */
        if (atoi(tmp_s) == sender) {
          memset(tmp_s, 0, 34+1);
          strncpy(tmp_s, messages_lookback[0][i]+9, 3); /* check the team_ID */
          if (atoi(tmp_s) == ext_team_ID || ext_team_ID == leader_ID) {
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
        memset(tmp_s, 0, 34+1);
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
        memset(tmp_s, 0, 34+1);
        strncpy(tmp_s, messages_lookback[2][i]+3, 3); /* check the sender */
        if (atoi(tmp_s) == sender) {
          memset(tmp_s, 0, 34+1);
          strncpy(tmp_s, messages_lookback[2][i]+6, 3); /* check the receiver */
          if (atoi(tmp_s) == receiver) {
            memset(tmp_s, 0, 34+1);
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
      for (i=0; i<messages_lookback_size[3]; i++) {
        memset(tmp_ss, 0, 3+1);
        // strncpy(tmp_s, messages_lookback[3][i]+3, 3); /* check the sender */
        // if (atoi(tmp_s) == sender) {
          // memset(tmp_s, 0, 34+1);
          strncpy(tmp_ss, messages_lookback[3][i]+9, 3); /* check the team_ID */
          // printf("team_ID %d and %d\n", atoi(tmp_ss), ext_team_ID);
          if (atoi(tmp_ss) == ext_team_ID) {
            memset(tmp_ss, 0, 3+1);
            strncpy(tmp_ss, messages_lookback[3][i]+15, 3); /* check the rest */
            // printf("bot %s and %s\n", tmp_ss, extra);
            if (strcmp(tmp_ss, extra) == 0) {
              // printf("--- duplicate\n");
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
      if (sender != leader_ID) {
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
            memset(tmp_s, 0, 34+1);
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
      for (i=0; i<messages_lookback_size[5]; i++) {
        memset(tmp_ss, 0, 3+1);
        strncpy(tmp_ss, messages_lookback[5][i]+18, 3); /* check the actual sender (+3 would have been the hop) */
        char actual_sender[3+1];
        strncpy(actual_sender, buffer+18, 3);
        // printf("%d checking tmp: %s actual sender: %s\n", my_ID, tmp_ss, actual_sender);
        if (atoi(tmp_ss) == atoi(actual_sender)) {
          
          ///////
          //
          // To Do: Inform the sender that the bot is already known so he doesn't need to be a hop for that bot
          //
          ///////
          // printf("problem 1\n");
          return true;
         
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
      if (receiver != my_ID) {
        return true;
      }
      to_relay = false;
      memset(tmp_ss, 0, 3+1);
      strncpy(tmp_ss, buffer+15, 3);
      // printf("%d checking LOC for relay: tmp_ss %s my_ID %d\n", my_ID, tmp_ss, my_ID);
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
          memset(tmp_s, 0, 34+1);
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
    default:
      printf("%d default %s\n", my_ID, buffer);
      return false;
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
  for (i=0; i<idx_team; i++) {
    if (team_IDs[i] == atoi(ext_ID_s) || atoi(ext_connections[i].ext_ID_s) == atoi(ext_ID_s)) { 
      printf("EBR overflow\n");
      return;
    }
  }
  
  
  // while it's moving we should discard it's comms
  
  int out;
  
  ext_connection_existing = false;
  
  if (ext_connections[atoi(hop_team_idx)].size > 0) {
    printf("%d step 1\n", my_ID);
    ext_connection_existing = true; /* we have already a team near that position*/
  }
  
  if (ext_connection_existing) {
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
    
      out = ((atoi(hop_team_idx) + 2) % 6) + 1;
      handle_position_f(my_x, my_y, out, false);
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
        out = atoi(hop_team_idx);
        printf("%d step 5n", my_ID);
      }
      handle_position_f(x_ref, y_ref, out, false);
    }  
  }
  else {
  /* I have the ext_team_size so I can guess the needed location
  However, we can directly send the location of the leader and let the ext team handle it */ 
    
    printf("%d step 6\n", my_ID);
    out = retrieve_closest_external_connection(atoi(hop_team_idx), ext_team_size);
    printf("  out: %d\n", out);
    if (out == -1) {
      printf("%d step 7\n", my_ID);
      out = ((atoi(hop_team_idx) + 2) % 6) + 1;
    }
    vals[0] = x_ref;
    // printf("%d testtttt x_goal %f y_goal %f\n", my_ID, x_ref, y_ref);
    vals[1] = y_ref;
    for (i=0; i<3; i++) { 
      handle_position_f(vals[0], vals[1], out, false);
    }
    // printf("%d vals are: %f %f\n", my_ID, vals[0], vals[1]);
    // handle_position_f(vals[0], vals[1], out, false);
    // printf("%d vals are: %f %f\n", my_ID, vals[0], vals[1]);
  }
  
  printf("%d leader storing ext conn %s\n", my_ID, ext_ID_s);
  
  leader_store_external_connection(out, ext_ID_s, ext_leader_ID_s, ext_team_size);
  memset(tmp_s, 0, 34+1); 
  
  // hop_team_idx is prolly wrong
  
  sprintf(tmp_s, "%s%07.3f%07.3f%1d", hop_ID_s, vals[0], vals[1], out);
  // memset(tmp_ss, 0, 3+1);
  // sprintf(tmp_ss, "%1d", out);
  // strcat(tmp_s, tmp_ss);
  
  // We send a LOC message from leader to excess bot with the ID of the hop as well as the location and the 
  construct_share_with_team_message("LOC", my_ID_s, ext_ID_s, leader_ID_s, "001", tmp_s);  // vals[0], vals[1], team_bearing);
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
  
  // if (strcmp(code_in, "LOC") == 0) {
    // printf("%d received LOC with code in %d\n", my_ID, hash_codes(code_in));
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
  if (ttl > 0 && receiver_ID != my_ID && (!duplicate_message || strcmp(code_in, "LOC") == 0 || strcmp(code_in, "ILM") == 0)) {
    /* relay the message if needed */
    // if (strcmp(code_in, "LOC") == 0) {// && my_ID == 3) {
      // printf("%d relayed %s\n", my_ID, buffer);
    // }
    memset(message, 0, 34+1);
    strncpy(message, buffer, 12);
    sprintf(tmp_s, "%03d", ttl-1);
    strcat(message, tmp_s);
    strcat(message, buffer+15);
    wb_emitter_send(emitter_bt, message, strlen(message) + 1);
  }
  
  // if (strcmp(code_in, "ILM") == 0) {
    // printf("%d received ILM from %d: %s\n", my_ID, ext_ID, buffer);
    // printf("  duplicate: %d\n", duplicate_message);
  // }
  
  if (strcmp(code_in, "LOC") == 0) {
    memset(tmp_ss, 0, 3+1);
    strncpy(tmp_ss, buffer+15, 3);
    if (atoi(tmp_ss) == my_ID) {
      relay = true;
      store_external_connection(receiver_ID_s, ext_leader_ID_s, ext_team_size);
    }
  }
  
  if (duplicate_message) {
    return;
  }
  
  
  
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
      
      ////
      //
      // To Do: If I'm the leader (e.g. at the real beginning there are 8 bots) I should declare a bot in charge of it
      //
      ////
      
      in_queue = false;
  
      for (i=0; i<idx_team; i++) {
        if(team_IDs[i] == ext_ID || ext_ID == my_ID) {
          in_queue = true; /* discard this communication */
          break;
        }
      }
      
      // if (ext_ID == 10)
        printf("%d received DNB from %d\n", my_ID, ext_ID);
        
      if (ext_team_size + idx_team + 1 > 7 && !in_queue) { // idx_team starts from 0, so the number of bots is actually idx_team + 1
        // if (my_ID == 1) {
          // printf("%d size overflow\n", my_ID);
          // printf("%d message %s\n", my_ID, buffer);
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
              memset(tmp_s, 0, 34+1);
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
        
        // store_external_connection(ext_ID_s, ext_leader_ID_s, ext_team_size);
        // relay = true;
        break;
      }
      
      if (!team_player) { // at this point for sure I'll either join or inglobate at least anothe robot
        team_player = true;
      }
      
      new_bot = true; // for debug
      
      /* check who has the lowest ID to determine who is the leader */
      if (leader_ID < ext_leader_ID) {
        /* external bot will join my team */
        // if (ext_ID == 10)
          // printf("%d inglobating 10\n", my_ID);
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
        join_external_team(ext_ID_s, ext_leader_ID_s);
      }
      break;
    /*
    
    Variable:
    
    S - last_queued - Y if this is the last bot in queue of the external bot
    
    */ 
    case 1: //"LJT" - List of other team-members to Join my Team
      /*
      
        There are two cases here:
        - If we are the leader we should save the info of the new bot
        and, at the same time, transmit a message to this new robot and containing
        his location
        - If we are not the leader we simply save the info about the new bot
      
      */ 
      strncpy(tmp_ss, buffer+15, 3);
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
      // if (receiver_ID != my_ID)
        // break;
      memset(tmp_ss, 0, 3+1);
      strncpy(tmp_ss, buffer+15, 3);
      // printf("%d received ITM for %d\n", my_ID, atoi(tmp_ss));
      if (atoi(tmp_ss) != my_ID) {
        append_bot(tmp_ss);
      }
      
      if (leader && !in_queue) {
        // printf("%d sending location to %d\n", my_ID, atoi(tmp_ss));
        inform_new_location(tmp_ss, "006"); // TTL set to 7 as the worst case
        if (atoi(tmp_ss) == 10)
          printf("%d sending location to 10\n", my_ID);
        waiting_new_bot = true;
      }
    break;
    /*
    
    Variable:
    
    SSSSSSS - x_ref_s - x coordinate of the leader
    SSSSSSS - y_ref_s - y coordinate of the leader
    S       - my_team_idx_s - ID given to me by the leader; based on this value I'll move to a certain location
    
    */
    case 4: // "ILM" - Inform of new Location Message
      if (my_ID == 10)
        printf("%d received ILM from %d: %s\n", my_ID, ext_ID, buffer);
      memset(x_ref_s, 0, 7+1);
      strncpy(x_ref_s, buffer+15, 7);
      x_ref = atof(x_ref_s);
      memset(y_ref_s, 0, 7+1);
      strncpy(y_ref_s, buffer+22, 7);
      y_ref = atof(y_ref_s);
      memset(my_team_idx_s, 0, 1+1);
      strncpy(my_team_idx_s, buffer+29, 1);
      my_team_idx = atoi(my_team_idx_s);
      if (leader) {
        break; /* shouldn't be a possibility */
      }
      else {
        handle_position_f(x_ref, y_ref, my_team_idx, true);
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
      // printf("  hop_ID_s %s hop_team_idx %s ext_ID_s %s \n", hop_ID_s, hop_team_idx, ext_ID_s);
      handle_excess_bot(hop_ID_s, hop_team_idx, ext_ID_s, ext_leader_ID_s, ext_team_size);
      break;
    /*
    
    Variable:
    
    SSSSSSS - 
    SSSSSSS - 
    
    */
    case 6: // "LOC" - Localization Bot Request
      printf("%d received LOC %s\n", my_ID, buffer);
      memset(x_ref_s, 0, 7+1);
      
      
      //
      //
      // Add hop_ID from idx=15 to idx=17
      //
      //
      printf("  my ref was x %f y %f\n", x_ref, y_ref);
      printf("  my goal was x %f y %f\n", x_goal, y_goal);
      memset(hop_ID_s, 0, 3+1);
      strncpy(hop_ID_s, buffer+15, 3);
      strncpy(x_ref_s, buffer+18, 7);
      x_ref = atof(x_ref_s);
      memset(y_ref_s, 0, 7+1);
      strncpy(y_ref_s, buffer+25, 7);
      y_ref = atof(y_ref_s);
      memset(my_team_idx_s, 0, 1+1);
      strncpy(my_team_idx_s, buffer+32, 1);
      my_team_idx = atoi(my_team_idx_s);
      
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
      
      in_queue = false;
      for (i=0; i<idx_team; i++) {
        if (team_IDs[i] == ext_ID) {
          in_queue = true;
        }
      }
      
      if (!in_queue) { /* Coming from outside */
        if (leader) { /* Scenario 1: Update my location */
          printf("%d scenario 1\n", my_ID);
          handle_position_f(x_ref, y_ref, my_team_idx, true);
        }
        else { /* Scenario 2: Inform the leader */
          printf("%d scenario 2\n", my_ID);
          // handle_position_f(x_ref, y_ref, my_team_idx, true);
          memset(tmp_s, 0, 34+1);
          /* 
          The leader will receive the message and he'll use the idx = 0
          However, we send the idx of our team's hop so the leader can save him there
          */
          sprintf(tmp_s, "%s%07.3f%07.3f%1d", hop_ID_s, x_ref, y_ref, my_team_idx);
          construct_share_with_team_message("LOC", my_ID_s, leader_ID_s, leader_ID_s, "001", tmp_s);  // vals[0], vals[1], team_bearing);
          wb_emitter_send(emitter_bt, message, strlen(message) + 1);
          relay = true;
        } 
      }
      else { /* Scenario 3: Coming from my follower; Update my location and inform my followers */
        printf("%d scenario 3\n", my_ID);
        printf("  my idx: %d\n", 0);
        handle_position_f(x_ref, y_ref, 0, true);
        
        /* We have to set the hop idx to the correct place */
        
        
        
        
        
        
        
        
        
        // We can do like: for i=0, i<7, i++ if team_IDs != -1 do stuff
        
        
        
        
        
        
        
        
        
        
        
        
        for (i=0; i<idx_team; i++) {
          memset(tmp_ss, 0, 3+1);
          sprintf(tmp_ss, "%03d", team_IDs[i]);
          construct_inform_location_message(my_ID_s, tmp_ss, leader_ID_s, "000", x_ref, y_ref, i+1);
          printf("  sending to %s: %s\n", tmp_ss, message);
          wb_emitter_send(emitter_bt, message, strlen(message) + 1);
        }
      }
      
      // relay = true;
      
      // handle_position_f(x_ref, y_ref, my_team_idx, true);
      printf("  my ref became x %f y %f\n", x_ref, y_ref);
      printf("  my goal became x %f y %f\n", x_goal, y_goal);
      
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
    default:
      break;
  }

}

////////////////////////////////////////////
// PHM - Position Handling Module
////////////////////////////////////////////



/////////////////////
//
// To Do: set initial ref position and update it once we get the DNB message
//
/////////////////////




double get_bearing_in_degrees(WbDeviceTag compass) { // no need 90 degree offset cause we are using two different reference planes
  const double *north = wb_compass_get_values(compass);
  double rad = atan2(north[2], north[0]);
  double bearing = (rad) * 180.0 / M_PI;
  
  if (bearing < 0.0)
    bearing = 360 +  bearing;
  
  return bearing;
}

int sign(float number) {
  if (number > 0)
    return 1;
  else
    return -1;
}

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
  
  // double *values = (double* )wb_gps_get_values(gps);
  // double x = values[2];
  // double z = values[1];
  // double y = values[0];
  
  // bearing = get_bearing_in_degrees(compass);

  // recover_ref_pos(x, y, my_team_idx);
  // printf("%d reference location x: %f, y: %f\n", my_ID, x_ref, y_ref);
  sprintf(x_ref_s, "%07.3f", x_ref);
  sprintf(y_ref_s, "%07.3f", y_ref);
  
  construct_inform_location_message(my_ID_s, ext_ID_s, leader_ID_s, TTL, x_ref, y_ref, idx_team);
  wb_emitter_send(emitter_bt, message, strlen(message) + 1);
}

////////////////////////////////////////////
// RSM - Reset Simulation Module
////////////////////////////////////////////

void reset_simulation(void){
  // double translation[3] = {0.0, 0.0, 0.0};
  // field = wb_supervisor_node_get_field(node, "translation");
  // translation[0] = (float)rand() / (float)(RAND_MAX / (0.92 * FLOOR_SIZE)) - (0.46 * FLOOR_SIZE);
  // translation[1] = 0.0;
  // translation[2] = (float)rand() / (float)(RAND_MAX / (0.92 * FLOOR_SIZE)) - (0.46 * FLOOR_SIZE);
  // translation[0] = (float)rand() / (float)(RAND_MAX / (0.92 * 2)) - (0.46 * 2);
  // translation[1] = 0.0;
  // translation[2] = (float)rand() / (float)(RAND_MAX / (0.92 * 2)) - (0.46 * 2);
  // wb_supervisor_field_set_sf_vec3f(field, translation);
  // double rotation[4] = {0.0, 1.0, 0.0, 0.0};
  // field = wb_supervisor_node_get_field(node, "rotation");
  // rotation[3] = (float)rand() / (float)(RAND_MAX / 6.28319);
  // wb_supervisor_field_set_sf_rotation(field, rotation);
  leader_ID = my_ID;
  sprintf(leader_ID_s, "%03d", my_ID);
  team_player = false;
  leader = true;
  double *values = (double* )wb_gps_get_values(gps);
  double x = values[2];
  double y = values[0];
  x_ref = x;
  y_ref = y;
  // printf("%d x: %f x_ref: %f\n", my_ID, x, x_ref);
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
  wb_emitter_set_range(emitter_bt, 10);
  
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
        reset_simulation();
        break;
      }
    }
  } 
  
  // printf("%d after reset ref: x %f y %f\n", my_ID, x_ref, y_ref);
  
  /* File setup */
  sprintf(filename, "Times%d_%d.txt", run+1, my_ID);
  fpt = fopen(filename, "w");
  if (strncmp(wb_robot_get_name(), "epuck1", 7) == 0){
    printf("Run%d Size:%.1f\n", run, FLOOR_SIZE);
    fprintf(fpt, "Run%d Size:%.1f\n", run, FLOOR_SIZE);
  }
  
  /* Main Loop */    
  while(wb_robot_step(TIME_STEP) != -1 && wb_robot_get_time() < (10800 * 5)) {  // Main loop - 5 times for max 3 hours each
    
    // if (my_ID == 1)
      // printf("---   ---   ----   ---   ---\n");
    // printf("---\n");
    
    /* Reset Simulation */
    if (wb_robot_get_time() >= (10800 * run)){
      reset_simulation(); // small probability of starting on the same point
      run += 1;
      if (strncmp(wb_robot_get_name(), "epuck1", 7) == 0){
        printf("\nRun%d Size:%.1f\n", run, FLOOR_SIZE);
        fprintf(fpt, "\nRun%d Size:%.1f\n", run, FLOOR_SIZE);
      }
    }

    // printf("%d my ID, my leader %d\n", my_ID, leader_ID);
    // printf("1 %f\n", (1.0 + ((float)(rand() % 6) / 10.0)));
    // printf("2 %f\n", (1.0 + ((float)(rand() % 600) / 1.0)));
    // printf("3 %f\n", ((float)rand() / (float)(RAND_MAX / (100 * FLOOR_SIZE))));
    // printf("4 %f\n", ((float)rand() / (float)(RAND_MAX / (1000 * FLOOR_SIZE))));
    
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
    if (!team_player) {
      RandomizeMovementModule();
    }
    
    /* Communication */
    /* Communication is execute one step in advance as we need time to */
    /* process the data which will be delivered to the other robot */
    /* one step in the future */
    
    /* Initial message exchange - to be repeated every step to engage new bots */ 
    if (!(leader && idx_team > 3) || (!relay)) {
      construct_discovery_message(my_ID, team_player, leader, idx_team, leader_ID_s); // saves the desired string in variable message      
      wb_emitter_send(emitter_bt, message, strlen(message) + 1);
    }
    
    /* Check for new messages and process them */
    while (wb_receiver_get_queue_length(receiver_bt) > 0) {
      const char *buffer = wb_receiver_get_data(receiver_bt);
      handle_message((char*)buffer);
      wb_receiver_next_packet(receiver_bt);      
    }
    
    if (team_player) {
      if (leader)
        wb_led_set(leddl, 2);
      else if (relay)
        wb_led_set(leddl, 4);
      else
        wb_led_set(leddl, 3);
    }    
    
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
      
    /*
    
    There are multiple cases and ways we can move:
    - If we are joining a team, we move to that direction. When we 
      are close to our goal, we move following the leader
    - If we are the follower in a simple scenario, we follow the 
      leader and the team
    - If we are the leader, we move following our direction
    
    */
    
    /* First of all we need to check whether we are requested to move somewhere because we are joining a team. */
    // if (location_change && !in_line && !oam_active) {
      /* We should do this until in line with the leader (when first join a team) or when redirected by leader */
      // update_movement_direction();
      
      /* Reset flag */
      
    // }
    
    /* If we reached our target location, we have to update our orientation to match the team's */
    // dist_to_goal = sqrtf(my_x*my_x + my_y*my_y);
    
    // if ( --- ) {
      /* If the distance is small enough, we diirectly turn in place to match the team's orientation */
      
      /* Reset the flag if the angle is correct */
      
    // }
    
    
    
    
    
    
    
    
    
    
    
    
    /* Last case, we move randomly */
    // if (dist_to_goal > 0.1) {
      
    // }
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    /* Move to location */
    if (location_change && !oam_active) {
      /* calculate necessary movement */
      double *values = (double* )wb_gps_get_values(gps);
      x = values[2];
      // double z = values[1];
      y = values[0];
      
      my_x = x - x_goal; //(x - x_ref) - x_goal;
      my_y = y - y_goal; //(y - y_ref) - y_goal;
      
      angle = atan2(my_y, my_x);
      
      if (my_y < 0) {
        angle = 360 + (angle * 180 / M_PI);
      }
      else {
        angle = angle * 180 / M_PI;
      }
      
      angle_compass = get_bearing_in_degrees(compass);
      
      diff_angle = angle - angle_compass;
      
      if (fabs(diff_angle) > 2 && (my_y > 0.01 || my_y < -0.01)) {
        speed[LEFT] = -150  * sign(diff_angle);
        speed[RIGHT] = 150  * sign(diff_angle);
      }

      /* if arrived reset the flag */
      
    }
    
    float dist_to_goal = sqrtf(my_x*my_x + my_y*my_y);
    
    if (dist_to_goal > 0.1) {// && !leader)
      // printf("%d I'm here 1\n", my_ID);
      // if (my_ID == 2)
        // printf("%d case 1\n", my_ID);
      wb_motor_set_velocity(left_motor, 0.00628 * speed[LEFT]); // speed[LEFT]);
      wb_motor_set_velocity(right_motor, 0.00628 * speed[RIGHT]); // speed[RIGHT]);
      // if (my_ID == 8 ) {//&& dist_to_goal < 0.2) {
        // printf("%d distance %f\n", my_ID, dist_to_goal);
        // printf("   x %f y %f\n", x, y);
        // printf("   my x %f my y %f\n", my_x, my_y);
        // printf("   angle %f\n", angle);
        // printf("   angle compass %f\n", angle_compass);
        // printf("   speed left %d right %d\n", speed[LEFT], speed[RIGHT]);
      // }
    }
    else if ((dist_to_goal <= 0.1 && location_change)){ // && !leader)
      // if (my_ID == 2)
        // printf("%d case 2\n", my_ID);
      if (dist_to_goal < min) {
        min = dist_to_goal;
        count = 0;
        wb_motor_set_velocity(left_motor, 0.00628 * speed[LEFT]); // speed[LEFT]);
        wb_motor_set_velocity(right_motor, 0.00628 * speed[RIGHT]); // speed[RIGHT]);
      }
      else {
        count += 1;
        wb_motor_set_velocity(left_motor, 0.00628 * speed[LEFT]);
        wb_motor_set_velocity(right_motor, 0.00628 * speed[RIGHT]);
      }
      
      if (count > 3) {
        double *values = (double* )wb_gps_get_values(gps);
        x = values[2];
        y = values[0];
        
        printf("%d arrived\n", my_ID);
        printf("  leader %d\n", leader_ID);
        printf("  error cm %f\n", dist_to_goal*100);
        printf("  reference x %f y %f\n", x_ref, y_ref);
        printf("  goal x %f y %f\n", x_goal, y_goal);
        printf("  real x %f y %f\n", x, y);
        printf("  my x %f y %f\n", my_x, my_y);
        // printf("   my x %f y %f\n", x, y);
        // * sign(my_y) * (-1)
        wb_motor_set_velocity(left_motor, 0); 
        wb_motor_set_velocity(right_motor, 0);
        // wb_robot_cleanup();
        location_change = false;
        // return 0;
      }
    }
    else if (leader && !location_change && !waiting_new_bot) {
      // printf("%d I'm here 2\n", my_ID);
      // if (my_ID == 2)
        // printf("%d case 3\n", my_ID);
      // wb_motor_set_velocity(left_motor, 0.0); 
      // wb_motor_set_velocity(right_motor, 0.0);
      wb_motor_set_velocity(left_motor, 0.00628 * speed[LEFT]);//.00628 * 150); 
      wb_motor_set_velocity(right_motor, 0.00628 * speed[RIGHT]);//.00628 * 150);
      wb_led_set(leddl, 6);
    }
    else if (!team_player) {
      // printf("%d I'm here 3\n", my_ID);
      wb_motor_set_velocity(left_motor, 0.00628 * speed[LEFT]);//.00628 * 150); 
      wb_motor_set_velocity(right_motor, 0.00628 * speed[RIGHT]);//.00628 * 150);
      wb_led_set(leddl, 6);
    }
    else {
      // printf("%d I'm here 4\n", my_ID);
      // printf("%d I'm here\n", my_ID);
      wb_motor_set_velocity(left_motor, 0.0);//.00628 * 150); 
      wb_motor_set_velocity(right_motor, 0.0);//.00628 * 150);
      wb_led_set(leddl, 5);
    }
    // printf("%d team ID: %d team index: %d team size: %d\n", my_ID, leader_ID, my_team_idx, idx_team+1);
    // printf("  ");
    // for (i=0; i<idx_team; i++) {
      // printf("%d ", team_IDs[i]);
    // }
    // printf("\n");
    // printf("  excess bot: %s\n", ext_connection.ext_ID_s);
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