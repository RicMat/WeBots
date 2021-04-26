// Red - X - Nord
// Green - Y - Up
// Blue - Z - East
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <webots/distance_sensor.h>
#include <webots/emitter.h>
#include <webots/motor.h>
#include <webots/receiver.h>
#include <webots/robot.h>
#include <webots/led.h>

// Global defines
#define TRUE 1
#define FALSE 0
#define NO_SIDE -1
#define LEFT 0
#define RIGHT 1
#define WHITE 0
#define BLACK 1
#define TIME_STEP 64  // [ms]
#define SPEED 6
#define COMMUNICATION_CHANNEL 1

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
const int PS_OFFSET_SIMULATION[NB_DIST_SENS] = {100, 100, 100, 100, 100, 100, 100, 100};
WbDeviceTag ps[NB_DIST_SENS]; /* proximity sensors */
int ps_value[NB_DIST_SENS] = {0, 0, 0, 0, 0, 0, 0, 0};

//------------------------------------------------------------------------------
//
//    BEHAVIORAL MODULES
//
//------------------------------------------------------------------------------


////////////////////////////////////////////
// ODM - Obstacle Detection Module
//
// The ODM routine detects obstacles in front of the robot, then records
// their side in "oam_side". ODM and OAM exist together, therefore there
// is no direct division in the variables name (odm and oam).
// Output obstacle presence and its direction are in oam_active and oam_side

#define OAM_OBST_THRESHOLD 50

int oam_active = FALSE;
//oam_reset;
int oam_side = NO_SIDE;

void ObstacleDetectionModule(void) {
  int max_ds_value, i;
  int Activation[] = {0, 0};

  // Module RESET
  // if (oam_reset) {
    // oam_active = FALSE;
    // oam_side = NO_SIDE;
  // }
  // oam_reset = 0;

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

  if (oam_active && oam_side == NO_SIDE)  // check for side of obstacle only when not already detected
  {
    if (Activation[RIGHT] > Activation[LEFT])
      oam_side = RIGHT;
    else
      oam_side = LEFT;
  }
}
  
////////////////////////////////////////////
// OAM - Obstacle Avoidance Module
//
// The OAM routine avoids the detected obstacle by turning away according
// to very simple weighted connections between proximity sensors and motors.
// "oam_active" becomes active as soon as an object is detected 
// and "oam_reset" inactivates the module and sets "oam_side" to NO_SIDE. 
// Output speeds are in oam_speed[LEFT] and oam_speed[RIGHT].

int oam_speed[2];

#define OAM_FORWARD_SPEED 150
#define OAM_K_PS_90 0.2
#define OAM_K_PS_45 0.9
#define OAM_K_PS_00 1.2
#define OAM_K_MAX_DELTAS 600

void ObstacleAvoidanceModule(void) {
  // Forward speed
  oam_speed[LEFT] = OAM_FORWARD_SPEED;
  oam_speed[RIGHT] = OAM_FORWARD_SPEED;

  // Go away from obstacle
  if (oam_active) {
    int DeltaS = 0;
    // The rotation of the robot is determined by the location and the side of the obstacle
    if (oam_side == LEFT) {
      //(((ps_value[PS_LEFT_90]-PS_OFFSET)<0)?0:(ps_value[PS_LEFT_90]-PS_OFFSET)));
      DeltaS -= (int)(OAM_K_PS_90 * ps_value[PS_LEFT_90]);
      //(((ps_value[PS_LEFT_45]-PS_OFFSET)<0)?0:(ps_value[PS_LEFT_45]-PS_OFFSET)));
      DeltaS -= (int)(OAM_K_PS_45 * ps_value[PS_LEFT_45]);
      //(((ps_value[PS_LEFT_00]-PS_OFFSET)<0)?0:(ps_value[PS_LEFT_00]-PS_OFFSET)));
      DeltaS -= (int)(OAM_K_PS_00 * ps_value[PS_LEFT_00]);
    } else {  // oam_side == RIGHT
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

////////////////////////////////////////////
// CSM - Communication Reset Module
//
// The CRM routine contains all the functions needed to initially 
// set up the communication module. 

bool leader = FALSE;
bool is_in_queue = 0;
bool is_stop = FALSE;
bool is_init = -1;
int comms_queue[3];
int leader_ID = 0;
int idx = 0;
int i = 0;
int my_ID = -1;

void CommunicationsReset(void) {
  
  my_ID = atoi(wb_robot_get_name() + 5);
  leader = FALSE;
  is_in_queue = FALSE;
  is_stop = FALSE;
  comms_queue[3] = [-1, -1, -1];
  leader_ID = 0;
  
}

void ExchangeIDs(void) {

  /* wait for a message and act accordingly */
  if (wb_receiver_get_queue_length(receiver) > 0) {
    const char *buffer = wb_receiver_get_data(receiver);
    is_init = strncmp(buffer, "initc", 5);
    is_ID = strncmp(buffer, "epuck", 5);
    if (is_init == 0) {
      
      
    }
    if (is_ID == 0) {
      ext_ID = atoi(buffer + 5);
      
    }
  }
  else { // send my message and go forward one step
    const char *message = wb_robot_get_name();
    SendMessageStr(message);   // To Do: check and discard other similar messages  
    wb_robot_step(TIME_STEP); 
  }
  
}

void SendMessageStr(const char* message) {
  
  wb_emitter_send(emitter, message, strlen(message) + 1);
  
}

void SendMessageInt(int num) {
  
  
  int length = snprintf(NULL, 0, "%d", num);
  char *messagee = malloc(length + 1);
  snprintf(messagee, length + 1, "%d", num);
  wb_emitter_send(emitter, messagee, strlen(messagee) + 1);
      
}

bool ReceiveMessage(const char* code) {
  
  
  exteam_ID = 
}


////////////////////////////////////////////
// LFM - Leader Follower Module
//
// The LFM routine contains the functions needed to decide whether the
// current robot is the leader or the follower of the team. Since most of
// the times the robot that bumps on another robot is not the leader, we
// need to consider the case where leader_ID is not -1 (no leader selected yet).

void LeaderFollowerModule(void) {
  
  /* check if we have a leader in place then transfer it to the other robot */
  if (leader_ID != NO_LEADER) {
    team_ID = leader_ID;
  }
  else {
    team_ID = my_ID;
  }
  
  SendMessageStr("teaID");
  SendMessageInt(team_ID);
  
  /* wait for the team_ID of the other robot */
  while (!ReceiveMessageStr("teaID")) {
    wb_robot_step(TIME_STEP);
  }
  
  /* save the team_ID of the other robot */
  ReceiveMessageInt();
  
  if (ext_ID == exteam_ID) {
    ext_is_leader = TRUE;
  }
  else {
    ext_is_leader = FALSE;
  }
  
  /* decide who is the leader -- if my ID is smaller I become the leader */
  if (exteam_ID > team_ID) {
    if (leader == FALSE) {
      leader = TRUE;
      ReceiveMessageStr("listc");
      ImportQueue(); // comms_queue[idx] = ext_ID; // idx += 1;
    }
    else {
      // To Do: transfer all the data to my leader
    }
  }
  else {
    
    /* if I'm not the leader, I gotta save the new leader ID and transfer the queue of our team */
    if (leader_ID != ext_ID) {
      SendMessageStr("listc");
      
      /* I also need to pass the list of robots in my queue */
      for(i = 0; i < idx; i++) {
        int length = snprintf(NULL, 0, "%d", comms_queue[i]);
        char *messagee = malloc(length + 1);
        snprintf(messagee, length + 1, "%d", comms_queue[i]);
        wb_emitter_send(emitter, messagee, strlen(messagee) + 1);          
      }
      
      SendMessageStr("lists");
      leader_ID = ext_ID;
    }
    
    /* communicate to my leader that we have lost leadership */
    if (leader == FALSE) {
      
    }
    else {
      
    }
    
    leader = FALSE;
  } 
  // To Do: move to position
}


//------------------------------------------------------------------------------
//
//    CONTROLLER
//
//------------------------------------------------------------------------------

int main() {
  
  int ps_offset[NB_DIST_SENS] = {0, 0, 0, 0, 0, 0, 0, 0}, i, speed[2];
  WbDeviceTag emitter, receiver, left_motor, right_motor;
  
  wb_robot_init();

  /* get a handler to the motors and set target position to infinity (speed control). */
  left_motor = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);
  
  /* communication sensors */
  emitter = wb_robot_get_device("nfc_e");
  wb_emitter_set_channel(emitter, COMMUNICATION_CHANNEL);
  wb_emitter_set_range(emitter, 0.04);
  
  receiver = wb_robot_get_device("nfc_r");
  wb_receiver_enable(receiver, TIME_STEP);
  
  /* proximity sensors */
  for (i = 0; i < NB_DIST_SENS; i++) {
    sprintf(name, "ps%d", i);
    ps[i] = wb_robot_get_device(name);
    wb_distance_sensor_enable(ps[i], TIME_STEP);
  }
  
  for (i = 0; i < NB_DIST_SENS; i++)
    ps_offset[i] = PS_OFFSET_SIMULATION[i];
  
  /* reset the communications module */
  CoomunicationsReset();
         
  while (wb_robot_step(TIME_STEP) != -1) {
  
    /* collect data from the proximity sensors */
    for (i = 0; i < NB_DIST_SENS; i++){
      ps_value[i] = (((int)wb_distance_sensor_get_value(ps[i]) - ps_offset[i]) < 0) ?
                      0 :
                      ((int)wb_distance_sensor_get_value(ps[i]) - ps_offset[i]);
    }
    
    /* if there is an obstacle, we turn towards it and check for
    any NFC messages */
    ObstacleDetectionModule();
    
    if (oam_side != NO_SIDE) { // there is an obstacle
      
      if (facing_obstacle == FALSE && is_robot == UNKNOWN) { 
      /* we turn towards the unknown obstacle to try to communicate */
        TurnToObstacle();
      }
      /* we are facing the obstacle therefore we try to communicate and
      check if we can establish a communication */
      else if (facing_obstacle == TRUE && is_robot == UNKNOWN) {
        TestNFCPresence(); // To Do: test it for the necessary cycles
      }
      
      if (is_robot == TRUE && connection_started == FALSE) {
        connection_started = TRUE;
        
        /* transmit my ID and get the ID of the other robot */
        ExchangeIDs();
        
        /* Leader Follower decision */
        LeaderFollowerModule();
        
        
        
      }
      else { // it's just an obstacle
        ObstacleAvoidanceModule();
      }
      
      
    }
    /* if there is no obstacle we continue our random navigation */
    else {
      RuntimeCommunication();
      RandomNavigation();
    }
    
    
    
    
    
    
    
    
    
    
    
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
    
    
    // First decide if I'm the leader

    
    if (wb_receiver_get_queue_length(receiver) > 0) {
      
      is_in_queue = 0;
      const char *buffer = wb_receiver_get_data(receiver); // receive the packet
      
      // Check if it is an ID
      int is_ID = strncmp(buffer, "epuck", 5);
      
      // If it is an ID, we check if we have it saved in our comms_queue
      if (is_ID == 0) {
      
        int ext_ID = atoi(buffer + 5);
        // printf("my idx is = %d\n", idx);
        // printf("got an ID, idx = %d"
        for(i = 0; i < idx; i++) {
            
          if(comms_queue[i] == ext_ID) {
            is_in_queue = 1;
          }
          // printf("ID = %s, idx = %d, i = %d, ext_ID = %d, comms_queue[i] = %d\n", wb_robot_get_name() + 5, idx, i, ext_ID, comms_queue[i]);
          
          
        }
           
        // if (my_ID == 2)
          // for(i = 0; i < idx; i++)
              // printf("I'm %s and %d is \n", wb_robot_get_name() + 5, comms_queue[i]);
            
        if (is_in_queue == 0) {
          // for(i = 0; i < idx; i++)
              // printf("I'm %s and my queue contains %d\n", wb_robot_get_name() + 5, comms_queue[i]);
            
          if (ext_ID > my_ID) {
            leader = true;
            printf("Communicating: I'm robot \"%d\" and I am the leader\n", my_ID);
            // printf("saving at index %d \n", idx);
            comms_queue[idx] = ext_ID;
            // printf("%d\n", idx);
            idx += 1;
            // printf("my new idx is = %d\n", idx);
            // printf("new index is %d \n", idx);
          }
          else {
            leader = false;
            if (leader_ID != ext_ID) {
              printf("Communicating: I'm robot \"%d\" and I am NOT the leader\n", my_ID);
              leader_ID = ext_ID;
              char *messagel = "listc";
              wb_emitter_send(emitter, messagel, strlen(messagel) + 1);
              /* I also need to pass the list of robots in my queue */
              for(i = 0; i < idx; i++) {
                int length = snprintf(NULL, 0, "%d", comms_queue[i]);
                char *messagee = malloc(length + 1);
                snprintf(messagee, length + 1, "%d", comms_queue[i]);
                // char *messagee = "aaa";
                // sprintf(messagee, "%d", comms_queue[i]);
                wb_emitter_send(emitter, messagee, strlen(messagee) + 1);
                // printf("sending queue = %d\n", comms_queue[i]);
                // idx--;
              }
              char *messages = "lists";
              wb_emitter_send(emitter, messages, strlen(messages) + 1);
            }
            
          }
        }
        
      }
      
      int is_list_comms = strncmp(buffer, "listc", 5);
      
      // We have already read the header so we can go to the next one
      wb_receiver_next_packet(receiver);
      
      // If it is not an ID, in this case we know that it contains a list of followers to be transmitted to the new leader
      if (is_list_comms == 0) {
      
        while (wb_receiver_get_queue_length(receiver) > 0) {
          
          // Read new message and save the ID in  the queue
          const char *buffer = wb_receiver_get_data(receiver); // receive the packet
          
          is_stop = strncmp(buffer, "lists", 5);
          
          if (is_stop == 0) {
            if (idx > 0)
              printf("ID: %s, my comms queue contains:", wb_robot_get_name() + 5);
            for(i = 0; i < idx; i++) {
              printf(" %d", comms_queue[i]);
            }
            printf("\n");
            break;
          }
          
          comms_queue[idx] = atoi(buffer);
          idx ++;
          
          wb_receiver_next_packet(receiver);
          
        }
        
        // if (message_printed < 1) {
          // if (leader) {
            // wb_led_set(led, 2); // set led green
            // printf("Communicating: I'm robot \"%d\" and I am the leader\n", my_ID);
            // //If I'm the leader, wait to read the queue
            
          // }
          // else {
            // wb_led_set(led, 1); // set led red
            // printf("Communicating: I'm robot \"%d\" and I am NOT the leader\n", my_ID);
            // //If I have a queue, I'll pass it to the leader
            
          // }
          // message_printed = 1;
        // }
        
      }
        
    }    
    
    ////////////////////////////////////////////
    // Final step of the simulation
    //
    // See that the code is actually running
    
    if (my_ID == 4) {
      wb_motor_set_velocity(left_motor, 0);
      wb_motor_set_velocity(right_motor, 0);
    }
    else {
      wb_motor_set_velocity(left_motor, -2);
      wb_motor_set_velocity(right_motor, -2);
    }
  }

  wb_robot_cleanup();

  return 0;
}
