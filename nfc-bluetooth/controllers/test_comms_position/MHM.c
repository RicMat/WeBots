/* Messages Handligh Module */
#include <stdio.h>
#include <string.h>

#include "MHM.h"

void construct_discovery_message(int my_ID, char team_player, char leader, int team_size, char* leader_ID_s) {
  
  memset(message, 0, 34+1);
  strcpy(message, "DNB"); // code_in
  sprintf(tmp_s, "%03d", my_ID);
  strcat(message, tmp_s); // sender
  strcat(message, "000"); // receiver
  strcat(message, leader_ID_s); // team_ID
  strcat(message, "000"); // TTL
  
  if (team_player) { // team_player
    strcat(message, "Y"); 
  }
  else {
    strcat(message, "N");
  }
  
  sprintf(tmp_s, "%03d", idx_team+1);
  strcat(message, tmp_s); // team_size
  
  if (leader) { // leader - if I'm leader of my team
    strcat(message, "Y");
  }
  else {
    strcat(message, "N");
  }
  // printf("%d sending DNB\n", my_ID);
  strcat(message, leader_ID_s); // leader_ID_s
}

void construct_join_team_message(char* sender, char* receiver, char* team_ID_s, char* TTL, char* last_queued) {
  /* join other bot in a new team */
  /* sends a series of LJT messages for each bot in the team */
  /* if not the leader, update all the slaves of the change */
  /* if the leader, update the status to slave */
  
  /* message structure: 
    
    SSS - code_in - code of the message
    SSS - ID - ID of the bot we are transferring
    SSS - ext_ID_s - ID of the receiving robot (for comms)
    S - last_queued - Y if this is the last bot in queue
    
  */
  memset(message, 0, 34+1);
  strcpy(message, "LJT");
  strcat(message, sender);
  strcat(message, receiver);
  strcat(message, team_ID_s);
  strcat(message, TTL);
  strcat(message, last_queued);
  
  // printf("sending %s  %c\n", message, last_queued);
  
}

void construct_transfer_team_message(char* sender, char* receiver, char* ext_team_ID_s, char* TTL, int team_idx) {
  /* similar to the LJT */
  /* we are here contacting our team members and requesting them to transfer to the new team*/
  
  /* message structure: 
    
    SSS - code_in - code of the message
    SSS - sender
    SSS - receiver
    SSS - team_ID
    SSS - ext_leader_ID - ID of the external leader (new leader)
    
  */
  memset(message, 0, 34+1);
  strcpy(message, "TTT");
  strcat(message, sender);
  strcat(message, receiver);
  strcat(message, ext_team_ID_s);
  strcat(message, TTL);
  sprintf(tmp_s, "%03d", team_idx);
  strcat(message, tmp_s);
  
}

void construct_share_with_team_message(char* code_in, char* sender, char* receiver, char* team_ID_s, char* TTL, char* extra) {
  
  // printf("%s sending %s\n", sender, code_in);
  memset(message, 0, 34+1);
  strcpy(message, code_in); // ITM
  strcat(message, sender);
  strcat(message, receiver);
  strcat(message, team_ID_s);
  strcat(message, TTL);
  strcat(message, extra);
  
}

void construct_inform_location_message(char* sender, char* receiver, char* team_ID_s, char* TTL,  float x_ref, float y_ref, int team_idx) {
  /* inform the bot of the reference location as well as its team_ID */
  
  // printf("%s sending location info to %s and my leader is %s\n", sender, receiver, team_ID_s);
  memset(message, 0, 34+1);
  strcpy(message, "ILM");
  strcat(message, sender);
  strcat(message, receiver);
  strcat(message, team_ID_s);
  strcat(message, TTL); // TTL
  sprintf(tmp_s, "%07.3f", x_ref);
  strcat(message, tmp_s);
  sprintf(tmp_s, "%07.3f", y_ref);
  strcat(message, tmp_s);
  sprintf(tmp_s, "%1d", team_idx);
  strcat(message, tmp_s);
   
}