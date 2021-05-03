/* Messages Handligh Module */
#include "MHM.h"

void construct_discovery_message(int my_ID, char team_player, char leader) {
  strcpy(message, "DNB");
  strcat(message, "%03d", my_ID);
  strcat(message, "000");
  
  if (team_player) {
    strcat(message, "Y"); 
  }
  else {
    strcat(message, "N");
  }
  
  if (leader) {
    strcat(message, "Y");
  }
  else {
    strcat(message, "N");
  }
  
}

void inglobate_external_team(int ext_ID, char* ext_leader) {
  
}

void construct_join_team_message(int transfer_ID, char* ext_ID_s, char last_in_queue) {
  /* join other bot in a new team */
  /* sends a series of LJT messages for each bot in the team */
  /* if not the leader, update all the slaves of the change */
  /* if the leader, update the status to slave */
  
  /* message structure: 
    
    SSS - code_in - code of the message
    III - ID - ID of the bot we are transferring
    SSS - ext_ID_s - ID of the receiving robot (for comms)
    S - last_queued - Y if this is the last bot in queue
    
  */
  strcpy(message, "LJT");
  strcat(message, "%03d", transfer_ID);
  strcat(message, ext_ID_s);
  strcat(message, last_in_queue);
  
}