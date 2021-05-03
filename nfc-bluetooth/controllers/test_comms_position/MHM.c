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

void construct_join_team_message(int sender, char* receiver, char last_queued) {
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
  strcpy(message, "LJT");
  strcat(message, "%03d", sender);
  strcat(message, receiver);
  strcat(message, last_queued);
  
}

void construct_transfer_team_message(int sender, char* receiver, char* ext_leader_ID) {
  /* similar to the JTM */
  /* we are here contacting our team members and requesting them to transfer to the new team*/
  
  /* message structure: 
    
    SSS - code_in - code of the message
    SSS - 
    SSS - ext_leader_ID - ID of the external leader (new leader)
    
  */
  strcpy(message, "TTT");
  strcat(message, "s", sender);
  strcat(message, receiver);
  strcat(message, ext_leader_ID);
  
}