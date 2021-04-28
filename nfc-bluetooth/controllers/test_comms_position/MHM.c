/* Messages Handligh Module */
#include "MHM.h"

void construct_discovery_message(char* code, int ID, char team_player, char leader) {
  strcpy(message, code);
  strcat(message, "%03d", ID);
  
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

void join_external_team(int ext_ID, char* ext_leader, char ) {
  /* join other bot in a new team */
  /* sends a series of LJT messages for each bot in the team */
  /* if not the leader, update all the slaves of the change */
  /* if the leader, update the status to slave */
  
  /* message structure: */
  
  int i = 0;
  char tmp_message[40+1];
  
  for (i=0; i<idx; i++) {
    tmp_message, 
  }
}