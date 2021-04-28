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

void inglobate_external_team(int ext_ID, char ext_leader) {
  
}

void join_external_team(int ext_ID, char ext_leader) {
  
}