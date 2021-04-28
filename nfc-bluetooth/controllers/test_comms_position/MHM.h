#ifndef MHM_H_   
#define MHM_H_

extern char message[40+1];

void construct_discovery_message(char* code, int ID, char team_player, char leader);
void inglobate_external_team(int ext_ID, char ext_leader);
void join_external_team(int ext_ID, char ext_leader);

#endif 