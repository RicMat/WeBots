#ifndef MHM_H_   
#define MHM_H_

extern char message[34+1], tmp_s[34+1];
extern int team_IDs[7];
extern int idx_team;

void construct_discovery_message(int my_ID, char team_player, char leader, int team_size, char* leader_ID_s);
void construct_join_team_message(char* sender, char* receiver, char* TTL, char last_queued);
void construct_transfer_team_message(char* sender, char* receiver, char* TTL, char* ext_leader_ID);
void construct_share_with_team_message(char* code_in, char* sender, char* receiver, char* TTL, char* extra);

#endif 