#ifndef MHM_H_   
#define MHM_H_

extern char message[34+1], tmp_s[34+1];
extern int team_IDs[7];
extern int idx_team;

void construct_discovery_message(int my_ID, char team_player, char leader, int team_size, char* leader_ID_s);
void construct_join_team_message(char* sender, char* receiver, char* team_ID_s, char* TTL, char* t_bot);
void construct_transfer_team_message(char* sender, char* receiver, char* ext_team_ID_s, char* TTL, int team_idx);
void construct_share_with_team_message(char* code_in, char* sender, char* receiver, char* team_ID_s, char* TTL, char* extra);
void construct_inform_location_message(char* sender, char* receiver, char* team_ID_s, char* TTL, float x_ref, float y_ref, int team_idx);

#endif 