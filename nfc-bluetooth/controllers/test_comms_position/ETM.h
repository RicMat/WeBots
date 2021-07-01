#ifndef ETM_H_   
#define ETM_H_

typedef struct LeaderExtConnection
{
  char hop_ID_s[3+1];
  char ext_ID_s[3+1];
  char ext_leader_ID_s[3+1];
  int size;
} leader_conn;

typedef struct ExtConnection
{
  char ext_ID_s[3+1];
  char ext_leader_ID_s[3+1];
  int size;
  int arrived;
} conn;

extern conn ext_connection;
extern leader_conn ext_connections[6];

extern char ext_leader_ID_s[3+1];
extern int i, idx_team;

void leader_store_external_connection(int hop_idx, char* ext_ID_s, char* ext_leader_ID_s, int ext_team_size);
void store_external_connection(char* ext_ID_s, char* ext_leader_ID_s, int ext_team_size);
int retrieve_closest_external_connection(int hop_team_idx, int ext_team_size);

#endif 