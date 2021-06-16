/* Extra Team Module */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "ETM.h"

void leader_store_external_connection(int hop_idx, char* ext_ID_s, char* ext_leader_ID_s, int ext_team_size) {
  strcpy(ext_connections[hop_idx].ext_ID_s, ext_ID_s);
  strcpy(ext_connections[hop_idx].ext_leader_ID_s, ext_leader_ID_s);
  ext_connections[hop_idx].size += ext_team_size;
}

void store_external_connection(char* ext_ID_s, char* ext_leader_ID_s, int ext_team_size) {
  strcpy(ext_connection.ext_ID_s, ext_ID_s);
  strcpy(ext_connection.ext_leader_ID_s, ext_leader_ID_s);
  ext_connection.size += ext_team_size;
}

int retrieve_closest_external_connection(int hop_team_idx, int ext_team_size) {
  
  // float x_ext, y_ext;
  int scores[6] = {0};
  int out, min = 1000;
  // printf("  line 1\n");
  struct option
  {
    int size;
    int idx;
  } options[idx_team];
  
  // printf("  line 2 idx team: %d\n", idx_team);
  for (i=0; i<idx_team; i++) {
    // printf("  line 3\n");
    /* if there is a connection established */
    if (ext_connections[i].size > 0) {
      // printf("  line 4 i: %d\n", i);
      /* only possible unions */
      // printf("  extcommsize: %d extsize: %d\n", ext_connections[i].size, ext_team_size);
      if (ext_connections[i].size + ext_team_size <= 7) {
        // printf("  line 5 size: %d idx: %d\n", ext_connections[i].size + ext_team_size, i);
        /* save the index and the total size */
        options[i].size = ext_connections[i].size + ext_team_size;
        options[i].idx = i;
      }
      else {
        options[i].size = ext_team_size;
        options[i].idx = i;
      }
    }
    else {
      // printf("  line 6 i: %d size: %d idx: %d\n", i, ext_connections[i].size + ext_team_size, i);
      /* save the index and the total size */
      options[i].size = ext_team_size;
      options[i].idx = i;
    }
  }
  
  /*
  
  For each possible connection we calculate a score:
  S = (1.5 * distance) + missing spots after union
  The lower the score the better the option is
  
  */
  // printf("---\n");
  for (i=0; i< idx_team; i++) {
    // printf("  line 7 i: %d scores[i]: %d\n", i, scores[i]);
    scores[i] += 7 - options[i].size;
    // printf("  plus 1 %d\n", 7 - options[i].size);
    scores[i] += 2 * abs(options[i].idx - hop_team_idx);
    // printf("  plus 2 %d\n", 2 * abs(options[i].idx - hop_team_idx));
    if (scores[i] < min) {
      // printf("  line 8 i: %d\n", i);
      // printf("  scores: %d\n", scores[i]);
      min = scores[i];
      out = i;
    }
  }
  
  return out;
  
}

