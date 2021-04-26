#include "RSM.h"

void reset_simulation(void){
  double translation[3] = {0.0, 0.0, 0.0};
  field = wb_supervisor_node_get_field(node, "translation");
  translation[0] = (float)rand() / (float)(RAND_MAX / (0.92 * FLOOR_SIZE)) - (0.46 * FLOOR_SIZE);
  translation[1] = 0.0;
  translation[2] = (float)rand() / (float)(RAND_MAX / (0.92 * FLOOR_SIZE)) - (0.46 * FLOOR_SIZE);
  wb_supervisor_field_set_sf_vec3f(field, translation);
  double rotation[4] = {0.0, 1.0, 0.0, 0.0};
  field = wb_supervisor_node_get_field(node, "rotation");
  rotation[3] = (float)rand() / (float)(RAND_MAX / 6.28319);
  wb_supervisor_field_set_sf_rotation(field, rotation);
        
}