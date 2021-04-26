#include "OAM.h"

////////////////////////////////////////////
// OAM - Obstacle Avoidance Module
//
// The OAM routine first detects obstacles in front of the robot, then records
// their side in "oam_side" and avoid the detected obstacle by
// turning away according to very simple weighted connections between
// proximity sensors and motors. "oam_active" becomes active when as soon as
// an object is detected and "oam_reset" inactivates the module and set
// "oam_side" to NO_SIDE. Output speeds are in oam_speed[LEFT] and oam_speed[RIGHT].

//0.2 0.9 1.2
// #define OAM_OBST_THRESHOLD 150
// #define OAM_FORWARD_SPEED 150

int kkk = 0;

void ObstacleAvoidanceModule(void) {
  int max_ds_value, i;
  int Activation[] = {0, 0};

  // Module RESET
  if (oam_reset) {
    oam_active = FALSE;
    oam_side = NO_SIDE;
    oam_obst = FALSE;
  }
  oam_reset = 0;

  // Determine the presence and the side of an obstacle
  max_ds_value = 0;
  for (i = PS_RIGHT_00; i <= PS_RIGHT_45; i++) {
    if (max_ds_value < ps_value[i])
      max_ds_value = ps_value[i];
    Activation[RIGHT] += ps_value[i];
  }
  for (i = PS_LEFT_45; i <= PS_LEFT_00; i++) {
    if (max_ds_value < ps_value[i])
      max_ds_value = ps_value[i];
    Activation[LEFT] += ps_value[i];
  }
  if (max_ds_value > OAM_OBST_THRESHOLD){
    oam_active = TRUE;
    oam_reset_counter = 0;
  }
  else {
    oam_reset_counter += 1;
    if (oam_reset_counter > 40) {
      oam_reset = TRUE;
      message_sent = FALSE;
    }
  }
  /* Check for obstacles, turn in place to look for connections */
  max_ds_value = 0;
  for (i = PS_RIGHT_00; i <= PS_LEFT_00; i++) {
    if (max_ds_value < ps_value[i]) {
      max_ds_value = ps_value[i];
      obst_direction = i;
    }
  }
  if (max_ds_value > OAM_OBST_THRESHOLD){
    oam_obst = TRUE;
    // printf("%d\n", obst_direction);
  }

  if (oam_active && oam_side == NO_SIDE)  // check for side of obstacle only when not already detected
  {
    if (Activation[RIGHT] > Activation[LEFT])
      oam_side = RIGHT;
    else
      oam_side = LEFT;
  }
  
  // Forward speed
  oam_speed[LEFT] = OAM_FORWARD_SPEED;
  oam_speed[RIGHT] = OAM_FORWARD_SPEED;

  // Go away from obstacle
  if (oam_active) {
    // if (strncmp(wb_robot_get_name(), "epuck1", 7) == 0) {
      // printf("test\n"); 
    // }
    int DeltaS = 0;
    // The rotation of the robot is determined by the location and the side of the obstacle
    if (oam_side == LEFT) {
      // printf("left %d %s\n", oam_active, wb_robot_get_name());
      //(((ps_value[PS_LEFT_90]-PS_OFFSET)<0)?0:(ps_value[PS_LEFT_90]-PS_OFFSET)));
      DeltaS -= (int)(OAM_K_PS_90 * ps_value[PS_LEFT_90]);
      //(((ps_value[PS_LEFT_45]-PS_OFFSET)<0)?0:(ps_value[PS_LEFT_45]-PS_OFFSET)));
      DeltaS -= (int)(OAM_K_PS_45 * ps_value[PS_LEFT_45]);
      //(((ps_value[PS_LEFT_00]-PS_OFFSET)<0)?0:(ps_value[PS_LEFT_00]-PS_OFFSET)));
      DeltaS -= (int)(OAM_K_PS_00 * ps_value[PS_LEFT_00]);
    } else {  // oam_side == RIGHT
      // printf("right\n");
      //(((ps_value[PS_RIGHT_90]-PS_OFFSET)<0)?0:(ps_value[PS_RIGHT_90]-PS_OFFSET)));
      DeltaS += (int)(OAM_K_PS_90 * ps_value[PS_RIGHT_90]);
      //(((ps_value[PS_RIGHT_45]-PS_OFFSET)<0)?0:(ps_value[PS_RIGHT_45]-PS_OFFSET)));
      DeltaS += (int)(OAM_K_PS_45 * ps_value[PS_RIGHT_45]);
      //(((ps_value[PS_RIGHT_00]-PS_OFFSET)<0)?0:(ps_value[PS_RIGHT_00]-PS_OFFSET)));
      DeltaS += (int)(OAM_K_PS_00 * ps_value[PS_RIGHT_00]);
    }
    if (DeltaS > OAM_K_MAX_DELTAS)
      DeltaS = OAM_K_MAX_DELTAS;
    if (DeltaS < -OAM_K_MAX_DELTAS)
      DeltaS = -OAM_K_MAX_DELTAS;

    // Set speeds
    oam_speed[LEFT] -= DeltaS;
    oam_speed[RIGHT] += DeltaS;
  }
  
  speed[LEFT] = oam_speed[LEFT]; 
  speed[RIGHT] = oam_speed[RIGHT];
  
}