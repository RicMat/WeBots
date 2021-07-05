#include <webots/gps.h>
#include <webots/compass.h>
#include <math.h>
#include <stdio.h>
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

void obstacle_avoidance(void) {
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

double get_bearing_in_degrees(WbDeviceTag compass) { // no need 90 degree offset cause we are using two different reference planes
  const double *north = wb_compass_get_values(compass);
  double rad = atan2(north[2], north[0]);
  double bearing = (rad) * 180.0 / M_PI;
  
  if (bearing < 0.0)
    bearing = 360 +  bearing;
  
  return bearing;
}

int sign(float number) {
  if (number > 0)
    return 1;
  else
    return -1;
}

void handle_rotation(float g_angle) {
  if (g_angle != -1) {
    // printf("wtf %d\n", my_ID);
    angle_compass = (get_bearing_in_degrees(compass));
    if (angle_compass > 360)
      angle_compass -= 360;
  
    diff_angle = g_angle - angle_compass; //fmod(fabs(angle - angle_compass), 360); // we lose the sign
    if (fabs(diff_angle) > 180) {
      diff_angle = sign(diff_angle) * (360.0 - fabs(diff_angle)) * (-1);
    }
    turn_dist_counter = 0;
    backup = fabs(diff_angle);    
  }
  
  if (turn_dist_counter == 0 && g_angle == -1) {
    // printf("ok 1 %d\n", my_ID);
    /* If this is the first time checking the angle, we do it */
    double *values = (double* )wb_gps_get_values(gps);
    x = values[2];
    y = values[0];
    // printf("%d x: %f, y: %f, x_goal: %f, y_goal: %f\n", my_ID, x, y, x_goal, y_goal);
    
    my_x = x - x_goal; 
    my_y = y - y_goal;
  
    angle = atan2(my_y, my_x);
    if (my_y < 0) {
      angle = 360 + (angle * 180 / M_PI);
    }
    else {
      angle = angle * 180 / M_PI;
    }
    // printf("%d first time here\n", my_ID);
     
    angle_compass = (get_bearing_in_degrees(compass));
    if (angle_compass > 360)
      angle_compass -= 360;
  
    diff_angle = angle - angle_compass; //fmod(fabs(angle - angle_compass), 360); // we lose the sign
    if (fabs(diff_angle) > 180) {
      diff_angle = sign(diff_angle) * (360.0 - fabs(diff_angle)) * (-1);
    }
    turn_dist_counter = 0;
    backup = fabs(diff_angle);
  }
   
  /* We check the angle 3 times along the way */
  if (turn_dist_counter < 3) {
  
    // printf("ok 2 %d\n", my_ID);
    // printf("%d step 1\n", my_ID);
    // printf("  my idx %d\n", my_team_idx);
    // printf("  angle %f compass %f\n", angle, angle_compass);
    // printf("  diff_angle %f, backup/3 %f\n", diff_angle, backup/3);
    speed[LEFT] = -150 * sign(diff_angle);
    speed[RIGHT] = 150 * sign(diff_angle);
    diff_angle -= (sign(diff_angle) * 360.0/150.0);
    
    if (fabs(diff_angle) < (backup / 3) + 1 && fabs(diff_angle) > (backup / 3) - 1) {
      // printf("%d step 2\n", my_ID);
      // printf("  objective diff %f - backup %f = %f\n", diff_angle, (backup / 3), fabs(diff_angle) - (backup / 3));
      angle_compass = (get_bearing_in_degrees(compass));
      diff_angle = angle - angle_compass;
      if (fabs(diff_angle) > 180) {
        diff_angle = sign(diff_angle) * (360.0 - fabs(diff_angle)) * (-1);
      }
      // printf("  recalc angle %f compass %f\n", angle, angle_compass);
      // printf("  recalc diff_angle %f\n", diff_angle);
      backup = fabs(diff_angle);
      turn_dist_counter += 1;
    }
        
    if (fabs(diff_angle) < 2) {
      // printf("%d step 3\n", my_ID);
      dist_to_goal = sqrtf(my_x*my_x + my_y*my_y);
      // printf("%d initial distance %f\n", my_ID, dist_to_goal);
      // printf("  initial angle offset %f\n", diff_angle);
      backup = dist_to_goal;
      turn_dist_counter = 0;
      in_line = true;
    }
  }
  else {
    // printf("%d step 4\n", my_ID);
    speed[LEFT] = -150 * sign(diff_angle);
    speed[RIGHT] = 150 * sign(diff_angle);
    diff_angle -= (sign(diff_angle) * 360.0/150.0);
    if (fabs(diff_angle) < 2) {
      dist_to_goal = sqrtf(my_x*my_x + my_y*my_y);
      // printf("%d initial distance %f\n", my_ID, dist_to_goal);
      // printf("  initial angle offset %f\n", diff_angle);
      backup = dist_to_goal;
      turn_dist_counter = 0;
      in_line = true;
    }
  }
}

