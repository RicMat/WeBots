///////
    // To Do
    ///////
    
    // if (mtl_active) {
      // angle = angle_offset();
      
      // if (turn) { /* we are facing the right direction */
        // forward_move = false;
        // turn_towards_location();
      // }
      // else { /* turn until we face the right direction */
        // forward_move = true;
      // }
      
      // if (forward_move) {
        // distance = calculate_distance_location();
        // wb_motor_set_velocity(left_motor, 0.00628 * speed[LEFT]);
        // wb_motor_set_velocity(right_motor, 0.00628 * speed[RIGHT]);
      // }
      
      // /* Arrived at location */
      // if (mtl_arrived) {
        // mtl_active = false;
        // mtl_arrived = false;
      // }
    // }  
    
    
    
    
    
    
    /* Move to location */
    if (location_change && !oam_active) {
      /* calculate necessary movement */
      double *values = (double* )wb_gps_get_values(gps);
      x = values[2];
      y = values[0];
      
      my_x = x - x_goal; //(x - x_ref) - x_goal;
      my_y = y - y_goal; //(y - y_ref) - y_goal;
      
      angle = atan2(my_y, my_x);
      
      if (my_y < 0) {
        angle = 360 + (angle * 180 / M_PI);
      }
      else {
        angle = angle * 180 / M_PI;
      }
      
      angle_compass = get_bearing_in_degrees(compass);
      
      diff_angle = angle - angle_compass;
      
      if (fabs(diff_angle) > 2 && (my_y > 0.01 || my_y < -0.01)) {
        speed[LEFT] = -150  * sign(diff_angle);
        speed[RIGHT] = 150  * sign(diff_angle);
      }

      /* if arrived reset the flag */
      
    }
    
    float dist_to_goal = sqrtf(my_x*my_x + my_y*my_y);
    
    if (dist_to_goal > 0.1) {// && !leader)
      // /*printf("%d I'm here 1\n", my_ID);
      // if (my_ID == 2)
        // printf("%d case 1\n", my_ID);
      wb_motor_set_velocity(left_motor, 0.00628 * speed[LEFT]); // speed[LEFT]);
      wb_motor_set_velocity(right_motor, 0.00628 * speed[RIGHT]); // speed[RIGHT]);
      // if (my_ID == 8 ) {//&& dist_to_goal < 0.2) {
        // printf("%d distance %f\n", my_ID, dist_to_goal);
        // printf("   x %f y %f\n", x, y);
        // printf("   my x %f my y %f\n", my_x, my_y);
        // printf("   angle %f\n", angle);
        // printf("   angle compass %f\n", angle_compass);
        // printf("   speed left %d right %d\n", speed[LEFT], speed[RIGHT]);
      // } */
    }
    else if ((dist_to_goal <= 0.1 && location_change)){ // && !leader)
      
      if (dist_to_goal < min) {
        min = dist_to_goal;
        count = 0;
        wb_motor_set_velocity(left_motor, 0.00628 * speed[LEFT]); // speed[LEFT]);
        wb_motor_set_velocity(right_motor, 0.00628 * speed[RIGHT]); // speed[RIGHT]);
      }
      else {
        count += 1;
        wb_motor_set_velocity(left_motor, 0.00628 * speed[LEFT]);
        wb_motor_set_velocity(right_motor, 0.00628 * speed[RIGHT]);
      }
      
      if (count > 3) {
        double *values = (double* )wb_gps_get_values(gps);
        x = values[2];
        y = values[0];
        
        printf("%d arrived\n", my_ID);
        printf("  leader %d\n", leader_ID);
        printf("  error cm %f\n", dist_to_goal*100);
        printf("  reference x %f y %f\n", x_ref, y_ref);
        printf("  goal x %f y %f\n", x_goal, y_goal);
        printf("  real x %f y %f\n", x, y);
        printf("  my x %f y %f\n", my_x, my_y);
        //  printf("   my x %f y %f\n", x, y);
        // * sign(my_y) * (-1)
        wb_motor_set_velocity(left_motor, 0); 
        wb_motor_set_velocity(right_motor, 0);
        // wb_robot_cleanup();
        location_change = false;
        // return 0; 
      }
    }
    else if (leader && !location_change && !waiting_new_bot) {
       printf("%d I'm here 2\n", my_ID);
       if (my_ID == 2)
         printf("%d case 3\n", my_ID);
       wb_motor_set_velocity(left_motor, 0.0); 
       wb_motor_set_velocity(right_motor, 0.0); 
      wb_motor_set_velocity(left_motor, 0.00628 * speed[LEFT]);//.00628 * 150); 
      wb_motor_set_velocity(right_motor, 0.00628 * speed[RIGHT]);//.00628 * 150);
      wb_led_set(leddl, 6);
    }
    else if (!team_player) {
      wb_motor_set_velocity(left_motor, 0.00628 * speed[LEFT]);//.00628 * 150); 
      wb_motor_set_velocity(right_motor, 0.00628 * speed[RIGHT]);//.00628 * 150);
      wb_led_set(leddl, 6);
    }
    else {
      wb_motor_set_velocity(left_motor, 0.0);//.00628 * 150); 
      wb_motor_set_velocity(right_motor, 0.0);//.00628 * 150);
      wb_led_set(leddl, 5);
    }
    printf("%d team ID: %d team index: %d team size: %d\n", my_ID, leader_ID, my_team_idx, idx_team+1);
    printf("  ");
    for (i=0; i<idx_team; i++) {
      printf("%d ", team_IDs[i]);
    }
    printf("\n");
    printf("  excess bot: %s\n", ext_connection.ext_ID_s);
  // }
  
  
  
  
///////////
//
// Perfect positioning
//
///////////

 // if (fmod(fabs(diff_angle), 360) > 2) {
        // printf("%d step 3\n", my_ID);
        /* 300 steps to turn 360 degrees */
        // speed[LEFT] = -150 ;// * sign(diff_angle);
        // speed[RIGHT] = 150; //  * sign(diff_angle);
        // printf("  left: %d, right %d\n", speed[LEFT], speed[RIGHT]);
        // diff_angle -= (sign(diff_angle) * 360.0/300.0);
        // angle_compass = (get_bearing_in_degrees(compass) + 90);
        // diff_angle = angle - angle_compass;
        // if (my_ID == 13)
          // printf("%f\n", diff_angle);
        /* We do not move forward so we can stop here */
      // }
      // else {
        /* We are not considering the following case: we meet an obstacle and our direction gets updated */
      
        /* We are in line with the goal, so we can calculate the distance */
        // if (dist_to_goal == -1 || oam_reset) 
        // printf("%d step 4\n", my_ID);
        // dist_to_goal = sqrtf(my_x*my_x + my_y*my_y);
        // printf("%d dist %f\n", my_ID, dist_to_goal);
        // in_line = true;
      // }
      /* Transfer outside: We should do this until in line with the leader (when first join a team) or when redirected by leader */
      // update_movement_direction();