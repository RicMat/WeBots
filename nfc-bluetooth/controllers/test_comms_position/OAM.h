#ifndef OAM_H_   
#define OAM_H_

// Global defines
#define TRUE 1
#define FALSE 0
#define NO_SIDE -1
#define LEFT 0
#define RIGHT 1
#define WHITE 0
#define BLACK 1
#define TIME_STEP 64  // [ms]
#define COMMUNICATION_CHANNEL 1
#define COMMUNICATION_CHANNEL_BT 2
// #define FLOOR_SIZE 1.0

// 8 IR proximity sensors
#define NB_DIST_SENS 8
#define PS_RIGHT_00 0
#define PS_RIGHT_45 1
#define PS_RIGHT_90 2
#define PS_RIGHT_REAR 3
#define PS_LEFT_REAR 4
#define PS_LEFT_90 5
#define PS_LEFT_45 6
#define PS_LEFT_00 7

// Obstacle defines
#define OAM_OBST_THRESHOLD 30
#define OAM_FORWARD_SPEED 150
#define OAM_K_PS_90 0.7 
#define OAM_K_PS_45 1.0 
#define OAM_K_PS_00 1.4 
#define OAM_K_MAX_DELTAS 600

extern int ps_value[NB_DIST_SENS];
extern int oam_active, oam_reset, oam_obst, obst_direction;
extern int oam_speed[2], speed[2];
extern int oam_side;
extern int message_sent;
extern int oam_reset_counter;

void obstacle_avoidance(void);

#endif 