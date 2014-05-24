#include "calibrate.hpp"

#include "devices.hpp"
#include "fpga.hpp"
#include "trajectory.hpp"

// CONFIG
static const s32 table_width = 3000;
static const s32 table_height = 2000;
static const s32 robot_width = 430;
static const s32 robot_height = 250;

static const s32 center_y_offset = 50;

static const s32 buffet_height = 300;

// RED
static const s32 RED_DIST_INIT = -3000;
static const s32 RED_DIST_2_INIT = 600;
static const s32 RED_A_INIT = 0;
static const s32 RED_X_INIT = -(table_width - robot_height) / 2;
static const s32 RED_Y_INIT = (table_height - robot_height) / 2 - buffet_height + center_y_offset;
static const s32 RED_ANGLE_CALIB = -90;

// YELLOW
static const s32 YELLOW_DIST_INIT = 3000;
static const s32 YELLOW_DIST_2_INIT = -600;
static const s32 YELLOW_A_INIT = 0;
static const s32 YELLOW_X_INIT = (table_width - robot_height) / 2;
static const s32 YELLOW_Y_INIT = (table_height - robot_height) / 2 - buffet_height + center_y_offset;
static const s32 YELLOW_ANGLE_CALIB = -90;

// GENERAL

static s32 DIST_INIT = 0;
static s32 DIST_2_INIT = 0;
static s32 A_INIT = 0;
static s32 X_INIT = 0;
static s32 Y_INIT = 0;
static s32 ANGLE_CALIB = 0;

///////////////////////////////////////////////////////////////////////

void set_angle(s32 new_angle) {
  s32 old_angle = odo.getValue().coord(1) >> 4;
  s32 new_zero = new_angle - old_angle;
  //s32 dist = odo.getValue().coord(0);

  //s32 cur_dist = odo.getValue().coord(0);

  io << "Show zero\n";
  traj.gotoAngle(new_zero);
  while((odo.getValue().coord(1) >> 4) != -new_zero);

  io << "reset...\n";
  robot.lock();
  
  //fpga_reset();
  fpga_position_reset();

  fpga_config();
  trajectory_reset();
  robot.unlock();  

  while(odo.getValue().coord(1) != 0) {
  }
  
  io << "Go back to angle\n";
  io << new_angle << "\n";
  traj.gotoAngle(-new_angle);
  while((odo.getValue().coord(1) >> 4) != new_angle);
  io << "angle ok\n";
}

void goto_wall(s32 dist) {
  io << "goto wall\n";

  traj.gotoDistance(dist);

  while(!robot.getValue());

  io << "wall touched\n";
  
  if(dist < 0) {
    traj.gotoDistance(500);
  }
  else {
    traj.gotoDistance(-500);
  }

  robot.unlock();

  while(!robot.getValue());

  io << "touching wall\n";
  traj.reset();
  robot.unlock();
}

void match_init(bool red_side) {
  (void) red_side;

  io << "Goto wall\n";
  goto_wall(DIST_INIT);
  io << "Reset X and angle\n";
  set_angle(A_INIT);
  pos.setX(X_INIT);
  
  io << "Go far from the wall\n";
  traj.gotoDistance(DIST_2_INIT);
  while(!traj.isEnded()) {
    robot.unlock();
  }

  io << "Face buffet\n";
  traj.gotoAngle(ANGLE_CALIB);
  while(!traj.isEnded());

  io << "Goto buffet\n";
  goto_wall(-3000);
  
  io << "Reset Y\n";
  pos.setY(Y_INIT);

  io << "Go far from the wall\n";
  traj.gotoDistance(500);
  while(!traj.isEnded()) {
    robot.unlock();
  }
  
  io << "Done !\n";
}

void side_init(bool red_side) {
  if(red_side) {
    io << "Init RED side\n";

    // Init
    DIST_INIT = RED_DIST_INIT;
    DIST_2_INIT = RED_DIST_2_INIT;
    A_INIT = RED_A_INIT;
    X_INIT = RED_X_INIT;
    Y_INIT = RED_Y_INIT;
    ANGLE_CALIB = RED_ANGLE_CALIB;

    // Strategy
    // BEGIN_POINT = RED_BEGIN_POINT;
    // FIRST_FIRE_POINT = RED_FIRST_FIRE_POINT;
    // SECOND_FIRE_POINT = RED_SECOND_FIRE_POINT;
    // FRONT_FRESQ_POINT = RED_FRONT_FRESQ_POINT;
    // FRESQ_POINT = RED_FRESQ_POINT;

    // ADV_SECOND_FIRE_POINT = RED_ADV_SECOND_FIRE_POINT;
    // ADV_BEFORE_FIRST_POINT = RED_ADV_BEFORE_FIRST_POINT;
    // ADV_FIRST_FIRE_POINT = RED_ADV_FIRST_FIRE_POINT;

    // ADV_BEFORE_THIRD_POINT = RED_ADV_BEFORE_THIRD_POINT;
    // ADV_THIRD_FIRE_POINT = RED_ADV_THIRD_FIRE_POINT;
    // THIRD_FIRE_POINT = RED_THIRD_FIRE_POINT;

  }
  else {
    io << "Init YELLOW side\n";

    // Init
    DIST_INIT = YELLOW_DIST_INIT;
    DIST_2_INIT = YELLOW_DIST_2_INIT;
    A_INIT = YELLOW_A_INIT;
    X_INIT = YELLOW_X_INIT;
    Y_INIT = YELLOW_Y_INIT;
    ANGLE_CALIB = YELLOW_ANGLE_CALIB;

    // Strategy
    // BEGIN_POINT = YELLOW_BEGIN_POINT;
    // FIRST_FIRE_POINT = YELLOW_FIRST_FIRE_POINT;
    // SECOND_FIRE_POINT = YELLOW_SECOND_FIRE_POINT;
    // FRONT_FRESQ_POINT = YELLOW_FRONT_FRESQ_POINT;
    // FRESQ_POINT = YELLOW_FRESQ_POINT;

    // ADV_SECOND_FIRE_POINT = YELLOW_ADV_SECOND_FIRE_POINT;
    // ADV_BEFORE_FIRST_POINT = YELLOW_ADV_BEFORE_FIRST_POINT;
    // ADV_FIRST_FIRE_POINT = YELLOW_ADV_FIRST_FIRE_POINT;

    // ADV_BEFORE_THIRD_POINT = YELLOW_ADV_BEFORE_THIRD_POINT;
    // ADV_THIRD_FIRE_POINT = YELLOW_ADV_THIRD_FIRE_POINT;
    // THIRD_FIRE_POINT = YELLOW_THIRD_FIRE_POINT;

  }
}
