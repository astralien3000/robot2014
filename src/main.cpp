#include <aversive.hpp>

#include "filters.hpp"
#include "asserv.hpp"
#include "eirbot_shell.hpp"
#include "devices.hpp"
#include <hardware/interrupts.hpp>
#include <system/scheduler.hpp>
#include "fpga.hpp"
#include "astar.hpp"

#include "curv_trajectory_manager.hpp"

#include "strategy.hpp"

#define F_CPU 16000000l
#include <util/delay.h>

Scheduler& sched = Scheduler::instance();

PidFilter pid_rt;
PidFilter pid_ct;
CurvTrajectoryManager traj(robot, odo, pos, pid_rt, pid_ct);

FpgaUartStream rds_io("rds_stream", UART_TX_1_DATA, UART_TX_1_OCUP, UART_RX_1_DATA, UART_RX_1_AVA);

Vect<2, s32> _cmd(0,0);
bool traj_mode = true;
s32 cmd_l = 0;
s32 cmd_r = 0;

void control_init(void) {
  Task t([](void) {
      //motc_l.setValue(cmd_l);
      //motc_r.setValue(cmd_r);
      //robot.setValue(_cmd);
      traj.update();
    });

  t.setPeriod(64000);
  t.setRepeat();
  sched.addTask(t);

  Interrupts::set();
}

void trajectory_reset(void) {
  pid_l.reset();
  pid_r.reset();
  pid_d.reset();
  pid_a.reset();

  qramp_d.reset(odo.getValue().coord(0));
  qramp_a.reset(odo.getValue().coord(1));

  pid_ct.reset();
  pid_rt.reset();

  traj.reset();
}

bool toggle = false;

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
  fpga_reset();
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

enum CoordCalib {
  X = 0,
  Y = 1
};

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

  traj_mode = false;

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
    BEGIN_POINT = RED_BEGIN_POINT;
    FIRST_FIRE_POINT = RED_FIRST_FIRE_POINT;
    SECOND_FIRE_POINT = RED_SECOND_FIRE_POINT;
    FRONT_FRESQ_POINT = RED_FRONT_FRESQ_POINT;
    FRESQ_POINT = RED_FRESQ_POINT;

    ADV_SECOND_FIRE_POINT = RED_ADV_SECOND_FIRE_POINT;
    ADV_BEFORE_FIRST_POINT = RED_ADV_BEFORE_FIRST_POINT;
    ADV_FIRST_FIRE_POINT = RED_ADV_FIRST_FIRE_POINT;

    ADV_BEFORE_THIRD_POINT = RED_ADV_BEFORE_THIRD_POINT;
    ADV_THIRD_FIRE_POINT = RED_ADV_THIRD_FIRE_POINT;
    THIRD_FIRE_POINT = RED_THIRD_FIRE_POINT;

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
    BEGIN_POINT = YELLOW_BEGIN_POINT;
    FIRST_FIRE_POINT = YELLOW_FIRST_FIRE_POINT;
    SECOND_FIRE_POINT = YELLOW_SECOND_FIRE_POINT;
    FRONT_FRESQ_POINT = YELLOW_FRONT_FRESQ_POINT;
    FRESQ_POINT = YELLOW_FRESQ_POINT;

    ADV_SECOND_FIRE_POINT = YELLOW_ADV_SECOND_FIRE_POINT;
    ADV_BEFORE_FIRST_POINT = YELLOW_ADV_BEFORE_FIRST_POINT;
    ADV_FIRST_FIRE_POINT = YELLOW_ADV_FIRST_FIRE_POINT;

    ADV_BEFORE_THIRD_POINT = YELLOW_ADV_BEFORE_THIRD_POINT;
    ADV_THIRD_FIRE_POINT = YELLOW_ADV_THIRD_FIRE_POINT;
    THIRD_FIRE_POINT = YELLOW_THIRD_FIRE_POINT;

  }
}

void print_pos(void) {
  io << "(x " << pos.getValue().coord(0) << ", ";
  io << "y " << pos.getValue().coord(1) << ")\n";
}

#include <device/eirbot2014/rds.hpp>

Rds rds("", rds_io);

void check_for_collision(void) {
  rds.update();
  List<2, Vect<2, s32> > adv = rds.getValue();
  bool can_unlock = true;
  for (int i=0; i<adv.usedSpace(); i++) {
    if (adv.get(i).coord(0) < 60 &&
	(adv.get(i).coord(1) < 40 || adv.get(i).coord(1) > 320)) {
      robot.lock();
      can_unlock = false;
    }
  }
  if (can_unlock) {
    trajectory_reset();
    robot.unlock();
  }
}

int main(int argc, char* argv[]) {
  (void)argc;
  (void)argv;

  // Need to be the first thing to be done (for security)
  fpga_init();
  MOT_R = 0;
  MOT_L = 0;

  Aversive::init();
  asserv_init();

  robot.lock();

  //io.setMode(Stream::BINARY);
  rds_io.setMode(Stream::BINARY);
  file.setMode(Stream::BINARY);
  
  cmd.coord(0) = 0;
  cmd.coord(1) = 0;

  control_init();

  mot_l.inverse();
  enc_r.inverse();
    
  pid_ct.setGains(1000, 50, 100);
  pid_ct.setMaxIntegral(25600);
  pid_ct.setOutShift(10);

  pid_rt.setGains(5, 0, 0);
  pid_rt.setMaxIntegral(100000);
  pid_rt.setOutShift(14);

  robot.unlock();

  qramp_a.setFirstOrderLimit(15,15);
  qramp_a.setSecondOrderLimit(2,2);

  qramp_d.setFirstOrderLimit(5,5);
  qramp_d.setSecondOrderLimit(1,2);

  traj.setMode(TrajectoryManager::FASTER);
  
  s16 dummy = 0;
  io << "Side \n";
  io >> dummy;
  io << dummy << "\n";

  side_init(dummy==1);
  match_init(dummy==1);

  robot.lock();

  io << "Place me please <3\n";
  io >> dummy;

  // while(!dummy) {
  //   io << "GO ?\n";
  //   io >> dummy;
  //   io << dummy << "\n";
    
  //   Vect<2, s32> adv_pos(0,0);

  //   s16 go = 0;
  //   while(!go) {
  //     rds.update();
  //     List<2, Vect<2, s32> > adv = rds.getValue();

  //     io << adv.usedSpace() << "\n";
  //     for(u8 i = 0 ; i < adv.usedSpace() ; i++) {
  //   	io << "dist " << adv.get(i).coord(0) << "\n";
  //   	io << "angle " << adv.get(i).coord(1) << "\n";
  //   	s32 abs_angle = 180 - adv.get(i).coord(1) + (odo.getValue().coord(1) >> 4);
  //   	s32 rel_x = (s32)(adv.get(i).coord(0) * Math::cos<Math::DEGREE, double>(abs_angle));
  //   	s32 rel_y = (s32)(adv.get(i).coord(0) * Math::sin<Math::DEGREE, double>(abs_angle));
	
  //   	s32 abs_x = pos.getValue().coord(0) + rel_x * 10;
  //   	s32 abs_y = pos.getValue().coord(1) + rel_y * 10;

  //   	io << "abs angle " << abs_angle << "\n";


  //   	io << "rel x " << rel_x << "\n";
  //   	io << "rel y " << rel_y << "\n";

  //   	io << "my x " << pos.getValue().coord(0) << "\n";
  //   	io << "my y " << pos.getValue().coord(1) << "\n";

  //   	io << "x " << abs_x << "\n";
  //   	io << "y " << abs_y << "\n";
  //   	adv_pos.coord(0) = abs_x;
  //   	adv_pos.coord(1) = abs_y;
  //     }

  //     io << "Go ?\n";
  //     io >> go;
  //     io << go << "\n";

  //   }


  //   World<WORLD_SIZE, AABB> world;
  //   Circle adv_shape(adv_pos, 200);
  //   //Circle adv_shape(Vect<2, s32>(0, 400), 200);
  //   world.addShape(&adv_shape);

  //   Astar astar(42, world);
  //   Vect<2, s32> *path;
  //   s32 targetX;
  //   s32 targetY;
  //   s32 nextX;
  //   s32 nextY;

  //   dummy = 0;
  //   while (!dummy) {
  //     io << "choose X then Y\n";
  //     io >> targetX;
  //     io >> targetY;
  //     io << "going to " << targetX << " " << targetY << "\n";
  //     path = astar.getTrajectory(pos.getValue(), Vect<2, s32>(targetX, targetY));
  //     io << "GO ?\n";
  //     io >> dummy;
  //   }

  //   io << "path length " << astar.getPathLengh() << "\n";
  //   for (uint8_t i = astar.getPathLengh(); i>0; i--) {
  //     nextX = path[i-1][0];
  //     nextY = path[i-1][1];
  //     io << "goto " << nextX << " " << nextY << " ?\n";
  //     robot.unlock();
  //     traj.gotoPosition(Vect<2, s32>(nextX, nextY));
  //     while(!traj.isEnded());
  //     robot.lock();
  //   }

  //   world.removeShape(&adv_shape);
  // }
  
  // while(1);

  traj.setMode(TrajectoryManager::FORWARD);
  traj.reset();
  trajectory_reset();
  robot.unlock();

  // qramp_a.setFirstOrderLimit(30, 30);
  // qramp_a.setSecondOrderLimit(4,4);

  // qramp_d.setFirstOrderLimit(10,10);
  // qramp_d.setSecondOrderLimit(2,2);

  qramp_a.setFirstOrderLimit(70, 70);
  qramp_a.setSecondOrderLimit(4,4);

  qramp_d.setFirstOrderLimit(35,35);
  qramp_d.setSecondOrderLimit(2,2);

  traj.gotoDistance(300);
  while(!traj.isEnded()) {
    check_for_collision();
  }

  // traj.setMode(TrajectoryManager::FASTER);
  
  // while(1) {
  //   traj.gotoPosition(Vect<2, s32>(-200 - 700, -200));
  //   while(!traj.isEnded());

  //   print_pos();
    
  //   traj.gotoPosition(Vect<2, s32>(-200 - 700, 200));
  //   while(!traj.isEnded());
    
  //   print_pos();

  //   traj.gotoPosition(Vect<2, s32>(200 - 700, 200));
  //   while(!traj.isEnded());
    
  //   print_pos();

  //   traj.gotoPosition(Vect<2, s32>(200 - 700, -200));
  //   while(!traj.isEnded());

  //   print_pos();
  // }

  traj.setMode(TrajectoryManager::FASTER);
  traj.gotoPosition(FIRST_FIRE_POINT);
  while(!traj.isEnded()) {
    check_for_collision();
  }

  traj.gotoPosition(BEGIN_POINT);
  while(!traj.isEnded()) {
    check_for_collision();
  }

  // traj.gotoPosition(SECOND_FIRE_POINT);
  // while(!traj.isEnded());  

  traj.gotoPosition(FRONT_FRESQ_POINT);
  while(!traj.isEnded()) {
    check_for_collision();
  }

  qramp_a.setFirstOrderLimit(20,20);
  qramp_a.setSecondOrderLimit(1,1);

  qramp_d.setFirstOrderLimit(10,10);
  qramp_d.setSecondOrderLimit(1,1);  

  traj.setMode(TrajectoryManager::BACKWARD);
  traj.gotoPosition(FRESQ_POINT);
  while(!robot.getValue());

  pos.setY((table_height - robot_height)/2 + center_y_offset);

  traj.setMode(TrajectoryManager::FASTER);
  traj.gotoDistance(300);
  while(!traj.isEnded()) {
    robot.unlock();
  }

  qramp_a.setFirstOrderLimit(70, 70);
  qramp_a.setSecondOrderLimit(8,8);
  
  qramp_d.setFirstOrderLimit(35,35);
  qramp_d.setSecondOrderLimit(2,3);

  print_pos();

  traj.setMode(TrajectoryManager::FASTER);
  traj.gotoPosition(FRONT_FRESQ_POINT);
  while(!traj.isEnded()) {
    check_for_collision();
  }

  // io << "front fresq -->";
  // print_pos();

  // traj.setMode(TrajectoryManager::FASTER);
  // traj.gotoPosition(ADV_SECOND_FIRE_POINT);
  // while(!traj.isEnded()) {
  //   if(robot.getValue()) {
  //     io << "skating !! -->";
  //     print_pos();
  //   }
  // }
  
  // io << "second fire -->";
  // print_pos();

  traj.gotoPosition(ADV_BEFORE_FIRST_POINT);
  while(!traj.isEnded()) {
    check_for_collision();
    if(robot.getValue()) {
      io << "skating !! -->";
      print_pos();
    }
  }

  // traj.gotoPosition(ADV_FIRST_FIRE_POINT);
  // while(!traj.isEnded());

  traj.gotoPosition(ADV_BEFORE_THIRD_POINT);
  while(!traj.isEnded()) {
    check_for_collision();
  }

  // traj.gotoPosition(ADV_THIRD_FIRE_POINT);
  // while(!traj.isEnded());

  traj.gotoPosition(THIRD_FIRE_POINT);
  while(!traj.isEnded()) {
    check_for_collision();
  }


  traj.gotoPosition(BEGIN_POINT);
  while(!traj.isEnded()) {
    check_for_collision();
  }
  
  while(1);

  // World<WORLD_SIZE, AABB> world;
  // Astar astar(42, world);
  // Vect<2, s32> *path;
  // s32 targetX;
  // s32 targetY;
  // s32 nextX;
  // s32 nextY;
  // while(Aversive::sync()) {
  //   dummy = 0;
  //   while (!dummy) {
  //     io << "choose X then Y\n";
  //     io >> targetX;
  //     io >> targetY;
  //     io << "going to " << targetX << " " << targetY << "\n";
  //     path = astar.getTrajectory(pos.getValue(), Vect<2, s32>(targetX, targetY));
  //     io << "GO ?\n";
  //     io >> dummy;
  //   }

  //   for (uint8_t i = astar.getPathLengh(); i>0; i--) {
  //     nextX = path[i-1][0];
  //     nextY = path[i-1][1];
  //     io << "goto " << nextX << " " << nextY << " ?\n";
  //     robot.unlock();
  //     traj.gotoPosition(Vect<2, s32>(nextX, nextY));
  //     while(!traj.isEnded());
  //     robot.lock();
  //   }
  // }

  Aversive::setReturnCode(0);
  return Aversive::exit();
}
