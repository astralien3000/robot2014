#include <aversive.hpp>

#include "filters.hpp"
#include "asserv.hpp"
#include "eirbot_shell.hpp"
#include "devices.hpp"
#include <hardware/interrupts.hpp>
#include <system/scheduler.hpp>
#include "fpga.hpp"

#include "curv_trajectory_manager.hpp"

#include "strategy.hpp"

#define F_CPU 16000000l
#include <util/delay.h>

Scheduler& sched = Scheduler::instance();

PidFilter pid_rt;
PidFilter pid_ct;
CurvTrajectoryManager traj(robot, odo, pos, pid_rt, pid_ct);

FpgaUartStream rds_stream("rds_stream", UART_TX_1_DATA, UART_TX_1_OCUP, UART_RX_1_DATA, UART_RX_1_AVA);

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

  t.setPeriod(8000);
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

  s32 cur_dist = odo.getValue().coord(0);

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

void goto_wall(void) {
  io << "goto wall\n";

  traj.gotoDistance(-3000);

  while(!robot.getValue());

  io << "wall touched\n";
  
  traj.gotoDistance(-20);

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
  goto_wall();
  io << "Reset X and angle\n";
  set_angle(A_INIT);
  pos.setX(X_INIT);
  
  io << "Go far from the wall\n";
  traj.gotoDistance(700);
  while(!traj.isEnded()) {
    robot.unlock();
  }

  io << "Face buffet\n";
  traj.gotoAngle(-90);
  while(!traj.isEnded());

  io << "Goto buffet\n";
  goto_wall();
  
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
    A_INIT = RED_A_INIT;
    X_INIT = RED_X_INIT;
    Y_INIT = RED_Y_INIT;
    ANGLE_CALIB = RED_ANGLE_CALIB;
  }
  else {
    io << "Init YELLOW side\n";
    A_INIT = YELLOW_A_INIT;
    X_INIT = YELLOW_X_INIT;
    Y_INIT = YELLOW_Y_INIT;
    ANGLE_CALIB = YELLOW_ANGLE_CALIB;
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
  rds_stream.setMode(Stream::BINARY);
  file.setMode(Stream::BINARY);
  
  cmd.coord(0) = 0;
  cmd.coord(1) = 0;

  control_init();

  mot_l.inverse();
  enc_r.inverse();
    
  pid_ct.setGains(1000, 50, 100);
  pid_ct.setMaxIntegral(25600);
  pid_ct.setOutShift(10);

  pid_rt.setGains(20, 0, 0);
  pid_rt.setMaxIntegral(100000);
  pid_rt.setOutShift(14);

  robot.unlock();

  // Init
  //pwm bas 1300
  //pwm haut 490

  // while(1) {
  //   io << "pos " << pos.getValue().coord(0) << " " << pos.getValue().coord(1) << "\n";
  // }

  qramp_a.setFirstOrderLimit(15,15);
  qramp_a.setSecondOrderLimit(2,2);

  qramp_d.setFirstOrderLimit(5,5);
  qramp_d.setSecondOrderLimit(1,2);

  traj.setMode(TrajectoryManager::FASTER);

  s16 dummy = 0;
  while(!dummy) {
    io << "GO ?\n";
    io >> dummy;
  }

  side_init(true);
  match_init(true);
  
  robot.lock();
  io << "Place me please <3\n";
  io >> dummy;

  qramp_a.setFirstOrderLimit(30, 30);
  qramp_a.setSecondOrderLimit(4,4);

  qramp_d.setFirstOrderLimit(10,10);
  qramp_d.setSecondOrderLimit(2,2);

  trajectory_reset();
  robot.unlock();

  traj.gotoPosition(BEGIN_POINT);
  while(!traj.isEnded());

  traj.gotoPosition(FIRST_FIRE_POINT);
  while(!traj.isEnded());

  traj.gotoPosition(BEGIN_POINT);
  while(!traj.isEnded());

  traj.gotoPosition(SECOND_FIRE_POINT);
  while(!traj.isEnded());  

  traj.gotoPosition(FRONT_FRESQ_POINT);
  while(!traj.isEnded());  

  traj.setMode(TrajectoryManager::BACKWARD);
  traj.gotoPosition(FRESQ_POINT);
  while(!robot.getValue());  

  traj.setMode(TrajectoryManager::FASTER);
  traj.gotoDistance(500);
  while(!traj.isEnded()) {
    robot.unlock();
  } 

  // set_angle(90);
  // traj.setMode(TrajectoryManager::BACKWARD);
  // traj.gotoPosition(Vect<2, s32>(500, 0));
  // while(!traj.isEnded());
  

  // traj.setMode(TrajectoryManager::FASTER);
  // s32 x = 0;
  // s32 y = 0;
  // while(1) {
  //   s16 dummy = 0;
  //   while(!dummy) {
  //     io << "GO ?\n";
  //     io >> dummy;
  //     io << dummy << "\n";
  //     io << "pos " << pos.getValue().coord(0) << " " << pos.getValue().coord(1) << "\n";
  //   }

  //   io << "X[" << x << "] = ";
  //   io >> x;
  //   io << x << "\n";

  //   io << "Y[" << y << "] = ";
  //   io >> y;
  //   io << y << "\n";

  //   traj.gotoPosition(Vect<2, s32>(x, y));

  //   while(!traj.isEnded()) {
  //     //io << MOT_L << " " << MOT_R << "\n";
  //     io << "pos " << pos.getValue().coord(0) << " " << pos.getValue().coord(1) << "\n";
  //   }
  // }

  // s16 dummy = 0;
  // while(!dummy) {
  //   io << "GO ?\n";
  //   io >> dummy;
  // }

  // traj.setMode(TrajectoryManager::FORWARD);
  // traj.gotoCurvPosition(Vect<2, s32>(0, 500), 0, false);
  // //traj.gotoPosition(Vect<2, s32>(0, 500));
  // while(!traj.isEnded());

  // io << "pos " << pos.getValue().coord(0) << " " << pos.getValue().coord(1) << "\n";

  // traj.setMode(TrajectoryManager::BACKWARD);
  // traj.gotoCurvPosition(Vect<2, s32>(0, 0), 0, true);
  // //traj.gotoPosition(Vect<2, s32>(0, 500));
  // while(!traj.isEnded());

  // io << "pos " << pos.getValue().coord(0) << " " << pos.getValue().coord(1) << "\n";
  // while(1) {
  //   io >> dummy;
  //   io << "angle " << (odo.getValue().coord(0) >> 4) << "\n";
  // }

  // traj.setMode(TrajectoryManager::BACKWARD);
  // traj.gotoCurvPosition(Vect<2, s32>(1250, -250), -250, false);
  // while(!traj.isEnded());

  // traj.setMode(TrajectoryManager::BACKWARD);
  // traj.gotoPosition(Vect<2, s32>(1250, -600));
  // while(!traj.isEnded());

  // traj.gotoPosition(Vect<2, s32>(-400, -400));
  // while(!traj.isEnded());

  // traj.gotoPosition(Vect<2, s32>(-400, 0));
  // while(!traj.isEnded());

  // traj.gotoPosition(Vect<2, s32>(0, -400));
  // while(!traj.isEnded());

  // traj.gotoPosition(Vect<2, s32>(0, 0));
  // while(!traj.isEnded());

  /// Test trajectory
  // robot.lock();
  // while(1) {
  //   dummy = 0;
  //   while(!dummy) {
  //     io << "GO ?\n";
  //     io >> dummy;
  //     io << pos.getValue().coord(0) << " " << pos.getValue().coord(1) << "\n";
  //   }
    
  //   trajectory_reset();
  //   robot.unlock();
  //   traj.gotoPosition(Vect<2, s32>(0, 0));
  //   while(!traj.isEnded()) {
  //     //io << pos.getValue().coord(0) << " " << pos.getValue().coord(1) << "\n";
  //   }
  //   robot.lock();
  // }

  /// Test asserv
  // while(1) {
  //   dummy = 0;
  //   while(!dummy) {
  //     cmd_l = 0;
  //     cmd_r = 0;

  //     io << "GO ?\n";
  //     io >> dummy;
  //     io << pid_l.lastOut() << "\n";
  //   }
  //   io << "OK GO ! GO ! GO !\n";
    
  //   if(dummy == 1) {
  //     cmd_l = 1000;
  //     cmd_r = 1000;
  //     _cmd = Vect<2,s32>(500, 0);
  //   }    
  //   else if(dummy == 2) {
  //     cmd_l = -1000;
  //     cmd_r = -1000;
  //     _cmd = Vect<2,s32>(0, 0);
  //   }    
  //   else if(dummy == 3) {
  //     _cmd = Vect<2,s32>(0, -900);
  //   }    
  //   else if(dummy == 4) {
  //     _cmd = Vect<2,s32>(0, 900);
  //   }    
  //   else if(dummy == 5) {
  //     _cmd = Vect<2,s32>(400, 900);
  //   }    

  //   dummy = 0;
  //   io << "STOP ?\n";
  //   io >> dummy;
  // }

  // u16 pwm = 0;
  // while(1) {
  //   io << "pwm ?\n";
  //   io >> pwm;
  //   io << pwm << "\n";
  //   SERVO1 = pwm;
  //   SERVO2 = pwm;
  //   SERVO3 = pwm;
  //   SERVO4 = pwm;
  //   SERVO5 = pwm;
  //   SERVO6 = pwm;
  //   SERVO7 = pwm;
  //   SERVO8 = pwm;
  //   SERVO9 = pwm;
  //   SERVO10 = pwm;
  //   SERVO11 = pwm;
  //   SERVO12 = pwm;
  //   SERVO13 = pwm;
  //   SERVO14 = pwm;
  //   SERVO15 = pwm;
  //   SERVO16 = pwm;
  // }

  // while(1) {
  //   u8 c = 0;
  //   //while(!UART_RX_1_AVA);
  //   //c = UART_RX_1_DATA;
  //   rds_stream >> c;

  //   rds_stream << c << "\n";
  //   //UART_TX_1_DATA = c+1;
  //   //_delay_ms(500);
  // }

  // /// Test RDS
  // while(1) {
  //   u8 rds_num = 0;
  //   u8 rds_angle[2] = {0,0};
  //   u8 rds_dist[2] = {0,0};
  //   s16 angle[2] = {0,0};
  //   s16 dist[2] = {0,0};

  //   Uart<0>::instance().send('p');

  //   Uart<0>::instance().recv(rds_num);
  //   for(u8 nb=0; nb<rds_num; nb++) {
  //     Uart<0>::instance().recv(rds_angle[nb]);
  //     Uart<0>::instance().recv(rds_dist[nb]);
  //   }
  //   bool send = false;
  //   if (send) {
  //     _delay_ms(2);
  //     Uart<0>::instance().send(rds_num);
  //     _delay_ms(2);
  //   }
    
  //   if(rds_num) {
  //     if (send) {
  // 	Uart<0>::instance().send(rds_angle[0]);
  // 	_delay_ms(2);
  // 	Uart<0>::instance().send(rds_dist[0]);
  // 	_delay_ms(2);
  //     }

      
  //     angle[0] = (((s16)rds_angle[0]) * 2);
  //     if(180 < angle[0]) {
  // 	angle[0] -= 360;
  //     }
  //     dist[0] = rds_dist[0];
  //     u8 target = 0;
  //     if (rds_num == 2) {
  // 	angle[1] = (((s16)rds_angle[1]) * 2);
  // 	if(180 < angle[1]) {
  // 	  angle[1] -= 360;
  // 	}
  // 	dist[1] = rds_dist[1];
  // 	if (rds_dist[1] > rds_dist[0]) {
  // 	  target = 1;
  // 	}
  //     }
      
  //     _cmd.coord(1) += angle[target]*10;
  //     while((odo.getValue().coord(1)/64) != (_cmd.coord(1)/64));
  //     _cmd.coord(0) += dist[target]*10 -400;
  //     while((odo.getValue().coord(0)/256) != (_cmd.coord(0)/256));
  //     _delay_ms(500);
  //   }
  // }

  // s16 side = 0;
  // while(!dummy) {
  //   io << "side ?\n";
  //   io >> side;
  //   io << "side : " << side << "\n";
  //   side_init(side);
  //   io << "begin calibrate ?\n";
  //   io >> dummy;
  // }

  // qramp_a.setFirstOrderLimit(5,5);
  // qramp_d.setFirstOrderLimit(5,5);
  
  // match_init(side);

  // pid_d.setMaxIntegral(0);
  // pid_a.setMaxIntegral(0);

  // dummy = 0;
  // while(dummy) {
  //   io >> dummy;
  // }

  // robot.lock();
  // // The user can place the robot at the spawn
  // io << "Place me please <3 \n";
  // dummy = 0;

  // while(!dummy) {
  //   io >> dummy;
  //   io << _cmd.coord(0) << " " << _cmd.coord(1) << "\n";
  //   _cmd = odo.getValue();
  // }

  // traj.reset();
  // traj_mode = true;
  // pid_a.reset();
  // pid_d.reset();
  // robot.unlock();

  // qramp_a.setFirstOrderLimit(10,10);
  // qramp_d.setFirstOrderLimit(10,10);

  // traj.gotoPosition(Vect<2, s32>(-800, 200));
  // robot.unlock();
  // while(!traj.isEnded()) {
  //   robot.unlock();
  // }

  // pid_d.setMaxIntegral(1000);
  // pid_a.setMaxIntegral(1000);

  // traj.gotoPosition(Vect<2, s32>(0, 450));
  // while(!traj.isEnded());

  // traj.gotoPosition(Vect<2, s32>(0, 1200));
  // while(!robot.getValue());

  // traj_mode = false;
  // s32 new_dist = odo.getValue().coord(0) - 200;
  // _cmd = Vect<2, s32>(new_dist, odo.getValue().coord(1));

  // while(odo.getValue().coord(0) > new_dist + 150) {
  //   robot.unlock();
  // }

  // while(odo.getValue().coord(0) > new_dist + 20);

  // traj.reset();
  // traj_mode = true;

  // traj.gotoPosition(Vect<2, s32>(-700, 0));

  while(1);

  // while(1) {
  //   s16 dummy = 0;
  //   while(!dummy) {
  //     io >> dummy;
  //   }

  //   traj.gotoPosition(Vect<2, s32>(1500, 0));
  //   while(!traj.isEnded());
  //   traj.gotoPosition(Vect<2, s32>(2000, -500), -500);
  //   while(!traj.isEnded());
  //   traj.gotoPosition(Vect<2, s32>(2000, -1000));
  //   while(!traj.isEnded());
  //   traj.gotoPosition(Vect<2, s32>(500, -1000));
  //   while(!traj.isEnded());
  //   traj.gotoPosition(Vect<2, s32>(0, -500), -500);
  //   while(!traj.isEnded());
  //   traj.gotoPosition(Vect<2, s32>(0, 0));
  // }

  while(Aversive::sync()) {
    if(traj.isEnded()) {
      //traj.gotoPosition(Vect<2, s32>(2000, -1000));
    }

    // if(traj.isEnded()) {
    //   if(toggle) {
    // 	traj.gotoPosition(Vect<2, s32>(0, 0), 0);
    //   }
    //   else {
    // 	traj.gotoPosition(Vect<2, s32>(0, -800), 0);
    //   }
    //   toggle = !toggle;
    //   //cmd_trajectory();
    // }

    //cmd_print_infos();
    //cmd_print_pos();
    //cmd_pid_set();
    //cmd_dist_angle();
    //cmd_odo_config();
    
    if(robot.getValue()) {
      io << "Skating !!! \n";
      s32 dummy;
      io >> dummy;
      robot.unlock();
    }
  }

  Aversive::setReturnCode(0);
  return Aversive::exit();
}
