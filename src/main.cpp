#include <aversive.hpp>

#include "filters.hpp"
#include "asserv.hpp"
#include "eirbot_shell.hpp"
#include "devices.hpp"
#include <hardware/interrupts.hpp>
#include <system/scheduler.hpp>
#include "fpga.hpp"
#include "astar.hpp"
#include "calibrate.hpp"
#include "trajectory.hpp"
#include "rds.hpp"
#include "avoidance.hpp"

#include "curv_trajectory_manager.hpp"

#include "master_action.hpp"
#include "harvest_action.hpp"
#include "hunt_action.hpp"
#include "paint_action.hpp"
#include "strategy.hpp"

List<20, Action*> actions;
PaintAction paint_action;
MasterAction act1(yellow_top_fire.p(), 180);
HarvestAction act2(red_tree.centre(), -90);
HuntAction act3(Vect<2, s32>(800, 400), 4);

#define F_CPU 16000000l
#include <util/delay.h>

static Scheduler& sched = Scheduler::instance();

void print_pos(void) {
  io << "(x " << pos.getValue().coord(0) << ", ";
  io << "y " << pos.getValue().coord(1) << ")\n";
}

void print_sp(void) {
  io << "STACK POINTER = " << SPH << " " << SPL << "\n";
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

  //rds_init();

  control_init();
  Interrupts::set();

  mot_l.inverse();
  enc_r.inverse();

  //TEST ALARACHE BENOIT LANCEBALLE
  // s16 wait = 0;
  // io << "Enter a number\n";
  // io >> wait;
  // io << "Do action\n";
  // act3.doAction();
  // while(1);

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
  dummy = 2;
  side_init(dummy==1);
  match_init(dummy==1);

  print_pos();

  robot.lock();
  print_sp();
  io << "Place me please <3\n";
  io >> dummy;
  
  traj.setMode(TrajectoryManager::FASTER);
  traj.reset();
  trajectory_reset();
  robot.unlock();

  // qramp_a.setFirstOrderLimit(40,40);
  // qramp_a.setSecondOrderLimit(4,4);

  // qramp_d.setFirstOrderLimit(13,13);
  // qramp_d.setSecondOrderLimit(2,2);

  qramp_a.setFirstOrderLimit(40,40);
  qramp_a.setSecondOrderLimit(4,4);

  qramp_d.setFirstOrderLimit(29,29);
  qramp_d.setSecondOrderLimit(2,2);

  print_pos();
  
  traj.gotoDistance(300);
  while(!traj.isEnded()) {
    //check_for_collision();
  }

  print_pos();

  io << "Conf\n";
  Action::setPositionManager(pos);
  Action::setRobot(robot);
  Action::setTrajectoryManager(traj);
  Action::side = YELLOW;

  // TEST STRATEGY
  // actions.append(&paint_action);
  // actions.append(&act1);
  // actions.append(&act2);
  // actions.append(&act3);
  // do_your_job();

  // TEST BEGIN A FOND !!
  // traj.gotoPosition(Vect<2, s32>(-100, 500));
  // while(!traj.isEnded()) {
  //   if(check_for_collision()) {
  //     robot.lock();
  //     io << "over\n";
  //     while(1);
  //   }
  // }

  // TEST AVOIDANCE / ASTAR
  // while(1) {
  //   io << "goto\n";
  //   avoidance_goto(Vect<2, s32>(1100, -600));
  //   volatile s32 attente = 0;
  //   while (attente < 3000000)
  //     attente++;
  // }

  // io << "Goto\n";
  // traj.gotoPosition(act1.controlPoint());
  // while(!traj.isEnded());

  // print_pos();

  // io << "Do action\n";
  // act1.doAction();
  // io << "DONE\n";

  io << "Goto3\n";
  traj.gotoPosition(act3.controlPoint());
  while(!traj.isEnded());

  io << "Do action3\n";
  act3.doAction();
  io << "DONE3\n";

  io << "Goto paint";
  avoidance_goto(paint_action.controlPoint());
  while(!traj.isEnded());

  io << "Do paint\n";
  paint_action.doAction();
  io << "DONE paint\n";

  while(1);

  Aversive::setReturnCode(0);
  return Aversive::exit();
}
