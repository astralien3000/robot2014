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
#include "servo.hpp"
#include "avoidance.hpp"
#include "secure_timer.hpp"

#include "curv_trajectory_manager.hpp"

#include "master_action.hpp"
#include "harvest_action.hpp"
#include "deposit_action.hpp"
#include "hunt_action.hpp"
#include "paint_action.hpp"
#include "strategy.hpp"

#include <device/servomotor/fpga_servomotor.hpp>
#include <device/other/pin.hpp>

// IHM
FpgaUartStream ihm_io("ihm", UART_TX_2_DATA, UART_TX_2_OCUP, UART_RX_2_DATA, UART_RX_2_AVA);

//

List<20, Action*> actions;

// PAINT
PaintAction paint_action;

// FIRE
MasterAction red_top_fire_action(red_top_fire.p(), 180);
MasterAction red_mid_fire_action(red_mid_fire.p(), 90);
MasterAction red_bot_fire_action(red_bot_fire.p(), 0);

MasterAction yellow_top_fire_action(yellow_top_fire.p(), 180);
MasterAction yellow_mid_fire_action(yellow_mid_fire.p(), -90);
MasterAction yellow_bot_fire_action(yellow_bot_fire.p(), 0);

// TREE
HarvestAction red_tree_action(red_tree.centre(), -90, 10);
HarvestAction yellow_tree_action(yellow_tree.centre(), 90, 0);
HarvestAction left_tree_action(left_tree.centre(), 0, 0);
HarvestAction right_tree_action(right_tree.centre(), 0, 10);

// BASKET
DepositAction basket_action;

// MAMMOUTH (HUNT)
HuntAction red_mammouth_action(Vect<2, s32>(700, 400),Vect<2, s32>(725,1050), 4);
HuntAction yellow_mammouth_action(Vect<2, s32>(-700, 400), Vect<2,s32>(-725,1050), 4);

// MAMMOUTH (CAPTURE)
// TODO : 2

// CALIBRATE
// TODO : 4

#define F_CPU 16000000l
#include <util/delay.h>

static Scheduler& sched = Scheduler::instance();

// void print_pos(void) {
//   io << "(x " << pos.getValue().coord(0) << ", ";
//   io << "y " << pos.getValue().coord(1) << ")\n";
// }

// void print_sp(void) {
//   io << "STACK POINTER = " << SPH << " " << SPL << "\n";
// }

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

  rds_init();
  servo_init();
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

  // TEST SERVO
  // while(1) {
  //   u16 cmd = 0;
  //   io << "Command ?\n";
  //   io >> cmd;
  //   io << cmd << "\n";
  //   // SERVO1 = cmd;
  //   // SERVO2 = cmd;
  //   // SERVO3 = cmd;
  //   // SERVO4 = cmd;
  //   // SERVO5 = cmd;
  //   // SERVO6 = cmd;
  //   // SERVO7 = cmd;
  //   // SERVO8 = cmd;
  //   // SERVO9 = cmd;
  //   // SERVO10 = cmd;
  //   // SERVO11 = cmd;
  //   // SERVO12 = cmd;
  //   // SERVO13 = cmd;
  //   // SERVO14 = cmd;
  //   // SERVO15 = cmd;
  //   // SERVO16 = cmd;

  //   FpgaServomotor<volatile u16, SERVO4_ADDR> servo("basket_servo");
  //   servo.setMinCommand(900);
  //   servo.setMaxCommand(1650);
  //   servo.setValue(cmd);
  // }


  robot.unlock();

  asserv_speed_slow();

  traj.setMode(TrajectoryManager::FASTER);

  Pin<37> tirette("tirette");
  tirette.setMode(PinMode::INPUT);

  s16 dummy = 0;
  
  dummy = 0;
  while(dummy < 100) {
    if(tirette.getValue()) {
      dummy++;
      _delay_ms(5);
    }
    else {
      dummy = 0;
    }
  }

  dummy = 0;
  while(tirette.getValue()) {
    if(ihm_io.inputUsedSpace() > 0) {
      //io << "used sp = " << ihm_io.inputUsedSpace() << "\n";
      u8 c = ihm_io.getValue();
      if(! (c & 1)) {
	dummy = 1;
	//io << "RED SIDE\n";
      }
      else {
	dummy = 2;
	//io << "YELLOW SIDE\n";
      }
    }
  }

  // io << "Side \n";
  // io >> dummy;
  // io << dummy << "\n";
  side_init(dummy==1);
  match_init(dummy==1);

  robot.lock();

  //io << "Conf\n";
  Action::setPositionManager(pos);
  Action::setRobot(robot);
  Action::setTrajectoryManager(traj);
  if (dummy == 1)
    Action::side = RED;
  else
    Action::side = YELLOW;

  //actions.append(&paint_action);

  actions.append(&red_top_fire_action);
  actions.append(&red_mid_fire_action);
  //actions.append(&red_bot_fire_action);

  actions.append(&yellow_top_fire_action);
  actions.append(&yellow_mid_fire_action);
  //actions.append(&yellow_bot_fire_action);

  //actions.append(&red_tree_action);
  //actions.append(&yellow_tree_action);
  // actions.append(&left_tree_action);
  // actions.append(&right_tree_action);

  // actions.append(&basket_action);

  //actions.append(&red_mammouth_action);
  //actions.append(&yellow_mammouth_action);
  
  //io << "Place me please <3\n";
  //io >> dummy;
  dummy = 0;
  while(dummy < 100) {
    if(tirette.getValue()) {
      dummy++;
      _delay_ms(5);
    }
    else {
      dummy = 0;
    }
  }

  while(tirette.getValue());
  
  // Demarrage du compteur des 90s APRES LA TIRETTE
  //secure_timer_init();
  
  traj.setMode(TrajectoryManager::FASTER);
  traj.reset();
  trajectory_reset();
  robot.unlock();

  asserv_speed_normal();

  // traj.gotoDistance(300);
  // while(!traj.isEnded()) {
  //   //check_for_collision();
  // }

  traj.gotoDistance(200);
  while (!traj.isEnded()) {
  }

  //TEST_EVITEMENT
  // u8 number_of_cacul = 0;
  // while (1) {
  //   avoidance_goto(Vect<2, s32>(1000, -200));
  //   io << "NOUVEAU CALCUL : " << number_of_cacul << "\n";
  //   number_of_cacul++;
  // }

  // TEST STRATEGY 
  //io << "Begin Strategy\n";
  do_your_job();

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

  // io << "Goto3\n";
  // traj.gotoPosition(yellow_mammouth_action.controlPoint());
  // while(!traj.isEnded());

  // io << "Do action3\n";
  // yellow_mammouth_action.doAction();
  // io << "DONE3\n";

  // io << "Goto paint";
  // avoidance_goto(paint_action.controlPoint());
  // while(!traj.isEnded());

  // io << "Do paint\n";
  // paint_action.doAction();
  // io << "DONE paint\n";

  while(1);

  Aversive::setReturnCode(0);
  return Aversive::exit();
}
