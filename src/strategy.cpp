#include "strategy.hpp"
#include "trajectory.hpp"
#include "devices.hpp"
#include "action.hpp"
#include "my_world.hpp"
#include "avoidance.hpp"
#include "rds.hpp"
#include "paint_action.hpp"

State state;
extern List<20, Action*> actions;
extern PaintAction paint_action;
Action* current_action;

inline void handler_begin(void) {
  traj.gotoDistance(200);
  while (!traj.isEnded()) {
  }
  
  if (Action::side == RED)
    traj.gotoPosition(yellow_top_fire.p());
  else
    traj.gotoPosition(red_top_fire.p());
  while (!traj.isEnded()) {
    //check_for_collision();
    //and make sure ennemy is not boring
  }

  traj.gotoPosition(paint_action.controlPoint());
  while (!traj.isEnded()) {
    //pareil
  }

  paint_action.doAction();
}

inline void handler_search(void) {
  s16 max_prio = -1;
  actions.doForeach(([&max_prio](Action* action) {
	s16 prio = action->priority();
	if (prio > max_prio) {
	  current_action = action;
	  max_prio = prio;
	}
	io << "prio = " << prio << "\n";
      }));
  if (max_prio >= 0) {
    state = REACH_ACTION;
  } else {
    robot.lock();
    io << "ERRRROOOOOOR MONSTER KILL\n";
    while(1);
  }
}

inline void handler_reach(void) {
  io << "reach action\n";
  u8 tries = 0;
  while (tries < 3) {
    robot.unlock();
    if (avoidance_goto(current_action->controlPoint())) {
      state = DO_ACTION;
      return;
    }
    io << "try again\n";
    robot.lock();
    tries++;
  }
  io << "game over\n";
  //il faut baisser la priorité de l'action en cours.
  state = SEARCH_ACTION;
}

inline void handler_do(void) {
  current_action->doAction();
  state = SEARCH_ACTION;
}


void do_your_job(void) {
  state = SEARCH_ACTION;
  while(1) {
    if (state == BEGIN) {
      handler_begin();
    } else if (state == SEARCH_ACTION) {
      handler_search();
    } else if (state == REACH_ACTION) {
      handler_reach();
    } else if (state == DO_ACTION) {
      handler_do();
    }
  }
}
