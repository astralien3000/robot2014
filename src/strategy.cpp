#include "strategy.hpp"
#include "trajectory.hpp"
#include "devices.hpp"
#include "action.hpp"
#include "my_world.hpp"
#include "avoidance.hpp"
#include "rds.hpp"
#include "paint_action.hpp"
#include "master_action.hpp"
#include "asserv.hpp"

State state;
extern List<20, Action*> actions;
extern PaintAction paint_action;
extern MasterAction red_top_fire_action;
extern MasterAction yellow_top_fire_action;
Action* current_action;

static void print_pos(void) {
  io << "(x " << pos.getValue().coord(0) << ", ";
  io << "y " << pos.getValue().coord(1) << ")\n";
}


inline void handler_begin(void) {  
  asserv_speed_fast();
  asserv_lockmode_activ();

  if (Action::side == RED) {
    io << "going to yellow fire !\n";
    traj.gotoPosition(yellow_top_fire.p());
  } else {
    io << "going to red fire !\n";
    traj.gotoPosition(red_top_fire.p());
  }
  while (!traj.isEnded()) {
    if(check_for_collision()) {
      robot.lock();
      traj.reset();
      state = SEARCH_ACTION;
      robot.unlock();
      return;
    }
    //and make sure ennemy is not boring
    print_pos();
  }
  // mettre le feu à done
  red_top_fire_action.done();
  yellow_top_fire_action.done();

  traj.gotoPosition(Vect<2, s32>(0, 450));
  while (!traj.isEnded()) {
    //hum hum
  }

  asserv_speed_normal();

  traj.gotoPosition(paint_action.controlPoint());
  while (!traj.isEnded()) {
    //pareil
  }

  paint_action.doAction();
  traj.gotoPosition(Vect<2, s32>(0, 450));
  while (!traj.isEnded()) {
    //pareil
  }
  traj.gotoPosition(Vect<2, s32>(-300, 450));
  while (!traj.isEnded()) {
    //pareil
  }
  
  state = SEARCH_ACTION;
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
  if (max_prio > 0) {
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
    enum Error ret = avoidance_goto(current_action->controlPoint());
    if (ret == SUCCESS) {
      state = DO_ACTION;
      return;
    }
    if (ret == SKATING) {
      state = SKATED;
      return;
    }

    io << "try again\n";
    robot.lock();
    tries++;
  }
  io << "game over\n";
  //il faut baisser la priorité de l'action en cours.
  current_action->resetPriority();
  state = SEARCH_ACTION;
}

inline void handler_do(void) {
  enum Error ret = current_action->doAction();
  if (ret == IMPOSSIBLE) {
    robot.lock();
    traj.reset();
    robot.unlock();
    io << "action impossible : someone detected\n";
  }
  if (ret == SKATING) {
    state = SKATED;
    return;
  }
  state = SEARCH_ACTION;
}

inline void handler_skating(void) {
  io << "SKAAAATING\n";
  
  s32 dist = 0;
  
  if(traj.isBackward()) {
    dist = 200;
  }
  else {
    dist = -200;
  }

  io << "Trying to go far\n";
  traj.gotoDistance(dist);
  for(u8 i = 0 ; i < 3 ; i++) {
    while(!traj.isEnded() || robot.getValue()) {
      
    }

    if(robot.getValue()) {
      robot.unlock();
    }
    else {
      current_action->resetPriority();
      state = SEARCH_ACTION;
      return;
    }
  }
    
  io << "FAIL...\n";
  while(1);
}


void do_your_job(void) {
  state = SEARCH_ACTION;
  //state = BEGIN;
  //io << "state is " << state << "\n";
  while(1) {
    if (state == BEGIN) {
      handler_begin();
    } else if (state == SEARCH_ACTION) {
      handler_search();
    } else if (state == REACH_ACTION) {
      handler_reach();
    } else if (state == DO_ACTION) {
      handler_do();
    } else if (state == SKATED) {
      handler_skating();
    }
  }
}
