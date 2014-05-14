#include "avoidance.hpp"
#include "rds.hpp"
#include "devices.hpp"
#include "trajectory.hpp"

World<WORLD_SIZE, AABB> world;
Astar astar(42, world);

void avoidance_init(void) {
  fill_world(world);
}

void avoidance_goto(const Vect<2, s32>& target) {
  Vect<2, s32> *path;
  io << "begin avoidance\n";

  path = astar.getTrajectory(pos.getValue(), (Vect<2, s32>)target);

  io << "path length " << astar.getPathLengh() << "\n";
  for (uint8_t i = astar.getPathLengh(); i>0; i--) {
    io << "goto " << path[i-1].coord(0) << " " << path[i-1].coord(1) << "...\n";
    traj.gotoPosition(path[i-1]);
    bool keep_going = true;
    bool collision = false;
    while(!traj.isEnded() && keep_going) {
      if (check_for_collision()) {
	if (world.collide(Segment(pos.getValue(), path[i-1]))) {
	  keep_going = false;
	  collision = true;
	}
	for (uint8_t j=i-1; i>0; i--) {
	  if (world.collide(Segment(path[j], path[j-1]))) {
	    keep_going = false;
	    collision = true;
	    break;
	  }
	}
      }
    }
    // WARNING a changer de place
    if (collision) {
      traj.reset();
      //à modifier plus tard, mais correct pour premiers tests
      avoidance_goto(target);
    }
  }

  io << "DONE !\n";
}
