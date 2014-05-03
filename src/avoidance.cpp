#include "avoidance.hpp"

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
    while(!traj.isEnded());
  }

  io << "DONE !\n";
}
