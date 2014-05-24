#include "avoidance.hpp"
#include "rds.hpp"
#include "devices.hpp"
#include "trajectory.hpp"
#include <geometry/segment.hpp>
#include <geometry/circle.hpp>

#define ROBOT_RADIUS 200

World<WORLD_SIZE, AABB> world;
World<2, Circle> mini_world;
Astar astar(42, world);

void avoidance_init(void) {
  fill_world(world);
}

bool check_collision_on_trajectory(Vect<2, s32> source, Vect<2, s32> target) {
  //TEST AVEC COLLISION SUR SEGMENTS QUI ENCADRENT LA TRAJECTOIRE
  s32 x1, y1, x2, y2, dx, dy;
  Segment seg;
  Circle cir;
  
  cir = Circle(target, ROBOT_RADIUS);
  if (mini_world.collide(cir))
    return true;

  dx = target.coord(0) - source.coord(0);
  dy = target.coord(1) - source.coord(1);

  s32 norm = (Vect<2, s32>(dx, dy)).norm();

  dx = dx * ROBOT_RADIUS / norm;
  dy = dy * ROBOT_RADIUS / norm;

  x1 = source.coord(0) - dy;
  y1 = source.coord(1) + dx;
  x2 = target.coord(0) - dy;
  y2 = target.coord(1) + dx;
  seg = Segment(x1, y1, x2, y2);
  if (mini_world.collide(seg))
    return true;
  
  x1 = source.coord(0) + dy;
  y1 = source.coord(1) - dx;
  x2 = target.coord(0) + dy;
  y2 = target.coord(1) - dx;
  seg = Segment(x1, y1, x2, y2); 
  if (mini_world.collide(seg))
    return true;
  
  return false;
}

enum AvoidanceError avoidance_goto(const Vect<2, s32>& target) {
  Vect<2, s32> *path;
  io << "begin avoidance\n";

  update_world();
  io << "BEFORE ASTAR : going from : " << pos.getValue()[0] << " " << pos.getValue()[1] << " to " << target[0] << " " << target[1] << "\n";
  path = astar.getTrajectory(pos.getValue(), (Vect<2, s32>)target);

  io << "AFTER ASTAR : going from : " << pos.getValue()[0] << " " << pos.getValue()[1] << " to " << target[0] << " " << target[1] << "\n";

  if (! astar.isPathEnded()) {
    io << "failed to find path\n";
    return IMPOSSIBLE;
  }
  io << "path length " << astar.getPathLengh() << "\n";
  for (uint8_t i = astar.getPathLengh(); i>0; i--) {
    io << "goto " << path[i-1].coord(0) << " " << path[i-1].coord(1) << "...\n";
    traj.gotoPosition(path[i-1]);
    while(!traj.isEnded()) {

      if (robot.getValue()) {
	return SKATING;
      }

      if (update_world()) {
	io << "DETECTED !\n";
	
	//premier segment
	if (check_collision_on_trajectory(pos.getValue(), path[i-1])) {
	  robot.lock();
	  traj.reset();
	  //trajectory_reset();
	  robot.unlock();
	  return IMPOSSIBLE;
	}
	
	//segments suivants
	//for (uint8_t j = i-1; j>0; j--) {
	// uint8_t j = i-1;
	// if (check_collision_on_trajectory(path[j], path[j-1])) {
	//   robot.lock();
	//   traj.reset();
	//   //trajectory_reset();
	//   robot.unlock();
	//   return false;
	// }
	//}
      }
    }
  }

  io << "DONE !\n";
  return SUCCESS;
}
