#ifndef AVOIDANCE_HPP
#define AVOIDANCE_HPP

#include "my_world.hpp"
#include "astar.hpp"
#include "action.hpp"

extern World<WORLD_SIZE, AABB> world;
extern Astar astar;

// enum AvoidanceError {
//   SUCCESS,
//   IMPOSSIBLE,
//   SKATING,
//   MAX_AVOIDANCE_ERROR
// };

void avoidance_init(void);
enum Error avoidance_goto(const Vect<2, s32>&);

#endif//AVOIDANCE_HPP
