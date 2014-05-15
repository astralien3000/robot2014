#ifndef AVOIDANCE_HPP
#define AVOIDANCE_HPP

#include "my_world.hpp"
#include "astar.hpp"

extern World<WORLD_SIZE, AABB> world;
extern Astar astar;

void avoidance_init(void);
bool avoidance_goto(const Vect<2, s32>&);

#endif//AVOIDANCE_HPP
