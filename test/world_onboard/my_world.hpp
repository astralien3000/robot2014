#ifndef MY_WORLD_HPP
#define MY_WORLD_HPP

#include <base/integer.hpp>
#include <world.hpp>
#include <point.hpp>
#include <aabb.hpp>
#include <segment.hpp>
#include <circle.hpp>

static const u8 WORLD_SIZE = 20;

// Fixed table elements
extern Segment top; // Mammoths side
extern Segment bottom; // Hearts of fire side
extern Segment left; // Red side
extern Segment right; // Yellow side

extern AABB red_basket;
extern AABB yellow_basket;

extern Circle center_heart;
extern Circle left_heart;
extern Circle right_heart;

extern AABB fixed_red_torch;
extern AABB fixed_yellow_torch;
extern AABB fixed_left_torch;
extern AABB fixed_right_torch;

extern Circle red_tree;
extern Circle yellow_tree;
extern Circle left_tree;
extern Circle right_tree;

// Movable elements
extern Point red_top_fire;
extern Point red_mid_fire;
extern Point red_bot_fire;
extern Point red_wall_fire;

extern Point yellow_top_fire;
extern Point yellow_mid_fire;
extern Point yellow_bot_fire;
extern Point yellow_wall_fire;

extern Point left_wall_fire;
extern Point right_wall_fire;

extern Circle red_movable_torch;
extern Circle yellow_movable_torch;

// Other robots
extern Circle my_pmi;
extern Circle ennemy_robot;
extern Circle ennemy_pmi;

void fill_world(World<WORLD_SIZE>& world);

#endif//MY_WORLD_HPP
