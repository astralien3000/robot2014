#include "my_world.hpp"

// Fixed table elements
Segment top(-1500, 1050, 1500, 1050);
Segment bottom(-1500, -950, 1500, -950);
Segment left(-1500, 1050, -1500, -950);
Segment right(1500, 1050, 1500, -950);

AABB red_basket(400, 750, 700, 300);
AABB yellow_basket(-1100, 750, 700, 300);

//Circle center_heart(0, 0, 150);
//Circle center_heart(0, 0, 250);
Circle center_heart(0, 0, 170);
Circle left_heart(-1500, -950, 250);
Circle right_heart(1500, -950, 250);

AABB fixed_red_torch(-1500, 181, 22, 138);
AABB fixed_yellow_torch(1478, 181, 22, 138);
AABB fixed_left_torch(-269, -950, 138, 22);
AABB fixed_right_torch(131, -950, 138, 22);

Circle red_tree(-1500, -250, 25);
Circle yellow_tree(1500, -250, 25);
Circle left_tree(-800, -950, 25);
Circle right_tree(800, -950, 25);

// Movable elements
Point red_top_fire(-600, 450);
Point red_mid_fire(-1100, -50);
Point red_bot_fire(-600, -550);
Point red_wall_fire(-1485, 250);

Point yellow_top_fire(600, 450);
Point yellow_mid_fire(1100, -50);
Point yellow_bot_fire(600, -550);
Point yellow_wall_fire(1485, 250);

Point left_wall_fire(-200, -935);
Point right_wall_fire(200, -935);

Circle red_movable_torch(-600, -50, 80);
Circle yellow_movable_torch(600, -50, 80);

// Other robots
Circle my_pmi(0, 1100, 10);
Circle ennemy_robot(-30, 1100, 10);
Circle ennemy_pmi(30, 1100, 10);

void fill_world(World<WORLD_SIZE, AABB>& world) {
  world.addShape(&top);
  world.addShape(&bottom);
  world.addShape(&left);
  world.addShape(&right);
  
  world.addShape(&red_basket);
  world.addShape(&yellow_basket);
  
  world.addShape(&center_heart);
  world.addShape(&left_heart);
  world.addShape(&right_heart);
  
  world.addShape(&fixed_red_torch);
  world.addShape(&fixed_yellow_torch);
  world.addShape(&fixed_left_torch);
  world.addShape(&fixed_right_torch);
  
  world.addShape(&red_tree);
  world.addShape(&yellow_tree);
  world.addShape(&left_tree);
  world.addShape(&right_tree);
}
