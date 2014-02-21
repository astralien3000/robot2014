#ifndef WORLD_HPP
#define WORLD_HPP

#include <base/integer.hpp>
#include <math/vect.hpp>

// You can change the name...
class Figure;

//! \brief Represents the robot's environnment
/*!

  Contains static and dynamic obstacles.

 */
class World {
public:
  World(void);

  bool isObstacle(Vect<2, s32>&);

  bool isCollided(Figure&);
};

#endif//WORLD_HPP
