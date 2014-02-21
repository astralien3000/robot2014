#ifndef OBSTACLE_AVOIDANCE_HPP
#define OBSTACLE_AVOIDANCE_HPP

#include <base/integer.hpp>
#include <math/vect.hpp>

#include <device/output.hpp>

#include "trajectory_manager.hpp"
#include "world.hpp"

//! \brief Gives the robot a way to go to a position, avoiding problems...
/*!
  
  May have also to simplify trajectory for trajectoery manager.

 */
class ObstacleAvoidance : public Output< Vect<2, s32> > {
private:
  TrajectoryManager& _traj;
  World& _world;

public:
  ObstacleAvoidance(TrajectoryManager& traj, World& world);

  void setValue(Vect<2, s32>);
};

#endif//OBSTACLE_AVOIDANCE_HPP
