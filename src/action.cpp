#include "action.hpp"

Input< Vect<2, s32> >* Action::_pos = 0;
RectTrajectoryManager* Action::_traj = 0;
SecureRobot* Action::_robot = 0;
u8 Action::_fruit = 0;

enum Side Action::side = RED;

Input< Vect<2, s32> >& Action::positionManager(void) {
  return *_pos;
}

RectTrajectoryManager& Action::trajectoryManager(void) {
  return *_traj;
}

SecureRobot& Action::robot(void) {
  return *_robot;
}

void Action::setPositionManager(Input< Vect<2, s32> >& p) {
  _pos = &p;
}

void Action::setTrajectoryManager(RectTrajectoryManager& t) {
  _traj = &t;
}

void Action::setRobot(SecureRobot& r) {
  _robot = &r;
}
