#include "harvest_action.hpp"
#include "asserv.hpp"


HarvestAction::HarvestAction(const Vect<2, s32>& pos, s32 angle, s8 bonus = 0) {
  const s32 MULT = 1024;
  Vect<2, s32> dir(MULT * Math::cos<Math::DEGREE, double>(angle), MULT * Math::sin<Math::DEGREE, double>(angle));
  Vect<2, s32> nor(MULT * Math::cos<Math::DEGREE, double>(angle + 90), MULT * Math::sin<Math::DEGREE, double>(angle + 90));
  Vect<2, s32> nor2 = nor;

  dir *= DIST_FROM_TREE;
  nor *= DIST_FROM_WALL;

  dir /= MULT;
  nor /= MULT;
  
  _begin_point = pos - dir + nor;
  _end_point = pos + dir + nor;

  nor2 *= CTRL_DIST_FROM_WALL;
  nor2 /= MULT;

  _ctrl_point = pos - dir + nor2;

  _done = false;
  _bonus = bonus;
}

s16 HarvestAction::priority(void) {
  if (_done)
    return 0;
  //return 0;
  // Future improvement ?
  return _bonus + 10000/((controlPoint() - positionManager().getValue()).norm() +1);
}

#include "devices.hpp"

Vect<2, s32> HarvestAction::controlPoint(void) {
  io << _begin_point.coord(0) << " " << _begin_point.coord(1) << "\n";
  io << _end_point.coord(0) << " " << _end_point.coord(1) << "\n";
  return _ctrl_point;
}


void HarvestAction::doAction(void) {
  io << "DO HARVERST\n";
  trajectoryManager().setMode(TrajectoryManager::FORWARD);

  // Face the wall
  trajectoryManager().gotoPosition(_begin_point);
  while(!trajectoryManager().isEnded()) {
  }

  // Do ?

  asserv_speed_slow();

  // Harvest
  trajectoryManager().gotoPosition(_end_point);
  while(!trajectoryManager().isEnded()) {
  }

  asserv_speed_normal();

  // Go far
  trajectoryManager().gotoAngle(pos.angle() - 90);
  while(!trajectoryManager().isEnded()) {
  }

  trajectoryManager().gotoDistance(100);
  while(!trajectoryManager().isEnded()) {
  }
  
  trajectoryManager().setMode(TrajectoryManager::FASTER);

  _fruit++;
  _done = true;
}

