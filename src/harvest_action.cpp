#include "harvest_action.hpp"

HarvestAction::HarvestAction(const Vect<2, s32>& pos, s32 angle) {
  const s32 MULT = 1024;
  Vect<2, s32> dir(MULT * Math::cos<Math::DEGREE, double>(angle), MULT * Math::sin<Math::DEGREE, double>(angle));
  Vect<2, s32> nor(MULT * Math::cos<Math::DEGREE, double>(angle + 90), MULT * Math::sin<Math::DEGREE, double>(angle + 90));

  dir *= DIST_FROM_TREE;
  nor *= DIST_FROM_WALL;

  dir /= MULT;
  nor /= MULT;
  
  _begin_point = pos - dir + nor;
  _end_point = pos + dir + nor;

  _done = false;
}

s16 HarvestAction::priority(void) {
  return 0;
}

#include "devices.hpp"

Vect<2, s32> HarvestAction::controlPoint(void) {
  io << _begin_point.coord(0) << " " << _begin_point.coord(1) << "\n";
  io << _end_point.coord(0) << " " << _end_point.coord(1) << "\n";
  return _begin_point;
}


void HarvestAction::doAction(void) {
  trajectoryManager().gotoPosition(_end_point);
  while(!trajectoryManager().isEnded()) {
    
  }
}
