#include "master_action.hpp"

#include <math/trigo.hpp>

#include "devices.hpp"

MasterAction::MasterAction(const Vect<2, s32>& pos, s32 angle) {
  _side_point[RED] = pos + Vect<2, s32>(DIST_MM * Math::cos<Math::DEGREE, double>(angle), DIST_MM * Math::sin<Math::DEGREE, double>(angle));

  _side_point[YELLOW] = pos - Vect<2, s32>(DIST_MM * Math::cos<Math::DEGREE, double>(angle), DIST_MM * Math::sin<Math::DEGREE, double>(angle));

  _done = false;
}

s16 MasterAction::priority(void) {
  s16 dist = (controlPoint() - positionManager().getValue()).norm();
  if(!_done && dist != 0) {
    return 2000 / dist;
  }
  return 0;
}

Vect<2, s32> MasterAction::controlPoint(void) {
  return _side_point[side];
}

#include "devices.hpp"

void MasterAction::doAction(void) {
  io << "DO MASTER\n";
  if((positionManager().getValue() - controlPoint()).norm() > 100) {
    io << "not at a good position\n";
    return;
  }

  Side anti_side = RED;
  if(side == RED) {
    anti_side = YELLOW;
  }

  io << "goto x = " << _side_point[anti_side].coord(0);
  io << " y = " << _side_point[anti_side].coord(1) << "\n";
  trajectoryManager().gotoPosition(_side_point[anti_side]);
  while(!trajectoryManager().isEnded()) {
    // Do sth ?
  }

  _done = true;
}
