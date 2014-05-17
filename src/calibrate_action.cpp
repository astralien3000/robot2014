#include "master_action.hpp"

#include <math/trigo.hpp>

#include "devices.hpp"

MasterAction::MasterAction(const Vect<2, s32>& pos, s32 angle) {
  
}

s16 MasterAction::priority(void) {
  return 0;
}

Vect<2, s32> MasterAction::controlPoint(void) {
  return Vect<2, s32>(0,0);
}

#include "devices.hpp"

void MasterAction::doAction(void) {
  if((positionManager().getValue() - controlPoint()).norm() > 100) {
    io << "not at a good position\n";
    return;
  }

  _done = true;
}

