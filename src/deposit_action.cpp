#include "deposit_action.hpp"
#include "devices.hpp"

DepositAction::DepositAction(void)
  : _done(false) {
}

s16 DepositAction::priority(void) {
  if(_fruit == 0) {
    return 0;
  }
  
  s16 dist = (controlPoint() - positionManager().getValue()).norm();
  if(dist != 0) {
    return 3000 / dist;
  }
  return 0;
}

Vect<2, s32> DepositAction::controlPoint(void) {
  return Vect<2, s32>(((side == RED) ? 750 : -750), 600);
}

void DepositAction::doAction(void) {
  if((positionManager().getValue() - controlPoint()).norm() > 100) {
    io << "not at a good position\n";
    return;
  }
  
  s32 x = ((side == RED) ? 750 : -750);
  
  // So we first get a bit closer to the basket
  trajectoryManager().setMode(TrajectoryManager::FORWARD);
  trajectoryManager().gotoPosition(Vect<2, s32>(x, 450));
  while(!trajectoryManager().isEnded()) {
  }
  
  // We stick to the basket
  trajectoryManager().gotoDistance(3000);
  while(!_robot.getValue()) {
  }
  _robot.unlock();
  
  // Actually deposit the fruits now
  _fruit = 0;
  
  // And finally we return to the control point
  trajectoryManager().setMode(TrajectoryManager::FASTER);
  trajectoryManager().gotoPosition(Vect<2, s32>(x, 450));
}
