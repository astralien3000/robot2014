#include "paint_action.hpp"
#include "devices.hpp"

PaintAction::PaintAction(void)
  : _done(false) {
}

s16 PaintAction::priority(void) {
  s16 dist = (controlPoint() - positionManager().getValue()).norm();
  if(!_done && dist != 0) {
    return 3000 / dist;
  }
  return 0;
}

Vect<2, s32> PaintAction::controlPoint(void) {
  return Vect<2, s32>(0, 550);
}

void PaintAction::doAction(void) {
  if((positionManager().getValue() - controlPoint()).norm() > 100) {
    io << "not at a good position\n";
    return;
  }
  
  // So we first get a bit closer
  trajectoryManager().setMode(TrajectoryManager::BACKWARD);
  trajectoryManager().gotoPosition(Vect<2, s32>(0, 875));
  while(!trajectoryManager().isEnded()) {
  }
  
  // We stick the painting to the wall
  trajectoryManager().gotoDistance(-3000);
  while(!_robot.getValue()) {
  }
  _robot.unlock();
  trajectoryManager().gotoDistance(150);
  
  // And finally we return to the control point
  trajectoryManager().setMode(TrajectoryManager::FASTER);
  trajectoryManager().gotoPosition(Vect<2, s32>(0, 550));
  while(!trajectoryManager().isEnded()) {
  }
  _done = true;
  return;
}
