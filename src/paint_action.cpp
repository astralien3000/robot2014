#include "paint_action.hpp"
#include "devices.hpp"

#include "asserv.hpp"

PaintAction::PaintAction(void) {
}

s16 PaintAction::priority(void) {
  if(_static_priority == 0) {
    return 0;
  }
  _static_priority++;

  s16 dist = (controlPoint() - positionManager().getValue()).norm();
  // pourquoi à chaque fois je vois un test "dist != 0" ???? si on est bien placé dès le départ on a pas le droit de faire l'action ?
  if( dist != 0) {
    return _static_priority * (4000 / dist);
  }
  return 0;
}

Vect<2, s32> PaintAction::controlPoint(void) {
  return Vect<2, s32>(0, 550);
}

void PaintAction::doAction(void) {
  io << "DO PAINT\n";
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
  asserv_speed_slow();
  for(int i = 0; i < 3; i++) {
    trajectoryManager().gotoDistance(-3000);
    while(!robot().getValue()) {
    }
    trajectoryManager().reset();
  }
  positionManager().setY(1050 - 125);
  robot().unlock();

  // Go far from wall
  asserv_speed_normal();
  trajectoryManager().gotoDistance(50);
  while(!trajectoryManager().isEnded()) {
    robot().unlock();
  }
  
  // And finally we return to the control point
  trajectoryManager().setMode(TrajectoryManager::FASTER);
  trajectoryManager().gotoPosition(Vect<2, s32>(0, 550));
  while(!trajectoryManager().isEnded()) {
  }

  _static_priority = 0;
  return;
}
