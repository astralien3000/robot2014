#include "deposit_action.hpp"
#include "devices.hpp"

#include "asserv.hpp"

#include <device/servomotor/fpga_servomotor.hpp>

#define F_CPU 16000000l
#include <util/delay.h>

DepositAction::DepositAction(void) {
}

s16 DepositAction::priority(void) {
  if(_fruit == 0) {
    return 0;
  }
  
  s16 dist = (controlPoint() - positionManager().getValue()).norm();
  if(dist != 0) {
    return _fruit * 7000 / dist;
  }
  return _fruit * 7000;
}

Vect<2, s32> DepositAction::controlPoint(void) {
  return Vect<2, s32>(((side == RED) ? 750 : -750), 350);
}

void DepositAction::doAction(void) {
  io << "DO DEPOSIT\n";
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
  asserv_speed_slow();
  for(int i = 0; i < 3; i++) {
    trajectoryManager().gotoDistance(3000);
    while(!robot().getValue()) {
    }
    trajectoryManager().reset();
  }
  positionManager().setY(1050 - 300 - 125);
  robot().unlock();
  
  // Actually deposit the fruits now
  basket_servo.setValue(BASKET_SERVO_DOWN_CMD);  
  _delay_ms(1000);
  basket_servo.setValue(BASKET_SERVO_UP_CMD);  
  _fruit = 0;
  
  // Go far from the basket
  asserv_speed_normal();
  trajectoryManager().gotoDistance(-200);
  while(!trajectoryManager().isEnded()) {
    robot().unlock();
  }

  // And finally we return to the control point
  trajectoryManager().setMode(TrajectoryManager::FASTER);
  trajectoryManager().gotoPosition(Vect<2, s32>(x, 450));
  while(!trajectoryManager().isEnded()) {
  }
}
