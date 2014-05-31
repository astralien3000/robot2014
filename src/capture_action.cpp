#include "capture_action.hpp"
#include "secure_timer.hpp"
#include "devices.hpp"

CaptureAction::CaptureAction(const Vect<2, s32>& pos, const Vect<2, s32>& mamouth)
  : _pos(pos), _mamouth(mamouth) {
}

s16 CaptureAction::priority(void) {
  if(secure_timer_time() < 70) {
    return 0;
  }
  
  s16 dist = (controlPoint() - positionManager().getValue()).norm();
  if(dist != 0) {
    return 64 * (10000 / dist);
  }
  
  return 30000;
}

Vect<2, s32> CaptureAction::controlPoint(void) {
  return _pos;
}

enum Error CaptureAction::doAction(void) {
  io << "DO CAPTURE\n";
  
  trajectoryManager().lookAt(_mamouth);
  while(!trajectoryManager().isEnded()) {
    if(robot().getValue()) {
      // The robot got locked
      return SKATING;
    }
  }
  
  // The robot is looking in the right direction
  // Let's wait for the funny action time to arrive
  while(secure_timer_time() < 93) {
  }

  io << "LAUNCH\n";
  
  // Let's fire now !!!
  arba_servo.setValue(ARBA_SERVO_UNLOCK_CMD);
  return SUCCESS;
}
