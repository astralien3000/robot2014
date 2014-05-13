#include "ball_action.hpp"
#include "devices.hpp"

BallAction::BallAction(const Vect<2, s32>& pos, s32 number);
: _pos(pos), _number(number), _done(false) {
}

s16 BallAction::priority(void) {
  if(_done) {
    return 0;
  }
  
  s16 dist = (controlPoint() - positionManager().getValue()).norm();
  if(dist != 0) {
    return 4000 / dist;
  }
  return 4000;
}

Vect<2, s32> BallAction::controlPoint(void) {
  return _pos;
}

void BallAction::doAction(void) {
  if((positionManager().getValue() - controlPoint()).norm() > 100) {
    io << "not at a good position\n";
    return;
  }

  // Let Benoit do its job !
  /*
    for (int i=0; i<_number; i++)
      driver_made_by_benoit.send();
  */

  _done = true;
}
