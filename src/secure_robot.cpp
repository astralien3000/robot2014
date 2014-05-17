#include "secure_robot.hpp"


SecureRobot::SecureRobot(Output< Vect<2, s32> >& robot, Input< Vect<2, s32> >& odo, Input<bool>& skd_l, Input<bool>& skd_r, Output<s32>& mot_l, Output<s32>& mot_r)
  : _robot(robot), _odo(odo),
    _skd_l(skd_l), _skd_r(skd_r),
    _mot_l(&mot_l), _mot_r(&mot_r) {
  _state = false;
  _last_update = 0;
  _sk_dur = 0;
  _lockable = true;
  _unlockable = true;
}

void SecureRobot::update(void) {
  if (_state)
    return;

  if(_skd_l.getValue() || _skd_r.getValue()) {
    if(_lockable) {
      _state = (2 < ++_sk_dur);
    }
  }
  else {
    if(0 < _sk_dur) {
      --_sk_dur;
    }
  }

  if (_state) {
    _robot.setValue(_odo.getValue());
  }
}

bool SecureRobot::getValue(void) {
  return _state;
}

void SecureRobot::setValue(Vect<2, s32> command) {
  update();
  if (_state) {
    //robot is skating, send "DO NOT MOVE"
    _mot_l->setValue(0);
    _mot_r->setValue(0);
  }
  else {
    _robot.setValue(command);
  }
}

void SecureRobot::unlock(void) {
  if(_unlockable) {
    _state = false;
    _sk_dur = 0;
  }
}

void SecureRobot::lock(void) {
  if(_lockable) {
    _state = true;
  }
}

void SecureRobot::setLockable(bool val) {
  _lockable = val;
}

void SecureRobot::setUnlockable(bool val) {
  _unlockable = val;
}

void SecureRobot::setLockableMotors(Output<s32>& mot_l, Output<s32>& mot_r) {
  _mot_l = &mot_l;
  _mot_r = &mot_r;
}
