#include "secure_robot.hpp"


SecureRobot::SecureRobot(Output< Vect<2, s32> >& robot, Input< Vect<2, s32> >& odo, Input<bool>& skd_l, Input<bool>& skd_r, Output<s32>& mot_l, Output<s32>& mot_r)
  : _robot(robot), _odo(odo),
    _skd_l(skd_l), _skd_r(skd_r),
    _mot_l(mot_l), _mot_r(mot_r) {
  _state = false;
  _last_update = 0;
}

void SecureRobot::update(void) {
  if (_state)
    return; //do not update _state while unlock() is not called
  _state = _skd_l.getValue() || _skd_r.getValue();
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
    _mot_l.setValue(0);
    _mot_r.setValue(0);
  }
  else {
    _robot.setValue(command);
  }
}

void SecureRobot::unlock(void) {
  _state = false;
}
