#include "trajectory_manager.hpp"


TrajectoryManager::TrajectoryManager(Output< Vect<2, s32> >& robot, Input< Vect<2, s32> >& odo, Input< Vect<2, s32> >& pos)
  : _robot(robot), _pos(pos), _odo(odo), 
    _state(STOP), _mod(FASTER) {

  for(s16 i = 0 ; i < MAX_STATES ; i++) {
    _state_handlers[i] = 0;
  }

  _state_handlers[STOP] = _update_stop;
}

void TrajectoryManager::reset(void) {
  _dist_cmd = odo.getValue().coord(0);
  _angle_cmd = odo.getValue().coord(1)/10;
  _state = STOP;
}

void TrajectoryManager::update(void) {
  if(_state_handlers[_state]) {
    _state_handlers[_state](*this);
  }
  else {
    _state = STOP;
    _state_handlers[_state](*this);
  }
}

bool TrajectoryManager::isEnded(void) {
  Vect<2, s32> vdst = _dst - _pos.getValue();
  s32 dist_err = vdst.norm();

  return (_state == STOP) || 
    ((_state == NEAR_END) && (dist_err < 20));
}

void TrajectoryManager::setMode(Mode m) {
  _mod = m;
}

inline s32 deg2raw(s32 val) {
  return val << 4;
}

void TrajectoryManager::update_stop() {
  _robot.setValue(Vect<2, s32>(_dist_cmd, deg2raw(_angle_cmd)));
}

void TrajectoryManager::_update_stop(TrajectoryManager& t) {
  t.update_stop();
}
