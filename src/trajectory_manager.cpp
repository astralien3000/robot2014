#include "trajectory_manager.hpp"

#include "devices.hpp"


TrajectoryManager::TrajectoryManager(Output< Vect<2, s32> >& robot, Input< Vect<2, s32> >& odo, Input< Vect<2, s32> >& pos, PidFilter& pid)
  : _robot(robot), _pos(pos), _odo(odo), _pid(pid) {
  _state = STOP;
}

void TrajectoryManager::gotoPosition(Vect<2, s32> pos, s32 pseudo_ray) {
  _src = _pos.getValue();
  _dst = pos;
  _pseudo_ray = pseudo_ray;

  _dir = _dst - _src;
  _mid = _src + _dir / 2;
  _nor = (normal(_dir) * 256) / _dir.norm();

  _cen = _mid + (_pseudo_ray * _nor) / 256;
  _ray = (_src - _cen).norm();

  _state = REACH_ANGLE;
}

void TrajectoryManager::update_stop(void) {
  _robot.setValue(_odo.getValue());
}

void TrajectoryManager::update_reach_angle(void) {
  Vect<2, s32> vray = _pos.getValue() - _cen;
  s32 angle_cmd = ((Math::atan2<Math::DEGREE>(vray.coord(1), vray.coord(0)) + 90 + 180) % 360) - 180;
  _robot.setValue(Vect<2, s32>(_odo.getValue().coord(0), angle_cmd));

  if(angle_cmd == _odo.getValue().coord(1)) {
    _state = FOLLOW_TRAJECTORY;
  }
}

void TrajectoryManager::update_follow_trajectory(void) {
  Vect<2, s32> vray = _pos.getValue() - _cen;

  s32 rdiff = vray.norm() - _ray;
  s32 angle_err = _pid.doFilter(rdiff);

  Vect<2,s32> pos_err = _dst - _pos.getValue();
  s32 dist_err = pos_err.norm();

  s32 d = (_diff_d.doFilter(_odo.getValue().coord(0)) << 16) / _ray;
  s32 a = (s32)(d * 180. / 3.14) >> 16;

  s32 angle_cmd = Math::atan2<Math::DEGREE>(vray.coord(1), vray.coord(0)) + 90;

  _robot.setValue(Vect<2, s32>(_odo.getValue().coord(0) + dist_err, angle_cmd + angle_err + a));

  if(dist_err < 50) {
    _state = NEAR_END;
  }
}

void TrajectoryManager::update_near_end(void) {
  Vect<2, s32> vdst = _dst - _pos.getValue();
  s32 dist_err = vdst.norm();
  s32 angle_cmd = Math::atan2<Math::DEGREE>(vdst.coord(1), vdst.coord(0));
  _robot.setValue(Vect<2, s32>(_odo.getValue().coord(0) + dist_err, angle_cmd));
  
  if(dist_err < 5) {
    _state = STOP;
  }
}

void TrajectoryManager::update(void) {
  switch(_state) {
  case STOP:
    update_stop();
    break;
  case REACH_ANGLE:
    update_reach_angle();
    break;
  case FOLLOW_TRAJECTORY:
    update_follow_trajectory();
    break;
  case NEAR_END:
    update_near_end();
    break;
  }
}

bool TrajectoryManager::isEnded(void) {
  Vect<2, s32> vdst = _dst - _pos.getValue();
  s32 dist_err = vdst.norm();
  return (_state == STOP) && (dist_err < 10);
}
