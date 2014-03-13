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
  io << "center (" << _cen.coord(0) << ", " << _cen.coord(1) << ")\n";

  _state = REACH_ANGLE;
  io<< "Reach angle...\n";
}

s32 _dist_cmd = 0;
s32 _angle_cmd = 0;

void TrajectoryManager::update_stop(void) {
  _robot.setValue(Vect<2, s32>(_dist_cmd, _angle_cmd));
}

s32 nearest_cmd_angle(s32 angle, s32 cmd) {
  s32 res = (angle % 360);
  angle -= res;
  cmd = ((cmd + 180) % 360) - 180;

  if(180 < res - cmd) {
    angle += 360;
  }
  else if(res - cmd < -180) {
    angle -= 360;
  }

  return angle + cmd;
}


void TrajectoryManager::update_reach_angle(void) {
  Vect<2, s32> vray = _pos.getValue() - _cen;
  s32 angle_cmd = nearest_cmd_angle(_odo.getValue().coord(1), 180 - (Math::atan2<Math::DEGREE>(vray.coord(1), vray.coord(0)) + 90));
  _robot.setValue(Vect<2, s32>(_odo.getValue().coord(0), angle_cmd));

  if(angle_cmd == _odo.getValue().coord(1)) {
    _state = FOLLOW_TRAJECTORY;
    io << "Follow trajectory...\n";
  }
}

void TrajectoryManager::update_follow_trajectory(void) {
  Vect<2, s32> vray = _pos.getValue() - _cen;

  s32 rdiff = vray.norm() - _ray;
  s32 angle_err = _pid.doFilter(rdiff);

  Vect<2,s32> pos_err = _dst - _pos.getValue();
  s32 dist_err = pos_err.norm();

  s32 d = (_diff_d.doFilter(_odo.getValue().coord(0)) << 16) / _ray;
  s32 a = (s32)(d * 180. / 3.14) >> 15; // --> * 2

  s32 angle_cmd = nearest_cmd_angle(_odo.getValue().coord(1), 180 - (Math::atan2<Math::DEGREE>(vray.coord(1), vray.coord(0)) + 90));

  _robot.setValue(Vect<2, s32>(_odo.getValue().coord(0) + dist_err, angle_cmd + angle_err - a));
  io << dist_err << "\n";
  //io << vray.coord(0) << " " << vray.coord(1) << " " << angle_cmd << " " << _odo.getValue().coord(1) << " " << Math::atan2<Math::DEGREE>(vray.coord(1), vray.coord(0)) + 90 << "\n";

  if(dist_err < 100) {
    _state = NEAR_END;
    io << "Near end...\n";
  }
}

bool first = true;

void TrajectoryManager::update_near_end(void) {
  if(first) {
    first = false;
    Vect<2, s32> vdst = _dst - _pos.getValue();
    _dist_cmd = _odo.getValue().coord(0) + vdst.norm();
    _angle_cmd = nearest_cmd_angle(_odo.getValue().coord(1), Math::atan2<Math::DEGREE>(vdst.coord(1), vdst.coord(0)));
  }

  _robot.setValue(Vect<2, s32>(_dist_cmd, _angle_cmd));
  
  if(Math::abs(_odo.getValue().coord(0) - _dist_cmd) < 5) {
    first = true;
    _state = STOP;
    io << "Stop !\n";
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
