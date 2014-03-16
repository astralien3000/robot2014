#include "trajectory_manager.hpp"

#include "devices.hpp"


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

TrajectoryManager::TrajectoryManager(Output< Vect<2, s32> >& robot, Input< Vect<2, s32> >& odo, Input< Vect<2, s32> >& pos, PidFilter& pid)
  : _robot(robot), _pos(pos), _odo(odo), _pid(pid) {
  _state = STOP;
}

void TrajectoryManager::gotoPosition(Vect<2, s32> pos, s32 pseudo_ray) {
  _src = _pos.getValue();
  _dst = pos;

  Vect<2, s32> dir = _dst - _src;
  io << "direction (" << dir.coord(0) << ", " << dir.coord(1) << ")\n";
  Vect<2, s32> mid = _src + dir / 2;
  io << "middle (" << mid.coord(0) << ", " << mid.coord(1) << ")\n";
  Vect<2, s32> nor = (normal(dir) * 256) / dir.norm();
  io << "normal (" << nor.coord(0) << ", " << nor.coord(1) << ")\n";

  _cen = mid + (pseudo_ray * nor) / 256;
  _ray = (_src - _cen).norm();
  io << "center (" << _cen.coord(0) << ", " << _cen.coord(1) << ")\n";

  Vect<2, s32> dst_ray = _dst - _cen;
  _dst_angle = Math::atan2<Math::DEGREE>(dst_ray.coord(1), dst_ray.coord(0));

  Vect<2, s32> src_ray = _src - _cen;
  _angle_cmd = nearest_cmd_angle(_odo.getValue().coord(1), 180 - (Math::atan2<Math::DEGREE>(src_ray.coord(1), src_ray.coord(0)) + 90));

  _state = REACH_ANGLE;
  io<< "Reach angle...\n";
}

void TrajectoryManager::update_stop(void) {
  _robot.setValue(Vect<2, s32>(_dist_cmd, _angle_cmd));
}

void TrajectoryManager::update_reach_angle(void) {
  //Vect<2, s32> vray = _pos.getValue() - _cen;
  _robot.setValue(Vect<2, s32>(_odo.getValue().coord(0), _angle_cmd));

  if(Math::abs(_odo.getValue().coord(1) - _angle_cmd) < 10) {
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

  s32 ray_angle = Math::atan2<Math::DEGREE>(vray.coord(1), vray.coord(0));
  s32 angle_cmd = nearest_cmd_angle(_odo.getValue().coord(1), 180 - (ray_angle + 90));

  _robot.setValue(Vect<2, s32>(_odo.getValue().coord(0) + dist_err, angle_cmd + angle_err - a));
  //io << dist_err << "\n";
  //io << vray.coord(0) << " " << vray.coord(1) << " " << angle_cmd << " " << _odo.getValue().coord(1) << " " << Math::atan2<Math::DEGREE>(vray.coord(1), vray.coord(0)) + 90 << "\n";

  if(Math::abs(_dst_angle - ray_angle) < 2) {
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
    _angle_cmd = nearest_cmd_angle(_odo.getValue().coord(1), 180 - (_dst_angle + 90));
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
  //io << "pos (" << _pos.getValue().coord(0) << ", " << _pos.getValue().coord(1) << ")\n";
  return (_state == STOP) && (dist_err < 20);
}
