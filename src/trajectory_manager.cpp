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

TrajectoryManager::TrajectoryManager(Output< Vect<2, s32> >& robot, Input< Vect<2, s32> >& odo, Input< Vect<2, s32> >& pos, PidFilter& pid_r, PidFilter& pid_c)
  : _robot(robot), _pos(pos), _odo(odo), 
    _pid_r(pid_r), _pid_c(pid_c),
    _mod(FASTER), _state(STOP) {

}

void TrajectoryManager::gotoPosition(Vect<2, s32> pos, s32 pseudo_ray, bool way) {
  if(way) {
    _data.curv._way_angle = 90;
  }
  else {
    _data.curv._way_angle = -90;
  }

  if(_state == NEAR_END_CURV || _state == NEAR_END_RECT) {
    _src = _dst;
  }
  else {
    _src = _pos.getValue();
  }

  _dst = pos;

  Vect<2, s32> dir = _dst - _src;
  io << "direction (" << dir.coord(0) << ", " << dir.coord(1) << ")\n";
  Vect<2, s32> mid = _src + dir / 2;
  io << "middle (" << mid.coord(0) << ", " << mid.coord(1) << ")\n";
  Vect<2, s32> nor = (normal(dir) * 256) / dir.norm();
  io << "normal (" << nor.coord(0) << ", " << nor.coord(1) << ")\n";

  _data.curv._cen = mid + (pseudo_ray * nor) / 256;
  _data.curv._ray = (_src - _data.curv._cen).norm();
  io << "center (" << _data.curv._cen.coord(0) << ", " << _data.curv._cen.coord(1) << ")\n";

  Vect<2, s32> dst_ray = _dst - _data.curv._cen;
  _data.curv._dst_angle = Math::atan2<Math::DEGREE>(dst_ray.coord(1), dst_ray.coord(0));

  Vect<2, s32> src_ray = _src - _data.curv._cen;
  _angle_cmd = nearest_cmd_angle(_odo.getValue().coord(1)/10, 180 - (Math::atan2<Math::DEGREE>(src_ray.coord(1), src_ray.coord(0)) + _data.curv._way_angle));
  _dist_cmd = _odo.getValue().coord(0);

  _state = REACH_ANGLE_CURV;
  io<< "Reach angle...\n";
}

void TrajectoryManager::gotoPosition(Vect<2, s32> pos) {

  if(_state == NEAR_END_CURV || _state == NEAR_END_RECT) {
    _src = _dst;
  }
  else {
    _src = _pos.getValue();
  }

  _dst = pos;

  Vect<2, s32> dir = _dst - _src;
  _data.rect._nor = (normal(dir) * 256) / dir.norm();

  _angle_cmd = nearest_cmd_angle(_odo.getValue().coord(1)/10, -Math::atan2<Math::DEGREE>(dir.coord(1), dir.coord(0)));
  _dist_cmd = _odo.getValue().coord(0);

  _state = REACH_ANGLE_RECT;
  io<< "Reach angle (rect)...\n";
}

void TrajectoryManager::reset(void) {
  _dist_cmd = odo.getValue().coord(0);
  _angle_cmd = odo.getValue().coord(1)/10;
  _state = STOP;
}

void TrajectoryManager::update_stop(void) {
  _robot.setValue(Vect<2, s32>(_dist_cmd, _angle_cmd*10));
}

void TrajectoryManager::update_reach_angle(void) {
  _robot.setValue(Vect<2, s32>(_dist_cmd, _angle_cmd*10));

  if(Math::abs(_odo.getValue().coord(1) - _angle_cmd*10) < 50) {
    _state = FOLLOW_TRAJECTORY_CURV;
    io << "Follow trajectory...\n";
  }
}

void TrajectoryManager::update_reach_angle_rect(void) {
  _robot.setValue(Vect<2, s32>(_dist_cmd, _angle_cmd*10));

  if(Math::abs(_odo.getValue().coord(1) - _angle_cmd*10) < 50) {
    _state = FOLLOW_TRAJECTORY_RECT;
    io << "Follow trajectory (rect)...\n";
  }
}

void TrajectoryManager::update_follow_trajectory(void) {
  Vect<2, s32> vray = _pos.getValue() - _data.curv._cen;

  s32 rdiff = vray.norm() - _data.curv._ray;
  s32 angle_err = _pid_c.doFilter(rdiff);

  Vect<2,s32> pos_err = _dst - _pos.getValue();
  s32 dist_err = pos_err.norm();

  s32 d = (_diff_d.doFilter(_odo.getValue().coord(0)) << 16) / _data.curv._ray;
  s32 a = (s32)(d * 180. / 3.14) >> 14; // --> * 2
  if(_data.curv._way_angle == -90) {
    angle_err = -angle_err;
    a = -a;
  }

  s32 ray_angle = Math::atan2<Math::DEGREE>(vray.coord(1), vray.coord(0));
  s32 angle_cmd = nearest_cmd_angle(_odo.getValue().coord(1)/10, 180 - (ray_angle + _data.curv._way_angle));

  _robot.setValue(Vect<2, s32>(_odo.getValue().coord(0) + dist_err, (angle_cmd + angle_err - a)*10));
  io << angle_cmd + angle_err - a << " " << _odo.getValue().coord(1)/10 << " " << angle_err << "\n";

  if(Math::abs(_data.curv._dst_angle - ray_angle) < 5) {
    _state = NEAR_END_CURV;
    
    _dist_cmd = _odo.getValue().coord(0) + pos_err.norm();
    _angle_cmd = nearest_cmd_angle(_odo.getValue().coord(1)/10, 180 - (_data.curv._dst_angle + _data.curv._way_angle));

    io << "Near end...\n";
  }
}

void TrajectoryManager::update_follow_trajectory_rect(void) {
  Vect<2,s32> pos_err = _dst - _pos.getValue();

  s32 ndiff = scal(pos_err, _data.rect._nor);
  s32 angle_err = _pid_r.doFilter(ndiff);
  
  s32 dist_err = pos_err.norm();
  
  _robot.setValue(Vect<2, s32>(_odo.getValue().coord(0) + dist_err, (_angle_cmd - angle_err)*10));

  if(Math::abs(scal(pos_err, pos_err)) < 10000 + ndiff) {
    _state = NEAR_END_RECT;

    _dist_cmd = _odo.getValue().coord(0) + pos_err.norm();

    io << "Near end...\n";
  }
}

void TrajectoryManager::update_near_end(void) {
  _robot.setValue(Vect<2, s32>(_dist_cmd, _angle_cmd*10));
  
  if(Math::abs(_odo.getValue().coord(0) - _dist_cmd) < 20) {
    _state = STOP;
    io << "Stop !\n";
  }
}

void TrajectoryManager::update_near_end_rect(void) {
  _robot.setValue(Vect<2, s32>(_dist_cmd, _angle_cmd*10));
  
  if(Math::abs(_odo.getValue().coord(0) - _dist_cmd) < 20) {
    _state = STOP;
    io << "Stop !\n";
  }
}

void TrajectoryManager::update(void) {
  // Sorted by priority
  switch(_state) {
  case FOLLOW_TRAJECTORY_CURV:
    update_follow_trajectory();
    break;
  case FOLLOW_TRAJECTORY_RECT:
    update_follow_trajectory_rect();
    break;
  case NEAR_END_CURV:
    update_near_end();
    break;
  case NEAR_END_RECT:
    update_near_end_rect();
    break;
  case REACH_ANGLE_CURV:
    update_reach_angle();
    break;
  case REACH_ANGLE_RECT:
    update_reach_angle_rect();
    break;
  case STOP:
    update_stop();
    break;
  default:
    // ERROR
    break;
  }
}

bool TrajectoryManager::isEnded(void) {
  Vect<2, s32> vdst = _dst - _pos.getValue();
  s32 dist_err = vdst.norm();

  return (_state == STOP) || 
    ((_state == NEAR_END_RECT || _state == NEAR_END_CURV)
     && (dist_err < 20));
}
