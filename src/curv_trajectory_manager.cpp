#include "curv_trajectory_manager.hpp"

CurvTrajectoryManager::CurvTrajectoryManager(Output< Vect<2, s32> >& robot, Input< Vect<2, s32> >& odo, Input< Vect<2, s32> >& pos, PidFilter& pid_r, PidFilter& pid_c)
  : RectTrajectoryManager(robot, odo, pos, pid_r),
    _pid_c(pid_c) {

}

void CurvTrajectoryManager::_update_reach_angle(TrajectoryManager& t) {
  reinterpret_cast<CurvTrajectoryManager&>(t).update_reach_angle();
}

void CurvTrajectoryManager::_update_follow_trajectory(TrajectoryManager& t) {
  reinterpret_cast<CurvTrajectoryManager&>(t).update_follow_trajectory();
}

void CurvTrajectoryManager::_update_near_end(TrajectoryManager& t) {
  reinterpret_cast<CurvTrajectoryManager&>(t).update_near_end();
}

void CurvTrajectoryManager::gotoCurvPosition(Vect<2, s32> pos, s32 pseudo_ray, bool way) {
  _state_handlers[REACH_ANGLE] = _update_reach_angle;
  _state_handlers[FOLLOW_TRAJECTORY] = _update_follow_trajectory;
  _state_handlers[NEAR_END] = _update_near_end;

  _way = way;
  if(way) {
    _way_angle = -90;
  }
  else {
    _way_angle = 90;
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

  _cen = mid + (pseudo_ray * nor) / 256;
  _ray = (_src - _cen).norm();
  io << "center (" << _cen.coord(0) << ", " << _cen.coord(1) << ")\n";

  Vect<2, s32> dst_ray = _dst - _cen;
  _dst_angle = Math::atan2<Math::DEGREE>(dst_ray.coord(1), dst_ray.coord(0));

  Vect<2, s32> src_ray = _src - _cen;

  if(_mod == FORWARD) {
    _angle_cmd = nearest_cmd_angle(_odo.getValue().coord(1)/10, 180 - (Math::atan2<Math::DEGREE>(src_ray.coord(1), src_ray.coord(0)) + _way_angle));
  }
  else if(_mod == BACKWARD) {
    _angle_cmd = nearest_cmd_angle(_odo.getValue().coord(1)/10, 180 + Math::atan2<Math::DEGREE>(src_ray.coord(1), src_ray.coord(0)) + _way_angle);
  }

  _dist_cmd = _odo.getValue().coord(0);

  _state = REACH_ANGLE_CURV;
  io<< "Reach angle...\n";
}


void CurvTrajectoryManager::update_reach_angle(void) {
  _robot.setValue(Vect<2, s32>(_dist_cmd, _angle_cmd*10));

  if(Math::abs(_odo.getValue().coord(1) - _angle_cmd*10) < 50) {
    _state = FOLLOW_TRAJECTORY_CURV;
    io << "Follow trajectory...\n";
  }
}

void CurvTrajectoryManager::update_follow_trajectory(void) {
  Vect<2, s32> vray = _pos.getValue() - _cen;

  s32 rdiff = vray.norm() - _ray;
  s32 angle_err = _pid_c.doFilter(rdiff);

  Vect<2,s32> pos_err = _dst - _pos.getValue();
  s32 dist_err = pos_err.norm();

  s32 d = (_diff_d.doFilter(_odo.getValue().coord(0)) << 16) / _ray;
  s32 a = (s32)(d * 180. / 3.14) >> 14; // --> * 2
  if(_way_angle == -90) {
    angle_err = -angle_err;
    a = -a;
  }

  s32 ray_angle = Math::atan2<Math::DEGREE>(vray.coord(1), vray.coord(0));

  
  s32 angle_cmd = 0;
  if(_mod == FORWARD) {
   angle_cmd = nearest_cmd_angle(_odo.getValue().coord(1)/10, 180 - (ray_angle + _way_angle));
  }
  else if(_mod == BACKWARD) {
    dist_err = -dist_err;
    angle_cmd = nearest_cmd_angle(_odo.getValue().coord(1)/10, 180 - (ray_angle - _way_angle));
  }

  _robot.setValue(Vect<2, s32>(_odo.getValue().coord(0) + dist_err, (angle_cmd + angle_err - a)*10));
  io << angle_cmd + angle_err - a << " " << _odo.getValue().coord(1)/10 << " " << angle_err << "\n";

  if(Math::abs(_dst_angle - ray_angle) < 5) {
    _state = NEAR_END_CURV;
    
    _dist_cmd = _odo.getValue().coord(0) + pos_err.norm();

    if(_mod == FORWARD) {
      _angle_cmd = nearest_cmd_angle(_odo.getValue().coord(1)/10, 180 - (_dst_angle + _way_angle));
    }
    else if(_mod == BACKWARD) {
      _angle_cmd = nearest_cmd_angle(_odo.getValue().coord(1)/10, 180 - (_dst_angle - _way_angle));
    }

    io << "Near end...\n";
  }
}

void CurvTrajectoryManager::update_near_end(void) {
  _robot.setValue(Vect<2, s32>(_dist_cmd, _angle_cmd*10));
  
  if(Math::abs(_odo.getValue().coord(0) - _dist_cmd) < 20) {
    _state = STOP;
    io << "Stop !\n";
  }
}
