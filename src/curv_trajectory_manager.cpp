#include "curv_trajectory_manager.hpp"

inline s32 deg2raw(s32 val) {
  return val << 4;
}

inline s32 raw2deg(s32 val) {
  return val >> 4;
}

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

  if(_state == NEAR_END) {
    _src = _dst;
  }
  else {
    _src = _pos.getValue();
  }

  _dst = pos;

  Vect<2, s32> dir = _dst - _src;
  //io << "direction (" << dir.coord(0) << ", " << dir.coord(1) << ")\n";
  Vect<2, s32> mid = _src + dir / 2;
  //io << "middle (" << mid.coord(0) << ", " << mid.coord(1) << ")\n";
  Vect<2, s32> nor = (normal(dir) * 256) / dir.norm();
  //io << "normal (" << nor.coord(0) << ", " << nor.coord(1) << ")\n";

  _cen = mid + (pseudo_ray * nor) / 256;
  _ray = (_src - _cen).norm();
  //io << "center (" << _cen.coord(0) << ", " << _cen.coord(1) << ")\n";

  Vect<2, s32> dst_ray = _dst - _cen;
  _dst_angle = Math::atan2<Math::DEGREE>(dst_ray.coord(1), dst_ray.coord(0));

  Vect<2, s32> src_ray = _src - _cen;


  if (_mod == FASTER) {
    s32 angle_b = nearest_cmd_angle(raw2deg(_odo.getValue().coord(1)), 180 + Math::atan2<Math::DEGREE>(src_ray.coord(1), src_ray.coord(0)) + _way_angle);
    s32 angle_f = nearest_cmd_angle(raw2deg(_odo.getValue().coord(1)), 180 - Math::atan2<Math::DEGREE>(src_ray.coord(1), src_ray.coord(0)) + _way_angle);

    angle_b -= raw2deg(_odo.getValue().coord(1));
    angle_f -= raw2deg(_odo.getValue().coord(1));

    if(Math::abs(angle_b) < Math::abs(angle_f)) {
      _angle_cmd = angle_f;
      _backward = false;
    }
    else {
      _angle_cmd = angle_b;
      _backward = true;
    }
  }
  else if(_mod == BACKWARD) {
    _angle_cmd = nearest_cmd_angle(raw2deg(_odo.getValue().coord(1)), 180 + Math::atan2<Math::DEGREE>(src_ray.coord(1), src_ray.coord(0)) + _way_angle);
    _backward = true;
  }
  else {
    _angle_cmd = nearest_cmd_angle(raw2deg(_odo.getValue().coord(1)), 180 - Math::atan2<Math::DEGREE>(src_ray.coord(1), src_ray.coord(0)) + _way_angle);
    _backward = false;
  }

  _dist_cmd = _odo.getValue().coord(0);

  _state = REACH_ANGLE;
  //io<< "Reach angle...\n";
}

void CurvTrajectoryManager::update_reach_angle(void) {
  _robot.setValue(Vect<2, s32>(_dist_cmd, deg2raw(_angle_cmd)));

  if(Math::abs(_odo.getValue().coord(1) - deg2raw(_angle_cmd)) < 50) {
    _state = FOLLOW_TRAJECTORY;
    //io << "Follow trajectory...\n";
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
  if(!_way) {
    angle_err = -angle_err;
    a = -a;
  }

  s32 ray_angle = Math::atan2<Math::DEGREE>(vray.coord(1), vray.coord(0));

  s32 angle_cmd = 0;
  if(_backward) {
    dist_err = -dist_err;
    angle_cmd = nearest_cmd_angle(raw2deg(_odo.getValue().coord(1)), ray_angle + _way_angle);
  }
  else {
    angle_cmd = nearest_cmd_angle(raw2deg(_odo.getValue().coord(1)), -(ray_angle + _way_angle));
  }

  _robot.setValue(Vect<2, s32>(_odo.getValue().coord(0) + dist_err, deg2raw(angle_cmd + angle_err - a)));

  if(Math::abs(_dst_angle - ray_angle) < 5) {
    _state = NEAR_END;
    
    _dist_cmd = _odo.getValue().coord(0) + pos_err.norm();

    if(_backward) {
      _angle_cmd = nearest_cmd_angle(raw2deg(_odo.getValue().coord(1)), _dst_angle + _way_angle);
    }
    else {
      _angle_cmd = nearest_cmd_angle(raw2deg(_odo.getValue().coord(1)),-(_dst_angle + _way_angle));
    }

    //io << "Near end...\n";
  }
}

void CurvTrajectoryManager::update_near_end(void) {
  _robot.setValue(Vect<2, s32>(_dist_cmd, deg2raw(_angle_cmd)));
  
  if(Math::abs(_odo.getValue().coord(0) - _dist_cmd) < 20) {
    _state = STOP;
    //io << "Stop !\n";
  }
}
