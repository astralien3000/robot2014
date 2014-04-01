#include "rect_trajectory_manager.hpp"

inline s32 deg2raw(s32 val) {
  return val << 4;
}

inline s32 raw2deg(s32 val) {
  return val >> 4;
}

RectTrajectoryManager::RectTrajectoryManager(Output< Vect<2, s32> >& robot, Input< Vect<2, s32> >& odo, Input< Vect<2, s32> >& pos, PidFilter& pid_r)
  : TrajectoryManager(robot, odo, pos), _pid_r(pid_r) {
  _diff_a.setDelta(1);
}

void RectTrajectoryManager::_update_reach_angle(TrajectoryManager& t) {
  reinterpret_cast<RectTrajectoryManager&>(t).update_reach_angle();
}

void RectTrajectoryManager::_update_follow_trajectory(TrajectoryManager& t) {
  reinterpret_cast<RectTrajectoryManager&>(t).update_follow_trajectory();
}

void RectTrajectoryManager::_update_near_end(TrajectoryManager& t) {
  reinterpret_cast<RectTrajectoryManager&>(t).update_near_end();
}

void RectTrajectoryManager::gotoDistance(s32 dist) {
  Vect<2, s32> pos = _pos.getValue();
  s32 angle = raw2deg(_odo.getValue().coord(1));
  if(_state == NEAR_END) {
    pos = _dst;
    angle = _angle_cmd;
  }

  TrajectoryManager::Mode save = _mod;
  if(dist < 0) {
    setMode(TrajectoryManager::BACKWARD);
    pos.coord(0) -= (double)dist * Math::cos<Math::DEGREE, double>(angle);
    pos.coord(1) -= (double)dist * Math::sin<Math::DEGREE, double>(angle);
  }
  else {
    setMode(TrajectoryManager::FORWARD);
    pos.coord(0) += (double)dist * Math::cos<Math::DEGREE, double>(angle);
    pos.coord(1) += (double)dist * Math::sin<Math::DEGREE, double>(angle);
  }
  io << pos.coord(0) << " " << pos.coord(1) << "\n";
  io << dist << " " << angle << "\n";
  gotoPosition(pos);
  setMode(save);
}

void RectTrajectoryManager::gotoAngle(s32 angle) {
  _state_handlers[REACH_ANGLE] = RectTrajectoryManager::_update_reach_angle;
  
  _angle_cmd = nearest_cmd_angle(raw2deg(_odo.getValue().coord(1)), -angle);
  _backward = false;

  _state = REACH_ANGLE;
}

void RectTrajectoryManager::gotoPosition(Vect<2, s32> pos) {
  _state_handlers[REACH_ANGLE] = RectTrajectoryManager::_update_reach_angle;
  _state_handlers[FOLLOW_TRAJECTORY] = RectTrajectoryManager::_update_follow_trajectory;
  _state_handlers[NEAR_END] = RectTrajectoryManager::_update_near_end;

  if(_state == NEAR_END) {
    _src = _dst;
  }
  else {
    _src = _pos.getValue();
  }

  _dst = pos;

  Vect<2, s32> dir = _dst - _src;
  _nor = (normal(dir) * 256) / dir.norm();
  _seg = _dst - _src;
  _seg_len = _seg.norm();

  if(_mod == FASTER) {
    s32 angle_b = nearest_cmd_angle(raw2deg(_odo.getValue().coord(1)), 180 - Math::atan2<Math::DEGREE>(dir.coord(1), dir.coord(0)));
    s32 angle_f = nearest_cmd_angle(raw2deg(_odo.getValue().coord(1)), -Math::atan2<Math::DEGREE>(dir.coord(1), dir.coord(0)));

    angle_b -= raw2deg(_odo.getValue().coord(1));
    angle_f -= raw2deg(_odo.getValue().coord(1));
    
    if(Math::abs(angle_b) < Math::abs(angle_f)) {
      _angle_cmd = raw2deg(_odo.getValue().coord(1)) + angle_b;
      _backward = true;
    }
    else {
      _angle_cmd = raw2deg(_odo.getValue().coord(1)) + angle_f;
      _backward = false;
    }
  }
  else if(_mod == BACKWARD) { 
    _angle_cmd = nearest_cmd_angle(raw2deg(_odo.getValue().coord(1)), 180 - Math::atan2<Math::DEGREE>(dir.coord(1), dir.coord(0)));
    _backward = true;    
  }
  else {
    _angle_cmd = nearest_cmd_angle(raw2deg(_odo.getValue().coord(1)), -Math::atan2<Math::DEGREE>(dir.coord(1), dir.coord(0)));
    _backward = false;
  }
  
  _dist_cmd = _odo.getValue().coord(0);

  _state = REACH_ANGLE_RECT;
  //io<< "Reach angle (rect)...\n";
}

void RectTrajectoryManager::update_reach_angle(void) {
  _robot.setValue(Vect<2, s32>(_dist_cmd, deg2raw(_angle_cmd)));

  if(Math::abs(raw2deg(_odo.getValue().coord(1)) - _angle_cmd) < 2 && Math::abs(_diff_a.doFilter(_odo.getValue().coord(1))) < 10) {
    _state = FOLLOW_TRAJECTORY_RECT;
    //io << "Follow trajectory (rect)...\n";
  }
}

void RectTrajectoryManager::update_follow_trajectory(void) {
  Vect<2,s32> pos_err = _dst - _pos.getValue();

  s32 ndiff = scal(pos_err, _nor);
  s32 angle_err = _pid_r.doFilter(ndiff);
  
  s32 dist_err = scal(pos_err, _seg) / _seg_len;

  if(_backward) {
    dist_err = -dist_err;
  }

  _robot.setValue(Vect<2, s32>(_odo.getValue().coord(0) + dist_err, deg2raw(_angle_cmd - angle_err)));

  if(scal(pos_err, pos_err) < 10000 + Math::abs(ndiff)) {
    _state = NEAR_END_RECT;

    if(_backward) {
      _dist_cmd = _odo.getValue().coord(0) - pos_err.norm();
    }
    else {
      _dist_cmd = _odo.getValue().coord(0) + pos_err.norm();
    }
    
    //io << "Near end...\n";
  }
}

void RectTrajectoryManager::update_near_end(void) {
  _robot.setValue(Vect<2, s32>(_dist_cmd, deg2raw(_angle_cmd)));
  
  if(Math::abs(_odo.getValue().coord(0) - _dist_cmd) < 20) {
    _state = STOP;
    //io << "Stop !\n";
  }
}
