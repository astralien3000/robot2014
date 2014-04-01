#include "rect_trajectory_manager.hpp"

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

  _angle_cmd = nearest_cmd_angle(_odo.getValue().coord(1)/10, -Math::atan2<Math::DEGREE>(dir.coord(1), dir.coord(0)));
  _backward = false;

  _dist_cmd = _odo.getValue().coord(0);

  _state = REACH_ANGLE_RECT;
  //io<< "Reach angle (rect)...\n";
}

void RectTrajectoryManager::update_reach_angle(void) {
  _robot.setValue(Vect<2, s32>(_dist_cmd, _angle_cmd*10));

  if(Math::abs(_odo.getValue().coord(1) - _angle_cmd*10) < 10 && Math::abs(_diff_a.doFilter(_odo.getValue().coord(1))) < 10) {
    _state = FOLLOW_TRAJECTORY_RECT;
    //io << "Follow trajectory (rect)...\n";
  }
}

void RectTrajectoryManager::update_follow_trajectory(void) {
  Vect<2,s32> pos_err = _dst - _pos.getValue();

  s32 ndiff = scal(pos_err, _nor);
  s32 angle_err = _pid_r.doFilter(ndiff);
  
  Vect<2, s32> trv = _dst - _src;
  s32 dist_err = scal(pos_err, trv) / trv.norm();
  
  _robot.setValue(Vect<2, s32>(_odo.getValue().coord(0) + dist_err, (_angle_cmd - angle_err)*10));

  if(Math::abs(scal(pos_err, pos_err)) < 10000 + ndiff) {
    _state = NEAR_END_RECT;

    _dist_cmd = _odo.getValue().coord(0) + pos_err.norm();
    
    //io << "Near end...\n";
  }
}

void RectTrajectoryManager::update_near_end(void) {
  _robot.setValue(Vect<2, s32>(_dist_cmd, _angle_cmd*10));
  
  if(Math::abs(_odo.getValue().coord(0) - _dist_cmd) < 20) {
    _state = STOP;
    //io << "Stop !\n";
  }
}
