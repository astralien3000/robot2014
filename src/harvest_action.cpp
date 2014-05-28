#include "harvest_action.hpp"
#include "asserv.hpp"


HarvestAction::HarvestAction(const Vect<2, s32>& pos, s32 angle, s8 bonus = 0) {
  const s32 MULT = 1024;
  Vect<2, s32> dir(MULT * Math::cos<Math::DEGREE, double>(angle), MULT * Math::sin<Math::DEGREE, double>(angle));
  Vect<2, s32> nor(MULT * Math::cos<Math::DEGREE, double>(angle + 90), MULT * Math::sin<Math::DEGREE, double>(angle + 90));
  Vect<2, s32> nor2 = nor;

  dir *= DIST_FROM_TREE;
  nor *= DIST_FROM_WALL;

  dir /= MULT;
  nor /= MULT;
  
  _begin_point = pos - dir + nor;
  _end_point = pos + dir + nor;

  nor2 *= CTRL_DIST_FROM_WALL;
  nor2 /= MULT;

  _ctrl_point = pos - dir + nor2;

  _bonus = bonus;
}

s16 HarvestAction::priority(void) {
  if (_static_priority == 0) {
    return 0;
  }
  // Future improvement ?
  _static_priority++;
  return _static_priority * (_bonus + 10000/((controlPoint() - positionManager().getValue()).norm() +1));
}

#include "devices.hpp"

Vect<2, s32> HarvestAction::controlPoint(void) {
  io << _begin_point.coord(0) << " " << _begin_point.coord(1) << "\n";
  io << _end_point.coord(0) << " " << _end_point.coord(1) << "\n";
  return _ctrl_point;
}


enum Error HarvestAction::doAction(void) {
  io << "DO HARVERST\n";
  trajectoryManager().setMode(TrajectoryManager::FORWARD);

  // Face the wall
  trajectoryManager().gotoPosition(_begin_point);
  while(!trajectoryManager().isEnded()) {
    if (robot().getValue()) {
      return SKATING;
    }
    if (check_for_collision(60)) {
      return IMPOSSIBLE;
    }
  }

  asserv_speed_slow();

  // Calibrate
  for(u8 i = 0 ; i < 4 ; i++) {
    trajectoryManager().gotoDistance(3000);
    robot().unlock();
    while(!robot().getValue()) {
    }
  }
  
  s32 angle = positionManager().angle() % 360;
  if(355 < angle || (-5 < angle && angle < 5)) {
    // ANGLE 0
    io << "ANGLE 0\n";
    positionManager().setX(1500 - 125);
  }
  else if(85 < angle && angle < 95) {
    // ANGLE 90
    io << "ANGLE 90\n";
    //positionManager().setY();
    io << "ERROR : NO FRUITS HERE !!\n";
  }
  else if(175 < angle && angle < 185) {
    // ANGLE 180
    io << "ANGLE 180\n";
    positionManager().setX(-1500 + 125);
  }
  else if(265 < angle && angle < 275) {
    // ANGLE 270
    io << "ANGLE 270\n";
    positionManager().setY(-950 + 125);
  }
  
  // Go far from wall
  trajectoryManager().gotoDistance(-200);
  while(!trajectoryManager().isEnded()) {
    if(robot().getValue()) {
      robot().unlock();
    }
  }

  // Goto begin point
  trajectoryManager().setMode(TrajectoryManager::FASTER);
  trajectoryManager().gotoPosition(_begin_point);
  while(!trajectoryManager().isEnded()) {
    if (robot().getValue()) {
      return SKATING;
    }
    if (check_for_collision(60)) {
      return IMPOSSIBLE;
    }
  }

  // Harvest
  trajectoryManager().setMode(TrajectoryManager::FORWARD);
  trajectoryManager().gotoPosition(_end_point);
  while(!trajectoryManager().isEnded()) {
    if (robot().getValue()) {
      return SKATING;
    }
    if (check_for_collision(60)) {
      return IMPOSSIBLE;
    }
  }

  asserv_speed_normal();

  // Go far
  trajectoryManager().gotoAngle(pos.angle() - 90);
  while(!trajectoryManager().isEnded()) {
    if (robot().getValue()) {
      return SKATING;
    }
    if (check_for_collision(60)) {
      return IMPOSSIBLE;
    }
  }

  trajectoryManager().gotoDistance(100);
  while(!trajectoryManager().isEnded()) {
    if (robot().getValue()) {
      return SKATING;
    }
    if (check_for_collision(60)) {
      return IMPOSSIBLE;
    }
  }
  
  trajectoryManager().setMode(TrajectoryManager::FASTER);

  _fruit++;
  _static_priority = 0;

  return SUCCESS;
}

