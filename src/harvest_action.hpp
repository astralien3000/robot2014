#ifndef HARVEST_ACTION_HPP
#define HARVEST_ACTION_HPP

#include "action.hpp"

class HarvestAction : public Action {
public:
  HarvestAction(const Vect<2, s32>& pos, s32 angle, s8 bonus);
  
  s16 priority(void);
  Vect<2, s32> controlPoint(void);
  enum Error doAction(void);

private:

  Vect<2, s32> _begin_point;
  Vect<2, s32> _ctrl_point;
  Vect<2, s32> _end_ctrl_point;
  Vect<2, s32> _end_point;
  s8 _bonus;

  static const s32 CTRL_DIST_FROM_WALL = 375;
  static const s32 DIST_FROM_WALL = 260;
  static const s32 DIST_FROM_TREE = 220;
};

#endif//HARVEST_ACTION_HPP
