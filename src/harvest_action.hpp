#ifndef HARVEST_ACTION_HPP
#define HARVEST_ACTION_HPP

#include "action.hpp"

class HarvestAction : public Action {
public:
  HarvestAction(const Vect<2, s32>& pos, s32 angle);
  
  s16 priority(void);
  Vect<2, s32> controlPoint(void);
  void doAction(void);

private:

  Vect<2, s32> _begin_point;
  Vect<2, s32> _end_point;

  static const s32 DIST_FROM_WALL = 300;
  static const s32 DIST_FROM_TREE = 300;
};

#endif//HARVEST_ACTION_HPP
