#ifndef MASTER_ACTION_HPP
#define MASTER_ACTION_HPP

#include "action.hpp"

class MasterAction : public Action {
public:
  MasterAction(const Vect<2, s32>& pos, s32 angle);

  s16 priority(void);
  Vect<2, s32> controlPoint(void);
  void doAction(void);

private:
  Vect<2, s32> _side_point[MAX_SIDE];

  static const s32 DIST_MM = 300;

};

#endif//MASTER_ACTION_HPP
