#ifndef HUNT_ACTION_HPP
#define HUNT_ACTION_HPP

#include "action.hpp"

class HuntAction : public Action {
public:
  static const s32 OFFSET = 20;
  HuntAction(const Vect<2, s32>& pos, const Vect<2, s32>& mamouth, s32 number);
  
  s16 priority(void);
  Vect<2, s32> controlPoint(void);
  void doAction(void);
private:
  Vect<2, s32> _pos;
  Vect<2, s32> _mamouth;
  s16 _number;
};

#endif//HUNT_ACTION_HPP
