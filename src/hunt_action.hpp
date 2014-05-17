#ifndef HUNT_ACTION_HPP
#define HUNT_ACTION_HPP

#include "action.hpp"

class HuntAction : public Action {
public:
  HuntAction(const Vect<2, s32>& pos, s32 number);
  
  s16 priority(void);
  Vect<2, s32> controlPoint(void);
  void doAction(void);
private:
  Vect<2, s32> _pos;
  s16 _number;
};

#endif//HUNT_ACTION_HPP
