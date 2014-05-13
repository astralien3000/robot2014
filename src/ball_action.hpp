#ifndef BALL_ACTION_HPP
#define BALL_ACTION_HPP

#include "action.hpp"

class BallAction : public Action {
public:
  BallAction(const Vect<2, s32>& pos, s32 number);
  
  s16 priority(void);
  Vect<2, s32> controlPoint(void);
  void doAction(void);
private:
  Vect<2, s32> _pos;
  s16 _number;
  bool _done;
};

#endif//Ball_ACTION_HPP
