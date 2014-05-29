#ifndef CAPTURE_ACTION_HPP
#define CAPTURE_ACTION_HPP

#include "action.hpp"

class CaptureAction : public Action {
public:
  CaptureAction(const Vect<2, s32>& pos, const Vect<2, s32>& mamouth);
  
  s16 priority(void);
  Vect<2, s32> controlPoint(void);
  enum Error doAction(void);

private:
  Vect<2, s32> _pos;
  Vect<2, s32> _mamouth;

  
};

#endif//CAPTURE_ACTION_HPP
