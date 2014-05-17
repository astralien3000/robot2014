#ifndef CALIBRATE_ACTION_HPP
#define CALIBRATE_ACTION_HPP

#include "action.hpp"

class CalibrateAction : public Action {
public:
  CalibrateAction(const Vect<2, s32>& pos, const Vect<2, s32>& corner);

  s16 priority(void);
  Vect<2, s32> controlPoint(void);
  void doAction(void);

private:
  
};

#endif//CALIBRATE_ACTION_HPP
