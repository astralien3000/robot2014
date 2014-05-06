#ifndef PAINT_ACTION_HPP
#define PAINT_ACTION_HPP

#include "action.hpp"

class PaintAction : public Action {
public:
  PaintAction(void);
  
  s16 priority(void);
  Vect<2, s32> controlPoint(void);
  void doAction(void);
private:
  bool _done;
};

#endif//PAINT_ACTION_HPP
