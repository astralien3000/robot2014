#ifndef DEPOSIT_ACTION_HPP
#define DEPOSIT_ACTION_HPP

#include "action.hpp"

class DepositAction : public Action {
public:
  DepositAction(void);
  
  s16 priority(void);
  Vect<2, s32> controlPoint(void);
  enum Error doAction(void);
private:
};

#endif//DEPOSIT_ACTION_HPP
