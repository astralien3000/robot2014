#ifndef POSITION_MANAGER_HPP
#define POSITION_MANAGER_HPP

#include <device/input.hpp>

#include <base/integer.hpp>
#include <math/vect.hpp>

class PositionManager : public Input< Vect<2, s32> > {
public:
  PositionManager(s32& pos_x_reg, s32& pos_y_reg, u16& a_reg);
  PositionManager(volatile s32& pos_x_reg, volatile s32& pos_y_reg, volatile u16& a_reg);

  Vect<2, s32> getValue(void);

  s16 angle(void);

  void setImpPerUnitX(s32);
  void setImpPerUnitY(s32);

private:
  volatile s32& _x;
  volatile s32& _y;
  volatile u16& _a;

  s32 _imp_per_u_x;
  s32 _imp_per_u_y;
};

#endif//POSITION_MANAGER_HPP
