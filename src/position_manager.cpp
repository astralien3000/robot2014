#include "position_manager.hpp"

PositionManager::PositionManager(s32& pos_x_reg, s32& pos_y_reg, u16& a_reg)
  : _x((volatile s32&)pos_x_reg), _y((volatile s32&)pos_y_reg),
    _a((volatile u16&)a_reg) {
}

PositionManager::PositionManager(volatile s32& pos_x_reg, volatile s32& pos_y_reg, volatile u16& a_reg)
  : _x(pos_x_reg), _y(pos_y_reg),
    _a(a_reg) {
}

Vect<2, s32> PositionManager::getValue(void) {
  return Vect<2, s32>(_x / _imp_per_u_x, _y / _imp_per_u_y);
}

s16 PositionManager::angle(void) {
  return (_a >> 7) & 0x1FF;
}

void PositionManager::setImpPerUnitX(s32 val) {
  _imp_per_u_x = val;
}

void PositionManager::setImpPerUnitY(s32 val) {
  _imp_per_u_y = val;
}
