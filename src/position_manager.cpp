#include "position_manager.hpp"

PositionManager::PositionManager(s32& pos_x_reg, s32& pos_y_reg, u16& a_reg)
  : _x((volatile s32&)pos_x_reg), _y((volatile s32&)pos_y_reg),
    _a((volatile u16&)a_reg),
    _off_x(0), _off_y(0), _off_a(0) {
}

PositionManager::PositionManager(volatile s32& pos_x_reg, volatile s32& pos_y_reg, volatile u16& a_reg)
  : _x(pos_x_reg), _y(pos_y_reg),
    _a(a_reg),
    _off_x(0), _off_y(0), _off_a(0) {
}

Vect<2, s32> PositionManager::getValue(void) {
  return Vect<2, s32>(_off_x + _x / _imp_per_u_x, 
  		      _off_y + _y / _imp_per_u_y);
}

s16 PositionManager::angle(void) {
  s16 ret = (_a >> 7) & 0x1FF;
  return (_off_a + ret) % 360;
}

void PositionManager::setImpPerUnitX(s32 val) {
  _imp_per_u_x = val;
}

void PositionManager::setImpPerUnitY(s32 val) {
  _imp_per_u_y = val;
}

void PositionManager::setAngle(s16 val) {
  _off_a = val - ((_a >> 7) & 0x1FF);
}

void PositionManager::setX(s32 val) {
  _off_x = val - _x / _imp_per_u_x;
}

void PositionManager::setY(s32 val) {
  _off_y = val - _y / _imp_per_u_y;
}

void PositionManager::setPosition(const Vect<2, s32>& pos) {
  setX(pos.coord(0));
  setY(pos.coord(1));
}
