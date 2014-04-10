#include "aversive--/include/common/base/integer.hpp"
#include <iostream>

class Node {
private:
  uint8_t a;
  uint16_t b;
public:
  static const uint8_t MAX_COST = 63;
  static const uint8_t MAX_NB = 255;
  static const uint8_t MAX_X = 31;
  static const uint8_t MAX_Y = 15;
  Node(void);
  void init(void);
  void setXY(uint8_t x, uint8_t y);
  uint8_t x(void) const;
  uint8_t y(void) const;
  void setCost(uint8_t cost);
  uint8_t cost(void) const;
  void setPred(uint8_t pred);
  uint8_t pred(void) const;
  void setClosed(void);
  bool isClosed(void) const;
};
