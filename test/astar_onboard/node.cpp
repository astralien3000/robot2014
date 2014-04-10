#include "node.hpp"

Node::Node(void) : a(0), b(0) {}

void Node::init(void) {
  this->a = 0;
  this->b = 0;
}

void Node::setXY(uint8_t x, uint8_t y) {
  // if (x > 31)
  //   std::cout << "x > 31 : " << (int) x << std::endl;
  // if (y > 15)
  //   std::cout << "y > 15 : " << (int) y << std::endl;
  this->b &= (~511);
  this->b |= x;
  this->b |= (y<<5);
}

uint8_t Node::x(void) const {
  return (this->b & (31));
}

uint8_t Node::y(void) const {
  return ((this->b & (15<<5)) >> 5);
}

void Node::setCost(uint8_t cost) {
  // if (cost > 63)
  //   std::cout << "cost > 63 : " << (int) cost << std::endl;
  this->b &= (~(63 << 10));
  this->b |= (cost<<10);
}

uint8_t Node::cost(void) const {
  return ((this->b & (63<<10)) >> 10);
}

void Node::setPred(uint8_t pred) {
  this->a &= (1<<7);
  this->a |= pred;
}

uint8_t Node::pred(void) const {
  return (this->a & 127);
}

void Node::setOpen(void) {
  this->a |= (1<<7);
}

bool Node::isOpen(void) const {
  return (((this->a & (1<<7)) >> 7) == 1);
}

void Node::setClosed(void) {
  this->b |= (1<<9);
}

bool Node::isClosed(void) const {
  return (((this->b & (1<<9)) >> 9) == 1);
}

/*
void Node::debug(void) {
  for (int i=7; i>=0; i--) {
    std::cout << this->a / (1<<i) % 2;
  }
  std::cout << std::endl;

  for (int i=15; i>=0; i--) {
    std::cout << this->b / (1<<i) % 2;
  }
  std::cout << std::endl;
}
*/
