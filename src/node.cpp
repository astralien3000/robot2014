#include "node.hpp"

Node::Node(void) : b(0), a(0) {}

void Node::init(void) {
  this->a = 0;
  this->b = 0;
}

void Node::setXY(uint8_t x, uint8_t y) {
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
  if (cost > 63) {
    cost = 63;
  }
  this->b &= (~(63 << 10));
  this->b |= (cost<<10);
}

uint8_t Node::cost(void) const {
  return ((this->b & (63<<10)) >> 10);
}

void Node::setPred(uint8_t pred) {
  this->a = pred;
}

uint8_t Node::pred(void) const {
  return this->a;
}

void Node::setClosed(void) {
  this->b |= (1<<9);
}

bool Node::isClosed(void) const {
  return (((this->b & (1<<9)) >> 9) == 1);
}

