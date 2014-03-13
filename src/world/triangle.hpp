#ifndef TRIANGLE_HPP
#define TRIANGLE_HPP

#include <base/integer.hpp>
#include <math/vect.hpp>
#include "shape.hpp"

class Triangle : public Shape {
  Vect<2, s32> _a;
  Vect<2, s32> _b;
  Vect<2, s32> _c;
public:
  inline Triangle(const Vect<2, s32>& a, const Vect<2, s32>& b, const Vect<2, s32>& c)
    : _a(a), _b(b), _c(c) {
  }
  
  inline Triangle(const s32& x1, const s32& y1, const s32& x2, const s32& y2, const s32& x3, const s32& y3)
    : _a(x1, y1), _b(x2, y2), _c(x3, y3) {
  }
  
  inline Triangle(const Triangle& t)
    : _a(t._a), _b(t._b), _c(t._c) {
  }
  
  inline Triangle& operator=(const Triangle& t) {
    _a = t._a;
    _b = t._b;
    _c = t._c;
    return (*this);
  }
  
  inline bool operator==(const Triangle& t) const {
    return _a == t._a && _b == t._b && _c == t._c;
  }
  
  inline Vect<2, s32>& getA(void) {
    return _a;
  }
  
  inline Vect<2, s32>& getB(void) {
    return _b;
  }
  
  inline Vect<2, s32>& getC(void) {
    return _c;
  }
  
  inline const Vect<2, s32>& getA(void) const {
    return _a;
  }
  
  inline const Vect<2, s32>& getB(void) const {
    return _b;
  }
  
  inline const Vect<2, s32>& getC(void) const {
    return _c;
  }
};

#endif//TRIANGLE_HPP
