#ifndef QUADRILATERAL_HPP
#define QUADRILATERAL_HPP

#include <base/integer.hpp>
#include <math/vect.hpp>
#include "shape.hpp"

class Quadrilateral : public Shape {
  Vect<2, s32> _a;
  Vect<2, s32> _b;
  Vect<2, s32> _c;
  Vect<2, s32> _d;
public:
  inline Quadrilateral(const Vect<2, s32>& a, const Vect<2, s32>& b, const Vect<2, s32>& c, const Vect<2, s32>& d)
    : _a(a), _b(b), _c(c), _d(d) {
  }
  
  inline Quadrilateral(const s32& x1, const s32& y1, const s32& x2, const s32& y2, const s32& x3, const s32& y3, const s32& x4, const s32& y4)
    : _a(x1, y1), _b(x2, y2), _c(x3, y3), _d(x4, y4) {
  }
  
  inline Quadrilateral(const Quadrilateral& q)
  : _a(q._a), _b(q._b), _c(q._c), _d(q._d) {
  }
  
  inline Quadrilateral& operator=(const Quadrilateral& q) {
    _a = q._a;
    _b = q._b;
    _c = q._c;
    _d = q._d;
    return (*this);
  }
  
  inline bool operator==(const Quadrilateral& q) const {
    return _a == q._a && _b == q._b && _c == q._c && _d == q._d;
  }
  
  inline Vect<2, s32>& a(void) {
    return _a;
  }
  
  inline Vect<2, s32>& b(void) {
    return _b;
  }
  
  inline Vect<2, s32>& c(void) {
    return _c;
  }
  
  inline Vect<2, s32>& d(void) {
    return _d;
  }
  
  inline const Vect<2, s32>& a(void) const {
    return _a;
  }
  
  inline const Vect<2, s32>& b(void) const {
    return _b;
  }
  
  inline const Vect<2, s32>& c(void) const {
    return _c;
  }
  
  inline const Vect<2, s32>& d(void) const {
    return _d;
  }
};

#endif//QUADRILATERAL_HPP
