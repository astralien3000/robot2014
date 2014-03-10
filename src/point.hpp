#ifndef POINT_HPP
#define POINT_HPP

#include <base/integer.hpp>
#include <math/vect.hpp>
#include "shape.hpp"

class Point : public Shape {
  Vect<2, s32> _p;
public:
  inline Point(const s32& x, const s32& y)
    : _p(x, y) {
  }
  
  inline Point(const Vect<2, s32>& p)
    : _p(p) {
  }
  
  inline Point(const Point& p)
    : _p(p._p) {
  }
  
  inline Point& operator=(const Point& p) {
    _p = p._p;
    return (*this);
  }
  
  inline Point& operator=(const Vect<2, s32>& p) {
    _p = p;
    return (*this);
  }
  
  inline bool operator==(const Point& p) const {
    return _p == p._p;
  }
  
  inline Vect<2, s32>& getP(void) {
    return _p;
  }
  
  inline const Vect<2, s32>& getP(void) const {
    return _p;
  }
};

#endif//POINT_HPP
