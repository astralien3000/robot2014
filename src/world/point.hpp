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
  
  inline Vect<2, s32>& p(void) {
    return _p;
  }
  
  inline const Vect<2, s32>& p(void) const {
    return _p;
  }
  
  inline const s32& x(void) const {
    return _p.coord(0);
  }
  
  inline s32& x(void) {
    return _p.coord(0);
  }
  
  inline s32& operator[](u8 index) {
    switch(index) {
    case 0:
      return _x;
    case 1:
      return _y;
    default:
      return *((s32*) 0);
    }
  }
  
  inline const s32& y(void) const {
    return _p.coord(1);
  }
  
  inline s32& y(void) {
    return _p.coord(1);
  }
  
  inline const s32& operator[](u8 index) const {
    switch(index) {
    case 0:
      return _x;
    case 1:
      return _y;
    default:
      return *((s32*) 0);
    }
  }
};

#endif//POINT_HPP
