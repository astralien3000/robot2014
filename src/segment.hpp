#ifndef SEGMENT_HPP
#define SEGMENT_HPP

#include <base/integer.hpp>
#include <math/vect.hpp>
#include "shape.hpp"

class Segment : public Shape {
  Vect<2, s32> _a;
  Vect<2, s32> _b;
public:
  inline Segment(const Vect<2, s32>& a, const Vect<2, s32>& b)
    : _a(a), _b(b) {
  }
  
  inline Segment(const Segment& s)
    : _a(s._a), _b(s._b) {
  }
  
  inline Segment& operator=(const Segment& s) {
    _a = s._a;
    _b = s._b;
    return (*this);
  }
  
  inline bool operator==(const Segment& s) const {
    return _a == s._a && _b == s._b;
  }
  
  inline Vect<2, s32>& getA(void) {
    return _a;
  }
  
  inline Vect<2, s32>& getB(void) {
    return _b;
  }
  
  inline const Vect<2, s32>& getA(void) const {
    return _a;
  }
  
  inline const Vect<2, s32>& getB(void) const {
    return _b;
  }
  
  inline s32 length(void) const {
    return (Vect<2, s32>(_a.coord(0) -_b.coord(0), _a.coord(1) - b._coord(0))).norm();
  }
};

#endif//SEGMENT_HPP
