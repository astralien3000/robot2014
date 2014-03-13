#ifndef AABB_HPP
#define AABB_HPP

#include <base/integer.hpp>
#include <math/vect.hpp>
#include "shape.hpp"

class AABB : public Shape {
  Vect<2, s32> _o;
  s32 _w;
  s32 _h;
public:
  inline AABB(const Vect<2, s32>& o, const s32& w, const s32& h)
    : _o(o), _w(w), _h(h) {
  }
  
  inline AABB(const s32& x, const s32& y, const s32& w, const s32& h)
    : _o(x, y), _w(w), _h(h) {
  }
  
  inline AABB(const AABB& a)
    : _o(a._o), _w(a._w), _h(a._h) {
  }
  
  inline AABB& operator=(const AABB& other) {
    _o = other._o;
    _w = other._w;
    _h = other._h;
    return (*this);
  }
  
  inline bool operator==(const AABB& other) const {
    return _o == other._o && _w == other._w && _h == other._h;
  }
  
  //! \brief Returns AABB's reference point
  inline Vect<2, s32>& o(void) {
    return _o;
  }
  
  inline s32& w(void) {
    return _w;
  }
  
  inline s32& h(void) {
    return _h;
  }
  
  inline const Vect<2, s32>& o(void) const {
    return _o;
  }
  
  inline const s32& w(void) const {
    return _w;
  }
  
  inline const s32& h(void) const {
    return _h;
  }
};

#endif//AABB_HPP
