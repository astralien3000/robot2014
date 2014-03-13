#ifndef RECTANGLE_HPP
#define RECTANGLE_HPP

#include <base/integer.hpp>
#include <math/vect.hpp>
#include "shape.hpp"

class Rectangle : public Shape {
  Vect<2, s32> _centre;
  Vect<2, s32> _topleft;
  Vect<2, s32> _topright;
  Vect<2, s32> _botleft;
  Vect<2, s32> _botright;
public:
  //! \todo Implement!
  inline Rectangle(const Vect<2, s32>& centre, const Vect<2, s32>& a, const Vect<2, s32>& b)
    : _centre(centre) {
    (void) a;
    (void) b;
  }
  
  //! \todo Implement!
  inline Rectangle(const Vect<2, s32>& centre, s32 h, s32 w, s32 alpha)
    : _centre(centre) {
    (void) h;
    (void) w;
    (void) alpha;
  }
  
  inline Rectangle(const Rectangle& other)
    : _centre(other._centre), _topleft(other._topleft), _topright(other._topright),
      _botleft(other._botleft), _botright(other._botright) {
  }
  
  inline Rectangle& operator=(const Rectangle& other) {
    _centre = other._centre;
    _topleft = other._topleft;
    _topright = other._topright;
    _botleft = other._botleft;
    _botright = other._botright;
    return (*this);
  }
  
  inline bool operator==(const Rectangle& other) const {
    return 
      _centre == other._centre &&
      _topleft == other._topleft &&
      _topright == other._topright &&
      _botleft == other._botleft &&
      _botright == other._botright;
  }
};

#endif//RECTANGLE_HPP
