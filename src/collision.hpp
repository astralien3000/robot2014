#ifndef COLLISION_HPP
#define COLLISION_HPP

#include "shape.hpp"
#include "rectangle.hpp"
#include "circle.hpp"
#include "segment.hpp"
#include "point.hpp"

class CollisionDetector {
public:
  static bool collide(const Point& p, const Shape& s);
  static bool collide(const Point& p1, const Point& p2);
  static bool collide(const Point& p, const Segment& s);
  static bool collide(const Point& p, const Circle& c);
  static bool collide(const Point& p, const Rectangle& r);
  
  static bool collide(const Segment& s1, const Shape& s2);
  inline static bool collide(const Segment& s, const Point& p) {
    return collide(p, s);
  }
  static bool collide(const Segment& s1, const Segment& s2);
  static bool collide(const Segment& s, const Circle& c);
  static bool collide(const Segment& s, const Rectangle& r);
  
  static bool collide(const Circle& c, const Shape& s);
  inline static bool collide(const Circle& c, const Point& p) {
    return collide(p, c);
  }
  inline static bool collide(const Circle& c, const Segment& s) {
    return collide(s, c);
  }
  static bool collide(const Circle& c1, const Circle& c2);
  static bool collide(const Circle& c, const Rectangle& r);
  
  static bool collide(const Rectangle& r, const Shape& s);
  inline static bool collide(const Rectangle& r, const Point& p) {
    return collide(p, r);
  }
  inline static bool collide(const Rectangle& r, const Segment& s) {
    return collide(s, r);
  }
  inline static bool collide(const Rectangle& r, const Circle& c) {
    return collide(c, r);
  }
  static bool collide(const Rectangle& r1, const Rectangle& r2);
  
  static bool collide(const Shape& s1, const Shape& s2);
};

#endif//COLLISION_HPP
