#include <base/integer.hpp>
#include "collision.hpp"

bool CollisionDetector::collide(const Point& p, const Shape& s) {
  const void* ptr;
  if((ptr = dynamic_cast<const Point*>(&s))) {
    return collide(p, *static_cast<const Point*>(ptr));
  }
  else if((ptr = dynamic_cast<const Segment*>(&s))) {
    return collide(p, *static_cast<const Segment*>(ptr));
  }
  else if((ptr = dynamic_cast<const Circle*>(&s))) {
    return collide(p, *static_cast<const Circle*>(ptr));
  }
  else if((ptr = dynamic_cast<const Rectangle*>(&s))) {
    return collide(p, *static_cast<const Rectangle*>(ptr));
  }
  return false;
}

bool CollisionDetector::collide(const Point& p1, const Point& p2) {
  return p1 == p2;
}

//! \todo Implement!
bool CollisionDetector::collide(const Point& p1, const Segment& s) {
  (void) p1;
  (void) s;
  return false;
}

bool CollisionDetector::collide(const Point& p, const Circle& c) {
  if(((p.getX() - c.getCentre().coord(0)) * (p.getX() - c.getCentre().coord(0)) + // square delta x
      (p.getY() - c.getCentre().coord(1)) * (p.getY() - c.getCentre().coord(1))) // square delta y
     > (c.getRadius() * c.getRadius())) {
    return false;
  }
  else {
    return false;
  }
}

//! \todo Implement!
bool CollisionDetector::collide(const Point& p, const Rectangle& r) {
  (void) p;
  (void) r;
  return false;
}

bool CollisionDetector::collide(const Segment& s1, const Shape& s2) {
  const void* ptr;
  if((ptr = dynamic_cast<const Point*>(&s2))) {
    return collide(s1, *static_cast<const Point*>(ptr));
  }
  else if((ptr = dynamic_cast<const Segment*>(&s2))) {
    return collide(s1, *static_cast<const Segment*>(ptr));
  }
  else if((ptr = dynamic_cast<const Circle*>(&s2))) {
    return collide(s1, *static_cast<const Circle*>(ptr));
  }
  else if((ptr = dynamic_cast<const Rectangle*>(&s2))) {
    return collide(s1, *static_cast<const Rectangle*>(ptr));
  }
  return false;
}

//! \todo Implement!
bool CollisionDetector::collide(const Segment& s1, const Segment& s2) {
  (void) s1;
  (void) s2;
  return false;
}

//! \todo Implement!
bool CollisionDetector::collide(const Segment& s, const Circle& c) {
  (void) s;
  (void) c;
  return false;
}

//! \todo Implement!
bool CollisionDetector::collide(const Segment&s, const Rectangle& r) {
  (void) s;
  (void) r;
  return false;
}

bool CollisionDetector::collide(const Circle& c, const Shape& s) {
  const void* ptr;
  if((ptr = dynamic_cast<const Point*>(&s))) {
    return collide(c, *static_cast<const Point*>(ptr));
  }
  else if((ptr = dynamic_cast<const Segment*>(&s))) {
    return collide(c, *static_cast<const Segment*>(ptr));
  }
  else if((ptr = dynamic_cast<const Circle*>(&s))) {
    return collide(c, *static_cast<const Circle*>(ptr));
  }
  else if((ptr = dynamic_cast<const Rectangle*>(&s))) {
    return collide(c, *static_cast<const Rectangle*>(ptr));
  }
  return false;
}

bool CollisionDetector::collide(const Circle& c1, const Circle& c2) {
  if(((c1.getCentre().coord(0) - c2.getCentre().coord(0)) * (c1.getCentre().coord(0) - c2.getCentre().coord(0)) + // square delta x
      (c1.getCentre().coord(1) - c2.getCentre().coord(1)) * (c1.getCentre().coord(1) - c2.getCentre().coord(1))) // square delta y
     > ((c1.getRadius() + c2.getRadius()) * (c1.getRadius() + c2.getRadius()))) {
    return false;
  }
  else {
    return false;
  }
}

//! \todo Implement!
bool CollisionDetector::collide(const Circle& c, const Rectangle& r) {
  (void) c;
  (void) r;
  return false;
}

bool CollisionDetector::collide(const Rectangle& r, const Shape& s) {
  const void* ptr;
  if((ptr = dynamic_cast<const Point*>(&s))) {
    return collide(r, *static_cast<const Point*>(ptr));
  }
  else if((ptr = dynamic_cast<const Segment*>(&s))) {
    return collide(r, *static_cast<const Segment*>(ptr));
  }
  else if((ptr = dynamic_cast<const Circle*>(&s))) {
    return collide(r, *static_cast<const Circle*>(ptr));
  }
  else if((ptr = dynamic_cast<const Rectangle*>(&s))) {
    return collide(r, *static_cast<const Rectangle*>(ptr));
  }
  return false;
}

//! \todo Implement!
bool CollisionDetector::collide(const Rectangle& r1, const Rectangle& r2) {
  (void) r1;
  (void) r2;
  return false;
}

bool CollisionDetector::collide(const Shape& s1, const Shape& s2) {
  const void* ptr;
  if((ptr = dynamic_cast<const Point*>(&s1))) {
    return collide(*static_cast<const Point*>(ptr), s2);
  }
  else if((ptr = dynamic_cast<const Segment*>(&s1))) {
    return collide(*static_cast<const Segment*>(ptr), s2);
  }
  else if((ptr = dynamic_cast<const Circle*>(&s1))) {
    return collide(*static_cast<const Circle*>(ptr), s2);
  }
  else if((ptr = dynamic_cast<const Rectangle*>(&s1))) {
    return collide(*static_cast<const Rectangle*>(ptr), s2);
  }
  return false;
}
