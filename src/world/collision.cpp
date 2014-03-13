#include <base/integer.hpp>
#include "collision.hpp"

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
  else if((ptr = dynamic_cast<const AABB*>(&s1))) {
    return collide(*static_cast<const AABB*>(ptr), s2);
  }
  else if((ptr = dynamic_cast<const Triangle*>(&s1))) {
    return collide(*static_cast<const Triangle*>(ptr), s2);
  }
  else if((ptr = dynamic_cast<const Quadrilateral*>(&s1))) {
    return collide(*static_cast<const Quadrilateral*>(ptr), s2);
  }
  return false;
}

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
  else if((ptr = dynamic_cast<const AABB*>(&s))) {
    return collide(p, *static_cast<const AABB*>(ptr));
  }
  else if((ptr = dynamic_cast<const Triangle*>(&s))) {
    return collide(p, *static_cast<const Triangle*>(ptr));
  }
  else if((ptr = dynamic_cast<const Quadrilateral*>(&s))) {
    return collide(p, *static_cast<const Quadrilateral*>(ptr));
  }
  return false;
}

bool CollisionDetector::collide(const Point& p1, const Point& p2) {
  return p1 == p2;
}

bool CollisionDetector::collide(const Point& p, const Segment& s) {
  const Vect<2, s32>& A = s.getA();
  const Vect<2, s32>& B = s.getB();
  
  // If the crossproduct is strictly positive, the three points are not aligned.
  if(((p.getY() - A.coord(1)) * (B.coord(0) - A.coord(0)) - (p.getX() - A.coord(0)) * (B.coord(1) - A.coord(1))) > 0) {
    return false;
  }
  
  // Dot product of B-A and C-A
  const s32 dot_product = (p.getX() - A.coord(0)) * (B.coord(0) - A.coord(0)) + (p.getX() - A.coord(0)) * (B.coord(1) - A.coord(1));
  if(dot_product < 0) {
    return false;
  }
  
  const s32 squared_delta_AB = (B.coord(0) - A.coord(0)) * (B.coord(0) - A.coord(0)) + (B.coord(1) - A.coord(1)) * (B.coord(1) - A.coord(1));
  if(dot_product >  squared_delta_AB) {
    return false;
  }
  
  return true;
}

bool CollisionDetector::collide(const Point& p, const Circle& c) {
  if(((p.getX() - c.getCentre().coord(0)) * (p.getX() - c.getCentre().coord(0)) + // square delta x
      (p.getY() - c.getCentre().coord(1)) * (p.getY() - c.getCentre().coord(1))) // square delta y
     > (c.getRadius() * c.getRadius())) {
    return false;
  }
  else {
    return true;
  }
}

//! \todo Implement!
bool CollisionDetector::collide(const Point& p, const AABB& a) {
  (void) p;
  (void) a;
  return false;
}

//! \todo Implement!
bool CollisionDetector::collide(const Point& p, const Triangle& t) {
  (void) p;
  (void) t;
  return false;
}

//! \todo Implement!
bool CollisionDetector::collide(const Point& p, const Quadrilateral& q) {
  (void) p;
  (void) q;
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
  else if((ptr = dynamic_cast<const AABB*>(&s2))) {
    return collide(s1, *static_cast<const AABB*>(ptr));
  }
  else if((ptr = dynamic_cast<const Triangle*>(&s2))) {
    return collide(s1, *static_cast<const Triangle*>(ptr));
  }
  else if((ptr = dynamic_cast<const Quadrilateral*>(&s2))) {
    return collide(s1, *static_cast<const Quadrilateral*>(ptr));
  }
  return false;
}

bool CollisionDetector::collide(const Segment& s1, const Segment& s2) {
  (void) s1;
  (void) s2;
  
  const Vect<2, s32>& A = s1.getA();
  const Vect<2, s32>& B = s1.getB();
  const Vect<2, s32>& O = s2.getA();
  const Vect<2, s32>& P = s2.getB();
  
  // We test collision between [AB] and (OP)
  const Vect<2, s32> AO(O.coord(0) - A.coord(0), O.coord(1) - A.coord(1));
  const Vect<2, s32> AP(P.coord(0) - A.coord(0), P.coord(1) - A.coord(1));
  const Vect<2, s32> AB(B.coord(0) - A.coord(0), B.coord(1) - A.coord(1));
  if((AB.coord(0) * AP.coord(1) - AB.coord(1) * AP.coord(0)) * (AB.coord(0) * AO.coord(1) - AB.coord(1) * AO.coord(0)) >= 0) {
    return false;
  }
  
  // We test collision between [OP] and (AB)
  const Vect<2, s32> OA(A.coord(0) - O.coord(0), A.coord(1) - O.coord(1));
  const Vect<2, s32> OB(B.coord(0) - O.coord(0), B.coord(1) - O.coord(1));
  const Vect<2, s32> OP(P.coord(0) - O.coord(0), P.coord(1) - O.coord(1));
  if((OP.coord(0) * OB.coord(1) - OP.coord(1) * OB.coord(0)) * (OP.coord(0) * OA.coord(1) - OP.coord(1) * OA.coord(0)) >= 0) {
    return false;
  }
  
  return true;
}

bool CollisionDetector::collide(const Segment& s, const Circle& c) {
  if(collide(Point(s.getA()), c) || collide(Point(s.getB()), c)) { // One of the extremums of the segment is in the circle.
    return true;
  }
  
  const Vect<2, s32>& A = s.getA();
  const Vect<2, s32>& B = s.getB();
  const Vect<2, s32>& C = c.getCentre();
  const s32& r = c.getRadius();
  
  // Does the line defined with s at least collides with c.
  const Vect<2, s32> u(B.coord(0) - A.coord(0), B.coord(1) - A.coord(1));
  const Vect<2, s32> AC(C.coord(0) - A.coord(0), C.coord(1) - A.coord(1));
  float numerator = u.coord(0) * AC.coord(1) - u.coord(1) * AC.coord(0);
  numerator *= numerator; // squared to avoid using sqrt on the divisor.
  float divisor = u.coord(0) * u.coord(0) + u.coord(1) * u.coord(1); // square norm of u.
  float CI = numerator / divisor;
  if(CI >= static_cast<float>(r * r)) { // The line does not collide with the circle.
    return false;
  }
  
  // So we know the line collides with c, but does s collide too ?
  const Vect<2, s32> AB(B.coord(0) - A.coord(0), B.coord(1) - A.coord(1));
  // AC has already been computed.
  const Vect<2, s32> BC(C.coord(0) - B.coord(0), C.coord(1) - B.coord(1));
  float scal1 = AB.coord(0) * AC.coord(0) + AB.coord(1) * AC.coord(1);
  float scal2 = (-AB.coord(0)) * BC.coord(0) - AB.coord(1) * BC.coord(1);
  if(scal1 >= 0.f && scal2 >= 0.f)
    return true;
  
  return false;
}

//! \todo Implement!
bool CollisionDetector::collide(const Segment&s, const AABB& a) {
  (void) s;
  (void) a;
  return false;
}

//! \todo Implement!
bool CollisionDetector::collide(const Segment&s, const Triangle& t) {
  (void) s;
  (void) t;
  return false;
}

//! \todo Implement!
bool CollisionDetector::collide(const Segment&s, const Quadrilateral& q) {
  (void) s;
  (void) q;
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
  else if((ptr = dynamic_cast<const AABB*>(&s))) {
    return collide(c, *static_cast<const AABB*>(ptr));
  }
  else if((ptr = dynamic_cast<const Triangle*>(&s))) {
    return collide(c, *static_cast<const Triangle*>(ptr));
  }
  else if((ptr = dynamic_cast<const Quadrilateral*>(&s))) {
    return collide(c, *static_cast<const Quadrilateral*>(ptr));
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
    return true;
  }
}

//! \todo Implement!
bool CollisionDetector::collide(const Circle& c, const AABB& a) {
  (void) c;
  (void) a;
  return false;
}

//! \todo Implement!
bool CollisionDetector::collide(const Circle& c, const Triangle& t) {
  (void) c;
  (void) t;
  return false;
}

//! \todo Implement!
bool CollisionDetector::collide(const Circle& c, const Quadrilateral& q) {
  (void) c;
  (void) q;
  return false;
}

bool CollisionDetector::collide(const AABB& a, const Shape& s) {
  const void* ptr;
  if((ptr = dynamic_cast<const Point*>(&s))) {
    return collide(a, *static_cast<const Point*>(ptr));
  }
  else if((ptr = dynamic_cast<const Segment*>(&s))) {
    return collide(a, *static_cast<const Segment*>(ptr));
  }
  else if((ptr = dynamic_cast<const Circle*>(&s))) {
    return collide(a, *static_cast<const Circle*>(ptr));
  }
  else if((ptr = dynamic_cast<const AABB*>(&s))) {
    return collide(a, *static_cast<const AABB*>(ptr));
  }
  else if((ptr = dynamic_cast<const Triangle*>(&s))) {
    return collide(a, *static_cast<const Triangle*>(ptr));
  }
  else if((ptr = dynamic_cast<const Quadrilateral*>(&s))) {
    return collide(a, *static_cast<const Quadrilateral*>(ptr));
  }
  return false;
}

//! \todo Implement!
bool CollisionDetector::collide(const AABB& a1, const AABB& a2) {
  (void) a1;
  (void) a2;
  return false;
}

//! \todo Implement!
bool CollisionDetector::collide(const AABB& a, const Triangle& t) {
  (void) a;
  (void) t;
  return false;
}

//! \todo Implement!
bool CollisionDetector::collide(const AABB& a, const Quadrilateral& q) {
  (void) a;
  (void) q;
  return false;
}

bool CollisionDetector::collide(const Triangle& t, const Shape& s) {
  const void* ptr;
  if((ptr = dynamic_cast<const Point*>(&s))) {
    return collide(t, *static_cast<const Point*>(ptr));
  }
  else if((ptr = dynamic_cast<const Segment*>(&s))) {
    return collide(t, *static_cast<const Segment*>(ptr));
  }
  else if((ptr = dynamic_cast<const Circle*>(&s))) {
    return collide(t, *static_cast<const Circle*>(ptr));
  }
  else if((ptr = dynamic_cast<const AABB*>(&s))) {
    return collide(t, *static_cast<const AABB*>(ptr));
  }
  else if((ptr = dynamic_cast<const Triangle*>(&s))) {
    return collide(t, *static_cast<const Triangle*>(ptr));
  }
  else if((ptr = dynamic_cast<const Quadrilateral*>(&s))) {
    return collide(t, *static_cast<const Quadrilateral*>(ptr));
  }
  return false;
}

//! \todo Implement!
bool CollisionDetector::collide(const Triangle& t1, const Triangle& t2) {
  (void) t1;
  (void) t2;
  return false;
}

//! \todo Implement!
bool CollisionDetector::collide(const Triangle& t, const Quadrilateral& q) {
  (void) t;
  (void) q;
  return false;
}

bool CollisionDetector::collide(const Quadrilateral& q, const Shape& s) {
  const void* ptr;
  if((ptr = dynamic_cast<const Point*>(&s))) {
    return collide(q, *static_cast<const Point*>(ptr));
  }
  else if((ptr = dynamic_cast<const Segment*>(&s))) {
    return collide(q, *static_cast<const Segment*>(ptr));
  }
  else if((ptr = dynamic_cast<const Circle*>(&s))) {
    return collide(q, *static_cast<const Circle*>(ptr));
  }
  else if((ptr = dynamic_cast<const AABB*>(&s))) {
    return collide(q, *static_cast<const AABB*>(ptr));
  }
  else if((ptr = dynamic_cast<const Triangle*>(&s))) {
    return collide(q, *static_cast<const Triangle*>(ptr));
  }
  else if((ptr = dynamic_cast<const Quadrilateral*>(&s))) {
    return collide(q, *static_cast<const Quadrilateral*>(ptr));
  }
  return false;
}

//! \todo Implement!
bool CollisionDetector::collide(const Quadrilateral& q1, const Quadrilateral& q2) {
  (void) q1;
  (void) q2;
  return false;
}
