#include <iostream>
#include <cstdlib>
#include <cassert>

#include <world.hpp>
#include <collision.hpp>

inline void println(const char* str) {
  std::cout << str << std::endl;
}

int main(int argc, char** argv) {
  (void) argc;
  (void) argv;
  
  Point p1(3, 2);
  Point p2(3, 2);
  Point p3(5, 2);
  Point p4(0, 2);
  Point p5(1, 5);
  Point p6(8, 5);
  Point p7(359,248);
  Point p8(-3, 6);
  Point p9(7, 5);
  
  Segment s1(0, 6, 4, 0);
  Segment s2(-3, 5, 4, 5);
  Segment s3(-3, 6, -4, 7);
  Segment s4(7, 8, 7, 3);
  Segment s5(7, 8, 12, 3);
  Segment s6(-3, 5, -6, 5);
  Segment s7(-4, 5, -2, 5);
  Segment s8(10, 6, 12, 8);
  
  Circle c1(1, 1, 3);
  Circle c2(7, 4, 7);
  Circle c3(-4, 2, 2);
  Circle c4(-3, 1, 1);
  
  AABB a1(7, 4, 5, 3);
  
  // Point/Point collisions
  println("Point/Point");
  assert(CollisionDetector::collide(p1, p2));
  assert(!CollisionDetector::collide(p1, p3));
  println("OK");
  
  // Point/Segment collisions
  println("Point/Segment");
  assert(CollisionDetector::collide(s2, p5));
  assert(!CollisionDetector::collide(s2, p6));
  assert(!CollisionDetector::collide(s2, p7));
  assert(!CollisionDetector::collide(s2, p8));
  println("OK");
  
  // Point/Circle collisions
  println("Point/Circle");
  assert(CollisionDetector::collide(c1, p4));
  assert(!CollisionDetector::collide(c1, p3));
  println("OK");
  
  // Point/AABB collisions
  println("Point/AABB");
  assert(CollisionDetector::collide(a1, p6));
  assert(!CollisionDetector::collide(a1, p7));
  assert(!CollisionDetector::collide(a1, p9));
  println("OK");
  
  // Segment/Segment collisions
  println("Segment/Segment");
  assert(CollisionDetector::collide(s1, s2));
  assert(!CollisionDetector::collide(s2, s3));
  assert(!CollisionDetector::collide(s2, s6));
  assert(!CollisionDetector::collide(s6, s7));
  println("OK");
  
  // Segment/Circle collisions
  println("Segment/Circle");
  assert(CollisionDetector::collide(c1, s1));
  assert(!CollisionDetector::collide(c1, s2));
  println("OK");
  
  // Segment/AABB collisions
  println("Segment/AABB");
  assert(CollisionDetector::collide(s5, a1));
  assert(CollisionDetector::collide(s8, a1));
  assert(!CollisionDetector::collide(s1, a1));
  assert(!CollisionDetector::collide(s2, a1));
  assert(!CollisionDetector::collide(s3, a1));
  assert(!CollisionDetector::collide(s4, a1));
  println("OK");
  
  // Circle/Circle collisions
  println("Circle/Circle");
  assert(CollisionDetector::collide(c1, c2));
  assert(!CollisionDetector::collide(c1, c3));
  assert(!CollisionDetector::collide(c1, c4));
  println("OK");
  
  return EXIT_SUCCESS;
}
