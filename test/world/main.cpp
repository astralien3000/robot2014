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
  
  // Point/Point collisions
  Point p1(3, 2), p2(3, 2), p3(5, 2);
  println("Point/Point");
  assert(CollisionDetector::collide(p1, p2));
  assert(!CollisionDetector::collide(p1, p3));
  println("OK");
  
  // Point/Circle collisions
  Point p4(0, 2);
  Circle c1(1, 1, 3);
  println("Point/Circle");
  assert(CollisionDetector::collide(c1, p4));
  assert(!CollisionDetector::collide(c1, p3));
  println("OK");
  
  // Segment/Circle collisions
  Segment s1(0, 6, 4, 0);
  Segment s2(-3, 5, 4, 5);
  println("Segment/Circle");
  assert(CollisionDetector::collide(c1, s1));
  assert(!CollisionDetector::collide(c1, s2));
  println("OK");
  
  // Circle/Circle collisions
  Circle c2(7, 4, 7), c3(-4, 2, 2);
  println("Circle/Circle");
  assert(CollisionDetector::collide(c1, c2));
  assert(!CollisionDetector::collide(c1, c3));
  println("OK");
  
  // Segment/Segment collisions
  Segment s3(-3, 6, -4, 7);
  println("Segment/Segment");
  assert(CollisionDetector::collide(s1, s2));
  assert(!CollisionDetector::collide(s2, s3));
  println("OK");
  
  return EXIT_SUCCESS;
}
