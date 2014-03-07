#ifndef WORLD_HPP
#define WORLD_HPP

#include <base/integer.hpp>
#include <math/vect.hpp>
#include <base/array.hpp>
#include <container/list.hpp>

class Shape {
public:
  virtual bool isCollided(const Shape&) const = 0;
};

class Circle : public Shape;
class Rectangle : public Shape;

class Circle : public Shape {
  Vect<2, s32> _centre;
  s32 _radius;
public:
  inline Circle(void)
    : _centre(0, 0), _radius(0) {
  }
  
  inline Circle(const Vect<2, s32>& centre, s32 radius)
    : _centre(centre), _radius(radius) {
  }
  
  inline Circle(s32 x, s32 y, s32 radius)
    : _centre(x, y), _radius(radius) {
  }
  
  inline Circle(const Circle& other)
    : _centre(other._centre), _radius(other._radius) {
  }
  
  inline Circle& operator=(const Circle& other) {
    _centre = other._centre;
    _radius = other._radius;
    return (*this);
  }
  
  inline bool operator==(const Circle& other) const {
    return _centre == other._centre && _radius = other._radius;
  }
  
  inline const Vect<2, s32>& getCentre(void) const {
    return _centre;
  }
  
  inline s32 getRadius(void) const {
    return _radius;
  }
  
  virtual bool isCollided(const Shape&) const;
  bool isCollided(const Circle&) const;
  bool isCollided(const Rectangle&) const;
};

class Rectangle : public Shape {
  Vect<2, s32> _centre;
  Vect<2, s32> _topleft;
  Vect<2, s32> _topright;
  Vect<2, s32> _botleft;
  Vect<2, s32> _botright;
private:
  inline Rectangle(const Vect<2, s32>& centre, const Vect<2, s32>& a, const Vect<2, s32>& b);
  inline Rectangle(const Vect<2, s32>& centre, s32 h, s32 w, s32 alpha);
  inline Rectangle(const Rectangle& other)
    : _centre(other._centre), _topleft(other._topleft), _topright(other._topright)
      _botleft(other._botleft), _botright(other._botright) {
  }
  
  inline Rectangle& operator=(const Rectangle& other) {
    _centre = other._centre;
    _topleft = other._topleft;
    _topright = other._topright;
    _botleft = other._botleft;
    _botright = other._botright;
  }
  
  inline bool operator==(const Rectangle& other) const;
  virtual bool isCollided(const Shape&) const;
  bool isCollided(const Circle&) const;
  bool isCollided(const Rectangle&) const;
};

//! \class World world.hpp "world.hpp"
//! \brief Represents the robot's environnment
/*!
  
  Contains static and dynamic obstacles.
  
*/
template<array_t _SIZE>
class World {
  List<_SIZE, Circle> _circles;
  List<_SIZE, Rectangle> _rectangles;
  
public:
  World(void);
  
  bool addCircle(const Circle& circle) {
    _circles
  
  bool isObstacle(Vect<2, s32>&);
  
  bool isCollided(Shape&);
};

#endif//WORLD_HPP
