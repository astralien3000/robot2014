#ifndef WORLD_HPP
#define WORLD_HPP

#include <base/integer.hpp>
#include <math/vect.hpp>
#include <container/list.hpp>
#include "shape.hpp"

//! \class World world.hpp "world.hpp"
//! \brief Represents a 2D environnment.
//! \param _SIZE : the amount of objects the world can handle.
template<list_t _SIZE>
class World {
public:
  static const list_t SIZE = _SIZE;
  
  List<SIZE, Shape*> _shapes;
  
public:
  inline World(void)
    : _shapes() {
  }
  
  inline bool addShape(const Shape* s) {
    if(_shapes.contains(s)) {
      return true;
    }
    return _shapes.append(s);
  }
  
  inline bool removeShape(const Shape* s) {
    return _shapes.remove(s);
  }
  
  inline list_t usedSpace() const {
    return _shapes.usedSpace();
  }
  
  inline bool collides(Vect<2, s32>& p) const {
    for(list_t  i = 0; i < _shapes.size(); i++) {
      if(_shapes.get(i)->isCollided(p)) {
	return true;
      }
    }
    return false;
  }
  
  inline bool collides(const Shape& s) const {
    for(list_t  i = 0; i < _shapes.size(); i++) {
      if(_shapes.get(i)->isCollided(s)) {
	return true;
      }
    }
    return false;
  }
};

#endif//WORLD_HPP
