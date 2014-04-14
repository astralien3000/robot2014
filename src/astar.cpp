#include "astar.hpp"
#include <geometry/world.hpp>
#include <geometry/point.hpp>
#include <geometry/segment.hpp>
#include <geometry/circle.hpp>
#include <geometry/aabb.hpp>
#include "my_world.hpp"

Astar::Astar(uint8_t mesh, World<WORLD_SIZE, AABB>& w)
  : world(w) {
  this->mesh = mesh;
  this->mesh = 100;
  fill_world(this->world);
}

Vect<2, s32>* Astar::getTrajectory(Vect<2, s32> &&source, Vect<2, s32> &&target) {
  this->targetX_real = target[0];
  this->targetY_real = target[1];

  if (source[0] < -1250)
    source[0] = -1250;
  if (source[0] > 1250)
    source[0] = 1250;
  if (source[1] < -750)
    source[1] = -750;
  if (source[1] > 750)
    source[1] = 750;
    if (target[0] < -1250)
    target[0] = -1250;
  if (target[0] > 1250)
    target[0] = 1250;
  if (target[1] < -750)
    target[1] = -750;
  if (target[1] > 750)
    target[1] = 750;

  this->pathLengh = 0;
  this->beginX = convertX_real2simple(source[0]);
  this->beginY = convertY_real2simple(source[1]);
  this->targetX = convertX_real2simple(target[0]);
  this->targetY = convertY_real2simple(target[1]);
  if (isObstacle(targetX, targetY))
    return 0;

  nodes[0].init();
  nodes[0].setXY(beginX, beginY);
  nodes[0].setCost(0);
  nodes[0].setPred(0);
  this->nbNode = 1;

  return loop();
}

uint8_t Astar::getPathLengh(void) {
  return this->pathLengh;
}

Vect<2, s32>* Astar::loop(void) {
  uint8_t val;
  uint8_t minVal;
  uint8_t minNode = 0;
  while (this->keepGoing(minNode)) {
    minVal = 255;
    for (uint8_t i=0; i<this->nbNode; i++) {
      if (!(nodes[i].isClosed()) && (val = nodes[i].cost() + distance(&nodes[i])) < minVal) {
	minVal = val;
	minNode = i;
      }
    }

    if (minVal == 255) {
      for (uint8_t i=0; i<this->nbNode; i++) {
	if ((val = distance(&nodes[i])) <= minVal) {
	  minVal = val;
	  minNode = i;
	}
      }
      return makePath(&nodes[0], &nodes[minNode]);
    }

    nodes[minNode].setClosed();
    if (updateNeighbors(minNode))
      return makePath(&nodes[0], &nodes[nbNode-1]);
  }
  return makePath(&nodes[0], &nodes[minNode]);
}

bool Astar::updateNeighbors(uint8_t node) {
  uint8_t x, y, dist, possibleCost;
  Node *nnode;
  bool nodeIsNew;
  for (int8_t dx=-1; dx<2; dx++) {
    for (int8_t dy=-1; dy<2; dy++) {
      if (dx == 0 && dy == 0)
	continue;
      x = this->nodes[node].x() + dx;
      y = this->nodes[node].y() + dy;
      if (x > Node::MAX_X || y > Node::MAX_Y)
	continue;
      if (this->isObstacle(x, y))
	continue;
      nodeIsNew = newNode(&nnode, x, y);
      if (nnode->isClosed())
	continue;
      if (dx == 0 || dy == 0)
	dist = 2;
      else
	dist = 3;
      if (this->nodes[this->nodes[node].pred()].x() - this->nodes[node].x() ==  -dx &&
	  this->nodes[this->nodes[node].pred()].y() - this->nodes[node].y() ==  -dy)
        dist = dist - 0;
      possibleCost = this->nodes[node].cost() + dist;
      if (nodeIsNew) {
	nnode->setXY(x, y);
	nnode->setCost(possibleCost);
	nnode->setPred(node);
	if (x == targetX && y == targetY)
	  return true; //means "ok, you can stop the loop !"
      } else {
	if (possibleCost < nnode->cost()) {
	  nnode->setCost(possibleCost);
	  nnode->setPred(node);
	}
      }
    }
  }
  return false;
}

bool Astar::keepGoing(uint8_t minNode) {
  return (this->nbNode < Node::MAX_NB || this->nodes[nodes[minNode].pred()].cost() == Node::MAX_COST);
}

Vect<2, s32>* Astar::makePath(Node *source, Node *target) {
  pathLengh = 0;
  Node *cursor = target;
  Node *previous = cursor;
  int8_t curDx = 0;
  int8_t curDy = 0;


  while (cursor != source) {    
    if (cursor->x() - previous->x() != curDx ||
	cursor->y() - previous->y() != curDy) {
      if (previous->x() == targetX && previous->y() == targetY) {
	path[pathLengh][0] = targetX_real;
	path[pathLengh][1] = targetY_real;
      } else {
	path[pathLengh][0] = convertX_simple2real(previous->x());
	path[pathLengh][1] = convertY_simple2real(previous->y());
      }
      pathLengh++;
    }
    curDx = cursor->x() - previous->x();
    curDy = cursor->y() - previous->y();
    previous = cursor;
    cursor = &nodes[cursor->pred()];
  }

  return path;
}

uint8_t Astar::distance(Node *node) {
  return distance(node->x(), node->y(), targetX, targetY);
}

uint8_t Astar::distance(uint8_t sx, uint8_t sy, uint8_t tx, uint8_t ty) {
  uint8_t dx = Math::abs(sx - tx);
  uint8_t dy = Math::abs(sy - ty);
  return dx + dy + Math::max(dx, dy);
}

bool Astar::newNode(Node **node, uint8_t x, uint8_t y) {
  for (uint8_t i=0; i<this->nbNode; i++) {
    if (this->nodes[i].x() == x && this->nodes[i].y() == y) {
      *node = &nodes[i];
      return false;
    }
  }
  *node = &nodes[this->nbNode++];
  (*node)->init();
  return true;
}

bool Astar::isObstacle(uint8_t x, uint8_t y) {
  if (x>25 || y>15)
    return true;
  Circle robot(convertX_simple2real(x), convertY_simple2real(y), 250);
  return this->world.collide(robot);
}

uint8_t Astar::convertX_real2simple(s32 x) {
  return (x+1300) / mesh;
}

uint8_t Astar::convertY_real2simple(s32 y) {
  return (y+800) / mesh;
}

s32 Astar::convertX_simple2real(uint8_t x) {
  return x*mesh - 1250;
}

s32 Astar::convertY_simple2real(uint8_t y) {
  return y*mesh - 750;
}

