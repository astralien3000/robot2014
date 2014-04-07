#include "astar.hpp"
#include "ncurses.h"

Astar::Astar(uint8_t mesh) {
  this->mesh = mesh;
}

Vect<2, s32>* Astar::getTrajectory(Vect<2, s32> &source, Vect<2, s32> &target) {
  this->beginX = (source[0] + 132) / mesh;
  this->beginY = (source[1] + 82) / mesh;
  this->targetX = (target[0] + 132) / mesh;
  this->targetY = (target[1] + 82) / mesh;
  nodes[0].setXY(beginX, beginY);
  nodes[0].setCost(0);
  nodes[0].setPred(0);
  nodes[0].setOpen();
  this->nbNode = 1;

  std::cout << (int) beginX << " " << (int) beginY << " -> " << (int) targetX << " " << (int) targetY << std::endl;
  return loop();
}

Vect<2, s32>* Astar::loop(void) {
  uint8_t val;
  uint8_t minVal;
  uint8_t minNode;
  while (this->keepGoing()) {
    minVal = 255;
    for (uint8_t i=0; i<this->nbNode; i++) {
      //std::cout << (int)nodes[i].x() << " " << (int)nodes[i].y() << " cost: " << nodes[i].cost() + distance(&nodes[i]) << std::endl;
      if (!(nodes[i].isClosed()) && (val = nodes[i].cost() + distance(&nodes[i])) <= minVal) {
	minVal = val;
	minNode = i;
      }
    }
    //std::cout << "min: " << (int)nodes[minNode].x() << " " << (int)nodes[minNode].y() << " : " << (int)minVal << std::endl;

    this->debug(&nodes[0], &nodes[minNode]);

    // if (nodes[minNode].x() == targetX && nodes[minNode].y() == targetY)
    //   return makePath(&nodes[0], &nodes[minNode]);

    nodes[minNode].setClosed();
    if (updateNeighbors(minNode))
      return makePath(&nodes[0], &nodes[nbNode-1]);
  }
  return NULL;
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
	nnode->setOpen();
	if (x == targetX && y == targetY)
	  return true;
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

bool Astar::keepGoing(void) {
  return (this->nbNode < Node::MAX_NB);
}

Vect<2, s32>* Astar::makePath(Node *source, Node *target) {
  pathLengh = 0;
  Node *cursor = target;
  Node *previous = cursor;
  int8_t curDx = -2;
  int8_t curDy = -2;

  this->debug(source, target);

  while (cursor != source) {    
    //std::cout << "{ " << (int)cursor->x() << " , " << (int)cursor->y() << " }" << std::endl;
    if (cursor->x() - previous->x() != curDx ||
	cursor->y() - previous->y() != curDy) {
      path[pathLengh][0] = cursor->x();
      path[pathLengh][1] = cursor->y();
      pathLengh++;
      std::cout << "{ " << (int)cursor->x() << " , " << (int)cursor->y() << " }" << std::endl;
      //std::cout << (int)curDx << ", " << (int)curDy << std::endl;
    }
    curDx = cursor->x() - previous->x();
    curDy = cursor->y() - previous->y();
    previous = cursor;
    cursor = &nodes[cursor->pred()];
  }

  for (uint8_t i=0; i<pathLengh; i++) {

  }

  std::cout << (int)nbNode << " nodes used" << std::endl;
  return path;
}

/*
uint8_t Astar::distance(Node *source, Node *target) {
  return distance(source->x(), source->(y), target->x(), target->y());
}
*/

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
  if (x>24 || y>15)
    return true;
  static uint8_t obstaclesX[13] = {4, 5, 6, 6, 6, 6, 11, 11, 11, 11, 11, 12, 13};
  static uint8_t obstaclesY[13] = {6, 6, 6, 5, 4, 3, 3, 4, 5, 6, 7, 3, 3};
  for (uint8_t i=0; i<13; i++) {
    if (obstaclesX[i] == x && obstaclesY[i] == y)
      return true;
  }
  return false;
}

void Astar::debug(Node *source, Node *target) {
  Node *cursor = target;

  initscr();
  start_color();
  init_pair(1, COLOR_RED,     COLOR_BLACK);
  init_pair(2, COLOR_GREEN,   COLOR_BLACK);
  init_pair(3, COLOR_WHITE,   COLOR_BLACK);
  init_pair(4, COLOR_YELLOW,     COLOR_BLACK);

  for (uint8_t x=0; x<26; x++) {
    for (uint8_t y=0; y<17; y++) {
      if (isObstacle(x, y)) {
	  attron(COLOR_PAIR(4));
	  mvprintw(1*x, 2*y, "o");
	  mvprintw(1*x, 2*y + 50, "o");
      }
    }
  }

  for (uint8_t i=0; i<nbNode; i++) {
    if (isObstacle(nodes[i].x(), nodes[i].y()))
      attron(COLOR_PAIR(1));
    else if (nodes[i].isClosed())
      attron(COLOR_PAIR(3));
    else
      attron(COLOR_PAIR(2));
    mvprintw(1*nodes[i].x(), 2*nodes[i].y(), "%d", nodes[i].cost() + distance(&nodes[i]));
    mvprintw(1*nodes[i].x(), 2*nodes[i].y() + 50, "%d", nodes[i].cost());
  }

  while (cursor != source) {
    attron(COLOR_PAIR(1));
    mvprintw(1*cursor->x(), 2*cursor->y(), "%d", cursor->cost() + distance(cursor));
    mvprintw(1*cursor->x(), 2*cursor->y() + 50, "%d", cursor->cost());
    
    cursor = &nodes[cursor->pred()];
  }

  refresh();
  getch();
  endwin();
}
