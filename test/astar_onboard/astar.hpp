#include "node.hpp"
#include <base/integer.hpp>
#include <math/vect.hpp>
//#include <iostream>

//g++ -std=c++11 main_astar.cpp astar.cpp node.cpp -Iaversive--/include/common/ -lncurses -o astar

class Astar {
private:
  Node nodes[128];
  Vect<2, s32> path[32];
  uint8_t pathLengh;
  uint8_t beginX;
  uint8_t beginY;
  uint8_t targetX;
  uint8_t targetY;
  uint8_t mesh;
  uint8_t nbNode;
  Vect<2, s32>* loop(void);
  bool updateNeighbors(uint8_t node);
  bool keepGoing(void);
  Vect<2, s32>* makePath(Node *source, Node *target);
  //uint8_t distance(Node *source, Node *target);
  uint8_t distance(Node *node);
  //uint8_t distance(uint8_t x, uint8_t y);
  uint8_t distance(uint8_t sx, uint8_t sy, uint8_t tx, uint8_t ty);
  bool newNode(Node **node, uint8_t x, uint8_t y);
  bool isObstacle(uint8_t x, uint8_t y);
  //void debug(Node *source, Node *target);
public:
  Astar(uint8_t mesh);
  Vect<2, s32>* getTrajectory(Vect<2, s32> &source, Vect<2, s32> &target);
};
