#include "node.hpp"
#include <base/integer.hpp>
#include <math/saturate.hpp>
#include <math/vect.hpp>
#include <geometry/world.hpp>
#include <geometry/aabb.hpp>
#include "my_world.hpp"

class Astar {
private:
  World<WORLD_SIZE, AABB> world;
  Node nodes[Node::MAX_NB];
  Vect<2, s32> path[8];
  uint8_t pathLengh;
  uint8_t beginX;
  uint8_t beginY;
  uint8_t targetX;
  uint8_t targetY;
  uint8_t targetX_real;
  uint8_t targetY_real;
  uint8_t mesh;
  uint8_t nbNode;
  Vect<2, s32>* loop(void);
  bool updateNeighbors(uint8_t node);
  bool keepGoing(uint8_t minNode);
  Vect<2, s32>* makePath(Node *source, Node *target);
  uint8_t distance(Node *node);
  uint8_t distance(uint8_t sx, uint8_t sy, uint8_t tx, uint8_t ty);
  bool newNode(Node **node, uint8_t x, uint8_t y);
  bool isObstacle(uint8_t x, uint8_t y);
  uint8_t convertX_real2simple(s32 x);
  uint8_t convertY_real2simple(s32 y);
  s32 convertX_simple2real(uint8_t x);
  s32 convertY_simple2real(uint8_t y);
public:
  Astar(uint8_t mesh);
  Vect<2, s32>* getTrajectory(Vect<2, s32> &source, Vect<2, s32> &target);
  uint8_t getPathLengh(void);
};
