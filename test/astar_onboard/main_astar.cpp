#include "astar.hpp"
#include "aversive--/include/common/math/vect.hpp"

/*
  fichier a supprimer
  /!\ l'astar utilise pour l'instant ncurses pour les tests /!\
*/



int main(void) {
  Astar astar(11);
  Vect<2, s32> source, target;
  source[0] = -1200;
  source[1] = -700;
  target[0] = 1200;
  target[1] = 600;
  astar.getTrajectory(source, target);

  for (uint8_t i=astar.getPathLengh(); i > 0; i--) {
    io << path[i-1][0] << " " << path[i-1][1] << "\n";
  }

  return 0;
}
