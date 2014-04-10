#include "astar.hpp"
#include <math/vect.hpp>

/*
  fichier a supprimer
  /!\ l'astar utilise pour l'instant ncurses pour les tests /!\
*/



int main(void) {
  Astar astar(11);
  Vect<2, s32> source, target;
  source[0] = -132;
  source[1] = -82;
  target[0] = 132;
  target[1] = 83;
  astar.getTrajectory(source, target);
  //astar.getTrajectory(source, target);
  return 0;
}
