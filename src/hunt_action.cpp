#include "hunt_action.hpp"
#include "devices.hpp"
#include "device/other/pin.hpp"

HuntAction::HuntAction(const Vect<2, s32>& pos, s32 number)
: _pos(pos), _number(number), _done(false) {
}

s16 HuntAction::priority(void) {
  if(_done) {
    return 0;
  }
  
  s16 dist = (controlPoint() - positionManager().getValue()).norm();
  if(dist != 0) {
    return 4000 / dist;
  }
  return 4000;
}

Vect<2, s32> HuntAction::controlPoint(void) {
  return _pos;
}

void HuntAction::doAction(void) {
  /*
  if((positionManager().getValue() - controlPoint()).norm() > 100) {
    io << "not at a good position\n";
    return;
  }
  */
  // Let Benoit do its job !
  Pin<44> sortie("PE4");//PE4, green
  sortie.setMode(PinMode::OUTPUT);
  Pin<45> entree("PE5");//PE5, yellow
  entree.setMode(PinMode::INPUT);

  for (int i=0; i<_number; i++) {
    sortie.setValue(true);
    while(entree.getValue() == false);//attendre que la carte lanceballe commence
    sortie.setValue(false);
    while(entree.getValue() == true);//attendre que la carte lanceballe commence
  }
  

  _done = true;
}
