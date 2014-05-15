#include "hunt_action.hpp"
#include "devices.hpp"
#include "device/other/pin.hpp"

static s16 ANTI_BOUNCE_LIMIT = 100;

HuntAction::HuntAction(const Vect<2, s32>& pos, s32 number)
: _pos(pos), _number(number), _done(false) {
  Pin<36> sortie("PE4");//PE4, green
  sortie.setMode(PinMode::OUTPUT);
  sortie.setValue(false);
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
  Pin<36> sortie("PE4");//PE4, green
  sortie.setMode(PinMode::OUTPUT);
  Pin<38> entree("PE5");//PE5, yellow
  entree.setMode(PinMode::INPUT);
  /*Pin<38> entree2("PE6");
  entree.setMode(PinMode::INPUT);
  while(1) {
    io << entree.getValue() << " " << entree2.getValue() << "\n";
    }*/

  for (int i=0; i<_number; i++) {
    io << "start sending" << i<<"\n";
    sortie.setValue(true);
    s16 anti_bounce = 0;
    while(anti_bounce < ANTI_BOUNCE_LIMIT) {
      if(entree.getValue() == true) {
	//attendre que la carte lanceballe commence
	anti_bounce++;
      }
      else {
	anti_bounce = 0;
      }
    }
    sortie.setValue(false);
    
    anti_bounce = 0;
    while(anti_bounce < ANTI_BOUNCE_LIMIT) {
      if(entree.getValue() == false) {
	//attendre que la carte lanceballe finisse
	anti_bounce++;
      }
      else {
	anti_bounce = 0;
      }
    }
  }
  io << "finished\n";

  _done = true;
}
