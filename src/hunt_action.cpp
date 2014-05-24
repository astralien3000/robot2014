#include "hunt_action.hpp"
#include "devices.hpp"
#include "device/other/pin.hpp"

static s16 ANTI_BOUNCE_LIMIT = 100;

HuntAction::HuntAction(const Vect<2, s32>& pos, const Vect<2, s32>& mamouth,s32 number)
: _pos(pos), _mamouth(mamouth), _number(number) {
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
    return 10000 / dist;
  }
  return 10000;
}

Vect<2, s32> HuntAction::controlPoint(void) {
  return _pos;
}

void HuntAction::doAction(void) {
  io << "DO HUNT\n";
  
    // Let Benoit do its job !
  Pin<36> sortie("PE4");//PE4, green
  sortie.setMode(PinMode::OUTPUT);
  Pin<38> entree("PE6");//PE6, yellow
  entree.setMode(PinMode::INPUT);
  /*Pin<38> entree2("PE6");
  entree.setMode(PinMode::INPUT);
  while(1) {
    io << entree.getValue() << " " << entree2.getValue() << "\n";
    }*/

  Vect<2, s32> look = _mamouth;
  //look[0] -= OFFSET*_number/2;

  trajectoryManager().lookAt(look);
  while(!trajectoryManager().isEnded());

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
	look[0]+=OFFSET;
  }
  io << "finished\n";
  trajectoryManager().gotoDistance(-150);
  while(trajectoryManager().isEnded()){};
  _done = true;
}
