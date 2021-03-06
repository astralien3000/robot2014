#include "hunt_action.hpp"
#include "devices.hpp"
#include "device/other/pin.hpp"

static s16 ANTI_BOUNCE_LIMIT = 100;
static u8 nb_balls = 7;

HuntAction::HuntAction(const Vect<2, s32>& pos, const Vect<2, s32>& mamouth,s32 number)
: _pos(pos), _mamouth(mamouth), _number(number) {
  Pin<36> sortie("PE4");//PE4, green
  sortie.setMode(PinMode::OUTPUT);
  sortie.setValue(false);
}

s16 HuntAction::priority(void) {
  if(_static_priority == 0) {
    return 0;
  }
  if(_static_priority < 10) _static_priority++;
  
  s16 dist = (controlPoint() - positionManager().getValue()).norm();
  if(dist != 0) {
    return _static_priority * (10000 / dist);
  }
  return 10000;
}

Vect<2, s32> HuntAction::controlPoint(void) {
  return _pos;
}

enum Error HuntAction::doAction(void) {
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


  trajectoryManager().lookAt(_mamouth);
  while(!trajectoryManager().isEnded()) {
    if(robot().getValue()) {
      return SKATING;
    }
  }
  
  Vect<2, s32> look = _mamouth;
  for (int i=0; i<_number && nb_balls > 0; i++, nb_balls--) {
    //io << "sending " << i << " soon\n";
    
    // trajectoryManager().gotoDistance(20);
    // while(!trajectoryManager().isEnded()) {}
    // Vect<2, s32> look = _mamouth;
    // look[0] -= OFFSET*_number/2;
    // //int look = 85-(OFFSET*_number/2)/10;
    // trajectoryManager().lookAt(look);
    // //trajectoryManager().gotoAngle(look);
    // while(!trajectoryManager().isEnded()) {
    //   if (robot().getValue()) {
    // 	trajectoryManager().gotoDistance(-50);
    // 	while(!trajectoryManager().isEnded()) {
    // 	  robot().unlock();
    // 	}
    //   }
    // }
    
    look.coord(0) += _mamouth.coord(0) + i * 12;
    trajectoryManager().lookAt(look);
    while(!trajectoryManager().isEnded()) {
      if(robot().getValue()) {
	return SKATING;
      }
    }
    
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
    look += OFFSET/10;
  }
  //io << "finished\n";
  trajectoryManager().gotoDistance(-150);
  while(trajectoryManager().isEnded()){
    if (robot().getValue()) {
      robot().unlock();
    }
    if (check_for_collision(60)) {
      return IMPOSSIBLE;
    }
  };
  _static_priority = 0;

  return SUCCESS;
}
