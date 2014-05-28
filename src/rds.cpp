#include "rds.hpp"

#include "devices.hpp"
#include <math/trigo.hpp>
#include "fpga.hpp"
#include <geometry/world.hpp>

#include "my_world.hpp"
#include "trajectory.hpp"

extern World<WORLD_SIZE, AABB> world;
extern World<2, Circle> mini_world;



FpgaUartStream rds_io("rds_stream", UART_TX_1_DATA, UART_TX_1_OCUP, UART_RX_1_DATA, UART_RX_1_AVA);

Rds rds("", rds_io);

void rds_init(void) {
  //rds_io.setMode(Stream::BINARY);
  world.addShape(&ennemy_robot);
  world.addShape(&ennemy_pmi);
  mini_world.addShape(&ennemy_robot);
  mini_world.addShape(&ennemy_pmi);
}

bool update_world(void) {
  rds.update();
  List<2, Vect<2, s32> > adv = rds.getValue();
  //io << "UPDATE_WORLD\n";
  //io << "i'm at : " << pos.getValue().coord(0) << " " << pos.getValue().coord(1) << "\n";
  //io << "detected : " << adv.usedSpace() << "\n";
  static const s32 DIST_TO_RM = 650;
  if((pos.getValue() - ennemy_robot.centre()).norm() > DIST_TO_RM) {
    ennemy_robot.centre() = Vect<2, s32>(0, 1500);
  }
  if((pos.getValue() - ennemy_pmi.centre()).norm() > DIST_TO_RM) {
    ennemy_pmi.centre() = Vect<2, s32>(0, 1500);
  }
  
  if(ennemy_pmi.centre().coord(1) != 1500 && ennemy_robot.centre().coord(1) == 1500) {
    ennemy_robot = ennemy_pmi;
    ennemy_pmi.centre() = Vect<2, s32>(0, 1500);
  }
  
  /*
  world.removeShape(&ennemy_robot);
  world.removeShape(&ennemy_pmi);
  mini_world.removeShape(&ennemy_robot);
  mini_world.removeShape(&ennemy_pmi);
  */
  
  bool detected = false;
  
  for (int i = 0; i< (int)adv.usedSpace(); i++) {
    //io << i << " : " << adv.get(i).coord(0) << " , " << adv.get(i).coord(1) << "\n";
    //s32 abs_angle = 180 - adv.get(i).coord(1) + (odo.getValue().coord(1) >> 4);
    s32 abs_angle = (- adv.get(i).coord(1) - (odo.getValue().coord(1) >> 4))%360;

    s32 rel_x = (s32)(adv.get(i).coord(0) * Math::cos<Math::DEGREE, double>(abs_angle));
    s32 rel_y = (s32)(adv.get(i).coord(0) * Math::sin<Math::DEGREE, double>(abs_angle));
    s32 abs_x = pos.getValue().coord(0) + rel_x * 10;
    s32 abs_y = pos.getValue().coord(1) + rel_y * 10;
    //io << "abs_angle = " << abs_angle << "\n";
    //io << "my_angle = " << -(odo.getValue().coord(1) >> 4) << "\n";
    //io << "rel_x = " << rel_x << " \trel_y = " << rel_y << "\n";
    //io << "abs_x = " << abs_x << " \tabs_y = " << abs_y << "\n";

    if (i==0) {
      ennemy_robot.centre() = Vect<2, s32>(abs_x, abs_y);
      //world.addShape(&ennemy_robot);
      //mini_world.addShape(&ennemy_robot);
    } else {
      //ennemy_pmi = Circle(abs_x, abs_y, 330);
      ennemy_pmi.centre() = Vect<2, s32>(abs_x, abs_y);
      //world.addShape(&ennemy_pmi);
      //mini_world.addShape(&ennemy_pmi);
    }

    detected = true;
  }

  return detected;
}

bool check_for_collision(u8 max_dist) {
  rds.update();
  List<2, Vect<2, s32> > adv = rds.getValue();
  //io << "CHECK_FOR_COLLISION\n";
  //io << "i'm at : " << pos.getValue().coord(0) << " " << pos.getValue().coord(1) << "\n";
  //io << "detected : " << adv.usedSpace() << "\n";
  
  for(int i = 0; i < (int) adv.usedSpace(); i++) {
    //io << i << " : " << adv.get(i).coord(0) << " , " << adv.get(i).coord(1) << "\n";
    
    static const s32 SHORT_RANGE_DIST = 60;
    static const s32 SHORT_RANGE_ANGLE = 45;
    s32 LONG_RANGE_DIST = max_dist;
    static const s32 LONG_RANGE_ANGLE = 30;
    
    if(!traj.isBackward() && adv.get(i).coord(0) < SHORT_RANGE_DIST &&
       (adv.get(i).coord(1) < SHORT_RANGE_ANGLE ||
	adv.get(i).coord(1) > (360 - SHORT_RANGE_ANGLE))) {
      return true;
    }
    if(!traj.isBackward() && adv.get(i).coord(0) < LONG_RANGE_DIST &&
       (adv.get(i).coord(1) < LONG_RANGE_ANGLE ||
	adv.get(i).coord(1) > (360 - LONG_RANGE_ANGLE))) {
      return true;
    }
    if(traj.isBackward() && adv.get(i).coord(0) < SHORT_RANGE_DIST &&
       (adv.get(i).coord(1) > (180 - SHORT_RANGE_ANGLE) &&
	adv.get(i).coord(1) < (180 + SHORT_RANGE_ANGLE))) {
      return true;
    }
    if(traj.isBackward() && adv.get(i).coord(0) < LONG_RANGE_DIST &&
       (adv.get(i).coord(1) > (180 - LONG_RANGE_ANGLE) &&
	adv.get(i).coord(1) < (180 + LONG_RANGE_ANGLE))) {
      return true;
    }
  }
  return false;
}
