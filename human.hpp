#ifndef __HUMAN_H_
#define __HUMAN_H_
#include <ode/ode.h>
#include "object.hpp"

class Human
{
  Object *head, *torso, *rthigh, *lthigh, *rleg, *lleg;
  dJointID neck, rbase, lbase, rnee, lnee;
  dReal recline_angle;
  dReal nee_angle;
public:
  Human(dWorldID world, Object& head, Box& torso, Box& rthigh,
        Box& lthigh, Box& rleg,  Box& lleg, dReal recline_angle, dReal nee_angle);
  void draw() const;
};

#endif // __HUMAN_H_