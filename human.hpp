#ifndef __HUMAN_H_
#define __HUMAN_H_
#include <ode/ode.h>
#include "object.hpp"

class Human
{
  Sphere *head;
  Box *torso, *rthigh, *lthigh, *rleg, *lleg;
  dJointID neck, rback, lback, rnee, lnee;
public:
  Human(dWorldID world, dSpaceID space, dReal x, dReal y, dReal z, dReal rec_angle, dReal nee_angle);
  void jointAttachToTorso(dJointID joint, dBodyID body);
  void jointAttachToRthigh(dJointID joint, dBodyID body);
  void jointAttachToLthigh(dJointID joint, dBodyID body);
  void draw() const;
  void rotation(dReal angle);
  void destroy();
  void move(dReal x, dReal y, dReal z);
  int thighIsOn(Box box);

};

#endif // __HUMAN_H_