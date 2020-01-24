#ifndef __HUMAN_H_
#define __HUMAN_H_
#include <ode/ode.h>
#include "object.h"

class Human
{
  Object head, torso, thigh, leg;
  dReal recline_angle;
  dReal nee_angle;
public:
  Human(Object head, Object torso, 
        Object thigh, Object leg, dReal recline_angle, dReal nee_angle);
};

#endif // __HUMAN_H_