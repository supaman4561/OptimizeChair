#include <ode/ode.h>
#include "object.hpp"
#include "human.hpp"

Human::Human(dWorldID world, Object& head, Box& torso, Box& rthigh,
             Box& lthigh, Box& rleg,  Box& lleg, dReal recline_angle, dReal nee_angle)
  : head(&head), torso(&torso), rthigh(&rthigh), lthigh(&lthigh), rleg(&rleg), lleg(&lleg)
{
  neck = dJointCreateBall(world, 0);
  rbase = dJointCreateHinge(world, 0);
  lbase = dJointCreateHinge(world, 0);
  rnee = dJointCreateHinge(world, 0);
  lnee = dJointCreateHinge(world, 0);
  dJointAttach(neck, this->head->getBodyId(), torso.getBodyId());
  dJointAttach(rbase, torso.getBodyId(), rthigh.getBodyId());
  dJointSetHingeAnchor(rbase, 0, 0, 0);
  dJointSetHingeAxis(rbase, 1, 0, 0);
  dJointAttach(lbase, torso.getBodyId(), lthigh.getBodyId());
  dJointSetHingeAnchor(lbase, 0, 0, 0);
  dJointSetHingeAxis(lbase, 1, 0, 0);
  dJointAttach(rnee, rthigh.getBodyId(), rleg.getBodyId());
  dJointSetHingeAnchor(rnee, 0, 0, 0);
  dJointSetHingeAxis(rnee, 1, 0, 0);
  dJointAttach(lnee, lthigh.getBodyId(), lleg.getBodyId());
  dJointSetHingeAnchor(lnee, 0, 0, 0);
  dJointSetHingeAxis(lnee, 1, 0, 0);
}

void Human::draw() const {
  this->head->draw();
  this->torso->draw();
  this->rthigh->draw();
  this->lthigh->draw();
  this->rleg->draw();
  this->lleg->draw();
}