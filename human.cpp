#include <ode/ode.h>
#include "object.hpp"
#include "human.hpp"

Human::Human(dWorldID world, dSpaceID space, dReal x, dReal y, dReal z, dReal rec_angle, dReal nee_angle)
{
  head = new Sphere(world, space, 0.12, 0+x, 0+y, 2.22+z, 0.48);
  torso = new Box(world, space, 0.4, 0.2, 0.6, 0+x, 0+y, 1.8+z, 2.8);
  rthigh = new Box(world, space, 0.15, 0.2, 0.4, -0.125+x, 0+y, 1.3+z, 0.84);
  lthigh = new Box(world, space, 0.15, 0.2, 0.4, 0.125+x, 0+y, 1.3+z, 0.84);
  rleg = new Box(world, space, 0.15, 0.2, 0.4, -0.125+x, 0+y, 0.9+z, 0.72);
  lleg = new Box(world, space, 0.15, 0.2, 0.4, 0.125+x, 0+y, 0.9+z, 0.72);

  neck = dJointCreateFixed(world, 0);
  dJointAttach(neck, head->getBodyId(), torso->getBodyId());
  dJointSetFixed(neck);

  rback = dJointCreateHinge(world, 0);
  dJointAttach(rback, torso->getBodyId(), rthigh->getBodyId());
  dJointSetHingeAnchor(rback, -0.125, -0.0, 1.5);
  dJointSetHingeAxis(rback, 1, 0, 0);

  lback = dJointCreateHinge(world, 0);
  dJointAttach(lback, torso->getBodyId(), lthigh->getBodyId());
  dJointSetHingeAnchor(lback, 0.125, -0.0, 1.5);
  dJointSetHingeAxis(lback, 1, 0, 0);
  
  rnee = dJointCreateHinge(world, 0);
  dJointAttach(rnee, rthigh->getBodyId(), rleg->getBodyId());
  dJointSetHingeAnchor(rnee, -0.125, 0.1, 1.1);
  dJointSetHingeAxis(rnee, 1, 0, 0);

  lnee = dJointCreateHinge(world, 0);
  dJointAttach(lnee, lthigh->getBodyId(), lleg->getBodyId());
  dJointSetHingeAnchor(lnee, 0.125, 0.1, 1.1);
  dJointSetHingeAxis(lnee, 1, 0, 0);
  

  setHingeJointAngle(rback, M_PI * rec_angle / 180);  
  dJointSetHingeParam(rback, dParamFudgeFactor, 0);

  setHingeJointAngle(lback, M_PI * rec_angle / 180);  
  dJointSetHingeParam(lback, dParamFudgeFactor, 0);

  setHingeJointAngle(rnee, -M_PI * nee_angle / 180);  
  dJointSetHingeParam(rnee, dParamFudgeFactor, 0);

  setHingeJointAngle(lnee, -M_PI * nee_angle / 180);    
  dJointSetHingeParam(lnee, dParamFudgeFactor, 0);
}

void Human::jointAttachToTorso(dJointID joint, dBodyID body)
{
  dJointAttach(joint, body, torso->getBodyId());
}

void Human::jointAttachToRthigh(dJointID joint, dBodyID body)
{
  dJointAttach(joint, body, rthigh->getBodyId());
}

void Human::jointAttachToLthigh(dJointID joint, dBodyID body)
{
  dJointAttach(joint, body, lthigh->getBodyId());
}

int Human::thighIsOn(Box box)
{
  dVector3 this_pos;
  dVector3 box_pos, box_length;
  dBodyCopyPosition(rthigh->getBodyId(), this_pos);
  dGeomCopyPosition(box.getGeomId(), box_pos);
  dGeomBoxGetLengths(box.getGeomId(), box_length);

  int inx = abs(this_pos[0] - box_pos[0]) <= (box_length[0] / 2);
  int iny = abs(this_pos[1] - box_pos[1]) <= (box_length[1] / 2);
  int onz = (this_pos[2] >= box_pos[2]);
  return inx * iny * onz;
}

void Human::draw() const {
  head->draw();
  torso->draw();
  rthigh->draw();
  lthigh->draw();
  rleg->draw();
  lleg->draw();
}

void Human::rotation(dReal angle)
{
  dMatrix3 R;
  dRFromAxisAndAngle(R, -1, 0, 0, M_PI * angle / 180);
  dGeomSetRotation(this->rthigh->getGeomId(), R);
  dGeomSetRotation(this->lthigh->getGeomId(), R);
}

void Human::destroy()
{
  head->destroy();
  torso->destroy();
  rthigh->destroy();
  lthigh->destroy();
  rleg->destroy();
  lleg->destroy();
}

void Human::move(dReal x, dReal y, dReal z)
{
  dBodySetPosition(this->torso->getBodyId(), x, y, z);
}