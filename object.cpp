#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include "object.hpp"

Object::Object(dWorldID world, dReal x, dReal y, dReal z, dReal mass)
              : x(x), y(y), z(z), mass(mass)
{
  this->body = dBodyCreate(world);
  dBodySetPosition(this->body, x, y, z);
}

dBodyID Object::getBodyId()
{
  return this->body;
}

dGeomID Object::getGeomId()
{
  return this->geom;
}

Box::Box(dWorldID world, dReal lx, dReal ly, dReal lz, dReal x, dReal y, dReal z, dReal mass)
        : lx(lx), ly(ly), lz(lz), Object::Object(world, x, y, z, mass) {}

void Box::setGeom(dSpaceID space)
{
  dMass m;
  dMassSetZero(&m);
  dMassSetBoxTotal(&m, this->mass, this->lx, this->ly, this->lz);
  dBodySetMass(this->body, &m);
  this->geom = dCreateBox(space, this->lx, this->ly, this->lz);
  dGeomSetBody(this->geom, this->body);
}

void Box::draw() const
{
  dReal shape[] = {this->lx, this->ly, this->lz};
  dsDrawBox(dBodyGetPosition(this->body), dBodyGetRotation(this->body), shape);
}