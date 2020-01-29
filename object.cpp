#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include "object.hpp"

#ifdef dDOUBLE
#define dsDrawBox      dsDrawBoxD
#define dsDrawSphere   dsDrawSphereD
#define dsDrawCylinder dsDrawCylinderD
#define dsDrawCapsule  dsDrawCapsuleD
#endif

/*--------------------- Object class ----------------------*/

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

/*--------------------- Box class ----------------------*/

Box::Box(dWorldID world, dSpaceID space, dReal lx, dReal ly, dReal lz, dReal x, dReal y, dReal z, dReal mass)
  : lx(lx), ly(ly), lz(lz), Object::Object(world, x, y, z, mass) 
{
  dMass m;
  dMassSetZero(&m);
  dMassSetBoxTotal(&m, this->mass, this->lx, this->ly, this->lz);
  dBodySetMass(this->body, &m);
  this->geom = dCreateBox(space, this->lx, this->ly, this->lz);
  dGeomSetBody(this->geom, this->body);
}

dBodyID Box::getBodyId()
{
  return this->body;
}

dGeomID Box::getGeomId()
{
  return this->geom;
}

void Box::draw() const
{
  dReal shape[] = {this->lx, this->ly, this->lz};
  dsDrawBox(dBodyGetPosition(this->body), dBodyGetRotation(this->body), shape);
}

/*--------------------- Sphere class ----------------------*/

Sphere::Sphere(dWorldID world, dSpaceID space, dReal radius, dReal x, dReal y, dReal z, dReal mass)
  : radius(radius), Object::Object(world, x, y, z, mass) 
{
  dMass m;
  dMassSetZero(&m);
  dMassSetSphereTotal(&m, this->mass, this->radius);
  dBodySetMass(this->body, &m);
  this->geom = dCreateSphere(space, this->radius);
  dGeomSetBody(this->geom, this->body);
}

void Sphere::draw() const
{
  dsDrawSphere(dBodyGetPosition(this->body), dBodyGetRotation(this->body), this->radius);
}

dBodyID Sphere::getBodyId()
{
  return this->body;
}

dGeomID Sphere::getGeomId()
{
  return this->geom;
}

/*--------------------- Capsule class ----------------------*/

Capsule::Capsule(dWorldID world, dSpaceID space, dReal length, dReal radius, int direction, dReal x, dReal y, dReal z, dReal mass)
  : length(length), radius(radius), direction(direction), Object::Object(world, x, y, z, mass) 
{
  dMass m;
  dMassSetZero(&m);
  dMassSetCapsuleTotal(&m, this->mass, this->direction, this->radius, this->length);
  dBodySetMass(this->body, &m);
  this->geom = dCreateCapsule(space, this->radius, this->radius);
  dGeomSetBody(this->geom, this->body);
}

void Capsule::draw() const
{
  dsDrawCapsule(dBodyGetPosition(this->body), dBodyGetRotation(this->body), this->length, this->radius);
}

/*--------------------- Cylinder class ----------------------*/

Cylinder::Cylinder(dWorldID world, dSpaceID space, dReal length, dReal radius, int direction, dReal x, dReal y, dReal z, dReal mass)
  : length(length), radius(radius), direction(direction), Object::Object(world, x, y, z, mass) 
{
  dMass m;
  dMassSetZero(&m);
  dMassSetCylinderTotal(&m, this->mass, this->direction, this->radius, this->length);
  dBodySetMass(this->body, &m);
  this->geom = dCreateCylinder(space, this->radius, this->radius);
  dGeomSetBody(this->geom, this->body);
}

void Cylinder::draw() const
{
  dsDrawCylinder(dBodyGetPosition(this->body), dBodyGetRotation(this->body), this->length, this->radius);
}