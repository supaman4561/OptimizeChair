#ifndef __OBJECT_H_
#define __OBJECT_H_
#include <ode/ode.h>

class Object
{
protected:
  dBodyID body;
  dGeomID geom;
  dReal x, y, z;
  dReal mass;
public:
  Object(dWorldID world, dReal x, dReal y, dReal z, dReal mass);
  dBodyID getBodyId();
  dGeomID getGeomId();
  void draw() const {};
  void destroy();
};

class Box : public Object
{
  dReal lx, ly, lz;
public:
  Box(dWorldID world, dSpaceID space, dReal lx, dReal ly, dReal lz, dReal x, dReal y, dReal z, dReal mass);
  dBodyID getBodyId();
  dGeomID getGeomId();
  void draw() const;
};

class Sphere : public Object
{
  dReal radius;
public:
  Sphere(dWorldID world, dSpaceID space, dReal radius, dReal x, dReal y, dReal z, dReal mass);
  dBodyID getBodyId();
  dGeomID getGeomId();
  void draw() const;
};

class Capsule : public Object
{
  dReal length, radius;
  int direction;
public:
  Capsule(dWorldID world, dSpaceID space, dReal length, dReal radius, int direction, dReal x, dReal y, dReal z, dReal mass);
  dBodyID getBodyId();
  dGeomID getGeomId();
  void draw() const;
};

class Cylinder : public Object
{
  dReal length, radius;
  int direction;
public:
  Cylinder(dWorldID world, dSpaceID space, dReal length, dReal radius, int direction, dReal x, dReal y, dReal z, dReal mass);
  dBodyID getBodyId();
  dGeomID getGeomId();
  void draw() const;
};

void setHingeJointAngle(dJointID joint, dReal value);

#endif // __OBJECT_H_