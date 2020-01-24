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
  virtual void setGeom(dSpaceID space) = 0;
  virtual void draw() const = 0;
};

class Box : public Object
{
  dReal lx, ly, lz;
public:
  Box(dWorldID world, dReal lx, dReal ly, dReal lz, dReal x, dReal y, dReal z, dReal mass);
  void setGeom(dSpaceID space);
  void draw() const;
};

#endif // __OBJECT_H_