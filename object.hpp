#ifndef __OBJECT_H_
#define __OBJECT_H_
#include <ode/ode.h>

typedef enum {BOX, }type_t;

typedef struct {
  dReal x, y, z;
} pos_t;

typedef struct {
  dReal x, y, z;
  dReal rad;
} shape_t;

class Object
{
  dBodyID body;
  dGeomID geom;
  pos_t pos;
  shape_t shape;
  dReal mass;
public:
  Object(dBodyID body, dGeomID geom, shape_t shape, pos_t pos, dReal mass);
  void Init(type_t t);
}

#endif // __OBJECT_H_