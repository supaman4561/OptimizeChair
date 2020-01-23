#include <ode/ode.h>
#include "object.hpp"

Object::Object(dBodyID body, dGeomID geom, shape_t shape, pos_t pos, dReal mass)
{
  this->body = body;
  this->geom = geom;
  this->shape = shape;
  this->pos = pos;
  this->mass = mass;
}