#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include "object.hpp"

#ifdef dDOUBLE
#define dsDrawBox      dsDrawBoxD
#define dsDrawSphere   dsDrawSphereD
#define dsDrawCylinder dsDrawCylinderD
#define dsDrawCapsule  dsDrawCapsuleD
#endif

#define DRAW

static dWorldID world;
static dSpaceID space;
static dGeomID ground;
static dJointID fixed;
static dJointGroupID contactgroup;
dJointFeedback *feedback = new dJointFeedback;
dsFunctions fn;

Box* box[2];

static void nearCallback(void *data, dGeomID o1, dGeomID o2)
{
  static const int MAX_CONTACTS = 64;

  dBodyID b1 = dGeomGetBody(o1);
  dBodyID b2 = dGeomGetBody(o2);
  if (!(b1 && b2 && dAreConnected(b1, b2))) {
    dContact contact[MAX_CONTACTS];
    int n = dCollide(o1, o2, MAX_CONTACTS, &contact[0].geom, 
                    sizeof(dContact));

    if (n > 0) {
      for (int i=0; i<n; i++) {
        contact[i].surface.mode = dContactSoftCFM | dContactSoftERP;
        contact[i].surface.mu   = dInfinity;
        contact[i].surface.soft_erp = 0.5;
        contact[i].surface.soft_cfm = 1e-8;
        dJointID c = dJointCreateContact(world, contactgroup, &contact[i]);

        dJointAttach(c, dGeomGetBody(contact[i].geom.g1),
              dGeomGetBody(contact[i].geom.g2));
      }
    }
  }
}

static void simLoop(int pause) 
{
  static int steps = 0;

  dSpaceCollide(space, 0, &nearCallback);
  dWorldStep(world, 0.01);
  dJointGroupEmpty(contactgroup);

  feedback = dJointGetFeedback(fixed);
  printf("%5d Force fx=%6.2f ",steps++,feedback->f1[0]);
  printf("fy=%6.2f ",feedback->f1[1]);
  printf("fz=%6.2f \n",feedback->f1[2]);

  box[0]->draw();
  box[1]->draw();
}

void start()
{
  static float xyz[3] = {0.0, -3.0, 1.0};
  static float hpr[3] = {90.0, 0.0, 0.0};
  dsSetViewpoint(xyz, hpr);
}

void setDrawStuff() {
  fn.version = DS_VERSION;
  fn.start = &start;
  fn.step = &simLoop;
  fn.command = NULL;
  fn.stop = NULL;
  fn.path_to_textures = "/usr/local/lib/drawstuff.textures";
}

int main(int argc, char **argv)
{
  setDrawStuff();

  dInitODE();
  world = dWorldCreate();
  space = dHashSpaceCreate(0);
  contactgroup = dJointGroupCreate(0);
  dWorldSetGravity(world, 0, 0, -9.8);

  ground = dCreatePlane(space, 0, 0, 1, 0);

  box[0] = new Box(world, 0.2, 0.2, 0.2, 0.0, 0.0, 0.5, 1.0);
  box[0]->createGeom(space);
  box[1] = new Box(world, 0.2, 0.2, 0.2, 0.0, 0.0, 0.8, 1.0);
  box[1]->createGeom(space);

  fixed = dJointCreateFixed(world, 0);
  dJointAttach(fixed, box[0]->getBodyId(), box[1]->getBodyId());
  dJointSetFixed(fixed);
  dJointSetFeedback(fixed, feedback);

  dsSimulationLoop(argc, argv, 400, 400, &fn);
  dWorldDestroy(world);
  dCloseODE();

  return 0;
}