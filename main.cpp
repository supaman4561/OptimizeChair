#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include "object.hpp"
#include "human.hpp"
#include "math.h"

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

dJointID neck, rback, lback, rnee, lnee;
Sphere *head;
Box *torso;
Box *rthigh;
Box *lthigh;
Box *rleg;
Box *lleg;

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
        contact[i].surface.mode = dContactBounce | dContactSoftCFM | dContactSoftERP;
        contact[i].surface.bounce = 1e-2;
        contact[i].surface.mu   = dInfinity;
        contact[i].surface.soft_erp = 0.2;
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
  dVector3 xyz;

  dSpaceCollide(space, 0, &nearCallback);
  dWorldStep(world, 0.01);
  dJointGroupEmpty(contactgroup);

  dsSetColor(1, 1, 0);
  head->draw();
  dsSetColor(0, 0, 1);
  torso->draw();
  dsSetColor(1, 0, 0);
  rthigh->draw();
  lthigh->draw();
  dsSetColor(0, 1, 0);
  rleg->draw();
  lleg->draw();

  // feedback = dJointGetFeedback(fixed);
  // printf("%5d Force fx=%6.2f ",steps++,feedback->f1[0]);
  // printf("fy=%6.2f ",feedback->f1[1]);
  // printf("fz=%6.2f \n",feedback->f1[2]);

}

void start()
{
  // static float xyz[3] = {0.0, -3.0, 1.0};
  // static float hpr[3] = {90.0, 0.0, 0.0}; 

  static float xyz[3] = {-3.0, 0.0, 1.0};
  static float hpr[3] = {0.0, 0.0, 0.0}; 
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
  dWorldSetERP(world, 0.1);
  dWorldSetCFM(world, 1e-8);
  space = dHashSpaceCreate(0);
  contactgroup = dJointGroupCreate(0);
  dWorldSetGravity(world, 0, 0, -9.8);

  ground = dCreatePlane(space, 0, 0, 1, 0);

  head = new Sphere(world, space, 0.15, 0, 0, 1.55, 0.48);
  torso = new Box(world, space, 0.4, 0.2, 0.6, 0, 0, 1.1, 2.8);
  rthigh = new Box(world, space, 0.15, 0.2, 0.4, -0.125, 0, 0.6, 0.84);
  lthigh = new Box(world, space, 0.15, 0.2, 0.4, 0.125, 0, 0.6, 0.84);
  rleg = new Box(world, space, 0.15, 0.2, 0.4, -0.125, 0, 0.2, 0.72);
  lleg = new Box(world, space, 0.15, 0.2, 0.4, 0.125, 0, 0.2, 0.72);

  neck = dJointCreateFixed(world, 0);
  dJointAttach(neck, head->getBodyId(), torso->getBodyId());
  dJointSetFixed(neck);

  rback = dJointCreateHinge(world, 0);
  dJointAttach(rback, torso->getBodyId(), rthigh->getBodyId());
  dJointSetHingeAnchor(rback, -0.125, -0.1, 0.8);
  dJointSetHingeAxis(rback, 1, 0, 0);

  lback = dJointCreateHinge(world, 0);
  dJointAttach(lback, torso->getBodyId(), lthigh->getBodyId());
  dJointSetHingeAnchor(lback, 0.125, -0.1, 0.8);
  dJointSetHingeAxis(lback, 1, 0, 0);
  
  rnee = dJointCreateHinge(world, 0);
  dJointAttach(rnee, rthigh->getBodyId(), rleg->getBodyId());
  dJointSetHingeAnchor(rnee, -0.125, 0.1, 0.4);
  dJointSetHingeAxis(rnee, 1, 0, 0);

  lnee = dJointCreateHinge(world, 0);
  dJointAttach(lnee, lthigh->getBodyId(), lleg->getBodyId());
  dJointSetHingeAnchor(lnee, 0.125, 0.1, 0.4);
  dJointSetHingeAxis(lnee, 1, 0, 0);
  
  
  dJointSetHingeParam(rback, dParamLoStop, M_PI/2);
  dJointSetHingeParam(rback, dParamHiStop, M_PI/2);
  dJointSetHingeParam(rback, dParamFudgeFactor, 0);

  dJointSetHingeParam(lback, dParamLoStop, M_PI/2);
  dJointSetHingeParam(lback, dParamHiStop, M_PI/2);
  dJointSetHingeParam(lback, dParamFudgeFactor, 0);

  dJointSetHingeParam(rnee, dParamLoStop, -M_PI/2);
  dJointSetHingeParam(rnee, dParamHiStop, -M_PI/2);
  dJointSetHingeParam(rnee, dParamFudgeFactor, 0);

  dJointSetHingeParam(lnee, dParamLoStop, -M_PI/2);
  dJointSetHingeParam(lnee, dParamHiStop, -M_PI/2);
  dJointSetHingeParam(lnee, dParamFudgeFactor, 0);

  dsSimulationLoop(argc, argv, 400, 400, &fn);
  dWorldDestroy(world);
  dCloseODE();

  return 0;
}