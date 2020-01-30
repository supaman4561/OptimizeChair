#include <stdio.h>
#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include <unistd.h>
#include <vector>
#include <algorithm>
#include "math.h"
#include "object.hpp"
#include "human.hpp"
#include "chair.hpp"

#ifdef dDOUBLE
#define dsDrawBox      dsDrawBoxD
#define dsDrawSphere   dsDrawSphereD
#define dsDrawCylinder dsDrawCylinderD
#define dsDrawCapsule  dsDrawCapsuleD
#endif

#define  ESC    0x1B
#define TORSO_SENSOR_WIDTH 5
#define TORSO_SENSOR_HEIGHT 10
#define RTHIGH_SENSOR_WIDTH 5
#define RTHIGH_SENSOR_HEIGHT 5
#define LTHIGH_SENSOR_WIDTH 5
#define LTHIGH_SENSOR_HEIGHT 5

static bool DRAWFLAG = false;
static bool DISPFLAG = false;
static bool nowSimulation = true;
static dWorldID world;
static dSpaceID space;
static dGeomID ground;
static dJointID fixed;
static dJointGroupID contactgroup;
dJointFeedback *feedback = new dJointFeedback;
dsFunctions fn;

// 人間
Human *human;

// 椅子
Box *seat;
Box *backrests[BACKREST_NUM];
dJointID backrest_joints[BACKREST_NUM];

// センサー
dJointID torso_sensor_joints[TORSO_SENSOR_HEIGHT][TORSO_SENSOR_WIDTH];
Box *torso_sensor_boxes[TORSO_SENSOR_HEIGHT][TORSO_SENSOR_WIDTH];
dJointFeedback *torso_feedbacks[TORSO_SENSOR_HEIGHT][TORSO_SENSOR_WIDTH];
dJointID rthigh_sensor_joints[RTHIGH_SENSOR_HEIGHT][RTHIGH_SENSOR_WIDTH];
Box *rthigh_sensor_boxes[RTHIGH_SENSOR_HEIGHT][RTHIGH_SENSOR_WIDTH];
dJointFeedback *rthigh_feedbacks[RTHIGH_SENSOR_HEIGHT][RTHIGH_SENSOR_WIDTH];
dJointID lthigh_sensor_joints[LTHIGH_SENSOR_HEIGHT][LTHIGH_SENSOR_WIDTH];
Box *lthigh_sensor_boxes[LTHIGH_SENSOR_HEIGHT][LTHIGH_SENSOR_WIDTH];
dJointFeedback *lthigh_feedbacks[LTHIGH_SENSOR_HEIGHT][LTHIGH_SENSOR_WIDTH];

// GAパラメータ
static int generation = 0;
static dReal fitness;
static int gen_size = 64;
static int elite_num = 8;
static dReal p_mutation = 0.05;
static dReal p_idv_mutation = 0.1;

dReal rss(const dReal v[3])
{
  return std::sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
}

void init_feedback()
{
  for (int i=0; i<TORSO_SENSOR_HEIGHT; i++) {
    for (int j=0; j<TORSO_SENSOR_WIDTH; j++) {
      torso_feedbacks[i][j] = new dJointFeedback;
    }
  }

  for (int i=0; i<RTHIGH_SENSOR_HEIGHT; i++) {
    for (int j=0; j<RTHIGH_SENSOR_WIDTH; j++) {
      rthigh_feedbacks[i][j] = new dJointFeedback;
    }
  }

  for (int i=0; i<RTHIGH_SENSOR_HEIGHT; i++) {
    for (int j=0; j<RTHIGH_SENSOR_WIDTH; j++) {
      lthigh_feedbacks[i][j] = new dJointFeedback;
    }
  }
}

static void nearCallback(void *data, dGeomID o1, dGeomID o2)
{
  static const int MAX_CONTACTS = 64;

  dBodyID b1 = dGeomGetBody(o1);
  dBodyID b2 = dGeomGetBody(o2);
  if (!(b1 && b2 && dAreConnected(b1, b2))) {
    dContact contact[MAX_CONTACTS];
    int n = dCollide(o1, o2, MAX_CONTACTS, &contact[0].geom, 
                    sizeof(dContact));
    int isGround = ((ground == o1) || (ground == o2));

    if (n > 0) {
      for (int i=0; i<n; i++) {
        contact[i].surface.mode = dContactBounce | dContactSoftCFM | dContactSoftERP;
        contact[i].surface.bounce = 0.02;
        if (isGround) { 
          contact[i].surface.mu   = 10.0;
        } else {
          contact[i].surface.mu   = 2.0;
        }
        contact[i].surface.soft_erp = 0.4;
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
  static dReal force = 0;
  dReal force_sum = 0;
  int valid_num = 0;

  dSpaceCollide(space, 0, &nearCallback);
  dWorldStep(world, 0.01);
  dJointGroupEmpty(contactgroup);

  if (DRAWFLAG) {
    dsSetColor(1, 1, 0);
    human->draw();
    seat->draw();
    for (int i=0; i<BACKREST_NUM; i++) {
      backrests[i]->draw();
    }
    dsSetColor(1, 0, 1);
  }

  if (DISPFLAG) printf("%c[2J", ESC);
  if (DISPFLAG) printf("GENERATION %d\n", generation);
  if (DISPFLAG) printf("STEP %d\n", steps);
  if (DISPFLAG) printf("====== TORSO ========\n");
  for (int i=0; i<TORSO_SENSOR_HEIGHT; i++) {
    for (int j=0; j<TORSO_SENSOR_WIDTH; j++) {
      if (DRAWFLAG){
        (torso_sensor_boxes[i][j])->draw();
      }
      torso_feedbacks[i][j] = dJointGetFeedback(torso_sensor_joints[i][j]);
      force_sum += rss(torso_feedbacks[i][j]->f1);
      if (rss(torso_feedbacks[i][j]->f1) >= 0.01) valid_num++;
      if (DISPFLAG) printf("%6.2f ", rss(torso_feedbacks[i][j]->f1));
    }
    if (DISPFLAG) printf("\n");
  }

  if (DISPFLAG) printf("====== RTHIGH ========\n");
  for (int i=0; i<RTHIGH_SENSOR_HEIGHT; i++) {
    for (int j=0; j<RTHIGH_SENSOR_WIDTH; j++) {
      if (DRAWFLAG) {
        (rthigh_sensor_boxes[i][j])->draw();
      }
      rthigh_feedbacks[i][j] = dJointGetFeedback(rthigh_sensor_joints[i][j]);
      force_sum += rss(rthigh_feedbacks[i][j]->f1);
      if (rss(rthigh_feedbacks[i][j]->f1) >= 0.01) valid_num++;
      if (DISPFLAG) printf("%6.2f ", rss(rthigh_feedbacks[i][j]->f1));
    }
    if (DISPFLAG) printf("\n");
  }

  if (DISPFLAG) printf("====== LTHIGH ========\n");
  for (int i=0; i<LTHIGH_SENSOR_HEIGHT; i++) {
    for (int j=0; j<LTHIGH_SENSOR_WIDTH; j++) {
      if (DRAWFLAG) {
        (lthigh_sensor_boxes[i][j])->draw();
      }
      lthigh_feedbacks[i][j] = dJointGetFeedback(lthigh_sensor_joints[i][j]);
      force_sum += rss(lthigh_feedbacks[i][j]->f1);
      if (rss(lthigh_feedbacks[i][j]->f1) >= 0.01) valid_num++;
      if (DISPFLAG) printf("%6.2f ", rss(lthigh_feedbacks[i][j]->f1));
    }
    if (DISPFLAG) printf("\n");
  }

  if (steps >= 200) {
    fitness = -1 * force + 10 * human->thighIsOn(*seat) + valid_num;
    if (DISPFLAG) printf("fitness = %6.4f\n", fitness);
    steps = 0;
    force = 0;
    dsStop();
    nowSimulation = false;
  } else if (steps >= 100) {
    force += force_sum / (TORSO_SENSOR_WIDTH + TORSO_SENSOR_HEIGHT + 
                          RTHIGH_SENSOR_WIDTH + RTHIGH_SENSOR_HEIGHT +
                          LTHIGH_SENSOR_WIDTH + LTHIGH_SENSOR_HEIGHT) / (valid_num+1);
  }
  steps++;
}

void start()
{
  static float xyz[3] = {-3.0, 0.0, 1.0};
  static float hpr[3] = {0.0, 0.0, 0.0}; 
  dsSetViewpoint(xyz, hpr);
}

void setDrawStuff() 
{
  fn.version = DS_VERSION;
  fn.start = &start;
  fn.step = &simLoop;
  fn.command = NULL;
  fn.stop = NULL;
  fn.path_to_textures = "/usr/local/lib/drawstuff.textures";
}

void generate_sensor()
{
  for(int i=0; i<TORSO_SENSOR_HEIGHT; i++) {
    for(int j=0; j<TORSO_SENSOR_WIDTH; j++) {
      dReal lx = 0.4 / TORSO_SENSOR_WIDTH;
      dReal lz = 0.6 / TORSO_SENSOR_HEIGHT;
      torso_sensor_boxes[i][j] = new Box(world, space, lx*0.9, 0.05, lz*0.9,  
                                         0.2-lx/2-lx*j, 0.125, 
                                         2.1-lz/2-lz*i, 1e-4);
      torso_sensor_joints[i][j] = dJointCreateFixed(world, 0);
      human->jointAttachToTorso(torso_sensor_joints[i][j],
                                torso_sensor_boxes[i][j]->getBodyId());
      dJointSetFixed(torso_sensor_joints[i][j]);
      dJointSetFeedback(torso_sensor_joints[i][j], torso_feedbacks[i][j]);
    }
  }

  for(int i=0; i<RTHIGH_SENSOR_HEIGHT; i++) {
    for(int j=0; j<RTHIGH_SENSOR_WIDTH; j++) {
      dReal lx = 0.15 / RTHIGH_SENSOR_WIDTH;
      dReal lz = 0.4 / RTHIGH_SENSOR_HEIGHT;
      rthigh_sensor_boxes[i][j] = new Box(world, space, lx*0.9, 0.05, lz*0.9,  
                                         -0.05-lx/2-lx*j, 0.125, 
                                         1.5-lz/2-lz*i, 1e-4);
      rthigh_sensor_joints[i][j] = dJointCreateFixed(world, 0);
      human->jointAttachToRthigh(rthigh_sensor_joints[i][j],
                                rthigh_sensor_boxes[i][j]->getBodyId());
      dJointSetFixed(rthigh_sensor_joints[i][j]);
      dJointSetFeedback(rthigh_sensor_joints[i][j], rthigh_feedbacks[i][j]);
    }
  }

  for(int i=0; i<LTHIGH_SENSOR_HEIGHT; i++) {
    for(int j=0; j<LTHIGH_SENSOR_WIDTH; j++) {
      dReal lx = 0.15 / LTHIGH_SENSOR_WIDTH;
      dReal lz = 0.4 / LTHIGH_SENSOR_HEIGHT;
      lthigh_sensor_boxes[i][j] = new Box(world, space, lx*0.9, 0.05, lz*0.9,  
                                         0.2-lx/2-lx*j, 0.125, 
                                         1.5-lz/2-lz*i, 1e-4);
      lthigh_sensor_joints[i][j] = dJointCreateFixed(world, 0);
      human->jointAttachToLthigh(lthigh_sensor_joints[i][j],
                                lthigh_sensor_boxes[i][j]->getBodyId());
      dJointSetFixed(lthigh_sensor_joints[i][j]);
      dJointSetFeedback(lthigh_sensor_joints[i][j], lthigh_feedbacks[i][j]);
    }
  }    
}

void generate_chair(chairInfo c)
{
  dMatrix3 R;
  seat = new Box(world, space, 0.6, c.ly, 0.01, 0.0, c.y, c.z, 1e-3);
  fixed = dJointCreateFixed(world, 0);
  dJointAttach(fixed, seat->getBodyId(), 0);
  for (int i=0; i<BACKREST_NUM; i++) {
    backrests[i] = new Box(world, space, 0.6, 0.01, 0.1, 0.0, c.y+c.ly/2, c.z+0.1*i, 1e-3);
    backrest_joints[i] = dJointCreateHinge(world, 0);
    if (i==0) {
      dJointAttach(backrest_joints[i], backrests[i]->getBodyId(), seat->getBodyId());
    } else {
      dJointAttach(backrest_joints[i], backrests[i]->getBodyId(), backrests[i-1]->getBodyId());
    }
    dJointSetHingeAnchor(backrest_joints[i], 0.0,  c.y+c.ly/2, c.z+0.1*i);
    dJointSetHingeAxis(backrest_joints[i], 1, 0, 0);
    dJointSetHingeParam(backrest_joints[i], dParamFudgeFactor, 0);
  }

  dRFromAxisAndAngle(R, -1, 0, 0, M_PI * c.seat_angle / 180);
  dGeomSetRotation(seat->getGeomId(), R);
  for (int i=0; i<BACKREST_NUM; i++) {
    setHingeJointAngle(backrest_joints[i], M_PI *  c.back_angle[i] / 180);
  }
  dJointSetFixed(fixed);
}

void simulation(int argc, char **argv, chairInfo ci)
{
  init_feedback();
  world = dWorldCreate();
  dWorldSetERP(world, 0.1);
  dWorldSetCFM(world, 1e-8);
  space = dHashSpaceCreate(0);
  contactgroup = dJointGroupCreate(0);
  dWorldSetGravity(world, 0, 0, -9.8);

  ground = dCreatePlane(space, 0, 0, 1, 0);

  human = new Human(world, space, 0.0, 0.0, 0.0, 80.0, 60.0);
  generate_sensor();
  generate_chair(ci);

  if (DRAWFLAG) {
    dsSimulationLoop(argc, argv, 400, 400, &fn);
  } else {
    while (nowSimulation) {
      simLoop(0);
    }
    nowSimulation = true;
  }
  human->destroy();
  dJointGroupDestroy(contactgroup);
  for (int i=0; i<TORSO_SENSOR_HEIGHT; i++) {
    for (int j=0; j<TORSO_SENSOR_WIDTH; j++) {
      torso_sensor_boxes[i][j]->destroy();
    }
  }
  for (int i=0; i<RTHIGH_SENSOR_HEIGHT; i++) {
    for (int j=0; j<RTHIGH_SENSOR_WIDTH; j++) {
      rthigh_sensor_boxes[i][j]->destroy();
    }
  }
  for (int i=0; i<LTHIGH_SENSOR_HEIGHT; i++) {
    for (int j=0; j<LTHIGH_SENSOR_WIDTH; j++) {
      lthigh_sensor_boxes[i][j]->destroy();
    }
  }
  dWorldDestroy(world); 
}

int main(int argc, char **argv)
{
  std::vector<chairInfo> ci(gen_size);
  setDrawStuff();

  dInitODE();
  for (int i=0; i<gen_size; i++) {
    initChairInfo(ci[i]);
  }

  while(1) {
    if (generation > 30) {
      DRAWFLAG = true;
      DISPFLAG = true;
    }
    for (int i=0; i<gen_size; i++) {
      fitness = 0;
      simulation(argc, argv, ci[i]);
      ci[i].fitness = fitness;
    }
    std::sort(ci.begin(), ci.end(), compare_fitness);
    printf("[%d] %2.6f\n", generation, ci[0].fitness);

    // 交叉により子を生成    
    int k=elite_num;
    for (int i=0; i<elite_num-1; i++) {
      for (int j=i+1; j<elite_num; j++) {
        crossover(ci[i], ci[j], ci[k], ci[k+1]);
        k += 2;
      }
    }

    // 突然変異
    for (auto &e : ci) {
      mutation(e, p_mutation, p_idv_mutation);
    }

    generation++;
  }
  dCloseODE();

  return 0;
}