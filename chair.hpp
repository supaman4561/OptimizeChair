#include <stdlib.h>
#include <ode/ode.h>
#include <random>

#define BACKREST_NUM 10

typedef struct {
  dReal y;
  dReal z;
  dReal ly;
  dReal seat_angle;
  dReal back_angle[BACKREST_NUM];
  dReal fitness;
}chairInfo;

void setY(dReal &y)
{
  std::random_device rd{};
  std::mt19937 mt(rd());
  std::uniform_real_distribution<dReal> score(0.3,0.5);
  y = score(mt);
}

void setZ(dReal &z)
{
  std::random_device rd{};
  std::mt19937 mt(rd());
  std::uniform_real_distribution<dReal> score(0.2,0.5);
  z = score(mt);
}

void setLy(dReal &ly)
{
  std::random_device rd{};
  std::mt19937 mt(rd());
  std::uniform_real_distribution<dReal> score(0.4,0.8);
  ly = score(mt);
}

void setSeatAngle(dReal &angle)
{
  std::random_device rd{};
  std::mt19937 mt(rd());
  std::uniform_real_distribution<dReal> score(0.0,10.0);
  angle = score(mt);
}

void setBackAngle(dReal &angle)
{
  std::random_device rd{};
  std::mt19937 mt(rd());
  std::uniform_real_distribution<dReal> score(-10.0,10.0);
  angle = score(mt);
}

void initChairInfo(chairInfo &c) 
{
  setY(c.y);
  setZ(c.z);
  setLy(c.ly);
  setSeatAngle(c.seat_angle);
  for (int i=0; i<BACKREST_NUM; i++) {
    setBackAngle(c.back_angle[i]);
  }
  c.fitness = 0;
}

bool compare_fitness(chairInfo c1, chairInfo c2) 
{
  return c1.fitness > c2.fitness;
}

void crossover(chairInfo p1, chairInfo p2, chairInfo &c1, chairInfo &c2)
{
  chairInfo child;
  std::random_device rd{};
  std::mt19937 mt(rd());
  std::uniform_int_distribution<> rand2(0, 1);
  std::uniform_int_distribution<> randN(0, BACKREST_NUM);

  // 一様交叉
  if (rand2(mt) == 0) {
    c1.y = p1.y;
    c2.y = p2.y;
  } else {
    c1.y = p2.y;
    c2.y = p1.y;
  }

  if (rand2(mt) == 0) {
    c1.z = p1.z;
    c2.z = p2.z;
  } else {
    c1.z = p2.z;
    c2.z = p1.z;
  }

  if (rand2(mt) == 0) {
    c1.ly = p1.ly;
    c2.ly = p2.ly;
  } else {
    c1.ly = p2.ly;
    c2.ly = p1.ly;
  }

  if (rand2(mt) == 0) {
    c1.seat_angle = p1.seat_angle;
    c2.seat_angle = p2.seat_angle;
  } else {
    c1.seat_angle = p2.seat_angle;
    c2.seat_angle = p1.seat_angle;
  }
  
  // 一点交叉
  int pibot = randN(mt);
  if (rand2(mt) == 0) {
    for (int i=0; i<pibot; i++) {
      c1.back_angle[i] = p1.back_angle[i];
      c2.back_angle[i] = p2.back_angle[i];
    }
    for (int i=pibot; i<BACKREST_NUM; i++) {
      c1.back_angle[i] = p2.back_angle[i];
      c2.back_angle[i] = p1.back_angle[i];
    }
  } else {
    for (int i=0; i<pibot; i++) {
      c1.back_angle[i] = p2.back_angle[i];
      c2.back_angle[i] = p1.back_angle[i];
    }
    for (int i=pibot; i<BACKREST_NUM; i++) {
      c1.back_angle[i] = p1.back_angle[i];
      c2.back_angle[i] = p2.back_angle[i];
    }
  }
}

void mutation(chairInfo &c, dReal p_mutation, dReal p_idv_mutation)
{
  std::random_device rd{};
  std::mt19937 mt(rd());
  std::uniform_real_distribution<dReal> randm(0.0, 1.0);

  if (randm(mt) < p_mutation) {
    if (randm(mt) < p_idv_mutation) setY(c.y);
    if (randm(mt) < p_idv_mutation) setZ(c.z);
    if (randm(mt) < p_idv_mutation) setLy(c.ly);
    if (randm(mt) < p_idv_mutation) setSeatAngle(c.seat_angle);
    for (int i=0; i<BACKREST_NUM; i++) {
      if (randm(mt) < p_idv_mutation) setBackAngle(c.back_angle[i]);
    }
  }
}