#include <stdlib.h>
#include <ode/ode.h>
#include <random>

typedef struct {
  dReal y;
  dReal z;
  dReal ly;
  dReal seat_angle;
  dReal *back_angle;
  dReal fitness;
}chairInfo;

void setY(dReal &y)
{
  std::random_device rd{};
  std::mt19937 mt(rd());
  std::uniform_real_distribution<dReal> score(0.3,0.8);
  y = score(mt);
}

void setZ(dReal &z)
{
  std::random_device rd{};
  std::mt19937 mt(rd());
  std::uniform_real_distribution<dReal> score(0.2,0.6);
  z = score(mt);
}

void setLy(dReal &ly)
{
  std::random_device rd{};
  std::mt19937 mt(rd());
  std::uniform_real_distribution<dReal> score(0.1,0.8);
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

void initChairInfo(chairInfo &c, int n_backrest) 
{
  setY(c.y);
  setZ(c.z);
  setLy(c.ly);
  setSeatAngle(c.seat_angle);
  c.back_angle = (dReal *)malloc(sizeof(dReal) * n_backrest);
  for (int i=0; i<n_backrest; i++) {
    setBackAngle(c.back_angle[i]);
  }
  c.fitness = 0;
}