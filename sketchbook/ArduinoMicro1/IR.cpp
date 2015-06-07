#include "IR.h"

int volt2cm[2][num_pts] = {
    {3150, 2990, 2750, 2300, 1650, 1300, 1090, 920, 730, 610, 510, 450, 400}, 
    {6, 7, 8, 10, 15, 20, 25, 30, 40, 50, 60, 70, 80}
};

int getCm(int raw)
{
  int y = map(raw, 0, 1023, 0, 5000);
//  Serial.print(y);
  int idx = 0;
  int *volt = volt2cm[0];
  int *cm = volt2cm[1];

  int x1, x2, y1, y2, x;

  if (y > volt[0])
    return cm[0];
  if (y < volt[num_pts - 1])
  { 
    return cm[num_pts - 1];
  }

  for (idx = 1; idx < num_pts; idx++)
  {
    if (y > volt[idx]) {
//        Serial.print(" idx: "); Serial.print(idx); Serial.print(", ");
        y1 = volt[idx - 1];
        y2 = volt[idx];
        x1 = cm[idx - 1];
        x2 = cm[idx];
        break;
    }
  }
  x = x1 + ((x2 - x1) * (y - y1)) / (y2 - y1);
  return x;
}

void IR_setup(int ir)
{
  analogReference(DEFAULT);
  pinMode (ir, INPUT);
}

int IR_get_dist(int ir)
{
  int raw=analogRead(ir);
  int *volt = volt2cm[0];
  int *cm = volt2cm[1];
  
  int dist = getCm(raw);

  return dist;  
  }
