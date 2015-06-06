#define ir A3
const int num_pts = 13;
float volt2cm[2][num_pts] = {
    {3150, 2990, 2750, 2300, 1650, 1300, 1090, 920, 730, 610, 510, 450, 400}, 
    {6.0, 7.0, 8.0, 10.0, 15.0, 20.0, 25.0, 30.0, 40.0, 50.0, 60.0, 70.0, 80.0}
};

int getCm(int raw)
{
  int y = map(raw, 0, 1023, 0, 5000);
  Serial.print(y);
  int idx = 0;
  float *volt = volt2cm[0];
  float *cm = volt2cm[1];

  float x1, x2, y1, y2, x;

  if (y > volt[0])
    return cm[0];
  if (y < volt[num_pts - 1])
  { 
//    Serial.print("   ");
//    Serial.print(cm[num_pts - 1]);
//    Serial.print("   ");
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
//        Serial.print("y1: "); Serial.print(y1); Serial.print(", ");
//        Serial.print("y2: "); Serial.print(y2); Serial.print(", ");
//        Serial.print("x1: "); Serial.print(x1); Serial.print(", ");
//        Serial.print("x2: "); Serial.print(x2); Serial.print(", ");
        break;
    }
  }
  x = x1 + ((x2 - x1) * (y - y1)) / (y2 - y1);
  return x;
}

void setup()
{
  
  analogReference(DEFAULT);
  Serial.begin(115200);
  pinMode (ir, INPUT);
}

void loop()
{
  int raw=analogRead(ir);
  float *volt = volt2cm[0];
  float *cm = volt2cm[1];
  
//  for(int a = 0; a < num_pts; ++a) {
//    Serial.print("idx: "); Serial.print(a); 
//    Serial.print(" volt: "); Serial.print(volt[a]);
//    Serial.print(" cm: "); Serial.print(cm[a]);
//    Serial.println("");
//  }
  Serial.print(raw);
  Serial.print("->");
  int dist = getCm(raw);
  Serial.print("->");
  Serial.print(dist);
  Serial.println("cm");
  delay(500);
  
  }
