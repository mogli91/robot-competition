#define ir 1


void setup()
{
  
  analogReference(DEFAULT);
  Serial.begin(9600);
  pinMode (ir, INPUT);
}

void loop()
{

  
  int raw=analogRead(ir);
  int volt=map(raw, 0, 1023, 0, 5000);
  int cm=(21.61/(volt-0.1696))*1000;
  
  Serial.println(cm); 
  
  delay(500);
  
  }
