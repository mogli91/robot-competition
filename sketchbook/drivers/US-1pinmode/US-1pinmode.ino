	
//HC RS04 Ultrasound sensor
int pin = 11;
 
void setup() {
  Serial.begin( 115200 );                     //Set the speed of serial communication to 9600 bit/s
  Serial.println( "Ultrasound Sensor: ");
 
}
 
void loop() {
  
long duration, cm;

pinMode(pin, OUTPUT);
digitalWrite(pin, LOW );
delayMicroseconds(2);
//send a 10microsecond impulse to the trigger (it makes the sensor send a wave): I keep it high for 10microsec
digitalWrite( pin, HIGH );
delayMicroseconds( 10 );
digitalWrite( pin, LOW );
 
pinMode(pin, INPUT);
duration = pulseIn( pin, HIGH );
 
cm = 0.034 * duration / 2;
 
//Serial.print( "duration: " );
//Serial.print( duration );
//Serial.print( " , " );
//Serial.print( "distance: " );
 
//after 38ms it's out of the range of the sensor
//if( duration > 38000 ) Serial.println( "out of range");
//else { Serial.print( cm ); Serial.println( "cm" );}

char dum[4];
sprintf(dum, "%03d", cm);
Serial.print('f'); Serial.write(dum, 3); Serial.println("");
//wait 1.5 seconds
delay( 500 );  //Dans ce cas on a essaye avec 0.5 s
}

