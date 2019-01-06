#include <Wire.h>

const int MPU1 = 0x68, MPU2 = 0x69;

long accelX, accelY, accelZ;
float gForceX, gForceY, gForceZ, gyroX, gyroY, gyroZ,rotX, rotY, rotZ;
long accelX2, accelY2, accelZ2;
float gForceX2, gForceY2, gForceZ2;
/*int a0 = 0; // Benennung als Variable für den Biegesensor
int a1 = 0; // Benennung als Variable für den Biegesensor
int a2 = 0; // Benennung als Variable für den Biegesensor
int a3 = 0; // Benennung als Variable für den Biegesensor*/
int a6 = 0; // Benennung als Variable für den Biegesensor


void setup()
{
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  digitalWrite(4, LOW); // MPU1 = 0x68 (Ring)
  digitalWrite(5, HIGH); // MPU2 = 0x69 (Handgelenk)
  
  Wire.begin();
  Wire.beginTransmission(MPU1);
  Wire.write(0x6B);
  Wire.write(0b00000000);
  Wire.endTransmission();  
  Wire.beginTransmission(MPU1);
  Wire.write(0x1B);
  Wire.write(0x00000000);
  Wire.endTransmission(); 
  Wire.beginTransmission(MPU1);
  Wire.write(0x1C);
  Wire.write(0b00000000);
  Wire.endTransmission(); 
  
  Wire.begin();
  Wire.beginTransmission(MPU2);
  Wire.write(0x6B);
  Wire.write(0b00000000); 
  Wire.endTransmission();  
  Wire.beginTransmission(MPU2); 
  Wire.write(0x1B);
  Wire.write(0x00000000);
  Wire.endTransmission(); 
  Wire.beginTransmission(MPU2);
  Wire.write(0x1C);
  Wire.write(0b00000000);
  Wire.endTransmission(); 
  Serial.begin (9600); // Start der seriellen Verbindung für den serial monitor.
}

void loop()
{

  /*a0 = analogRead(A0);
  a1 = analogRead(A1);
  a2 = analogRead(A2);
  a3 = analogRead(A3);*/
  a6 = analogRead(A6);
  /*Serial.print ("a0: ");
  Serial.print (a0); // Ausgabe des Sensorwertes der x-Achse an den serial monitor
  Serial.print (" a1: "); // Ausgabe des Sensorwertes der y-Achse an den serial monitor
  Serial.print (a1); // Ausgabe des Sensorwertes der y-Achse an den serial monitor
  Serial.print (" a2: "); // Ausgabe des Sensorwertes der y-Achse an den serial monitor
  Serial.print (a2); // Ausgabe des Sensorwertes der y-Achse an den serial monitor
  Serial.print (" a3: "); // Ausgabe des Sensorwertes der y-Achse an den serial monitor
  Serial.print (a3); // Ausgabe des Sensorwertes der y-Achse an den serial monitor
  Serial.print (" a6: "); // Ausgabe des Sensorwertes der y-Achse an den serial monitor*/
  Serial.print ("Biege: "); // Ausgabe des Sensorwertes der y-Achse an den serial monitor
  Serial.println (a6); // Ausgabe von vier Leerzeichen

  GetMpuValue(MPU1);

  delay(100); // Wartezeit zwischen den einzelnen Ausgaben der Sensorwerte
}

void GetMpuValue(const int MPU){
  Wire.beginTransmission(MPU); 
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(MPU,6);
  while(Wire.available() < 6);
  accelX = Wire.read()<<8|Wire.read(); 
  accelY = Wire.read()<<8|Wire.read(); 
  accelZ = Wire.read()<<8|Wire.read();
  
  Wire.beginTransmission(MPU);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(MPU,6);
  while(Wire.available() < 6);
  gyroX = Wire.read()<<8|Wire.read();
  gyroY = Wire.read()<<8|Wire.read();
  gyroZ = Wire.read()<<8|Wire.read(); 


  gForceX = accelX / 16384.0;
  gForceY = accelY / 16384.0; 
  gForceZ = accelZ / 16384.0;
  rotX = gyroX / 131.0;
  rotY = gyroY / 131.0; 
  rotZ = gyroZ / 131.0;
  Serial.print("Gyro: ");
  Serial.print(rotX);
  Serial.print(", ");
  Serial.print(rotY);
  Serial.print(", ");
  Serial.println(rotZ);
  Serial.print("Acc: ");
  Serial.print(gForceX);
  Serial.print(", ");
  Serial.print(gForceY);
  Serial.print(", ");
  Serial.println(gForceZ);
}
