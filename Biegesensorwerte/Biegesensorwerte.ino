#include <Wire.h>

const int MPU1 = 0x68, MPU2 = 0x69;

long accelX, accelY, accelZ;
float gForceX, gForceY, gForceZ, gyroX, gyroY, gyroZ,rotX, rotY, rotZ;
long accelX2, accelY2, accelZ2;
float gForceX2, gForceY2, gForceZ2;


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
  Serial.begin (115200); // Start der seriellen Verbindung für den serial monitor.
}

struct SFloatRawData {unsigned long t; int flex6; double ax, ay, az, gx, gy, gz;};

void loop()
{
  GetMpuValue(MPU1);

  delay(33); // Wartezeit zwischen den einzelnen Ausgaben der Sensorwerte
}

void printCSVData(struct SFloatRawData fdata) {
    static bool printHeader = true;
    if (printHeader) {
        //Serial.println("time,Flex6,GyroX,GyroY,GyroZ,AccX,AccY,AccZ");
        Serial.println("time;Flex6;GyroY");
        printHeader = false;
    }
    Serial.print(fdata.t);  Serial.print(";");
    Serial.print(fdata.flex6); Serial.print(";");
    //Serial.print(fdata.gx); Serial.print(";");
    Serial.println(fdata.gy); 
    //Serial.print(fdata.gz); Serial.print(";");
    //Serial.print(fdata.ax); Serial.print(";");
    //Serial.print(fdata.ay); Serial.print(";");
    //Serial.println(fdata.az);
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

  struct SFloatRawData fdata;

  /*a0 = analogRead(A0);
  a1 = analogRead(A1);
  a2 = analogRead(A2);
  a3 = analogRead(A3);*/
  fdata.t = millis();
  fdata.flex6 = analogRead(A6);
  fdata.ax = accelX / 16384.0; fdata.ay = accelY / 16384.0; fdata.az = accelZ / 16384.0;
  fdata.gx = gyroX / 131.0; fdata.gy = gyroY / 131.0; fdata.gz = gyroZ / 131.0;
  printCSVData(fdata);
}
