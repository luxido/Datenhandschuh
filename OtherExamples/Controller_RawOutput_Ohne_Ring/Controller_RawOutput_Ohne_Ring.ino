#include <Wire.h>
#include "MPU6050.h"
#include <Adafruit_NeoPixel.h>
#include <math.h>

Adafruit_NeoPixel strip = Adafruit_NeoPixel(8, 3, NEO_GRB + NEO_KHZ800);

void LedRingSetAll(uint8_t red, uint8_t green, uint8_t blue){
    for(int i=0; i<8; i++) {
        strip.setPixelColor(i, red, green, blue);
    }
    strip.show();
}


const int MPUArm = 0x69;
MPU6050 mpu(MPUArm);


void setup_MPU_AddrPins() {
  pinMode(5, OUTPUT);
  digitalWrite(5, HIGH); // MPU2 = 0x69 (Handgelenk)
}

void setup_MPU(const int addr) {
    mpu.initialize();
    if(!mpu.testConnection()) {
        Serial.println("Failed to connect to mpu");
        while(1){
            LedRingSetAll(0, 0, 0);
            delay(100);
            LedRingSetAll(255, 0, 0);
            delay(100);
        }
    }
}

void setup(){  
    strip.begin();
    LedRingSetAll(255, 0, 0);    
    Serial.begin(115200);
    setup_MPU_AddrPins();
    setup_MPU(MPUArm);

    delay(1000);
    LedRingSetAll(0, 255, 0);    
}
struct SFloatRawData {unsigned long t; double ax, ay, az, gx, gy, gz;};

void printData(struct SFloatRawData fdata) {
    Serial.print("D.gr.txyz:");
    Serial.print(fdata.t); Serial.print(",");
    Serial.print(fdata.gx); Serial.print(",");
    Serial.print(fdata.gy); Serial.print(",");
    Serial.print(fdata.gz);
    Serial.println("");
    Serial.print("D.a.txyz:");
    Serial.print(fdata.t); Serial.print(",");
    Serial.print(fdata.ax); Serial.print(",");
    Serial.print(fdata.ay); Serial.print(",");
    Serial.print(fdata.az);
    Serial.println("");
}    

void printCSVData(struct SFloatRawData fdata) {
    static bool printHeader = true;
    if (printHeader) {
        Serial.println("time,GyroX,GyroY,GyroZ,AccX,AccY,AccZ");
        printHeader = false;
    }
    Serial.print(fdata.t);  Serial.print(",");
    Serial.print(fdata.gx); Serial.print(",");
    Serial.print(fdata.gy); Serial.print(",");
    Serial.print(fdata.gz); Serial.print(",");
    Serial.print(fdata.ax); Serial.print(",");
    Serial.print(fdata.ay); Serial.print(",");
    Serial.print(fdata.az); Serial.print("\n");
}

void loop(){
    int16_t ax, ay, az, gx, gy, gz;
    struct SFloatRawData fdata;

    
    mpu.getMotion6(&ax,&ay,&az,&gx,&gy,&gz);

    fdata.t = millis();
    fdata.ax = ax/16384.; fdata.ay = ay/16384.; fdata.az = az/16384.; 
    fdata.gx = gx/131.; fdata.gy = gy/131.; fdata.gz = gz/131.; 
    printCSVData(fdata);
    
    
    LedRingSetAll(0, 200 + (int) (50*sin(2*3.14*3*fdata.t/1000.)), 0);    

    delay(30);
}
