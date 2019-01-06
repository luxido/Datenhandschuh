// I2Cdev and MPU6050 müssen als Libraries installiert sein
#include "I2Cdev.h"
#include "MPU6050_Wrapper.h"
#include "TogglePin.h"
#include "DeathTimer.h"
#include "Wire.h"

#include <Adafruit_NeoPixel.h>
Adafruit_NeoPixel strip = Adafruit_NeoPixel(8, 3, NEO_GRB + NEO_KHZ800);

MPU6050_Array mpus(2);

#define AD0_PIN_0 4  // Ring
#define AD0_PIN_1 5  // Handgelenk

#define LED_PIN 13

uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;        // [w, x, y, z]         quaternion container
VectorInt16 aa;      // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;  // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld; // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity; // [x, y, z]            gravity vector
float euler[3];      // [psi, theta, phi]    Euler angle container
float ypr[3];        // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

TogglePin activityLed(LED_PIN, 100);
DeathTimer deathTimer(5000L);

// --- LED Ring --- //

int ledValsRed[8] = {0};      // Array für rote Pixel
int ledValsGreen[8] = {0};    // Array für grüne Pixel
int ledValsBlue[8] = {0};     // Array für blaue Pixel
int fadeVal = 60;             // 0-255, 0 = immer an, 255 = kein Fade;

int yawH = 0;
int pitchR = 0;
int pitchH = 0;
int pitchDiff = 0;
int rollR = 0;
int rollH = 0;
int rollAbs = 0;

void onPixelRunColor(int x,int y,int z){
  for(int i=0; i<8; i++) {
    strip.setPixelColor(i, strip.Color(x, y, z));
    strip.show();
  }
}

void fadePixel(){
  for(int i=0; i<8; i++) {
    ledValsRed[i] = ledValsRed[i] - fadeVal;
    if (ledValsRed[i] <= 0){
      ledValsRed[i] = 0;
    }
    ledValsGreen[i] = ledValsGreen[i] - fadeVal;
    if (ledValsGreen[i] <= 0){
      ledValsGreen[i] = 0;
    }
    ledValsBlue[i] = ledValsBlue[i] - fadeVal;
    if (ledValsBlue[i] <= 0){
      ledValsBlue[i] = 0;
    }
  }
}

void applyColorToAll(int x){
  for(int i=0; i<8; i++) {
    ledValsBlue[i] = x;
  }
}

void calcPixel(int x, int y){
  ledValsRed[x] = 255;
  ledValsGreen[y] = 255;
  ledValsBlue[x] = 0;         // Blauer Pixel aus, wenn Rot an
  ledValsBlue[y] = 0;         // Blauer Pixel aus, wenn Grün an
}

void showPixel(){
  for(int i=0; i<8; i++) {
    strip.setPixelColor(i, strip.Color(ledValsRed[i], ledValsGreen[i], ledValsBlue[i]));  
  }
  strip.show();   
}

int gamma(int in){
  if(in > 255){
    in = 255;
  }else if(in < 0){
    in = 0;
  }
  float wert = (float)in / 255.0f;
  wert = pow(wert,2.35f);
  return (int)(wert*255.0f);
}

// --- SETUP --- //

void setup() {
  strip.begin();
  strip.show();

  onPixelRunColor(0,0,0);
  
  // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif

  Serial.begin(115200);

  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpus.add(AD0_PIN_0);
  mpus.add(AD0_PIN_1);

  mpus.initialize();

  // configure LED for output
  pinMode(LED_PIN, OUTPUT);

  // verify connection
  Serial.println(F("Testing device connections..."));
  if (mpus.testConnection()) {
    Serial.println(F("MPU6050 connection successful"));
  } else {
    mpus.halt(F("MPU6050 connection failed, halting"));
  }

  // wait for ready
  Serial.println(F("\nSend any character to begin DMP programming and demo: "));
  while (Serial.available() && Serial.read())
    ; // empty buffer
  while (!Serial.available())
    activityLed.update(); // flash led while waiting for data
  while (Serial.available() && Serial.read())
    ; // empty buffer again
  activityLed.setPeriod(500); // slow down led to 2Hz

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  mpus.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  MPU6050_Wrapper* currentMPU = mpus.select(0);
  // Ring
  currentMPU->_mpu.setXGyroOffset(0);
  currentMPU->_mpu.setYGyroOffset(0);
  currentMPU->_mpu.setZGyroOffset(0);
  currentMPU->_mpu.setXAccelOffset(0);
  currentMPU->_mpu.setYAccelOffset(0);
  currentMPU->_mpu.setZAccelOffset(0);
  // Handgelenk
  currentMPU = mpus.select(1);
  currentMPU->_mpu.setXGyroOffset(0);
  currentMPU->_mpu.setYGyroOffset(0);
  currentMPU->_mpu.setZGyroOffset(0);
  currentMPU->_mpu.setXAccelOffset(0);
  currentMPU->_mpu.setYAccelOffset(0);
  currentMPU->_mpu.setZAccelOffset(0);
  
  mpus.programDmp(0);+
  mpus.programDmp(1);
}

void handleMPUevent(uint8_t mpu) {

  MPU6050_Wrapper* currentMPU = mpus.select(mpu);
  // reset interrupt flag and get INT_STATUS byte
  currentMPU->getIntStatus();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((currentMPU->_mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT))
      || currentMPU->_fifoCount >= 1024) {
    // reset so we can continue cleanly
    currentMPU->resetFIFO();
    Serial.println(F("FIFO overflow!"));
    return;
  }
  // otherwise, check for DMP data ready interrupt (this should happen frequently)
  if (currentMPU->_mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) {

    // read and dump a packet if the queue contains more than one
    while (currentMPU->_fifoCount >= 2 * currentMPU->_packetSize) {
      // read and dump one sample
      //Serial.print("DUMP"); // this trace will be removed soon
      currentMPU->getFIFOBytes(fifoBuffer);
    }

    // read a packet from FIFO
    currentMPU->getFIFOBytes(fifoBuffer);
    
    // display Euler angles in degrees
    currentMPU->_mpu.dmpGetQuaternion(&q, fifoBuffer);
    currentMPU->_mpu.dmpGetGravity(&gravity, &q);
    currentMPU->_mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    // Anzeige der Werte
    if (mpu == 0){
      Serial.print("Ring:");
      pitchR = ypr[1] * 180 / M_PI;
      rollR = ypr[2] * 180 / M_PI;
    }else{
      Serial.print("Handgelenk:");
      yawH = ypr[0] * 180 / M_PI;
      pitchH = ypr[1] * 180 / M_PI;
      rollH = ypr[2] * 180 / M_PI;
    }
    
    Serial.print("\t");
    Serial.print(ypr[0] * 180 / M_PI);
    Serial.print("\t");
    Serial.print(ypr[1] * 180 / M_PI);
    Serial.print("\t");
    Serial.print(ypr[2] * 180 / M_PI);
    Serial.print("\t");
  }
}

// --- Hauptprogramm --- //

void loop() {

  static uint8_t mpu = 0;
  static MPU6050_Wrapper* currentMPU = NULL;
  for (int i=0;i<2;i++) {
    mpu=(mpu+1)%2; // failed attempt at round robin
    currentMPU = mpus.select(mpu);
    if (currentMPU->isDue()) {
      handleMPUevent(mpu);
    }
  }
  
  pitchDiff = abs(pitchR - pitchH);
  rollAbs = (rollR + rollH) / 2;
  if (rollAbs < -45){
    rollAbs = -45;
  }else if (rollAbs > 45){
    rollAbs = 45;
  }

  fadePixel();
  applyColorToAll(gamma(map(pitchDiff, 0, 90, 0, 255)));                       // Alle Pixel blau je nach Fingerkrümmung (Differenz Pitch der beiden Sensoren)
  calcPixel(map(rollAbs, -45, 45, 1, 7), map(yawH, -180, 180, 0, 7));   // Roter Pixel je nach Rollposition, Grüner Pixel je nach Rotation
  showPixel();

  /* Serial.print("Pitch Difference: ");
   * Serial.print(pitchDiff); */
   
  Serial.println("");
  activityLed.update();
  deathTimer.update();
}

