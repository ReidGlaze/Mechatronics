
#include <Wire.h>
#include "Adafruit_TCS34725.h"
byte multiAddress = 0x70;
byte count = 0;
int currentTime = 0;
char data[2][3] = {{'r', 'g', 'b'},
                 {'r', 'g', 'b'}
};
Adafruit_TCS34725 tcs[] =
{Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_1X),
Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_1X)
int Left=0;
int Right=0;
class piComms{
};
public:
piComms(int balloonModePin, int fPin, int bPin, int lPin, int rPin){
  this->bModePin = balloonModePin;
  this->fPin = fPin;
  this->bPin = bPin;
  this->lPin = lPin;
  this->rPin = rPin;
  pinMode(bModePin, INPUT);
  pinMode(fPin, INPUT);
  pinMode(bPin, INPUT);
  pinMode(lPin, INPUT);
  pinMode(rPin, INPUT);
}
13
   void refresh(){
    values[0] = digitalRead(bModePin);
    values[1] = digitalRead(fPin);
    values[2] = digitalRead(bPin);
    values[3] = digitalRead(lPin);
    values[4] = digitalRead(rPin);
  }
  public:
  int values[5] = {0,0,0,0,0};
  private:
  int bModePin;
  int fPin;
  int bPin;
  int lPin;
  int rPin;
};
class sparkFunMDriver{
  public:
  sparkFunMDriver(int ai1,
                  int ai2,
                  int pwmA,
                  int stby,
                  int bi1,
                  int bi2,
                  int pwmB){
    this->ai1 = ai1;
    this->ai2 = ai2;
    this->pwmA = pwmA;
    this->stby = stby;
    this->bi1 = bi1;
    this->bi2 = bi2;
    this->pwmB = pwmB;
    pinMode(ai1, OUTPUT);
    pinMode(ai2, OUTPUT);
    pinMode(pwmA, OUTPUT);
    pinMode(stby, OUTPUT);
    pinMode(bi1, OUTPUT);
    pinMode(bi2, OUTPUT);
    pinMode(pwmB, OUTPUT);
    digitalWrite(stby, HIGH);
  }
  void setDirs(int dir1, int dir2){
    if(dir1 == 1){ //CCW
      digitalWrite(ai1,LOW);
      digitalWrite(ai2,HIGH);
    } else if (dir1 == 0){ //CW
      digitalWrite(ai1,HIGH);
      digitalWrite(ai2,LOW);
    }
    if(dir2 == 1){ //CCW
      digitalWrite(bi1,LOW);
      digitalWrite(bi2,HIGH);
    } else if (dir2 == 0){ //CW
      digitalWrite(bi1,HIGH);
14

       digitalWrite(bi2,LOW);
    }
  }
  void setSpeeds(int spd1,int spd2){
    analogWrite(pwmA, spd1);
    analogWrite(pwmB, spd2);
  }
  private:
    int ai1;
    int ai2;
    int pwmA;
    int stby;
    int bi1;
    int bi2;
    int pwmB;
};
sparkFunMDriver* mDriver;
piComms* comm;
int moveSpd = 70;
int rlSpeed = 40;
void goForward(){
  mDriver->setDirs(1, 0);
  mDriver->setSpeeds(moveSpd,moveSpd);
}
void goBack(){
  mDriver->setDirs(0, 1);
  mDriver->setSpeeds(moveSpd,moveSpd);
}
void goRight(){
    mDriver->setDirs(1, 1);
  mDriver->setSpeeds(rlSpeed,rlSpeed);
}
void goLeft(){
    mDriver->setDirs(0, 0);
  mDriver->setSpeeds(rlSpeed,rlSpeed);
}
void goStop(){
  mDriver->setDirs(0, 0);
  mDriver->setSpeeds(0,0);
}
void lineFollow(int shouldntGoForward){
if (Right==0 && Left==1){
  goBack();
  delay(300);
  goRight();
  delay(500);
} else if (Right==1 && Left==0){
  goBack();
  delay(300);
15

 goLeft();
  delay(500);
} else if (Left==1 && Right==1){
  goBack();
  delay(700);
  goRight();
  delay(700);
} else{
  if (shouldntGoForward==0){
    goForward();
  }
}
//delay(300);
}
void balloonTrack(piComms* comm){
  comm->refresh();
  if (comm->values[1]==1){
    goForward();
  } else if (comm->values[2]==1){
    goBack();
  } else if (comm->values[3]==1){
    goRight();
  } else if (comm->values[4]==1){
    goLeft();
  } else {
} }
int readColors(byte sensorNum){
    chooseBus(sensorNum);
    uint16_t r, g, b, c;
    tcs[sensorNum].getRawData(&r, &g, &b, &c); // reading the rgb values
16bits at a time from the i2c channel
    processColors(r, g, b, c); // processing by dividng by clear value and
then multiplying by 256
delay(30);
    float ratio = float(g)/float(r);
   if((ratio > 0.64) && (ratio < 0.86))
   {
    return 1;
   } else {
return 0; }
}
void setup() {
      Serial.begin(9600);
    Wire.begin();
    initColorSensors();
  mDriver = new sparkFunMDriver(11,
                                10,
9, 13, 4,
16

5, 3);
comm = new piComms(12, 8, 7, 6, 2);
}
void loop() {
  Right=readColors(0);
  Left=readColors(1);
  comm->refresh();
  int shouldTrack = comm->values[0];
  lineFollow(shouldTrack);
  if (comm->values[0] == 1){
      balloonTrack(comm);
   }
}
void changeLED(){
    if(millis() - currentTime > 200){
        count++;
        currentTime = millis();
    }
    if(count > 2){
count = 0; }
}
void initColorSensors(){
    for(int i = 0; i < 2; i++){
        Serial.println(i);
        chooseBus(i);
        if (tcs[i].begin()){
            Serial.print("Found sensor "); Serial.println(i+1);
        } else{
            Serial.println("No Sensor Found");
            while (true);
        }
} }
void processColors(uint16_t r, uint16_t g, uint16_t b, uint32_t c){
        // getting rid of IR component of light
       r /= c;
       g /= c;
       b /= c;
       r *= 256;
       g *= 256;
       b *= 256;
void chooseBus(uint8_t bus){
    Wire.beginTransmission(0x70);
    Wire.write(1 << (bus+2)); // will be using 2-7 instead of 0-5 because
of convience (placed better on the breadboard)
    Wire.endTransmission();
}