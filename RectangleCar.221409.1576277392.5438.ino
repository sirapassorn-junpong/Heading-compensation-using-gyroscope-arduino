#include "I2Cdev.h"
#include "MPU6050.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 accelgyro;

int16_t ax, ay, az;
int16_t gx, gy, gz;
double theta;
double angle;
int16_t dt;
unsigned long t1,t2;
//boolean first = true;
double offset;

//dc motor_left
int enA = 11;
int in1 = 10;
int in2 = 9;
//dc motor_right
int in3 = 8;
int in4 = 7;
int enB = 6;
//motor speed
int motorSpeed1 = 0; //left
int motorSpeed2 = 0; //right
int speed1=0;

int targetAngle;
int count;

void setup() {
    Serial.begin(38400);
    
    //gyroscope
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
    
    // initialize device
    Serial.println("Initializing I2C devices...");
    accelgyro.initialize();
    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

    offset =0.0;
    for (int i=0; i<1000; i++){
      accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
      offset = offset + gz;
      delay(10);
    }
    offset  = offset / 1000.0;
 
    //motor
    pinMode(enA, OUTPUT);
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
    pinMode(in3, OUTPUT);
    pinMode(in4, OUTPUT);
    pinMode(enB, OUTPUT);
    
    digitalWrite(enA, LOW);
    digitalWrite(enB, LOW);
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
    
    delay(1000);
    digitalWrite(enA, HIGH);
    digitalWrite(enB, HIGH);
    
    speed1=90;
    theta=0;
    targetAngle=0;
    count=0;
}

void loop() {
    // read raw accel/gyro measurements from device
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    theta = theta + (((double)gz-offset)/32678.0)*360.0*dt/1000.0;
    //Serial.println(theta);
    t2 = millis();
    dt = t2-t1;
    t1 = millis();

    angle=theta;

    //Serial.println(angle);
    //Serial.println(targetAngle-angle);
    //delay(20);

    if(accelgyro.testConnection()==true) {
      motorSpeed1=speed1-(targetAngle-angle);
      motorSpeed2=speed1+(targetAngle-angle);

      motorSpeed1=clamp(motorSpeed1, 0, 200);
      motorSpeed2=clamp(motorSpeed2, 0, 200);
//      Serial.print(motorSpeed1);
//      Serial.print("   ");
//      Serial.println(motorSpeed2);
    
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
      digitalWrite(in3, LOW);
      digitalWrite(in4, HIGH);
      analogWrite(enA, motorSpeed1);
      analogWrite(enB, motorSpeed2);
      
//      if(targetAngle-angle<1.0 && targetAngle-angle>-1.0) {
//        count++;
//        Serial.print("Count: ");
//        Serial.println(count);
//      }

      count++;
      if(count>=1000) { 
        count=0;
        brake();
        delay(1000);
        theta=0;
        targetAngle=135; //use 135 because the Accerelometer doesn't do an accurate 90degree turn
      }
    } else {
      brake();
    }
}

void brake() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}
int clamp(int val, int min, int max) {
  if(val<=min) {
    return val=min;
  } else if(val>=max) {
    return val=max;
  } else {
    return val;
  }
}




