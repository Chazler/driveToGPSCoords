#include <Adafruit_GPS.h>
SoftwareSerial mySerial(3, 2);
Adafruit_GPS GPS(&mySerial);

#include <Zumo32U4.h>
Zumo32U4Motors motors;
Zumo32U4ButtonA buttonA;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(4800);
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
}

void loop() {
  switch(){
    case AVOID:
    break;
    case DRIVE:
    break;
  }
}

void drive(){
  motors.setSpeeds(200,200;
}

//turn to target coord using gps.Angle to get current heading degrees
void turnToTarget(float targetLat, float targetLong){
  float currentAngle = GPS.angle;
  float targetAngle = calculateTargetDegrees(targetLat, targetLong);
  float relativeAngle = targetAngle - currentAngle;
  float turnAngle;

  //Calculate angle which zumo needs to turn to.
  if(relativeAngle < -180){
    turnAngle = 360 - relativeAngle;
  }
  else{
    turnAngle = relativeAngle;
  }

  ///turn to zumo
  motors.turn(turnAngle);
}

void 

//calculates degrees from current coord to target coord
float calculateTargetDegrees(float targetLat, float targetLong){
  float currentLat = GPS.latitudeDegrees;
  float currentLong = GPS.longitudeDegrees;
  float dLong = targetLong - currentLong;

  //??
  float X = cos(targetLat) * sin(dLong);
  float Y = cos(currentLat) * sin(targetLat) - sin(currentLat) * cos(targetLat) * cos(dLong);

  float targetRadians = atan2(X,Y);
  float targetDegrees = toDeg(radian);

  return targetDrees;
}

float toRad(float convert){
  return convert * PI/180;
}

float toDeg(float convert){
  return convert * 180/PI;
}
