//GPS
#include <Adafruit_GPS.h>
SoftwareSerial mySerial(3, 2);
Adafruit_GPS GPS(&mySerial);

//Zumo
#include <ZumoShield.h>
ZumoMotors motors;
#define SPEED           200
#define TURN_BASE_SPEED 100

//UltraSonic
#include <NewPing.h>
#define maximum_distance 200
NewPing sonarLeft(3,2, maximum_distance);
NewPing sonarRight(4,5, maximum_distance);

enum action {AVOID, DRIVE, TURN};
float targetLatitude;
float targetLongitude;

//Compass
#include <LSM303.h>
#define CALIBRATION_SAMPLES 70
#define DEVIATION_THRESHOLD 5
LSM303::vector<int16_t> running_min = {32767, 32767, 32767}, running_max = {-32767, -32767, -32767};

#define CRB_REG_M_2_5GAUSS 0x60 // CRB_REG_M value for magnetometer +/-2.5 gauss full scale
#define CRA_REG_M_220HZ    0x1C // CRA_REG_M value for magnetometer 220 Hz update rate5
LSM303 compass;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(4800);
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);

  compass.init();

  // Enables accelerometer and magnetometer
  compass.enableDefault();

  compass.writeReg(LSM303::CRB_REG_M, CRB_REG_M_2_5GAUSS); // +/- 2.5 gauss sensitivity to hopefully avoid overflow problems
  compass.writeReg(LSM303::CRA_REG_M, CRA_REG_M_220HZ);
  Serial.println("Calibrating compass");
  
  calibrateCompass();
  delay(1000);
  Serial.println("Waiting for satellite fix");
  while(GPS.fixquality == 0){
    Serial.println("# of connected satellites = ");
    Serial.print(GPS.satellites);
    delay(100);
  }
}

void loop() {
  switch(decideAction()){
    case AVOID:
      if (sonarLeft.ping_cm() <15){
        Serial.println("Avoid to the left");
        motors.setSpeeds(-200,-200);
        delay(200);
        motors.setSpeeds(-100,200);
        delay(100);
        break;
      }
      else if (sonarRight.ping_cm() < 15) {
        Serial.println("Avoid the right");
        motors.setSpeeds(-200,-200);
        delay(200);
        motors.setSpeeds(200,-100);
        delay(100);
        break;
      }

      motors.setSpeeds(-200,-200);
      delay(300);
    break;
    case DRIVE:
    drive();
    break;
    case TURN:
    Serial.println("Turning");
    turnToTarget(targetLatitude, targetLongitude);
    break;
  }
}


action decideAction(){
  //if ultrasonics detect obstacle avoid said obstacle
  if(sonarLeft.ping_cm() < 15 || sonarRight.ping_cm() < 15){
    return AVOID;
  }
  else {
    //if delta between heading and target is too much adjust course
    float dDegrees = calculateTargetDegrees(targetLatitude, targetLongitude) - GPS.angle;
    if(dDegrees <= -10 && dDegrees >= 10){
      return TURN;
    }
  }
  //if everything OK keep driving
  return DRIVE;
}

void drive(){
  motors.setSpeeds(200,200);
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
    Serial.println("Turn angle = ");
    Serial.print("turn angle");
  }
  else{
    turnAngle = relativeAngle;
    Serial.println("Turn angle = ");
    Serial.print("turn angle");
  }

  ///turn to zumo
  if(abs(turnAngle) < DEVIATION_THRESHOLD){
    int speed = SPEED*turnAngle/180;

    if (speed < 0)
      speed -= TURN_BASE_SPEED;
    else
      speed += TURN_BASE_SPEED;

    motors.setSpeeds(speed, -speed);
  }
}

//calculates degrees from current coord to target coord
float calculateTargetDegrees(float targetLat, float targetLong){
  float currentLat = GPS.latitudeDegrees;
  float currentLong = GPS.longitudeDegrees;
  float dLong = targetLong - currentLong;

  //math magic to calculate true heading
  float X = cos(targetLat) * sin(dLong);
  float Y = cos(currentLat) * sin(targetLat) - sin(currentLat) * cos(targetLat) * cos(dLong);

  float targetRadians = atan2(X,Y);
  float targetDegrees = toDeg(targetRadians);

  Serial.println("Target degrees = ");
  Serial.print(targetDegrees);
  return targetDegrees;
}

float toRad(float convert){
  return convert * PI/180;
}

float toDeg(float convert){
  return convert * 180/PI;
}


void calibrateCompass(){
  for(int index = 0; index < CALIBRATION_SAMPLES; index ++){
    // Take a reading of the magnetic vector and store it in compass.
    compass.read();

    running_min.x = min(running_min.x, compass.m.x);
    running_min.y = min(running_min.y, compass.m.y);

    running_max.x = max(running_max.x, compass.m.x);
    running_max.y = max(running_max.y, compass.m.y);

    Serial.println(index);

    delay(50);
  }
}

float averageHeading()
{
  LSM303::vector<int32_t> avg = {0, 0, 0};
  for(int i = 0; i < 10; i ++)
  {
    compass.read();
    avg.x += compass.m.x;
    avg.y += compass.m.y;
  }
  avg.x /= 10.0;
  avg.y /= 10.0;

  // avg is the average measure of the magnetic vector.
  return heading(avg);
}

template <typename T> float heading(LSM303::vector<T> v)
{
  float x_scaled =  2.0*(float)(v.x - compass.m_min.x) / ( compass.m_max.x - compass.m_min.x) - 1.0;
  float y_scaled =  2.0*(float)(v.y - compass.m_min.y) / (compass.m_max.y - compass.m_min.y) - 1.0;

  float angle = atan2(y_scaled, x_scaled)*180 / M_PI;
  if (angle < 0)
    angle += 360;
  return angle;
}
