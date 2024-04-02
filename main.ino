#include <Arduino.h>
#include <AFMotor.h>
// #include <Servo.h>
#include <NewPing.h>
#include <FastLED.h>

#include <TinyGPS++.h>
#include <SoftwareSerial.h>
//#include <DFRobot_QMC5883.h>
#include <QMC5883L.h>
#include <Wire.h>
// #include <RH_ASK.h>  // Radiohead library for RF modules
// #include <SPI.h>
QMC5883L compass;
// RH_ASK driver(2000, A2, 3);
/*
VCC  O ---- O +5v
GND  O ---- O GND
SCL  O ---- O A5
SDA  O ---- O A4
DRDY O ---- X NOT CONNECTED
*/
#define LED_PIN A3  // Digital/Analog pin connected to the RGB strip
#define PING_PIN A2
#define NUM_LEDS 8      // Number of LEDs in your strip
#define BRIGHTNESS 255  // Adjust the brightness (0 to 255)
AF_DCMotor motorL(2);   // Left motora
AF_DCMotor motorR(3);   // Right motor
TinyGPSPlus gps;
CRGB leds[NUM_LEDS];
SoftwareSerial mygps(A0, A1);  // GPS Tx Pin - D4  GPS Rx Pin D3

// Servo cap;              // Head servo
// DFRobot_QMC5883 compass;
bool ok = 0;
int oldneed = 0;
int y = 50;  //delay
int x = 7;
int angle = 90;
int distance_tolerance = 3;  //meters
int becrosu = 11;
int becgalben = 12;
int becverde = 13;
int calibration_angle = 0;
int pnt = 0;
float pointx[5] = {  };
float pointy[5] = {};
int size_of_coordinates = sizeof(pointx) / sizeof(int);

constexpr double EARTH_RADIUS = 6371000;  // Radius of the Earth in meters

double toRadians(double degrees) {
  return degrees * PI / 180.0;
}

double calculateDistance(double lat1, double lon1, double lat2, double lon2) {
  // Convert latitude and longitude from degrees to radians
  lat1 = toRadians(lat1);
  lon1 = toRadians(lon1);
  lat2 = toRadians(lat2);
  lon2 = toRadians(lon2);

  // Calculate differences in latitude and longitude
  double dlat = lat2 - lat1;
  double dlon = lon2 - lon1;

  // Haversine formula to calculate distance
  double a = sin(dlat / 2) * sin(dlat / 2) + cos(lat1) * cos(lat2) * sin(dlon / 2) * sin(dlon / 2);
  double c = 2 * atan2(sqrt(a), sqrt(1 - a));
  double distance = EARTH_RADIUS * c;

  return distance;
}
double calculateHeading(double x, double y, double x1, double y1) {
  // Calculate the difference in x and y coordinates
  double dx = x1 - x;
  double dy = y1 - y;

  // Calculate the angle in radians using atan2
  double angle = atan2(dy, dx);

  // Convert radians to degrees
  double heading = angle * 180.0 / M_PI;

  // Adjust the heading to be between 0 and 360 degrees
  //uncomment line for non-gps
  //heading = 90.0 - heading;


  if (heading < 0) {
    heading += 360.0;
  }

  return heading;
}
void galben() {
  digitalWrite(becrosu, LOW);
  digitalWrite(becgalben, HIGH);
  digitalWrite(becverde, LOW);
}
void verde() {
  digitalWrite(becrosu, LOW);
  digitalWrite(becgalben, LOW);
  digitalWrite(becverde, HIGH);
}
void rosu() {
  digitalWrite(becrosu, HIGH);
  digitalWrite(becgalben, LOW);
  digitalWrite(becverde, LOW);
}
void fw() {
  motorL.setSpeed(200);
  motorR.setSpeed(250);
  motorL.run(FORWARD);
  motorR.run(FORWARD);
}

void bw() {
  motorL.setSpeed(250);
  motorR.setSpeed(250);
  motorL.run(BACKWARD);
  motorR.run(BACKWARD);
}
void stop() {
  motorL.run(RELEASE);
  motorR.run(RELEASE);
}

void rights() {
  motorL.setSpeed(250);
  motorR.setSpeed(150);
  motorL.run(FORWARD);
  motorR.run(FORWARD);
}
void right() {
  motorL.setSpeed(250);
  motorR.setSpeed(20);
  motorL.run(FORWARD);
  motorR.run(FORWARD);
}
void right2wheel() {
  motorL.setSpeed(250);
  motorR.setSpeed(250);
  motorL.run(FORWARD);
  motorR.run(BACKWARD);
}
void left() {
  motorL.setSpeed(20);
  motorR.setSpeed(250);
  motorL.run(FORWARD);
  motorR.run(FORWARD);
}
void lefts() {
  motorL.setSpeed(150);
  motorR.setSpeed(250);
  motorL.run(FORWARD);
  motorR.run(FORWARD);
}
void left2wheel() {
  motorL.setSpeed(250);
  motorR.setSpeed(250);
  motorL.run(BACKWARD);
  motorR.run(FORWARD);
}
void allwipe(CRGB color, int wait) {
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i] = color;
    FastLED.show();
    delay(wait);
  }
}
void colorWipe(CRGB color, int wait) {
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i] = color;
    FastLED.show();
    delay(wait);
    leds[i] = CRGB::Black;
    FastLED.show();
  }
}
long microsecondsToInches(long microseconds) {

  return microseconds / 74 / 2;
}

long microsecondsToCentimeters(long microseconds) {

  return microseconds / 29 / 2;
}

void dualcontrol() {
  //CONTROL WITH KEYBOARD or JOYSTICK
  if (Serial.available() > 0) {
    char x = Serial.read();  // Read the incoming byte

    if (x == byte('w')) {
      fw();
      delay(y);

      // colorWipe(CRGB::DarkRed, 100);
      stop();

      Serial.println("fw");
    } else if (x == 'a') {
      Serial.println("left");

      left();
      delay(y);

      stop();
    } else if (x == 's') {
      Serial.println("nw");

      bw();
      delay(y);

      stop();
    } else if (x == 'd') {
      Serial.println("right");

      right();
      delay(y);

      stop();
    }
    // else if (x == 'q') {
    //   angle += 10;
    //   cap.write(angle);
    // } else if (x == 'e') {
    //   angle -= 10;
    //   cap.write(angle);
  } else {
    stop();
  }
}
int calculate_turn_direction(double current_heading, double desired_heading, double error_tolerance) {
  double angular_difference = desired_heading - current_heading;

  // Normalize the angular difference to the range [-180, 180] degrees
  if (angular_difference > 180) {
    angular_difference -= 360;
  } else if (angular_difference < -180) {
    angular_difference += 360;
  }

  // Check if the angular difference is within the error tolerance
  if (abs(angular_difference) <= error_tolerance) {
    return 0;
  }

  // Determine the turn direction
  if (angular_difference > 0) {
    if (angular_difference > 120) {
      return 2;
    } else {
      return 1;
    }
  } else {
    if (abs(angular_difference) > 120) {
      return -2;
    } else {
      return -1;
    }
  }
}
void dip() {
  double courseToDestination = TinyGPSPlus::courseTo(gps.location.lat(), gps.location.lng(), pointx[pnt], pointy[pnt]);
  const char *directionToDestination = TinyGPSPlus::cardinal(courseToDestination);
  int courseChangeNeeded = (int)(360 + courseToDestination - gps.course.deg()) % 360;
  if (courseChangeNeeded >= 345 || courseChangeNeeded < 15)
    Serial.println("Keep on straight ahead!"), allwipe(CRGB::Green, 0), fw();

  else if (courseChangeNeeded >= 315 && courseChangeNeeded < 345)

    Serial.println("Veer slightly to the left."), allwipe(CRGB::LightBlue, 0), lefts();

  else if (courseChangeNeeded >= 15 && courseChangeNeeded < 45)

    Serial.println("Veer slightly to the right."), allwipe(CRGB::Orange, 0), rights();

  else if (courseChangeNeeded >= 255 && courseChangeNeeded < 315)

    Serial.println("Turn to the left."), allwipe(CRGB::Blue, 0), lefts();

  else if (courseChangeNeeded >= 45 && courseChangeNeeded < 105)

    Serial.println("Turn to the right."), allwipe(CRGB::Red, 0), rights();

  else

    Serial.println("Turn completely around."), allwipe(CRGB::White, 0), rights();
}
void ultrasonic_distance() {
  long duration, inches, cm;

  pinMode(PING_PIN, OUTPUT);
  digitalWrite(PING_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(PING_PIN, HIGH);
  delayMicroseconds(5);
  digitalWrite(PING_PIN, LOW);
  pinMode(PING_PIN, INPUT);
  duration = pulseIn(PING_PIN, HIGH);
  inches = microsecondsToInches(duration);
  cm = microsecondsToCentimeters(duration);
  Serial.print(duration);
  Serial.print(" ");
  Serial.print(cm);
  Serial.print("cm");
  Serial.println();
  delay(700);
}
void setup() {

  Serial.begin(9600);
  mygps.begin(9600);
  FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);
  motorL.setSpeed(250);
  motorR.setSpeed(250);
  motorL.run(RELEASE);
  motorR.run(RELEASE);
  for (int i = 0; i <= 8; i++) {
    leds[i] = CRGB::DarkBlue;
    FastLED.show();
  }
  Wire.begin();
  compass.init();
  	compass.setSamplingRate(50);
  Serial.println("CALIBRATING, TURN THE ROBOT AROUND");
  right2wheel();
  for(int i = 1; i<= 50; i++){
    int heading = compass.readHeading();
    Serial.print(" Heading for cali = ");
    Serial.println(heading);
    delay(i * 10);
  }
  stop();
  Serial.println("POINT THE ROBOT TO NORTH");
  delay(20000);

  
  int heading = compass.readHeading();
  delay(1000);
  if(heading <= 180)calibration_angle = (-heading);
  else calibration_angle = (360 - heading);
  Serial.print("Calibration angle: ");
  Serial.println(calibration_angle);
  Serial.println("The compass is done setting up");
  int i = 0, cmd = 0;
  while (!mygps.available() > 0) {
    Serial.println("sarching for signal ...");
  }
    Serial.println("Signal almost found");

  fw();
  delay(100);
  stop();
  allwipe(CRGB::DarkRed, 0);
}

void loop() {
  if (mygps.available() > 0) {
    gps.encode(mygps.read());

    if (gps.location.isUpdated()) {
      ok = 1;


      float lat = gps.location.lat();
      float lng = gps.location.lng();
      Serial.print("Latitude= ");
      Serial.print(lat, 6);
      Serial.print(" Longitude= ");
      Serial.println(lng, 6);
      int dist = calculateDistance(lat, lng, pointx[pnt], pointy[pnt]);
      if (dist < distance_tolerance) pnt++;
      if (pnt >= size_of_coordinates) {  //go to the destination
        Serial.println("Arrived at the destination");
        pnt = 0;
      }
      int need = calculateHeading(lat, lng, pointx[pnt], pointy[pnt]);
      oldneed = need;
      Serial.print("needed heading : ");
      Serial.println(need);
    }

    // delay(100);
  }
  if (ok == 1) {

    Serial.println("Signal Found");

    int heading = compass.readHeading() + calibration_angle;
    if (heading >= 360) {
        heading = heading % 360;
    } else if (heading < 0) {
        heading = 360 + (heading % 360);
    }
    Serial.print(" Heading = ");
    Serial.println(heading);

    int x = calculate_turn_direction(heading, oldneed, 5);
    if (x == 0) fw();
    if (x == 1) rights();
    if (x == -1) lefts();
    if (x == 2) right();
    if (x == -2) left();
    delay(500);
  }
}
