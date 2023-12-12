#include "src/c_library_v2/ardupilotmega/mavlink.h"
#include <math.h>

#define PI 3.14159265358979323846
#define EARTH_RADIUS 6372797.56085
#define RADIANS PI / 180

//SET ANTENNA TRACKER LOCATION
#define ATT_LAT //LAT
#define ATT_LON //LON

float vehicle_lat;
float vehicle_lon;
float vehicle_alt;
float dist;
float bear;
int inc = 0;
mavlink_message_t msg;
mavlink_status_t status;
double haversine;
double temp;
double point_dist;

void setup() {
  Serial.begin(115200);
  Serial1.begin(57600); 
}

double toRadians(double degrees) {
  return degrees * (RADIANS);
}

double toDegrees(double radians) {
  return radians * (pow(RADIANS, -1));
}

void readPos() {

  while (Serial1.available() > 0) {
    uint8_t c = Serial1.read();

    if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
      switch(msg.msgid)
      {
 
        case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: {
      mavlink_global_position_int_t position;
      mavlink_msg_global_position_int_decode(&msg, &position);

      vehicle_lat = position.lat / 1e7;
      vehicle_lon = position.lon / 1e7;
      vehicle_alt = position.relative_alt / 1e3;

      break;
    }
    }
    }
}
}

//https://stackoverflow.com/questions/27126714/c-latitude-and-longitude-distance-calculator
float calcGPSDist(float lat1, float lon1, float lat2, float lon2) {
  lat1  = lat1  * RADIANS;
  lon1 = lon1 * RADIANS;
  lat2  = lat2  * RADIANS;
  lon2 = lon2 * RADIANS;
  haversine = (pow(sin((1.0 / 2) * (lat2 - lat1)), 2)) + ((cos(lat1)) * (cos(lat2)) * (pow(sin((1.0 / 2) * (lon2 - lon1)), 2)));
  temp = 2 * asin(min(1.0, sqrt(haversine)));
  point_dist = EARTH_RADIUS * temp;
  return point_dist;
}

//https://stackoverflow.com/questions/21060891/android-how-can-i-get-the-bearing-degree-between-two-locations
double calculateBearing(double baseLat, double baseLon, double droneLat, double droneLon) {
  double startLat = toRadians(baseLat);
  double startLon = toRadians(baseLon);
  double endLat = toRadians(droneLat);
  double endLon = toRadians(droneLon);

  double dLon = endLon - startLon;

  double x = sin(dLon) * cos(endLat);
  double y = cos(startLat) * sin(endLat) - sin(startLat) * cos(endLat) * cos(dLon);
  double initialBearing = atan2(x, y);

  // Convert bearing from radians to degrees
  double bearing = toDegrees(initialBearing);

  // Normalize the bearing
  bearing = fmod((bearing + 360), 360);

  return bearing;
}

void loop() {
  readPos();
  dist = calcGPSDist(ATT_LAT, ATT_LON, vehicle_lat, vehicle_lon);
  bear = calculateBearing(ATT_LAT, ATT_LON, vehicle_lat, vehicle_lon);
  Serial.print("Latitude: ");
  Serial.println(vehicle_lat, 6);
  Serial.print("Longitude: ");
  Serial.println(vehicle_lon, 6);
  Serial.print("Altitude: ");
  Serial.println(vehicle_alt, 2);
  Serial.print("Dist: ");
  Serial.println(dist, 2);
  Serial.print("Bearing: ");
  Serial.println(bear, 2);
  Serial.print("Incremental: ");
  Serial.println(inc);
  inc = inc+1;
  Serial.print("\n");
  delay(200);
}