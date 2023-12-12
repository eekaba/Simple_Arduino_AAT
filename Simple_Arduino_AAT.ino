#include "src/c_library_v2/ardupilotmega/mavlink.h"

void setup() {
  Serial.begin(115200);
  Serial1.begin(57600); 
}
void loop() {
  mavlink_message_t msg;
  mavlink_status_t status;

  while (Serial1.available() > 0) {
    uint8_t c = Serial1.read();

    if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
      switch(msg.msgid)
      {
 
        case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: {
      mavlink_global_position_int_t position;
      mavlink_msg_global_position_int_decode(&msg, &position);

      float lat = position.lat / 1e7;
      float lon = position.lon / 1e7;
      float alt = position.relative_alt / 1e3;

      Serial.print("Latitude: ");
      Serial.println(lat, 6);
      Serial.print("Longitude: ");
      Serial.println(lon, 6);
      Serial.print("Altitude: ");
      Serial.println(alt, 2);
      Serial.print("\n");

      break;
    }
    }
    }
}
}