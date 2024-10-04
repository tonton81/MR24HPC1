#include "Arduino.h"
#include <MR24HPC1_RADAR.h>

MR24HPC1 radar = MR24HPC1(&Serial1);


void setup() {
  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, 16, 17); // RX/TX
  bool custom = true;
  delay(1000);
  radar.module_reset();
  Serial.println("Ready");
  delay(1000);

  if ( custom == false ) {
    radar.Standard(
      RADAR_BATHROOM, // LIVINGROOM(4m), BEDROOM(3.5m), BATHROOM(2.5m), AREA_DETECTION(3m)
      RADAR_SENSITIVITY_LEVEL1, // 1 ,2 ,3
      RADAR_UNMANNED_10s // None, 10s, 30s, 1min, 2min, 5min, 10min, 30min, 60min
    );
    radar.onReceive(RadarCB);

    Serial.println(radar.scene_settings_inquiry());
    Serial.println(radar.sensitivity_settings_inquiry());
    Serial.println(radar.time_for_entering_no_person_state_inquiry());
    Serial.println("--------------------------------------------");
  }
  else {
    radar.Custom(
      1, // Choose Custom mode (1~4) (Default 1)
      33, // Existance Judgement Threshold (0~250) (Default 33)
      4, // Motion Trigger Threshold (0~250) (Default 4)
      5, // Motion Trigger Boundary (0.5m~5m, 0.5m increments) (Default 5m)
      150, // Motion Trigger Time (0~1000ms) (Default 150ms)
      3000, // Motion To Still Time (0~60000ms) (Default 3000ms)
      5, // Existance Perception Boundary (0.5m~5m, 0.5m increments) (Default 5m)
      1000 // Time for unmanned state (0~3600000ms) (Default 30000ms)
    );
    radar.onReceive(RadarCB);
    Serial.print("CURRENT CUSTOM MODE: ");
    Serial.println(radar.custom_mode_query());
  }

  /* CUSTOM EXAMPLE
    Bathroom:
    Existence judgment threshold: 33 (Default) 0~250
    Motion trigger threshold: 4 (Default) 0~250
    Existence perception boundary: 2.5m 0.5m~5m
    Motion Trigger Boundary: 2.5m 0.5m~5m
    Time for entering no person state: 30s 0~3600s
  */
}

void RadarCB(const RADAR_message_t &msg) {
  if (strcmp(msg.mode.info, "Standard") == 0) {
    Serial.print("Mode: "); Serial.print(msg.mode.info);
    Serial.print(" Heartbeat: "); Serial.print(msg.heartbeat);
    Serial.print(" Movement: "); Serial.print(msg.movement.info);
    Serial.print(" "); Serial.print(msg.movement.value);
    Serial.print(" Presence: "); Serial.print(msg.presence.info);
    Serial.print(" "); Serial.print(msg.presence.value);
    Serial.print(" Motion: "); Serial.print(msg.motion.info);
    Serial.print(" "); Serial.print(msg.motion.value);
    Serial.print(" Proximity: "); Serial.print(msg.proximity.info);
    Serial.print(" "); Serial.print(msg.proximity.value);
    Serial.print(" Occupation Time: "); Serial.print(msg.duration); Serial.print(" ");
    Serial.println();
  }
  else { // Custom
    Serial.print("Mode: "); Serial.print(msg.mode.info);
    Serial.print(" Movement: "); Serial.print(msg.movement.info);
    Serial.print(" "); Serial.print(msg.movement.value);
    Serial.print(" Presence: "); Serial.print(msg.presence.info);
    Serial.print(" "); Serial.print(msg.presence.value);
    Serial.print(" Proximity: "); Serial.print(msg.proximity.info);
    Serial.print(" "); Serial.print(msg.proximity.value);
    Serial.print(" Existence Energy: "); Serial.print(msg.custom.existanceEnergy); Serial.print(" ");
    Serial.print("\tStatic Distance: "); Serial.print(msg.custom.staticDistance); Serial.print(" ");
    Serial.print("\tMotion Energy: "); Serial.print(msg.custom.motionEnergy); Serial.print(" ");
    Serial.print("\tMotion Distance: "); Serial.print(msg.custom.motionDistance); Serial.print(" ");
    Serial.print("\tMotion Speed: "); Serial.print(msg.custom.motionSpeed); Serial.print(" ");
    Serial.print("\tOccupation Time: "); Serial.print(msg.duration); Serial.print(" ");


    Serial.println();
  }
}


void loop() {
  radar.events();
}
