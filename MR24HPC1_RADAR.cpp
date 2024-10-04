#include "MR24HPC1_RADAR.h"



void MR24HPC1::onReceive(RADARCB_ptr handler) {
	_handler = handler;
}

void MR24HPC1::processing() {
  if ( !cbData.presence.value ) {
    _time_occupied = millis();
  }
  cbData.duration = millis() - _time_occupied;
}

void MR24HPC1::Standard(RADAR_SCENE_MODE scene, RADAR_SENSITIVITY_LEVEL sens, RADAR_UNMANNED_TIME t) {
  underlying_open_function_information_output_switch(false);
  scene_settings(scene);
  sensitivity_settings(sens);
  time_for_entering_no_person_state_setting(t);
}

void MR24HPC1::Custom(uint8_t mode, uint8_t existT, uint8_t motT, float mTB, uint32_t motTT, uint32_t motST, float ePB, uint32_t unmanT) {
  mTB = round(constrain(mTB, 0.5, 5) * 2) / 2;
  ePB = round(constrain(ePB, 0.5, 5) * 2) / 2;
  underlying_open_function_information_output_switch(true);
  custom_mode_setting(mode);
  existence_judgment_threshold_settings(existT);
  motion_trigger_threshold_settings(motT);
  motion_trigger_boundary_setting(mTB);
  motion_trigger_time_setting(motTT);
  motion_to_still_time_setting(motST);
  existence_perception_boundary_settings(ePB);
  time_for_entering_no_person_state_setting(unmanT);
  end_of_custom_mode_settings();
}


String MR24HPC1::motion_trigger_time_inquiry() {
  uint8_t arr[] = {0x53, 0x59, 0x8, 0x8C, 0x00, 0x01, 0xF, 0xFF, 0x54, 0x43};
  checksum_MR24HPC1(arr, sizeof(arr), true);

  uint32_t result = -1;
  uint32_t t = millis();
  bool once = true;
  while (result == -1) {
    if ( once && millis() - t > 500 ) {
      once = false;
      checksum_MR24HPC1(arr, sizeof(arr), true); // 2nd attempt
    }
    else if ( millis() - t > 2000 ) return "TIMEOUT";
    result = read_sensor((arr[2] << 24) | (arr[3] << 16) | (arr[4] << 8) | arr[5]);
  }
  return String(result) + "s.";
}

String MR24HPC1::motion_to_still_time_inquiry() {
  uint8_t arr[] = {0x53, 0x59, 0x8, 0x8D, 0x00, 0x01, 0xF, 0xFF, 0x54, 0x43};
  checksum_MR24HPC1(arr, sizeof(arr), true);

  uint32_t result = -1;
  uint32_t t = millis();
  bool once = true;
  while (result == -1) {
    if ( once && millis() - t > 500 ) {
      once = false;
      checksum_MR24HPC1(arr, sizeof(arr), true); // 2nd attempt
    }
    else if ( millis() - t > 2000 ) return "TIMEOUT";
    result = read_sensor((arr[2] << 24) | (arr[3] << 16) | (arr[4] << 8) | arr[5]);
  }
  return String(result) + "s.";
}


String MR24HPC1::module_reset() {
  uint8_t arr[] = {0x53, 0x59, 0x01, 0x2, 0x00, 0x01, 0x0F, 0xFF, 0x54, 0x43};
  checksum_MR24HPC1(arr, sizeof(arr), true);

  uint32_t result = -1;
  uint32_t t = millis();
  bool once = true;
  while (result == -1) {
    if ( once && millis() - t > 500 ) {
      once = false;
      checksum_MR24HPC1(arr, sizeof(arr), true); // 2nd attempt
    }
    else if ( millis() - t > 2000 ) return "FAIL";
    result = read_sensor((arr[2] << 24) | (arr[3] << 16) | (arr[4] << 8) | arr[5]);
  }
  return "SUCCESS";
}


String MR24HPC1::heartbeat_pack_query() {
  uint8_t arr[] = {0x53, 0x59, 0x01, 0x1, 0x00, 0x01, 0x0F, 0xFF, 0x54, 0x43};
  checksum_MR24HPC1(arr, sizeof(arr), true);

  uint32_t result = -1;
  uint32_t t = millis();
  bool once = true;
  while (result == -1) {
    if ( once && millis() - t > 500 ) {
      once = false;
      checksum_MR24HPC1(arr, sizeof(arr), true); // 2nd attempt
    }
    else if ( millis() - t > 2000 ) return "FAIL";
    result = read_sensor((arr[2] << 24) | (arr[3] << 16) | (arr[4] << 8) | arr[5]);
  }
  return "SUCCESS";
}


String MR24HPC1::product_model_query() {
  uint8_t arr[] = {0x53, 0x59, 0x02, 0xA1, 0x00, 0x01, 0x0F, 0xFF, 0x54, 0x43};
  checksum_MR24HPC1(arr, sizeof(arr), true);

  uint32_t result = -1;
  uint32_t t = millis();
  bool once = true;
  while (result == -1) {
    if ( once && millis() - t > 500 ) {
      once = false;
      checksum_MR24HPC1(arr, sizeof(arr), true); // 2nd attempt
    }
    else if ( millis() - t > 2000 ) return "TIMEOUT";
    result = read_sensor((arr[2] << 24) | (arr[3] << 16) | (arr[4] << 8) | arr[5]);
  }
  return String(response_str);
}

String MR24HPC1::product_id_query() {
  uint8_t arr[] = {0x53, 0x59, 0x02, 0xA2, 0x00, 0x01, 0x0F, 0xFF, 0x54, 0x43};
  checksum_MR24HPC1(arr, sizeof(arr), true);

  uint32_t result = -1;
  uint32_t t = millis();
  bool once = true;
  while (result == -1) {
    if ( once && millis() - t > 500 ) {
      once = false;
      checksum_MR24HPC1(arr, sizeof(arr), true); // 2nd attempt
    }
    else if ( millis() - t > 2000 ) return "TIMEOUT";
    result = read_sensor((arr[2] << 24) | (arr[3] << 16) | (arr[4] << 8) | arr[5]);
  }
  return String(response_str);
}
String MR24HPC1::hardware_model_query() {
  uint8_t arr[] = {0x53, 0x59, 0x02, 0xA3, 0x00, 0x01, 0x0F, 0xFF, 0x54, 0x43};
  checksum_MR24HPC1(arr, sizeof(arr), true);

  uint32_t result = -1;
  uint32_t t = millis();
  bool once = true;
  while (result == -1) {
    if ( once && millis() - t > 500 ) {
      once = false;
      checksum_MR24HPC1(arr, sizeof(arr), true); // 2nd attempt
    }
    else if ( millis() - t > 2000 ) return "TIMEOUT";
    result = read_sensor((arr[2] << 24) | (arr[3] << 16) | (arr[4] << 8) | arr[5]);
  }
  return String(response_str);
}
String MR24HPC1::firmware_version_query() {
  uint8_t arr[] = {0x53, 0x59, 0x02, 0xA4, 0x00, 0x01, 0x0F, 0xFF, 0x54, 0x43};
  checksum_MR24HPC1(arr, sizeof(arr), true);

  uint32_t result = -1;
  uint32_t t = millis();
  bool once = true;
  while (result == -1) {
    if ( once && millis() - t > 500 ) {
      once = false;
      checksum_MR24HPC1(arr, sizeof(arr), true); // 2nd attempt
    }
    else if ( millis() - t > 2000 ) return "TIMEOUT";
    result = read_sensor((arr[2] << 24) | (arr[3] << 16) | (arr[4] << 8) | arr[5]);
  }
  return String(response_str);
}


String MR24HPC1::time_for_entering_no_person_state_inquiry() {
  //	The Time for entering no person state in the low-level open parameters is different
  // from that in the standard mode. In the lowlevel open parameters, this time value can
  // be freely set to any value (not exceeding 1 hour), but in the standard mode, only
  // specific values can be set

  if ( underlying_open_function_information_output_switch_inquiry() == "DISABLED" ) { // "ENABLED" "DISABLED"
    uint8_t arr[] = {0x53, 0x59, 0x80, 0x8A, 0x00, 0x01, 0xF, 0xFF, 0x54, 0x43};
    checksum_MR24HPC1(arr, sizeof(arr), true);

    uint32_t result = -1;
    uint32_t t = millis();
    bool once = true;
    while (result == -1) {
      if ( once && millis() - t > 500 ) {
        once = false;
        checksum_MR24HPC1(arr, sizeof(arr), true); // 2nd attempt
      }
      else if ( millis() - t > 2000 ) return "TIMEOUT";
      result = read_sensor((arr[2] << 24) | (arr[3] << 16) | (arr[4] << 8) | arr[5]);
    }
    if ( result == 0 ) return "None";
    if ( result == 1 ) return "10s";
    if ( result == 2 ) return "30s";
    if ( result == 3 ) return "1min";
    if ( result == 4 ) return "2min";
    if ( result == 5 ) return "5min";
    if ( result == 6 ) return "10min";
    if ( result == 7 ) return "30min";
    if ( result == 8 ) return "60min";
  }
  else { // CUSTOM
    uint8_t arr[] = {0x53, 0x59, 0x8, 0x8E, 0x00, 0x01, 0xF, 0xFF, 0x54, 0x43};
    checksum_MR24HPC1(arr, sizeof(arr), true);

    uint32_t result = -1;
    uint32_t t = millis();
    bool once = true;
    while (result == -1) {
      if ( once && millis() - t > 500 ) {
        once = false;
        checksum_MR24HPC1(arr, sizeof(arr), true); // 2nd attempt
      }
      else if ( millis() - t > 2000 ) return "TIMEOUT";
      result = read_sensor((arr[2] << 24) | (arr[3] << 16) | (arr[4] << 8) | 0x4);
    }
    return String(result) + "ms.";

  }
  return "";
}

int16_t MR24HPC1::existence_judgement_threshold_inquiry() {
  uint8_t arr[] = {0x53, 0x59, 0x8, 0x88, 0x00, 0x01, 0xF, 0xFF, 0x54, 0x43};
  checksum_MR24HPC1(arr, sizeof(arr), true);

  uint32_t result = -1;
  uint32_t t = millis();
  bool once = true;
  while (result == -1) {
    if ( once && millis() - t > 500 ) {
      once = false;
      checksum_MR24HPC1(arr, sizeof(arr), true); // 2nd attempt
    }
    else if ( millis() - t > 2000 ) return -1;
    result = read_sensor((arr[2] << 24) | (arr[3] << 16) | (arr[4] << 8) | arr[5]);
  }
  return result;
}


String MR24HPC1::existence_perception_boundary_inquiry() {
  uint8_t arr[] = {0x53, 0x59, 0x8, 0x8A, 0x00, 0x01, 0xF, 0xFF, 0x54, 0x43};
  checksum_MR24HPC1(arr, sizeof(arr), true);

  uint32_t result = -1;
  uint32_t t = millis();
  bool once = true;
  while (result == -1) {
    if ( once && millis() - t > 500 ) {
      once = false;
      checksum_MR24HPC1(arr, sizeof(arr), true); // 2nd attempt
    }
    else if ( millis() - t > 2000 ) return "TIMEOUT";
    result = read_sensor((arr[2] << 24) | (arr[3] << 16) | (arr[4] << 8) | arr[5]);
  }
  if ( result == 1 ) return "0.5m";
  if ( result == 2 ) return "1m";
  if ( result == 3 ) return "1.5m";
  if ( result == 4 ) return "2m";
  if ( result == 5 ) return "2.5m";
  if ( result == 6 ) return "3m";
  if ( result == 7 ) return "3.5m";
  if ( result == 8 ) return "4m";
  if ( result == 9 ) return "4.5m";
  if ( result == 10 ) return "5m";
  return "ERROR";
}


int16_t MR24HPC1::motion_trigger_threshold_inquiry() {
  uint8_t arr[] = {0x53, 0x59, 0x8, 0x89, 0x00, 0x01, 0xF, 0xFF, 0x54, 0x43};
  checksum_MR24HPC1(arr, sizeof(arr), true);

  uint32_t result = -1;
  uint32_t t = millis();
  bool once = true;
  while (result == -1) {
    if ( once && millis() - t > 500 ) {
      once = false;
      checksum_MR24HPC1(arr, sizeof(arr), true); // 2nd attempt
    }
    else if ( millis() - t > 2000 ) return -1;
    result = read_sensor((arr[2] << 24) | (arr[3] << 16) | (arr[4] << 8) | arr[5]);
  }
  return result;
}

String MR24HPC1::motion_trigger_boundary_inquiry() {
  uint8_t arr[] = {0x53, 0x59, 0x8, 0x8B, 0x00, 0x01, 0xF, 0xFF, 0x54, 0x43};
  checksum_MR24HPC1(arr, sizeof(arr), true);

  uint32_t result = -1;
  uint32_t t = millis();
  bool once = true;
  while (result == -1) {
    if ( once && millis() - t > 500 ) {
      once = false;
      checksum_MR24HPC1(arr, sizeof(arr), true); // 2nd attempt
    }
    else if ( millis() - t > 2000 ) return "TIMEOUT";
    result = read_sensor((arr[2] << 24) | (arr[3] << 16) | (arr[4] << 8) | arr[5]);
  }
  if ( result == 1 ) return "0.5m";
  if ( result == 2 ) return "1m";
  if ( result == 3 ) return "1.5m";
  if ( result == 4 ) return "2m";
  if ( result == 5 ) return "2.5m";
  if ( result == 6 ) return "3m";
  if ( result == 7 ) return "3.5m";
  if ( result == 8 ) return "4m";
  if ( result == 9 ) return "4.5m";
  if ( result == 10 ) return "5m";
  return "ERROR";
}

String MR24HPC1::sensitivity_settings_inquiry() {
  uint8_t arr[] = {0x53, 0x59, 0x05, 0x88, 0x00, 0x01, 0xF, 0xFF, 0x54, 0x43};
  checksum_MR24HPC1(arr, sizeof(arr), true);

  uint32_t result = -1;
  uint32_t t = millis();
  bool once = true;
  while (result == -1) {
    if ( once && millis() - t > 500 ) {
      once = false;
      checksum_MR24HPC1(arr, sizeof(arr), true); // 2nd attempt
    }
    else if ( millis() - t > 2000 ) return "TIMEOUT";
    result = read_sensor((arr[2] << 24) | (arr[3] << 16) | (arr[4] << 8) | arr[5]);
  }
  if ( result == 0 ) return "SENSITIVITY NOT SET (active default: Level 3)";
  if ( result == 1 ) return "SENSITIVITY LEVEL 1";
  if ( result == 2 ) return "SENSITIVITY LEVEL 2";
  if ( result == 3 ) return "SENSITIVITY LEVEL 3";
  return "ERROR";
}

String MR24HPC1::scene_settings_inquiry() {
  uint8_t arr[] = {0x53, 0x59, 0x05, 0x87, 0x00, 0x01, 0xF, 0xFF, 0x54, 0x43};
  checksum_MR24HPC1(arr, sizeof(arr), true);

  uint32_t result = -1;
  uint32_t t = millis();
  bool once = true;
  while (result == -1) {
    if ( once && millis() - t > 500 ) {
      once = false;
      checksum_MR24HPC1(arr, sizeof(arr), true); // 2nd attempt
    }
    else if ( millis() - t > 2000 ) return "TIMEOUT";
    result = read_sensor((arr[2] << 24) | (arr[3] << 16) | (arr[4] << 8) | arr[5]);
  }
  if ( result == 0 ) return "SCENE MODE NOT SET (active default: Living Room)";
  if ( result == 1 ) return "LIVINGROOM 4m";
  if ( result == 2 ) return "BEDROOM 3.5m";
  if ( result == 3 ) return "BATHROOM 2.5m";
  if ( result == 4 ) return "AREA DETECTION 3m";
  return "ERROR";
}

String MR24HPC1::initialization_status_inquiry() {
  uint8_t arr[] = {0x53, 0x59, 0x05, 0x81, 0x00, 0x01, 0xF, 0xFF, 0x54, 0x43};
  checksum_MR24HPC1(arr, sizeof(arr), true);

  uint32_t result = -1;
  uint32_t t = millis();
  bool once = true;
  while (result == -1) {
    if ( once && millis() - t > 500 ) {
      once = false;
      checksum_MR24HPC1(arr, sizeof(arr), true); // 2nd attempt
    }
    else if ( millis() - t > 2000 ) return "TIMEOUT";
    result = read_sensor((arr[2] << 24) | (arr[3] << 16) | (arr[4] << 8) | arr[5]);
  }
  if ( result == 1 ) return "COMPLETED";
  if ( result == 2 ) return "INCOMPLETE";
  return "ERROR";
}


String MR24HPC1::scene_settings(RADAR_SCENE_MODE scene) { // 1LR 4m, 2BEDR 3.5m, 3BATHR 2.5m, 4Area D 3m
  uint8_t arr[] = {0x53, 0x59, 0x05, 0x7, 0x00, 0x01, scene, 0xFF, 0x54, 0x43};
  checksum_MR24HPC1(arr, sizeof(arr), true);

  uint32_t result = -1;
  uint32_t t = millis();
  bool once = true;
  while (result == -1) {
    if ( once && millis() - t > 500 ) {
      once = false;
      checksum_MR24HPC1(arr, sizeof(arr), true); // 2nd attempt
    }
    else if ( millis() - t > 2000 ) return "TIMEOUT";
    result = read_sensor((arr[2] << 24) | (arr[3] << 16) | (arr[4] << 8) | arr[5]);
  }
  if ( result == scene ) return "SUCCESS";
  return "FAIL";
}

String MR24HPC1::sensitivity_settings(RADAR_SENSITIVITY_LEVEL sens) { // level 1,2,3
  uint8_t arr[] = {0x53, 0x59, 0x05, 0x8, 0x00, 0x01, sens, 0xFF, 0x54, 0x43};
  checksum_MR24HPC1(arr, sizeof(arr), true);

  uint32_t result = -1;
  uint32_t t = millis();
  bool once = true;
  while (result == -1) {
    if ( once && millis() - t > 500 ) {
      once = false;
      checksum_MR24HPC1(arr, sizeof(arr), true); // 2nd attempt
    }
    else if ( millis() - t > 2000 ) return "TIMEOUT";
    result = read_sensor((arr[2] << 24) | (arr[3] << 16) | (arr[4] << 8) | arr[5]);
  }
  if ( result == sens ) return "SUCCESS";
  return "FAIL";
}



int16_t MR24HPC1::body_movement_parameter_inquiry() {
  uint8_t arr[] = {0x53, 0x59, 0x80, 0x83, 0x00, 0x01, 0xF, 0xFF, 0x54, 0x43};
  checksum_MR24HPC1(arr, sizeof(arr), true);

  uint32_t result = -1;
  uint32_t t = millis();
  bool once = true;
  while (result == -1) {
    if ( once && millis() - t > 500 ) {
      once = false;
      checksum_MR24HPC1(arr, sizeof(arr), true); // 2nd attempt
    }
    else if ( millis() - t > 2000 ) return -1;
    result = read_sensor((arr[2] << 24) | (arr[3] << 16) | (arr[4] << 8) | arr[5]);
  }
  return result;
}

String MR24HPC1::motion_information_inquiry() {
  uint8_t arr[] = {0x53, 0x59, 0x80, 0x82, 0x00, 0x01, 0xF, 0xFF, 0x54, 0x43};
  checksum_MR24HPC1(arr, sizeof(arr), true);

  uint32_t result = -1;
  uint32_t t = millis();
  bool once = true;
  while (result == -1) {
    if ( once && millis() - t > 500 ) {
      once = false;
      checksum_MR24HPC1(arr, sizeof(arr), true); // 2nd attempt
    }
    else if ( millis() - t > 2000 ) return "TIMEOUT";
    result = read_sensor((arr[2] << 24) | (arr[3] << 16) | (arr[4] << 8) | arr[5]);
  }
  if ( result == 0 ) return "None";
  if ( result == 1 ) return "Motionless";
  if ( result == 2 ) return "Active";
  return "ERROR";
}

String MR24HPC1::presence_information_inquiry() {
  uint8_t arr[] = {0x53, 0x59, 0x80, 0x81, 0x00, 0x01, 0xF, 0xFF, 0x54, 0x43};
  checksum_MR24HPC1(arr, sizeof(arr), true);

  uint32_t result = -1;
  uint32_t t = millis();
  bool once = true;
  while (result == -1) {
    if ( once && millis() - t > 500 ) {
      once = false;
      checksum_MR24HPC1(arr, sizeof(arr), true); // 2nd attempt
    }
    else if ( millis() - t > 2000 ) return "TIMEOUT";
    result = read_sensor((arr[2] << 24) | (arr[3] << 16) | (arr[4] << 8) | arr[5]);
  }
  if ( result == 0 ) return "UNOCCUPIED";
  if ( result == 1 ) return "OCCUPIED";
  return "ERROR";
}


String MR24HPC1::proximity_inquiry() {
  uint8_t arr[] = {0x53, 0x59, 0x80, 0x8B, 0x00, 0x01, 0xF, 0xFF, 0x54, 0x43};
  checksum_MR24HPC1(arr, sizeof(arr), true);

  uint32_t result = -1;
  uint32_t t = millis();
  bool once = true;
  while (result == -1) {
    if ( once && millis() - t > 500 ) {
      once = false;
      checksum_MR24HPC1(arr, sizeof(arr), true); // 2nd attempt
    }
    else if ( millis() - t > 2000 ) return "TIMEOUT";
    result = read_sensor((arr[2] << 24) | (arr[3] << 16) | (arr[4] << 8) | arr[5]);
  }
  if ( result == 0 ) return "NO STATE";
  if ( result == 1 ) return "NEAR";
  if ( result == 2 ) return "FAR";
  return "ERROR";
}

String MR24HPC1::underlying_open_function_information_output_switch(bool enable) {
  uint8_t arr[] = {0x53, 0x59, 0x8, 0x0, 0x00, 0x01, enable, 0xFF, 0x54, 0x43};
  checksum_MR24HPC1(arr, sizeof(arr), true);

  uint32_t result = -1;
  uint32_t t = millis();
  bool once = true;
  while (result == -1) {
    if ( once && millis() - t > 500 ) {
      once = false;
      checksum_MR24HPC1(arr, sizeof(arr), true); // 2nd attempt
    }
    else if ( millis() - t > 2000 ) return "TIMEOUT";
    result = read_sensor((arr[2] << 24) | (arr[3] << 16) | (arr[4] << 8) | arr[5]);
  }
  if ( result == enable ) {
    strcpy(cbData.mode.info, ((enable) ? "Custom":"Standard"));
    cbData.mode.value = 0;
    return "SUCCESS";
  }
  return "FAIL";
}

String MR24HPC1::underlying_open_function_information_output_switch_inquiry() {
  uint8_t arr[] = {0x53, 0x59, 0x8, 0x80, 0x00, 0x01, 0xF, 0xFF, 0x54, 0x43};
  checksum_MR24HPC1(arr, sizeof(arr), true);

  uint32_t result = -1;
  uint32_t t = millis();
  bool once = true;
  while (result == -1) {
    if ( once && millis() - t > 500 ) {
      once = false;
      checksum_MR24HPC1(arr, sizeof(arr), true); // 2nd attempt
    }
    else if ( millis() - t > 2000 ) return "TIMEOUT";
    result = read_sensor((arr[2] << 24) | (arr[3] << 16) | (arr[4] << 8) | arr[5]);
  }
  return (result == 1) ? "ENABLED" : "DISABLED";
}

int16_t MR24HPC1::static_distance_inquiry() {
  uint8_t arr[] = {0x53, 0x59, 0x8, 0x83, 0x00, 0x01, 0xF, 0xFF, 0x54, 0x43};
  checksum_MR24HPC1(arr, sizeof(arr), true);

  uint32_t result = -1;
  uint32_t t = millis();
  bool once = true;
  while (result == -1) {
    if ( once && millis() - t > 500 ) {
      once = false;
      checksum_MR24HPC1(arr, sizeof(arr), true); // 2nd attempt
    }
    else if ( millis() - t > 2000 ) return -1;
    result = read_sensor((arr[2] << 24) | (arr[3] << 16) | (arr[4] << 8) | arr[5]);
  }
  return result;
}

int16_t MR24HPC1::motion_distance_inquiry() {
  uint8_t arr[] = {0x53, 0x59, 0x8, 0x84, 0x00, 0x01, 0xF, 0xFF, 0x54, 0x43};
  checksum_MR24HPC1(arr, sizeof(arr), true);

  uint32_t result = -1;
  uint32_t t = millis();
  bool once = true;
  while (result == -1) {
    if ( once && millis() - t > 500 ) {
      once = false;
      checksum_MR24HPC1(arr, sizeof(arr), true); // 2nd attempt
    }
    else if ( millis() - t > 2000 ) return -1;
    result = read_sensor((arr[2] << 24) | (arr[3] << 16) | (arr[4] << 8) | arr[5]);
  }
  return result;
}

int16_t MR24HPC1::motion_speed_inquiry() {
  uint8_t arr[] = {0x53, 0x59, 0x5, 0x85, 0x00, 0x01, 0xF, 0xFF, 0x54, 0x43};
  checksum_MR24HPC1(arr, sizeof(arr), true);

  uint32_t result = -1;
  uint32_t t = millis();
  bool once = true;
  while (result == -1) {
    if ( once && millis() - t > 500 ) {
      once = false;
      checksum_MR24HPC1(arr, sizeof(arr), true); // 2nd attempt
    }
    else if ( millis() - t > 2000 ) return -1;
    result = read_sensor((arr[2] << 24) | (arr[3] << 16) | (arr[4] << 8) | arr[5]);
  }
  return result;
}



int16_t MR24HPC1::motion_energy_value_inquiry() {
  uint8_t arr[] = {0x53, 0x59, 0x8, 0x82, 0x00, 0x01, 0xF, 0xFF, 0x54, 0x43};
  checksum_MR24HPC1(arr, sizeof(arr), true);

  uint32_t result = -1;
  uint32_t t = millis();
  bool once = true;
  while (result == -1) {
    if ( once && millis() - t > 500 ) {
      once = false;
      checksum_MR24HPC1(arr, sizeof(arr), true); // 2nd attempt
    }
    else if ( millis() - t > 2000 ) return -1;
    result = read_sensor((arr[2] << 24) | (arr[3] << 16) | (arr[4] << 8) | arr[5]);
  }
  return result;
}

int16_t MR24HPC1::existence_energy_value_inquiry() {
  uint8_t arr[] = {0x53, 0x59, 0x8, 0x81, 0x00, 0x01, 0xF, 0xFF, 0x54, 0x43};
  checksum_MR24HPC1(arr, sizeof(arr), true);

  uint32_t result = -1;
  uint32_t t = millis();
  bool once = true;
  while (result == -1) {
    if ( once && millis() - t > 500 ) {
      once = false;
      checksum_MR24HPC1(arr, sizeof(arr), true); // 2nd attempt
    }
    else if ( millis() - t > 2000 ) return -1;
    result = read_sensor((arr[2] << 24) | (arr[3] << 16) | (arr[4] << 8) | arr[5]);
  }
  return result;
}

String MR24HPC1::existence_perception_boundary_settings(uint8_t value) { // 1~10, 10 default
  value = constrain(value, 1, 10);
  uint8_t arr[] = {0x53, 0x59, 0x8, 0xA, 0x00, 0x01, value, 0xFF, 0x54, 0x43};
  checksum_MR24HPC1(arr, sizeof(arr), true);

  uint32_t result = -1;
  uint32_t t = millis();
  bool once = true;
  while (result == -1) {
    if ( once && millis() - t > 500 ) {
      once = false;
      checksum_MR24HPC1(arr, sizeof(arr), true); // 2nd attempt
    }
    else if ( millis() - t > 2000 ) return "TIMEOUT";
    result = read_sensor((arr[2] << 24) | (arr[3] << 16) | (arr[4] << 8) | arr[5]);
  }
  if ( result == value ) return "SUCCESS";
  return "FAIL";
}

String MR24HPC1::motion_trigger_boundary_setting(uint8_t value) { // 1~10, 10 default
  value = constrain(value, 1, 10);
  uint8_t arr[] = {0x53, 0x59, 0x8, 0xB, 0x00, 0x01, value, 0xFF, 0x54, 0x43};
  checksum_MR24HPC1(arr, sizeof(arr), true);

  uint32_t result = -1;
  uint32_t t = millis();
  bool once = true;
  while (result == -1) {
    if ( once && millis() - t > 500 ) {
      once = false;
      checksum_MR24HPC1(arr, sizeof(arr), true); // 2nd attempt
    }
    else if ( millis() - t > 2000 ) return "TIMEOUT";
    result = read_sensor((arr[2] << 24) | (arr[3] << 16) | (arr[4] << 8) | arr[5]);
  }
  if ( result == value ) return "SUCCESS";
  return "FAIL";
}


String MR24HPC1::motion_trigger_threshold_settings(uint8_t value) { // 0-250, 4 default
  value = constrain(value, 0, 250);
  uint8_t arr[] = {0x53, 0x59, 0x8, 0x9, 0x00, 0x01, value, 0xFF, 0x54, 0x43};
  checksum_MR24HPC1(arr, sizeof(arr), true);

  uint32_t result = -1;
  uint32_t t = millis();
  bool once = true;
  while (result == -1) {
    if ( once && millis() - t > 500 ) {
      once = false;
      checksum_MR24HPC1(arr, sizeof(arr), true); // 2nd attempt
    }
    else if ( millis() - t > 2000 ) return "TIMEOUT";
    result = read_sensor((arr[2] << 24) | (arr[3] << 16) | (arr[4] << 8) | arr[5]);
  }
  if ( result == value ) return "SUCCESS";
  return "FAIL";
}

String MR24HPC1::existence_judgment_threshold_settings(uint8_t value) { // 0-250, 33 default
  value = constrain(value, 0, 250);
  uint8_t arr[] = {0x53, 0x59, 0x8, 0x8, 0x00, 0x01, value, 0xFF, 0x54, 0x43};
  checksum_MR24HPC1(arr, sizeof(arr), true);

  uint32_t result = -1;
  uint32_t t = millis();
  bool once = true;
  while (result == -1) {
    if ( once && millis() - t > 500 ) {
      once = false;
      checksum_MR24HPC1(arr, sizeof(arr), true); // 2nd attempt
    }
    else if ( millis() - t > 2000 ) return "TIMEOUT";
    result = read_sensor((arr[2] << 24) | (arr[3] << 16) | (arr[4] << 8) | arr[5]);
  }
  if ( result == value ) return "SUCCESS";
  return "FAIL";
}

String MR24HPC1::motion_trigger_time_setting(uint32_t value) { // 0~1000ms, 150ms default
  value = constrain(value, 0, 1000);
  uint8_t arr[] = {0x53, 0x59, 0x8, 0xC, 0x00, 0x04, (uint8_t)(value >> 24), (uint8_t)(value >> 16), (uint8_t)(value >> 8), (uint8_t)(value), 0xFF, 0x54, 0x43};
  checksum_MR24HPC1(arr, sizeof(arr), true);

  uint32_t result = -1;
  uint32_t t = millis();
  bool once = true;
  while (result == -1) {
    if ( once && millis() - t > 500 ) {
      once = false;
      checksum_MR24HPC1(arr, sizeof(arr), true); // 2nd attempt
    }
    else if ( millis() - t > 2000 ) return "TIMEOUT";
    result = read_sensor((arr[2] << 24) | (arr[3] << 16) | (arr[4] << 8) | arr[5]);
  }
  if ( result == value ) return "SUCCESS";
  return "FAIL";
}

int16_t MR24HPC1::custom_mode_query() {
  uint8_t arr[] = {0x53, 0x59, 0x5, 0x89, 0x00, 0x01, 0xF, 0xFF, 0x54, 0x43};
  checksum_MR24HPC1(arr, sizeof(arr), true);

  uint32_t result = -1;
  uint32_t t = millis();
  bool once = true;
  while (result == -1) {
    if ( once && millis() - t > 500 ) {
      once = false;
      checksum_MR24HPC1(arr, sizeof(arr), true); // 2nd attempt
    }
    else if ( millis() - t > 2000 ) return -1;
    result = read_sensor((arr[2] << 24) | (arr[3] << 16) | (arr[4] << 8) | arr[5]);
  }
  return result;
}

String MR24HPC1::end_of_custom_mode_settings() {
  uint8_t arr[] = {0x53, 0x59, 0x5, 0x0A, 0x00, 0x01, 0xF, 0xFF, 0x54, 0x43};
  checksum_MR24HPC1(arr, sizeof(arr), true);

  uint32_t result = -1;
  uint32_t t = millis();
  bool once = true;
  while (result == -1) {
    if ( once && millis() - t > 500 ) {
      once = false;
      checksum_MR24HPC1(arr, sizeof(arr), true); // 2nd attempt
    }
    else if ( millis() - t > 2000 ) return "TIMEOUT";
    result = read_sensor((arr[2] << 24) | (arr[3] << 16) | (arr[4] << 8) | arr[5]);
  }
  if ( result == 1 ) return "SUCCESS";
  return "FAIL";
}


String MR24HPC1::motion_to_still_time_setting(uint32_t value) { // 1~60000ms, 3000ms default
  value = constrain(value, 1, 60000);
  uint8_t arr[] = {0x53, 0x59, 0x8, 0xD, 0x00, 0x04, (uint8_t)(value >> 24), (uint8_t)(value >> 16), (uint8_t)(value >> 8), (uint8_t)(value), 0xFF, 0x54, 0x43};
  checksum_MR24HPC1(arr, sizeof(arr), true);

  uint32_t result = -1;
  uint32_t t = millis();
  bool once = true;
  while (result == -1) {
    if ( once && millis() - t > 500 ) {
      once = false;
      checksum_MR24HPC1(arr, sizeof(arr), true); // 2nd attempt
    }
    else if ( millis() - t > 2000 ) return "TIMEOUT";
    result = read_sensor((arr[2] << 24) | (arr[3] << 16) | (arr[4] << 8) | arr[5]);
  }
  if ( result == value ) return "SUCCESS";
  return "FAIL";
}

String MR24HPC1::time_for_entering_no_person_state_setting(uint32_t ms) { // 0~3600000ms, 30000ms default

  if ( underlying_open_function_information_output_switch_inquiry() == "DISABLED" ) { // "ENABLED" "DISABLED"
	ms = constrain(ms, 0, 8);
  	uint8_t arr[] = {0x53, 0x59, 0x80, 0xA, 0x00, 0x01, (uint8_t)ms, 0xFF, 0x54, 0x43};
	checksum_MR24HPC1(arr, sizeof(arr), true);

      uint32_t result = -1;
      uint32_t t = millis();
      bool once = true;
      while (result == -1) {
        if ( once && millis() - t > 500 ) {
          once = false;
          checksum_MR24HPC1(arr, sizeof(arr), true); // 2nd attempt
        }
        else if ( millis() - t > 2000 ) return "TIMEOUT";
        result = read_sensor((arr[2] << 24) | (arr[3] << 16) | (arr[4] << 8) | arr[5]);
      }
      if ( result == ms ) return "SUCCESS";
  }
  else { // CUSTOM
    ms = constrain(ms, 0, 3600000);
    uint8_t arr[] = {0x53, 0x59, 0x8, 0xE, 0x00, 0x04, (uint8_t)(ms >> 24), (uint8_t)(ms >> 16), (uint8_t)(ms >> 8), (uint8_t)(ms), 0xFF, 0x54, 0x43};
    checksum_MR24HPC1(arr, sizeof(arr), true);

    uint32_t result = -1;
    uint32_t t = millis();
    bool once = true;
    while (result == -1) {
      if ( once && millis() - t > 500 ) {
        once = false;
        checksum_MR24HPC1(arr, sizeof(arr), true); // 2nd attempt
      }
      else if ( millis() - t > 2000 ) return "TIMEOUT";
      result = read_sensor((arr[2] << 24) | (arr[3] << 16) | (arr[4] << 8) | arr[5]);
    }
    if ( result == ms ) return "SUCCESS";

  }

/*
  checksum_MR24HPC1(arr, sizeof(arr), true);

  uint32_t result = -1;
  uint32_t t = millis();
  bool once = true;
  while (result == -1) {
    if ( once && millis() - t > 500 ) {
      once = false;
      checksum_MR24HPC1(arr, sizeof(arr), true); // 2nd attempt
    }
    else if ( millis() - t > 2000 ) return "TIMEOUT";
    result = read_sensor((arr[2] << 24) | (arr[3] << 16) | (arr[4] << 8) | arr[5]);
  }
  if ( result == ms ) return "SUCCESS";
*/
  return "TIMEOUT";
}

uint8_t MR24HPC1::checksum_MR24HPC1(uint8_t* arr, uint8_t len, bool send) {
  uint32_t checksum = 0;
  for ( int i = 0; i < len - 3; i++ ) checksum = checksum + arr[i];
  if ( send ) {
    arr[len - 3] = (uint8_t)checksum;
    stream->write(arr, len);
  }
  return (uint8_t)checksum;
}

String MR24HPC1::custom_mode_setting(uint8_t _mode) { // 1~4, 1 default
  _mode = constrain(_mode, 1, 4);
  uint8_t arr[] = {0x53, 0x59, 0x5, 0x9, 0x00, 0x01, _mode, 0xFF, 0x54, 0x43};
  checksum_MR24HPC1(arr, sizeof(arr), true);

  uint32_t result = -1;
  uint32_t t = millis();
  bool once = true;
  while (result == -1) {
    if ( once && millis() - t > 500 ) {
      once = false;
      checksum_MR24HPC1(arr, sizeof(arr), true); // 2nd attempt
    }
    else if ( millis() - t > 2000 ) return "TIMEOUT";
    result = read_sensor((arr[2] << 24) | (arr[3] << 16) | (arr[4] << 8) | arr[5]);
  }
  if ( result == _mode ) {
    cbData.mode.value = _mode;
    return "SUCCESS";
  }
  return "FAIL";
}


uint32_t MR24HPC1::read_sensor(uint32_t frame_id_request) {
  int len = 0;
  unsigned char _data[32] = {0};
  char _hex[4];
  while (stream->available()) {
    if (stream->read() == 0x53) {
      if (stream->read() == 0x59) {
        len = stream->readBytesUntil(0x43, _data + 2, 32);
        if (len) {
          len += 3;
          _data[0] = 0x53;
          _data[1] = 0x59;
          _data[len - 1] = 0x43;
          uint32_t checksum = 0;
          for ( int i = 0; i < len - 3; i++ ) checksum = checksum + _data[i];
          if ( _data[len - 3] == (uint8_t)checksum ) {
            uint32_t frame = (_data[2] << 24) | (_data[3] << 16) | (_data[4] << 8) | _data[5];
            if ( frame == 0x1010001 ) { // heartbeat_pack_query
              if ( _handler ) { cbData.heartbeat = true; processing(); _handler(cbData); cbData.heartbeat = false; }
              if ( frame == frame_id_request ) return _data[6];
              if ( _debug ) {
                Serial.print("Heartbeat pack query: ");
                Serial.println(_data[6]);
              }
              continue;
            }
            else if ( frame == 0x1020001 ) { // module reset
              if ( frame == frame_id_request ) return _data[6];
              if ( _debug ) {
                Serial.print("Module reset: ");
                Serial.println(_data[6]);
              }
              continue;
            }
            else if ( (frame & 0xFFFFFF00) == 0x2A10000 ) { // Product Model Query (length unknown)
              if ( _debug ) Serial.print("Product model query: ");
              for ( int x = 0; x < _data[5]; x++ ) {
                response_str[x] = _data[6+x];
                if ( _debug ) Serial.print((char)_data[6+x]);
              }
              response_str[_data[5]] = '\0';
              if ( _debug ) Serial.println();
              if ( 0x2A10000 == (frame_id_request & 0xFFFFFF00) ) return 1;
              continue;
            }
            else if ( (frame & 0xFFFFFF00) == 0x2A20000 ) { // Product Id Query (length unknown)
              if ( _debug ) Serial.print("Product id query: ");
              for ( int x = 0; x < _data[5]; x++ ) {
                response_str[x] = _data[6+x];
                if ( _debug ) Serial.print((char)_data[6+x]);
              }
              response_str[_data[5]] = '\0';
              if ( _debug ) Serial.println();
              if ( 0x2A20000 == (frame_id_request & 0xFFFFFF00) ) return 1;
              continue;
            }
            else if ( (frame & 0xFFFFFF00) == 0x2A30000 ) { // Hardware Model Query (length unknown)
              if ( _debug ) Serial.print("Hardware model query: ");
              for ( int x = 0; x < _data[5]; x++ ) {
                response_str[x] = _data[6+x];
                if ( _debug ) Serial.print((char)_data[6+x]);
              }
              response_str[_data[5]] = '\0';
              if ( _debug ) Serial.println();
              if ( 0x2A30000 == (frame_id_request & 0xFFFFFF00) ) return 1;
              continue;
            }
            else if ( (frame & 0xFFFFFF00) == 0x2A40000 ) { // Firmware Version Query (length unknown)
              if ( _debug ) Serial.print("Firmware version query: ");
              for ( int x = 0; x < _data[5]; x++ ) {
                response_str[x] = _data[6+x];
                if ( _debug ) Serial.print((char)_data[6+x]);
              }
              response_str[_data[5]] = '\0';
              if ( _debug ) Serial.println();
              if ( 0x2A40000 == (frame_id_request & 0xFFFFFF00) ) return 1;
              continue;
            }
            else if ( frame == 0x5010001 ) { // Initialization complete
              if ( frame == frame_id_request ) return _data[6];
              if ( _debug ) {
                Serial.print("Initialization complete");
                Serial.println();
              }
              continue;
            }
            else if ( frame == 0x5070001 ) { // Scene mode setting
              if ( frame == frame_id_request ) return _data[6];
              if ( _debug ) {
                Serial.print("Scene Mode set to ");
                Serial.println(_data[6]);
              }
              continue;
            }
            else if ( frame == 0x5080001 ) { // Sensitivity setting
              if ( frame == frame_id_request ) return _data[6];
              if ( _debug ) {
                Serial.print("Sensitivity set to ");
                Serial.println(_data[6]);
              }
              continue;
            }
            else if ( frame == 0x5090001 ) { // Custom mode setting
              if ( frame == frame_id_request ) return _data[6];
              if ( _debug ) {
                Serial.print("Custom Mode set to ");
                Serial.println(_data[6]);
              }
              continue;
            }
            else if ( frame == 0x50A0001 ) { // End of custom mode settings (Save)
              if ( frame == frame_id_request ) return 1;
              if ( _debug ) {
               Serial.println("End of Custom Mode Settings (Saved)");
              }
              continue;
            }
            else if ( frame == 0x5810001 ) { // Initialization status inquiry
              if ( frame == frame_id_request ) return _data[6];
              if ( _debug ) {
                Serial.print("Initialization status inquiry: ");
                Serial.println(_data[6]);
              }
              continue;
            }
            else if ( frame == 0x5870001 ) { // scene settings inquiry
              if ( frame == frame_id_request ) return _data[6];
              if ( _debug ) {
                Serial.print("Scene settings inquiry: ");
                Serial.println(_data[6]);
              }
              continue;
            }
            else if ( frame == 0x5880001 ) { // sensitivity settings inquiry
              if ( frame == frame_id_request ) return _data[6];
              if ( _debug ) {
                Serial.print("Sensitivity settings inquiry: ");
                Serial.println(_data[6]);
              }
              continue;
            }
            else if ( frame == 0x5890001 ) { // Custom Mode query
              cbData.mode.value = _data[6];
              if ( frame == frame_id_request ) return _data[6];
              if ( _debug ) {
                Serial.print("Custom Mode query: ");
                Serial.println(_data[6]);
              }
              continue;
            }
            else if ( frame == 0x8000001 ) { // Underlying output function enable/disable
              if ( frame == frame_id_request ) return _data[6];
              if ( _debug ) {
                Serial.print("Underlying output function status: ");
                Serial.println(_data[6]);
              }
              continue;
            }
            else if ( frame == 0x8010005 ) { // Reporting of Sensor information
              cbData.custom.staticDistance = _data[5 + 2];
              cbData.custom.existanceEnergy = _data[5 + 1];
              cbData.custom.motionEnergy = _data[5 + 3];
              cbData.custom.motionDistance = _data[5 + 4];
              cbData.custom.motionSpeed = _data[5 + 5];
              processing();
              if ( _handler ) {
                _handler(cbData);
              }
              if ( frame == frame_id_request ) return -1;
              if ( _debug ) {
                Serial.print("Existence Energy: ");
                Serial.print(_data[5 + 1]);
                Serial.print("\tStatic Distance: ");
                Serial.print(_data[5 + 2]);
                Serial.print("\tMotion Energy: ");
                Serial.print(_data[5 + 3]);
                Serial.print("\tMotion Distance: ");
                Serial.print(_data[5 + 4]);
                Serial.print("\tMotion Speed: ");
                Serial.print(_data[5 + 5]);
                Serial.println();
              }
              continue;
            }
            else if ( frame == 0x8060001 ) { // Proximity Reporting // custom
              if ( _data[6] == 0 ) strcpy(cbData.proximity.info, "No State");
              if ( _data[6] == 1 ) strcpy(cbData.proximity.info, "Near (Approaching sensor for 3s)");
              if ( _data[6] == 2 ) strcpy(cbData.proximity.info, "Far (Moving away from sensor for 3s.)");
              cbData.proximity.value = _data[6];
              processing();
              if ( _handler ) { 
                _handler(cbData);
              }
              if ( frame == frame_id_request ) return _data[6];
              if ( _debug ) {
                Serial.print("Proximity Reporting: ");
			if ( _data[6] == 0 ) Serial.print("No State");
			if ( _data[6] == 1 ) Serial.print("Near (Approaching sensor for 3s)");
			if ( _data[6] == 2 ) Serial.print("Far (Moving away from sensor for 3s.)");
                Serial.println(".");
              }
              continue;
            }
            else if ( frame == 0x80A0001 ) { // Existance perception boundary set
              if ( frame == frame_id_request ) return _data[6];
              if ( _debug ) {
                Serial.print("Existance perception boundary was set to ");
                if ( _data[6] == 0x1 ) Serial.print("0.5");
                else if ( _data[6] == 0x2 ) Serial.print("1");
                else if ( _data[6] == 0x3 ) Serial.print("1.5");
                else if ( _data[6] == 0x4 ) Serial.print("2.0");
                else if ( _data[6] == 0x5 ) Serial.print("2.5");
                else if ( _data[6] == 0x6 ) Serial.print("3");
                else if ( _data[6] == 0x7 ) Serial.print("3.5");
                else if ( _data[6] == 0x8 ) Serial.print("4");
                else if ( _data[6] == 0x9 ) Serial.print("4.5");
                else if ( _data[6] == 0xA ) Serial.print("5");
                Serial.println("m.");
              }
              continue;
            }
            else if ( frame == 0x80B0001 ) { // Motion trigger boundary set
              if ( frame == frame_id_request ) return _data[6];
              if ( _debug ) {
                Serial.print("Motion trigger boundary was set to ");
                if ( _data[6] == 0x1 ) Serial.print("0.5");
                else if ( _data[6] == 0x2 ) Serial.print("1");
                else if ( _data[6] == 0x3 ) Serial.print("1.5");
                else if ( _data[6] == 0x4 ) Serial.print("2.0");
                else if ( _data[6] == 0x5 ) Serial.print("2.5");
                else if ( _data[6] == 0x6 ) Serial.print("3");
                else if ( _data[6] == 0x7 ) Serial.print("3.5");
                else if ( _data[6] == 0x8 ) Serial.print("4");
                else if ( _data[6] == 0x9 ) Serial.print("4.5");
                else if ( _data[6] == 0xA ) Serial.print("5");
                Serial.println("m.");
              }
              continue;
            }
            else if ( frame == 0x80C0004 ) { // Motion trigger time set
              if ( frame == frame_id_request ) return (_data[6] << 24) | (_data[7] << 16) | (_data[8] << 8) | _data[9];
              if ( _debug ) {
                Serial.print("Motion trigger time was set to ");
                Serial.print((_data[6] << 24) | (_data[7] << 16) | (_data[8] << 8) | _data[9]);
                Serial.println("ms.");
              }
              continue;
            }
            else if ( frame == 0x80D0004 ) { // Motion to Still time set
              if ( frame == frame_id_request ) return (_data[6] << 24) | (_data[7] << 16) | (_data[8] << 8) | _data[9];
              if ( _debug ) {
                Serial.print("Motion to Still time was set to ");
                Serial.print((_data[6] << 24) | (_data[7] << 16) | (_data[8] << 8) | _data[9]);
                Serial.println("ms.");
              }
              continue;
            }
            else if ( frame == 0x80E0004 ) { // Time for entering no person state set
              if ( frame == frame_id_request ) return (_data[6] << 24) | (_data[7] << 16) | (_data[8] << 8) | _data[9];
              if ( _debug ) {
                Serial.print("?Time for entering no person state set to ");
                Serial.print((_data[6] << 24) | (_data[7] << 16) | (_data[8] << 8) | _data[9]);
                Serial.println("ms.");
              }
              continue;
            }
            else if ( frame == 0x8090001 ) { // Motion trigger threshold set
              if ( frame == frame_id_request ) return _data[6];
              if ( _debug ) {
                Serial.print("Motion trigger threshold was set to ");
                Serial.print(_data[6]);
                Serial.println(".");
              }
              continue;
            }
            else if ( frame == 0x8080001 ) { // Existence judgement threshold set
              if ( frame == frame_id_request ) return _data[6];
              if ( _debug ) {
                Serial.print("Existence judgement threshold was set to ");
                Serial.print(_data[6]);
                Serial.println(".");
              }
              continue;
            }
            else if ( frame == 0x8800001 ) { // underlying_open_function_information_output_switch_inquiry
              if ( frame == frame_id_request ) return _data[6];
              if ( _debug ) {
                Serial.print("Underlying Open function is ");
                Serial.print(_data[6]);
                Serial.println(".");
              }
              continue;
            }
            else if ( frame == 0x8810001 ) { // Existence Energy Value Inquiry
              if ( frame == frame_id_request ) return _data[6];
              if ( _debug ) {
                Serial.print("Existence energy value is ");
                Serial.print(_data[6]);
                Serial.println(".");
              }
              continue;
            }
            else if ( frame == 0x8820001 ) { // Motion Energy Value Inquiry
              if ( frame == frame_id_request ) return _data[6];
              if ( _debug ) {
                Serial.print("Motion Energy value is ");
                Serial.print(_data[6]);
                Serial.println(".");
              }
              continue;
            }
            else if ( frame == 0x8830001 ) { // Static Distance Inquiry
              if ( frame == frame_id_request ) return _data[6];
              if ( _debug ) {
                Serial.print("Static distance is ");
                Serial.print(_data[6]);
                Serial.println(".");
              }
              continue;
            }
            else if ( frame == 0x8840001 ) { // Motion Distance Inquiry
              if ( frame == frame_id_request ) return _data[6];
              if ( _debug ) {
                Serial.print("Motion distance is ");
                Serial.print(_data[6]);
                Serial.println(".");
              }
              continue;
            }
            else if ( frame == 0x8880001 ) { // Existence judgement threshold Inquiry
              if ( frame == frame_id_request ) return _data[6];
              if ( _debug ) {
                Serial.print("Existence judgement threshold is ");
                Serial.print(_data[6]);
                Serial.println(".");
              }
              continue;
            }
            else if ( frame == 0x8890001 ) { // Motion trigger threshold Inquiry
              if ( frame == frame_id_request ) return _data[6];
              if ( _debug ) {
                Serial.print("Motion trigger threshold is ");
                Serial.print(_data[6]);
                Serial.println(".");
              }
              continue;
            }
            else if ( frame == 0x88A0001 ) { // existence_perception_boundary_inquiry
              if ( frame == frame_id_request ) return _data[6];
              if ( _debug ) {
                Serial.print("Existence perception boundary is ");
                Serial.print(_data[6]);
                Serial.println(".");
              }
              continue;
            }
            else if ( frame == 0x88B0001 ) { // motion_trigger_boundary_inquiry
              if ( frame == frame_id_request ) return _data[6];
              if ( _debug ) {
                Serial.print("Motion trigger boundary is ");
                Serial.print(_data[6]);
                Serial.println(".");
              }
              continue;
            }
            else if ( frame == 0x88C0004 ) { // motion_trigger_time_inquiry
              if ( frame - 3 == frame_id_request ) return (_data[6] << 24) | (_data[7] << 16) | (_data[8] << 8) | _data[9];
              if ( _debug ) {
                Serial.print("Motion trigger time inquiry is ");
                Serial.print(_data[6]);
                Serial.println(".");
              }
              continue;
            }
            else if ( frame == 0x88D0004 ) { // motion_to_still_time_inquiry
              if ( frame - 3 == frame_id_request ) return (_data[6] << 24) | (_data[7] << 16) | (_data[8] << 8) | _data[9];
              if ( _debug ) {
                Serial.print("Motion to still time inquiry is ");
                Serial.print(_data[6]);
                Serial.println(".");
              }
              continue;
            }
            else if ( frame == 0x88E0004 ) { // time_for_entering_no_person_state_inquiry
              if ( frame == frame_id_request ) return (_data[6] << 24) | (_data[7] << 16) | (_data[8] << 8) | _data[9];
              if ( _debug ) {
                Serial.print("Time for entering no person state set to ");
                Serial.print((_data[6] << 24) | (_data[7] << 16) | (_data[8] << 8) | _data[9]);
                Serial.println(".");
              }
              continue;
            }
            else if ( frame == 0x800A0001 ) { // time_for_entering_no_person_state (standard)
              if ( frame == frame_id_request ) return _data[6];
              if ( _debug ) {
                Serial.print("Time for entering no person state set to ");
                if ( _data[6] == 0x0 ) Serial.println("None");
                else if ( _data[6] == 0x1 ) Serial.println("10s");
                else if ( _data[6] == 0x2 ) Serial.println("30s");
                else if ( _data[6] == 0x3 ) Serial.println("1min");
                else if ( _data[6] == 0x4 ) Serial.println("2min");
                else if ( _data[6] == 0x5 ) Serial.println("5min");
                else if ( _data[6] == 0x6 ) Serial.println("10min");
                else if ( _data[6] == 0x7 ) Serial.println("30min");
                else if ( _data[6] == 0x8 ) Serial.println("60min");
              }
              continue;
            }
            else if ( frame == 0x80810001 ) { // Presence Information Inquiry
              if ( _data[6] == 0 ) strcpy(cbData.presence.info, "Unoccupied");
              if ( _data[6] == 1 ) strcpy(cbData.presence.info, "Occupied");
              cbData.presence.value = _data[6];
              processing();
              if ( _handler ) { 
                _handler(cbData);
              }
              if ( frame == frame_id_request ) return _data[6];
              if ( _debug ) {
                Serial.print("Presence information inquiry is ");
                Serial.print(_data[6]);
                Serial.println(".");
              }
              continue;
            }
            else if ( frame == 0x80820001 ) { // Motion Information Inquiry
              if ( frame == frame_id_request ) return _data[6];
              if ( _debug ) {
                Serial.print("Motion information inquiry is ");
                Serial.print(_data[6]);
                Serial.println(".");
              }
              continue;
            }
            else if ( frame == 0x800B0001 ) { // Proximity Reporting
              if ( _data[6] == 0 ) strcpy(cbData.proximity.info, "No State");
              if ( _data[6] == 1 ) strcpy(cbData.proximity.info, "Near (Approaching sensor for 3s)");
              if ( _data[6] == 2 ) strcpy(cbData.proximity.info, "Far (Moving away from sensor for 3s.)");
              cbData.proximity.value = _data[6];
              processing();
              if ( _handler ) { 
                _handler(cbData);
              }
              if ( frame == frame_id_request ) return _data[6];
              if ( _debug ) {
                Serial.print("Proximity Reporting: ");
			if ( _data[6] == 0 ) Serial.print("No State");
			if ( _data[6] == 1 ) Serial.print("Near (Approaching sensor for 3s)");
			if ( _data[6] == 2 ) Serial.print("Far (Moving away from sensor for 3s.)");
                Serial.println(".");
              }
              continue;
            }
            else if ( frame == 0x80010001 ) { // Presence Information
              if ( _data[6] == 0 ) strcpy(cbData.presence.info, "Unoccupied");
              if ( _data[6] == 1 ) strcpy(cbData.presence.info, "Occupied");
              cbData.presence.value = _data[6];
              processing();
              if ( _handler ) { 
                _handler(cbData);
              }
              if ( frame == frame_id_request ) return _data[6];
              if ( _debug ) {
                Serial.print("Presence Information: ");
			if ( _data[6] == 0 ) Serial.print("Unoccupied");
			if ( _data[6] == 1 ) Serial.print("Occupied");
                Serial.println(".");
              }
              continue;
            }
            else if ( frame == 0x80020001 ) { // Motion information
              if ( _data[6] == 0 ) strcpy(cbData.motion.info, "None");
              if ( _data[6] == 1 ) strcpy(cbData.motion.info, "Motionless");
              if ( _data[6] == 2 ) strcpy(cbData.motion.info, "Active");
              cbData.motion.value = _data[6];
              processing();
              if ( _handler ) { 
                _handler(cbData);
              }
              if ( frame == frame_id_request ) return _data[6];
              if ( _debug ) {
                Serial.print("Motion information: ");
			if ( _data[6] == 0 ) Serial.print("None");
			if ( _data[6] == 1 ) Serial.print("Motionless");
			if ( _data[6] == 2 ) Serial.print("Active");
                Serial.println(".");
              }
              continue;
            }
            else if ( frame == 0x8070001 ) { // body_movement_parameter  (underlying = true)
              strcpy(cbData.mode.info, "Custom");
              if ( _data[6] == 0 ) {
                strcpy(cbData.movement.info, "None");
                strcpy(cbData.presence.info, "Unoccupied");
                cbData.presence.value = 0;
              }
              if ( _data[6] == 1 ) strcpy(cbData.movement.info, "Stationary");
              if ( _data[6] > 1 ) strcpy(cbData.movement.info, "Moving");
              if ( _data[6] != 0 ) {
                strcpy(cbData.presence.info, "Occupied");
                cbData.presence.value = 1;
              }
              cbData.movement.value = _data[6];
              processing();
              if ( _handler ) { 
                _handler(cbData);
              }
              if ( frame == frame_id_request ) return _data[6];
              if ( _debug ) {
                Serial.print("(0: None, 1: Stationary, 2-100: Moving) 1B Body Movement Parameter is ");
                Serial.print(_data[6]);
                Serial.println(".");
              }
              continue;
            }
            else if ( frame == 0x80030001 ) { // body_movement_parameter (underlying = false)
              strcpy(cbData.mode.info, "Standard"); cbData.mode.value = 0;
              if ( _data[6] == 0 ) {
                strcpy(cbData.movement.info, "None");
                strcpy(cbData.presence.info, "Unoccupied");
                cbData.presence.value = 0;
              }
              if ( _data[6] == 1 ) strcpy(cbData.movement.info, "Stationary");
              if ( _data[6] > 1 ) strcpy(cbData.movement.info, "Moving");
              if ( _data[6] != 0 ) {
                strcpy(cbData.presence.info, "Occupied");
                cbData.presence.value = 1;
              }
              cbData.movement.value = _data[6];
              processing();
              if ( _handler ) { 
                _handler(cbData);
              }
              if ( frame == frame_id_request ) return _data[6];
              if ( _debug ) {
                Serial.print("(0: None, 1: Stationary, 2-100: Moving) 1B Body Movement Parameter is ");
                Serial.print(_data[6]);
                Serial.println(".");
              }
              continue;
            }
            else if ( frame == 0x80830001 ) { // body_movement_parameter_inquiry
              if ( frame == frame_id_request ) return _data[6];
              if ( _debug ) {
                Serial.print("Body Movement parameter inquiry is ");
                Serial.print(_data[6]);
                Serial.println(".");
              }
              continue;
            }
            else if ( frame == 0x808A0001 ) { // time_for_entering_no_person_state_inquiry
              if ( frame == frame_id_request ) return _data[6];
              if ( _debug ) {
                Serial.print("Time for entering no person state is ");
                Serial.print(_data[6]);
                Serial.println(".");
              }
              continue;
            }
            else if ( frame == 0x808B0001 ) { // Proximity Inquiry
              if ( frame == frame_id_request ) return _data[6];
              if ( _debug ) {
                Serial.print("Proximity inquiry is ");
                Serial.print(_data[6]);
                Serial.println(".");
              }
              continue;
            }

            if ( _debug ) { // print remaining unhandled frames
              Serial.print("DATA: ");
              for ( int i = 0; i < len; i++ ) {
                snprintf(_hex, sizeof(_hex), "%02X", _data[i]);
                Serial.print(_hex);
                Serial.print(" ");
              }
              Serial.println();
            }


          }
        }
      }
    }
  }
  return -1;
}