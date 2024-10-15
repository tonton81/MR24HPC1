#ifndef MR24HPC1_RADAR_H
#define MR24HPC1_RADAR_H
#include "Arduino.h"


typedef struct RADAR_message_t {
  bool heartbeat = false;

  struct {
    uint32_t duration = millis();
  } occupied;

  struct {
    char info[12] = ""; 
    uint8_t value = 0;
    uint32_t duration = millis();
    struct {
      uint32_t count = 0;
    } trigger;
  } movement;
  struct {
    char info[40] = "No State"; 
    uint8_t value = 0;
  } proximity;
  struct {
    char info[12] = ""; 
    uint8_t value = 0;
  } mode;
  struct {
    char info[12] = ""; 
    uint8_t value = 0;
  } motion;
  struct {
    char info[12] = "Unoccupied"; 
    uint8_t value = 0;
  } presence;

  struct {
    uint8_t staticDistance = 0;
    uint8_t existanceEnergy = 0;
    uint8_t motionEnergy = 0;
    uint8_t motionDistance = 0;
    uint8_t motionSpeed = 0;
  } custom;


} RADAR_message_t;


typedef enum RADAR_UNMANNED_TIME {
  RADAR_UNMANNED_None = 0,
  RADAR_UNMANNED_10s = 1,
  RADAR_UNMANNED_30s = 2,
  RADAR_UNMANNED_1min = 3,
  RADAR_UNMANNED_2min = 4,
  RADAR_UNMANNED_5min = 5,
  RADAR_UNMANNED_10min = 6,
  RADAR_UNMANNED_30min = 7,
  RADAR_UNMANNED_60min = 8,
} RADAR_UNMANNED_TIME;

/*
typedef enum RADAR_EXISTANCE_PERCEPTION_BOUNDARY_SETTINGS{
  EXISTANCE_PERCEPTION_BOUNDARY_0.5m = 1,
  EXISTANCE_PERCEPTION_BOUNDARY_1m = 2,
  EXISTANCE_PERCEPTION_BOUNDARY_1.5m = 3,
  EXISTANCE_PERCEPTION_BOUNDARY_2m = 4,
  EXISTANCE_PERCEPTION_BOUNDARY_2.5m = 5,
  EXISTANCE_PERCEPTION_BOUNDARY_3m = 6,
  EXISTANCE_PERCEPTION_BOUNDARY_3.5m = 7,
  EXISTANCE_PERCEPTION_BOUNDARY_4m = 8,
  EXISTANCE_PERCEPTION_BOUNDARY_4.5m = 9,
  EXISTANCE_PERCEPTION_BOUNDARY_5m = 10,
} RADAR_EXISTANCE_PERCEPTION_BOUNDARY_SETTINGS;

typedef enum RADAR_MOTION_TRIGGER_BOUNDARY_SETTINGS{
  MOTION_TRIGGER_BOUNDARY_0.5m = 1,
  MOTION_TRIGGER_BOUNDARY_1m = 2,
  MOTION_TRIGGER_BOUNDARY_1.5m = 3,
  MOTION_TRIGGER_BOUNDARY_2m = 4,
  MOTION_TRIGGER_BOUNDARY_2.5m = 5,
  MOTION_TRIGGER_BOUNDARY_3m = 6,
  MOTION_TRIGGER_BOUNDARY_3.5m = 7,
  MOTION_TRIGGER_BOUNDARY_4m = 8,
  MOTION_TRIGGER_BOUNDARY_4.5m = 9,
  MOTION_TRIGGER_BOUNDARY_5m = 10,
} RADAR_MOTION_TRIGGER_BOUNDARY_SETTINGS;

typedef enum RADAR_CUSTOM_MODE {
  CUSTOM_MODE_1 = 1,
  CUSTOM_MODE_2 = 2,
  CUSTOM_MODE_3 = 3,
  CUSTOM_MODE_4 = 4,
} RADAR_CUSTOM_MODE;
*/

typedef enum RADAR_SCENE_MODE {
  RADAR_LIVINGROOM = 1, // 4m
  RADAR_BEDROOM = 2, // 3.5m
  RADAR_BATHROOM = 3, // 2.5m
  RADAR_AREA_DETECTION = 4, // 3m
} RADAR_SCENE_MODE;
typedef enum RADAR_SENSITIVITY_LEVEL {
  RADAR_SENSITIVITY_LEVEL1 = 1,
  RADAR_SENSITIVITY_LEVEL2 = 2,
  RADAR_SENSITIVITY_LEVEL3 = 3,
} RADAR_SENSITIVITY_LEVEL;

typedef void (*RADARCB_ptr)(const RADAR_message_t &msg); /* callback */

class MR24HPC1 {
	private:
		Stream *stream;
		bool _debug = false;
		char response_str[16] = {'\0'};
		bool standard = false;
		RADARCB_ptr _handler;
		RADAR_message_t cbData;
		uint32_t read_sensor(uint32_t frame_id_request = 0x99999999);
		uint8_t checksum_MR24HPC1(uint8_t* arr, uint8_t len, bool send);
		uint32_t _time_occupied = 0;
		uint32_t _time_movement = 0;
		uint8_t last_movement = 0;
		void processing();
		bool _communication = false;
		uint32_t _communication_timeout = 0;

	public:
		MR24HPC1(Stream *s):stream(s){;}
		bool communication() { return _communication; }
		void debug(bool state) { _debug = state; }
		
		uint8_t convert_distance_float(float f);
		String module_reset(); // soft reset, active profile continues.
		String heartbeat_pack_query();
		void onReceive(RADARCB_ptr handler); /* callback function */

		// PRODUCT INFORMATION
		String product_model_query();
		String product_id_query();
		String hardware_model_query();
		String firmware_version_query();
		
		//STANDARD MODE
		void Standard(RADAR_SCENE_MODE scene, RADAR_SENSITIVITY_LEVEL sens, RADAR_UNMANNED_TIME t);
		String scene_settings(RADAR_SCENE_MODE scene);
		String sensitivity_settings(RADAR_SENSITIVITY_LEVEL sens);
		String initialization_status_inquiry();
		String scene_settings_inquiry();
		String sensitivity_settings_inquiry();

		// CUSTOM MODE
		void Custom(uint8_t mode, uint8_t existT, uint8_t motT, float mTB, uint32_t motTT, uint32_t motST, float ePB, uint32_t unmanT);
		String custom_mode_setting(uint8_t _mode); // 1~4, 1 default
		String end_of_custom_mode_settings();
		int16_t custom_mode_query();
		String motion_trigger_time_inquiry();
		String motion_to_still_time_inquiry();
		String motion_trigger_boundary_inquiry();
		int16_t motion_trigger_threshold_inquiry();
		String existence_perception_boundary_inquiry();
		int16_t existence_judgement_threshold_inquiry();
		String existence_judgment_threshold_settings(uint8_t value); // 0-250, 33 default
		String motion_to_still_time_setting(uint32_t value); // 1~60000ms, 3000ms default
		String motion_trigger_time_setting(uint32_t value); // 0~1000ms, 150ms default
		String motion_trigger_threshold_settings(uint8_t value); // 0-250, 4 default
		String motion_trigger_boundary_setting(uint8_t value); // 1~10, 10 default
		String existence_perception_boundary_settings(uint8_t value); // 1~10, 10 default
		int16_t existence_energy_value_inquiry();
		int16_t motion_energy_value_inquiry();
		int16_t motion_distance_inquiry();
		int16_t motion_speed_inquiry(); // no response from request (chip unsupported?)
		int16_t static_distance_inquiry();
		String motion_information_inquiry();

		// COMMON
		String time_for_entering_no_person_state_setting(uint32_t ms); // 0~3600000ms, 30000ms default, 0-8 presets for standard mode
		String presence_information_inquiry();
		String proximity_inquiry();
		int16_t body_movement_parameter_inquiry();
		String time_for_entering_no_person_state_inquiry();
		String underlying_open_function_information_output_switch(bool enable);
		String underlying_open_function_information_output_switch_inquiry();
		void events(){ read_sensor(); }
};

#endif