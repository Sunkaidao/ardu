/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 *
 *      Author: Breeder Bai
 */
#pragma once


#include <uavcan/uavcan.hpp>

#include <string.h>
#include "stdio.h"
#include <AP_HAL/CAN.h>
#include <AP_Math/AP_Math.h>
#include <AP_HAL/Semaphores.h>
#include <AP_HAL_ChibiOS/CAN.h>
#include <AC_Sprayer/AC_Sprayer.h>
#include <AP_Param/AP_Param.h>
#include <AP_BoardConfig/AP_BoardConfig_CAN.h>
#include "./../../ArduCopter/config.h"
#include <uavcan/helpers/heap_based_pool_allocator.hpp>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <AP_Stats/AP_Stats.h>     // statistics library
#include <AP_Arming/AP_Arming.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_GPS/AP_GPS.h>             // ArduPilot GPS library

#include "AP_NewBroadcast_CAN.h"
#include "AP_NewBroadcast_serial.h"
#include "NewBroadcast_Backend.h"

#if NEWBROADCAST == ENABLED 

class AP_NewBroadcast {
public:

// 4G driver types
enum NewBroadcast_Type {
	NewBroadcast_TYPE_NONE  = 0,
   	NewBroadcast_TYPE_GK_CAN  = 1,
   	NewBroadcast_TYPE_SERIAL  = 2,

};
    AP_NewBroadcast();
    ~AP_NewBroadcast();

	static const struct AP_Param::GroupInfo var_info[];

private:
    bool _initialized;
	AP_HAL::CANManager* _parent_can_mgr;
	AP_NewBroadcast_Backend* drive;

    AP_Int8  _enable;
	AP_Int8  _reg_no[REG_NO_STRING_LEN];
    AP_Int32 _flight_seq;

	int32_t flight_seq_pre;
	uint64_t flight_area_m2_curr;	
	uint64_t flight_area_m2_pre;
	uint64_t timer;

	uint32_t flight_time_last;

    Message_info view;
	Message_send_union payload;

	bool send_flag;	
	uint64_t send_last_time;
		
	static const uint16_t crc16tab[];
public:
	void update();
	void init();

	void send_reg_no(mavlink_channel_t chan);
	void send_flight_status(mavlink_channel_t chan);
	uint16_t get_view_flight_area(){return view.flight_area;}
	uint16_t get_view_flight_length(){return view.flight_length;}
	
	MAV_RESULT handle_msg_newbroadcast_str(const mavlink_newbroadcast_str_t &packet,mavlink_channel_t chan);
    void update_view();
    void update_view_action();
    void update_reg_no(const mavlink_newbroadcast_str_t &packet,mavlink_channel_t chan);
    void update_view_reg_no();
    void update_view_flight_seq();
    void update_view_now_time();
    void update_view_state();
    void update_view_flight_time();
    void update_view_longitude();
    void update_view_latitude();
    void update_view_height();
    void update_view_altitude();
    void update_view_path_angle();
    void update_view_pitch_angle();
    void update_view_roll_angle();
    void update_view_horizontal_velocity();
    void update_view_is_nozzle_work();
    void update_view_nozzle_diameter();
    void update_view_nozzle_angle();
    void update_view_nozzle_pressure();
    void update_view_spray_range();	
	void update_view_flight_control();
	void update_view_task_id(const mavlink_newbroadcast_str_t &packet,mavlink_channel_t chan);
	void update_view_app_ver_no(const mavlink_newbroadcast_str_t &packet,mavlink_channel_t chan);
	void update_view_tp_reg_no(const mavlink_newbroadcast_str_t &packet,mavlink_channel_t chan);
	void update_view_remain_dose();
	void update_view_used_dose();
	void update_view_cur_flow();
	void update_view_flight_area();
	void update_view_flight_length();
	void update_payload();

	uint8_t detect_backends();

};
#endif

