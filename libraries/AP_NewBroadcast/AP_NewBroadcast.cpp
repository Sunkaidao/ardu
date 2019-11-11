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
 * AP_NewBroadcast.cpp
 *
 *      Author: Breeder Bai
 */

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>


#include "AP_NewBroadcast.h"
#include <GCS_MAVLink/GCS.h>

#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_BoardConfig/AP_BoardConfig_CAN.h>


#if NEWBROADCAST == ENABLED

extern const AP_HAL::HAL& hal;

// table of user settable New Broadcast parameters
const AP_Param::GroupInfo AP_NewBroadcast::var_info[] = {

    // @Param: REG_NO
    // @DisplayName: NBC_REG_NO
    // @Description:  UAV registration number
    // @User: Advanced
    //AP_GROUPINFO("REG_NO", 0 , AP_NewBroadcast, _reg_no, 0),

    // @Param: FLI_SEQ
    // @DisplayName: NBC_FLI_SEQ
    // @Description:  Number of flights
    // @User: Advanced
    AP_GROUPINFO("FLI_SEQ", 1 , AP_NewBroadcast, _flight_seq, 0),

    // @Param: ENABLE
    // @DisplayName: NBC_ENABLE
    // @Description:  Enable new Broadcast
    // @User: Advanced
    AP_GROUPINFO("ENABLE", 2 , AP_NewBroadcast, _enable, 0),

	AP_GROUPINFO("REG_NO0", 3 , AP_NewBroadcast, _reg_no[0], 0),
	AP_GROUPINFO("REG_NO1", 4 , AP_NewBroadcast, _reg_no[1], 0),
	AP_GROUPINFO("REG_NO2", 5 , AP_NewBroadcast, _reg_no[2], 0),
	AP_GROUPINFO("REG_NO3", 6 , AP_NewBroadcast, _reg_no[3], 0),
	AP_GROUPINFO("REG_NO4", 7 , AP_NewBroadcast, _reg_no[4], 0),
	AP_GROUPINFO("REG_NO5", 8 , AP_NewBroadcast, _reg_no[5], 0),
	AP_GROUPINFO("REG_NO6", 9 , AP_NewBroadcast, _reg_no[6], 0),
	AP_GROUPINFO("REG_NO7", 10 , AP_NewBroadcast, _reg_no[7], 0),
	AP_GROUPINFO("REG_NO8", 11 , AP_NewBroadcast, _reg_no[8], 0),
	AP_GROUPINFO("REG_NO9", 12 , AP_NewBroadcast, _reg_no[9], 0),
	AP_GROUPINFO("REG_NO10", 13 , AP_NewBroadcast, _reg_no[10], 0),
	AP_GROUPINFO("REG_NO11", 14 , AP_NewBroadcast, _reg_no[11], 0),
	AP_GROUPINFO("REG_NO12", 15 , AP_NewBroadcast, _reg_no[12], 0),
	AP_GROUPINFO("REG_NO13", 16 , AP_NewBroadcast, _reg_no[13], 0),
	AP_GROUPINFO("REG_NO14", 17 , AP_NewBroadcast, _reg_no[14], 0),
	AP_GROUPINFO("REG_NO15", 18 , AP_NewBroadcast, _reg_no[15], 0),
	AP_GROUPINFO("REG_NO16", 19 , AP_NewBroadcast, _reg_no[16], 0),
	AP_GROUPINFO("REG_NO17", 20 , AP_NewBroadcast, _reg_no[17], 0),
	AP_GROUPINFO("REG_NO18", 21 , AP_NewBroadcast, _reg_no[18], 0),
	AP_GROUPINFO("REG_NO19", 22 , AP_NewBroadcast, _reg_no[19], 0),

    AP_GROUPEND
};


AP_NewBroadcast::AP_NewBroadcast()
{
	AP_Param::setup_object_defaults(this, var_info);
	_initialized = false;
}

AP_NewBroadcast::~AP_NewBroadcast()
{
}

void AP_NewBroadcast::init()
{
	if(_enable == NewBroadcast_TYPE_NONE)
		return;
	
	send_flag = COMPLETE;
	flight_seq_pre = _flight_seq.get();
	flight_area_m2_curr = 0;
	flight_area_m2_pre = 0;
	memset( & view, 0, sizeof(view));
	memset( & payload, 0, sizeof(payload));
	update_view_flight_control();
	update_view_reg_no();
	
	if(detect_backends())
	{
		if(drive->init())
		{
			_initialized = true;
			printf("New Broadcast init success\n");
		}
		else
		{
			delete drive;
			drive = nullptr;
			_initialized = false;
			printf("New Broadcast init false\n");
		}
	}
	else
	{
		_initialized = false;
	}
}

uint8_t AP_NewBroadcast::detect_backends()
{
	uint8_t res = 0;
	AP_NewBroadcast_Backend *new_backend = nullptr;
	
	switch(_enable)
	{
		case NewBroadcast_TYPE_NONE:
			break;
		case NewBroadcast_TYPE_GK_CAN:
			new_backend = new AP_NewBroadcast_CAN(payload);
			break;
		default:
			break;
	}

	if (new_backend != nullptr) 
	{
		drive = new_backend;
		res = 1;
	}

	return res;
}

 /*
   handle an msg_newbroadcast_str
  */
MAV_RESULT AP_NewBroadcast::handle_msg_newbroadcast_str(const mavlink_newbroadcast_str_t &packet,mavlink_channel_t chan)
{
	 MAV_RESULT result = MAV_RESULT_FAILED;
 
	 switch (packet.type) {
		 case REG_NO: {

			 update_reg_no(packet,chan);
			 result = MAV_RESULT_ACCEPTED; 
			 break;
		 }
		 case REG_NO_REQUEST: {

			 send_reg_no(chan);
			 result = MAV_RESULT_ACCEPTED; 
			 break;
		 }
			 
		 case TASK_ID: {
		 	
		 	 update_view_task_id(packet,chan);
			 result = MAV_RESULT_ACCEPTED; 
			 break;
		 }
		 case APP_VER_NO: {

			 update_view_app_ver_no(packet,chan);
			 result = MAV_RESULT_ACCEPTED;
			 break;
		 }
		 case TP_REG_NO: {

			 update_view_tp_reg_no(packet,chan);
			 result = MAV_RESULT_ACCEPTED;
			 break;
		 }
		 default:
		 	break;
	 }

	 return result;
}

void AP_NewBroadcast ::send_reg_no(mavlink_channel_t chan)
{
	int8_t reg_no[REG_NO_STRING_LEN];
		
	for(int i = 0; i < REG_NO_STRING_LEN; i++)
    {
    	reg_no[i] = _reg_no[i];  //May be wrong
    }
	
	mavlink_msg_newbroadcast_str_send(
		chan,
		REG_NO_REQUEST,
		0,
		reg_no);
}

void AP_NewBroadcast ::send_flight_status(mavlink_channel_t chan)
{

	mavlink_msg_newbroadcast_flight_sta_send(
		chan,
		view.flight_area,
		view.flight_length,
		view.flight_seq,
		view.state);

}

void AP_NewBroadcast :: update_view_action()
{
    view.action = 0;
}

void AP_NewBroadcast :: update_reg_no(const mavlink_newbroadcast_str_t &packet,mavlink_channel_t chan)
{
   	for(int i = 0; i < REG_NO_STRING_LEN; i++)
   	{
		_reg_no[i].set_and_save_ifchanged(packet.string[i]);
   	}

	mavlink_msg_newbroadcast_str_send_struct(chan,&packet);
}

void AP_NewBroadcast :: update_view_reg_no()
{
   	for(int i = 0; i < REG_NO_STRING_LEN; i++)
   	{
    	view.reg_no[i] = _reg_no[i];
	}
}

void AP_NewBroadcast ::update_view_task_id(const mavlink_newbroadcast_str_t &packet,mavlink_channel_t chan)
{
	for(int i = 0; i < TASK_ID_STRING_LEN; i++)
   	{
    	view.task_id[i] = packet.string[i];
   	}

	mavlink_msg_newbroadcast_str_send_struct(chan,&packet);
}

void AP_NewBroadcast ::update_view_app_ver_no(const mavlink_newbroadcast_str_t &packet,mavlink_channel_t chan)
{
	for(int i = 0; i < APP_VER_NO_STRING_LEN; i++)
   	{
    	view.app_ver_no[i] = packet.string[i];
   	}

	mavlink_msg_newbroadcast_str_send_struct(chan,&packet);
}

void AP_NewBroadcast ::update_view_tp_reg_no(const mavlink_newbroadcast_str_t &packet,mavlink_channel_t chan)
{
   	view.tp_reg_no = packet.tp_reg_no;
	mavlink_msg_newbroadcast_str_send_struct(chan,&packet);
}

void AP_NewBroadcast :: update_view_flight_seq()
{
    static int8_t view_step = 0;

    switch(view_step)
    {
        case 0:
            if(AP::arming().is_armed())
            {
                timer = AP_HAL::millis64();
                view_step = 1;
            }
            else
            {
                view_step = 0;
            }
            break;
        case 1:
            if(AP::arming().is_armed())
            {
                if(AP_HAL::millis64()-timer>=5000)
                {
                    _flight_seq += 1;
                    _flight_seq.set_and_save(_flight_seq);
                    view_step = 2;
                }
            }
            else
            {
                view_step = 0;
            }
            break;
        case 2:
             if(!AP::arming().is_armed())
             {
                view_step = 0;
             }
             break;
    }

    view.flight_seq = _flight_seq.get();
}

void AP_NewBroadcast :: update_view_now_time()
{
    uint32_t system_clock = 0; // in seconds
	uint64_t rtc_clock_us;
	if (AP::rtc().get_utc_usec(rtc_clock_us)) {
		system_clock = rtc_clock_us / 1000000;
		// can't store Unix seconds in a 32-bit float.	Change the
		// time base to Jan 1st 2016:
		system_clock -= 1451606400;
	}

    view.now_time = system_clock;
}

void AP_NewBroadcast :: update_view_state()
{
    static int8_t state_step = 0;

    switch(state_step)
    {
        case 0:
            if(AP::arming().is_armed())
            {
                timer = AP_HAL::millis64();
                view.state = 1;
                state_step = 1;
            }
            else
            {
                view.state = 3;
                state_step = 0;
            }
            break;
        case 1:
            if(AP::arming().is_armed())
            {
                if(AP_HAL::millis64()-timer>=5000)
                {
                    view.state = 2;
                    state_step = 2;
                }
            }
            else
            {
                view.state = 3;
                state_step = 0;
            }
            break;
        case 2:
             if(!AP::arming().is_armed())
             {
                view.state = 3;
                state_step = 0;
             }
             break;
    }

}

void AP_NewBroadcast :: update_view_flight_time()
{
	view.flight_time = AP::stats()->get_flight_time_s() * 1000;
}

void AP_NewBroadcast :: update_view_longitude()
{
    Location curr_loc;
	AP::ahrs_navekf().get_location(curr_loc);
    view.longitude = curr_loc.lng;
}

void AP_NewBroadcast :: update_view_latitude()
{
    Location curr_loc;
	AP::ahrs_navekf().get_location(curr_loc);
    view.latitude = curr_loc.lat;
}

void AP_NewBroadcast :: update_view_height()
{
	float alt_m_d;
	if (AP::ahrs().get_relative_position_D_origin(alt_m_d))
	{
	    view.height = -alt_m_d * 100;
	}
}

void AP_NewBroadcast :: update_view_altitude()
{
	Location curr_loc;
	AP::ahrs_navekf().get_location(curr_loc);
    view.altitude = curr_loc.alt*100;
}

void AP_NewBroadcast :: update_view_path_angle()
{
    view.path_angle = AP::ahrs().yaw_sensor/10;
}

void AP_NewBroadcast :: update_view_pitch_angle()
{
    view.pitch_angle = AP::ahrs().pitch_sensor/10;
}

void AP_NewBroadcast :: update_view_roll_angle()
{
    view.roll_angle = AP::ahrs().roll_sensor/10;
}

void AP_NewBroadcast :: update_view_horizontal_velocity()
{
    view.horizontal_velocity = AP::gps().ground_speed()*10;
}

void AP_NewBroadcast :: update_view_is_nozzle_work()
{
    view.is_nozzle_work = AP::sprayer()->spraying();
}

void AP_NewBroadcast :: update_view_nozzle_diameter()
{
    view.nozzle_diameter = 0;
}
void AP_NewBroadcast :: update_view_nozzle_angle()
{
    view.nozzle_angle = 0;
}

void AP_NewBroadcast :: update_view_nozzle_pressure()
{
    view.nozzle_pressure = 0;
}

void AP_NewBroadcast :: update_view_spray_range()
{
    view.spray_range = (AP::sprayer()->get_unspray_dist())/10;
}

void AP_NewBroadcast :: update_view_flight_control()
{
    // send system ID if we can
	char sysid[40];
	int8_t id_index = 0;
	bool should_log = false;
	if (hal.util->get_system_id(sysid)) {
		for(int i = 0; i < 40; i++) {
            if (sysid [i] == ' ') {
				should_log = true;
		    } else if(should_log) {
				if (id_index >= FLIGHT_CONTROL_STRING_LEN) {
					break;
				}
                view.flight_control[id_index] = sysid[i];
				id_index ++;
		    }
		}
	}
}
void AP_NewBroadcast :: update_view_remain_dose()
{
	view.remain_dose = 0;
}
void AP_NewBroadcast :: update_view_used_dose()
{
	view.used_dose = 0;
}
void AP_NewBroadcast :: update_view_cur_flow()
{
	view.cur_flow = 0;
}
void AP_NewBroadcast :: update_view_flight_area()
{
	static uint8_t step = 0;
	static int16_t wp_start = 0; //uint: meter

	switch(step)
	{
		case 0:
			if(AP::sprayer()->spraying()&& AP::arming().is_armed())
			{
				wp_start = view.flight_length;
				step = 1;
				flight_area_m2_pre += flight_area_m2_curr;
			}
			break;
		case 1:
			if(AP::sprayer()->spraying()&& AP::arming().is_armed())
			{				
				flight_area_m2_curr = (view.flight_length - wp_start)*((AP::sprayer()->get_unspray_dist())/100.0f); 			
				view.flight_area = (flight_area_m2_curr + flight_area_m2_pre)*0.015f;
			}
			else if(!AP::arming().is_armed())
			{
				flight_area_m2_pre = 0;
				flight_area_m2_curr = 0;
			}
			else
			{
				step = 0;
			}
			break;
	}
}
void AP_NewBroadcast :: update_view_flight_length()
{
	static uint8_t step = 0;
	static float velocity_xy_m = 0;
	static float flight_length = 0;

	Vector3f velNED;
    if (AP::ahrs_navekf().get_velocity_NED(velNED)) {
        velNED.z = 0; // Clear Z-axis speed
	}
	
	switch(step)
	{
		case 0:
			if(flight_seq_pre != _flight_seq.get())
			{ 
	          	velocity_xy_m = velNED.length();
				view.flight_length = 0;
				flight_length = 0;
				step = 1;
				flight_seq_pre = _flight_seq.get();
			}
			break;
		case 1:
			if(flight_seq_pre == _flight_seq.get())
			{
				//10Hz update,so velocity_xy*0.1 
				flight_length += velocity_xy_m*0.1f;
				view.flight_length = flight_length;
				velocity_xy_m = velNED.length();

				if(!AP::arming().is_armed())
				{
					step = 0;
					flight_seq_pre = _flight_seq.get();
				}
			}
			else
			{
				step = 0;
				flight_seq_pre = _flight_seq;
			}
			break;
	}
}

void AP_NewBroadcast :: update_view()
{
    update_view_action();
    update_view_reg_no();
    update_view_now_time();
    update_view_state();
    update_view_flight_time();
    update_view_longitude();
    update_view_latitude();
    update_view_height();
    update_view_altitude();
    update_view_path_angle();
    update_view_pitch_angle();
    update_view_roll_angle();
    update_view_horizontal_velocity();
    update_view_is_nozzle_work();
    update_view_nozzle_diameter();
    update_view_nozzle_angle();
    update_view_nozzle_pressure();
    update_view_spray_range();
	update_view_remain_dose();
	update_view_used_dose();
	update_view_cur_flow();
	update_view_flight_seq();
   	update_view_flight_length();
   	update_view_flight_area();
}

void AP_NewBroadcast :: update_payload()
{
	payload._payload_s.frame_header0 = 0xAA;
	payload._payload_s.frame_header1 = 0x55;
	payload._payload_s.action = view.action;

	for(int i = 0; i < REG_NO_STRING_LEN; i++)
	{		
		payload._payload_s.reg_no[i] = view.reg_no[i];
	}
	
	payload._payload_s.flight_seq = view.flight_seq;
	
	payload._payload_s.now_time = view.now_time;
	payload._payload_s.state = view.state;
	payload._payload_s.flight_time = view.flight_time;
	payload._payload_s.longitude = view.longitude;
	payload._payload_s.latitude = view.latitude;
	payload._payload_s.height = view.height;
	payload._payload_s.altitude = view.altitude;
	payload._payload_s.path_angle = view.path_angle;
	payload._payload_s.pitch_angle = view.pitch_angle;
	payload._payload_s.roll_angle = view.roll_angle;
	payload._payload_s.horizontal_velocity = view.horizontal_velocity;
	payload._payload_s.is_nozzle_work = view.is_nozzle_work;
	payload._payload_s.nozzle_diameter = view.nozzle_diameter;
	payload._payload_s.nozzle_angle= view.nozzle_angle;
	payload._payload_s.nozzle_pressure = view.nozzle_pressure;
	payload._payload_s.spray_range = view.spray_range;

	for(int i = 0; i < FLIGHT_CONTROL_STRING_LEN; i++)
	{		
		payload._payload_s.flight_control[i] = view.flight_control[i];
	}
	for(int i = 0; i < TASK_ID_STRING_LEN; i++)
	{		
		payload._payload_s.task_id[i] = view.task_id[i];
	}
	for(int i = 0; i < APP_VER_NO_STRING_LEN; i++)
	{		
		payload._payload_s.app_ver_no[i] = view.app_ver_no[i];
	}

	payload._payload_s.tp_reg_no = view.tp_reg_no;
	payload._payload_s.remain_dose = view.remain_dose;
	payload._payload_s.used_dose = view.used_dose;
	payload._payload_s.cur_flow = view.cur_flow;
	payload._payload_s.flight_area = view.flight_area;
	payload._payload_s.flight_length = view.flight_length;
	
	for(int i = 0; i < RESERVED_NUM; i++)
	{
		payload._payload_s.reserved[i] = 0;
	}
}

void AP_NewBroadcast ::update()
{	
   if(!_initialized)
		return;

   if(!_enable)
   {
       return;
   }

/*
   if(AP::gps().status() < AP_GPS::GPS_OK_FIX_3D) 
   {
   		send_last_time = AP_HAL::micros();
		return;
   }
*/

    //1Hz send 
    if(AP_HAL::micros()-send_last_time < 1000000)
        return;
	
	//TOOL:timeout detect,if true,do something,needn't use return.
	//
	//if(AP_HAL::micros()-send_last_time > 3000000)
   	
    if(send_flag == COMPLETE)
    {
        update_payload();
    }   

    if(drive == nullptr)
        return;
   
    drive->send(send_flag,send_last_time);
   
}

#endif

