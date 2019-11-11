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
 * NewBroadcast driver backend class
 *
 * @file NewBroadcast_Backend.h
 * @author Breeder Bai <songshu_bai@icloud.com>
 *
 */

#pragma once

#include "stdint.h"

#define REG_NO_STRING_LEN 20
#define PAYLOAD_ARRAY_LEN 148
#define FLIGHT_CONTROL_STRING_LEN 24
#define TASK_ID_STRING_LEN 16
#define APP_VER_NO_STRING_LEN 16
#define RESERVED_NUM 6

#define PROCESSING 0
#define COMPLETE 1 

typedef struct
{
    uint8_t action;
    char reg_no[REG_NO_STRING_LEN]; /* temporary buffer to print the number into */
    uint16_t flight_seq;
    uint32_t now_time;  //uint: s
    uint8_t state;
    uint32_t flight_time;
    int32_t longitude;
    int32_t latitude;
    int32_t height;
    int32_t altitude;
    int16_t path_angle;
    int16_t pitch_angle;
    int16_t roll_angle;
    int16_t horizontal_velocity;
    int16_t is_nozzle_work;
    int16_t nozzle_diameter;
    int16_t nozzle_angle;
    int16_t nozzle_pressure;
    int16_t spray_range;
	char flight_control[FLIGHT_CONTROL_STRING_LEN];
	char task_id[TASK_ID_STRING_LEN];
	char app_ver_no[APP_VER_NO_STRING_LEN];
	uint64_t tp_reg_no;
	uint16_t remain_dose;
	uint16_t used_dose;
	uint16_t cur_flow;
	uint16_t flight_area;    //uint: mu*10
	uint16_t flight_length;  //uint: m
}Message_info;

#pragma  pack (push,1)
typedef struct
{
	uint8_t frame_header0;
	uint8_t frame_header1;
    uint8_t action;
    char reg_no[REG_NO_STRING_LEN]; /* temporary buffer to print the number into */
    uint16_t flight_seq;
    uint32_t now_time;      //uint: s
    uint8_t state;
    uint32_t flight_time;
    int32_t longitude;
    int32_t latitude;
    int32_t height;
    int32_t altitude;
    int16_t path_angle;
    int16_t pitch_angle;
    int16_t roll_angle;
    int16_t horizontal_velocity;
    int16_t is_nozzle_work;
    int16_t nozzle_diameter;
    int16_t nozzle_angle;
    int16_t nozzle_pressure;
    int16_t spray_range;
	char flight_control[FLIGHT_CONTROL_STRING_LEN];
	char task_id[TASK_ID_STRING_LEN];
	char app_ver_no[APP_VER_NO_STRING_LEN];
	uint64_t tp_reg_no;
	uint16_t remain_dose;
	uint16_t used_dose;
	uint16_t cur_flow;
	uint16_t flight_area;    //uint: mu*10
	uint16_t flight_length;  //uint: m
	uint8_t reserved[RESERVED_NUM];
}Payload_s;

typedef union
{
	Payload_s _payload_s;
	char paylod_array[PAYLOAD_ARRAY_LEN];
}Message_send_union;

#pragma pack(pop)  

enum frameType
{
    STANDARDID,
    EXTENDID
};

class AP_NewBroadcast_Backend {
public:
	//AP_NewBroadcast_Backend();
    virtual ~AP_NewBroadcast_Backend() {}

	// init - initialised the device
    virtual uint8_t init(void) = 0;

	//Send agreement content
	virtual uint8_t send(bool &send_flag,uint64_t &send_last_time) = 0;

	uint16_t crc16_ccitt(const char *buf, int len);
	
private:
	
	static const uint16_t crc16tab[];
};

