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
 * @file AP_NewBroadcast_CAN.h
 * @author Breeder Bai <songshu_bai@icloud.com>
 *
 */

#pragma once

#include "stdio.h"
#include <AP_SerialManager/AP_SerialManager.h>
#include "NewBroadcast_Backend.h"

class AP_NewBroadcast_serial : public AP_NewBroadcast_Backend 
{
public:
	
	AP_NewBroadcast_serial(Message_send_union &_payload);
   	~AP_NewBroadcast_serial() {}

	// init - initialised the device
   	uint8_t init(void) override;

	//Send agreement content
	uint8_t send(bool &send_flag,uint64_t &send_last_time) override;

	//Send paylod array and crc16
	uint8_t send(char *pArray,uint16_t len,bool &send_flag,uint64_t &send_last_time);

	bool sendDataStream(frameType type,char *pArray,uint16_t index,uint8_t length);
	bool sendCrc16(frameType type);
	
private:
	AP_HAL::UARTDriver *uart = nullptr;

	Message_send_union &payload;
};

