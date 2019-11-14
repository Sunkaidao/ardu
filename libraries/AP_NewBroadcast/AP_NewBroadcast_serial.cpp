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
 * @file AP_NewBroadcast_CAN.cpp
 * @author Breeder Bai <songshu_bai@icloud.com>
 *
 */

#include <AP_HAL/AP_HAL.h>
#include "AP_NewBroadcast_serial.h"
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

AP_NewBroadcast_serial::AP_NewBroadcast_serial(Message_send_union &_payload) :
	payload(_payload)
{
}

uint8_t AP_NewBroadcast_serial::init()
{
	uint8_t res = 0;
		
	const AP_SerialManager &serial_manager = AP::serialmanager();
    uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_NewBroadcast, 0);
    if (uart != nullptr) {
        uart->begin(serial_manager.find_baudrate(AP_SerialManager::SerialProtocol_NewBroadcast, 0));
		res = 1;
    }

	//res, 0:failure,1:success
	return res;
}

uint8_t AP_NewBroadcast_serial :: send(bool &send_flag,uint64_t &send_last_time)
{
    return send(payload.paylod_array,PAYLOAD_ARRAY_LEN,send_flag,send_last_time);
}

//Send paylod array and crc16
uint8_t AP_NewBroadcast_serial :: send(char *pArray,uint16_t len,bool &send_flag,uint64_t &send_last_time)
{
    uint8_t res = 0;
	
    if (uart == nullptr) {
        return false;
    }

	uint16_t crc16 = crc16_ccitt(payload.paylod_array,PAYLOAD_ARRAY_LEN);
	uint8_t crc16_l = crc16 & 0x00FF;
	uint8_t crc16_h = (crc16 & 0xFF00) >> 8;
	
    // read any available lines from the lidar
    uint32_t nbytes = uart->txspace();
	if (nbytes > PAYLOAD_ARRAY_LEN +2) {
        uart->write((const uint8_t *)pArray, len);
		uart->write(crc16_l);
        uart->write(crc16_h);
	    send_last_time = AP_HAL::micros();
		res = 1;
	}
	
	return res;
}


