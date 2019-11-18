#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include "AP_Flowmeter.h"

class AP_Flowmeter_Backend
{
public:
    // constructor. This incorporates initialisation as well.
	AP_Flowmeter_Backend(AP_Flowmeter &frontend,AP_Flowmeter::Flowmeter_State &status,AP_Flowmeter::Flowmeter_data &data);

    // we declare a virtual destructor so that WheelEncoder drivers can
    // override with a custom destructor if need be
    ~AP_Flowmeter_Backend();

    // update the state structure. All backends must implement this.
    virtual void update() = 0;
	virtual void read(){};

	
    virtual void init() = 0;

protected:
	
	uint8_t get_type(){return _frontend._type.get();}
	uint8_t get_pin(){return _frontend._pin.get();}
	uint16_t get_amount_area(){return _frontend._amount_area.get();}
		
	uint32_t get_output_min(){return _frontend._output_min.get();}

	uint8_t get_coeffcient(){return _frontend._coefficient.get();}
	
	uint8_t get_slope(){return _frontend._slope.get();}
	uint8_t get_rtl_switch(){return _frontend._rtl_switch.get();}
	AC_P get_P(){return _frontend._P;}

	

	uint8_t get_time_backend(){return _frontend._time.get();}
	uint8_t get_alarm_backend(){return  _frontend._alarm.get();}
	uint16_t get_high_backend(){return _frontend._high.get();}
	uint16_t get_volume_backend(){return _frontend._volume.get();}



	void copy_state_to_frontend(uint32_t total_count, uint32_t last_reading_ms);
    
    AP_Flowmeter &_frontend;
    AP_Flowmeter::Flowmeter_State &_status;
	AP_Flowmeter::Flowmeter_data &_data;

};

