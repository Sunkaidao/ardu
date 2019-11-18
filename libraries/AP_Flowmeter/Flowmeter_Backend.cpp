
#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include "Flowmeter_Backend.h"

// base class constructor.
AP_Flowmeter_Backend::AP_Flowmeter_Backend(AP_Flowmeter &frontend,AP_Flowmeter::Flowmeter_State &status,AP_Flowmeter::Flowmeter_data &data):
	_frontend(frontend),
	_status(status),
	_data(data)

{

}

AP_Flowmeter_Backend::~AP_Flowmeter_Backend()
{

}


// copy state to front end helper function
void AP_Flowmeter_Backend::copy_state_to_frontend(uint32_t total_count, uint32_t last_reading_ms)
{
/*
    if(_status.last_reading_ms == last_reading_ms)
		_status.dt_ms = 0 ;
	else
*/
    _status.dt_ms = last_reading_ms;
//    _status.last_reading_ms = last_reading_ms;

	_status._pulses_each = total_count - _status._pulse_val;
	_status._pulse_val = total_count;
}


