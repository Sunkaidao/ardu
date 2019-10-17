#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>

#include <AC_PID/AC_P.h>

#include <AC_Sprayer/AC_Sprayer.h>
#include <AP_InertialNav/AP_InertialNav.h>     // Inertial Navigation library
#include <AC_WPNav/AC_WPNav.h>           // ArduCopter waypoint navigation library
#include "Flowmeter_Backend.h"

#include <Filter/Filter.h>


class AP_Flowmeter_WL : public AP_Flowmeter_Backend
{
public:
	AP_Flowmeter_WL(AP_Flowmeter &frontend,AP_Flowmeter::Flowmeter_State &status,AP_Flowmeter::Flowmeter_data &data,const AP_InertialNav* inav,AC_Sprayer* sprayer,const AC_WPNav* wp_nav);
	~AP_Flowmeter_WL(void);
	void init() override;
	void update() override;
	void calculate();
	uint16_t expect();
	void control(uint16_t _expect);
	void NO_drug();
	void rtl_do();

protected:
    // parameters for each instance

private:
	
	//uint8_t detect_backends();	
	AC_Sprayer* _sprayer;	
	const AP_InertialNav* const _inav;
	const AC_WPNav* const _wp_nav;
		
	uint8_t res;//init flag
	uint16_t expect_output;
	int32_t difference; 		// actual acceleration
	int32_t actual_pump_rate;

	struct rtfflag{
	uint8_t		delay;
	bool		flag;
	bool		last_flag;
	bool		_return;
	}rtf;

		
    // check if pin has changed and initialise gpio event callback
    void update_pin();

    // gpio interrupt handlers
    void irq_handler(uint8_t pin, bool pin_value, uint32_t timestamp);  // combined irq handler

    // convert pin a and b status to phase
	void flow_drivers_update();

    struct IrqState {
        uint8_t  phase;             // current phase of encoder (from 0 to 3)
        int32_t  distance_count;    // distance measured by cumulative steps forward or backwards since last update
        uint32_t total_count;       // total number of successful readings from sensor (used for sensor quality calcs)
        uint32_t error_count;       // total number of errors reading from sensor (used for sensor quality calcs)
        uint32_t last_reading_ms;   // system time of last update from encoder
        uint8_t	 pulse_four_flag;
		uint32_t pulse_four_time_flag;
		uint32_t pulse_four_time;
    } irq_state;

    // private members
    int8_t last_pin = -1;

    uint8_t last_pin_value;
};


