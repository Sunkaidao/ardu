#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>

#include <AC_PID/AC_P.h>

#include <AC_Sprayer/AC_Sprayer.h>
#include <AP_InertialNav/AP_InertialNav.h>     // Inertial Navigation library
#include <AC_WPNav/AC_WPNav.h>           // ArduCopter waypoint navigation library

typedef enum MAVLINK_PAYLOD{
	MAVLINK_PAYLOD_NULL=0,
	MAVLINK_PAYLOD_START=1,
	MAVLINK_PAYLOD_STOP=2

}MAVLINK_PAYLOD;


class AP_Flowmeter_Backend; 
 
class AP_Flowmeter
{
public:
    friend class AP_Flowmeter_Backend;
	friend class AP_Flowmeter_WL;
	
	friend class AP_Flowmeter_li;
	struct Flowmeter_State {
		
		uint32_t			   last_reading_ms; // time of last reading
		uint32_t			   dt_ms;			  // time change (in milliseconds) for the previous period (used to calculating rate)
		uint32_t				_pulse_val;
		uint32_t				_time;
		uint32_t				_pulses_each;
	};
	
	struct Flowmeter_data {
		uint32_t	_pulses_num;
		uint16_t	_volume;
		uint32_t	_pulses_each;
		uint8_t 	_flow_rate;
		uint32_t	_time;
		//uint32_t	_package_val;
		uint32_t 	_heart_beat;
		uint8_t		_warning;
		uint8_t		_height;
		uint16_t	_expect;
		uint32_t	_output;
	};


	AP_Flowmeter(const AP_InertialNav* inav,AC_Sprayer* sprayer,const AC_WPNav* wp_nav);
	~AP_Flowmeter(void);
	void init(void);
	void update(void);
	void NO_drug();
	void rtl_do();
	Flowmeter_data get_data(){return data;}
	uint8_t get_type(){return _type.get();}
	uint8_t get_armed();
	
	void set_coeffcient(float coe);
	
	uint16_t get_adjustment_amount(){return _adjustment_amount;}
static const struct AP_Param::GroupInfo var_info[];

protected:
    // parameters for each instance

private:
	
	uint8_t detect_backends();
	Flowmeter_State status;
	Flowmeter_data data;
	AP_Flowmeter_Backend *drivers;	
	
	AC_Sprayer* _sprayer;	
	const AP_InertialNav* const _inav;
	const AC_WPNav* const _wp_nav;
	
	AC_P	_P;
    AP_Int8		_pin;
    AP_Int8		_type;
	AP_Int16	_amount_area;
	AP_Int32	_output_min;
	AP_Int16	_coefficient;
	AP_Int8		_slope;
	AP_Int8		_rtl_switch;
	AP_Int32	_adjustment_amount;

	AP_Int16        _volume;                //if the sensor reach the number of volume that the user want to send warning message
	AP_Int16        _high;                  //if the sensor reach the number of level that the user want to send warning message
	AP_Int8         _alarm;              //the type of message that the user want to use
	AP_Int8         _time;               //delay time
		
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
};
