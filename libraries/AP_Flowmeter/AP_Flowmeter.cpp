#include "AP_Flowmeter.h"
#include <AP_Logger/AP_Logger.h>

#include "./../ArduCopter/Copter.h"
#include "AP_Flowmeter/AP_Flowmeter_WL.h"
#include "AP_Flowmeter/AP_Flowmeter_li.h"

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_Flowmeter::var_info[] = {
    // @Param: 2_PINB
    // @DisplayName: Second Encoder Input Pin B
    // @Description: Second Encoder Input Pin B
    // @Values: -1:Disabled,50:PixhawkAUX1,51:PixhawkAUX2,52:PixhawkAUX3,53:PixhawkAUX4,54:PixhawkAUX5,55:PixhawkAUX6
    // @User: Standard
    AP_GROUPINFO("PIN",   1, AP_Flowmeter, _pin, 54),

	AP_GROUPINFO("TYPE",   2, AP_Flowmeter, _type, 1),

	AP_GROUPINFO("COE",3,AP_Flowmeter,_coefficient,200),
	
	
    AP_SUBGROUPINFO(_P, "", 4, AP_Flowmeter, AC_P),

	
	AP_GROUPINFO("SLOPE",5,AP_Flowmeter,_slope,1),

	
	AP_GROUPINFO("AMOUNT",6,AP_Flowmeter,_amount_area,1000),

	
	AP_GROUPINFO("OUTPUT_MIN",7,AP_Flowmeter,_output_min,2000),


	AP_GROUPINFO("RTL_SWITCH",8,AP_Flowmeter,_rtl_switch,1),

	
	
    AP_GROUPINFO("VOLUME", 9,  AP_Flowmeter, _volume, 0),
    AP_GROUPINFO("HIGH", 10,  AP_Flowmeter, _high, 0),
    AP_GROUPINFO("ALARM", 11,  AP_Flowmeter, _alarm, 0),
    AP_GROUPINFO("TIME",   12,  AP_Flowmeter, _time, 0),
    
	AP_GROUPINFO("CAL_AM",13,	AP_Flowmeter, _adjustment_amount,0), 

    AP_GROUPEND
};


AP_Flowmeter::AP_Flowmeter(const AP_InertialNav* inav,AC_Sprayer* sprayer,const AC_WPNav* wp_nav):
	_sprayer(sprayer),
	_inav(inav),
	_wp_nav(wp_nav),
	_P(200)
{
	res = 0 ;
}
AP_Flowmeter::~AP_Flowmeter(void)
{

}
void AP_Flowmeter::init(void)
{
	if(detect_backends()==1)
	{
		res=1;
	}
}

uint8_t AP_Flowmeter::detect_backends()
{
	AP_Flowmeter_Backend *new_backend = nullptr;
		
	switch(_type)
	{
		case 1:
			new_backend = new AP_Flowmeter_li(*this,status,data,_sprayer,copter.serial_manager);
			//printf("AP_Flowmeter_WL new\n");
			break;
		case 2:
			new_backend = new AP_Flowmeter_WL(*this,status,data,_inav,_sprayer,_wp_nav);
			//printf("AP_Flowmeter_WL new\n");
			break;
		default:
			break;
	}
	if (new_backend != nullptr) 
	{
		drivers = new_backend;
		return 1;
	}
	return 0;


}




void AP_Flowmeter::update(void)
{
	if(0 == _type)
	{
		return;
	}
	if(res == 0)
	{
		init();
		return;
	}
	if(drivers == nullptr)
		return;
	drivers->update();	
/////////////////////////////////
//	No drugs return add here
//	rtl_do;
////////////////////////////////
	//rtl_do();
}
void AP_Flowmeter::rtl_do()
{
////////////////////////////////
//_rtl_switch is switch  change to 
//data._warning is no drugs flag
//_type 2 is wl  1 is GKXN li
//close sprayer run,test send text payload emergency!
////////////////////////////////

	if(\
		(_rtl_switch == 1) && \
		(data._warning== 1) && \
		(2 ==_type)  && \
		copter.motors->armed()
	)
	{
		gcs().send_text(MAV_SEVERITY_CRITICAL, "payload emergency!");
		copter.sprayer.run(false);		
		copter.sprayer.test_pump(false);
		copter.set_mode(RTL,MODE_REASON_FLOWMETER_PAYLOAD);
	}

}
uint8_t AP_Flowmeter::get_armed()
{	
	return copter.motors->armed();
}
////////////////////
//about calibration
//change _coefficient
///////////////////
void AP_Flowmeter::set_coeffcient(float coe)
{
	_coefficient.set_and_save(coe);

}

#if GROUPING == ENABLED
void AP_Flowmeter::spray_amount(uint8_t grouping)
{
	if(grouping > 30 || grouping == 0)
		group_amount = grouping;
	else
		group_amount = 30;
}
#endif


