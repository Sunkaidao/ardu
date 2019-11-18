#include "AP_Flowmeter/AP_Flowmeter_WL.h"
#include "./../ArduCopter/Copter.h"

extern const AP_HAL::HAL& hal;

AP_Flowmeter_WL::AP_Flowmeter_WL(AP_Flowmeter &frontend,AP_Flowmeter::Flowmeter_State &status,AP_Flowmeter::Flowmeter_data &data,const AP_InertialNav* inav,AC_Sprayer* sprayer,const AC_WPNav* wp_nav):
	AP_Flowmeter_Backend(frontend,status,data),
	_sprayer(sprayer),
	_inav(inav),
	_wp_nav(wp_nav)
{
	res = 0 ;
}
AP_Flowmeter_WL::~AP_Flowmeter_WL(void)
{

}
void AP_Flowmeter_WL::init()
{

}


void AP_Flowmeter_WL::update()
{
	flow_drivers_update();
	if(res != 1)
		return;
	calculate();
////////////////////////////////
//Add short sides here
//if short != 1 	
//NO_drug
////////////////////////////////
	NO_drug();
	control(expect());
	_sprayer->set_flow(get_type(),actual_pump_rate);
}
void AP_Flowmeter_WL::calculate()
{
	_data._pulses_each = _status._pulse_val - _data._pulses_num;
	_data._pulses_num = _status._pulse_val;
	_data._volume = _status._pulse_val*100/get_coeffcient();
	_data._pulses_time = _status.dt_ms;
	_data._flow_rate = 500000*4*100/_status.dt_ms/get_coeffcient();
	_data._pulses_each = _status._pulses_each;
	_data._warning = rtf._return;
	_data._expect = expect_output;
	_data._output = actual_pump_rate;
	//printf("\n%d 	%d	%d\nrate is %d amount is %d pulse each is %d\n",get_pin(),_data._pulses_num,_data._time,_data._flow_rate,_data._volume,_data._pulses_each);
}
uint16_t AP_Flowmeter_WL::expect()
{
	const Vector3f &velocity = _inav->get_velocity();
    float ground_speed = norm(velocity.x,velocity.y);//cm
    float max_speed = _wp_nav->get_default_speed_xy();
	if(max_speed == 0)
		max_speed = 500;
//	printf("	max_speed is %f		ground_speed is %f\n ",max_speed,ground_speed);
	if(_sprayer->spraying()==0||_sprayer->running()==0)
	{
		return 0;
	}
	else if(2 == get_type())//Fixed speed mode
	{
		//Problems start small spray
		
		float output;
		float input_coefficient;
		float out_max;
			
		out_max=((max_speed*(_sprayer->get_unspray_dist()))*get_amount_area()/6666666);
		input_coefficient=ground_speed/(max_speed-0);
		if(get_slope()!=0)
			output = ((get_slope()-1.0f) + safe_sqrt((1.0f-get_slope())*(1.0f-get_slope()) + 4.0f*get_slope()*input_coefficient))/(2.0f*get_slope());
		else
			output = input_coefficient;
	
		expect_output=output*out_max;
		return expect_output;
	}	
	else
	{
		return	get_slope();
	}
}

void AP_Flowmeter_WL::control(uint16_t _expect)
{
		float	control_P;

		if(_expect == 0)
		{
			actual_pump_rate = get_output_min();
			return;
		}
		difference = _expect - _data._flow_rate;
		control_P =get_P().get_p(difference);
		difference=control_P;
		if(difference<0)
		{	
			if(actual_pump_rate>0)
			actual_pump_rate=actual_pump_rate+difference;
			if(actual_pump_rate<0)
				actual_pump_rate=0;
		}
		else if(difference>0)
		{	
			if(actual_pump_rate<10000)
			actual_pump_rate=actual_pump_rate+difference;
			if(actual_pump_rate>10000)
				actual_pump_rate=10000;
		}
		else
			{
	
		}
		if(actual_pump_rate < get_output_min())
			actual_pump_rate = get_output_min();
		return;
}

void AP_Flowmeter_WL::NO_drug(){
	if(_sprayer->spraying()==1&&_sprayer->running()==1&&_data._pulses_each==0)
	{
		rtf.delay++;
		if(rtf.delay>5)
		{
			rtf.flag=1;
			rtf.delay=0;
		}
	}
	else
	{
		rtf.flag=0;
		rtf.delay=0;
		_data._heart_beat++;
	}
		
	if(rtf.last_flag<rtf.flag)
		rtf._return=1;
	else
		rtf._return=0;
	
	if(rtf._return == 1)
	{
		if(0 == _frontend.get_armed())
			rtf.last_flag=rtf.flag;
	}
	else	
		rtf.last_flag=rtf.flag;
		
}

	
void AP_Flowmeter_WL::update_pin()
{
	int8_t pin = get_pin();
	if (last_pin != pin) {
		// detach from last pin
		if (last_pin != -1) {
			hal.gpio->detach_interrupt(last_pin);
		}
		// attach to new pin
		last_pin = pin;
		if (last_pin > 0) {
			hal.gpio->pinMode(last_pin, HAL_GPIO_INPUT);
			if (!hal.gpio->attach_interrupt(
					last_pin,
					FUNCTOR_BIND_MEMBER(&AP_Flowmeter_WL::irq_handler,
										void,
										uint8_t,
										bool,
										uint32_t),
					AP_HAL::GPIO::INTERRUPT_RISING)) {
					
					printf("error\n",last_pin);
				gcs().send_text(MAV_SEVERITY_WARNING, "FuelLevelPWM: Failed to attach to pin %u", unsigned(last_pin));
			}
		else
		{
			res = 1;
		}
		}
	}
}
void AP_Flowmeter_WL::flow_drivers_update(void)
{
	if(res != 1)
    update_pin();

	void *irqstate = hal.scheduler->disable_interrupts_save();
	//printf("\nlast_reading_ms is %d time_flag is %d\nreturn is %d\n",irq_state.last_reading_ms,irq_state.pulse_four_time_flag,irq_state.pulse_four_time);
	copy_state_to_frontend(	   irq_state.total_count,
							   irq_state.pulse_four_time);
	hal.scheduler->restore_interrupts(irqstate);
}

void AP_Flowmeter_WL::irq_handler(uint8_t pin,
                                             bool pin_value,
                                             uint32_t timestamp)
{
    // sanity check    
    if (last_pin == 0) {
		
        return;
    }

    // update distance and error counts
    if (pin == last_pin) {
        last_pin_value = pin_value;
    } 
	else {
        return;
    };
    // record update time
    if(irq_state.pulse_four_flag<4)
    {
    	irq_state.pulse_four_flag++;
		irq_state.pulse_four_time_flag = 0;		
    }
	else
	{
		irq_state.pulse_four_time_flag = timestamp - irq_state.last_reading_ms;
		irq_state.pulse_four_time = irq_state.pulse_four_time_flag;
		irq_state.last_reading_ms = timestamp;//61490215 1min
		irq_state.pulse_four_flag = 0;
	}
	irq_state.total_count++;
	//printf("			irq%d\n",irq_state.total_count);
}

