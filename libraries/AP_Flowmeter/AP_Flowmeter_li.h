#pragma once

#include <AP_Math/AP_Math.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <stdio.h>
#include <AP_Param/AP_Param.h>


#include "AP_Flowmeter/Flowmeter_Backend.h"


#define AP_FLOWMETER_EXHAUSTED_SHRESHOLD		3

class AP_Flowmeter_li : public AP_Flowmeter_Backend
{
public:
	
	enum Flowmeter_CMD {
		Read_Data = 0x00,
		LIQ_RESET = 0x01,
		Enter_Calib = 0x02,
		Quit_Calib = 0x03
	};
	enum error_Flowmeter {
		Right_data = 0,
		Head_error = 1,
		Invalid_data = 2,
		Parity_error = 3
	};

	AP_Flowmeter_li(AP_Flowmeter &frontend,AP_Flowmeter::Flowmeter_State &status,AP_Flowmeter::Flowmeter_data &data,AC_Sprayer* _sprayer,const AP_SerialManager& _serial_manager);

	~AP_Flowmeter_li();


//	int8_t enabled() {return _enabled.get();}
	uint16_t get_volume();
	uint16_t get_high();
	void SendCMD(uint8_t CMD);
	int GetDate();
	void get_Flowmeter_data();
	void update() override;
		//const AP_SerialManager& serial_manager);
	void init() override;
	//const AP_SerialManager& serial_manager);

	uint8_t get_warning();
	uint8_t get_packet_cnt();

	bool exhausted();
//added by xusiming in 20180821 and used for counting down
	void get_sprayer_state(uint8_t state);
	uint8_t get_time();
//added end 
	void read() override;
	//static const struct AP_Param::GroupInfo var_info[];

protected:
	AP_HAL::UARTDriver *_port;
	
	bool _initialised;

private:
	uint8_t Tx_Buff[4];
	uint8_t Rx_Buff[11];
	const AP_SerialManager& serial_manager;
	AC_Sprayer* sprayer;

	struct Flowmeter_data
	{
		uint16_t high;//depend on the document in20180515 and the uint is 0.01cm
		uint16_t volume;//depend on the document in20180515 and the uint is 0.01cm
		//double high;
		//double volume;
		uint8_t	warning;
		uint8_t flag;
		uint8_t	mode;
		uint8_t packet_cnt;
		int8_t time;
	}_Flo_data;


	
	struct Time_Flowmeter_error
	{
		uint8_t Time_Head_error;
		uint8_t Time_Invalid_data;
		uint8_t Time_Parity_error;
	}_num_error;
	uint8_t state_tim;
	uint8_t sprayer_state;
	



};
