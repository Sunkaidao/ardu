#pragma once

#include <AP_Math/AP_Math.h>
#include <AP_SerialManager/AP_SerialManager.h>



class AP_Gassensor
{
public:
	

	AP_Gassensor();

//	int8_t enabled() {return _enabled.get();}
	
	void SendCMD(uint8_t CMD);
	
	void update(const AP_SerialManager& serial_manager);
	~AP_Gassensor();
	void init(const AP_SerialManager& serial_manager); 
	void get_sensor12();//Read packet --12
	void get_sensor6();//Read packet --6
	uint32_t  merge(unsigned char high,unsigned char low);//Get the decimal value of two hexadecimal numbers
	unsigned short CRC16(unsigned char *puchMsg, unsigned int usDataLen);//Find the value of crc(modbus)
	void InvertUint8(unsigned char *DesBuf, unsigned char *SrcBuf);
	void InvertUint16(unsigned short *DesBuf, unsigned short *SrcBuf);	
	float square(float number,uint16_t power);//Find the "power" square of "number"
	
	uint8_t* get_rx_date_6(){return mav_data6;}
	uint8_t* get_rx_date_12(){return mav_data12;}

	float* get_sensor12_data(){return Sensor12_data;}

protected:
	AP_HAL::UARTDriver *uart;
	
	bool _initialised;

private:
	// parameters
 //   AP_Int8         _enabled;               // top level enable/disable control
	
	uint8_t Tx_Buff[10];

	uint8_t rx_data12[29];//rx	sensor
	uint8_t rx_data6[65];

	uint8_t mav_data12[29];
	uint8_t mav_data6[65];
	//unsigned char rx_data12[29];
	
	//the sensor data 
	
	float Sensor12_data[13];// 03 80	--data	last is CRC
	/*
	float Sensor6_data[7];// 03 00 --data   last is CRC
	uint16_t Sensor6_name[7];//03 00 --name
	uint16_t Sensor6_unit[7];//03 00 --unit
	uint16_t Sensor6_float[7];//03 00 --Decimal point
	uint16_t Sensor6_warning[7];//03 00 -- warning
	*/
	uint32_t Sen_flag;
	uint8_t rxdata12_len;
	uint8_t rxdata6_len;
	uint16_t rx_len_flag;
	int readdata;
	//uint8_t tx_six;
	uint8_t turn_read;

	enum Gassensor_CMD {
			fixed = 0x00,
			onboard =0x01
		};

	
};
