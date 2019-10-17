#include <AP_Flowmeter/AP_Flowmeter_li.h>


extern const AP_HAL::HAL& hal;

AP_Flowmeter_li::AP_Flowmeter_li(AP_Flowmeter &frontend,AP_Flowmeter::Flowmeter_State &status,AP_Flowmeter::Flowmeter_data &data,AC_Sprayer* _sprayer,const AP_SerialManager& _serial_manager):
	AP_Flowmeter_Backend(frontend,status,data),
	sprayer(_sprayer),
	serial_manager(_serial_manager)
{
	_initialised = false;
	
	//AP_Param::setup_object_defaults(this, var_info);

//	printf("0.%d", _initialised);
	
}

AP_Flowmeter_li::~AP_Flowmeter_li()
{}


void AP_Flowmeter_li::init()
{
	uint8_t i;
	//printf("flowmeter_li %d\n", _initialised);

	if(1 != get_type())
		return;

	
	_port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_FlowMeter_GKXN, 0);

	
	if(nullptr != _port)
	{
		

	
		for(i=0;i<4;i++)
		{
			Tx_Buff[i]=0;
		}
		for(i=0;i<11;i++)
		{
			Rx_Buff[i]=0;
		}
		_Flo_data.high=0;
		_Flo_data.volume=0;
		_Flo_data.warning=0;
		_Flo_data.mode=0;
		_Flo_data.time=get_time_backend();
		_Flo_data.packet_cnt = 0;
		_Flo_data.flag=0;
  	 	 state_tim=0;
   	 	_num_error.Time_Head_error=0;
   		_num_error.Time_Invalid_data=0;
    	_num_error.Time_Parity_error=0;

		_initialised = true;


		
	}

	//printf("1.%d\n", _initialised);
}



void AP_Flowmeter_li::SendCMD(uint8_t CMD)
{
	uint8_t checkByte=0;
	uint8_t i;

	if(false == _initialised)
		return;
	
	Tx_Buff[0]=0x55;
	Tx_Buff[1]=0x03;
	Tx_Buff[2]=CMD;
	checkByte=0x55+0x03+CMD;
	Tx_Buff[3]=checkByte;
	
	for(i=0;i<4;i++)
    	_port->write(Tx_Buff[i]);
}

int AP_Flowmeter_li::GetDate()
{
	uint16_t j;
	uint8_t checksum = 0x00;
	_Flo_data.warning=0;


	if((Rx_Buff[0] != 0x55) && (Rx_Buff[1] != 0x03) && (Rx_Buff[2] != 11))
	{
		_num_error.Time_Head_error++;
		return	Head_error;
	}
	if( Rx_Buff[9] != 0)
	{
		_num_error.Time_Invalid_data++;
		return	Invalid_data;
	}
	for(j=0;j<10; j++)
		checksum += Rx_Buff[j];
	if(checksum != Rx_Buff[10])
	{
	    _num_error.Time_Parity_error++;
		return	Parity_error;
	}
	_Flo_data.high =(Rx_Buff[3]<<8)+Rx_Buff[4];
	_Flo_data.volume =(Rx_Buff[5]<<8)+Rx_Buff[6];
	_Flo_data.flag = Rx_Buff[7];
	_Flo_data.mode = Rx_Buff[8];

	if((get_alarm_backend()&0x01)==0x01)
	{
		if(_Flo_data.flag!=0)
		{
			_Flo_data.warning=1;
		}
	}
	if(((get_alarm_backend()>>1)&0x01)==0x01)
	{
		if (_Flo_data.volume<get_volume_backend())
		{
			_Flo_data.warning=1;
		}
	}
	if(((get_alarm_backend()>>2)&0x01)==0x01)
	{
		if (_Flo_data.high<get_high_backend())
		{
				_Flo_data.warning=1;
		}
	}
	
	_Flo_data.packet_cnt ++;
	return Right_data;
}
void AP_Flowmeter_li::get_Flowmeter_data()
{
	uint16_t i;
	int16_t numc;
	
	//	wite someting to flsuh the buffer of UART
	

	numc = _port->available();

//	printf("%d\n", numc);
	
	if(0 == numc)
		return;

	for (i = 0; i < numc; i++) {        // Process bytes received
        Rx_Buff[i] = _port->read();

//		printf("0x%x\n", Rx_Buff[i]);
	}

	
	if(GetDate()==Right_data)
	{
//		printf("liquid high 	  is %f\n",_Flo_data.high);
//		printf("liquid volume  is %f\n",_Flo_data.volume);
//		printf("liquid warning is %x\n",_Flo_data.warning);
//		printf("liquid mode 	  is %x\n",_Flo_data.mode);
	}
}

void AP_Flowmeter_li::update()
{

//	printf("2.%d\n", _initialised);
	if(1 != get_type())
		return;

	//	to support warm plug
	if(!_initialised)
	{
		init();
	}

	if(!_initialised)
		return;
//	hal.uartD->printf("the pulse count:\n");


	get_Flowmeter_data();

//	if(state_tim%100==0)
//	{
		//hal.uartD->printf("Have output the CMD!\n");
	SendCMD(Read_Data);
		//hal.uartD->printf("Please input: \n");
//		state_tim=0;
//	}
//	printf("_Flo_data.warning=%d\n",_Flo_data.warning);
//	state_tim++;

}


uint8_t AP_Flowmeter_li::get_warning()
{

	if(1 != get_type())
		return 0;

	if(0 == _initialised)
		return 0;

	//printf("_Flo_data.warning=%d\n",_Flo_data.warning);
	
	return _Flo_data.warning;
}


bool AP_Flowmeter_li::exhausted()
{
	static uint8_t lcl_cnt = 0;

	if(1 != get_type())
	{
		return false;
	}

	if(!_initialised)

	{
		return false;
	}
	if(1 == _Flo_data.warning)
	{
		if(sprayer->running()==1)
			{
				lcl_cnt ++;
				if(lcl_cnt > (AP_FLOWMETER_EXHAUSTED_SHRESHOLD*get_time_backend()))
				{
				lcl_cnt = 0;
				_Flo_data.time=0;
				return true;
				}
				else
				{
					if(lcl_cnt%3==0)
					{
						if(_Flo_data.time<=0)
							_Flo_data.time=0;
						else _Flo_data.time=get_time_backend()-(lcl_cnt/3);
					}
				}
		}
	}
	else if(0 == _Flo_data.warning)
	{
		lcl_cnt = 0;
		_Flo_data.time=get_time_backend();
	}
	else
	{
//		printf("unsupportted value %d\n", _Flo_data.warning);
	}


	return false;
	
}


uint8_t AP_Flowmeter_li::get_packet_cnt()
{

	if(1 != get_type())
	{
		return false;
	}

	if(!_initialised)

	{
		return false;
	}

	return _Flo_data.packet_cnt;
}

uint16_t AP_Flowmeter_li::get_volume()
{

	
if(1 != get_type())
		return 0;

	if(0 == _initialised)
		return 0;
		
	return _Flo_data.volume;
}
uint16_t AP_Flowmeter_li::get_high()
{


	if(1 != get_type())
			return 0;
	
	if(false == _initialised)
			return 0;

	return _Flo_data.high;
}
//added by xusiming in 20180821 and used for counting down
void AP_Flowmeter_li::get_sprayer_state(uint8_t state)
{
	sprayer_state=state;
}
uint8_t AP_Flowmeter_li::get_time()
{

	if(1 != get_type())
				return 0;
		
	if(false == _initialised)
				return 0;
				
	return _Flo_data.time;
}
//added end
void AP_Flowmeter_li::read(){
		if(false == _initialised)
		return;
		_data._warning=get_warning();
		_data._heart_beat = get_packet_cnt();
		_data._volume = get_volume();
		_data._height = get_high();
		_data._time = get_time();
}
