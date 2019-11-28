#include <AP_Gassensor/AP_Gassensor.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_Math/crc.h>



AP_Gassensor::AP_Gassensor()
{
	_initialised = false;
	turn_read = 0;
//	printf("0.%d", _initialised);
	
}

AP_Gassensor::~AP_Gassensor()
{}


void AP_Gassensor::init(const AP_SerialManager& serial_manager)
{
	//if(0 == _enabled)
		//return;
	int i;
	uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_Airsensor, 0);

	if(uart != nullptr)
	{
		
	for(i=0;i<29;i++)
	{
		rx_data12[i]=0;
		mav_data12[i]=0;
	}

	for(i = 0;i<65;i++)
	{
		rx_data6[i] = 0;
		mav_data6[i] = 0;
	}
	_initialised = true;


	}

//	printf("1.%d\n", _initialised);
}


void AP_Gassensor::SendCMD(uint8_t CMD)
{
//	uint8_t checksum_msb=0;
//	uint8_t checksum_lsb=0;
	uint8_t i;
//	uint16_t crc=0xFFFF;
//	uint8_t b;
	
	Tx_Buff[0]=0x01;
	Tx_Buff[1]=0x03;
	Tx_Buff[2]=0x03;
	if (CMD==0X00)
	{
	Tx_Buff[3]=0x80;
	}
	else 
	{
	Tx_Buff[3]=0x00;
	}
	Tx_Buff[4]=0x00;
	if (CMD==0X00)
	{
	Tx_Buff[5]=0x0C;
	Tx_Buff[6]=0x44;
	Tx_Buff[7]=0x63;
	}
	else 
	{
	Tx_Buff[5]=0x1E;
	Tx_Buff[6]=0xC5;
	Tx_Buff[7]=0x86;
	}
	Tx_Buff[8]=0x0d;
	Tx_Buff[9]=0x0a;

	for(i=0;i<10;i++)
	{
		uart->write(Tx_Buff[i]);
	}
}

void AP_Gassensor::update(const AP_SerialManager& serial_manager)
{
	if(!_initialised)
	{
		init(serial_manager);//serial init 
		
		//get_sensor6();//rx-03 00
	}

	if(!_initialised)
	{
		printf("gas is error\n");
		return;
	}
	/*
	SendCMD(fixed);//tx-03 80
	get_sensor12();//rx-03 80

	SendCMD(onboard);//tx-03 80
	get_sensor6();//rx-03 80
*/

	
	if(turn_read == 1)
	{
		SendCMD(onboard);//tx-03 80
	}
	if(turn_read == 2)
	{
		get_sensor6();//rx-03 80
	}
	if(turn_read == 3)
		SendCMD(fixed);//tx-03 80
	if(turn_read == 4)
	{
		get_sensor12();//rx-03 80
		turn_read = 0;
	}
	turn_read++;
				
}
void AP_Gassensor::get_sensor6()
{
	rxdata6_len=uart->available();
	rx_len_flag=0;
	while(1)
	{
		readdata = uart->read();
		if(readdata == -1)
			break;
		if(rx_len_flag < 65)
			rx_data6[rx_len_flag]=readdata;
		//uart->write(rx_data12[rx_len_flag]);
		//printf("6  %d is %x\n",rx_len_flag,rx_data6[rx_len_flag]);
		if(rx_len_flag == 256)
			break;
		rx_len_flag++;
	}

	/*
	while(rx_len_flag<rxdata6_len)
	{
		rx_data6[rx_len_flag]=uart->read();
		//uart->write(rx_data6[rx_len_flag]);
		printf("6  %d is %x CRC is %d\n",rx_len_flag,rx_data6[rx_len_flag],merge(rx_data6[64],rx_data6[63]));
		rx_len_flag++;
	}*/
	
//	printf("CRC is %d\n",merge(rx_data6[64],rx_data6[63]));
	if(CRC16(rx_data6,63)==merge(rx_data6[64],rx_data6[63]))
	{
		//printf("get 6 is ture CRC is %d",merge(rx_data6[64],rx_data6[63]));
		int i;
		for(i = 0;i < 65; i++)
		{
			mav_data6[i] = rx_data6[i];
		}
	}
	
	
}
	

void AP_Gassensor::get_sensor12()
{
	rxdata12_len=uart->available();
	rx_len_flag=0;
	//uart->printf("\n %d \n",rxdata12_len);
	while(1)
	{
		readdata = uart->read();
		if(readdata == -1)
			break;
		if(rx_len_flag<29)
			rx_data12[rx_len_flag]=readdata;
		//uart->write(rx_data12[rx_len_flag]);
		//printf("12	%d is %x\n",rx_len_flag,rx_data12[rx_len_flag]);		
		if(rx_len_flag == 256)
			break;
		rx_len_flag++;
	}


	/*while(rx_len_flag<rxdata12_len)
	{
		rx_data12[rx_len_flag]=uart->read();
		//uart->write(rx_data12[rx_len_flag]);
		printf("12	%d is %x	CRC is %d\n",rx_len_flag,rx_data12[rx_len_flag],merge(rx_data12[28],rx_data12[27]));		
		rx_len_flag++;
	}*/

//	printf("CRC is %d\n",merge(rx_data12[28],rx_data12[27]));
	if(CRC16(rx_data12,27)==merge(rx_data12[28],rx_data12[27]))
	{
		
		//printf("get 12 is ture  CRC is %d",merge(rx_data12[28],rx_data12[27]));
	
		int i;
		for(i = 0;i < 29; i++)
		{
			mav_data12[i] = rx_data12[i];
		}
		Sensor12_data[0]=merge(rx_data12[3],rx_data12[4]);
		Sensor12_data[1]=merge(rx_data12[5],rx_data12[6]);
		Sensor12_data[2]=merge(rx_data12[7],rx_data12[8]);
		Sensor12_data[3]=merge(rx_data12[9],rx_data12[10]);
		Sensor12_data[4]=merge(rx_data12[11],rx_data12[12]);
		Sensor12_data[5]=merge(rx_data12[13],rx_data12[14]);
		Sensor12_data[6]=merge(rx_data12[15],rx_data12[16]);
		Sensor12_data[7]=merge(rx_data12[17],rx_data12[18]);
		Sensor12_data[8]=merge(rx_data12[19],rx_data12[20]);
		Sensor12_data[9]=merge(rx_data12[21],rx_data12[22]);
		Sensor12_data[10]=merge(rx_data12[23],rx_data12[24]);
		Sensor12_data[11]=merge(rx_data12[25],rx_data12[26]);
		Sensor12_data[12]=merge(rx_data12[28],rx_data12[27]);
		if(rx_data12[23]==0x80)
			Sensor12_data[10]=Sensor12_data[10]*(-1);	
	}
	
}

//char Conversion to uint32_t
uint32_t AP_Gassensor::merge(unsigned char high,unsigned char low)
{
	uint32_t val;
	val=(uint32_t)(high)*256+(uint32_t)low;
	return val;
}
//crc
unsigned short AP_Gassensor::CRC16(unsigned char *puchMsg, unsigned int usDataLen)  
{  
 	unsigned short wCRCin = 0xFFFF;  
    unsigned short wCPoly = 0x8005;  
    unsigned char wChar = 0;  
    
    while (usDataLen--)     
    {  
        wChar = *(puchMsg++);
        InvertUint8(&wChar,&wChar);
        wCRCin ^= (wChar << 8); 
        
        for(int i = 0; i < 8; i++)  
        {  
            if(wCRCin & 0x8000) 
			{
                 wCRCin = (wCRCin << 1) ^ wCPoly;  
             }
            else  
            {
                wCRCin = wCRCin << 1; 
            }            
        }  
     }  
     InvertUint16(&wCRCin, &wCRCin);  
	 //uart->printf("\n				%d\n",wCRCin);

     return (wCRCin) ;  

}  

void AP_Gassensor::InvertUint8(unsigned char *DesBuf, unsigned char *SrcBuf)
{
    int i;
    unsigned char temp = 0;
    
    for(i = 0; i < 8; i++)
    {
        if(SrcBuf[0] & (1 << i))
        {
            temp |= 1<<(7-i);
        }
    }
    DesBuf[0] = temp;
}
void AP_Gassensor::InvertUint16(unsigned short *DesBuf, unsigned short *SrcBuf)	
{	
	 int i;  
	 unsigned short temp = 0;	 
	 
	 for(i = 0; i < 16; i++)  
	 {	
		 if(SrcBuf[0] & (1 << i))
		 {			
			 temp |= 1<<(15 - i);  
		 }
	 }	
	 DesBuf[0] = temp;	
}

float AP_Gassensor::square(float number,uint16_t power)
{
	float num=number;
	if(power==0)
		number=1;
	else
	{
		for(;power>1;power--)
		{
			number=number*num;
		}
	}
	return number;
}
