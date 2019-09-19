#include "Copter.h"
//#include <stdio.h> 
//#include <cstdio>


#ifdef USERHOOK_INIT
void Copter::userhook_init()
{
    // put your initialisation code here
    // this will be called once at start-up
#if FXTX_AUTH == 1
	char serial_id[AUTH_ID_LEN];
	char lcl_char_l;
	char lcl_char_h;
	uint8_t lcl_cnt;
	uint8_t lcl_out_cnt;

    memset(auth_msg, 0, 50);
    memset(serial_id, 0, AUTH_ID_LEN);
	memset(auth_id, 0, AUTH_ID_LEN);

    memset(&curr_gps_week_ms, 0, sizeof(struct current_gps_week_ms));
    

    (void)hal.util->get_system_id(serial_id);

//	gcs().send_text(MAV_SEVERITY_INFO, serial_id);


	for(lcl_out_cnt = 0; lcl_out_cnt < 3; lcl_out_cnt ++)
	{
		for(lcl_cnt = 0; lcl_cnt < 4; lcl_cnt++)
		{
			lcl_char_h = serial_id[6 + lcl_cnt *2 + lcl_out_cnt * 9];

//				printf("%d\n", 6 + lcl_cnt *2 + lcl_out_cnt * 9);


			if(lcl_char_h < 0x40)
				lcl_char_h -= 0x30;
			else
				lcl_char_h -= 0x37;

			lcl_char_h = lcl_char_h << 4;


			lcl_char_l = serial_id[6 + lcl_cnt *2 + 1 + lcl_out_cnt * 9];

//				printf("%d\n", 6 + lcl_cnt *2 + 1 + lcl_out_cnt * 9);


			if(lcl_char_l < 0x40)
				lcl_char_l -= 0x30;
			else
				lcl_char_l -= 0x37;

			lcl_char_h |= lcl_char_l;

			auth_id[lcl_cnt + lcl_out_cnt * 4] = lcl_char_h;
//			gcs().send_text(MAV_SEVERITY_INFO, "%02x", auth_id[lcl_cnt + lcl_out_cnt * 4]);
			
		}
	}

/*	gcs().send_text(MAV_SEVERITY_INFO, "3. %c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c", \
					(unsigned)serial_id[6],	 (unsigned)serial_id[7],	 (unsigned)serial_id[8],	 (unsigned)serial_id[9],	 (unsigned)serial_id[10], (unsigned)serial_id[11],	 (unsigned)serial_id[12], (unsigned)serial_id[13], \
			  		(unsigned)serial_id[15],	 (unsigned)serial_id[16],	 (unsigned)serial_id[17],	 (unsigned)serial_id[18],	 (unsigned)serial_id[19], (unsigned)serial_id[20],	 (unsigned)serial_id[21], (unsigned)serial_id[22], \
			  		(unsigned)serial_id[24],	 (unsigned)serial_id[25],	 (unsigned)serial_id[26],	 (unsigned)serial_id[27],	 (unsigned)serial_id[28], (unsigned)serial_id[29],	 (unsigned)serial_id[30], (unsigned)serial_id[31]);
*/

//	gcs().send_text(MAV_SEVERITY_INFO, "4. %s", auth_id);

//	for();

	//lcl_char_l

/*	gcs().send_text(MAV_SEVERITY_INFO, "3. %c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c", \
					(unsigned)auth_id[6],	 (unsigned)serial_id[7],	 (unsigned)serial_id[8],	 (unsigned)serial_id[9],	 (unsigned)serial_id[10], (unsigned)serial_id[11],	 (unsigned)serial_id[12], (unsigned)serial_id[13], \
			  		(unsigned)auth_id[15],	 (unsigned)serial_id[16],	 (unsigned)serial_id[17],	 (unsigned)serial_id[18],	 (unsigned)serial_id[19], (unsigned)serial_id[20],	 (unsigned)serial_id[21], (unsigned)serial_id[22], \
			  		(unsigned)auth_id[24],	 (unsigned)serial_id[25],	 (unsigned)serial_id[26],	 (unsigned)serial_id[27],	 (unsigned)serial_id[28], (unsigned)serial_id[29],	 (unsigned)serial_id[30], (unsigned)serial_id[31]);
*/

	

	snprintf(auth_msg, sizeof(auth_msg), "0123456789%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%cX",	\
    	     (unsigned)serial_id[6], 	(unsigned)serial_id[7], 	(unsigned)serial_id[8], 	(unsigned)serial_id[9], 	(unsigned)serial_id[10], (unsigned)serial_id[11],	(unsigned)serial_id[12], (unsigned)serial_id[13], \
    	     (unsigned)serial_id[15], 	(unsigned)serial_id[16], 	(unsigned)serial_id[17], 	(unsigned)serial_id[18], 	(unsigned)serial_id[19], (unsigned)serial_id[20], 	(unsigned)serial_id[21], (unsigned)serial_id[22], \
    	     (unsigned)serial_id[24], 	(unsigned)serial_id[25], 	(unsigned)serial_id[26], 	(unsigned)serial_id[27], 	(unsigned)serial_id[28], (unsigned)serial_id[29], 	(unsigned)serial_id[30], (unsigned)serial_id[31]);

	gcs().send_text(MAV_SEVERITY_INFO, &auth_msg[0]);

		//	20180418		 
/*	printf("0123456789%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c\n",	 \
			  (unsigned)serial_id[6],	 (unsigned)serial_id[7],	 (unsigned)serial_id[8],	 (unsigned)serial_id[9],	 (unsigned)serial_id[10], (unsigned)serial_id[11],	 (unsigned)serial_id[12], (unsigned)serial_id[13], \
			  (unsigned)serial_id[15],	 (unsigned)serial_id[16],	 (unsigned)serial_id[17],	 (unsigned)serial_id[18],	 (unsigned)serial_id[19], (unsigned)serial_id[20],	 (unsigned)serial_id[21], (unsigned)serial_id[22], \
			  (unsigned)serial_id[24],	 (unsigned)serial_id[25],	 (unsigned)serial_id[26],	 (unsigned)serial_id[27],	 (unsigned)serial_id[28], (unsigned)serial_id[29],	 (unsigned)serial_id[30], (unsigned)serial_id[31]);
		
*/
#endif		//	FXTX_AUTH == 1    
#if MODE_ZIGZAG_AB_ENABLED == ENABLED
	mode_zigzag_ab.mission.init();
    mode_zigzag_ab.mission.set_turning_type_parameter(g2.wp_turn_type);
#endif

}
#endif

#ifdef USERHOOK_FASTLOOP
void Copter::userhook_FastLoop()
{
    // put your 100Hz code here
}
#endif

#ifdef USERHOOK_50HZLOOP
void Copter::userhook_50Hz()
{
    // put your 50Hz code here
}
#endif

#ifdef USERHOOK_MEDIUMLOOP
void Copter::userhook_MediumLoop()
{
    // put your 10Hz code here
}
#endif

#ifdef USERHOOK_SLOWLOOP
void Copter::userhook_SlowLoop()
{
    // put your 3.3Hz code here

#if GPS_YAW_EKF_ENABLED == ENABLED
    check_failsafe_gps_yaw();
#endif

#if MODE_ZIGZAG_AB_ENABLED == ENABLED
    mode_zigzag_ab.mission.set_turning_type_parameter(g2.wp_turn_type);
    mode_zigzag_ab.mission.check_break_mode();
#endif

}
#endif

#ifdef USERHOOK_SUPERSLOWLOOP
void Copter::userhook_SuperSlowLoop()
{
    // put your 1Hz code here
    if (!motors->armed())
	{	
		//printf("reason err: %d\n",mission.regenerate_airline());
		mode_auto.mission.regenerate_airline();
	}
}
#endif

#ifdef USERHOOK_AUXSWITCH
void Copter::userhook_auxSwitch1(uint8_t ch_flag)
{
    // put your aux switch #1 handler here (CHx_OPT = 47)
}

void Copter::userhook_auxSwitch2(uint8_t ch_flag)
{
    // put your aux switch #2 handler here (CHx_OPT = 48)
}

void Copter::userhook_auxSwitch3(uint8_t ch_flag)
{
    // put your aux switch #3 handler here (CHx_OPT = 49)
}
#endif
