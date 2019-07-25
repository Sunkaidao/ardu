#include "Copter.h"

// ---------------------------------------------
void Copter::set_auto_armed(bool b)
{
    // if no change, exit immediately
    if( ap.auto_armed == b )
        return;

    ap.auto_armed = b;
    if(b){
        Log_Write_Event(DATA_AUTO_ARMED);
    }
}

// ---------------------------------------------
/**
 * Set Simple mode
 *
 * @param [in] b 0:false or disabled, 1:true or SIMPLE, 2:SUPERSIMPLE
 */
void Copter::set_simple_mode(uint8_t b)
{
    if (ap.simple_mode != b) {
        switch (b) {
            case 0:
                Log_Write_Event(DATA_SET_SIMPLE_OFF);
                gcs().send_text(MAV_SEVERITY_INFO, "SIMPLE mode off");
                break;
            case 1:
                Log_Write_Event(DATA_SET_SIMPLE_ON);
                gcs().send_text(MAV_SEVERITY_INFO, "SIMPLE mode on");
                break;
            case 2:
            default:
                // initialise super simple heading
                update_super_simple_bearing(true);
                Log_Write_Event(DATA_SET_SUPERSIMPLE_ON);
                gcs().send_text(MAV_SEVERITY_INFO, "SUPERSIMPLE mode on");
                break;
        }
        ap.simple_mode = b;
    }
}

// ---------------------------------------------
void Copter::set_failsafe_radio(bool b)
{
    // only act on changes
    // -------------------
    if(failsafe.radio != b) {

        // store the value so we don't trip the gate twice
        // -----------------------------------------------
        failsafe.radio = b;

        if (failsafe.radio == false) {
            // We've regained radio contact
            // ----------------------------
            failsafe_radio_off_event();
        }else{
            // We've lost radio contact
            // ------------------------
            failsafe_radio_on_event();
        }

        // update AP_Notify
        AP_Notify::flags.failsafe_radio = b;
    }
}

// ---------------------------------------------
void Copter::set_failsafe_gps_yaw(bool b)
{
    //failsafe.gps_head = b;
    //AP_Notify::flags.failsafe_battery = b;

    // only act on changes
    // -------------------
    if(failsafe.gps_yaw != b) {

        // store the value so we don't trip the gate twice
        // -----------------------------------------------
        failsafe.gps_yaw = b;

        if (failsafe.gps_yaw == false) {
            // We've regained gps head data
            // ----------------------------
            failsafe_gps_yaw_off_event();
        }else{
            // We've lost gps head data
            // ------------------------
            failsafe_gps_yaw_on_event();
        }

        // update AP_Notify
        AP_Notify::flags.failsafe_radio = b;
    }
}

void Copter::check_failsafe_gps_yaw()
{
    static uint8_t step = 0;

    switch (step){
	    case 0:
			if (EKF2.getIsGpsYawFusion()){
                step ++;
			}
            break;
		case 1:
			if (!EKF2.getIsGpsYawFusion()){
                set_failsafe_gps_yaw(true);
			}else{
                set_failsafe_gps_yaw(false);
			}
            break;
        default:
			step = 0;
			break;
	}
}

// ---------------------------------------------
void Copter::set_failsafe_gcs(bool b)
{
    failsafe.gcs = b;
}

// ---------------------------------------------

void Copter::update_using_interlock()
{
#if FRAME_CONFIG == HELI_FRAME
    // helicopters are always using motor interlock
    ap.using_interlock = true;
#else
    // check if we are using motor interlock control on an aux switch or are in throw mode
    // which uses the interlock to stop motors while the copter is being thrown
    ap.using_interlock = rc().find_channel_for_option(RC_Channel::AUX_FUNC::MOTOR_INTERLOCK) != nullptr;
#endif
}
