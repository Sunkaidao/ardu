#include "Copter.h"

#if MODE_ZIGZAG_AB_ENABLED == ENABLED

/*
 * Init and run calls for auto flight mode
 *
 * This file contains the implementation for Land, Waypoint navigation and Takeoff from Auto mode
 * Command execution code (i.e. command_logic.pde) should:
 *      a) switch to Auto flight mode with set_mode() function.  This will cause auto_init to be called
 *      b) call one of the three auto initialisation functions: auto_wp_start(), auto_takeoff_start(), auto_land_start()
 *      c) call one of the verify functions auto_wp_verify(), auto_takeoff_verify, auto_land_verify repeated to check if the command has completed
 * The main loop (i.e. fast loop) will call update_flight_modes() which will in turn call auto_run() which, based upon the auto_mode variable will call
 *      correct auto_wp_run, auto_takeoff_run or auto_land_run to actually implement the feature
 */

/*
 *  While in the auto flight mode, navigation or do/now commands can be run.
 *  Code in this file implements the navigation commands
 */

// auto_init - initialise auto controller
bool ModeZigZagAB::init(bool ignore_checks)
{
    if (copter.position_ok() || ignore_checks) {
        _mode = Auto_Loiter;

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        if (copter.g2.ab_sitl == 1) {
            mission.abmode_set_pos_a_sitl();
            mission.abmode_set_pos_b_sitl();
        }
#endif
        // stop ROI from carrying over from previous runs of the mission
        // To-Do: reset the yaw as part of auto_wp_start when the previous command was not a wp command to remove the need for this special ROI check
        if (auto_yaw.mode() == AUTO_YAW_ROI) {
            auto_yaw.set_mode(AUTO_YAW_HOLD);
        }

        // initialise waypoint and spline controller
        wp_nav->wp_and_spline_init();
		
        // initialise wpnav to stopping point
        Vector3f stopping_point;
        wp_nav->get_wp_stopping_point(stopping_point);

        stopping_point.z = mission.get_target_alt();
                       
        // no need to check return status because terrain data is not used
        wp_nav->set_wp_destination(stopping_point, false);

        // initialise yaw
        auto_yaw.set_mode_to_default(false);

        // clear guided limits
        copter.mode_guided.limit_clear();

        // start the ab mission
        if (!mission.start()) {
            return false;
        }
		
        return true;
    } else {
        return false;
    }
}

// auto_run - runs the auto controller
//      should be called at 100hz or more
//      relies on run_autopilot being called at 10hz which handles decision making and non-navigation related commands
void ModeZigZagAB::run()
{
    // call the correct auto controller
    switch (_mode) {

    case Auto_WP:
	case Auto_Spline:
    case Auto_CircleMoveToEdge:
        wp_run();
        break;

    case Auto_Circle:
        circle_run();
        break;

    case Auto_Loiter:
        loiter_run();
        break;

    case Auto_LoiterToAlt:
        loiter_to_alt_run();
		mission.change_ab_wp(pos_control->get_pos_target());
        break;
    default:
        break;
    }

	if (_mode == Auto_WP || \
		_mode == Auto_Spline || \
		_mode == Auto_LoiterToAlt )
    {
        abmode_switch_nav_mode();
    }
}

// auto_loiter_start - initialises loitering in auto mode
//  returns success/failure because this can be called by exit_mission
bool ModeZigZagAB::loiter_start()
{
    // return failure if GPS is bad
    if (!copter.position_ok()) {
        return false;
    }
    _mode = Auto_Loiter;

    // calculate stopping point
    Vector3f stopping_point;
    wp_nav->get_wp_stopping_point(stopping_point);

    // initialise waypoint controller target to stopping point
    wp_nav->set_wp_destination(stopping_point);

    // hold yaw at current heading
    auto_yaw.set_mode(AUTO_YAW_HOLD);

    return true;
}

// auto_wp_start - initialises waypoint controller to implement flying to a particular destination
void ModeZigZagAB::wp_start(const Vector3f& destination, bool terrain_alt)
{
    _mode = Auto_WP;

    // initialise wpnav (no need to check return status because terrain data is not used)
    wp_nav->set_wp_destination(destination, terrain_alt);

    // initialise yaw
    // To-Do: reset the yaw only when the previous navigation command is not a WP.  this would allow removing the special check for ROI
    if (auto_yaw.mode() != AUTO_YAW_ROI) {
        auto_yaw.set_mode_to_default(false);
    }
}

// auto_wp_start - initialises waypoint controller to implement flying to a particular destination
void ModeZigZagAB::wp_start(const Location& dest_loc)
{
    _mode = Auto_WP;

    // send target to waypoint controller
    if (!wp_nav->set_wp_destination(dest_loc)) {
        // failure to set destination can only be because of missing terrain data
        copter.failsafe_terrain_on_event();
        return;
    }

    // initialise yaw
    // To-Do: reset the yaw only when the previous navigation command is not a WP.  this would allow removing the special check for ROI
    if (auto_yaw.mode() != AUTO_YAW_ROI) {
        auto_yaw.set_mode_to_default(false);
    }
}

// auto_wp_start - initialises waypoint controller to implement flying to a particular destination
void ModeZigZagAB::spline_start_smooth(const Location& dest_loc,float width)
{
    _mode = Auto_Spline;

    // send target to waypoint controller
    if (!wp_nav->set_wp_destination_smooth(dest_loc,width)) {
        // failure to set destination can only be because of missing terrain data
        copter.failsafe_terrain_on_event();
        return;
    }

    // initialise yaw
    // To-Do: reset the yaw only when the previous navigation command is not a WP.  this would allow removing the special check for ROI
    if (auto_yaw.mode() != AUTO_YAW_ROI) {
        auto_yaw.set_mode_to_default(false);
    }
}

// auto_circle_movetoedge_start - initialise waypoint controller to move to edge of a circle with it's center at the specified location
//  we assume the caller has performed all required GPS_ok checks
void ModeZigZagAB::circle_movetoedge_start(const Location &circle_center, float radius_m)
{
    // convert location to vector from ekf origin
    Vector3f circle_center_neu;
    if (!circle_center.get_vector_from_origin_NEU(circle_center_neu)) {
        // default to current position and log error
        circle_center_neu = inertial_nav.get_position();
        AP::logger().Write_Error(LogErrorSubsystem::NAVIGATION, LogErrorCode::FAILED_CIRCLE_INIT);
    }
    copter.circle_nav->set_center(circle_center_neu);

    // set circle radius
    if (!is_zero(radius_m)) {
        copter.circle_nav->set_radius(radius_m * 100.0f);
    }

    circle_start();
}

// auto_circle_start - initialises controller to fly a circle in AUTO flight mode
//   assumes that circle_nav object has already been initialised with circle center and radius
void ModeZigZagAB::circle_start()
{
    _mode = Auto_Circle;

    // initialise circle controller
    copter.circle_nav->init_smooth(copter.circle_nav->get_center());

	// initialise yaw
    // To-Do: reset the yaw only when the previous navigation command is not a WP.  this would allow removing the special check for ROI
    if (auto_yaw.mode() != AUTO_YAW_ROI) {
        auto_yaw.set_mode_to_default(false);
    }
}

// start_command - this function will be called when the ap_mission lib wishes to start a new command
bool ModeZigZagAB::start_command(const AP_Mission::Mission_Command& cmd)
{
    // To-Do: logging when new commands start/end
    if (copter.should_log(MASK_LOG_CMD)) {
        copter.logger.Write_Mission_Cmd2(cmd,pos_control->get_desired_velocity().z);
    }

    switch(cmd.id) {

    ///
    /// navigation commands
    ///

    case MAV_CMD_NAV_WAYPOINT:                  // 16  Navigate to Waypoint
        do_nav_wp(cmd);
        break;

    case MAV_CMD_NAV_LOITER_TURNS:              //18 Loiter N Times
        do_circle(cmd);
        break;

    case MAV_CMD_NAV_LOITER_TO_ALT:
        do_loiter_to_alt(cmd);
        break;

    case MAV_CMD_NAV_SPLINE_WAYPOINT:           // 82  Navigate to Waypoint using spline
        do_nav_wp_smooth(cmd);
        break;

    case MAV_CMD_CONDITION_YAW:             // 115
        do_yaw(cmd);
        break;

    default:
        // unable to use the command, allow the vehicle to try the next command
        return false;
    }

    // always return success
    return true;
}

// exit_mission - function that is called once the mission completes
void ModeZigZagAB::exit_mission()
{
    // play a tone
    AP_Notify::events.mission_complete = 1;
    // if we are not on the ground switch to loiter or land
    if (!copter.ap.land_complete) {
        // try to enter loiter but if that fails land
        if (!loiter_start()) {
            set_mode(LAND, MODE_REASON_MISSION_END);
        }
    } else {
        // if we've landed it's safe to disarm
        copter.arming.disarm();
    }
}

uint32_t ModeZigZagAB::wp_distance() const
{
    switch (_mode) {
    case Auto_Circle:
        return copter.circle_nav->get_distance_to_target();
    case Auto_WP:
    case Auto_CircleMoveToEdge:
    default:
        return wp_nav->get_wp_distance_to_destination();
    }
}

int32_t ModeZigZagAB::wp_bearing() const
{
    switch (_mode) {
    case Auto_Circle:
        return copter.circle_nav->get_bearing_to_target();
    case Auto_WP:
    case Auto_CircleMoveToEdge:
    default:
        return wp_nav->get_wp_bearing_to_destination();
    }
}

bool ModeZigZagAB::get_wp(Location& destination)
{
    switch (_mode) {
    case Auto_NavGuided:
        return copter.mode_guided.get_wp(destination);
    case Auto_WP:
        return wp_nav->get_wp_destination(destination);
    default:
        return false;
    }
}

// update mission
void ModeZigZagAB::run_autopilot()
{
    mission.update();
}

/*******************************************************************************
Verify command Handlers

Each type of mission element has a "verify" operation. The verify
operation returns true when the mission element has completed and we
should move onto the next mission element.
Return true if we do not recognize the command so that we move on to the next command
*******************************************************************************/

// verify_command - callback function called from ap-mission at 10hz or higher when a command is being run
//      we double check that the flight mode is AUTO to avoid the possibility of ap-mission triggering actions while we're not in AUTO mode
bool ModeZigZagAB::verify_command(const AP_Mission::Mission_Command& cmd)
{
    if (copter.flightmode != &copter.mode_zigzag_ab && \
		copter.flightmode != &copter.mode_auto) {
        return false;
    }

    bool cmd_complete = false;

    switch (cmd.id) {
    //
    // navigation commands
    //

    case MAV_CMD_NAV_WAYPOINT:
        cmd_complete = verify_nav_wp(cmd);
        break;

    case MAV_CMD_NAV_LOITER_TURNS:
        cmd_complete = verify_circle(cmd);
        break;

    case MAV_CMD_NAV_LOITER_TO_ALT:
        return verify_loiter_to_alt();

    case MAV_CMD_NAV_SPLINE_WAYPOINT:
        cmd_complete = verify_spline_wp(cmd);
        break;


    case MAV_CMD_CONDITION_YAW:
        cmd_complete = verify_yaw();
        break;

    default:
        // error message
        gcs().send_text(MAV_SEVERITY_WARNING,"Skipping invalid cmd #%i",cmd.id);
        // return true if we do not recognize the command so that we move on to the next command
        cmd_complete = true;
        break;
    }


    // send message to GCS
    if (cmd_complete) {
        //gcs().send_mission_item_reached_message(cmd.index);
    }

    return cmd_complete;
}

// auto_wp_run - runs the auto waypoint controller
//      called by auto_run at 100hz or more
void ModeZigZagAB::wp_run()
{
    // process pilot's yaw input
    float target_yaw_rate = 0;
	float target_climb_rate = 0.0f;
    if (!copter.failsafe.radio) {
        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());
        if (!is_zero(target_yaw_rate)) {
            auto_yaw.set_mode(AUTO_YAW_HOLD);
        }
		
		// get pilot desired climb rate
        target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());
        target_climb_rate = constrain_float(target_climb_rate, -get_pilot_speed_dn(), g.pilot_speed_up);
    }

    // if not armed set throttle to zero and exit immediately
    if (is_disarmed_or_landed()) {
        make_safe_spool_down();
        wp_nav->wp_and_spline_init();
        return;
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

	// adjust climb rate using rangefinder
    target_climb_rate = copter.surface_tracking.adjust_climb_rate(target_climb_rate);
	
    // get avoidance adjusted climb rate
    target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);
	
    pos_control->set_alt_target_from_climb_rf_rate_ff(target_climb_rate, G_Dt, false);

    // run waypoint controller
    copter.failsafe_terrain_set_status(wp_nav->update_wpnav_rf());

    // call z-axis position controller (wpnav should have already updated it's alt target)
    pos_control->update_z_controller();

    // call attitude controller
    if (auto_yaw.mode() == AUTO_YAW_HOLD) {
        // roll & pitch from waypoint controller, yaw rate from pilot
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), target_yaw_rate);
    } else {
        // roll, pitch from waypoint controller, yaw heading from auto_heading()
        attitude_control->input_euler_angle_roll_pitch_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), auto_yaw.yaw(), true);
    }
}

// auto_circle_run - circle in AUTO flight mode
//      called by auto_run at 100hz or more
void ModeZigZagAB::circle_run()
{
    // process pilot's yaw input
    float target_yaw_rate = 0;
	float target_climb_rate = 0.0f;
	
    if (!copter.failsafe.radio) {
        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());
        if (!is_zero(target_yaw_rate)) {
            auto_yaw.set_mode(AUTO_YAW_HOLD);
        }
		
		// get pilot desired climb rate
        target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());
        target_climb_rate = constrain_float(target_climb_rate, -get_pilot_speed_dn(), g.pilot_speed_up);
    }
	
    // call circle controller
    copter.circle_nav->update_smooth();

    // adjust climb rate using rangefinder
    target_climb_rate = copter.surface_tracking.adjust_climb_rate(target_climb_rate);
	
    // get avoidance adjusted climb rate
    target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);
	
    pos_control->set_alt_target_from_climb_rf_rate_ff(target_climb_rate, G_Dt, false);
	
    // call z-axis position controller
    pos_control->update_z_controller();


    if (auto_yaw.mode() == AUTO_YAW_HOLD) {
        // roll, pitch from waypoint controller, yaw heading from auto_heading()
        //attitude_control->input_euler_angle_roll_pitch_yaw(copter.circle_nav->get_roll(), copter.circle_nav->get_pitch(), auto_yaw.yaw(), true);
		attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(copter.circle_nav->get_roll(), copter.circle_nav->get_pitch(), 0);
    } else {
		// roll & pitch from waypoint controller, yaw rate from pilot
        attitude_control->input_euler_angle_roll_pitch_yaw(copter.circle_nav->get_roll(), copter.circle_nav->get_pitch(), copter.circle_nav->get_yaw(), true);
    }

    //attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(copter.circle_nav->get_roll(), copter.circle_nav->get_pitch(), 0);
}

// auto_loiter_run - loiter in AUTO flight mode
//      called by auto_run at 100hz or more
void ModeZigZagAB::loiter_run()
{
    // if not armed set throttle to zero and exit immediately
    if (is_disarmed_or_landed()) {
        make_safe_spool_down();
        wp_nav->wp_and_spline_init();
        return;
    }

    // accept pilot input of yaw
    float target_yaw_rate = 0;
    if (!copter.failsafe.radio) {
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // run waypoint and z-axis position controller
    copter.failsafe_terrain_set_status(wp_nav->update_wpnav());

    pos_control->update_z_controller();
	
    // call attitude controller
    if (auto_yaw.mode() == AUTO_YAW_HOLD) {
        // roll & pitch from waypoint controller, yaw rate from pilot
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), target_yaw_rate);
    } else {
        // roll, pitch from waypoint controller, yaw heading from auto_heading()
        attitude_control->input_euler_angle_roll_pitch_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), auto_yaw.yaw(), true);
    }
}

// auto_loiter_run - loiter to altitude in AUTO flight mode
//      called by auto_run at 100hz or more
void ModeZigZagAB::loiter_to_alt_run()
{
	// call regular rtl flight mode run function
    copter.mode_loiter.run_without_roll();
}

/********************************************************************************/
//	Nav (Must) commands
/********************************************************************************/

Location ModeZigZagAB::loc_from_cmd(const AP_Mission::Mission_Command& cmd) const
{
    Location ret(cmd.content.location);

    // use current lat, lon if zero
    if (ret.lat == 0 && ret.lng == 0) {
        ret.lat = copter.current_loc.lat;
        ret.lng = copter.current_loc.lng;
    }
    // use current altitude if not provided
    if (ret.alt == 0) {
        // set to current altitude but in command's alt frame
        int32_t curr_alt;
        if (copter.current_loc.get_alt_cm(ret.get_alt_frame(),curr_alt)) {
            ret.set_alt_cm(curr_alt, ret.get_alt_frame());
        } else {
            // default to current altitude as alt-above-home
            ret.set_alt_cm(copter.current_loc.alt,
                           copter.current_loc.get_alt_frame());
        }
    }
    return ret;
}

// do_nav_wp - initiate move to next waypoint
void ModeZigZagAB::do_nav_wp(const AP_Mission::Mission_Command& cmd)
{
	// convert back to location
    Location target_loc(cmd.content.location);

    // this will be used to remember the time in millis after we reach or pass the WP.
    loiter_time = 0;
    // this is the delay, stored in seconds
    loiter_time_max = cmd.p1;
	
    // use current location if not provided
    if (target_loc.lat == 0 && target_loc.lng == 0) {
		// initialise waypoint and spline controller
        wp_nav->wp_and_spline_init();
		
        // To-Do: make this simpler
        Vector3f temp_pos;
        copter.wp_nav->get_wp_stopping_point_xy(temp_pos);
        const Location temp_loc(temp_pos);
        target_loc.lat = temp_loc.lat;
        target_loc.lng = temp_loc.lng;
    }

    // use current altitude if not provided
    // To-Do: use z-axis stopping point instead of current alt
    if (target_loc.alt == 0) {
        // set to current altitude but in command's alt frame
        int32_t curr_alt;
        if (copter.current_loc.get_alt_cm(target_loc.get_alt_frame(),curr_alt)) {
            target_loc.set_alt_cm(curr_alt, target_loc.get_alt_frame());
        } else {
            // default to current altitude as alt-above-home
            target_loc.set_alt_cm(copter.current_loc.alt,
                                  copter.current_loc.get_alt_frame());
        }
    }
	
    // Set wp navigation target
    wp_start(target_loc);

    // if no delay as well as not final waypoint set the waypoint as "fast"
    bool fast_waypoint = false;
    if (loiter_time_max == 0) {
		fast_waypoint = true;
        copter.wp_nav->set_fast_waypoint(fast_waypoint);
    }
}

// do_nav_wp - initiate move to next waypoint
void ModeZigZagAB::do_nav_wp_smooth(const AP_Mission::Mission_Command& cmd)
{
    Location target_loc = loc_from_cmd(cmd);

    // this will be used to remember the time in millis after we reach or pass the WP.
    loiter_time = 0;
    // this is the delay, stored in seconds
    loiter_time_max = 0;

    // initialise position controller (sets target roll angle, pitch angle and I terms based on vehicle current lean angles)
    pos_control->set_desired_accel_xy(0.0f,0.0f);
    pos_control->set_desired_velocity_xy(0,0);
    pos_control->init_xy_controller_smooth();

    if (_mode != Auto_WP && _mode != Auto_Spline) {
		pos_control->set_desired_velocity_z(inertial_nav.get_velocity_z());
        pos_control->set_target_to_stopping_point_z();
	}
	
    // Set wp navigation target
    spline_start_smooth(target_loc,cmd.p1);
}


// do_loiter_unlimited - start loitering with no end conditions
// note: caller should set yaw_mode
void ModeZigZagAB::do_loiter_unlimited(const AP_Mission::Mission_Command& cmd)
{
    // convert back to location
    Location target_loc(cmd.content.location);

    // use current location if not provided
    if (target_loc.lat == 0 && target_loc.lng == 0) {
        // To-Do: make this simpler
        Vector3f temp_pos;
        copter.wp_nav->get_wp_stopping_point_xy(temp_pos);
        const Location temp_loc(temp_pos);
        target_loc.lat = temp_loc.lat;
        target_loc.lng = temp_loc.lng;
    }

    // use current altitude if not provided
    // To-Do: use z-axis stopping point instead of current alt
    if (target_loc.alt == 0) {
        // set to current altitude but in command's alt frame
        int32_t curr_alt;
        if (copter.current_loc.get_alt_cm(target_loc.get_alt_frame(),curr_alt)) {
            target_loc.set_alt_cm(curr_alt, target_loc.get_alt_frame());
        } else {
            // default to current altitude as alt-above-home
            target_loc.set_alt_cm(copter.current_loc.alt,
                                  copter.current_loc.get_alt_frame());
        }
    }

    // start way point navigator and provide it the desired location
    wp_start(target_loc);
}

// do_circle - initiate moving in a circle
void ModeZigZagAB::do_circle(const AP_Mission::Mission_Command& cmd)
{
    const Location circle_center = loc_from_cmd(cmd);

    // calculate radius
    float circle_radius_m = (float)cmd.p1/100.0f; // circle radius held in high byte of p1

	float rate = cmd.content.location.loiter_ccw ? -30 : 30;
    copter.circle_nav->set_rate(rate); 

    // move to edge of circle (verify_circle) will ensure we begin circling once we reach the edge
    circle_movetoedge_start(circle_center, circle_radius_m);
}

// do_loiter_time - initiate loitering at a point for a given time period
// note: caller should set yaw_mode
void ModeZigZagAB::do_loiter_time(const AP_Mission::Mission_Command& cmd)
{
    // re-use loiter unlimited
    do_loiter_unlimited(cmd);

    // setup loiter timer
    loiter_time     = 0;
    loiter_time_max = cmd.p1;     // units are (seconds)
}

// do_loiter_alt - initiate loitering at a point until a given altitude is reached
// note: caller should set yaw_mode
void ModeZigZagAB::do_loiter_to_alt(const AP_Mission::Mission_Command& cmd)
{
    _mode = Auto_LoiterToAlt;
	copter.mode_loiter.init_without_roll(false);
}

/********************************************************************************/
//	Condition (May) commands
/********************************************************************************/

void ModeZigZagAB::do_yaw(const AP_Mission::Mission_Command& cmd)
{
	auto_yaw.set_fixed_yaw(
		cmd.content.yaw.angle_deg,
		cmd.content.yaw.turn_rate_dps,
		cmd.content.yaw.direction,
		cmd.content.yaw.relative_angle > 0);
}

/********************************************************************************/
//	Verify Nav (Must) commands
/********************************************************************************/

bool ModeZigZagAB::verify_loiter_unlimited()
{
    return false;
}

// verify_loiter_time - check if we have loitered long enough
bool ModeZigZagAB::verify_loiter_time(const AP_Mission::Mission_Command& cmd)
{
    // return immediately if we haven't reached our destination
    if (!copter.wp_nav->reached_wp_destination()) {
        return false;
    }

    // start our loiter timer
    if ( loiter_time == 0 ) {
        loiter_time = millis();
    }

    // check if loiter timer has run out
    if (((millis() - loiter_time) / 1000) >= loiter_time_max) {
        gcs().send_text(MAV_SEVERITY_INFO, "Reached command #%i",cmd.index);
        return true;
    }

    return false;
}

// verify_loiter_to_alt - check if we have reached both destination
// (roughly) and altitude (precisely)
bool ModeZigZagAB::verify_loiter_to_alt()
{
/*
    if (loiter_to_alt.reached_destination_xy &&
        loiter_to_alt.reached_alt) {
        return true;
    }
*/
    return false;
}

// verify_yaw - return true if we have reached the desired heading
bool ModeZigZagAB::verify_yaw()
{
    // set yaw mode if it has been changed (the waypoint controller often retakes control of yaw as it executes a new waypoint command)
    if (auto_yaw.mode() != AUTO_YAW_FIXED) {
        auto_yaw.set_mode(AUTO_YAW_FIXED);
    }

    // check if we are within 2 degrees of the target heading
    return (fabsf(wrap_180_cd(ahrs.yaw_sensor-auto_yaw.yaw())) <= 200);
}

// verify_nav_wp - check if we have reached the next way point
bool ModeZigZagAB::verify_nav_wp(const AP_Mission::Mission_Command& cmd)
{
    // check if we have reached the waypoint
    if ( !copter.wp_nav->reached_wp_destination() ) {
        return false;
    }

    // start timer if necessary
    if (loiter_time == 0) {
        loiter_time = millis();
		if (loiter_time_max > 0) {
			// play a tone
			AP_Notify::events.waypoint_complete = 1;
			}
    }

    // check if timer has run out
    if ((millis() - loiter_time) >= loiter_time_max) {
		if (loiter_time_max == 0) {
			// play a tone
			AP_Notify::events.waypoint_complete = 1;
			}
        //gcs().send_text(MAV_SEVERITY_INFO, "Reached command #%i",cmd.index);
        return true;
    }
    return false;
}

// verify_circle - check if we have circled the point enough
bool ModeZigZagAB::verify_circle(const AP_Mission::Mission_Command& cmd)
{
    // check if we've reached the edge
    if (mode() == Auto_CircleMoveToEdge) {
        if (copter.wp_nav->reached_wp_destination()) {
            Vector3f circle_center;
            if (!cmd.content.location.get_vector_from_origin_NEU(circle_center)) {
                // should never happen
                return true;
            }
            const Vector3f curr_pos = copter.inertial_nav.get_position();
            // set target altitude if not provided
            if (is_zero(circle_center.z)) {
                circle_center.z = curr_pos.z;
            }

            // set lat/lon position if not provided
            if (cmd.content.location.lat == 0 && cmd.content.location.lng == 0) {
                circle_center.x = curr_pos.x;
                circle_center.y = curr_pos.y;
            }

            // start circling
            circle_start();
        }
        return false;
    }

	bool res = mission.reached_circle_angale();
    if (res) {
        pos_control->set_desired_velocity_xy(0, 0);
	    pos_control->set_desired_accel_xy(0, 0);
	}
		
    // check if we have completed circling
    return res;
}

// verify_spline_wp - check if we have reached the next way point using spline
bool ModeZigZagAB::verify_spline_wp(const AP_Mission::Mission_Command& cmd)
{
    // check if we have reached the waypoint
    if ( !copter.wp_nav->reached_wp_destination() ) {
        return false;
    }
/*
    // check if timer has run out
    if (copter.wp_nav->get_wp_distance_to_destination() < 50) {
		AP_Notify::events.waypoint_complete = 1;
        pos_control->set_desired_velocity_xy(0, 0);
	    pos_control->set_desired_accel_xy(0, 0);
        return true;
    }
*/
	Vector3f pos_delta_unit = copter.wp_nav->get_pos_delta_unit();
	Vector3f des2curr =  inertial_nav.get_position() - copter.wp_nav->get_wp_destination();

    float len = des2curr*pos_delta_unit;
	return len > 0 ? true : false;
}

void ModeZigZagAB::abmode_switch_nav_mode()
{
	static uint8_t step_pitch_a = 0;
	static uint8_t step_pitch_b = 0;
	static uint8_t state = 0;
	bool should_jump = false;

	//Check if radio control intervention is required
	switch(state)
	{
		case 0:
			if (mission.get_previous_wp() == mission.get_next_wp())
	       {
               state = 1;
	       }
			break;
		case 1:
			if (wp_nav->get_wp_distance_to_destination() < 300)
			{
			    should_jump = true;
			    state = 2;
			}
			break;
		case 2:
			if (mission.get_previous_wp() == mission.get_next_wp())
	       {
				if (wp_nav->get_wp_distance_to_destination() > 300)
				{
				    should_jump = false;
				    state = 0;
				}
				else
				{
				    should_jump = true;
				}
	       }
			else
			{
			    Vector2f current_cm;
			    Vector2f ab_pos_cm = mission.get_previous_wp() == 'A' ? mission.get_a_pos() : mission.get_b_pos();
                current_cm.x = inertial_nav.get_position().x;
                current_cm.y = inertial_nav.get_position().y;

				if ((current_cm - ab_pos_cm).length() > 300)
				{
				    should_jump = false;
				    state = 0;
				}
				else
				{
				    should_jump = true;
				}
			}
			break;
	}			

	if (mission.get_previous_wp() == mission.get_next_wp() || should_jump)
	{
		return;
	}


	if (mission.get_next_wp() == 'A')
    {
		switch(step_pitch_a)
		{
			case 0:
					//The pitch channel is pushed forward to a negative value and pushed back to a positive value.
					if (copter.channel_pitch->get_control_in() > ROLL_PITCH_YAW_INPUT_MAX/20 && \
						!wp_nav->reached_wp_destination())
					{
						step_pitch_a = 1;
					}
					else if (copter.channel_pitch->get_control_in() < -ROLL_PITCH_YAW_INPUT_MAX/20 && \
						!wp_nav->reached_wp_destination())
					{
						step_pitch_a = 3;
					}
					else
					{
						step_pitch_a = 0;
					}
			       break;
			case 1: 
					if (channel_pitch->get_control_in() > ROLL_PITCH_YAW_INPUT_MAX/20 && \
					 	  wp_nav->get_wp_distance_to_destination() < 300 )
					{
						 mission.start_loiter_to_alt();
					     step_pitch_a = 2;
						 
						 mission.set_change_route(1);
						 mission.set_is_fast_waypoint(false);
					}
			       break;
			case 2:
					if (channel_pitch->get_control_in() < ROLL_PITCH_YAW_INPUT_MAX/20)
					{
						mission.stop_flight_forward();
						mission.change_ab_wp(pos_control->get_pos_target());
				        step_pitch_a = 0;

						mission.set_change_route(1);
						mission.set_is_fast_waypoint(false);
					}
					break;
			case 3:
					if (channel_pitch->get_control_in() < -ROLL_PITCH_YAW_INPUT_MAX/20)
					{
                        mission.stop_flight_forward();
						mission.change_ab_wp(pos_control->get_pos_target());
				        step_pitch_a = 4;

						mission.set_change_route(1);
						mission.set_is_fast_waypoint(false);
					}
					break;
			case 4:
					if (channel_pitch->get_control_in() > -ROLL_PITCH_YAW_INPUT_MAX/20)
					{
				        step_pitch_a = 0;
					}
					break;
			default:
					step_pitch_a = 0;
					break;
		}
    }
    else if (mission.get_next_wp() == 'B')
    {
       switch(step_pitch_b)
		{
			case 0:
				//The pitch channel is pushed forward to a negative value and pushed back to a positive value.
				if (copter.channel_pitch->get_control_in() < -ROLL_PITCH_YAW_INPUT_MAX/20 && \
						!wp_nav->reached_wp_destination())
				{
					step_pitch_b = 1;
				}
				else if (copter.channel_pitch->get_control_in() > ROLL_PITCH_YAW_INPUT_MAX/20 && \
						!wp_nav->reached_wp_destination())
				{
					step_pitch_b = 3;
				}
				else
				{

					step_pitch_b = 0;
				}
			    break;
			case 1: 
				if (channel_pitch->get_control_in() < -ROLL_PITCH_YAW_INPUT_MAX/20 && \
					 wp_nav->get_wp_distance_to_destination() < 300 )
				{
					mission.start_loiter_to_alt();
					step_pitch_b = 2;

					mission.set_change_route(1);
					mission.set_is_fast_waypoint(false);
				}
			    break;
			case 2:
				if (channel_pitch->get_control_in() > -ROLL_PITCH_YAW_INPUT_MAX/20)
				{
                    mission.stop_flight_forward();
                    mission.change_ab_wp(pos_control->get_pos_target());
					step_pitch_b = 0;

					mission.set_change_route(1);
					mission.set_is_fast_waypoint(false);
				}
				break;
			case 3:
				if (channel_pitch->get_control_in() > -ROLL_PITCH_YAW_INPUT_MAX/20)
				{
					mission.stop_flight_forward();
                    mission.change_ab_wp(pos_control->get_pos_target());
				    step_pitch_b = 4;

                    mission.set_change_route(1);
                    mission.set_is_fast_waypoint(false);
                }
                break;
			case 4:
                if (channel_pitch->get_control_in() < ROLL_PITCH_YAW_INPUT_MAX/20)
                {
                    step_pitch_b = 0;
				}
                break;
			default:
				step_pitch_b = 0;
				break;
			}
    }
}

#endif
