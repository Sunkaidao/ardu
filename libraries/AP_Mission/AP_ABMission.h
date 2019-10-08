#pragma once

#include <stdio.h>
#include <math.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AC_WPNav/AC_WPNav.h>
#include <AP_Logger/AP_Logger.h>
#include "AP_Mission/AP_Mission.h"
#include <AP_InertialNav/AP_InertialNav.h>
#include <AP_SerialManager/AP_SerialManager.h>


#define AB_MISSION_YAW          0
#define AB_MISSION_POSITION     1
#define AB_MISSION_DRAWING_CIRCLE     2


#define AB_MISSION_ISRIGHT 1
#define AB_MISSION_ISLIFT  0

#define AB_MISSION_BUZZER 0
#define AB_MISSION_RGB    1
#define AB_MISSION_BUZZER_AND_RGB 2


class AP_ABMission
{
public:
    struct AB_Mode_Info
    {
        bool is_record_a;
        bool is_record_b;
        bool is_calc_wp;
        Location a_loc{};
        Location b_loc{};
        Vector2f a_pos;
        Vector2f b_pos;
        bool is_start;
        bool is_first_start;
        bool direction;		// 1:right		0:left
        float yaw;
    };

    struct Break_Point
    {
        AP_Int32 lat_break;
        AP_Int32 lng_break;
        AP_Int32 alt_break;
        AP_Int32 lat_bp1;
        AP_Int32 lng_bp1;
        AP_Int32 lat_bp2;
        AP_Int32 lng_bp2;
        AP_Int32 index;
        AP_Int8  order;
        AP_Float yaw;
        AP_Int8 direction;
    };

    struct Mavlink_Info
    {
        Location wp{};
        int32_t wp_mavlink_index;
    };

    enum Switch_Type
    {
        SEMIAUTO = 0,
        MANUAL = 1,
    };

    // main program function pointers
    FUNCTOR_TYPEDEF(mission_cmd_fn_t, bool, const AP_Mission::Mission_Command&);
    FUNCTOR_TYPEDEF(mission_complete_fn_t, void);
	
	AP_ABMission(mission_cmd_fn_t cmd_start_fn, mission_cmd_fn_t cmd_verify_fn, mission_complete_fn_t mission_complete_fn);
	~AP_ABMission();

	// get singleton instance
    static AP_ABMission *get_singleton() {
        return _singleton;
    }

    /* Do not allow copies */
    AP_ABMission(const AP_ABMission &other) = delete;
    AP_ABMission &operator=(const AP_ABMission&) = delete;    
	
	bool init();
	void update();
	void run();

	bool calc_two_wp( Vector2f &p1 , Vector2f &p2 , const float &d , const bool inside);
	Vector2f calc_center();

	Vector2f calThreePointCoord(const Vector2f p1 , const Vector2f p2 , const float d , const bool inside);
	Vector2f calc_three_wp(const Vector2f &p1 , const Vector2f &p2);
	Vector3d calc_three_wp_test();
	
	bool abmode_set_pos_a(void);
	bool abmode_set_pos_b(void);

	bool abmode_set_pos_a_sitl(void);
	bool abmode_set_pos_b_sitl(void);
	
	bool start(void);
	bool abmode_reset(void);

	void set_p1_p2_NED(const Location &home);

	void set_target_wp();
	void set_wp_cmd(uint8_t type,const Location &target, AP_Mission::Mission_Command &cmd,float yaw_degree = 0);
	void set_wp_alt_and_type(Location &target_cmd);

	void record_break_point(void);
	void clear_break_point(void);

	void restore_spray(const Location& home);

	void adjust_yaw();
	void adjust_yaw_test();

	//Change the direction of AB
	void invert_direction(Switch_Type        type = MANUAL,int8_t direction = AB_MISSION_ISRIGHT);

	void set_direction_from_rc_roll();
	bool get_direction(){return ab_mode.direction;}
	void direction_debug();

	void update_index();
	void update_order();
	
	//It is used to differentiate the waypoints from the aircraft 
	void mark_wp_mavlink_index(int32_t _order) {ab_wp.wp_mavlink_index = _order;}
	int32_t get_wp_order(){return ab_wp.wp_mavlink_index;}

	void mark_wp_loc(const Location wp){ab_wp.wp = wp;}
	const struct Location& get_wp_loc(){return ab_wp.wp;}
	
	int8_t get_relay_spray_mode(){return relay_spray_mode;}
	AP_Mission::Mission_Command get_target_cmd(){return target_cmd;}
	bool get_abmode_direction(){return ab_mode.direction;}
	int32_t get_wp_mavlink_index(){return ab_wp.wp_mavlink_index;}
	int8_t read_aux_switch_ab();

	void set_break_mode(int8_t _mode){break_mode = _mode;}
	void clear_break_mode(){break_mode = 0;}
	void check_break_mode();

	void set_relay_spray(){relay_spray.set_and_save_ifchanged(1);}
	void clear_relay_spray(){relay_spray.set_and_save_ifchanged(0);}

	void send_message();
	void handle_message();

	void update_rgb();
	void trigger_buzzer_and_rgb(int8_t type);

	void update_spray_dist();

	char get_next_wp();
	char get_previous_wp();

	void change_ab_wp(const Vector3f target_pos_cm);

	Vector2f get_a_pos() { return p_1; }
	Vector2f get_b_pos() { return p_2; }

	bool get_overrange();

	void set_change_route(int8_t _change_route) {change_route = _change_route;}
	void clear_change_route() {change_route = 0;}
	
	int8_t get_change_route() const {return change_route;}
	int8_t get_change_route_last() const {return change_route_last;}

	void set_is_fast_waypoint(bool _is_fast_waypoint) {is_fast_waypoint = _is_fast_waypoint;}
	void clear_is_fast_waypoint() {is_fast_waypoint = 0;}
	bool get_is_fast_waypoint() {return is_fast_waypoint;}

	bool reached_circle_angale();

	void set_turning_type_parameter(int8_t _turning_type_parameter) {turning_type_parameter = _turning_type_parameter;}

	float get_radius() { return width/2;}

	float get_curr_spd_pos_angle() {return curr_spd_pos_angle;}

	int8_t get_move_mode() const {return move_mode;}

	bool get_circle_ccw() const {return circle_ccw;}

	void start_loiter_to_alt();
	void stop_flight_forward();

	int32_t get_target_alt() {
		//int32_t temp_alt = MAX(ab_mode.a_loc.alt,ab_mode.b_loc.alt);
        flight_alt = MAX(alt_break,ab_mode.b_loc.alt);

		return flight_alt;
	}

	int16_t get_turn_angle() {return turn_angle.get();}
	float get_speed_angle() {return speed_angle.get();}
	
	static const struct AP_Param::GroupInfo     var_info[];
protected:
		
	bool _initialised;

private:

    static AP_ABMission *_singleton;
	
	struct AP_Mission::Mission_Command target_cmd = {};

	// internal variables
    struct AP_Mission::Mission_Command  _nav_cmd;   // current "navigation" command.  It's position in the command list is held in _nav_cmd.index
    struct AP_Mission::Mission_Command  _do_cmd;    // current "do" command.  It's position in the command list is held in _do_cmd.index

    struct Ab_Mission_Flags {
        //mission_state state;
        uint8_t nav_cmd_loaded  : 1; // true if a "navigation" command has been loaded into _nav_cmd
        uint8_t do_cmd_loaded   : 1; // true if a "do"/"conditional" command has been loaded into _do_cmd
        uint8_t do_cmd_all_done : 1; // true if all "do"/"conditional" commands have been completed (stops unnecessary searching through eeprom for do commands)
        uint8_t do_cmd_change_airline  : 1; // true if it need to use breakpoints to regenerate airline
        uint8_t nav_cmd_breakpoint     : 1; // true if it need continues to work after flying to the breakpoint
        uint8_t nav_cmd_manual_obstacle_avoidance     : 1; // true if it is in manual obstacle avoidance mode
        uint8_t breakpoint_valid       : 1; // true if the breakpoint is available
        uint8_t send_breakpoint       : 1; // true if the breakpoint should be sended 
    } _flags;
	
    // pointer to main program functions
    mission_cmd_fn_t        _cmd_start_fn;  // pointer to function which will be called when a new command is started
    mission_cmd_fn_t        _cmd_verify_fn; // pointer to function which will be called repeatedly to ensure a command is progressing
    mission_complete_fn_t   _mission_complete_fn;   // pointer to function which will be called when mission completes
    
	//AP_Float width;
	AP_Int8  relay_spray;
	AP_Float stop_time;
	AP_Int8 relay_spray_mode;  //1:Ground station command  -1: remote controler command
	AP_Float rgb_time;
	AP_Int16 turn_angle;
	AP_Float speed_angle;
	AP_Float width;

	bool overrange;
	AB_Mode_Info ab_mode;
	Break_Point relay;
	Mavlink_Info ab_wp;

	int8_t order;
	int8_t break_mode;  // 0:AB flight   1: break point, 2: calculate point
	float_t rgb_timer;
	int8_t rgb_flag;
    //float width;
	
	int32_t alt_break;
	int32_t index;
	uint64_t timer;
	uint8_t step;
	uint8_t step_first;
	Vector2f p_1;  //unit: cm
	Vector2f p_2;  //unit: cm
	
	Location target_wp;
	Location home_loc;

	int8_t turning_type;
	
	Vector2f circle_center;
	bool circle_ccw; //0 if clockwise, 1 if counter clockwise
	float rotation_angle;

	bool is_fast_waypoint;
	int8_t turning_type_parameter;
	bool is_route_short;
	int8_t change_route;
	int8_t change_route_last;

	Vector3f zero;
	Vector3f origin_ab;

	float curr_spd_pos_angle;

	int8_t move_mode; // 0:straight line, 1:Arc

	int32_t flight_alt;
	//	added by zhangyong for
};

namespace AP {
    AP_ABMission *ab_mission();
};

