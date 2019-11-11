#include "./../ArduCopter/Copter.h"


#define NO  0
#define YES 1

#define INVALID -3

#define A_TO_B      0
#define B_TO_A      1

//0: A to B,1: B to A
#define AB_SEQUENCE A_TO_B

#define ENABLE_DEFAULT 1
#define WIDTH_DEFAULT 3
#define RELAY_DEFAULT 0
#define ST_TIME_DEFAULT 1
#define RELAY_MD_DEFAULT 1
#define RGB_TIME_DEFAULT 1.5f
#define EXT_SP_DEFAULT 5.0f
#define T_ANG_DEFAULT 175
#define SPD_ANG_DEFAULT 5


const AP_Param::GroupInfo AP_ABMission::var_info[] = {
	
	// @Param: ST_TIME
	// @DisplayName: After reaching the target point, the time to pause
	// @Parameter Type: float
	// @unit: second
	AP_GROUPINFO("ST_TIME", 0, AP_ABMission, stop_time, ST_TIME_DEFAULT),

	AP_GROUPINFO("LAT_BP1", 1, AP_ABMission, relay.lat_bp1, 0),
	AP_GROUPINFO("LNG_BP1", 2, AP_ABMission, relay.lng_bp1, 0),
	AP_GROUPINFO("LAT_BP2", 3, AP_ABMission, relay.lat_bp2, 0),
	AP_GROUPINFO("LNG_BP2", 4, AP_ABMission, relay.lng_bp2, 0),
	AP_GROUPINFO("LAT_BK", 5, AP_ABMission, relay.lat_break, 0),
	AP_GROUPINFO("LNG_BK", 6, AP_ABMission, relay.lng_break, 0),
	AP_GROUPINFO("ORDER", 7, AP_ABMission, relay.order, 0),
	AP_GROUPINFO("YAW", 8, AP_ABMission, relay.yaw, 0),
	AP_GROUPINFO("INDEX", 9, AP_ABMission, relay.index, 0),
	AP_GROUPINFO("ALT_BK", 10, AP_ABMission, relay.alt_break, 0),
	AP_GROUPINFO("DIR", 11, AP_ABMission, relay.direction, 0),
	
	// @Param: ST_TIME
	// @DisplayName: After reaching the target point, the time to pause
	// @Parameter Type: float
	// @unit: second
	AP_GROUPINFO("RGB_TIME", 12, AP_ABMission, rgb_time, RGB_TIME_DEFAULT),

	// @Param: EXT_SP
	// @DisplayName: Maximum speed when extending the route
	// @Parameter Type: float
	// @unit: meter
	AP_GROUPINFO("T_ANG", 13, AP_ABMission, turn_angle, T_ANG_DEFAULT),

	// @Param: SPD_ANG
	// @DisplayName: Maximum speed when extending the route
	// @Parameter Type: float
	// @unit: meter
	AP_GROUPINFO("SPD_ANG", 14, AP_ABMission, speed_angle, SPD_ANG_DEFAULT),

	// @Param: SPD_ANG
	// @DisplayName: Maximum speed when extending the route
	// @Parameter Type: float
	// @unit: meter
	AP_GROUPINFO("WIDTH", 15, AP_ABMission, width, 500),
	
    AP_GROUPEND
};




AP_ABMission::AP_ABMission(mission_cmd_fn_t cmd_start_fn, mission_cmd_fn_t cmd_verify_fn, mission_complete_fn_t mission_complete_fn) :
    _cmd_start_fn(cmd_start_fn),
    _cmd_verify_fn(cmd_verify_fn),
    _mission_complete_fn(mission_complete_fn)
{
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (_singleton != nullptr) {
        AP_HAL::panic("Mission must be singleton");
    }
#endif
    _singleton = this;

	AP_Param::setup_object_defaults(this, var_info);	
  	_initialised = false;
	ab_mode.direction = AB_MISSION_ISRIGHT; 		// the right is default

	zero.zero();
}

AP_ABMission::~AP_ABMission()
{}

bool AP_ABMission::init()
{
	index = 0;
	step = 0;
	order = 0;
	step_first = 0;
	ab_mode.is_record_a = NO;
	ab_mode.is_record_b = NO;
	ab_mode.is_start = NO;
	ab_mode.is_calc_wp = NO;
	ab_mode.is_first_start = YES;
	alt_break = 0;
	rgb_timer = 0;
	rgb_flag = 0;
	ab_mode.yaw = 0;
	overrange = false;
	turning_type = 0;
	circle_ccw = false;
	is_route_short = false;
	change_route = 0;
	change_route_last = 0;
	move_mode = 0;
	
	memset( & ab_mode.a_loc, 0, sizeof(ab_mode.a_loc));
	memset( & ab_mode.b_loc, 0, sizeof(ab_mode.b_loc));
	memset( & ab_mode.a_pos, 0, sizeof(ab_mode.a_pos));
	memset( & ab_mode.b_pos, 0, sizeof(ab_mode.b_pos));
	memset( & target_cmd, 0, sizeof(target_cmd));
	memset( & p_1, 0, sizeof(p_1));
	memset( & p_2, 0, sizeof(p_2));
	mark_wp_mavlink_index(INVALID);
	set_break_mode(1);
	
	_initialised = true; 


	return true;
}

bool AP_ABMission::abmode_set_pos_a(void)
{
	if(ab_mode.is_start == YES)
	{
		gcs().send_text(MAV_SEVERITY_CRITICAL,"ABMODE: flight started,set failure");
		return false;
	}
	
	if(LOITER != copter.control_mode)
	{
		gcs().send_text(MAV_SEVERITY_CRITICAL,"ABMODE: Need Loiter");
		return false;
	}

	if(!copter.motors->armed())
	{
		gcs().send_text(MAV_SEVERITY_CRITICAL,"ABMODE: Need Armed");
		return false;
	}

	if(!copter.position_ok())
	{
		gcs().send_text(MAV_SEVERITY_CRITICAL,"ABMODE: Need 3D Fix");
		return false;
	}	
	
	if (LOITER == copter.control_mode \
		&& copter.motors->armed()\
		&& copter.position_ok() \
		&& ab_mode.is_start == NO)
	{   
		AP::ahrs_navekf().get_location(ab_mode.a_loc);
		ab_mode.a_loc.set_alt_cm(copter.pos_control->get_alt_target(),Location::AltFrame::ABOVE_ORIGIN);
		
		if (!ab_mode.a_loc.get_vector_xy_from_origin_NE(ab_mode.a_pos)) {
            return false;
        }

		ab_mode.is_record_a = YES;
        alt_break = 0;

		//A point -1
		//copter.DataFlash.Log_Write_Target_WP(ab_mode.a_loc,-1,ab_mode.direction,ab_mode.yaw,home);
		mark_wp_mavlink_index(-1);
		mark_wp_loc(ab_mode.a_loc);
		
		gcs().send_text(MAV_SEVERITY_CRITICAL,"ABMODE: Set A point success");

		trigger_buzzer_and_rgb(AB_MISSION_BUZZER_AND_RGB);

		return true;
	}

	return false;
}

bool AP_ABMission::abmode_set_pos_b(void)
{
	if(ab_mode.is_start == YES)
	{
		gcs().send_text(MAV_SEVERITY_CRITICAL,"ABMODE: flight started,set failure");
		return false;
	}
	
	if(LOITER != copter.control_mode)
	{
		gcs().send_text(MAV_SEVERITY_CRITICAL,"ABMODE: Need Loiter");
		return false;
	}

	if(!copter.motors->armed())
	{
		gcs().send_text(MAV_SEVERITY_CRITICAL,"ABMODE: Need Armed");
		return false;
	}

	if(!copter.position_ok())
	{
		gcs().send_text(MAV_SEVERITY_CRITICAL,"ABMODE: Need 3D Fix");
		return false;
	}	

	if(ab_mode.is_record_a == NO)
	{
		gcs().send_text(MAV_SEVERITY_CRITICAL,"ABMODE: Not record A");
		return false;
	}
	
	if (LOITER == copter.control_mode \
		&& copter.motors->armed() \
		&& copter.position_ok() \
		&& ab_mode.is_start == NO)
	{
		AP::ahrs_navekf().get_location(ab_mode.b_loc);            
        ab_mode.b_loc.set_alt_cm(copter.pos_control->get_alt_target(),Location::AltFrame::ABOVE_ORIGIN);
		
        if (!ab_mode.b_loc.get_vector_xy_from_origin_NE(ab_mode.b_pos)) {
            return false;
        }
		
		ab_mode.is_record_b = YES;
		alt_break = 0;

		//B point -2
		//copter.DataFlash.Log_Write_Target_WP(ab_mode.b_loc,-2,ab_mode.direction,ab_mode.yaw,home);
		mark_wp_mavlink_index(-2);
		mark_wp_loc(ab_mode.b_loc);
		
		gcs().send_text(MAV_SEVERITY_CRITICAL,"ABMODE: Set B point success");

		trigger_buzzer_and_rgb(AB_MISSION_BUZZER_AND_RGB);
		
		return true;
	}

	return false;
}

bool AP_ABMission::abmode_set_pos_a_sitl(void)
{
	if(ab_mode.is_start == YES)
	{
		gcs().send_text(MAV_SEVERITY_CRITICAL,"ABMODE: flight started,set failure");
		return false;
	}
	
	if(LOITER != copter.control_mode)
	{
		gcs().send_text(MAV_SEVERITY_CRITICAL,"ABMODE: Need Loiter");
		return false;
	}

	if(!copter.motors->armed())
	{
		gcs().send_text(MAV_SEVERITY_CRITICAL,"ABMODE: Need Armed");
		return false;
	}

	if(!copter.position_ok())
	{
		gcs().send_text(MAV_SEVERITY_CRITICAL,"ABMODE: Need 3D Fix");
		return false;
	}	
	
	if (LOITER == copter.control_mode \
		&& copter.motors->armed()\
		&& copter.position_ok() \
		&& ab_mode.is_start == NO)
	{   
		ab_mode.a_loc.lat = -353638000;
		ab_mode.a_loc.lng = 1491653194;
         
        ab_mode.a_loc.set_alt_cm(copter.pos_control->get_alt_target(),Location::AltFrame::ABOVE_ORIGIN);
		
		if (!ab_mode.a_loc.get_vector_xy_from_origin_NE(ab_mode.a_pos)) {
            return false;
        }

		ab_mode.is_record_a = YES;
		alt_break = 0;
		
		//A point -1
		//copter.DataFlash.Log_Write_Target_WP(ab_mode.a_loc,-1,ab_mode.direction,ab_mode.yaw,home);
		mark_wp_mavlink_index(-1);
		mark_wp_loc(ab_mode.a_loc);
		
		gcs().send_text(MAV_SEVERITY_CRITICAL,"ABMODE: Set A point success");

		trigger_buzzer_and_rgb(AB_MISSION_BUZZER_AND_RGB);

		return true;
	}

	return false;
}

bool AP_ABMission::abmode_set_pos_b_sitl(void)
{
	if(ab_mode.is_start == YES)
	{
		gcs().send_text(MAV_SEVERITY_CRITICAL,"ABMODE: flight started,set failure");
		return false;
	}
	
	if(LOITER != copter.control_mode)
	{
		gcs().send_text(MAV_SEVERITY_CRITICAL,"ABMODE: Need Loiter");
		return false;
	}

	if(!copter.motors->armed())
	{
		gcs().send_text(MAV_SEVERITY_CRITICAL,"ABMODE: Need Armed");
		return false;
	}

	if(!copter.position_ok())
	{
		gcs().send_text(MAV_SEVERITY_CRITICAL,"ABMODE: Need 3D Fix");
		return false;
	}	

	if(ab_mode.is_record_a == NO)
	{
		gcs().send_text(MAV_SEVERITY_CRITICAL,"ABMODE: Not record A");
		return false;
	}
	
	if (LOITER == copter.control_mode \
		&& copter.motors->armed() \
		&& copter.position_ok() \
		&& ab_mode.is_start == NO)
	{
		ab_mode.b_loc.lat = -353632615;
		ab_mode.b_loc.lng = 1491652288;
            
        ab_mode.b_loc.set_alt_cm(copter.pos_control->get_alt_target(),Location::AltFrame::ABOVE_ORIGIN);
		
        if (!ab_mode.b_loc.get_vector_xy_from_origin_NE(ab_mode.b_pos)) {
            return false;
        }

		
		ab_mode.is_record_b = YES;

		alt_break = 0;
		
		//B point -2
		//copter.DataFlash.Log_Write_Target_WP(ab_mode.b_loc,-2,ab_mode.direction,ab_mode.yaw,home);
		mark_wp_mavlink_index(-2);
		mark_wp_loc(ab_mode.b_loc);
		
		gcs().send_text(MAV_SEVERITY_CRITICAL,"ABMODE: Set B point success");

		trigger_buzzer_and_rgb(AB_MISSION_BUZZER_AND_RGB);
		
		return true;
	}

	return false;
}

Vector2f AP_ABMission::calc_center()
{
    Vector2f center_ned;
	
    Vector2f p1_center_cm = calThreePointCoord(p_1 , p_2 , width/2 , ab_mode.direction);
	Vector2f p2_center_cm = calThreePointCoord(p_2 , p_1 , width/2 , !ab_mode.direction);

	if (get_next_wp() == 'A')
	{
	    center_ned.x = p1_center_cm.x;
		center_ned.y = p1_center_cm.y;
	}
	else if (get_next_wp() == 'B')
	{
	    center_ned.x = p2_center_cm.x;
		center_ned.y = p2_center_cm.y;
	}

	return center_ned;
}

bool AP_ABMission::calc_two_wp(Vector2f &p1 , Vector2f &p2 , const float &d , const bool inside)
{
	update_index();
	update_order();
	
	Vector2f p1_next;
	Vector2f p2_next;

	is_fast_waypoint = false;
	turning_type = turning_type_parameter;
	
	if((index % 2) != 0)
	{
	    if (turning_type_parameter == 0)
	    {
			p1_next = calThreePointCoord(p1 , p2 , d , inside);
			p2_next = calThreePointCoord(p2 , p1 , d , !inside);

					
			p1 = p1_next;
			p2 = p2_next;
	    }
		else
		{
		    switch(index)
			{
				case 1:
					p1_next = calThreePointCoord(p1 , p2 , d , inside);
			        p2_next = calThreePointCoord(p2 , p1 , d , !inside);

					if ((p1_next - p2_next).length() > d / 2)
					{
					    p1 = p2_next + (p1_next - p2_next).normalized() * (double)((p1_next - p2_next).length() - d / 2);
			            p2 = p2_next;
					}
					else
					{
					    turning_type = 0;
						is_route_short = true;
						p1 = p1_next;
			            p2 = p2_next;
					}
			        
					break;
				case 3:
					p1_next = calThreePointCoord(p1 , p2 , d , inside);
			        p2_next = calThreePointCoord(p2 , p1 , d , !inside);

					circle_center = (p1_next + p1) / 2;
					
					if (inside)
					{
					    circle_ccw = true;
					}
					else
					{
					    circle_ccw = false;
					}

					if ((p2_next - p1_next).length() > d / 2)
			        {
			            p1 = p1_next;
			            p2 = p1_next + (p2_next - p1_next).normalized() * (double)((p2_next - p1_next).length() - d / 2);
					}
					else
					{
					    turning_type = 0;
						is_route_short = true;
						circle_center.zero();
						p1 = p1_next;
			            p2 = p2_next;
					}

					if (is_route_short || change_route)
					{
					    turning_type = 0;
						circle_center.zero();
					}
					
					break;
				default:
					p1_next = calThreePointCoord(p1 , p2 , d , inside);
			        p2_next = calThreePointCoord(p2 , p1 , d , !inside);

					circle_ccw = !circle_ccw;

                    switch(order)
					{
					    case 0:
						case 1:
						case 4:
						case 5:
							circle_center = (p2_next + p2) / 2;

						    p2 = p2_next;
							if (change_route_last)
							{
							    if ((p1_next - p2_next).length() > d / 2)
								{
								    p1 = p2_next + (p1_next - p2_next).normalized() * (double)((p1_next - p2_next).length() - d / 2);
									is_route_short = false;
								}
								else
								{
								    turning_type = 0;
									is_route_short = true;
									p1 = p1_next;
								}
							}
							else
							{
							    p1 = p1_next;
							}
							
							break;
						case 2:
						case 3:
							circle_center = (p1_next + p1) / 2;

						    p1 = p1_next;
							if (change_route_last)
							{
								if ((p2_next - p1_next).length() > d / 2)
						        {
						            p2 = p1_next + (p2_next - p1_next).normalized() * (double)((p2_next - p1_next).length() - d / 2);
									is_route_short = false;
								}
								else
								{
								    turning_type = 0;
									is_route_short = true;									
						            p2 = p2_next;
								}
							}
							else
							{
							    p2 = p2_next;
							}
							break;
						default:
							p1 = p1_next;
			                p2 = p2_next;
							break;
					}

					if (is_route_short || change_route)
					{
					    turning_type = 0;
						circle_center.zero();
					}

					break;
			}

			change_route_last = change_route;
		}
	}

	if(order == 4)
	{
		order = 0;
	}

    Vector2f uint1d = (p2 - p1).normalized();
	Vector3f uint_ab(uint1d.x,uint1d.y,0);

	float vel_xy = copter.inertial_nav.get_speed_xy();

    Location target_loc1(Vector3f(p1.x, p1.y,0));
	Location target_loc2(Vector3f(p2.x, p2.y,0));
	//GKXN
	switch(order)
	{
		case 0:
		case 1:
	    case 4:
			target_wp = target_loc2;
			
			origin_ab.x = p1.x;
		    origin_ab.y = p1.y;

			origin_ab = origin_ab + uint_ab * vel_xy;
			break;

		case 2:
		case 3:
			target_wp = target_loc1;

		    origin_ab.x = p2.x;
		    origin_ab.y = p2.y;

			origin_ab = origin_ab - uint_ab * vel_xy;
			break;
	}

    if (turning_type != 0 && index > 1 && !is_route_short && !change_route)
	{
	    is_fast_waypoint = true;
	}

	//change_route_last = change_route;
    change_route = 0;
	
    return true;
}

//Calculate on long side
Vector2f AP_ABMission::calc_three_wp(const Vector2f &p1 , const Vector2f &p2)
{
	Vector2f current_cm(copter.inertial_nav.get_position().x,copter.inertial_nav.get_position().y);

	Vector2f temp = Vector2f::closest_point(current_cm, p1, p2);
	
    return temp;
}

Vector3d AP_ABMission::calc_three_wp_test()
{
	Vector3d temp;
	Vector3d current_cm;
	Vector3d p1;
	Vector3d p2;

	p1.x = 1.0f;
	p1.y = 1.0f;
	p1.z = 0;

	p2.x = 3.0f;
	p2.y = 5.0f;
	p2.z = 0;
	
	current_cm.x = 3.0f;
	current_cm.y = 3.0f;
	current_cm.z = 0;
	
	temp.x = (current_cm.x*(p2.x-p1.x)+current_cm.y*(p2.y-p1.y)+p1.x*(sq(p2.y-p1.y))/(p2.x-p1.x)-p1.y*(p2.y-p1.y))*(p2.x-p1.x)/(sq(p2.x-p1.x)+sq(p2.y-p1.y));
	temp.y = (p2.y-p1.y)/(p2.x-p1.x)*(temp.x-p1.x)+p1.y;
	temp.z = 0;

	printf("temp x %3.4f\n",temp.x);
	printf("temp y %3.4f\n",temp.y);

	double _current_m = (double)300.0f/(double)100.0f;
	printf("_current_m %4.4f\n",_current_m);
    return temp;
}


void AP_ABMission::update_index()
{
	index++;
}

void AP_ABMission::update_order()
{
	order++;
}

bool AP_ABMission::start(void)
{
	copter.ahrs.get_origin(home_loc);

	if(ab_mode.is_start == YES)
	{
		gcs().send_text(MAV_SEVERITY_CRITICAL,"ABMODE: flight started,set failure");
		return false;
	}
	
	if(LOITER != copter.control_mode)
	{
		gcs().send_text(MAV_SEVERITY_CRITICAL,"ABMODE: Need Loiter");
		return false;
	}

	if(!copter.motors->armed())
	{
		gcs().send_text(MAV_SEVERITY_CRITICAL,"ABMODE: Need Armed");
		return false;
	}

	move_mode = 0;

	if (ab_mode.is_record_a && ab_mode.is_record_b)
	{
		index = 0;
		order = 0;
		p_1 = ab_mode.a_pos;
		p_2 = ab_mode.b_pos;

		target_wp = ab_mode.b_loc;

		clear_break_mode();
		
		ab_mode.is_first_start = YES;
		ab_mode.is_calc_wp = YES;
        is_fast_waypoint = false;

	}
	else if (!ab_mode.is_record_a && !ab_mode.is_record_b)
	{
		if(break_mode == 1)                     //Return from the current location to the breakpoint
		{			
			restore_spray(home_loc);

			if(order % 2 == 0)
			{
				target_wp.lat = relay.lat_break;
				target_wp.lng = relay.lng_break;
			}
			else
			{			
				switch(order)
				{
					case 1:
						target_wp.lat = relay.lat_bp2;
						target_wp.lng = relay.lng_bp2;
						break;
					case 3:
						target_wp.lat = relay.lat_bp1;
						target_wp.lng = relay.lng_bp1;
						break;
					default:
		        		memset( & target_wp, 0, sizeof(target_wp));
						break;
				}
			}
			
			//if (get_distance(home_loc,target_wp) > (float)copter.g2.breakpoint_dis_limit_m)
			if (home_loc.get_distance_NE(target_wp).length() > 500)
			{
			    overrange = true;
			    gcs().send_text(MAV_SEVERITY_ERROR,"ABMODE: Breakpoint overrange");
				//copter.Log_Write_Error(ERROR_SUBSYSTEM_BK_OVERRANGE, ERROR_CODE_FAILSAFE_OCCURRED);
				return false;
			}
			
			if(order%2 == 0)
			{
				index--;
				order--;
			}

			//is_fast_waypoint = false;
		}
		else if(break_mode == 2)                //Return from the current position to the flight path
		{			
			restore_spray(home_loc);
		
			if(order % 2 == 0)
			{
			    Vector2f p1_t(p_1.x,p_1.y);
				Vector2f p2_t(p_2.x,p_2.y);
				Vector2f temp = calc_three_wp(p1_t,p2_t);
				Vector3f offset_neu(temp.x,temp.y,0);
				Location target_wp_temp(offset_neu);
				target_wp = target_wp_temp;
			}
			else
			{
				switch(order)
				{
					case 1:
						target_wp.lat = relay.lat_bp2;
						target_wp.lng = relay.lng_bp2;
						break;
					case 3:
						target_wp.lat = relay.lat_bp1;
						target_wp.lng = relay.lng_bp1;
						break;
					default:
		        		memset( & target_wp, 0, sizeof(target_wp));
						break;
				}
			}
			if(order%2 == 0)
			{
				index--;
				order--;
			}

			//is_fast_waypoint = false;
		}
		else
		{
			//copter.Log_Write_Error(ERROR_SUBSYSTEM_NAVIGATION, ERROR_CODE_FAILED_TO_SET_DESTINATION);
		}

		ab_mode.is_first_start = YES;
		ab_mode.is_calc_wp = YES;
        ab_mode.is_start = YES;

		is_fast_waypoint = false;
	}
	else
	{
		gcs().send_text(MAV_SEVERITY_CRITICAL,"ABMODE: Need record B");
		return false;
	}

	return true;
}

bool AP_ABMission:: abmode_reset(void)
{	
	ab_mode.is_record_a = NO;
	ab_mode.is_record_b = NO;
	ab_mode.is_start = NO;
	ab_mode.is_calc_wp = NO;
	ab_mode.is_first_start = YES;
	alt_break = 0;  // unit:cm
	rgb_timer = 0;
	rgb_flag = 0;
	ab_mode.yaw = 0;

	memset( & p_1, 0, sizeof(p_1));
	memset( & p_2, 0, sizeof(p_2));
	memset( & target_wp, 0, sizeof(target_wp));
	memset( & target_cmd, 0, sizeof(target_cmd));

	return true;
}

void AP_ABMission:: record_break_point(void)
{	
	Vector2f break_point((double)copter.inertial_nav.get_position().x,(double)copter.inertial_nav.get_position().y);
	Vector2f p1(p_1.x,p_1.y);
	Vector2f p2(p_2.x,p_2.y);
	Vector2f break_point_online =  Vector2f::closest_point(break_point, p1, p2);
	Vector3f temp(break_point_online.x,break_point_online.y,0);
	Location break_location(temp);

	relay.lng_break.set_and_save_ifchanged(break_location.lng);
	relay.lat_break.set_and_save_ifchanged(break_location.lat);
	
	if(ab_mode.is_first_start == YES)
		relay.alt_break.set_and_save_ifchanged((int32_t)copter.inertial_nav.get_position().z);
	else
		relay.alt_break.set_and_save_ifchanged(target_cmd.content.location.alt);

	Location p1_loc(Vector3f(p_1.x,p_1.y,0));
	relay.lat_bp1.set_and_save_ifchanged(p1_loc.lat);
	relay.lng_bp1.set_and_save_ifchanged(p1_loc.lng);

    Location p2_loc(Vector3f(p_2.x,p_2.y,0));
	relay.lat_bp2.set_and_save_ifchanged(p2_loc.lat);
	relay.lng_bp2.set_and_save_ifchanged(p2_loc.lng);
	
	relay.index.set_and_save_ifchanged(index); //Records really "index", in the abmode_start() to decide whether to back off
	relay.order.set_and_save_ifchanged(order); //Records really "order", in the abmode_start() to decide whether to back off

	relay.yaw.set_and_save((float)copter.ahrs.yaw_sensor/100.0f);    //degrees

	relay.direction.set_and_save_ifchanged(ab_mode.direction);       // 1:right		0:left

}

void AP_ABMission:: restore_spray(const Location& home)
{	
    Location temp1;
	Location temp2;
	
	temp1.lat = relay.lat_bp1;
	temp1.lng = relay.lng_bp1;

	temp2.lat = relay.lat_bp2;
	temp2.lng = relay.lng_bp2;

	alt_break = relay.alt_break;
	
	index = relay.index;
	order = relay.order;
	ab_mode.direction = relay.direction;

	if (!temp1.get_vector_xy_from_origin_NE(p_1) || \
	    !temp2.get_vector_xy_from_origin_NE(p_2))
	{
	    gcs().send_text(MAV_SEVERITY_CRITICAL,"ABMODE: restore spray failure");
	    return;
	}
}

void AP_ABMission:: set_wp_cmd(uint8_t type,const Location &target, AP_Mission::Mission_Command &cmd,float yaw_degree) 
{
    memset(&cmd, 0, sizeof(cmd));
	
  	if (type == AB_MISSION_YAW) 
	{
	    cmd.id = MAV_CMD_CONDITION_YAW;
	    cmd.content.yaw.angle_deg = yaw_degree;    // target angle in degrees (0 = north, 90 = east)
	    cmd.content.yaw.turn_rate_dps = 0;         // turn rate in degrees / second (0 = use default)
	    cmd.content.yaw.relative_angle = 0;        // 0 = absolute angle, 1 = relative angle

	    if ((int32_t)(copter.ahrs.yaw_sensor - yaw_degree*100) < 18000 \
			&& (int32_t)(copter.ahrs.yaw_sensor - yaw_degree*100) > -18000)
	    {
			cmd.content.yaw.direction = 1;         // cw(Clockwise)
		}
		else
		{
			cmd.content.yaw.direction = -1; 	   // ccw(Counterclockwise)
		}
  	} 
	else if (type == AB_MISSION_POSITION) 
  	{
	    // Set the command to return to the specified height(comm_alt) above the
	    // station  Station coordinates(lat_station,lng_station)
	    cmd.id = MAV_CMD_NAV_WAYPOINT;
	    cmd.content.location = target;
		cmd.p1 = stop_time*1000;
		cmd.index = index;

		if (is_fast_waypoint)
		{
		    cmd.p1 = width/2; // Special mark of unfinished tasks, no actual physical meaning
		    cmd.id = MAV_CMD_NAV_SPLINE_WAYPOINT;
		}
		
		set_wp_alt_and_type(cmd.content.location);  
  	}
	else if (type == AB_MISSION_DRAWING_CIRCLE) 
  	{
	    // Set the command to return to the specified height(comm_alt) above the
	    // station  Station coordinates(lat_station,lng_station)
	    cmd.id = MAV_CMD_NAV_LOITER_TURNS;
	    cmd.content.location = target;		
		
		set_wp_alt_and_type(cmd.content.location);
		cmd.content.location.loiter_ccw = circle_ccw;
		cmd.p1 = width/2;
		cmd.index = index;
  	}
}

void AP_ABMission:: set_wp_alt_and_type(Location &cmd_location)
{
	int32_t temp_alt;
	
	//temp_alt = MAX(ab_mode.a_loc.alt,ab_mode.b_loc.alt);
	temp_alt = ab_mode.b_loc.alt;
    flight_alt = MAX(alt_break,temp_alt);

	cmd_location.set_alt_cm(flight_alt,Location::AltFrame::ABOVE_ORIGIN);
}

void AP_ABMission:: adjust_yaw()
{	
	if(
#if CONFIG_HAL_BOARD != HAL_BOARD_SITL
		copter.control_mode == ZIGZAG_AB && 
#endif
		break_mode !=0 )
	{
		ab_mode.yaw = relay.yaw;
		
		set_wp_cmd(AB_MISSION_YAW,target_wp, target_cmd,ab_mode.yaw);
	}
	else if(
#if CONFIG_HAL_BOARD != HAL_BOARD_SITL
		copter.control_mode == ZIGZAG_AB && 
#endif
		break_mode ==0)
	{
		ab_mode.yaw = wrap_360(degrees(atan2f((p_2.y-p_1.y),(p_2.x-p_1.x))));
		set_wp_cmd(AB_MISSION_YAW,target_wp, target_cmd,ab_mode.yaw);
	}
		
    _cmd_start_fn(target_cmd);
}

void AP_ABMission::adjust_yaw_test()
{
	Vector3d p1,p2;
	Location home;
	Location a_loc;
	Location b_loc;

	home.lat = 225622460;
	home.lng = 1134840987;
	home.alt = 0;

	a_loc.lat = 225622473;
	a_loc.lng = 1134842446;
	a_loc.alt = 0;

	b_loc.lat = 225620318;
	b_loc.lng = 1134842470;
	b_loc.alt = 0;

/*	printf("p1.x %4.4f\n",p1.x);
	printf("p1.y %4.4f\n",p1.y);
	printf("p2.x %4.4f\n",p2.x);
	printf("p2.y %4.4f\n",p2.y);
	printf("yaw    %4.4f\n",wrap_360(degrees(atan2f((p2.y-p1.y),(p2.x-p1.x)))));
*/
}

//set direction,when ab mode isn't running
void AP_ABMission:: set_direction_from_rc_roll()
{
	bool direction_rc = AB_MISSION_ISRIGHT;

	if(ab_mode.is_start)
	{
		return;
	}

#if CONFIG_HAL_BOARD != HAL_BOARD_SITL
	if(ZIGZAG_AB != copter.control_mode)
	{
		return;
	}
#endif

	if(!ab_mode.is_record_a || !ab_mode.is_record_b)
	{
		return;
	}

	if(copter.channel_roll == nullptr)
	{
		return;
	}
	
	if(copter.channel_roll->get_control_in() == 0)
	{
		return;
	}
	else if(copter.channel_roll->get_control_in() < -ROLL_PITCH_YAW_INPUT_MAX/4)
	{
		direction_rc = AB_MISSION_ISLIFT;
	}
	else if(copter.channel_roll->get_control_in() > ROLL_PITCH_YAW_INPUT_MAX/4)
	{
		direction_rc = AB_MISSION_ISRIGHT;
	}
	else
	{
		return;
	}

	if(ab_mode.direction != direction_rc)
	{
		//ab_mode.direction = direction_rc;
		invert_direction(MANUAL,direction_rc);
	}

	ab_mode.is_start = YES;
}

char AP_ABMission:: get_next_wp()
{
	int8_t order_temp = order - 1;
	char next_wp = '\0';
	
	//GKXN
	switch(order_temp)
	{
		case -1:
			next_wp = 'B';
			break;
		case 0:
			next_wp = 'B';
			break;
		case 1:
			next_wp = 'A';
			break;
		case 2:
			next_wp = 'A';
			break;
		case 3:
			next_wp = 'B';
			break;
		case 4:
			next_wp = 'B';
	}

	return next_wp;
}

char AP_ABMission:: get_previous_wp()
{
	int8_t order_temp = order - 2;
	char next_wp = '\0';

	//GKXN
	switch(order_temp)
	{
		case -2:
			next_wp = 'A';
			break;
		case -1:
			next_wp = 'B';
			break;
		case 0:
			next_wp = 'B';
			break;
		case 1:
			next_wp = 'A';
			break;
		case 2:
			next_wp = 'A';
			break;
		case 3:
			next_wp = 'B';
			break;
	}

	return next_wp;
}

void AP_ABMission:: change_ab_wp(const Vector3f target_pos_cm)
{
    Vector3f temp(target_pos_cm.x,target_pos_cm.y,target_pos_cm.z);
	Location target_loc(target_pos_cm);
	
    if(get_next_wp() == 'A')
    {
        p_1.x = temp.x;
		p_1.y = temp.y;
    }
    else if(get_next_wp() == 'B')
    {
        p_2.x = temp.x;
		p_2.y = temp.y;
    }

	mark_wp_loc(target_loc);
}

void AP_ABMission:: invert_direction(Switch_Type        type,int8_t direction)
{	
	if(ab_mode.is_start == YES)
	{
		gcs().send_text(MAV_SEVERITY_CRITICAL,"ABMODE: flight started,direction failure");
		return;
	}
	switch(type)
	{
		case SEMIAUTO:
			ab_mode.direction = !ab_mode.direction;
			if(ab_mode.direction == AB_MISSION_ISRIGHT)
				gcs().send_text(MAV_SEVERITY_CRITICAL,"ABMODE: Right");
			else
				gcs().send_text(MAV_SEVERITY_CRITICAL,"ABMODE: Lift");
			break;
		case MANUAL:
			ab_mode.direction = direction;
			if(ab_mode.direction == AB_MISSION_ISRIGHT)
				gcs().send_text(MAV_SEVERITY_CRITICAL,"ABMODE: Right");
			else
				gcs().send_text(MAV_SEVERITY_CRITICAL,"ABMODE: Lift");
			break;
	}
}

void AP_ABMission:: direction_debug()
{
	if(get_direction() == AB_MISSION_ISRIGHT)
	{
		printf("ab mode: Right\n");
	}
	else
	{
		printf("ab mode: Lift\n");
	}
}

/*
 * Param[in]:	p1 - waypoint A(Vector3d)
 * 				p2 - waypoint B(Vector3d)
 *				d - width in cm
 *				inside - direction (left and right)
 * Notes: this function for calculate two waypoint(latitude and longtitude), then return
 */
Vector2f AP_ABMission:: calThreePointCoord(const Vector2f p1 , const Vector2f p2 , const float d , const bool inside)
{
    double x = 0 , y = 0;

    if(inside)
    {
        x = p1.x - (d * (p2.y - p1.y)) / safe_sqrt((p2.x - p1.x) * (p2.x - p1.x) + (p2.y - p1.y) * (p2.y - p1.y));
        y = p1.y + (d * (p2.x - p1.x)) / safe_sqrt((p2.x - p1.x) * (p2.x - p1.x) + (p2.y - p1.y) * (p2.y - p1.y));
    }
    else
    {
        x = p1.x + (d * (p2.y - p1.y)) / safe_sqrt((p2.x - p1.x) * (p2.x - p1.x) + (p2.y - p1.y) * (p2.y - p1.y));
        y = p1.y - (d * (p2.x - p1.x)) / safe_sqrt((p2.x - p1.x) * (p2.x - p1.x) + (p2.y - p1.y) * (p2.y - p1.y));
    }

    return Vector2f(x, y);
}

void AP_ABMission:: run()
{
	if (ab_mode.is_first_start)
	{
        switch(step_first)
        {
			case 0:
                if(
#if CONFIG_HAL_BOARD != HAL_BOARD_SITL
                    copter.control_mode == ZIGZAG_AB && 
#endif
                    target_wp.lat != 0 && \
                    target_wp.lng != 0)
                {
                    adjust_yaw();
                    timer = AP_HAL::millis64();
						
                    step_first = 1;
                }
                else if(target_wp.lat == 0 || target_wp.lng == 0)
                {
                    gcs().send_text(MAV_SEVERITY_CRITICAL,"ABMODE: target wp invalid");
				}
			break;
				
		    case 1:
                if((AP_HAL::millis64()-timer) > stop_time*1500)
                {
                    if(
#if CONFIG_HAL_BOARD != HAL_BOARD_SITL
                        copter.control_mode == ZIGZAG_AB && 
#endif
                        target_wp.lat != 0 && \
                        target_wp.lng != 0)
                    {
                        set_wp_cmd(AB_MISSION_POSITION, target_wp, target_cmd);
							
                        if (!_cmd_start_fn(target_cmd))
                        {
                            gcs().send_text(MAV_SEVERITY_CRITICAL,"ABMODE: do_abmode failure");
                        }

                        //fisrt point 0
                        move_mode = 0;
                        //copter.DataFlash.Log_Write_Target_WP(target_cmd.content.location,0,ab_mode.direction,ab_mode.yaw,home_loc);
                        mark_wp_mavlink_index(0);
                        mark_wp_loc(target_cmd.content.location);
							
                        step = 0;
                        step_first = 0;
                        ab_mode.is_first_start = NO;
                    }
                    else if(target_wp.lat == 0 || target_wp.lng == 0)
                    {
                        gcs().send_text(MAV_SEVERITY_CRITICAL,"ABMODE: target wp invalid");
                    }
						
                }
            break;
        }			
	}
    else
    {		
        switch(step)
        {
            case 0:
                if (_cmd_verify_fn(target_cmd))
                {

					timer = AP_HAL::millis64();
					step = 1;
    			}
				break;
            case 1:
                if(
#if CONFIG_HAL_BOARD != HAL_BOARD_SITL
                    copter.control_mode == ZIGZAG_AB && 
#endif
                    target_wp.lat != 0 && \
                    target_wp.lng != 0)
                {		
                    calc_two_wp(p_1 , p_2 , width, ab_mode.direction);
							
                    set_wp_cmd(AB_MISSION_POSITION, target_wp, target_cmd);
                    AP_Mission::Mission_Command target_cmd_temp = target_cmd;

                    if (turning_type == 0 || (index % 2) == 0 || index == 1)
                    {
						if (!_cmd_start_fn(target_cmd))
                        {
                            gcs().send_text(MAV_SEVERITY_CRITICAL,"ABMODE: do_abmode failure");
                        }
						move_mode = 0;
					}
                    else if (turning_type != 0)
                    {    
                        if (circle_center.is_zero())
                        {
                            if (!_cmd_start_fn(target_cmd))
                            {
                                gcs().send_text(MAV_SEVERITY_CRITICAL,"ABMODE: do_abmode failure");
                            }
                            move_mode = 0;
                        }
                        else
                        {
                            set_wp_cmd(AB_MISSION_DRAWING_CIRCLE, Location(Vector3f(circle_center.x,circle_center.y,0)), target_cmd);
				            _cmd_start_fn(target_cmd);
                            move_mode = 1;
                        }							    								
                    }
							
                    //copter.DataFlash.Log_Write_Target_WP(target_cmd.content.location,index,ab_mode.direction,ab_mode.yaw,home_loc);
                    mark_wp_mavlink_index(index);
                    mark_wp_loc(target_cmd_temp.content.location);
							
                    step = 0;
                }
                else if(target_wp.lat == 0 || target_wp.lng == 0)
                {
                        gcs().send_text(MAV_SEVERITY_CRITICAL,"ABMODE: target wp invalid");
                }
					
				break;
	    }
	}
	   
}

bool AP_ABMission::reached_circle_angale()
{
    bool res = false;
	Vector2f pos_delta_unit;
	
	Vector3f curr_vel = copter.inertial_nav.get_velocity();
	curr_vel.z = 0;
	
	Vector2f target_pos;
	
	curr_vel.normalize();
	
    AP_Mission::Mission_Command cmd;
	cmd.p1 = turn_angle.get();

	if (get_next_wp() == 'A')
	{
	    pos_delta_unit = p_2 - p_1;

		target_pos.x = p_1.x;
		target_pos.y = p_1.y;
	}
	else if (get_next_wp() == 'B')
	{
	    pos_delta_unit = p_1 - p_2;

		target_pos.x = p_2.x;
		target_pos.y = p_2.y;
	}

	pos_delta_unit.normalize();

	// get bearing from circle center to vehicle in radians
    float curr_spd_rad = atan2f(curr_vel.y,curr_vel.x);
    curr_spd_rad = wrap_PI(curr_spd_rad);

	float curr_pos_rad = atan2f(pos_delta_unit.y,pos_delta_unit.x);
    curr_pos_rad = wrap_PI(curr_pos_rad);

	curr_spd_pos_angle = ToDeg(wrap_PI(curr_spd_rad - curr_pos_rad));

	bool pass_spd = circle_ccw == true ? curr_spd_pos_angle < speed_angle : curr_spd_pos_angle > -speed_angle;

	if (pass_spd && turn_angle < 270)
	{
	    res = true;
	}
	
    return (copter.circle_nav->get_angle_total()/M_2PI*360 >= (float)turn_angle.get()) && res;
}

void AP_ABMission::trigger_buzzer_and_rgb(int8_t type)
{
	switch(type)
	{
		case AB_MISSION_BUZZER:
			AP_Notify::events.user_mode_change = 1;
			break;
		case AB_MISSION_RGB:
			AP_Notify::flags.esc_calibration = 1;
			rgb_flag = 1;
			rgb_timer = 0;
			break;
		case AB_MISSION_BUZZER_AND_RGB:
			AP_Notify::events.user_mode_change = 1;
			AP_Notify::flags.esc_calibration = 1;
			rgb_flag = 1;
			rgb_timer = 0;
			break;
	}
}

void AP_ABMission::update_rgb()
{
	if(rgb_flag == 1)
	{
		if(is_equal(rgb_timer,50*rgb_time))
		{
			AP_Notify::flags.esc_calibration = 0;
			rgb_flag = 0;
			rgb_timer = 0;
		}
		
		rgb_timer++;
	}
}

void AP_ABMission::update_spray_dist()
{
	//width = 700;//(copter.sprayer.get_unspray_dist());
}

void AP_ABMission::update()
{
	if (!_initialised ) 
	{
    	return;
  	}

	update_spray_dist();
	update_rgb();
	set_direction_from_rc_roll();
	
	if (!ab_mode.is_start) 
	{
    	return;
  	}

	run();

}

void AP_ABMission::check_break_mode()
{
    if (!copter.motors->armed())
    {
        set_break_mode(1);
    }
}

bool AP_ABMission::get_overrange()
{
    bool temp = overrange;
	if (overrange)
	{
	    overrange = false;
	}

	return temp;
}


void AP_ABMission::start_loiter_to_alt()
{
    memset( & target_cmd, 0, sizeof(target_cmd));
    target_cmd.id = MAV_CMD_NAV_LOITER_TO_ALT;
	target_cmd.content.location.set_alt_cm(flight_alt,Location::AltFrame::ABOVE_ORIGIN);

	if (!_cmd_start_fn(target_cmd)) {
        gcs().send_text(MAV_SEVERITY_CRITICAL,"ABMODE: start loiter to alt failure");
    }
}

void AP_ABMission::stop_flight_forward()
{
    memset( & target_cmd, 0, sizeof(target_cmd));
    target_cmd.id = MAV_CMD_NAV_WAYPOINT;
    target_cmd.p1 = 1;
	target_cmd.content.location.set_alt_cm(flight_alt,Location::AltFrame::ABOVE_ORIGIN);
	
	if (!_cmd_start_fn(target_cmd)) {
        gcs().send_text(MAV_SEVERITY_CRITICAL,"ABMODE: stop loiter to alt failure");
    }
}


// singleton instance
AP_ABMission *AP_ABMission::_singleton;

namespace AP {

AP_ABMission *ab_mission()
{
    return AP_ABMission::get_singleton();
}

}


