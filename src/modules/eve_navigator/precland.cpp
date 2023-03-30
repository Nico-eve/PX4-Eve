/****************************************************************************
 *
 *   Copyright (c) 2017 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
/**
 * @file precland.cpp
 *
 * Helper class to do precision landing with a landing target
 *
 * @author Nicolas de Palezieux (Sunflower Labs) <ndepal@gmail.com>
 */

#include "precland.h"
#include "eve_navigator.h"

#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <fcntl.h>

#include <systemlib/err.h>
#include <systemlib/mavlink_log.h>

#include <uORB/uORB.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_command.h>
// #include <typeinfo>
// #include <iostream>
#define SEC2USEC 1000000.0f

#define STATE_TIMEOUT 10000000 // [us] Maximum time to spend in any state

static constexpr const char *LOST_TARGET_ERROR_MESSAGE = "Lost landing target while landing";
float delta_z_ref;

PrecLand::PrecLand(EveNavigator *navigator) :
	MissionBlock(navigator),
	ModuleParams(navigator)
{
	_handle_param_acceleration_hor = param_find("MPC_ACC_HOR");
	_handle_param_xy_vel_cruise = param_find("MPC_XY_CRUISE");
	safe_alt = false;
	updateParams();
}

void
PrecLand::on_activation()
{
	land_trigger_param = param_find("LNDMC_TRIG_TIME");
	_last_update = hrt_absolute_time();
	_state = PrecLandState::Start;
	_search_cnt = 0;
	_last_slewrate_time = 0;
	param_t height_param = param_find("UAVCAN_RNG_MAX");
	param_get(height_param, &max_height);
	if (max_height<10.0f){
		max_height = 10.0f;
		PX4_INFO("Using default max height value: %f",double(max_height));
	}
	else{
		max_height = 10.0f;
		PX4_INFO("Using max height value: %f",double(max_height));
	}
	last_full_beacon = hrt_absolute_time();
	// strncpy(dbg.name, "myrate", sizeof(dbg.name));
	// dbg.x = 0.0f;
	// dbg.y = 0.0f;
	// dbg.z = 0.0f;
	// pub_dbg = orb_advertise(ORB_ID(debug_vect), &dbg);
	

	vehicle_local_position_s *vehicle_local_position = _navigator->get_local_position();

	if (!_map_ref.isInitialized()) {
		_map_ref.initReference(vehicle_local_position->ref_lat, vehicle_local_position->ref_lon);
	}

	position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();
	z_init =  _navigator->get_global_position()->alt;
	pos_sp_triplet->next.valid = false;
	pos_sp_triplet->previous.valid = false;

	// Check that the current position setpoint is valid, otherwise land at current position
	if (!pos_sp_triplet->current.valid) {
		PX4_WARN("Reset");
		pos_sp_triplet->current.lat = _navigator->get_global_position()->lat;
		pos_sp_triplet->current.lon = _navigator->get_global_position()->lon;
		pos_sp_triplet->current.alt = _navigator->get_global_position()->alt;
		pos_sp_triplet->current.valid = true;
		pos_sp_triplet->current.timestamp = hrt_absolute_time();
	}

	_sp_pev = matrix::Vector2f(0, 0);
	_sp_pev_prev = matrix::Vector2f(0, 0);
	_last_slewrate_time = 0;
	K_speed = 1.0f;

	switch_to_state_start();

	_is_activated = true;
	PX4_INFO("PrecLand activated");
}

void PrecLand::reset_precland()
{
	PX4_INFO("Precision landing is reset");
	float land_trigger_val = 1.0f;
	param_set(land_trigger_param,&land_trigger_val);
	reset_param = true;
	full_beacon = false;
	nest_z_fit = false;
}

void
PrecLand::on_active()
{
	// if (_is_activated)
	// {
		// get new target measurement
		_target_pose_updated = _target_pose_sub.update(&_target_pose);

		if (_target_pose_updated) {
			_target_pose_valid = true;
		}

		if ((hrt_elapsed_time(&_target_pose.timestamp) / 1e6f) > _param_pld_btout.get()) {
			_target_pose_valid = false;
		}

		// stop if we are landed
		if (_navigator->get_land_detected()->landed) {
			switch_to_state_done();
		}

		// if (_server_mission_request_sub.updated()) {
		// 	server_mission_request_s update_mission;
		// 	if(_server_mission_request_sub.copy(&update_mission)){
		// 		global_target.lon = update_mission.lon;
		// 		global_target.lat = update_mission.lat;
		// 		global_target.alt = update_mission.alt;
		// 	}
		// }

		switch (_state) {
		case PrecLandState::Start:
			run_state_start();
			break;
		case PrecLandState::ReachAltitude:
			// run_state_descend_above_target();
			run_state_reach_altitude();
			break;
		case PrecLandState::HorizontalApproach:
			// run_state_descend_above_target();
			run_state_horizontal_approach();
			break;

		case PrecLandState::DescendAboveTarget:
			run_state_descend_above_target();
			break;

		case PrecLandState::FinalApproach:
			run_state_final_approach();
			break;

		case PrecLandState::Search:
			run_state_search();
			break;

		case PrecLandState::Fallback:
			run_state_fallback();
			break;

		case PrecLandState::Done:
			// nothing to do
			break;

		default:
			// unknown state
			break;
		}
	// }
}

void 
PrecLand::StartState(){
	_state = PrecLandState::Start;
}

void
PrecLand::on_inactivation()
{
	PX4_INFO("PrecLand deactivated");
	_is_activated = false;
}

void
PrecLand::updateParams()
{
	ModuleParams::updateParams();

	if (_handle_param_acceleration_hor != PARAM_INVALID) {
		param_get(_handle_param_acceleration_hor, &_param_acceleration_hor);
	}

	if (_handle_param_xy_vel_cruise != PARAM_INVALID) {
		param_get(_handle_param_xy_vel_cruise, &_param_xy_vel_cruise);
	}
}

// void
// PrecLand::run_go_to(){
// 	if (switch_to_state_start()){
// 		PX4_INFO("GPS target reached, looking for beacon");
// 		return;
// 	}
// 	vehicle_global_position_s* global_pos = _navigator->get_global_position();
// 	position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();
// 	if (!safe_alt){
// 		if (global_pos->alt < global_target.alt-min_thresh){
// 			pos_sp_triplet->current.lon = global_pos->lon;
// 			pos_sp_triplet->current.lat = global_pos->lat;
// 			pos_sp_triplet->current.alt = global_target.alt;
// 		}
// 		else{
// 			safe_alt = true;
// 		}
// 	}
// 	else{
// 		pos_sp_triplet->current.lon = global_target.lon;
// 		pos_sp_triplet->current.lat = global_target.lat;
// 		pos_sp_triplet->current.alt = global_target.alt;
// 		if (global_pos->alt < global_target.alt-max_thresh){
// 			safe_alt = false;
// 		}
// 	}
// 	// pos_sp_triplet->current.valid = true;
// 	pos_sp_triplet->current.type = position_setpoint_s::SETPOINT_TYPE_POSITION;
// 	_navigator->set_position_setpoint_triplet_updated();
// }

void
PrecLand::run_state_start()
{
	// check if target visible and go to horizontal approach
	// PX4_INFO("Running State Start");
	// distance_sensor_s local_dist;
	// if (_distance_sensor_sub.copy(&local_dist)) {
	// 	PX4_INFO("Going down: %f",double(local_dist.current_distance));
	// 	position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();
	// 	pos_sp_triplet->current.type = position_setpoint_s::SETPOINT_TYPE_POSITION;	
	// 	pos_sp_triplet->current.alt = z_init - (local_dist.current_distance-4.5f);
	// 	if (local_dist.current_distance <= 7.0f)
	// 	{
			if (switch_to_state_reach_altitude()) {
				PX4_INFO("Adjusting altitude");
				return;
			}
		// }
		_navigator->set_position_setpoint_triplet_updated();
	// }
	return;
}

void PrecLand::run_state_reach_altitude()
{
	if (switch_to_state_horizontal_approach()) {
		return;
	}
	position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();
	pos_sp_triplet->current.alt = _approach_alt;
	_navigator->set_position_setpoint_triplet_updated();
}


void
PrecLand::run_state_horizontal_approach()
{
	position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();

	// check if target visible, if not go to start
	// if (!c


	// if (check_state_conditions(PrecLandState::DescendAboveTarget)) {
	// 	PX4_INFO("Successful checking descent above target");
	// 	if (!_point_reached_time) {
	// 		_point_reached_time = hrt_absolute_time();
	// 	}

		// if (hrt_absolute_time() - _point_reached_time > 2000000) {
			// if close enough for descent above target go to descend above target
			if (switch_to_state_descend_above_target()) {
				old_delta_aim = pos_sp_triplet->current.alt;
				old_time = hrt_absolute_time();
				vehicle_local_position_s *vehicle_local_position = _navigator->get_local_position();
				old_xy = matrix::Vector2f(vehicle_local_position->x,vehicle_local_position->y);
				PX4_INFO("XY threshold reached, initiating descent");
				return;
			}
		// }

	// }
	// else{
	// 	PX4_INFO("unsuccessful checking descent above target");
	// }
	float x = _target_pose.x_abs;
	float y = _target_pose.y_abs;

	slewrate(x, y);

	// XXX need to transform to GPS coords because mc_pos_control only looks at that
	_map_ref.reproject(x, y, pos_sp_triplet->current.lat, pos_sp_triplet->current.lon);

	// pos_sp_triplet->current.alt = _approach_alt;
	pos_sp_triplet->current.type = position_setpoint_s::SETPOINT_TYPE_POSITION;

	_navigator->set_position_setpoint_triplet_updated();

}

void
PrecLand::run_state_descend_above_target()
{
	position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();
	bool lost_target = false;
	// check if target visible
	if (!check_state_conditions(PrecLandState::DescendAboveTarget)) {
		if (!switch_to_state_final_approach()) {
			PX4_WARN("%s, state: %i", LOST_TARGET_ERROR_MESSAGE, (int) _state);
			lost_target = true;
			// Stay at current position for searching for the target
			pos_sp_triplet->current.lat = _navigator->get_global_position()->lat;
			pos_sp_triplet->current.lon = _navigator->get_global_position()->lon;
			pos_sp_triplet->current.alt = _navigator->get_global_position()->alt;
		}
	}

	// XXX need to transform to GPS coords because mc_pos_control only looks at that
	_map_ref.reproject(_target_pose.x_abs, _target_pose.y_abs, pos_sp_triplet->current.lat, pos_sp_triplet->current.lon);

	// pos_sp_triplet->current.type = position_setpoint_s::SETPOINT_TYPE_LAND;

	// float aim_alt = find_setpoint();
	float aim_alt = find_setpoint_v2(lost_target);
	update_beacon();
	update_z_fit();
	update_setpoint_type(pos_sp_triplet);
	if (nest_z_fit && full_beacon &&reset_param){
		float land_trigger_val = 0.1f;
		param_set(land_trigger_param,&land_trigger_val);
		reset_param = false;
	}

	pos_sp_triplet->current.alt = aim_alt;
	_navigator->set_position_setpoint_triplet_updated();
}

void
PrecLand::update_setpoint_type(position_setpoint_triplet_s *pos_sp_triplet)
{
	if (nest_z_fit && full_beacon)
	{
		if (pos_sp_triplet->current.type != position_setpoint_s::SETPOINT_TYPE_LAND)
		{PX4_INFO("Switched to landing setpoint");pos_sp_triplet->current.type = position_setpoint_s::SETPOINT_TYPE_LAND;}
	}
	else
	{
		if (pos_sp_triplet->current.type != position_setpoint_s::SETPOINT_TYPE_POSITION)
		{PX4_INFO("Switched to position setpoint");pos_sp_triplet->current.type = position_setpoint_s::SETPOINT_TYPE_POSITION;}
	}
}

float 
PrecLand::find_setpoint()
{
	position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();
	vehicle_local_position_s *vehicle_local_position = _navigator->get_local_position();

	float xy_dist = sqrtf(pow(fabsf(_target_pose.x_abs - vehicle_local_position->x),2) + 
					pow(fabsf(_target_pose.y_abs - vehicle_local_position->y),2));
	float aim_alt = _navigator->get_global_position()->alt;

	float min_desc = 0.5f;
	float delta_z = fabsf(_target_pose.z_abs - vehicle_local_position->z);
	// PX4_INFO("DELTA Z: %f \n %f \n %f",double(_target_pose.z_abs),double(vehicle_local_position->z),double(delta_z));

	// if (delta_z > delta_z_ref){
	// 	pos_sp_triplet->current.type = position_setpoint_s::SETPOINT_TYPE_LAND;	
	// 	aim_alt -= (delta_z-delta_z_ref);
	// }
	// else{
	if (xy_dist < _param_pld_hacc_rad.get()){
		float cub_desc = min_desc;
		if (delta_z < delta_z_ref){
			cub_desc = (min_desc + (1.0f-min_desc)*sqrtf( delta_z/delta_z_ref ));
		}
		float lin_xy_filt = 1.0f - (xy_dist/_param_pld_hacc_rad.get());
		aim_alt -= K_speed * (cub_desc*delta_z+0.1f)*lin_xy_filt;
	}
	else if(xy_dist > _param_pld_hacc_rad.get() && delta_z < 0.6f){
		aim_alt += 0.3f;
	}
	if (delta_z > _param_pld_fappr_alt.get())
	{
		pos_sp_triplet->current.type = position_setpoint_s::SETPOINT_TYPE_POSITION;
	}
	else if (delta_z <= _param_pld_fappr_alt.get() && xy_dist < _param_pld_hacc_rad.get()*0.5f) {
		pos_sp_triplet->current.type = position_setpoint_s::SETPOINT_TYPE_POSITION;
	}
	else if (delta_z <= _param_pld_fappr_alt.get() && xy_dist > _param_pld_hacc_rad.get()*0.9f) {
		// aim_alt += 0.3f;
		pos_sp_triplet->current.type = position_setpoint_s::SETPOINT_TYPE_POSITION;
	}
	return aim_alt;
}

float 
PrecLand::find_setpoint_v2(bool lost_target)
{
	// position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();
	vehicle_local_position_s *vehicle_local_position = _navigator->get_local_position();
	float xy_dist = sqrtf(pow(fabsf(_target_pose.x_abs - vehicle_local_position->x),2) + 
					pow(fabsf(_target_pose.y_abs - vehicle_local_position->y),2));
	float aim_alt = _navigator->get_global_position()->alt;
	// float K_speed = 0.3f;
	if (vehicle_local_position->dist_bottom_valid)
	{
		if (vehicle_local_position->dist_bottom > 4.5f)	{
			if (fabsf(K_speed-1.0f)>=0.1f){PX4_INFO("Full speed");K_speed = 1.0f;}}
		if (vehicle_local_position->dist_bottom < 3.5f)	{
			if (fabsf(K_speed-0.8f)>=0.1f){PX4_INFO("Low speed");K_speed = 0.8f;}}
	}
	float K_emergency = 0.3f;
	float D = 0.0f;
	float K_vxy = 10.0f;
	float nest_height = 0.7f;
	float delta_z = fabsf(_target_pose.z_abs - vehicle_local_position->z);
	float height_to_xy = 0.31512856257f; // tan(17.5f*3.14f/180.0f) = 0.31512856257 
	float xy_dist_thresh = fmax(delta_z*height_to_xy,_param_pld_hacc_rad.get());

	float alt_index = fmax((xy_dist_thresh-xy_dist)/xy_dist_thresh,-1.0f);
	matrix::Vector2f xy(vehicle_local_position->x,vehicle_local_position->y);
	float dt = (hrt_absolute_time()-old_time);
	dt /= 1000000;
	int descent_state;
	// pos_sp_triplet->current.type = position_setpoint_s::SETPOINT_TYPE_POSITION;
	if (alt_index >= 0.0f)
	{	
		if (lost_target)
		{
			descent_state = DescentState::TargetLost;
			alt_index = -K_emergency;
		}
		if (alt_index >= 0.8f && delta_z<0.1f)
		{
			descent_state = DescentState::TargetCentered;
			alt_index = 1.0f;
			// pos_sp_triplet->current.type = position_setpoint_s::SETPOINT_TYPE_LAND;
		}
		// else{
		matrix::Vector2f vel_xy = (xy - old_xy)/dt;
		float vel_bump = fmax(1.0f,K_vxy*(vel_xy.norm())*2.0f);
		alt_index /= vel_bump;
		// }
	}
	else{
		descent_state = DescentState::TargetOffset;
	}
	if (descent_state != old_state)
	{
		switch (descent_state){
			case DescentState::TargetCentered:
				{PX4_INFO("Target is aligned with the drone");break;}
			case DescentState::TargetOffset:
				{PX4_INFO("Target is offset from the drone");break;}		
			case DescentState::TargetLost:
				{PX4_INFO("Target is lost");break;}
		}
		old_state = descent_state;
	}
	// float delta_aim = fmin(2.0f,(delta_z+0.1f));
	float delta_aim = fmin(3.0f*nest_height,delta_z);
	// PX4_INFO("delta_z : %f", double(delta_z));
	// PX4_INFO("alt_index : %f", double(alt_index));
	float delta_aim_d = (delta_aim - old_delta_aim)/dt;

	aim_alt -= alt_index*(delta_aim*K_speed+delta_aim_d*D);
	old_delta_aim = delta_aim;
	old_time = hrt_absolute_time();
	old_xy = xy;
	return aim_alt;
}

void
PrecLand::run_state_final_approach()
{
	position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();
	bool lost_target = false;
	// check if target visible
	if (!check_state_conditions(PrecLandState::DescendAboveTarget)) {
		// PX4_INFO("Attempting switch to final approach");
		if (!switch_to_state_final_approach()) {
			PX4_WARN("%s, state: %i", LOST_TARGET_ERROR_MESSAGE, (int) _state);
			lost_target = true;
			// Stay at current position for searching for the target
			pos_sp_triplet->current.lat = _navigator->get_global_position()->lat;
			pos_sp_triplet->current.lon = _navigator->get_global_position()->lon;
			pos_sp_triplet->current.alt = _navigator->get_global_position()->alt;
		}
	}

	// XXX need to transform to GPS coords because mc_pos_control only looks at that
	_map_ref.reproject(_target_pose.x_abs, _target_pose.y_abs, pos_sp_triplet->current.lat, pos_sp_triplet->current.lon);

	float aim_alt = find_setpoint_v2(lost_target);
	update_beacon();
	update_z_fit();
	update_setpoint_type(pos_sp_triplet);
	pos_sp_triplet->current.alt = aim_alt;
	_navigator->set_position_setpoint_triplet_updated();
}

void PrecLand::update_beacon()
{
	irlock_report_s irlock_report;
	if (_irlock_report_sub.update(&irlock_report)){
		if (irlock_report.size_x > 1.1f && irlock_report.size_y > 0.5f)
			{full_beacon = true;
			 last_full_beacon = hrt_absolute_time();
			}
		// else
		// 	{full_beacon = false;}
	}
}


void PrecLand::update_z_fit()
{
	distance_sensor_s distance_sensor;
	if (_distance_sensor_sub.update(&distance_sensor)){
		if (distance_sensor.current_distance < 0.1f)
			{nest_z_fit = true;}
		// else
		// 	{nest_z_fit = false;}
	}
}

void
PrecLand::run_state_search()
{
	// check if we can see the target
	if (check_state_conditions(PrecLandState::HorizontalApproach)) {
		if (!_target_acquired_time) {
			// target just became visible. Stop climbing, but give it some margin so we don't stop too apruptly
			_target_acquired_time = hrt_absolute_time();
			position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();
			float new_alt = _navigator->get_global_position()->alt + 1.0f;
			pos_sp_triplet->current.alt = new_alt < pos_sp_triplet->current.alt ? new_alt : pos_sp_triplet->current.alt;
			_navigator->set_position_setpoint_triplet_updated();
		}

	}

	// stay at that height for a second to allow the vehicle to settle
	if (_target_acquired_time && (hrt_absolute_time() - _target_acquired_time) > 1000000) {
		// try to switch to horizontal approach
		if (switch_to_state_horizontal_approach()) {
			return;
		}
	}

	// check if search timed out and go to fallback
	if (hrt_absolute_time() - _state_start_time > _param_pld_srch_tout.get()*SEC2USEC) {
		PX4_WARN("Search timed out");
		switch_to_state_fallback();
	}
}

void
PrecLand::run_state_fallback()
{
	// nothing to do, will land
}

bool
PrecLand::switch_to_state_start()
{
	if (check_state_conditions(PrecLandState::Start)) {
		position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();
		pos_sp_triplet->current.type = position_setpoint_s::SETPOINT_TYPE_POSITION;
		_navigator->set_position_setpoint_triplet_updated();
		_search_cnt++;

		_point_reached_time = 0;

		_state = PrecLandState::Start;
		return true;
	}

	return false;
}

bool PrecLand::switch_to_state_reach_altitude()
{
	if (check_state_conditions(PrecLandState::ReachAltitude)) {
		print_state_switch_message("adjusting altitude for lidar");
		PX4_INFO("Going to alt %f from %f",double(_approach_alt),double(_navigator->get_global_position()->alt));
		position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();
		_approach_alt = fmin(_global_target.alt+max_height,pos_sp_triplet->current.alt);
		pos_sp_triplet->current.alt = _approach_alt;
		_navigator->set_position_setpoint_triplet_updated();

		_point_reached_time = 0;

		_state = PrecLandState::ReachAltitude;
		return true;
	}

	return false;
}

bool
PrecLand::switch_to_state_horizontal_approach()
{
	if (check_state_conditions(PrecLandState::HorizontalApproach)) {
		print_state_switch_message("horizontal approach");
		_approach_alt = _navigator->get_global_position()->alt;

		_point_reached_time = 0;

		_state = PrecLandState::HorizontalApproach;
		PX4_INFO("Beacon found, adjusting XY");
		return true;
	}

	return false;
}

bool
PrecLand::switch_to_state_descend_above_target()
{
	distance_sensor_s local_dist;
	bool state_pass = false;
	bool sensor_pass = false;
	if (check_state_conditions(PrecLandState::DescendAboveTarget)) {state_pass = true;}
	if(_distance_sensor_sub.copy(&local_dist)) {sensor_pass = true;}
	if (sensor_pass && state_pass){
		print_state_switch_message("descend");
		vehicle_local_position_s *vehicle_local_position = _navigator->get_local_position();
		float delta_z = fabsf(_target_pose.z_abs - vehicle_local_position->z);
		if (local_dist.current_distance > 4.0f){
			delta_z_ref = delta_z - local_dist.current_distance + 4.0f;
		}
		else{
			delta_z_ref = delta_z;
		}
		_state = PrecLandState::DescendAboveTarget;
		_state_start_time = hrt_absolute_time();
		PX4_INFO("XY threshold reached, initiating descent");	
		return true;
	}
	// PX4_INFO("Failed to switch to descent state");
	// PX4_INFO("state pass %f",double(state_pass));
	// PX4_INFO("sensor pass %f",double(sensor_pass));
	return false;
}

bool
PrecLand::switch_to_state_final_approach()
{
	if (check_state_conditions(PrecLandState::FinalApproach)) {
		print_state_switch_message("final approach");
		_state = PrecLandState::FinalApproach;
		_state_start_time = hrt_absolute_time();
		return true;
	}
	else{
		switch_to_state_descend_above_target();
	}

	return false;
}

bool
PrecLand::switch_to_state_search()
{
	PX4_INFO("Climbing to search altitude.");
	vehicle_local_position_s *vehicle_local_position = _navigator->get_local_position();

	position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();
	pos_sp_triplet->current.alt = vehicle_local_position->ref_alt + _param_pld_srch_alt.get();
	pos_sp_triplet->current.type = position_setpoint_s::SETPOINT_TYPE_POSITION;
	_navigator->set_position_setpoint_triplet_updated();

	_target_acquired_time = 0;

	_state = PrecLandState::Search;
	_state_start_time = hrt_absolute_time();
	return true;
}

bool
PrecLand::switch_to_state_fallback()
{
	print_state_switch_message("fallback");
	position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();
	pos_sp_triplet->current.lat = _navigator->get_global_position()->lat;
	pos_sp_triplet->current.lon = _navigator->get_global_position()->lon;
	pos_sp_triplet->current.alt = _navigator->get_global_position()->alt;
	pos_sp_triplet->current.type = position_setpoint_s::SETPOINT_TYPE_LAND;
	_navigator->set_position_setpoint_triplet_updated();

	_state = PrecLandState::Fallback;
	_state_start_time = hrt_absolute_time();
	return true;
}

bool
PrecLand::switch_to_state_done()
{
	_state = PrecLandState::Done;
	_state_start_time = hrt_absolute_time();
	return true;
}

void PrecLand::print_state_switch_message(const char *state_name)
{
	PX4_INFO("Precland: switching to %s", state_name);
}

bool PrecLand::check_state_conditions(PrecLandState state)
{
	vehicle_local_position_s *vehicle_local_position = _navigator->get_local_position();

	switch (state) {
	case PrecLandState::Start:
		if (safe_alt){
			vehicle_global_position_s* global_pos = _navigator->get_global_position();
			// PX4_INFO("Distance: %f",double(sqrtf( 	pow((global_target.lon-global_pos->lon),2)+
			// 			pow((global_target.lat-global_pos->lat),2)+
			// 			pow((global_target.alt-global_pos->alt)*1e-6f,2))));
			if (sqrtf( 	pow((_global_target.lon-global_pos->lon),2)+
						pow((_global_target.lat-global_pos->lat),2)) < 2.5f*1e-7f)
			{
				return true;}
		}
		return false;
	case PrecLandState::ReachAltitude:
	// NEED TO ADD: What to check for ?
		return true;

	case PrecLandState::HorizontalApproach:

		// if we're already in this state, only want to make it invalid if we reached the target but can't see it anymore
		if (_state == PrecLandState::HorizontalApproach) {
			if (fabsf(_target_pose.x_abs - vehicle_local_position->x) < _param_pld_hacc_rad.get()
			    && fabsf(_target_pose.y_abs - vehicle_local_position->y) < _param_pld_hacc_rad.get()) {
				if (!(_target_pose_valid && _target_pose.abs_pos_valid))
					{PX4_INFO("Within reach, but can't get target position");}
				// we've reached the position where we last saw the target. If we don't see it now, we need to do something
				return _target_pose_valid && _target_pose.abs_pos_valid;

			} else {
				// We've seen the target sometime during horizontal approach.
				// Even if we don't see it as we're moving towards it, continue approaching last known location
				return true;
			}
		}
		return fabsf(_approach_alt-_navigator->get_global_position()->alt)< 1.0f && _target_pose_updated && _target_pose_valid && _target_pose.abs_pos_valid;

	case PrecLandState::DescendAboveTarget:
		// if we're already in this state, only leave it if target becomes unusable, don't care about horizontall offset to target
		if (_state == PrecLandState::DescendAboveTarget) {
			// if we're close to the ground, we're more critical of target timeouts so we quickly go into descend
			// PX4_INFO("Checking same state");
			if (check_state_conditions(PrecLandState::FinalApproach)) {
				return hrt_absolute_time() - _target_pose.timestamp < 500000; // 0.5s

			} else {
				return (_target_pose_valid && _target_pose.abs_pos_valid)||(hrt_absolute_time()-last_full_beacon<1000000);
			}

		} else {
			// if not already in this state, need to be above target to enter it
		// if (hrt_absolute_time() - _last_update > 2000000){
			_last_update = hrt_absolute_time();
			// float xy_dist = sqrtf(pow(fabsf(_target_pose.x_abs - vehicle_local_position->x),2) + 
			// 				pow(fabsf(_target_pose.y_abs - vehicle_local_position->y),2));
			// PX4_INFO("Current distance to target: %f | Target pose updated: %f | Target pose abs valid: %f",
				// double(xy_dist),double(_target_pose_updated),double(_target_pose.abs_pos_valid));
		// }
		return _target_pose_updated && _target_pose.abs_pos_valid
		       && fabsf(_target_pose.x_abs - vehicle_local_position->x) < _param_pld_hacc_rad.get()
		       && fabsf(_target_pose.y_abs - vehicle_local_position->y) < _param_pld_hacc_rad.get();
		}

	// case PrecLandState::FinalApproach:
	// 	return _target_pose_valid && _target_pose.abs_pos_valid
	// 	       && (_target_pose.z_abs - vehicle_local_position->z) < _param_pld_fappr_alt.get();

	case PrecLandState::FinalApproach:
	{
		float xy_dist = sqrtf(pow(fabsf(_target_pose.x_abs - vehicle_local_position->x),2) + 
						pow(fabsf(_target_pose.y_abs - vehicle_local_position->y),2));
		return _target_pose_valid && _target_pose.abs_pos_valid
		       && (_target_pose.z_abs - vehicle_local_position->z) < _param_pld_fappr_alt.get()
		       && (xy_dist < _param_pld_hacc_rad.get()*0.5f);}

	case PrecLandState::Search:
		return true;

	case PrecLandState::Fallback:
		return true;

	default:
		return false;
	}
}

void PrecLand::slewrate(float &sp_x, float &sp_y)
{
	matrix::Vector2f sp_curr(sp_x, sp_y);
	uint64_t now = hrt_absolute_time();

	float dt = (now - _last_slewrate_time);

	if (dt < 1) {
		// bad dt, can't divide by it
		return;
	}

	dt /= SEC2USEC;

	if (!_last_slewrate_time) {
		// running the first time since switching to precland

		// assume dt will be about 50000us
		dt = 50000 / SEC2USEC;

		// set a best guess for previous setpoints for smooth transition
		_sp_pev = _map_ref.project(_navigator->get_position_setpoint_triplet()->current.lat,
					   _navigator->get_position_setpoint_triplet()->current.lon);
		_sp_pev_prev(0) = _sp_pev(0) - _navigator->get_local_position()->vx * dt;
		_sp_pev_prev(1) = _sp_pev(1) - _navigator->get_local_position()->vy * dt;
	}

	_last_slewrate_time = now;

	// limit the setpoint speed to the maximum cruise speed
	matrix::Vector2f sp_vel = (sp_curr - _sp_pev) / dt; // velocity of the setpoints

	if (sp_vel.length() > _param_xy_vel_cruise) {
		sp_vel = sp_vel.normalized() * _param_xy_vel_cruise;
		sp_curr = _sp_pev + sp_vel * dt;
	}

	// limit the setpoint acceleration to the maximum acceleration
	matrix::Vector2f sp_acc = (sp_curr - _sp_pev * 2 + _sp_pev_prev) / (dt * dt); // acceleration of the setpoints

	if (sp_acc.length() > _param_acceleration_hor) {
		sp_acc = sp_acc.normalized() * _param_acceleration_hor;
		sp_curr = _sp_pev * 2 - _sp_pev_prev + sp_acc * (dt * dt);
	}

	// limit the setpoint speed such that we can stop at the setpoint given the maximum acceleration/deceleration
	float max_spd = sqrtf(_param_acceleration_hor * ((matrix::Vector2f)(_sp_pev - matrix::Vector2f(sp_x,
			      sp_y))).length());
	sp_vel = (sp_curr - _sp_pev) / dt; // velocity of the setpoints

	if (sp_vel.length() > max_spd) {
		sp_vel = sp_vel.normalized() * max_spd;
		sp_curr = _sp_pev + sp_vel * dt;
	}

	_sp_pev_prev = _sp_pev;
	_sp_pev = sp_curr;

	sp_x = sp_curr(0);
	sp_y = sp_curr(1);
}
