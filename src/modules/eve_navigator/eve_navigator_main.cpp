/****************************************************************************
 *
 *   Copyright (c) 2013-2017 PX4 Development Team. All rights reserved.
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
 * @file navigator_main.cpp
 *
 * Handles mission_ items, geo fencing and failsafe navigation behavior.
 * Published the position setpoint triplet for the position controller.
 *
 * @author Lorenz Meier <lorenz@px4.io>
 * @author Jean Cyr <jean.m.cyr@gmail.com>
 * @author Julian Oes <julian@oes.ch>
 * @author Anton Babushkin <anton.babushkin@me.com>
 * @author Thomas Gubler <thomasgubler@gmail.com>
 */
/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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
#include "eve_navigator.h"

#include <float.h>
#include <sys/stat.h>

#include <dataman/dataman.h>
#include <drivers/drv_hrt.h>
#include <lib/geo/geo.h>
#include <lib/mathlib/mathlib.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/events.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/tasks.h>
#include <systemlib/mavlink_log.h>

// #include "eve_navigator.h"
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>
#include <uORB/uORB.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_combined.h>

using namespace time_literals;

namespace eve_navigator
{
EveNavigator *g_navigator;
}

static bool send_vehicle_command(const uint32_t cmd, const float param1 = NAN, const float param2 = NAN,
				 const float param3 = NAN,  const float param4 = NAN, const double param5 = static_cast<double>(NAN),
				 const double param6 = static_cast<double>(NAN), const float param7 = NAN)
{
	vehicle_command_s vcmd{};
	vcmd.command = cmd;
	vcmd.param1 = param1;
	vcmd.param2 = param2;
	vcmd.param3 = param3;
	vcmd.param4 = param4;
	vcmd.param5 = param5;
	vcmd.param6 = param6;
	vcmd.param7 = param7;

	uORB::SubscriptionData<vehicle_status_s> vehicle_status_sub{ORB_ID(vehicle_status)};
	vcmd.source_system = vehicle_status_sub.get().system_id;
	vcmd.target_system = vehicle_status_sub.get().system_id;
	vcmd.source_component = vehicle_status_sub.get().component_id;
	vcmd.target_component = vehicle_status_sub.get().component_id;

	uORB::Publication<vehicle_command_s> vcmd_pub{ORB_ID(vehicle_command)};
	vcmd.timestamp = hrt_absolute_time();
	return vcmd_pub.publish(vcmd);
}


static bool wait_for_vehicle_command_reply(const uint32_t cmd,
		uORB::SubscriptionData<vehicle_command_ack_s> &vehicle_command_ack_sub)
{
	hrt_abstime start = hrt_absolute_time();

	while (hrt_absolute_time() - start < 100_ms) {
		if (vehicle_command_ack_sub.update()) {
			if (vehicle_command_ack_sub.get().command == cmd) {
				return vehicle_command_ack_sub.get().result == vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED;
			}
		}
		px4_usleep(10000);
	}
	return false;
}

EveNavigator::EveNavigator() :
	ModuleParams(nullptr),
	_loop_perf(perf_alloc(PC_ELAPSED, "eve_navigator")),
	_geofence(this),
	_gf_breach_avoidance(this),
	_mission(this),
	_loiter(this),
	_takeoff(this),
	_vtol_takeoff(this),
	_land(this),
	_precland(this),
	_move_to(this),
	_rtl(this)
{
	/* Create a list of our possible navigation types */
	_navigation_mode_array[0] = &_takeoff;
	_navigation_mode_array[1] = &_precland;
	_navigation_mode_array[2] = &_move_to;	
	_navigation_mode_array[3] = &_mission;
	_navigation_mode_array[4] = &_loiter;
	_navigation_mode_array[5] = &_rtl;
	_navigation_mode_array[6] = &_vtol_takeoff;
	_handle_back_trans_dec_mss = param_find("VT_B_DEC_MSS");
	_handle_reverse_delay = param_find("VT_B_REV_DEL");

	_handle_mpc_jerk_auto = param_find("MPC_JERK_AUTO");
	_handle_mpc_acc_hor = param_find("MPC_ACC_HOR");

	_local_pos_sub = orb_subscribe(ORB_ID(vehicle_local_position));
	// _mission_sub = orb_subscribe(ORB_ID(mission_));
	_vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));

	// Update the timeout used in mission_block (which can't hold it's own parameters)
	// _mission.set_payload_deployment_timeout(_param_mis_payload_delivery_timeout.get());
	loop_time = hrt_absolute_time();	
	loop_time2 = hrt_absolute_time();
	mission_timestamp = hrt_absolute_time()+1000000000;
	reset_triplets();
}

EveNavigator::~EveNavigator()
{
	perf_free(_loop_perf);
	orb_unsubscribe(_local_pos_sub);
	// orb_unsubscribe(_mission_sub);
	orb_unsubscribe(_vehicle_status_sub);
}

void EveNavigator::params_update()
{
	updateParams();

	if (_handle_back_trans_dec_mss != PARAM_INVALID) {
		param_get(_handle_back_trans_dec_mss, &_param_back_trans_dec_mss);
	}

	if (_handle_reverse_delay != PARAM_INVALID) {
		param_get(_handle_reverse_delay, &_param_reverse_delay);
	}

	if (_handle_mpc_jerk_auto != PARAM_INVALID) {
		param_get(_handle_mpc_jerk_auto, &_param_mpc_jerk_auto);
	}

	if (_handle_mpc_acc_hor != PARAM_INVALID) {
		param_get(_handle_mpc_acc_hor, &_param_mpc_acc_hor);
	}

	// _mission.set_payload_deployment_timeout(_param_mis_payload_delivery_timeout.get());
}

void EveNavigator::update_mission(mission_ &m){
	if (_server_mission_waypoints_request_sub.updated()) {
		server_mission_waypoints_request_s mission_update;
		if (_server_mission_waypoints_request_sub.copy(&mission_update)) {
			// if (check_timestamp(mission_update.timestamp)){
				// mission_ new_m = convert_mission(mission_update);
				// check_mission(m,new_m);
			// }
			// PX4_INFO("timestamp: %f",double(mission_update.timestamp));
			// PX4_INFO("timestamp - 1_s: %f",double(mission_update.timestamp-2_s));

			// if (mission_update.timestamp-mission_timestamp<3_s)	
			// {	
				set_mission(m,mission_update);
				PX4_INFO("Mission updated");
			// }
			// else{
			// 	mission_timestamp = mission_update.timestamp;
			// }
		}
	}
}

bool EveNavigator::check_timestamp(uint64_t request_time)
{
	// if (_system_time_sub.updated()){
	// 	system_time_s now_time;
	// 	if (_system_time_sub.copy(&new_time)) {
	// 		if (now_time.time_unix_usec/1000000.0f-request_time<5.0f)
	// 		{
	// 			return true
	// 		}
	// 	}
	// }
	// return false
	PX4_INFO("Now time: %f",double(hrt_absolute_time()));
	PX4_INFO("Request time: %f",double(hrt_absolute_time()));
	return (hrt_absolute_time()-request_time<5.0f);
}

void EveNavigator::set_mission(mission_ &m,server_mission_waypoints_request_s &incoming_msg)
{
	PX4_INFO("Setting mission");
	if (m.waypoints != nullptr)
	{		
		for (int i = 0; i < numWaypoints; i++) {
		    	delete[] m.waypoints[i];
			}
		delete[] m.waypoints;	
	}	
	mission_ new_mission;
	PX4_INFO("Message infos: lon %f , lat %f",double(incoming_msg.lon),double(incoming_msg.lat));
	new_mission.type = incoming_msg.mission_type;
	new_mission.lon = incoming_msg.lon;
	new_mission.lat = incoming_msg.lat;
	new_mission.alt = incoming_msg.alt;
	new_mission.yaw = incoming_msg.yaw;
	new_mission.cruise_alt = incoming_msg.cruise_alt;
	float **tempWaypoints;
	int totalWaypoints = sizeof(incoming_msg.waypoints);
	numWaypoints = totalWaypoints;
	for (int i = 0; i < totalWaypoints; i++) {
		PX4_INFO("Checking waypoint %d",i);
		if (incoming_msg.waypoints[i*3] < 0.0001f && incoming_msg.waypoints[i*3+1] < 0.0001f && incoming_msg.waypoints[i*3+2] < 0.0001f)
		{
			numWaypoints = i;
			break;
		}
	}
	PX4_INFO("numWaypoints updated");
	tempWaypoints = new float*[numWaypoints];
	for (int i = 0; i < numWaypoints; i++) {
	    float *waypoint_x = new float[3];
	    waypoint_x[0] = incoming_msg.waypoints[i*3];
	    waypoint_x[1] = incoming_msg.waypoints[i*3+1];
		waypoint_x[2] = incoming_msg.waypoints[i*3+2];
	    tempWaypoints[i] = waypoint_x;
	}
	// for (int i = 0; i < numWaypoints; i++){
	// 	// printf("\n Waypoint %d \n",i);
	// 	for (int j = 0; j < 3; j++)
	// 	{
	// 		// printf(" %f \n",double(tempWaypoints[i][j]));
	// 	}
	// }
	// PX4_INFO("Generated waypoints array");
	new_mission.waypoints = tempWaypoints;
	// PX4_INFO("set waypoints array to mission");
	waypoint_i = 0;
	new_mission.updated = true;
	m = new_mission;
	PX4_INFO("Mission type: %d",m.type);
	PX4_INFO("Mission updated: %d",m.updated);
	PX4_INFO("Mission lon: %f",double(m.lon));
	PX4_INFO("Mission lat: %f",double(m.lat));
	PX4_INFO("Mission alt: %f",double(m.alt));
	PX4_INFO("Mission yaw: %f",double(m.yaw));
	PX4_INFO("Mission cruise_alt: %f",double(m.cruise_alt));

	return;
}

// mission_ EveNavigator::convert_mission(server_mission_request_s incoming_msg){
// 	mission_ new_mission;
// 	// PX4_INFO("Message infos: lon %f , lat %f",(incoming_msg.lon),(incoming_msg.lat));
// 	new_mission.type = incoming_msg.mission_type;
// 	new_mission.lon = incoming_msg.lon;
// 	new_mission.lat = incoming_msg.lat;
// 	new_mission.alt = incoming_msg.alt;
// 	new_mission.yaw = incoming_msg.yaw;
// 	new_mission.cruise_alt = incoming_msg.cruise_alt;
// 	float **tempWaypoints;
// 	int totalWaypoints = sizeof(incoming_msg.waypoints)/3;
// 	numWaypoints = totalWaypoints;
// 	for (int i = 0; i < totalWaypoints; i++) {
// 		if (incoming_msg.waypoints[i*3] < 0.0001f && incoming_msg.waypoints[i*3+1] < 0.0001f && incoming_msg.waypoints[i*3+2] < 0.0001f)
// 		{
// 			numWaypoints = i;
// 			break;
// 		}
// 	}
// 	tempWaypoints = new float*[numWaypoints];
// 	for (int i = 0; i < numWaypoints; i++) {
// 	    float *waypoint_x = new float[3];
// 	    waypoint_x[0] = incoming_msg.waypoints[i*3];
// 	    waypoint_x[1] = incoming_msg.waypoints[i*3+1];
// 		waypoint_x[2] = incoming_msg.waypoints[i*3+2];
// 	    tempWaypoints[i] = waypoint_x;
// 	}
// 	for (int i = 0; i < numWaypoints; i++){
// 		printf("\n Waypoint %d \n",i);
// 		for (int j = 0; j < 3; j++)
// 		{
// 			printf(" %f \n",double(tempWaypoints[i][j]));
// 		}
// 	}
// 	new_mission.waypoints = tempWaypoints;
// 	waypoint_i = 0;
// 	return new_mission;
// }

// void EveNavigator::check_mission(mission_ &m,mission_ mission_update){
// 	if (m.type != mission_update.type || fabsf(m.lon - mission_update.lon) > fabsf(1e-6) 
// 		|| fabsf(m.lat - mission_update.lat) > fabsf(1e-6) || fabsf(m.cruise_alt - mission_update.cruise_alt)>0.5f)
// 	{
// 		m.waypoints
// 		m = mission_update;
// 		m.updated = true;
// 		PX4_INFO("New mission_ received! \n mission_type: %i | lat: %f | lon: %f | alt: %f | yaw %f | cruise alt %f",
// 			m.type,double(m.lat),double(m.lon),double(m.alt),double(m.yaw),double(m.cruise_alt));
// 	}
// 	for (int i = 0; i < numWaypoints; i++) {
//     	delete[] mission_update[i];
// 	}
// 	delete[] tempWaypoints;
// }

void EveNavigator::emergency_landing(){
	send_vehicle_command(vehicle_command_s::VEHICLE_CMD_NAV_LAND);
}

void EveNavigator::next_phase(mission_ &m,int &p,int &sub_p){
	switch(m.type){
		case MissionType::None:
		{
			switch(p){
				case Phase::Wait_:{
					break;
				}
				case Phase::Takeoff_:
				{
					next(p,sub_p,Phase::PrecLand_);
					break;
				}
				case Phase::PrecLand_:{
					if (run_precision_land(m,sub_p))
						{next(p,sub_p,Phase::Disarm_);}
					break;
				}
				case Phase::Disarm_:{
					if (wait_for_disarm(sub_p))
						{next(p,sub_p,Phase::Wait_);}
					break;
				}
			}
			break;
		}
		case MissionType::LandingLoop:
		{
			if (m.updated){
				PX4_INFO("Updating!");
				switch(p){
					case Phase::Wait_:
					{	
						p = Phase::Arm_;
						sub_p = SubPhase::Init;
						break;
					}
					case Phase::Arm_:
					{
						p = Phase::Arm_;
						sub_p = SubPhase::Init;
						break;
					}
					case Phase::Takeoff_:
					{
						sub_p = SubPhase::Init;
						break;
					}
					case Phase::PrecLand_:{
						if (!wait_for_altitude(m)){
							p = Phase::Takeoff_;
							sub_p = SubPhase::Init;
						}
						break;
					}
					case Phase::Disarm_:{
						p = Phase::Arm_;
						sub_p = SubPhase::Init;
						break;
					}
				}
				m.updated = false;
			}
			else{
				switch(p){
					case Phase::Wait_:{
						next(p,sub_p,Phase::Arm_);
						break;
					}
					case Phase::Arm_:
					{
						if (wait_for_arm(sub_p))
							{next(p,sub_p,Phase::Takeoff_);}
						break;
					}
					case Phase::Takeoff_:
					{
						if (run_takeoff(m,sub_p))
							{next(p,sub_p,Phase::PrecLand_);}
						break;
					}
					case Phase::PrecLand_:{
						if (run_precision_land(m,sub_p))
							{next(p,sub_p,Phase::Disarm_);}
						break;
					}
					case Phase::Disarm_:{
						if (wait_for_disarm(sub_p))
							{next(p,sub_p,Phase::Wait_);}
						break;
					}
				}
				break;
			}
			break;
		}
		case MissionType::MoveToNest:
		{
			if (m.updated){
				PX4_INFO("Updating!");
				switch(p){
					case Phase::Wait_:
					{	
						p = Phase::Arm_;
						sub_p = SubPhase::Init;
						break;
					}
					case Phase::Arm_:
					{
						p = Phase::Arm_;
						sub_p = SubPhase::Init;
						break;
					}
					case Phase::Takeoff_:
					{
						sub_p = SubPhase::Init;
						break;
					}
					case Phase::MoveTo_:{
						sub_p = SubPhase::Init;
						break;
					}
					case Phase::PrecLand_:{
						if (!check_arrival(m)){
							p = Phase::MoveTo_;
							sub_p = SubPhase::Init;
						}
						break;
					}
					case Phase::Disarm_:{
						p = Phase::Arm_;
						sub_p = SubPhase::Init;
						break;
					}
				}
			m.updated = false;
			}
			else{
				switch(p){
					case Phase::Arm_:
					{
						if (wait_for_arm(sub_p))
							{next(p,sub_p,Phase::Takeoff_);}
						break;
					}
					case Phase::Takeoff_:
					{
						if (run_takeoff(m,sub_p))
							{next(p,sub_p,Phase::MoveTo_);}
						break;
					}
					case Phase::MoveTo_:{
						if (move_to(m,sub_p))
							{next(p,sub_p,Phase::PrecLand_);}
						break;
					}
					case Phase::PrecLand_:{
						if (run_precision_land(m,sub_p))
							{next(p,sub_p,Phase::Disarm_);}
						break;
					}
					case Phase::Disarm_:{
						if (wait_for_disarm(sub_p))
							{next(p,sub_p,Phase::Wait_);}
						break;
					}
				}
				break;
			}
			break;
		}
		case MissionType::MoveToWaypoint:
		{
			if (m.updated){
				PX4_INFO("Updating!");
				switch(p){
					case Phase::Wait_:
					{	
						p = Phase::Arm_;
						sub_p = SubPhase::Init;
						break;
					}
					case Phase::Arm_:
					{
						p = Phase::Arm_;
						sub_p = SubPhase::Init;
						break;
					}
					case Phase::Takeoff_:
					{
						sub_p = SubPhase::Init;
						break;
					}
					case Phase::MoveTo_:{
						sub_p = SubPhase::Init;
						break;
					}
				}
			m.updated = false;
			}
			else{
				// PX4_INFO("%d , %d ",p,sub_p);
				switch(p){
					case Phase::Arm_:
					{
						if (wait_for_arm(sub_p))
							{next(p,sub_p,Phase::Takeoff_);}
						break;
					}
					case Phase::Takeoff_:
					{
						if (run_takeoff(m,sub_p))
							{next(p,sub_p,Phase::MoveTo_);}
						break;
					}
					case Phase::MoveTo_:{
						if (move_to(m,sub_p))
							{next(p,sub_p,Phase::PrecLand_);}
						break;
					}
				}
				break;
			}
			break;
		}
		case MissionType::Patrol:
		{
			if (m.updated){
				PX4_INFO("Updating!");
				switch(p){
					case Phase::Wait_:
					{	
						p = Phase::Arm_;
						sub_p = SubPhase::Init;
						break;
					}
					case Phase::Arm_:
					{
						p = Phase::Arm_;
						sub_p = SubPhase::Init;
						break;
					}
					case Phase::Takeoff_:
					{
						sub_p = SubPhase::Init;
						break;
					}
					case Phase::MoveToWaypoints_:{
						sub_p = SubPhase::Init;
						break;
					}
					case Phase::MoveTo_:{
						sub_p = SubPhase::Init;
						break;
					}
					case Phase::PrecLand_:{
						if (!check_arrival(m)){
							p = Phase::MoveTo_;
							sub_p = SubPhase::Init;
						}
						break;
					}
					case Phase::Disarm_:{
						p = Phase::Arm_;
						sub_p = SubPhase::Init;
						break;
					}
				}
				m.updated = false;
			}
			switch(p){
				case Phase::Arm_:
				{
					if (wait_for_arm(sub_p))
						{next(p,sub_p,Phase::Takeoff_);}
					break;
				}
				case Phase::Takeoff_:
				{
					if (run_takeoff(m,sub_p))
						{next(p,sub_p,Phase::MoveToWaypoints_);}
					break;
				}
				case Phase::MoveToWaypoints_:{
					// PX4_INFO("MoveToWaypoints_ phase");
					if (move_to_waypoint_i(m,sub_p))
						{	
							waypoint_i += 1;
							PX4_INFO("Aiming for waypoint_i: %d",waypoint_i);
							if (waypoint_i>=numWaypoints)
							{
								next(p,sub_p,Phase::MoveTo_);
							}
							else
							{next(p,sub_p,Phase::MoveToWaypoints_);}
						}
					break;
				}
				case Phase::MoveTo_:{
					if (move_to(m,sub_p))
						{next(p,sub_p,Phase::PrecLand_);}
					break;
				}
				case Phase::PrecLand_:{
					if (run_precision_land(m,sub_p))
						{next(p,sub_p,Phase::Disarm_);}
					break;
				}
				case Phase::Disarm_:{
					if (wait_for_disarm(sub_p))
						{next(p,sub_p,Phase::Wait_);}
					break;
				}
			}
			break;
		}
	}
	if (hrt_absolute_time()-loop_time > 30_s){ 	
		loop_time = hrt_absolute_time();
		switch(p)
		{
			case Phase::Takeoff_:{
				// distance_sensor_s local_dist;
				// if (_distance_sensor_sub.copy(&local_dist)) 
				PX4_INFO("Altitude percentage: %f",double(fabsf(100.0f*(init_alt-get_global_position()->alt)/(init_alt-m.cruise_alt))));
				// else{
				// 	PX4_INFO("Can't access altitude data");
				// }
				break;
			}
			case Phase::MoveTo_:{

				float K_amp = 100000.0f;
				if (fabsf(init_lon*K_amp-m.lon*K_amp)>0.01f)
				{
					PX4_INFO("Lon percentage: %f",double(100.0f*fabsf(init_lon*K_amp-float(get_global_position()->lon)*K_amp)/fabsf(init_lon*K_amp-m.lon*K_amp)));
				}
				else{					
					PX4_INFO("Lon percentage: %f",double(100.0f*fabsf(init_lon*K_amp-float(get_global_position()->lon)*K_amp)/fabsf(init_lon*K_amp-m.lon*K_amp+0.01f)));
				}
				if (fabsf(init_lat*K_amp-m.lat*K_amp)>0.01f)
				{
					PX4_INFO("Lat percentage: %f",double(100.0f*fabsf(init_lat*K_amp-float(get_global_position()->lat)*K_amp)/fabsf(init_lat*K_amp-m.lat*K_amp)));
				}
				else{
					PX4_INFO("Lat percentage: %f",double(100.0f*fabsf(init_lat*K_amp-float(get_global_position()->lat)*K_amp)/fabsf(init_lat*K_amp-m.lat*K_amp+0.01f)));	
				}
				
				break;
			}
			case Phase::MoveToWaypoints_:{

				float K_amp = 100000.0f;
				if (fabsf(init_lon*K_amp-m.lon*K_amp)>0.01f)
				{
					PX4_INFO("Lon percentage: %f",double(100.0f*fabsf(init_lon*K_amp-float(get_global_position()->lon)*K_amp)/fabsf(init_lon*K_amp-m.lon*K_amp)));
				}
				else{					
					PX4_INFO("Lon percentage: %f",double(100.0f*fabsf(init_lon*K_amp-float(get_global_position()->lon)*K_amp)/fabsf(init_lon*K_amp-m.lon*K_amp+0.01f)));
				}
				if (fabsf(init_lat*K_amp-m.lat*K_amp)>0.01f)
				{
					PX4_INFO("Lat percentage: %f",double(100.0f*fabsf(init_lat*K_amp-float(get_global_position()->lat)*K_amp)/fabsf(init_lat*K_amp-m.lat*K_amp)));
				}
				else{
					PX4_INFO("Lat percentage: %f",double(100.0f*fabsf(init_lat*K_amp-float(get_global_position()->lat)*K_amp)/fabsf(init_lat*K_amp-m.lat*K_amp+0.01f)));	
				}
				
				break;
			}
			case Phase::PrecLand_:{
				PX4_INFO("Precision landing ongoing");
				break;
			}
		}
		// if (m.type != MissionType::MoveToNest || p == Phase::Wait_)
		// {
		// 	m.type = MissionType::MoveToNest;
		// 	m.lon = 8.5456073f +0.00001f;
		// 	m.lat = 47.3977507f+0.0003f;
		// 	m.alt = 500.106f;
		// 	m.yaw = 1.0f;
		// 	m.updated = true;
		// 	p = Phase::Wait_;
		// 	sub_p = SubPhase::Init;
		// 	px4_usleep(10000);
		// }
		// update_mission(m);
	}			

	return;
}

void EveNavigator::next(int &p,int &sub_p,int new_phase){
	p = new_phase;
	sub_p = SubPhase::Init;
}

bool EveNavigator::wait_for_disarm(int &sub_p){
	switch (sub_p){
		case SubPhase::Init:
		{
			uORB::SubscriptionData<actuator_armed_s> actuator_armed_sub{ORB_ID(actuator_armed)};
			hrt_abstime start = hrt_absolute_time();
			while (hrt_absolute_time() - start < 1_s) {
				if (actuator_armed_sub.update()) {
					return !actuator_armed_sub.get().armed;
				}
			}
			break;
		}
	}
	return false;
}

bool EveNavigator::wait_for_arm(int &sub_p){
	switch (sub_p){
		case SubPhase::Init:
		{
			uORB::SubscriptionData<vehicle_command_ack_s> vehicle_command_ack_sub{ORB_ID(vehicle_command_ack)};
			send_vehicle_command(vehicle_command_s::VEHICLE_CMD_COMPONENT_ARM_DISARM,
			 static_cast<float>(vehicle_command_s::ARMING_ACTION_ARM),
			 0.f);
			if (!wait_for_vehicle_command_reply(vehicle_command_s::VEHICLE_CMD_COMPONENT_ARM_DISARM, vehicle_command_ack_sub)) 
			{
				PX4_WARN("Arming command rejected");
				return false;
			}	
			uORB::SubscriptionData<actuator_armed_s> actuator_armed_sub{ORB_ID(actuator_armed)};
			hrt_abstime start = hrt_absolute_time();
			while (hrt_absolute_time() - start < 1_s) {
				if (actuator_armed_sub.update()) {
					PX4_INFO("Arming approved");
					return actuator_armed_sub.get().armed;
				}
			}
			break;
		}
	}

	return false;
}

bool EveNavigator::battery_check(int &p){
	uORB::SubscriptionData<battery_status_s> battery_status_sub{ORB_ID(battery_status)};
	hrt_abstime start = hrt_absolute_time();
	while (hrt_absolute_time() - start < 10_s) {
		if (battery_status_sub.update()) {
			if (hrt_absolute_time()-ref_time > 10_s){ 				
				PX4_INFO("Battery remaining: %f %%",double(battery_status_sub.get().remaining*100.0f));
				ref_time = hrt_absolute_time();
			}
			if (p != 999){
				return battery_status_sub.get().remaining > 0.4f;
			}
			else{
				if (battery_status_sub.get().remaining > 0.9f){
					p = 0;
					return true;
				}
			}
		}
		px4_usleep(10000);
	}

	return false;
}
bool EveNavigator::run_takeoff(mission_ &m,int &sub_p){
	switch (sub_p){
		case SubPhase::Init:
		{	
			// px4_sleep(1);
			send_vehicle_command(vehicle_command_s::VEHICLE_CMD_NAV_TAKEOFF);
			setup_takeoff(m);
			_navigation_mode_array[AutoPhase::AutoTakeoff]->run(true);
			sub_p = SubPhase::Progress;
			break;
		}
		case SubPhase::Progress:
		{
			// setup_takeoff(m);
			_navigation_mode_array[AutoPhase::AutoTakeoff]->run(true);
			if (wait_for_altitude(m)){
				sub_p = SubPhase::Complete;
			}
			break;
		}
		case SubPhase::Complete:
			_navigation_mode_array[AutoPhase::AutoTakeoff]->run(false);
			return true;
	}
	publish_position_setpoint_triplet();
	return false;
}

bool EveNavigator::wait_for_altitude(mission_ &m){
	if (fabsf(m.cruise_alt-get_global_position()->alt) <= 0.5f){
		PX4_INFO("Desired altitude reached");
		return true;
	}
	return false;
}

bool EveNavigator::move_to_waypoint_i(mission_ &m,int &sub_p){
	switch (sub_p){
		case SubPhase::Init:
		{
			setup_moveto_waypoint_i(m);
			_navigation_mode_array[AutoPhase::AutoLoiter]->run(true);
			sub_p = SubPhase::Progress;
			break;
		}
		case SubPhase::Progress:
		{
			_navigation_mode_array[AutoPhase::AutoLoiter]->run(true);
			if (check_arrival_waypoint_i(m))
			{sub_p = SubPhase::Complete;}
			break;
		}
		case SubPhase::Complete:
		{
			_navigation_mode_array[AutoPhase::AutoLoiter]->run(false);
			return true;
		}
	}
	publish_position_setpoint_triplet();
	return false;
}


bool EveNavigator::move_to(mission_ &m,int &sub_p){
	switch (sub_p){
		case SubPhase::Init:
		{
			setup_moveto(m);
			_navigation_mode_array[AutoPhase::AutoLoiter]->run(true);
			sub_p = SubPhase::Progress;
			break;
		}
		case SubPhase::Progress:
		{
			_navigation_mode_array[AutoPhase::AutoLoiter]->run(true);
			if (check_arrival(m))
			{sub_p = SubPhase::Complete;}
			break;
		}
		case SubPhase::Complete:
		{
			_navigation_mode_array[AutoPhase::AutoLoiter]->run(false);
			return true;
		}
	}
	publish_position_setpoint_triplet();
	return false;
}

void EveNavigator::setup_precland(mission_ &m){
	position_setpoint_s global_target;
	global_target.lat = m.lat;	
	global_target.lon = m.lon;	
	global_target.alt = m.alt;
	_precland.set_target(global_target);
}

bool EveNavigator::run_precision_land(mission_ &m,int &sub_p){
	switch (sub_p){
		case SubPhase::Init:
		{
			setup_precland(m);
			send_vehicle_command(vehicle_command_s::VEHICLE_CMD_NAV_LOITER_UNLIM,220);
			_navigation_mode_array[AutoPhase::AutoPrecLand]->run(true);
			_precland.StartState();
			sub_p = SubPhase::Progress;
			break;
		}
		case SubPhase::Progress:
		{			
			_navigation_mode_array[AutoPhase::AutoPrecLand]->run(true);
			if (landing_update()){
				sub_p = SubPhase::Complete;
			}
		break;
		}
		case SubPhase::Complete:
			_navigation_mode_array[AutoPhase::AutoPrecLand]->run(false);
			return true;
	}
	publish_position_setpoint_triplet();
	return false;
}

bool EveNavigator::landing_update()
{
	if (_vehicle_land_detected_sub.updated()) {
		vehicle_land_detected_s	_vehicle_land_detected;
		if (_vehicle_land_detected_sub.copy(&_vehicle_land_detected)) {
			// PX4_INFO("Landing state: %i",_vehicle_land_detected.landed);
			if (_vehicle_land_detected.landed){_precland.reset_precland();};
			return _vehicle_land_detected.landed;
		}
	}
	return false;
}

bool EveNavigator::check_arrival(mission_ &m){
	float lon = get_global_position()->lon;
	float lat = get_global_position()->lat;
	if (sqrtf( 	pow((m.lon-lon),2)+	pow((m.lat-lat),2)+
			pow((m.cruise_alt-get_global_position()->alt)*1e-7f,2)) < 2.5f*1e-7f){
		return true;
	}
	else{
		return false;
	}
}

bool EveNavigator::check_arrival_waypoint_i(mission_ &m){
	float lon = get_global_position()->lon;
	float lat = get_global_position()->lat;
	float m_lat = m.waypoints[waypoint_i][0];
	float m_lon = m.waypoints[waypoint_i][1];
	if (sqrtf( 	pow((m_lon-lon),2)+	pow((m_lat-lat),2)+
			pow((m.cruise_alt-get_global_position()->alt)*1e-7f,2)) < 7.5f*1e-7f){
		return true;
	}
	else{
		return false;
	}
}


bool EveNavigator::wait_for_state(unsigned state){
	hrt_abstime start_time = hrt_absolute_time();
	while (hrt_absolute_time() - start_time < 3_s)
	{
		vehicle_command_s cmd;
		if (_vehicle_command_sub.copy(&cmd)) {
			if (cmd.command == state){
				return true;
			}
		}
	} 
	return false;
}

int EveNavigator::query_vehicle_state(){
	const unsigned last_generation = _vehicle_command_sub.get_last_generation();
	vehicle_command_s cmd;
	if (_vehicle_command_sub.copy(&cmd)) {
		if (_vehicle_command_sub.get_last_generation() != last_generation + 1) {
			PX4_INFO("vehicle_command lost, generation %u -> %u", last_generation, _vehicle_command_sub.get_last_generation());
		}
		return cmd.command;
	}
	return 99999;
}

bool EveNavigator::force_vtol()
{
	return _vstatus.is_vtol &&
	       (_vstatus.vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING || _vstatus.in_transition_to_fw)
	       && _param_nav_force_vt.get();
}

void EveNavigator::parameters_update(bool force)
{
	// check for parameter updates
	if (_parameter_update_sub.updated() || force) {
		// clear update
		parameter_update_s update;
		_parameter_update_sub.copy(&update);

		// update parameters from storage
		updateParams();
	}
}

void EveNavigator::setup_moveto_waypoint_i(mission_ &m){
	position_setpoint_triplet_s *rep = get_reposition_triplet();
	position_setpoint_triplet_s *curr = get_position_setpoint_triplet();
	rep->previous.yaw = get_local_position()->heading;
	rep->previous.lat = get_global_position()->lat;
	rep->previous.lon = get_global_position()->lon;
	rep->previous.alt = get_global_position()->alt;
	curr->previous.yaw = get_local_position()->heading;
	curr->previous.lat = get_global_position()->lat;
	curr->previous.lon = get_global_position()->lon;
	curr->previous.alt = get_global_position()->alt;
	rep->current.type = position_setpoint_s::SETPOINT_TYPE_LOITER;
	rep->current.cruising_speed = get_cruising_speed();
	rep->current.cruising_throttle = get_cruising_throttle();
	rep->current.acceptance_radius = get_acceptance_radius();
	rep->current.yaw = m.yaw;
	rep->current.yaw_valid = true;
	rep->current.lat = m.waypoints[waypoint_i][0];
	rep->current.lon = m.waypoints[waypoint_i][1];
	rep->current.alt = m.cruise_alt;
	curr->current.yaw = m.yaw;
	curr->current.lat = m.waypoints[waypoint_i][0];
	curr->current.lon = m.waypoints[waypoint_i][1];
	curr->current.alt = m.cruise_alt;
	rep->previous.timestamp = hrt_absolute_time();
	rep->current.valid = true;
	rep->current.timestamp = hrt_absolute_time();
	rep->next.valid = false;
}


void EveNavigator::setup_moveto(mission_ &m){

	// bool reposition_valid = true;

	// vehicle_global_position_s position_setpoint{};
	// position_setpoint.lat = m.lat;
	// position_setpoint.lon = m.lon;
	// position_setpoint.alt = m.alt;

	// if (have_geofence_position_data) {
	// 	reposition_valid = geofence_allows_position(position_setpoint);
	// }

	// if (reposition_valid) {
		position_setpoint_triplet_s *rep = get_reposition_triplet();
		position_setpoint_triplet_s *curr = get_position_setpoint_triplet();

		// store current position as previous position and goal as next
		rep->previous.yaw = get_local_position()->heading;
		rep->previous.lat = get_global_position()->lat;
		rep->previous.lon = get_global_position()->lon;
		rep->previous.alt = get_global_position()->alt;
		curr->previous.yaw = get_local_position()->heading;
		curr->previous.lat = get_global_position()->lat;
		curr->previous.lon = get_global_position()->lon;
		curr->previous.alt = get_global_position()->alt;


		rep->current.type = position_setpoint_s::SETPOINT_TYPE_LOITER;

		// bool only_alt_change_requested = false;

		// If no argument for ground speed, use default value.
		// if (cmd.param1 <= 0 || !PX4_ISFINITE(cmd.param1)) {
			rep->current.cruising_speed = get_cruising_speed();

		// } else {
		// 	rep->current.cruising_speed = cmd.param1;
		// }

		rep->current.cruising_throttle = get_cruising_throttle();
		rep->current.acceptance_radius = get_acceptance_radius();

		// Go on and check which changes had been requested
		// if (PX4_ISFINITE(cmd.param4)) {
			// rep->current.yaw = cmd.param4;
			rep->current.yaw = m.yaw;
			rep->current.yaw_valid = true;

		// } else {
		// 	rep->current.yaw = NAN;
		// 	rep->current.yaw_valid = false;
		// }

		// if (PX4_ISFINITE(cmd.param5) && PX4_ISFINITE(cmd.param6)) {
			// Position change with optional altitude change
			rep->current.lat = m.lat;
			rep->current.lon = m.lon;
			rep->current.alt = m.cruise_alt;

			curr->current.yaw = m.yaw;
			curr->current.lat = m.lat;
			curr->current.lon = m.lon;
			curr->current.alt = m.cruise_alt;

			// if (PX4_ISFINITE(cmd.param7)) {
			// 	rep->current.alt = cmd.param7;

			// } else {
			// 	rep->current.alt = get_global_position()->alt;
			// }

		// } else if (PX4_ISFINITE(cmd.param7) || PX4_ISFINITE(cmd.param4)) {
		// 	// Position is not changing, thus we keep the setpoint
		// 	rep->current.lat = PX4_ISFINITE(curr->current.lat) ? curr->current.lat : get_global_position()->lat;
		// 	rep->current.lon = PX4_ISFINITE(curr->current.lon) ? curr->current.lon : get_global_position()->lon;

		// 	if (PX4_ISFINITE(cmd.param7)) {
		// 		rep->current.alt = cmd.param7;
		// 		only_alt_change_requested = true;

		// 	} else {
		// 		rep->current.alt = get_global_position()->alt;
		// 	}

		// } else {
		// 	// All three set to NaN - pause vehicle
		// 	rep->current.alt = get_global_position()->alt;

		// 	if (_vstatus.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING
		// 	    && (get_position_setpoint_triplet()->current.type != position_setpoint_s::SETPOINT_TYPE_TAKEOFF)) {

		// 		calculate_breaking_stop(rep->current.lat, rep->current.lon, rep->current.yaw);
		// 		rep->current.yaw_valid = true;

		// 	} else {
		// 		// For fixedwings we can use the current vehicle's position to define the loiter point
		// 		rep->current.lat = get_global_position()->lat;
		// 		rep->current.lon = get_global_position()->lon;
		// 	}
		// }

		// if (only_alt_change_requested) {
		// 	if (PX4_ISFINITE(curr->current.loiter_radius) && curr->current.loiter_radius > 0) {
		// 		rep->current.loiter_radius = curr->current.loiter_radius;


		// 	} else {
		// 		rep->current.loiter_radius = get_loiter_radius();
		// 	}

		// 	rep->current.loiter_direction_counter_clockwise = curr->current.loiter_direction_counter_clockwise;
		// }

		rep->previous.timestamp = hrt_absolute_time();

		rep->current.valid = true;
		rep->current.timestamp = hrt_absolute_time();

		rep->next.valid = false;
	// position_setpoint_triplet_s *rep = get_moveto_triplet();

	// // store current position as previous position and goal as next
	// rep->previous.yaw = get_local_position()->heading;
	// rep->previous.lat = get_global_position()->lat;
	// rep->previous.lon = get_global_position()->lon;
	// rep->previous.alt = get_global_position()->alt;

	// rep->current.loiter_radius = get_loiter_radius();
	// rep->current.loiter_direction_counter_clockwise = false;
	// rep->current.type = position_setpoint_s::SETPOINT_TYPE_POSITION;

	// if (home_global_position_valid()) {
	// 	// Only set yaw if we know the true heading
	// 	// We assume that the heading is valid when the global position is valid because true heading
	// 	// is required to fuse NE (e.g.: GNSS) data. // TODO: we should be more explicit here
	// 	rep->current.yaw = m.yaw;

	// 	rep->previous.valid = true;
	// 	rep->previous.timestamp = hrt_absolute_time();

	// } else {
	// 	rep->current.yaw = get_local_position()->heading;
	// 	rep->previous.valid = false;
	// }
	// rep->current.lat = m.lat;
	// rep->current.lon = m.lon;
	// rep->current.alt = m.alt;

	// rep->current.valid = true;
	// rep->current.timestamp = hrt_absolute_time();

	// rep->next.valid = false;
}

void EveNavigator::setup_takeoff(mission_ &m){
	// alt_ref = get_global_position()->alt;
	// delta_z_ref = m.alt-alt_ref;
	position_setpoint_triplet_s *rep = get_takeoff_triplet();

	// store current position as previous position and goal as next
	rep->previous.yaw = get_local_position()->heading;
	rep->previous.lat = get_global_position()->lat;
	rep->previous.lon = get_global_position()->lon;
	rep->previous.alt = get_global_position()->alt;

	rep->current.loiter_radius = get_loiter_radius();
	rep->current.loiter_direction_counter_clockwise = false;
	rep->current.type = position_setpoint_s::SETPOINT_TYPE_TAKEOFF;

	if (home_global_position_valid()) {
		// Only set yaw if we know the true heading
		// We assume that the heading is valid when the global position is valid because true heading
		// is required to fuse NE (e.g.: GNSS) data. // TODO: we should be more explicit here
		rep->current.yaw = m.yaw;

		rep->previous.valid = true;
		rep->previous.timestamp = hrt_absolute_time();

	} else {
		rep->current.yaw = get_local_position()->heading;
		rep->previous.valid = false;
	}
	rep->current.lat = rep->previous.lat;
	rep->current.lon = rep->previous.lon;
	rep->current.alt = m.cruise_alt;

	rep->current.valid = true;
	rep->current.timestamp = hrt_absolute_time();

	rep->next.valid = false;
	// PX4_INFO("Takeoff infos: %f | %f | %f | %f | %f | %f | %f " ,double(rep->previous.yaw),double(rep->previous.lat)
	// 														,double(rep->previous.lon),double(rep->previous.alt)
	// 														,double(m.yaw),double(rep->previous.timestamp)
	// 														,double(get_local_position()->heading)															 
	// 														);
	// CMD_NAV_TAKEOFF is acknowledged by commander
}

// void EveNavigator::setup_takeoff_cmd(vehicle_command_s cmd){
// 	position_setpoint_triplet_s *rep = get_takeoff_triplet();

// 	// store current position as previous position and goal as next
// 	rep->previous.yaw = get_local_position()->heading;
// 	rep->previous.lat = get_global_position()->lat;
// 	rep->previous.lon = get_global_position()->lon;
// 	rep->previous.alt = get_global_position()->alt;

// 	rep->current.loiter_radius = get_loiter_radius();
// 	rep->current.loiter_direction_counter_clockwise = false;
// 	rep->current.type = position_setpoint_s::SETPOINT_TYPE_TAKEOFF;

// 	if (home_global_position_valid()) {
// 		// Only set yaw if we know the true heading
// 		// We assume that the heading is valid when the global position is valid because true heading
// 		// is required to fuse NE (e.g.: GNSS) data. // TODO: we should be more explicit here
// 		rep->current.yaw = cmd.param4;

// 		rep->previous.valid = true;
// 		rep->previous.timestamp = hrt_absolute_time();

// 	} else {
// 		rep->current.yaw = get_local_position()->heading;
// 		rep->previous.valid = false;
// 	}

// 	if (PX4_ISFINITE(cmd.param5) && PX4_ISFINITE(cmd.param6)) {
// 		rep->current.lat = cmd.param5;
// 		rep->current.lon = cmd.param6;

// 	} else {
// 		// If one of them is non-finite set the current global position as target
// 		rep->current.lat = get_global_position()->lat;
// 		rep->current.lon = get_global_position()->lon;

// 	}

// 	rep->current.alt = cmd.param7;

// 	rep->current.valid = true;
// 	rep->current.timestamp = hrt_absolute_time();

// 	rep->next.valid = false;

// 	// CMD_NAV_TAKEOFF is acknowledged by commander

// }

void EveNavigator::run()
{
	params_update();

	/* wakeup source(s) */
	px4_pollfd_struct_t fds[3] {};

	/* Setup of loop */
	fds[0].fd = _local_pos_sub;
	fds[0].events = POLLIN;
	fds[1].fd = _vehicle_status_sub;
	fds[1].events = POLLIN;
	fds[2].fd = _mission_sub;
	fds[2].events = POLLIN;

	/* rate-limit position subscription to 20 Hz / 50 ms */
	orb_set_interval(_local_pos_sub, 50);
	int current_phase = Phase::Wait_;
	int current_subphase = SubPhase::Init;
	mission_ current_mission;
	current_mission.type = MissionType::None; //mission_type
	current_mission.updated = false; //updated

	ref_time = hrt_absolute_time();
	int old_type = MissionType::None;
	int old_phase = Phase::Wait_;
	int old_subphase = SubPhase::Init;

	while (!should_exit()) {
		/* wait for up to 1000ms for data */
		int pret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 1000);

		if (pret == 0) {
			/* Let the loop run anyway, don't do `continue` here. */

		} else if (pret < 0) {
			/* this is undesirable but not much we can do - might want to flag unhappy status */
			PX4_ERR("poll error %d, %d", pret, errno);
			px4_usleep(10000);
			continue;
		}

		perf_begin(_loop_perf);

		orb_copy(ORB_ID(vehicle_local_position), _local_pos_sub, &_local_pos);

		/* gps updated */
		if (_gps_pos_sub.updated()) {
			_gps_pos_sub.copy(&_gps_pos);
		}

		/* global position updated */
		if (_global_pos_sub.updated()) {
			_global_pos_sub.copy(&_global_pos);
		}

		/* check for parameter updates */
		if (_parameter_update_sub.updated()) {
			// clear update
			parameter_update_s pupdate;
			_parameter_update_sub.copy(&pupdate);
			params_update();
		}

		_land_detected_sub.update(&_land_detected);
		_position_controller_status_sub.update();
		_home_pos_sub.update(&_home_pos);

		/* Check for traffic */
		check_traffic();
		// current_mission.type = MissionType::MoveToNest;
		if (old_type != current_mission.type || old_phase != current_phase || old_subphase != current_subphase)
		{
			PX4_INFO("Mission: %i | Phase: %i | SubPhase: %i",current_mission.type,current_phase,current_subphase);
			old_type = current_mission.type;
			old_phase = current_phase;
			old_subphase = current_subphase;
		}
	
		// vehicle_local_position_s *vehicle_local_position = get_local_position();
		// PX4_INFO("Local Pos: ");
		// PX4_INFO("X: %f",double(vehicle_local_position->x));
		// PX4_INFO("Y: %f",double(vehicle_local_position->y));
		// PX4_INFO("Z: %f",double(vehicle_local_position->z));
		// if (current_mission.type == MissionType::MoveToNest){

		update_mission(current_mission);
		if (hrt_absolute_time()-loop_time2 > 50_ms) 	
		{
			server_mission_state_s mission_state;
			mission_state.timestamp = hrt_absolute_time();
			mission_state.lat = current_mission.lat;
			mission_state.lon = current_mission.lon;
			mission_state.alt = current_mission.alt;
			mission_state.cruise_alt = current_mission.cruise_alt;
			mission_state.mission_type = current_mission.type;
			mission_state.state = current_phase;
			_server_mission_state_pub.publish(mission_state);
			next_phase(current_mission,current_phase,current_subphase);
			loop_time2 = hrt_absolute_time();
		}
		if (_pos_sp_triplet_updated) {
			publish_position_setpoint_triplet();
		}
		// }
		perf_end(_loop_perf);
		px4_usleep(50000);

	}
}


int EveNavigator::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("EveNavigator",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_NAVIGATION,
				      PX4_STACK_ADJUSTED(1952),
				      (px4_main_t)&run_trampoline,
				      (char *const *)argv);

	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}

	return 0;
}

EveNavigator *EveNavigator::instantiate(int argc, char *argv[])
{
	EveNavigator *instance = new EveNavigator();

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
	}

	return instance;
}

int EveNavigator::print_status()
{
	PX4_INFO("Running");

	// _geofence.printStatus();
	return 0;
}

void EveNavigator::publish_position_setpoint_triplet()
{
	_pos_sp_triplet.timestamp = hrt_absolute_time();
	_pos_sp_triplet_pub.publish(_pos_sp_triplet);
	_pos_sp_triplet_updated = false;
}

float EveNavigator::get_default_acceptance_radius()
{
	return _param_nav_acc_rad.get();
}

float EveNavigator::get_altitude_acceptance_radius()
{
	
	float alt_acceptance_radius = _param_nav_mc_alt_rad.get();

	const position_controller_status_s &pos_ctrl_status = _position_controller_status_sub.get();

	if ((pos_ctrl_status.timestamp > _pos_sp_triplet.timestamp)
	    && pos_ctrl_status.altitude_acceptance > alt_acceptance_radius) {
		alt_acceptance_radius = pos_ctrl_status.altitude_acceptance;
	}

	return alt_acceptance_radius;
}

float EveNavigator::get_cruising_speed()
{
	/* there are three options: The mission_-requested cruise speed, or the current hover / plane speed */
	if (_vstatus.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING) {
		if (_mission_cruising_speed_mc > 0.0f) {
			return _mission_cruising_speed_mc;

		} else {
			return -1.0f;
		}

	} else {
		if (_mission_cruising_speed_fw > 0.0f) {
			return _mission_cruising_speed_fw;

		} else {
			return -1.0f;
		}
	}
}

void EveNavigator::set_cruising_speed(float speed)
{
	if (_vstatus.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING) {
		_mission_cruising_speed_mc = speed;

	} else {
		_mission_cruising_speed_fw = speed;
	}
}

void EveNavigator::reset_cruising_speed()
{
	_mission_cruising_speed_mc = -1.0f;
	_mission_cruising_speed_fw = -1.0f;
}

void EveNavigator::reset_triplets()
{
	reset_position_setpoint(_pos_sp_triplet.previous);
	reset_position_setpoint(_pos_sp_triplet.current);
	reset_position_setpoint(_pos_sp_triplet.next);

	_pos_sp_triplet_updated = true;
}

void EveNavigator::reset_position_setpoint(position_setpoint_s &sp)
{
	sp = position_setpoint_s{};
	sp.timestamp = hrt_absolute_time();
	sp.lat = static_cast<double>(NAN);
	sp.lon = static_cast<double>(NAN);
	sp.loiter_radius = get_loiter_radius();
	sp.acceptance_radius = get_default_acceptance_radius();
	sp.cruising_speed = get_cruising_speed();
	sp.cruising_throttle = get_cruising_throttle();
	sp.valid = false;
	sp.type = position_setpoint_s::SETPOINT_TYPE_IDLE;
	sp.disable_weather_vane = false;
	sp.loiter_direction_counter_clockwise = false;
}

float EveNavigator::get_cruising_throttle()
{
	/* Return the mission_-requested cruise speed, or default FW_THR_TRIM value */
	if (_mission_throttle > FLT_EPSILON) {
		return _mission_throttle;

	} else {
		return NAN;
	}
}

float EveNavigator::get_acceptance_radius()
{
	float acceptance_radius = get_default_acceptance_radius(); // the value specified in the parameter NAV_ACC_RAD
	const position_controller_status_s &pos_ctrl_status = _position_controller_status_sub.get();

	// for fixed-wing and rover, return the max of NAV_ACC_RAD and the controller acceptance radius (e.g. L1 distance)
	if (_vstatus.vehicle_type != vehicle_status_s::VEHICLE_TYPE_ROTARY_WING
	    && PX4_ISFINITE(pos_ctrl_status.acceptance_radius) && pos_ctrl_status.timestamp != 0) {

		acceptance_radius = math::max(acceptance_radius, pos_ctrl_status.acceptance_radius);
	}

	return acceptance_radius;
}

float EveNavigator::get_yaw_acceptance(float mission_item_yaw)
{
	float yaw = mission_item_yaw;

	const position_controller_status_s &pos_ctrl_status = _position_controller_status_sub.get();

	// if yaw_acceptance from position controller is NaN overwrite the mission_ item yaw such that
	// the waypoint can be reached from any direction
	if ((pos_ctrl_status.timestamp > _pos_sp_triplet.timestamp) && !PX4_ISFINITE(pos_ctrl_status.yaw_acceptance)) {
		yaw = pos_ctrl_status.yaw_acceptance;
	}

	return yaw;
}

void EveNavigator::fake_traffic(const char *callsign, float distance, float direction, float traffic_heading,
			     float altitude_diff, float hor_velocity, float ver_velocity, int emitter_type)
{
	double lat{0.0};
	double lon{0.0};

	waypoint_from_heading_and_distance(get_global_position()->lat, get_global_position()->lon, direction, distance, &lat,
					   &lon);
	float alt = get_global_position()->alt + altitude_diff;

	// float vel_n = get_global_position()->vel_n;
	// float vel_e = get_global_position()->vel_e;
	// float vel_d = get_global_position()->vel_d;

	transponder_report_s tr{};
	tr.timestamp = hrt_absolute_time();
	tr.icao_address = 1234;
	tr.lat = lat; // Latitude, expressed as degrees
	tr.lon = lon; // Longitude, expressed as degrees
	tr.altitude_type = 0;
	tr.altitude = alt;
	tr.heading = traffic_heading; //-atan2(vel_e, vel_n); // Course over ground in radians
	tr.hor_velocity	= hor_velocity; //sqrtf(vel_e * vel_e + vel_n * vel_n); // The horizontal velocity in m/s
	tr.ver_velocity = ver_velocity; //-vel_d; // The vertical velocity in m/s, positive is up
	strncpy(&tr.callsign[0], callsign, sizeof(tr.callsign) - 1);
	tr.callsign[sizeof(tr.callsign) - 1] = 0;
	tr.emitter_type = emitter_type; // Type from ADSB_EMITTER_TYPE enum
	tr.tslc = 2; // Time since last communication in seconds
	tr.flags = transponder_report_s::PX4_ADSB_FLAGS_VALID_COORDS | transponder_report_s::PX4_ADSB_FLAGS_VALID_HEADING |
		   transponder_report_s::PX4_ADSB_FLAGS_VALID_VELOCITY |
		   transponder_report_s::PX4_ADSB_FLAGS_VALID_ALTITUDE |
		   (transponder_report_s::ADSB_EMITTER_TYPE_UAV & emitter_type ? 0 :
		    transponder_report_s::PX4_ADSB_FLAGS_VALID_CALLSIGN); // Flags to indicate various statuses including valid data fields
	tr.squawk = 6667;

#ifndef BOARD_HAS_NO_UUID
	px4_guid_t px4_guid;
	board_get_px4_guid(px4_guid);
	memcpy(tr.uas_id, px4_guid, sizeof(px4_guid_t)); //simulate own GUID
#else

	for (int i = 0; i < PX4_GUID_BYTE_LENGTH ; i++) {
		tr.uas_id[i] = 0xe0 + i; //simulate GUID
	}

#endif /* BOARD_HAS_NO_UUID */

	uORB::Publication<transponder_report_s> tr_pub{ORB_ID(transponder_report)};
	tr_pub.publish(tr);
}

void EveNavigator::check_traffic()
{
	// double lat = get_global_position()->lat;
	// double lon = get_global_position()->lon;
	// float alt = get_global_position()->alt;

	// // TODO for non-multirotors predicting the future
	// // position as accurately as possible will become relevant
	// // float vel_n = get_global_position()->vel_n;
	// // float vel_e = get_global_position()->vel_e;
	// // float vel_d = get_global_position()->vel_d;

	// bool changed = _traffic_sub.updated();

	// char uas_id[11]; //GUID of incoming UTM messages

	// float NAVTrafficAvoidUnmanned = _param_nav_traff_a_radu.get();
	// float NAVTrafficAvoidManned = _param_nav_traff_a_radm.get();
	// float horizontal_separation = NAVTrafficAvoidManned;
	// float vertical_separation = NAVTrafficAvoidManned;

	// while (changed) {

	// 	//vehicle_status_s vs{};
	// 	transponder_report_s tr{};
	// 	_traffic_sub.copy(&tr);

	// 	uint16_t required_flags = transponder_report_s::PX4_ADSB_FLAGS_VALID_COORDS |
	// 				  transponder_report_s::PX4_ADSB_FLAGS_VALID_HEADING |
	// 				  transponder_report_s::PX4_ADSB_FLAGS_VALID_VELOCITY | transponder_report_s::PX4_ADSB_FLAGS_VALID_ALTITUDE;

	// 	if ((tr.flags & required_flags) != required_flags) {
	// 		changed = _traffic_sub.updated();
	// 		continue;
	// 	}

	// 	//convert UAS_id byte array to char array for User Warning
	// 	for (int i = 0; i < 5; i++) {
	// 		snprintf(&uas_id[i * 2], sizeof(uas_id) - i * 2, "%02x", tr.uas_id[PX4_GUID_BYTE_LENGTH - 5 + i]);
	// 	}

	// 	uint64_t uas_id_int = 0;

	// 	for (int i = 0; i < 8; i++) {
	// 		uas_id_int |= (uint64_t)(tr.uas_id[PX4_GUID_BYTE_LENGTH - i - 1]) << (i * 8);
	// 	}

	// 	//Manned/Unmanned Vehicle Seperation Distance
	// 	if (tr.emitter_type == transponder_report_s::ADSB_EMITTER_TYPE_UAV) {
	// 		horizontal_separation = NAVTrafficAvoidUnmanned;
	// 		vertical_separation = NAVTrafficAvoidUnmanned;
	// 	}

	// 	float d_hor, d_vert;
	// 	get_distance_to_point_global_wgs84(lat, lon, alt,
	// 					   tr.lat, tr.lon, tr.altitude, &d_hor, &d_vert);


	// 	// predict final altitude (positive is up) in prediction time frame
	// 	float end_alt = tr.altitude + (d_vert / tr.hor_velocity) * tr.ver_velocity;

	// 	// Predict until the vehicle would have passed this system at its current speed
	// 	float prediction_distance = d_hor + 1000.0f;

	// 	// If the altitude is not getting close to us, do not calculate
	// 	// the horizontal separation.
	// 	// Since commercial flights do most of the time keep flight levels
	// 	// check for the current and for the predicted flight level.
	// 	// we also make the implicit assumption that this system is on the lowest
	// 	// flight level close to ground in the
	// 	// (end_alt - horizontal_separation < alt) condition. If this system should
	// 	// ever be used in normal airspace this implementation would anyway be
	// 	// inappropriate as it should be replaced with a TCAS compliant solution.

	// 	if ((fabsf(alt - tr.altitude) < vertical_separation) || ((end_alt - horizontal_separation) < alt)) {

	// 		double end_lat, end_lon;
	// 		waypoint_from_heading_and_distance(tr.lat, tr.lon, tr.heading, prediction_distance, &end_lat, &end_lon);

	// 		struct crosstrack_error_s cr;

	// 		if (!get_distance_to_line(&cr, lat, lon, tr.lat, tr.lon, end_lat, end_lon)) {

	// 			if (!cr.past_end && (fabsf(cr.distance) < horizontal_separation)) {

	// 				bool action_needed = buffer_air_traffic(tr.icao_address);

	// 				if (action_needed) {
	// 					// direction of traffic in human-readable 0..360 degree in earth frame
	// 					int traffic_direction = math::degrees(tr.heading) + 180;
	// 					int traffic_seperation = (int)fabsf(cr.distance);

	// 					switch (_param_nav_traff_avoid.get()) {

	// 					case 0: {
	// 							/* Ignore */
	// 							PX4_WARN("TRAFFIC %s! dst %d, hdg %d",
	// 								 tr.flags & transponder_report_s::PX4_ADSB_FLAGS_VALID_CALLSIGN ? tr.callsign : uas_id,
	// 								 traffic_seperation,
	// 								 traffic_direction);
	// 							break;
	// 						}

	// 					case 1: {
	// 							/* Warn only */
	// 							mavlink_log_critical(&_mavlink_log_pub, "Warning TRAFFIC %s! dst %d, hdg %d\t",
	// 									     tr.flags & transponder_report_s::PX4_ADSB_FLAGS_VALID_CALLSIGN ? tr.callsign : uas_id,
	// 									     traffic_seperation,
	// 									     traffic_direction);
	// 							/* EVENT
	// 							 * @description
	// 							 * - ID: {1}
	// 							 * - Distance: {2m}
	// 							 * - Direction: {3} degrees
	// 							 */
	// 							events::send<uint64_t, int32_t, int16_t>(events::ID("eve_navigator_traffic"), events::Log::Critical, "Traffic alert",
	// 									uas_id_int, traffic_seperation, traffic_direction);
	// 							break;
	// 						}

	// 					case 2: {
	// 							/* RTL Mode */
	// 							mavlink_log_critical(&_mavlink_log_pub, "TRAFFIC: %s Returning home! dst %d, hdg %d\t",
	// 									     tr.flags & transponder_report_s::PX4_ADSB_FLAGS_VALID_CALLSIGN ? tr.callsign : uas_id,
	// 									     traffic_seperation,
	// 									     traffic_direction);
	// 							/* EVENT
	// 							 * @description
	// 							 * - ID: {1}
	// 							 * - Distance: {2m}
	// 							 * - Direction: {3} degrees
	// 							 */
	// 							events::send<uint64_t, int32_t, int16_t>(events::ID("eve_navigator_traffic_rtl"), events::Log::Critical,
	// 									"Traffic alert, returning home",
	// 									uas_id_int, traffic_seperation, traffic_direction);

	// 							// set the return altitude to minimum
	// 							// _rtl.set_return_alt_min(true);

	// 							// ask the commander to execute an RTL
	// 							vehicle_command_s vcmd = {};
	// 							vcmd.command = vehicle_command_s::VEHICLE_CMD_NAV_RETURN_TO_LAUNCH;
	// 							publish_vehicle_cmd(&vcmd);
	// 							break;
	// 						}

	// 					case 3: {
	// 							/* Land Mode */
	// 							mavlink_log_critical(&_mavlink_log_pub, "TRAFFIC: %s Landing! dst %d, hdg % d\t",
	// 									     tr.flags & transponder_report_s::PX4_ADSB_FLAGS_VALID_CALLSIGN ? tr.callsign : uas_id,
	// 									     traffic_seperation,
	// 									     traffic_direction);
	// 							/* EVENT
	// 							 * @description
	// 							 * - ID: {1}
	// 							 * - Distance: {2m}
	// 							 * - Direction: {3} degrees
	// 							 */
	// 							events::send<uint64_t, int32_t, int16_t>(events::ID("eve_navigator_traffic_land"), events::Log::Critical,
	// 									"Traffic alert, landing",
	// 									uas_id_int, traffic_seperation, traffic_direction);

	// 							// ask the commander to land
	// 							vehicle_command_s vcmd = {};
	// 							vcmd.command = vehicle_command_s::VEHICLE_CMD_NAV_LAND;
	// 							publish_vehicle_cmd(&vcmd);
	// 							break;

	// 						}

	// 					case 4: {
	// 							/* Position hold */
	// 							mavlink_log_critical(&_mavlink_log_pub, "TRAFFIC: %s Holding position! dst %d, hdg %d\t",
	// 									     tr.flags & transponder_report_s::PX4_ADSB_FLAGS_VALID_CALLSIGN ? tr.callsign : uas_id,
	// 									     traffic_seperation,
	// 									     traffic_direction);
	// 							/* EVENT
	// 							 * @description
	// 							 * - ID: {1}
	// 							 * - Distance: {2m}
	// 							 * - Direction: {3} degrees
	// 							 */
	// 							events::send<uint64_t, int32_t, int16_t>(events::ID("eve_navigator_traffic_hold"), events::Log::Critical,
	// 									"Traffic alert, holding position",
	// 									uas_id_int, traffic_seperation, traffic_direction);

	// 							// ask the commander to Loiter
	// 							vehicle_command_s vcmd = {};
	// 							vcmd.command = vehicle_command_s::VEHICLE_CMD_NAV_LOITER_UNLIM;
	// 							publish_vehicle_cmd(&vcmd);
	// 							break;

	// 						}
	// 					}
	// 				}
	// 			}
	// 		}
	// 	}

	// 	changed = _traffic_sub.updated();
	// }
}

bool EveNavigator::buffer_air_traffic(uint32_t icao_address)
{
	bool action_needed = true;

	if (_traffic_buffer.icao_address == icao_address) {

		if (hrt_elapsed_time(&_traffic_buffer.timestamp) > 60_s) {
			_traffic_buffer.timestamp = hrt_absolute_time();

		} else {
			action_needed = false;
		}

	} else {
		_traffic_buffer.timestamp = hrt_absolute_time();
		_traffic_buffer.icao_address = icao_address;
	}

	return action_needed;
}

bool EveNavigator::abort_landing()
{
	// only abort if currently landing and position controller status updated
	bool should_abort = false;

	if (_pos_sp_triplet.current.valid
	    && _pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_LAND) {

		if (_pos_ctrl_landing_status_sub.updated()) {
			position_controller_landing_status_s landing_status{};

			// landing status from position controller must be newer than EveNavigator's last position setpoint
			if (_pos_ctrl_landing_status_sub.copy(&landing_status)) {
				if (landing_status.timestamp > _pos_sp_triplet.timestamp) {
					should_abort = (landing_status.abort_status > 0);
				}
			}
		}
	}

	return should_abort;
}

int EveNavigator::custom_command(int argc, char *argv[])
{
	if (!is_running()) {
		print_usage("not running");
		return 1;
	}

	if (!strcmp(argv[0], "fencefile")) {
		// get_instance()->load_fence_from_file(GEOFENCE_FILENAME);
		return 0;

	} else if (!strcmp(argv[0], "fake_traffic")) {
		get_instance()->fake_traffic("LX007", 500, 1.0f, -1.0f, 100.0f, 90.0f, 0.001f,
					     transponder_report_s::ADSB_EMITTER_TYPE_LIGHT);
		get_instance()->fake_traffic("LX55", 1000, 0, 0, 100.0f, 90.0f, 0.001f, transponder_report_s::ADSB_EMITTER_TYPE_SMALL);
		get_instance()->fake_traffic("LX20", 15000, 1.0f, -1.0f, 280.0f, 90.0f, 0.001f,
					     transponder_report_s::ADSB_EMITTER_TYPE_LARGE);
		get_instance()->fake_traffic("UAV", 10, 1.0f, -2.0f, 10.0f, 10.0f, 0.01f, transponder_report_s::ADSB_EMITTER_TYPE_UAV);
		return 0;
	}

	return print_usage("unknown command");
}

void EveNavigator::set_mission_failure_heading_timeout()
{
	if (!_mission_result.failure) {
		_mission_result.failure = true;
		set_mission_result_updated();
		mavlink_log_critical(&_mavlink_log_pub, "unable to reach heading within timeout\t");
		events::send(events::ID("eve_navigator_mission_failure_heading"), events::Log::Critical,
			     "Mission failure: unable to reach heading within timeout");
	}
}

void EveNavigator::publish_vehicle_cmd(vehicle_command_s *vcmd)
{
	vcmd->timestamp = hrt_absolute_time();
	vcmd->source_system = _vstatus.system_id;
	vcmd->source_component = _vstatus.component_id;
	vcmd->target_system = _vstatus.system_id;
	vcmd->confirmation = false;
	vcmd->from_external = false;

	// The camera commands are not processed on the autopilot but will be
	// sent to the mavlink links to other components.
	switch (vcmd->command) {
	case NAV_CMD_IMAGE_START_CAPTURE:

		if (static_cast<int>(vcmd->param3) == 1) {
			// When sending a single capture we need to include the sequence number, thus camera_trigger needs to handle this cmd
			vcmd->param1 = 0.0f;
			vcmd->param2 = 0.0f;
			vcmd->param3 = 0.0f;
			vcmd->param4 = 0.0f;
			vcmd->param5 = 1.0;
			vcmd->param6 = 0.0;
			vcmd->param7 = 0.0f;
			vcmd->command = vehicle_command_s::VEHICLE_CMD_DO_DIGICAM_CONTROL;

		} else {
			// We are only capturing multiple if param3 is 0 or > 1.
			// For multiple pictures the sequence number does not need to be included, thus there is no need to go through camera_trigger
			_is_capturing_images = true;
		}

		vcmd->target_component = 100; // MAV_COMP_ID_CAMERA
		break;

	case NAV_CMD_IMAGE_STOP_CAPTURE:
		_is_capturing_images = false;
		vcmd->target_component = 100; // MAV_COMP_ID_CAMERA
		break;

	case NAV_CMD_VIDEO_START_CAPTURE:
	case NAV_CMD_VIDEO_STOP_CAPTURE:
		vcmd->target_component = 100; // MAV_COMP_ID_CAMERA
		break;

	default:
		vcmd->target_component = _vstatus.component_id;
		break;
	}

	_vehicle_cmd_pub.publish(*vcmd);
}

void EveNavigator::publish_vehicle_command_ack(const vehicle_command_s &cmd, uint8_t result)
{
	vehicle_command_ack_s command_ack = {};

	command_ack.timestamp = hrt_absolute_time();
	command_ack.command = cmd.command;
	command_ack.target_system = cmd.source_system;
	command_ack.target_component = cmd.source_component;
	command_ack.from_external = false;

	command_ack.result = result;
	command_ack.result_param1 = 0;
	command_ack.result_param2 = 0;

	_vehicle_cmd_ack_pub.publish(command_ack);
}

void EveNavigator::acquire_gimbal_control()
{
	vehicle_command_s vcmd = {};
	vcmd.command = vehicle_command_s::VEHICLE_CMD_DO_GIMBAL_MANAGER_CONFIGURE;
	vcmd.param1 = _vstatus.system_id;
	vcmd.param2 = _vstatus.component_id;
	vcmd.param3 = -1.0f; // Leave unchanged.
	vcmd.param4 = -1.0f; // Leave unchanged.
	publish_vehicle_cmd(&vcmd);
}

void EveNavigator::release_gimbal_control()
{
	vehicle_command_s vcmd = {};
	vcmd.command = vehicle_command_s::VEHICLE_CMD_DO_GIMBAL_MANAGER_CONFIGURE;
	vcmd.param1 = -3.0f; // Remove control if it had it.
	vcmd.param2 = -3.0f; // Remove control if it had it.
	vcmd.param3 = -1.0f; // Leave unchanged.
	vcmd.param4 = -1.0f; // Leave unchanged.
	publish_vehicle_cmd(&vcmd);
}


void
EveNavigator::stop_capturing_images()
{
	if (_is_capturing_images) {
		vehicle_command_s vcmd = {};
		vcmd.command = NAV_CMD_IMAGE_STOP_CAPTURE;
		vcmd.param1 = 0.0f;
		publish_vehicle_cmd(&vcmd);

		// _is_capturing_images is reset inside publish_vehicle_cmd.
	}
}

// bool EveNavigator::geofence_allows_position(const vehicle_global_position_s &pos)
// {
// 	if ((_geofence.getGeofenceAction() != geofence_result_s::GF_ACTION_NONE) &&
// 	    (_geofence.getGeofenceAction() != geofence_result_s::GF_ACTION_WARN)) {

// 		if (PX4_ISFINITE(pos.lat) && PX4_ISFINITE(pos.lon)) {
// 			return _geofence.check(pos, _gps_pos);
// 		}
// 	}

// 	return true;
// }

void EveNavigator::calculate_breaking_stop(double &lat, double &lon, float &yaw)
{
	// For multirotors we need to account for the braking distance, otherwise the vehicle will overshoot and go back
	float course_over_ground = atan2f(_local_pos.vy, _local_pos.vx);

	// predict braking distance

	const float velocity_hor_abs = sqrtf(_local_pos.vx * _local_pos.vx + _local_pos.vy * _local_pos.vy);

	float multirotor_braking_distance = math::trajectory::computeBrakingDistanceFromVelocity(velocity_hor_abs,
					    _param_mpc_jerk_auto, _param_mpc_acc_hor, 0.6f * _param_mpc_jerk_auto);

	waypoint_from_heading_and_distance(get_global_position()->lat, get_global_position()->lon, course_over_ground,
					   multirotor_braking_distance, &lat, &lon);
	yaw = get_local_position()->heading;
}

int EveNavigator::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Module that is responsible for autonomous flight modes. This includes missions (read from dataman),
takeoff and RTL.
It is also responsible for geofence violation checking.

### Implementation
The different internal modes are implemented as separate classes that inherit from a common base class `EveNavigatorMode`.
The member `_navigation_mode` contains the current active mode.

EveNavigator publishes position setpoint triplets (`position_setpoint_triplet_s`), which are then used by the position
controller.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("EveNavigator", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_COMMAND_DESCR("fencefile", "load a geofence file from SD card, stored at etc/geofence.txt");
	PRINT_MODULE_USAGE_COMMAND_DESCR("fake_traffic", "publishes 4 fake transponder_report_s uORB messages");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

/**
 * EveNavigator app start / stop handling function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int eve_navigator_main(int argc, char *argv[])
{
	return EveNavigator::main(argc, argv);
}
