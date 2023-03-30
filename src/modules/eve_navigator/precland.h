/***************************************************************************
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
 * @file precland.h
 *
 * Helper class to do precision landing with a landing target
 *
 * @author Nicolas de Palezieux (Sunflower Labs) <ndepal@gmail.com>
 */

#pragma once

#include <matrix/math.hpp>
#include <lib/geo/geo.h>
#include <px4_platform_common/module_params.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/landing_target_pose.h>
// #include <uORB/topics/debug_vect.h>
#include <uORB/topics/server_mission_request.h>

#include <uORB/topics/irlock_report.h>
#include <uORB/topics/distance_sensor.h>

#include "navigator_mode.h"
#include "mission_block.h"
#include <parameters/param.h>

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>

enum class PrecLandState {
	GoTo,
	Start, // Starting state
	ReachAltitude,
	HorizontalApproach, // Positioning over landing target while maintaining altitude
	DescendAboveTarget, // Stay over landing target while descending
	FinalApproach, // Final landing approach, even without landing target
	Search, // Search for landing target
	Fallback, // Fallback landing method
	Done // Done landing
};

enum class PrecLandMode {
	Opportunistic = 1, // only do precision landing if landing target visible at the beginning
	Required = 2 // try to find landing target if not visible at the beginning
};

class PrecLand : public MissionBlock, public ModuleParams
{
public:
	PrecLand(EveNavigator *navigator);
	~PrecLand() override = default;

	void on_activation() override;
	void on_active() override;
	void on_inactivation() override;

	void set_target(position_setpoint_s global_target) {_global_target = global_target;};

	void set_mode(PrecLandMode mode) { _mode = mode; };

	PrecLandMode get_mode() { return _mode; };

	bool is_activated() { return _is_activated; };
	void reset_precland();
	void StartState();
private:

	void updateParams() override;
	// run the control loop for each state
	void run_go_to();
	void run_state_start();
	void run_state_reach_altitude();
	void run_state_horizontal_approach();
	void run_state_descend_above_target();
	void run_state_final_approach();
	void run_state_search();
	void run_state_fallback();

	float find_setpoint();
	float find_setpoint_v2(bool lost_target);
	// attempt to switch to a different state. Returns true if state change was successful, false otherwise
	bool switch_to_state_start();
	bool switch_to_state_reach_altitude();
	bool switch_to_state_horizontal_approach();
	bool switch_to_state_descend_above_target();
	bool switch_to_state_final_approach();
	bool switch_to_state_search();
	bool switch_to_state_fallback();
	bool switch_to_state_done();
	void update_beacon();
	void update_z_fit();
	void update_setpoint_type(position_setpoint_triplet_s *pos_sp_triplet);
	void print_state_switch_message(const char *state_name);

	// check if a given state could be changed into. Return true if possible to transition to state, false otherwise
	bool check_state_conditions(PrecLandState state);
	void slewrate(float &sp_x, float &sp_y);

	landing_target_pose_s _target_pose{}; /**< precision landing target position */
	uORB::Subscription _target_pose_sub{ORB_ID(landing_target_pose)};
	uORB::Subscription _server_mission_request_sub{ORB_ID(server_mission_request)};

	uORB::Subscription _irlock_report_sub{ORB_ID(irlock_report)};
	uORB::Subscription _distance_sensor_sub{ORB_ID(distance_sensor)};

	bool _target_pose_valid{false}; /**< whether we have received a landing target position message */
	bool _target_pose_updated{false}; /**< wether the landing target position message is updated */
	
	// Debug setup
	// struct debug_vect_s dbg;
	orb_advert_t pub_dbg;

	bool safe_alt;
	float max_height;
	float min_thresh = 0.2f;
	float max_thresh = 1.0f;
	position_setpoint_s _global_target;
	float old_delta_aim;
	bool full_beacon = false;
	bool nest_z_fit = false;
	enum DescentState{TargetCentered,TargetOffset,TargetLost};
	int old_state;
	bool reset_param = true;
	param_t land_trigger_param;
	float K_speed = 1.0f; 
	uint64_t old_time{0};
	uint64_t last_full_beacon{0};
	matrix::Vector2f old_xy;
	MapProjection _map_ref{}; /**< class for local/global projections */

	uint64_t _state_start_time{0}; /**< time when we entered current state */
	uint64_t _last_slewrate_time{0}; /**< time when we last limited setpoint changes */
	uint64_t _target_acquired_time{0}; /**< time when we first saw the landing target during search */
	uint64_t _point_reached_time{0}; /**< time when we reached a setpoint */
	uint64_t _last_update{0};
	
	int _search_cnt{0}; /**< counter of how many times we had to search for the landing target */
	float _approach_alt{0.0f}; /**< altitude at which to stay during horizontal approach */

	matrix::Vector2f _sp_pev;
	matrix::Vector2f _sp_pev_prev;
	float z_init;
	PrecLandState _state{PrecLandState::GoTo};

	PrecLandMode _mode{PrecLandMode::Opportunistic};
	float _target_altitude;
	bool _is_activated {false}; /**< indicates if precland is activated */

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::PLD_BTOUT>) _param_pld_btout,
		(ParamFloat<px4::params::PLD_HACC_RAD>) _param_pld_hacc_rad,
		(ParamFloat<px4::params::PLD_FAPPR_ALT>) _param_pld_fappr_alt,
		(ParamFloat<px4::params::PLD_SRCH_ALT>) _param_pld_srch_alt,
		(ParamFloat<px4::params::PLD_SRCH_TOUT>) _param_pld_srch_tout,
		(ParamInt<px4::params::PLD_MAX_SRCH>) _param_pld_max_srch
	)

	// non-EveNavigator parameters
	param_t	_handle_param_acceleration_hor{PARAM_INVALID};
	param_t	_handle_param_xy_vel_cruise{PARAM_INVALID};
	float	_param_acceleration_hor{0.0f};
	float	_param_xy_vel_cruise{0.0f};

};
