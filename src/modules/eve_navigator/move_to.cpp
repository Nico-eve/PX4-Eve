/****************************************************************************
 *
 *   Copyright (c) 2013-2014 PX4 Development Team. All rights reserved.
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
 * @file move_to.cpp
 *
 * Helper class to MoveTo
 *
 * @author Nicolas Brissonneau <nico@evevehicles.com>
 */

#include "move_to.h"
#include "eve_navigator.h"
#include <px4_platform_common/events.h>

MoveTo::MoveTo(EveNavigator *navigator) :
	MissionBlock(navigator)
{
}

void
MoveTo::on_activation()
{   
    safe_alt = false;
	set_moveto_position();
}

void
MoveTo::on_active()
{
    // struct position_setpoint_triplet_s *rep = _navigator->get_moveto_triplet();

    // if (rep->current.valid) {
        // reset the position
        set_moveto_position();

    // } else if (is__mission_item_reached_or_completed() && !_navigator->get_mission_result()->finished) {
    //     _navigator->get_mission_result()->finished = true;
    //     _navigator->set_mission_result_updated();

    //     position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();
    //     // set loiter item so position controllers stop doing takeoff logic
    //     // if (_navigator->get_land_detected()->landed) {
    //     //     _mission_item.nav_cmd = NAV_CMD_IDLE;

    //     // } else {
    //     //     if (pos_sp_triplet->current.valid) {
    //     //         setLoiterItemFromCurrentPositionSetpoint(&_mission_item);

    //     //     } else {
    //     //         setLoiterItemFromCurrentPosition(&_mission_item);
    //     //     }
    //     // }

    //     mission_apply_limitation(_mission_item);

    //     _mission_item_to_position_setpoint(_mission_item, &pos_sp_triplet->current);

    //     _navigator->set_position_setpoint_triplet_updated();
    // }
	// struct position_setpoint_triplet_s *rep = _navigator->get_moveto_triplet();
	// _mission_item_to_position_setpoint(_mission_item,rep->current);

	// if (PX4_ISFINITE(pos_sp_triplet->current.lat) && PX4_ISFINITE(pos_sp_triplet->current.lon)) {
	// 	_navigator->set_can_loiter_at_sp(true);

	// } else {
	// 	_navigator->set_can_loiter_at_sp(false);
	// }

	_navigator->set_position_setpoint_triplet_updated();
}

void
MoveTo::set_moveto_position()
{
    struct position_setpoint_triplet_s *rep = _navigator->get_moveto_triplet();

    // set current mission item to takeoff
    set_moveto_item(&_mission_item);

    setLoiterItemFromCurrentPositionWithBreaking(&_mission_item);
    _navigator->get_mission_result()->finished = false;
    _navigator->set_mission_result_updated();
    reset_mission_item_reached();

    // convert mission item to current setpoint
    struct position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();
    mission_apply_limitation(_mission_item);
    mission_item_to_position_setpoint(_mission_item, &pos_sp_triplet->current);

    pos_sp_triplet->previous.valid = false;
    pos_sp_triplet->current.yaw_valid = true;
    pos_sp_triplet->next.valid = false;

    // if (rep->current.valid) {
    //     PX4_INFO("Current valid");
    //     // Go on and check which changes had been requested
    //     if (PX4_ISFINITE(rep->current.yaw)) {
    //         pos_sp_triplet->current.yaw = rep->current.yaw;
    //     }

    //     // Set the current latitude and longitude even if they are NAN
    //     // NANs are handled in FlightTaskAuto.cpp
    //     pos_sp_triplet->current.lat = rep->current.lat;
    //     pos_sp_triplet->current.lon = rep->current.lon;
    //     pos_sp_triplet->current.alt = rep->current.alt;

    //     // mark this as done
    //     memset(rep, 0, sizeof(*rep));
    // }

    // if (PX4_ISFINITE(pos_sp_triplet->current.lat) && PX4_ISFINITE(pos_sp_triplet->current.lon)) {
    //     _navigator->set_can_loiter_at_sp(true);

    // } else {
    //     _navigator->set_can_loiter_at_sp(false);
    // }
    // PX4_INFO("Looping");
    // pos_sp_triplet->current.lat = rep->current.lat;
    // pos_sp_triplet->current.lon = rep->current.lon;
    // pos_sp_triplet->current.alt = rep->current.alt;
    vehicle_global_position_s* global_pos = _navigator->get_global_position();
    if (!safe_alt){
        if (global_pos->alt < rep->current.alt-min_thresh){
            pos_sp_triplet->current.lon = global_pos->lon;
            pos_sp_triplet->current.lat = global_pos->lat;
            pos_sp_triplet->current.alt = rep->current.alt;
        }
        else{
            safe_alt = true;
        }
    }
    else{
        pos_sp_triplet->current.lon = rep->current.lon;
        pos_sp_triplet->current.lat = rep->current.lat;
        pos_sp_triplet->current.alt = rep->current.alt;
        if (global_pos->alt < rep->current.alt+max_thresh){
            safe_alt = false;
        }
    }

    _navigator->set_position_setpoint_triplet_updated();
}