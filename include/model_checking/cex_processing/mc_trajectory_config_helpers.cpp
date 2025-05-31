//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file

#include "model_checking/cex_processing/mc_trajectory_config_helpers.h"

#include <cmath>

namespace vfm {
namespace mc {
namespace trajectory_generator {
namespace helpers {

	void scale_param(FullTrajectory& editable_trajectory, PossibleParameter param, double scale_factor)
	{
		for (int i = 0; i < editable_trajectory.size(); i++)
		{
			editable_trajectory[i].second[param] *= scale_factor;
		}
	}

	void scale_duration(FullTrajectory& editable_trajectory, double duration_scale_factor)
	{
		for (int i = 0; i < editable_trajectory.size(); i++)
		{
			editable_trajectory[i].first *= duration_scale_factor;
		}
	}

	void add_to_param(FullTrajectory& editable_trajectory, PossibleParameter param, double value_to_add)
	{
		for (int i = 0; i < editable_trajectory.size(); i++)
		{
			editable_trajectory[i].second[param] += value_to_add;
		}
	}

	void extrapolate_param_by_derivation(FullTrajectory& editable_trajectory, double start_value, PossibleParameter param, PossibleParameter derivation_param)
	{
		// extrapolate a position parameter based on a velocity parameter
		double vehicle_pos = start_value;
		double last_timestamp = 0;

		for (int i = 0; i < editable_trajectory.size(); i++)
		{
			TrajectoryPosition& trajectory_pos = editable_trajectory[i];

			double step_time = trajectory_pos.first - last_timestamp;
			last_timestamp = trajectory_pos.first;

			// increase the position by the velocity of the current step
			vehicle_pos += step_time * trajectory_pos.second[derivation_param];
			trajectory_pos.second[param] = vehicle_pos;
		}
	}

	void extrapolate_trajectory_to_t0(FullTrajectory& editable_trajectory, PossibleParameter pos_param, PossibleParameter velocity_param)
	{
		// Extrapolate to timestamp t = 0 based on the velocities and prepend that to the trajectories.

		TrajectoryPosition trajectory_pos_for_t0 = editable_trajectory[0];
		trajectory_pos_for_t0.first = 0;
		trajectory_pos_for_t0.second[pos_param] += trajectory_pos_for_t0.second[velocity_param];
		editable_trajectory.insert(editable_trajectory.begin(), trajectory_pos_for_t0);
	}


	std::vector<LaneChange> detect_lane_changes_from_on_lane(FullTrajectory trajectory_positions)
	{
		std::vector<LaneChange> lane_changes{};

		double previous_on_lane_pos_y = 0;
		double previous_timestamp = 0;

		int total_states = trajectory_positions.size();
		for (int state_index = 0; state_index < total_states; state_index++)
		{
			TrajectoryPosition& trajectory_position = trajectory_positions[state_index];
			double timestamp = trajectory_position.first;
			ParameterMap& params = trajectory_position.second;

			double on_lane_pos_y = params[PossibleParameter::pos_y];

			if (state_index > 0 && on_lane_pos_y != previous_on_lane_pos_y)
			{
				LaneChange lc{};

				lc.start_index = state_index - 1;
				lc.end_index = state_index;
				lc.y_at_start = previous_on_lane_pos_y;
				lc.y_at_end = on_lane_pos_y;
				lc.duration = timestamp - previous_timestamp;

				lane_changes.push_back(lc);
			}

			previous_timestamp = timestamp;
			previous_on_lane_pos_y = on_lane_pos_y;
		}

		return lane_changes;
	}

	std::vector<LaneChange> detect_lane_changes_from_states(FullTrajectory trajectory_positions)
	{
		enum class LaneChangeState
		{
			None, Signalized, Started, Changing
		};

		std::vector<LaneChange> lane_changes{};
		LaneChange current_lane_change{};

		int state_index = 0;
		LaneChangeState lane_change_state = LaneChangeState::None;
		double timestamp = 0;
		double last_timestamp = 0;

		auto log = Failable::getSingleton("Lane Change Detection");

		int total_states = trajectory_positions.size();

		for (int state_index = 0; state_index < total_states; state_index++)
		{
			TrajectoryPosition& trajectory_position = trajectory_positions[state_index];

			std::string state_ind_str = std::to_string(state_index) + "/" + std::to_string(total_states);

			timestamp = trajectory_position.first;
			ParameterMap& params = trajectory_position.second;

			switch (lane_change_state)
			{
			case LaneChangeState::None:
				if (params[PossibleParameter::do_lane_change] == 1)
				{
					log->addNotePlain("Signalized a lanechange at state #: " + state_ind_str);
					current_lane_change.y_at_start = params[PossibleParameter::pos_y];
					lane_change_state = LaneChangeState::Signalized;
				}
				break;

			case LaneChangeState::Signalized:
				{
					bool change_lane_now = params[PossibleParameter::change_lane_now] == 1;

					if (params[PossibleParameter::do_lane_change] == 0 && !change_lane_now)
					{
						log->addNotePlain("Aborted a signalized lanechange at state #: " + state_ind_str);
						lane_change_state = LaneChangeState::None;
					}
					else
					{
						if (params[PossibleParameter::pos_y] != current_lane_change.y_at_start || change_lane_now) // lane actually started to change
						{
							log->addNotePlain("Started a signalized lanechange at state #: " + state_ind_str);
							lane_change_state = LaneChangeState::Started;
							current_lane_change.start_index = state_index -1; // previeous index as that's before y_at_start started changed
							current_lane_change.duration = last_timestamp; // store the begin-time in "duration"
						}
					}
				}
				break;

			case LaneChangeState::Started:
				if (params[PossibleParameter::do_lane_change] == 0)
				{
					log->addWarningPlain("Aborted a started lane change at state #: " + state_ind_str
						+ "\nNote that this will not be visualized on the trajectory!");
					lane_change_state = LaneChangeState::None;
				}
				else
				{
					// reached a new full lane or the end of the trace
					if (std::fmod(params[PossibleParameter::pos_y], LANE_WIDTH) < 0.01 || state_index == (total_states - 1))
					{
						current_lane_change.end_index = state_index;
						current_lane_change.y_at_end = params[PossibleParameter::pos_y];
						// set the actual duration of the lane change
						current_lane_change.duration = timestamp - current_lane_change.duration;

						log->addNotePlain("Finished a lanechange at state #: " + state_ind_str + " in " + std::to_string(state_index - current_lane_change.start_index) + " state steps and duration of " + std::to_string(current_lane_change.duration) + "s.");

						lane_changes.push_back(current_lane_change);
						lane_change_state = LaneChangeState::None;
					}
				}
				break;
			}

			last_timestamp = timestamp;
			state_index++;
		}

		return lane_changes;
	}

} // helpers
} // trajectory_generator
} // mc
} // vfm
