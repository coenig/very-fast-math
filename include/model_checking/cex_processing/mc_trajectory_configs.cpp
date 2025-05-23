//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file

#include "model_checking/cex_processing/mc_trajectory_generator.h"
#include "model_checking/cex_processing/mc_trajectory_configs.h"
#include "model_checking/cex_processing/mc_trajectory_config_helpers.h"

#include <cmath>

using namespace vfm::mc::trajectory_generator::helpers;

namespace vfm {
namespace mc {
namespace trajectory_generator {

	InterpretationConfiguration InterpretationConfiguration::getLaneChangeConfiguration()
	{
		// CONFIGURATION
		std::string ego_vehicle_name = "ego_vehicle";

		InterpretationCommands general_interpretations{
			{"timestamp", // The key provided here will be directly compared with the keys in the MCTrace
			std::make_shared<InterpretableKey<double>>(
				[](MCinterpretedTrace& traj, std::vector<std::string> key_elements, double timestamp) {

					traj.pushTimestamp(timestamp);

				})},


			// EGO
			{"ego.v",
			std::make_shared<InterpretableKey<double>>(
				[=](MCinterpretedTrace& traj, std::vector<std::string> key_elements, double velocity) {

					traj.pushVehicleParameter(ego_vehicle_name, velocity, PossibleParameter::vel_x);

				})},

			{"ego.on_lane",
			std::make_shared<InterpretableKey<double>>(
				[=](MCinterpretedTrace& traj, std::vector<std::string> key_elements, double lane) {

					double lane_y = lane * LANE_WIDTH_HALF;
					traj.pushVehicleParameter(ego_vehicle_name, lane_y, PossibleParameter::pos_y);

				})},

			{R"(planner."flCond.cond18_driver_lanechange_request")",
			std::make_shared<InterpretableKey<bool>>(
				[=](MCinterpretedTrace& traj, std::vector<std::string> key_elements, bool lc) {

					// Currently only supporting left LC (due to flCond -> fast lane)
					traj.pushVehicleParameter(ego_vehicle_name, lc, PossibleParameter::turn_signal_left);

				})},

			{"ego.gaps___609___.i_agent_front",
			std::make_shared<InterpretableKey<std::string>>(
				[=](MCinterpretedTrace& traj, std::vector<std::string> key_elements, std::string car_id) {
					traj.pushVehicleParameter(ego_vehicle_name, std::stoi(car_id), PossibleParameter::gap_0_i_agent_front);
				})},

			{"ego.gaps___619___.i_agent_front",
			std::make_shared<InterpretableKey<std::string>>(
				[=](MCinterpretedTrace& traj, std::vector<std::string> key_elements, std::string car_id) {
					traj.pushVehicleParameter(ego_vehicle_name, std::stoi(car_id), PossibleParameter::gap_1_i_agent_front);
				})},

			{"ego.gaps___629___.i_agent_front",
			std::make_shared<InterpretableKey<std::string>>(
				[=](MCinterpretedTrace& traj, std::vector<std::string> key_elements, std::string car_id) {
					traj.pushVehicleParameter(ego_vehicle_name, std::stoi(car_id), PossibleParameter::gap_2_i_agent_front);
				})},

			{"ego.gaps___609___.i_agent_rear",
			std::make_shared<InterpretableKey<std::string>>(
				[=](MCinterpretedTrace& traj, std::vector<std::string> key_elements, std::string car_id) {
					traj.pushVehicleParameter(ego_vehicle_name, std::stoi(car_id), PossibleParameter::gap_0_i_agent_rear);
				})},

			{"ego.gaps___619___.i_agent_rear",
			std::make_shared<InterpretableKey<std::string>>(
				[=](MCinterpretedTrace& traj, std::vector<std::string> key_elements, std::string car_id) {
					traj.pushVehicleParameter(ego_vehicle_name, std::stoi(car_id), PossibleParameter::gap_1_i_agent_rear);
				})},

			{"ego.gaps___629___.i_agent_rear",
			std::make_shared<InterpretableKey<std::string>>(
				[=](MCinterpretedTrace& traj, std::vector<std::string> key_elements, std::string car_id) {
					traj.pushVehicleParameter(ego_vehicle_name, std::stoi(car_id), PossibleParameter::gap_2_i_agent_rear);
				})},

			{"ego.abs_pos",
			std::make_shared<InterpretableKey<double>>(
				[=](MCinterpretedTrace& traj, std::vector<std::string> key_elements, double ego_abs_pos) {

					// Set the x position of ego (interpolation will happen in postprocessing)
					traj.pushVehicleParameter(ego_vehicle_name, ego_abs_pos, PossibleParameter::pos_x);

				})},

			{"Array.CurrentState",
			std::make_shared<InterpretableKey<std::string>>(
				[=](MCinterpretedTrace& traj, std::vector<std::string> key_elements, std::string state_str) {

					double state_value = -1;
					if (state_str == "state_EnvironmentModel") // The state names should be stable now (Lukas).
						state_value = STATE_ENVIRONMENT_MODEL;
					if (state_str == "state_TacticalPlanner")
						state_value = STATE_TACTICAL_PLANNER;
					if (state_str == "state_Invalid")
						state_value = STATE_INVALID;
					if (state_value == -1)
						throw std::invalid_argument("Unknown state: " + state_str);

					traj.pushVehicleParameter(ego_vehicle_name, state_value, PossibleParameter::special_current_state);

				})},
		};


		InterpretationCommands vehicle_interpretations{
			{"v", // The key here will be prefixed with possible vehicle names before compared in the MCTrace
			std::make_shared<InterpretableKey<double>>(
				[](MCinterpretedTrace& traj, std::vector<std::string> key_elements, double velocity) {

					auto vehicle_name = key_elements[0];
					traj.pushVehicleParameter(vehicle_name, velocity, PossibleParameter::vel_x);

				})},

			{"a",
			std::make_shared<InterpretableKey<double>>(
				[](MCinterpretedTrace& traj, std::vector<std::string> key_elements, double accel) {

					auto vehicle_name = key_elements[0];
					traj.pushVehicleParameter(vehicle_name, accel, PossibleParameter::acc_x);

				})},

			{"rel_pos",
			std::make_shared<InterpretableKey<double>>(
				[=](MCinterpretedTrace& traj, std::vector<std::string> key_elements, double x_relative_to_ego) {

					// Push the position value relative to the ego. In the post-processing step, we turn this into absolute values once we have calculated the ego position.
					auto vehicle_name = key_elements[0];
					traj.pushVehicleParameter(vehicle_name, x_relative_to_ego, PossibleParameter::pos_x);

				})},

			{"on_lane",
			std::make_shared<InterpretableKey<double>>(
				[=](MCinterpretedTrace& traj, std::vector<std::string> key_elements, double lane_index) {

					// Set the y position according to the lane (interpolation will happen in postprocessing)
					auto vehicle_name = key_elements[0];
					traj.pushVehicleParameter(vehicle_name, lane_index * LANE_WIDTH_HALF, PossibleParameter::pos_y);

				})},

			{"do_lane_change",
			std::make_shared<InterpretableKey<bool>>(
				[=](MCinterpretedTrace& traj, std::vector<std::string> key_elements, bool do_lane_change) {

					// Whether currently doing a lanechange
					auto vehicle_name = key_elements[0];
					traj.pushVehicleParameter(vehicle_name, do_lane_change ? 1 : 0, PossibleParameter::do_lane_change);

				})},

			{"change_lane_now",
			std::make_shared<InterpretableKey<bool>>(
				[=](MCinterpretedTrace& traj, std::vector<std::string> key_elements, bool change_lane_now) {

					// Whether currently doing a lanechange
					auto vehicle_name = key_elements[0];
					traj.pushVehicleParameter(vehicle_name, change_lane_now ? 1 : 0, PossibleParameter::change_lane_now);

				})},

			{"turn_signals",
			std::make_shared<InterpretableKey<std::string>>(
				[=](MCinterpretedTrace& traj, std::vector<std::string> key_elements, std::string turn_signal) {

					auto vehicle_name = key_elements[0];

					if (StaticHelper::stringContains(turn_signal, "CENTER") || turn_signal == "0") // TODO: @Alex, please check if there's a better solution - the numbers come from plain nuxmv runs, will go away eventually.
					{
						traj.pushVehicleParameter(vehicle_name, 0.0, PossibleParameter::turn_signal_left);
						traj.pushVehicleParameter(vehicle_name, 0.0, PossibleParameter::turn_signal_right);
						return;
					}
					if (StaticHelper::stringContains(turn_signal, "LEFT") || turn_signal == "1") // TODO: remove hardcoded stringContains to capture ActionDir#LEFT etc.
					{
						traj.pushVehicleParameter(vehicle_name, 1.0, PossibleParameter::turn_signal_left);
						return;
					}
					if (StaticHelper::stringContains(turn_signal, "RIGHT") || turn_signal == "2")
					{
						traj.pushVehicleParameter(vehicle_name, 1.0, PossibleParameter::turn_signal_right);
						return;
					}

					throw std::invalid_argument("Unknown turn signal: " + turn_signal);
				})},

		};

		// TODO: Maybe replace this with a regex search
		std::vector<std::string> possible_vehicle_keys{
			"veh___609___", "veh___619___", "veh___629___", "veh___639___", "veh___649___",
			"veh___659___", "veh___669___", "veh___679___", "veh___689___", "veh___699___",
			"veh___6109___", "veh___6119___", "veh___6129___", "veh___6139___", "veh___6149___",
         "veh___6159___", "veh___6169___", "veh___6179___", "veh___6189___", "veh___6199___",
			"veh___6209___", "veh___6219___", "veh___6229___", "veh___6239___", "veh___6249___",
			"veh___6259___", "veh___6269___", "veh___6279___", "veh___6289___", "veh___6299___" };


		auto preprocessing = [ego_vehicle_name](MCinterpretedTrace& traj) {
			traj.noteVehicle(ego_vehicle_name);
		};

		auto lane_width = LANE_WIDTH;
		auto step_time = 1.0; // default step time

		auto postprocessing = [ego_vehicle_name, lane_width, step_time](MCinterpretedTrace& traj) {

			// Make the ego move along its x based on its velocity (because in the trace, the ego is always at x == 0)
			//extrapolate_param_by_derivation(traj.getEditableVehicleTrajectory(ego_vehicle_name), 0, PossibleParameter::pos_x, PossibleParameter::vel_x);

			auto smoothBlend = [](double t, double param) {
				return (t * t) / (param * ((t * t) - t) + 1.0f);
			};

			for (const auto& vehicle_name : traj.getVehicleNames())
			{
				if (vehicle_name != ego_vehicle_name) // exclude the ego
				{
					// Set the other vehicle x position relative to the ego
					transform_relative_to_absolute_trajectory(traj, vehicle_name, ego_vehicle_name, PossibleParameter::pos_x);
				}

				// Detect and interpolate lane changes (movement along y)

				auto& trajectory = traj.getEditableVehicleTrajectory(vehicle_name);
				auto lane_changes = detect_lane_changes_from_on_lane(trajectory);

				for (const LaneChange& lane_change : lane_changes)
				{
					int step_index = 0;
					double start_time = -1;

					for (TrajectoryPosition& trajectory_position : trajectory)
					{
						double timestamp = trajectory_position.first;
						if (step_index == lane_change.start_index)
							start_time = timestamp;

						if (step_index >= lane_change.start_index && step_index <= lane_change.end_index)
						{
							double ratio = ((timestamp - start_time) / lane_change.duration);
							trajectory_position.second[PossibleParameter::pos_y] = lane_change.y_at_start + smoothBlend(ratio, 2.0) * (lane_change.y_at_end - lane_change.y_at_start);
						}
						step_index++;
					}
				}
			}

			// Repeat the last frame to ensure that when an OSC simulation is used, the trajectory actually reaches the end
			for (const auto& vehicle_name : traj.getVehicleNames())
			{
				std::vector<TrajectoryPosition>& trajectory = traj.getEditableVehicleTrajectory(vehicle_name);

				TrajectoryPosition additional_last = trajectory.back();
				additional_last.first += step_time;
				trajectory.push_back(additional_last);
			}

			traj.appendDummyDataAtTheEndOfTrace();
		};



		auto required_parameters = getStandardRequiredParameters();

		// add parameters specific to lane change traces

		required_parameters.push_back(ParameterDetails{
			PossibleParameter::special_current_state, "current_state", false, ParameterDetails::linearInterpolation,
			[](double last_value) {return -1;} });

		required_parameters.push_back(ParameterDetails{ PossibleParameter::do_lane_change, "do_lane_change", false });
		required_parameters.push_back(ParameterDetails{ PossibleParameter::change_lane_now, "change_lane_now", false });

		auto no_interpolation = [](double current_value, double next_value, double factor) {return current_value;};

		required_parameters.push_back(ParameterDetails{
			PossibleParameter::turn_signal_left, "turn_signal_left", false, no_interpolation, [](double last_value) {
				return 0;
			}
			});

		required_parameters.push_back(ParameterDetails{
			PossibleParameter::turn_signal_right, "turn_signal_right", false, no_interpolation, [](double last_value) {
				return 0;
			}
			});

		required_parameters.push_back(ParameterDetails{
			PossibleParameter::gap_0_i_agent_front, "ego.gaps___609___.i_agent_front", false, no_interpolation, [](double last_value) {
				return std::isnan(last_value) ? -1 : last_value;
			}
			});

		required_parameters.push_back(ParameterDetails{
			PossibleParameter::gap_1_i_agent_front, "ego.gaps___619___.i_agent_front", false, no_interpolation, [](double last_value) {
				return std::isnan(last_value) ? -1 : last_value;
			}
			});

		required_parameters.push_back(ParameterDetails{
			PossibleParameter::gap_2_i_agent_front, "ego.gaps___629___.i_agent_front", false, no_interpolation, [](double last_value) {
				return std::isnan(last_value) ? -1 : last_value;
			}
			});

		required_parameters.push_back(ParameterDetails{
			PossibleParameter::gap_0_i_agent_rear, "ego.gaps___609___.i_agent_rear", false, no_interpolation, [](double last_value) {
				return std::isnan(last_value) ? -1 : last_value;
			}
			});

		required_parameters.push_back(ParameterDetails{
			PossibleParameter::gap_1_i_agent_rear, "ego.gaps___619___.i_agent_rear", false, no_interpolation, [](double last_value) {
				return std::isnan(last_value) ? -1 : last_value;
			}
			});

		required_parameters.push_back(ParameterDetails{
			PossibleParameter::gap_2_i_agent_rear, "ego.gaps___629___.i_agent_rear", false, no_interpolation, [](double last_value) {
				return std::isnan(last_value) ? -1 : last_value;
			}
			});



		// State-end condition

		auto check_state_end = [](const std::vector<ParameterMap>& params_until_now)
		{
			const size_t param_count = params_until_now.size();
			if (param_count < 2)
				return false;

			const ParameterMap current_parameters = params_until_now[param_count - 1];

			auto has_a_current_state = [](const ParameterMap& param_map)
			{
				return (param_map.find(PossibleParameter::special_current_state) != param_map.end());
			};

			// State end only occurs when the current parameter map contains a state change
			if (!has_a_current_state(current_parameters))
				return false;

			for (int i = param_count - 2; i >= 0; i--)
			{
				const ParameterMap& previous_parameters = params_until_now[i];

				// Found the last parameter map which had a state change
				if (has_a_current_state(previous_parameters))
				{
					// value in last state was state_0 (invalid state) and value in new state is state_1 -> this is the start of the very first "real" state
					if (current_parameters.at(PossibleParameter::special_current_state) == STATE_ENVIRONMENT_MODEL)
						if (previous_parameters.at(PossibleParameter::special_current_state) == STATE_INVALID)
							return true;

					// value in last state was state_2 and value in new state is state_1 -> this is the start of a new "real" state
					if (current_parameters.at(PossibleParameter::special_current_state) == STATE_TACTICAL_PLANNER)
						if (previous_parameters.at(PossibleParameter::special_current_state) == STATE_ENVIRONMENT_MODEL)
							return true;

					break;
				}
			}

			return false;
		};

		return InterpretationConfiguration(
			ego_vehicle_name,
			general_interpretations,
			vehicle_interpretations,
			possible_vehicle_keys,
			required_parameters,
			check_state_end,
			preprocessing,
			postprocessing,
			step_time,
			'.');
	}


	std::vector<ParameterDetails> InterpretationConfiguration::getStandardRequiredParameters()
	{
		std::vector<ParameterDetails> tracked_parameters{
			// position
			{PossibleParameter::pos_x, "x", true},

			{PossibleParameter::pos_y, "y", true},
			{PossibleParameter::pos_z, "z", true},

			// velocity
			{PossibleParameter::vel_x, "vx", true},
			{PossibleParameter::vel_y, "vy", true},
			{PossibleParameter::vel_z, "vz", true},

			// acceleration
			{PossibleParameter::acc_x, "dvx", true},
			{PossibleParameter::acc_y, "dvy", true},
			{PossibleParameter::acc_z, "dvz", true},

			// others
			{PossibleParameter::heading, "head", true},
			{PossibleParameter::yawrate, "yaw", true}
		};

		return tracked_parameters;
	}



} // trajectory_generator
} // mc
} // vfm
