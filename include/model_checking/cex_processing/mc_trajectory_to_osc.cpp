//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file

#include "model_checking/cex_processing/mc_trajectory_visualizers.h"
#include "simulation/env2d_simple.h"
#include "geometry/gif_writer.h"

#include <sstream>
#include <iomanip>
#include <thread>

using namespace vfm;
using namespace mc;
using namespace trajectory_generator;


std::string OSCgenerator::generate(std::string scenario_name, bool control_ego, std::string prerequisites)
{
	std::replace(scenario_name.begin(), scenario_name.end(), '.', '_'); // replace all '.' to '_'
	std::replace(scenario_name.begin(), scenario_name.end(), '-', '_'); // replace all '-' to '_'

	std::stringstream osc_text{};

	osc_text << R"(
# AUTO GENERATED OSC: )" << scenario_name << R"( - Source: M²oRTy

# Prerequisites
# Import map and helper nodes/subscenarios
import "StraightThreeLaneHighwayRQ36.osm"
import "../../../vehicle/system_cuts.osc"
import "../../scenario_blocks/cufu/aca_hands_on.osc"
import "../../scenario_blocks/map_agnostic/scenario_blocks.osc"

# Main Scenario
scenario )" << scenario_name << R"(:

  # Vehicles
  ego: ego_vehicle_pace  # replace with specific vehicle type if necessary
  )" << getVechicleDefinitions("  ") << R"(

  # Trajectories
  )"<<getTrajectoryDefinitions("  ", control_ego)<<R"(

  keep(nsdm_simulation_time() < )" << (m_interpreted_trace.getDuration() + 4.0) << R"()
  do serial():
    # INIT NODES
    parallel():
      )" << getPositionAndSpeedNodes("      ") << R"(

    # MANEUVER NODES
    parallel():
      run_customer_function_with_vmm_mock(ego)
      )"<<getTrajectoryReplayNodes("      ", control_ego)<<R"(

      serial():
        parallel(): # Engage hands on mode
          keep_speed_and_lane_until_stack_engaged(ego)
          activate_hands_on(ego)

        parallel():
          # Ego Maneuvers (rough attempt at reproducing the modelchecker output for ego)
          )"<<getEgoManeuvers("          ", control_ego)<<R"(
          ego.drive() with:
            keep(ego.is_fully_controlled_by_sut())
)";

	return osc_text.str();
}

std::string OSCgenerator::getVechicleDefinitions(std::string indent_prefix) const
{
	std::stringstream osc_ss{};
	osc_ss << std::endl;
	for (auto& vehicle_name : m_interpreted_trace.getVehicleNames(true))
	{
		osc_ss << indent_prefix << vehicle_name << ": vehicle" << std::endl;
	}
   return osc_ss.str();
}

std::string OSCgenerator::getPositionAndSpeedNodes(std::string indent_prefix) const
{
	std::stringstream osc_ss{};

	osc_ss << getPositionAndSpeedNode(indent_prefix, "ego", m_interpreted_trace.getEgoTrajectory());

	for (auto& vehicle_name : m_interpreted_trace.getVehicleNames(true))
	{
		osc_ss << getPositionAndSpeedNode(indent_prefix, vehicle_name, m_interpreted_trace.getVehicleTrajectory(vehicle_name));
	}

	return osc_ss.str();
}

std::string OSCgenerator::getPositionAndSpeedNode(std::string indent_prefix, std::string vehicle_name, const FullTrajectory& trajectory) const
{
	std::stringstream osc_ss{};
	osc_ss << std::endl;

	const ParameterMap& start_parameters = trajectory.front().second;

	osc_ss << indent_prefix << vehicle_name << ".assign_position(position_3d(";

	osc_ss << "x: " << std::to_string(start_parameters.at(PossibleParameter::pos_x));
	osc_ss << ", y: " << std::to_string(start_parameters.at(PossibleParameter::pos_y));
	osc_ss << ", z: " << std::to_string(start_parameters.at(PossibleParameter::pos_z));
	osc_ss << "))" << std::endl;

   osc_ss << indent_prefix << vehicle_name << ".assign_speed(" << std::to_string(start_parameters.at(PossibleParameter::vel_x)) << ")" << std::endl;

	return osc_ss.str();
}

std::string OSCgenerator::getTrajectoryDefinitions(std::string indent_prefix, bool control_ego) const
{
	std::stringstream osc_ss{};
	osc_ss << std::endl;

	if (control_ego)
	{
		osc_ss << getTrajectoryDefinition(indent_prefix, m_interpreted_trace.getEgoTrajectory(), "ego", true) << std::endl;
	}

	for (auto& vehicle_name : m_interpreted_trace.getVehicleNames(true))
	{
		osc_ss << getTrajectoryDefinition(indent_prefix, m_interpreted_trace.getVehicleTrajectory(vehicle_name), vehicle_name, true) << std::endl;
	}

   return osc_ss.str();
}

std::string OSCgenerator::getTrajectoryDefinition(std::string indent_prefix, FullTrajectory trajectory, std::string vehicle_name, bool wrap_in_osc_node) const
{
	const int decimals = 6;

	std::stringstream osc_ss{};
	osc_ss << indent_prefix << vehicle_name << "_trajectory : trajectory = trajectory(\\" << std::endl;

	// // // All timestamps in a single line
	osc_ss << indent_prefix << "time_stamps : [";
	for (const auto& trajectory_position : trajectory)
	{
		osc_ss << std::to_string(trajectory_position.first).substr(0, decimals) << ", ";
	}
	// move the write head two steps back to replace the last ", "
	osc_ss.seekp(-2, osc_ss.cur);
	osc_ss << "],\\" << std::endl;

	// // // Points
	osc_ss << indent_prefix << "points: [\\" << std::endl;

	auto append_param = [&osc_ss, decimals](const ParameterMap& params, PossibleParameter param_type) {
		osc_ss << std::setw(decimals + 1) << std::setfill(' ') << std::to_string(params.at(param_type)).substr(0, decimals);
		};

	for (const auto& trajectory_position : trajectory)
	{
		const ParameterMap& parameters = trajectory_position.second;
		const std::string timestamp_str = std::to_string(trajectory_position.first).substr(0, decimals);

		osc_ss << indent_prefix << "pose_3d(position: position_3d(x:";
		append_param(parameters, PossibleParameter::pos_x);
		osc_ss << ", y:";
		append_param(parameters, PossibleParameter::pos_y);
		osc_ss << ", z:";
		append_param(parameters, PossibleParameter::pos_z);
		osc_ss << "), orientation: orientation_3d(yaw:";
		append_param(parameters, PossibleParameter::yawrate);
		osc_ss << ", pitch: 0.0";
		osc_ss << ", roll: 0.0";
		if (&trajectory_position != &trajectory.back())
			osc_ss << ")),\\ ";
		else
			osc_ss << "))],\\";
		osc_ss << " # T: " << timestamp_str << std::endl;
	}

	// // // Velocities
   osc_ss << indent_prefix << "nsdm_translational_velocities: [\\" << std::endl;

	for (const auto& trajectory_position : trajectory)
	{
		const ParameterMap& parameters = trajectory_position.second;
		const std::string timestamp_str = std::to_string(trajectory_position.first).substr(0, decimals);

		osc_ss << indent_prefix << "translational_velocity_3d(x:";
		append_param(parameters, PossibleParameter::vel_x);
		osc_ss << ", y:";
		append_param(parameters, PossibleParameter::vel_y);
		osc_ss << ", z:";
		append_param(parameters, PossibleParameter::vel_z);
		if (&trajectory_position != &trajectory.back())
			osc_ss << "),\\";
		else
			osc_ss << ")])";
		osc_ss << " # T: " << timestamp_str << std::endl;
	}

	return osc_ss.str();
}

std::string OSCgenerator::getTrajectoryReplayNodes(std::string indent_prefix, bool control_ego) const
{
	std::stringstream osc_ss{};
	osc_ss << std::endl;

	if (control_ego)
	{
		osc_ss << indent_prefix << "ego.replay_trajectory(absolute: ego_trajectory)" << std::endl;
	}

	for (auto& vehicle_name : m_interpreted_trace.getVehicleNames(true))
	{
		osc_ss << indent_prefix << vehicle_name << ".replay_trajectory(absolute: " << vehicle_name << "_trajectory)" << std::endl;
	}

	return osc_ss.str();
}

std::string OSCgenerator::getEgoManeuvers(std::string indent_prefix, bool control_ego) const
{
	std::stringstream osc_ss{};
	osc_ss << std::endl;

	// Increase speed via the driver speed_control
	auto append_acc_maneuver = [&osc_ss, &indent_prefix](double t, int ten_kmh_steps, std::string inc_or_dec) {
		osc_ss << indent_prefix << "ego.set_driver_input_set_speed_control_" << inc_or_dec << "(value: true, delay_in_ms : " << (t * 1000) << ", duration_in_ms : 50)" << std::endl;
		if (ten_kmh_steps > 1)
		{
			osc_ss << indent_prefix << "nsdm_repeat(" << (ten_kmh_steps - 1) << "):" << std::endl;
			osc_ss << indent_prefix << "  ego.set_driver_input_set_speed_control_" << inc_or_dec << "(value: true, delay_in_ms: 20, duration_in_ms: 50)" << std::endl;
		}
	};

	// Trigger a Lanechange
	auto append_lc_maneuver = [&osc_ss, &indent_prefix](double t, std::string dir_name) {
		osc_ss << indent_prefix << "ego.set_driver_input_turn_indicator_lever(value: turn_indicator_lever_state!tipped_"
			<< dir_name << ", delay_in_ms : " << (t * 1000) << ", duration_in_ms : 200)" << std::endl;
		};

	double epsilon = 0.0001f;
	double last_indicator_left = 0;
	double last_indicator_right = 0;
	double current_target_velocity = m_interpreted_trace.getEgoTrajectory().front().second.at(PossibleParameter::vel_x);
	double last_timestamp = 0;

	for (auto& trajectory_point : m_interpreted_trace.getEgoTrajectory())
	{
		// Control velocity
		double velocity = trajectory_point.second.at(PossibleParameter::vel_x);
		int speed_change_ammount = 0;

		while (velocity > (current_target_velocity + epsilon))
		{
			speed_change_ammount++;
			current_target_velocity += 2.77778; // 10 km/h
		}
		while (velocity < (current_target_velocity - epsilon))
		{
			speed_change_ammount--;
			current_target_velocity -= 2.77778; // 10 km/h
		}
		if (speed_change_ammount > 0)
			append_acc_maneuver(last_timestamp, speed_change_ammount, "increase");
		if (speed_change_ammount < 0)
			append_acc_maneuver(last_timestamp, -speed_change_ammount, "decrease");

      last_timestamp = trajectory_point.first;

		// Control lane changes
		double indicator_left = trajectory_point.second.at(PossibleParameter::turn_signal_left);
		double indicator_right = trajectory_point.second.at(PossibleParameter::turn_signal_right);

		if ((indicator_left - last_indicator_left) > epsilon && last_indicator_left < epsilon)
		{
			append_lc_maneuver(trajectory_point.first, "left");
		}
		if ((indicator_right - last_indicator_right) > epsilon && last_indicator_right < epsilon)
		{
			append_lc_maneuver(trajectory_point.first, "right");
		}

		last_indicator_left = indicator_left;
		last_indicator_right = indicator_right;
	}

	return osc_ss.str();
}

void OSCgenerator::addVehicleTrajectoryFollower(std::stringstream& osc_ss, FullTrajectory trajectory, std::string vehicle_name, bool wrap_in_osc_node)
{
	// Only used for the CSV Output now.

	// param headers
	const int decimals = 5;
	std::set<PossibleParameter> ignored_parameters;

	osc_ss << getIndent() << "# " << std::setw(decimals + 1) << std::setfill(' ') << "time" << "  ";

	for (ParameterDetails required_param : m_interpreted_trace.getRequiredParameters())
	{
		if (required_param.needed_for_osc_trajectory)
		{
			osc_ss << std::setw(decimals + 1) << std::setfill(' ') << required_param.name << "  ";
		}
		else
		{
			// skip empty names because those are not needed in the trajectory
			ignored_parameters.insert(required_param.identifier);
		}
	}
	osc_ss << "\n";


	if (wrap_in_osc_node)
	{
		// follower node
		addIndented(osc_ss, vehicle_name + ".nsdm_replay_csv_trajectory(\"");
	}

	std::stringstream trajectory_ss;

	for (const auto& trajectory_position : trajectory)
	{
		double timestamp = trajectory_position.first;
		const ParameterMap parameters = trajectory_position.second;

		std::string timestamp_str = std::to_string(timestamp).substr(0, decimals);
		trajectory_ss << getIndent() << "  " << std::setw(decimals + 1) << std::setfill(' ') << timestamp_str << ", ";

		// append the value
		for (const auto& param : parameters)
		{
			if (ignored_parameters.find(param.first) == ignored_parameters.end())
			{
				std::string param_str = std::to_string(param.second).substr(0, decimals);
				trajectory_ss << std::setw(decimals + 1) << std::setfill(' ') << param_str << ", ";
			}
		}

		// move the write head two steps back to replace the last ", " occurence with the new-line
		trajectory_ss.seekp(-2, trajectory_ss.cur);
		trajectory_ss << " \n";
	}

	std::string trajectory_str = trajectory_ss.str();

	// remove excess newline
	trajectory_str.pop_back();
	trajectory_str.pop_back();

	osc_ss << trajectory_str << (wrap_in_osc_node ? "\")\n" : "\n");
}

// Helper methods

std::string OSCgenerator::getIndent(int additional_indentation)
{
	std::string str;
	for (int i = 0; i < m_current_indentations + additional_indentation; i++)
	{
		str += "  ";
	}
	return str;
}

void OSCgenerator::addIndented(std::stringstream& string_stream, std::string text, int additional_indentation)
{
	string_stream << getIndent(additional_indentation) << text << std::endl;
}

std::string OSCgenerator::generate_as_csv()
{
	double max_x = 0;
	double max_y = 0;
	for (auto& vehicle_name : m_interpreted_trace.getVehicleNames())
	{
		auto& trajectory = m_interpreted_trace.getVehicleTrajectory(vehicle_name);
		for (const auto& trajectory_position : trajectory)
		{
			double x = trajectory_position.second.at(PossibleParameter::pos_x);
			double y = trajectory_position.second.at(PossibleParameter::pos_y);
			if (x > max_x) {
				max_x = x;
			}
			if (y > max_y) {
				max_y = y;
			}
		}
	}

	std::stringstream osc_ss;
	double lanes = round(max_y / 3.5) + 1;
	osc_ss << "Track\n"
		"Longitude, Offset, Lanes\n"
		"0.00, 0.00, " + std::to_string(lanes) + "\n"
		"" + std::to_string(max_x * 1.1) + ", 0.00, " + std::to_string(lanes) + "\n"
		"-----\n";


	m_current_indentations = 0;

	addIndented(osc_ss, "");
	addIndented(osc_ss, "Trajectory: Ego");
	addVehicleTrajectoryFollower(osc_ss, m_interpreted_trace.getEgoTrajectory(), "ego", false);
	addIndented(osc_ss, "-----");

	for (auto& vehicle_name : m_interpreted_trace.getVehicleNames(true))
	{
		addIndented(osc_ss, "");
		addIndented(osc_ss, "Trajectory: " + vehicle_name);
		addVehicleTrajectoryFollower(osc_ss, m_interpreted_trace.getVehicleTrajectory(vehicle_name), vehicle_name, false);
		addIndented(osc_ss, "-----");
	}

	return osc_ss.str();
}
