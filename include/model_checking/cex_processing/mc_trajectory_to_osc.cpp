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
	std::stringstream osc_ss;
	m_current_indentations = 0;

	std::replace(scenario_name.begin(), scenario_name.end(), '.', '_'); // replace all '.' to '_'
	std::replace(scenario_name.begin(), scenario_name.end(), '-', '_'); // replace all '-' to '_'

	addIndented(osc_ss, "# AUTO GENERATED OSC: " + scenario_name + " - Source: Model Checker\n");

	if (prerequisites != "")
	{
		osc_ss << "# Prerequisites\n";
		addIndented(osc_ss, prerequisites);
	}
	osc_ss << "\n";

	addIndented(osc_ss, "# Main Scenario");
	addIndented(osc_ss, "scenario " + scenario_name + ":");
	m_current_indentations = 1;

	addVariables(osc_ss);
	osc_ss << "\n";

	// root node
	addIndented(osc_ss, "do stop_at(elapsed(seconds: " + std::to_string(m_interpreted_trace.getDuration()) + ")):");
	m_current_indentations = 2;


	if (!control_ego)
	{
		// If not controlling the ego, we need to set the initial position and velocity
		addIndented(osc_ss, "serial():");
		m_current_indentations = 3;
		addEgoStartConditions(osc_ss);
	}

	addIndented(osc_ss, "parallel():");
	m_current_indentations = control_ego ? 3 : 4;

	addCustomerSetup(osc_ss, control_ego);

	addVehicleTrajectoryFollowers(osc_ss, control_ego);

	return osc_ss.str();
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
		"" + std::to_string(max_x * 1.1)+", 0.00, " + std::to_string(lanes) + "\n"
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

void OSCgenerator::addVariables(std::stringstream& osc_ss)
{
	addIndented(osc_ss, "ego: ego_vehicle_SIM_VEHICLE_NAME # make sure to replace the vehicle name here before usage");
	for (auto& vehicle_name : m_interpreted_trace.getVehicleNames(true))
	{
		addIndented(osc_ss, vehicle_name + ": vehicle");
	}
}

void OSCgenerator::addCustomerSetup(std::stringstream& osc_ss, bool control_ego)
{
	// Configure Ego with or without HWP mode enabled
	if (control_ego)
	{
		addIndented(osc_ss, "customer_function_setup(ego)");
	}
	else
	{
		addIndented(osc_ss, "customer_function_setup_hwp_engaged(ego)");
		addIndented(osc_ss, "serial():");
		addIndented(osc_ss, "# Ego follows lane with constant speed until system is engaged", 1);
		addIndented(osc_ss, "parallel():", 1);
		addIndented(osc_ss, "ego.keep_speed()", 2);
		addIndented(osc_ss, "ego.follow_lane()", 2);
		addIndented(osc_ss, "with:", 1);
		addIndented(osc_ss, "until ego.is_laterally_controlled_by_sut() and ego.is_longitudinally_controlled_by_sut()", 2);
		addIndented(osc_ss, "", 1);
		addIndented(osc_ss, "nsdm_repeat(8):", 1);
		addIndented(osc_ss, "ego.press_and_hold_button(button_type: button_type::acc_speed_increase, delay_in_ms: 50, duration_in_ms: 50)", 2);
	}
}

void OSCgenerator::addEgoStartConditions(std::stringstream& osc_ss)
{
	const FullTrajectory& ego_traj = m_interpreted_trace.getEgoTrajectory();
	const ParameterMap& start_parameters = ego_traj.front().second;

	addIndented(osc_ss, "ego.assign_position(position_3d("
		"x: " + std::to_string(start_parameters.at(PossibleParameter::pos_x))
		+ ", y: " + std::to_string(start_parameters.at(PossibleParameter::pos_y))
		+ ", z: " + std::to_string(start_parameters.at(PossibleParameter::pos_z))
		+ "))");

	addIndented(osc_ss, "ego.assign_speed(" + std::to_string(start_parameters.at(PossibleParameter::vel_x)) + ")");

	osc_ss << "\n";
}

void OSCgenerator::addVehicleTrajectoryFollowers(std::stringstream& osc_ss, bool control_ego)
{
	if (control_ego)
	{
		addIndented(osc_ss, "");
		addVehicleTrajectoryFollower(osc_ss, m_interpreted_trace.getEgoTrajectory(), "ego", true);
	}

	for (auto& vehicle_name : m_interpreted_trace.getVehicleNames(true))
	{
		addIndented(osc_ss, "");
		addVehicleTrajectoryFollower(osc_ss, m_interpreted_trace.getVehicleTrajectory(vehicle_name), vehicle_name, true);
	}
}

void OSCgenerator::addVehicleTrajectoryFollower(std::stringstream& osc_ss, FullTrajectory trajectory, std::string vehicle_name, bool wrap_in_osc_node)
{
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
