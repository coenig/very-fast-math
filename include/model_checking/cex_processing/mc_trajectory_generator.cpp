//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file

#include "model_checking/cex_processing/mc_trajectory_generator.h"
#include "model_checking/cex_processing/mc_trajectory_config_helpers.h"
#include "data_pack.h"

#include <sstream>
#include <iomanip>
#include <thread>


using namespace vfm::mc::trajectory_generator;
using namespace vfm::mc::trajectory_generator::helpers;

void associateEnums(vfm::DataPack& data)
{
	data.addEnumMapping("ActionDir", { {0, "LEFT"}, {1, "CENTER"}, {2, "RIGHT"} });
	data.addEnumMapping("ActionType", { 
      {0, "UNDEFINED"}, 
      {1, "OFFROAD"}, 
      {2, "LANE_FOLLOWING"},
      {3, "LANECHANGE_LEFT"},
      {4, "LANECHANGE_RIGHT"},
      {5, "LANECHANGE_ABORT"},
      {6, "CONVERGE_TO_LANE"},
      {7, "EGO_OPEN_LOOP"},
      {8, "IN_COLLISION"},
   });
	data.associateVarToEnum("debug.gaps___609___.turn_signals_front", "ActionDir");
	data.associateVarToEnum("debug.gaps___629___.turn_signals_front", "ActionDir");
	data.associateVarToEnum("ego.lc_direction", "ActionDir");
	data.associateVarToEnum(R"(planner."!{tar_dir}")", "ActionDir");
	data.associateVarToEnum("env.tar_dir", "ActionDir");
	data.associateVarToEnum("ego.mode", "ActionType");
	data.associateVarToEnum("env.ego.mode", "ActionType");
	data.associateVarToEnum("tar_dir", "ActionDir");
}

MCinterpretedTrace::MCinterpretedTrace(
   const int trace_num, const std::string trace_file, const InterpretationConfiguration config) : MCinterpretedTrace(
		trace_num, StaticHelper::extractMCTracesFromNusmvFile(trace_file), config)
{}

MCinterpretedTrace::MCinterpretedTrace(
	const int trace_num, const std::vector<vfm::MCTrace>& traces, const InterpretationConfiguration config) : Failable("Visu_Trajectory"),
	m_config(config), m_required_parameters(config.m_required_parameters), m_current_data{ std::make_shared<DataPack>() }
{
	config.m_preprocessing(*this);

	std::vector<ParameterMap> params_until_now{};

	associateEnums(*m_current_data);

	for (const auto& state : traces.at(trace_num).getConstTrace()) // for every state
	{
		startState(state.first);

		for (const auto& varVal : state.second) // for every entry in that state
		{
			std::string key = varVal.first;
			std::string value_string = varVal.second;
			std::string processed_value{ StaticHelper::replaceAll(value_string, "____", "::") };

			float value{};
			if (StaticHelper::isParsableAsFloat(value_string)) {
				value = std::stof(value_string);
			}
			else if (StaticHelper::stringContains(processed_value, "::") && m_current_data->isDeclared(processed_value)) {
				value = m_current_data->getSingleVal(processed_value); // Works for :: and ____.
			}
			else if (value_string == "state_EnvironmentModel" || value_string == "state_TacticalPlanner") {
				value = std::numeric_limits<float>::min();
			}
			else {
				value = StaticHelper::isBooleanTrue(value_string);
			}

			m_current_data->addOrSetSingleVal(key, value);

			// extract the elements of the key
			std::stringstream key_ss(key);
			std::vector<std::string> key_elements;
			std::string tmp;
			while (std::getline(key_ss, tmp, config.m_key_separator)) {
				key_elements.push_back(tmp);
			}

			// first apply the general_interpretations that check against the key directly
			if (config.m_general_interpretations.count(key))
			{
				config.m_general_interpretations.at(key)->interpret(*this, key_elements, value_string);
			}

			// check if the first key element is a potential vehicle (like "veh1")
			if (std::find(config.m_possible_vehicle_keys.begin(), config.m_possible_vehicle_keys.end(), key_elements[0]) != config.m_possible_vehicle_keys.end())
			{
				auto vehicle_name = key_elements[0];
				noteVehicle(vehicle_name);

				// get the key without the vehicle name and the dot (e.g. "veh1.vel" -> "vel")
				auto sub_key = key.erase(0, vehicle_name.length() + 1);

				// apply the vehicle-specific interpretations based on the sub key
				if (config.m_vehicle_interpretations.count(sub_key))
				{
					config.m_vehicle_interpretations.at(sub_key)->interpret(*this, key_elements, value_string);
				}
			}
		}

		params_until_now.push_back(m_vehicle_trajectories.at(config.m_ego_vehicle_name).m_parameters_this_state);
		
		if (m_config.m_check_state_end(params_until_now))
		{
			endState();
			params_until_now.clear();
		}

		m_current_trace_position++;
	}

	// Additional final state if ended without an endState() call
	if (m_current_state_name != "")
		endState();

	printDebug("BEFORE POSTPROCESSING");

	config.m_postprocessing(*this);

	// print warnings
	std::cout << "WARNINGS" << std::endl;

	for (const TraceStepWarning& warning : m_trace_step_warnings)
	{
		addWarning("In state'" + warning.first + "': " + warning.second);
	}

	if (!verifyConsistency())
	{
		addError("Consistency Check Failed! Aborting.");
	}
}

MCinterpretedTrace MCinterpretedTrace::clone() const
{
	return *this;
}

void MCinterpretedTrace::applyScaling(const double x_scale_factor, const double y_scale_factor, const double duration_scale_factor)
{
	for (const std::string& vehicle_name : getVehicleNames())
	{
		auto& trajectory = getEditableVehicleTrajectory(vehicle_name);

		scale_param(trajectory, PossibleParameter::pos_x, x_scale_factor);
		scale_param(trajectory, PossibleParameter::vel_x, x_scale_factor * (1.0/duration_scale_factor));

		scale_param(trajectory, PossibleParameter::pos_y, y_scale_factor);
		scale_param(trajectory, PossibleParameter::vel_y, duration_scale_factor);

		scale_duration(trajectory, duration_scale_factor);
	}
}

void MCinterpretedTrace::applyOffset(const double x_offset, const double y_offset, const bool to_ego, const bool to_other_vehicles)
{
	if (to_ego)
	{
		auto& trajectory = getEditableVehicleTrajectory(m_config.m_ego_vehicle_name);
		add_to_param(trajectory, PossibleParameter::pos_x, x_offset);
		add_to_param(trajectory, PossibleParameter::pos_y, y_offset);
	}

	if (to_other_vehicles)
	{
		for (const std::string& vehicle_name : getVehicleNames(true))
		{
			auto& trajectory = getEditableVehicleTrajectory(vehicle_name);

			add_to_param(trajectory, PossibleParameter::pos_x, x_offset);
			add_to_param(trajectory, PossibleParameter::pos_y, y_offset);
		}
	}
}

void MCinterpretedTrace::printDebug(const std::string header) const
{
	addNotePlain("\n\033[1mPrint States [" + header + "]\033[0m");

	for (const ParameterDetails& param : m_config.m_required_parameters)
	{
		addNotePlain("\n\nValues of parameter: \033[1m" + param.name_ + "\033[0m");

		bool first = true;

		for (const std::string& vehicle_name : getVehicleNames())
		{
			const std::vector<TrajectoryPosition>& trajectory = getVehicleTrajectory(vehicle_name);
			const int decimals = 5;

			if (first)
			{
				std::stringstream line{};
				line << std::setw(27) << std::setfill(' ') << "Time ->";
				for (const TrajectoryPosition& traj_pos : trajectory)
				{
					double time = traj_pos.first;
					line << " |" << std::setw(decimals + 1) << std::setfill(' ') << std::to_string(time).substr(0, decimals);
				}
				addNotePlain(line.str());
				first = false;
			}

			std::stringstream line{};
			line << "Vehicle: " << std::setw(15) << std::setfill(' ') << vehicle_name << " ->";
			for (const TrajectoryPosition& traj_pos : trajectory)
			{
				ParameterMap param_data_map = traj_pos.second;
				double value = param_data_map.at(param.identifier_);

				line << " |" << std::setw(decimals + 1) << std::setfill(' ') << std::to_string(value).substr(0, decimals);
			}

			addNotePlain(line.str());
		}
	}

	addNotePlain("\n");
}

void MCinterpretedTrace::applyInterpolation(const int steps_to_insert, const MCTrace& trace)
{
	if (steps_to_insert <= 0)
	{
		return;
	}

   for (const std::string& vehicle_name : getVehicleNames())
	{
		auto& trajectory = getEditableVehicleTrajectory(vehicle_name);
      interpolate(vehicle_name, trajectory, trace, steps_to_insert);
      int x{};
	}

   interpolateDataPacks(m_data_trace, steps_to_insert);
}

void MCinterpretedTrace::interpolate(const std::string& vehicle_name, FullTrajectory& editable_trajectory, const MCTrace& trace, const size_t steps_between)
{
	double factor = 1.0 / (steps_between + 1);
   int trace_cnt{ 0 };

	for (size_t i = 0; i < editable_trajectory.size() - 1; i++)
	{
		TrajectoryPosition prev = editable_trajectory[i];
		TrajectoryPosition next = editable_trajectory[i + 1];
		double next_time = next.first;
		ParameterMap next_param = next.second;
		ParameterMap prev_param = prev.second;

      // Here come the helper variables for interpolation when jumping over borders of straight sections and curved junctions.
      // TODO: Maybe this all should better go in the actual interpolation function??
      const int on_straight_section{ (int)std::stof(trace.getLastValueOfVariableAtStep(vehicle_name + ".on_straight_section", trace_cnt)) };
      const int traversion_from{ (int) std::stof(trace.getLastValueOfVariableAtStep(vehicle_name + ".traversion_from", trace_cnt)) };
      const int traversion_to{ (int) std::stof(trace.getLastValueOfVariableAtStep(vehicle_name + ".traversion_to", trace_cnt)) };

      if (on_straight_section < 0 && traversion_from < 0 && traversion_to < 0) {
         addError("Car '" + vehicle_name + "' is neither on straight section nor on curved junction.");
      }

      const int on_lane{ (int)(std::stof(trace.getLastValueOfVariableAtStep(vehicle_name + ".on_lane", trace_cnt)) / 2) };
      const int current_seclet_length{ (on_straight_section >= 0
            ? (int)std::stof(trace.getLastValueOfVariableAtStep("env.section_" + std::to_string(on_straight_section) + "_end", trace_cnt))
            : (int)std::stof(trace.getLastValueOfVariableAtStep("env.arclength_from_sec_" + std::to_string(traversion_from) + "_to_sec_" + std::to_string(traversion_to) + "_on_lane_" + std::to_string(on_lane), trace_cnt))
            ) };
      const bool is_switching_towards_next_step{ trace.getLastValueOfVariableAtStep(vehicle_name + ".on_straight_section", trace_cnt) != trace.getLastValueOfVariableAtStep(vehicle_name + ".on_straight_section", trace_cnt + 2) };

		for (size_t j = 1; j < (steps_between + 1); j++)
		{
			double interp_time = prev.first + (j * factor) * (next_time - prev.first);

			TrajectoryPosition interp_p;
			interp_p.first = interp_time;
         double store_regular_interp_x{ -1 };

			ParameterMap interp_params;
			for (auto& pp : prev_param)
			{
				PossibleParameter param_key = pp.first;
				double current_value = pp.second;
				double next_value = next_param[param_key];

				const ParameterDetails& details = getParameterDetails(param_key);

            if (is_switching_towards_next_step) {
               if (param_key == PossibleParameter::pos_x) {
                  store_regular_interp_x = details.interpolation_method_->interpolate(current_value, next_value + current_seclet_length, j * factor);
                  details.interpolation_method_->addAdditionalData({ (double)current_seclet_length });
               }
               if (param_key == PossibleParameter::on_straight_section || param_key == PossibleParameter::traversion_from || param_key == PossibleParameter::traversion_to) {
                  if (store_regular_interp_x < 0) addError("Wrong order - pos_x should have been calculated before.");

                  if (store_regular_interp_x > current_seclet_length) {
                     details.interpolation_method_->addAdditionalData({ 1 });
                  }
               }
            }

				interp_params[pp.first] = details.interpolation_method_->interpolate(current_value, next_value, j*factor);
            details.interpolation_method_->clearAdditionalData();

				/*
				if (param_key == PossibleParameter::turn_signal_left || param_key == PossibleParameter::turn_signal_right)
					interp_params[pp.first] = current_value;
				else
					interp_params[pp.first] = current_value + (j * factor) * (next_value - current_value);
				*/
			}

			interp_p.second = interp_params;

			editable_trajectory.insert(editable_trajectory.begin() + i + (j), interp_p);
		}

		i += steps_between;
      trace_cnt += 2;
	}
}

void MCinterpretedTrace::interpolateDataPacks(DataPackTrace& data_trace, const size_t steps_between)
{
	double factor = 1.0 / (steps_between + 1);

	for (size_t i = 0; i < data_trace.size() - 1; i++)
	{
		auto data = data_trace[i + 1];

		for (size_t j = 1; j < (steps_between + 1); j++)
		{
			data_trace.insert(data_trace.begin() + i + j, data);
		}

		i += steps_between;
	}
}

const std::vector<std::string> MCinterpretedTrace::getVehicleNames(const bool skip_ego) const
{
	std::vector<std::string> keys;
	keys.reserve(m_vehicle_trajectories.size());
	for (auto const& imap : m_vehicle_trajectories)
		if (!skip_ego || imap.first != m_config.m_ego_vehicle_name)
			keys.push_back(imap.first);

	return keys;
}

const std::vector<TrajectoryPosition>& MCinterpretedTrace::getVehicleTrajectory(const std::string name) const
{
	return m_vehicle_trajectories.at(name).m_trajectory;
}

const std::vector<TrajectoryPosition>& MCinterpretedTrace::getEgoTrajectory() const
{
	return m_vehicle_trajectories.at(m_config.m_ego_vehicle_name).m_trajectory;
}

std::vector<TrajectoryPosition>& MCinterpretedTrace::getEditableVehicleTrajectory(const std::string name)
{
	return m_vehicle_trajectories[name].m_trajectory;
}

const std::vector<ParameterDetails>& MCinterpretedTrace::getRequiredParameters() const
{
	return m_required_parameters;
}

const ParameterDetails& MCinterpretedTrace::getParameterDetails(PossibleParameter param) const
{
	for (const auto& param_detail : m_required_parameters)
	{
		if (param_detail.identifier_ == param)
			return param_detail;
	}
	throw std::invalid_argument("Parameter is not among the required parameters");
}

void vfm::mc::trajectory_generator::MCinterpretedTrace::appendDummyDataAtTheEndOfTrace()
{
	m_data_trace.push_back(m_data_trace.back());
}

std::vector<std::shared_ptr<vfm::DataPack>> vfm::mc::trajectory_generator::MCinterpretedTrace::getDataTrace() const
{
	return m_data_trace;
}

double MCinterpretedTrace::getDuration() const
{
	double longest_trajectory_duration = 0;

	for (auto& vehicle_with_name : m_vehicle_trajectories)
	{
		const VehicleWithTrajectory& vehicle = vehicle_with_name.second;
		// timestamp of the last element in the trajectory
		const double duration = vehicle.getDuration();
		if (duration > longest_trajectory_duration)
			longest_trajectory_duration = duration;
	}

	return longest_trajectory_duration;
}


void MCinterpretedTrace::startState(const std::string state_name)
{
	m_current_state_name = state_name;
}

void MCinterpretedTrace::endState()
{
	assert(m_current_state_name != "");

	if (std::isnan(m_timestamp_this_state))
	{
		m_timestamp_this_state = std::isnan(m_timestamp_last_state) ? 0 : m_timestamp_last_state + m_config.m_default_step_time;
	}

	// Add to the trajectory
	for (auto& vehicle_with_name : m_vehicle_trajectories)
	{
		VehicleWithTrajectory& vehicle = vehicle_with_name.second;
		vehicle.appendCurrentParametersToTrajectory(m_timestamp_this_state, m_config.m_required_parameters);
	}

	m_data_trace.push_back(m_current_data);

	m_timestamp_last_state = m_timestamp_this_state;

	// Clear everything related to the current state that has ended.
	auto temp_data{ std::make_shared<DataPack>() };

	if (m_current_data) {
		temp_data->initializeValuesBy(m_current_data);
	}

	associateEnums(*temp_data);

	m_current_data = temp_data;
	m_current_state_name = "";
	m_timestamp_this_state = std::numeric_limits<double>::quiet_NaN();

	for (auto& vehicle_with_name : m_vehicle_trajectories)
	{
		vehicle_with_name.second.clearCurrentParameters();
	}
}

void MCinterpretedTrace::noteVehicle(std::string vehicle_name)
{
	if (m_vehicle_trajectories.count(vehicle_name) == 0)
	{
		// note down that this vehicle exists
		m_vehicle_trajectories[vehicle_name] = VehicleWithTrajectory();
	}
}

void MCinterpretedTrace::squashStates(std::function<bool(const VehicleWithTrajectory& traj, int index_to_check)> started_new_step)
{
	for (auto it = m_vehicle_trajectories.begin(); it != m_vehicle_trajectories.end(); ++it)
	{
		auto veh_name = it->first;
		VehicleWithTrajectory& veh_with_trajectory = it->second;

		std::vector<int> indicies_to_squash_at;

		for (int i = 0; i < veh_with_trajectory.m_trajectory.size(); i++)
		{
			if (started_new_step(veh_with_trajectory, i))
			{
				indicies_to_squash_at.push_back(i - 1);
			}
		}

		std::vector<TrajectoryPosition> squashed_trajectory;

		double time = 0;
		for (auto trajectory_index : indicies_to_squash_at)
		{
			ParameterMap currently_final_states = veh_with_trajectory.m_trajectory[trajectory_index - 1].second;
			TrajectoryPosition traj_pos(time, currently_final_states);
			squashed_trajectory.push_back(traj_pos);

			time += m_config.m_default_step_time;
		}

		veh_with_trajectory.m_trajectory = squashed_trajectory;
	}
}

void MCinterpretedTrace::pushTimestamp(double t)
{
	if (!std::isnan(m_timestamp_this_state))
		addWarning("This state had two timestamps!");
	m_timestamp_this_state = t;
}

void MCinterpretedTrace::pushVehicleParameter(std::string vehicle_name, double value, PossibleParameter param, bool allow_multiple)
{
	auto& parameters_this_state_this_vehicle = m_vehicle_trajectories[vehicle_name].m_parameters_this_state;

	if (!allow_multiple && parameters_this_state_this_vehicle.count(param) != 0)
		addWarning("A vehicle parameter occured twice this state for vehicle: " + vehicle_name);

	parameters_this_state_this_vehicle[param] = value;
}


bool MCinterpretedTrace::verifyConsistency()
{
	if (m_vehicle_trajectories.size() == 0)
	{
		addWarning("No vehicles found!");
		return false;
	}

	return true;
}

void MCinterpretedTrace::appendWarning(const std::string warning_text)
{
	TraceStepWarning warning(m_current_state_name, warning_text);
	m_trace_step_warnings.push_back(warning);
}


// VehicleWithTrajectory

void VehicleWithTrajectory::appendCurrentParametersToTrajectory(double timestamp, std::vector<ParameterDetails> required_parameters)
{
   TrajectoryPosition last_trajectory_entry{};

   if (!m_trajectory.empty())
   {
      last_trajectory_entry = m_trajectory.back();

      if (last_trajectory_entry.first > timestamp)
      {
         addError("The timestamp of a new trajectory entry is behind the last one.\n Note that a cause of this can be that the default duration of missing timestamps is too long.");
      }
   }

   ParameterMap parameters{};

   // Transfer parameters
   for (ParameterDetails param : required_parameters)
   {
      if (m_parameters_this_state.count(param.identifier_) == 0) // parameter is missing
      {
         double last_val_or_nan = (last_trajectory_entry.second.count(param.identifier_) == 0) ? std::numeric_limits<double>::quiet_NaN() : last_trajectory_entry.second[param.identifier_];
         // call the method to retrive what's valid if it's missing
         parameters[param.identifier_] = param.provide_if_missing_(last_val_or_nan);
      }
      else
      {
         parameters[param.identifier_] = m_parameters_this_state[param.identifier_];
      }
   }
   // Note: The loop above only transfers the requirec parameters.
   // Other ones that have been found in the current MC state are thrown away (but could be used here in this method if needed).

   TrajectoryPosition trajectory_position(timestamp, parameters);
   m_trajectory.push_back(trajectory_position);
}

void VehicleWithTrajectory::clearCurrentParameters()
{
	m_parameters_this_state.clear();
}

double VehicleWithTrajectory::getDuration() const
{
	return m_trajectory.back().first;
}

