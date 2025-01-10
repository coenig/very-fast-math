//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file
#pragma once

#include "model_checking/cex_processing/mc_trajectory_configs.h"
#include "static_helper.h"
#include "geometry/images.h"
#include "simulation/highway_image.h"

#include <string>
#include <limits>
#include "failable.h"

namespace vfm {
namespace mc {
namespace trajectory_generator {

class VehicleWithTrajectory : public Failable
{
public:
   VehicleWithTrajectory() : Failable("VehicleWithTrajectory") {}

	ParameterMap m_parameters_this_state{};
	//std::string m_vehicle_name;
	FullTrajectory m_trajectory;

	void appendCurrentParametersToTrajectory(double timestamp, std::vector<ParameterDetails> required_parameters);

	void clearCurrentParameters();
	double getDuration() const;
};


// This represents an OSC file and fill be filled with data while parsing through an MCTrace.
class MCinterpretedTrace : public Failable
{
public:
	MCinterpretedTrace(
		const std::string trace_file, const InterpretationConfiguration config);
	MCinterpretedTrace(
		const vfm::MCTrace& trace, const InterpretationConfiguration config);

	void applyScaling(const double x_scale_factor, const double y_scale_factor, const double time_scale_factor);
	void applyOffset(const double x_offset, const double y_offset, const bool to_ego = true, const bool to_other_vehicles = true);
	void printDebug(const std::string header) const;
	void applyInterpolation(const int steps_to_insert);
	void interpolate(FullTrajectory& editable_trajectory, const size_t steps_between);
	void interpolateDataPacks(DataPackTrace& data, const size_t steps_between);

	void pushTimestamp(const double t);
	void pushVehicleParameter(const std::string vehicle_name, const double value, const PossibleParameter param, const bool allow_multiple = true);

	double getDuration() const;

	void noteVehicle(std::string name);
	void squashStates(std::function<bool(const VehicleWithTrajectory& traj, int index_to_check)> started_new_step);

	MCinterpretedTrace clone() const;

	const std::vector<std::string> getVehicleNames(const bool skip_ego = false) const;
	const FullTrajectory& getVehicleTrajectory(const std::string name) const;
	const FullTrajectory& getEgoTrajectory() const;
	FullTrajectory& getEditableVehicleTrajectory(const std::string name);

	const std::vector<ParameterDetails>& getRequiredParameters() const;
	const ParameterDetails& getParameterDetails(PossibleParameter param) const;

	int m_current_trace_position{ 0 };

	void appendDummyDataAtTheEndOfTrace();
	DataPackTrace getDataTrace() const;

private:
	void startState(const std::string state_name);
	void endState();

	bool verifyConsistency();
	void appendWarning(const std::string warning_text);

	const InterpretationConfiguration m_config;

	std::map<std::string, VehicleWithTrajectory> m_vehicle_trajectories;
	const std::vector<ParameterDetails> m_required_parameters {};
	double m_timestamp_last_state{ std::numeric_limits<double>::quiet_NaN() };

	std::string m_current_state_name{ "" };
	double m_timestamp_this_state{ std::numeric_limits<double>::quiet_NaN() };

	std::vector<TraceStepWarning> m_trace_step_warnings{ 0 };

	DataPackPtr m_current_data{};
	DataPackTrace m_data_trace{};
};

std::shared_ptr<Image> selectImage(const std::shared_ptr<vfm::Image> img1, const std::shared_ptr<vfm::HighwayImage> img2);

} // trajectory_generator
} // mc
} // vfm
