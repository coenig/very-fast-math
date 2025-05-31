//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file
#pragma once

#include "mc_trajectory_generator.h"
#include "static_helper.h"

namespace vfm {
namespace mc {
namespace trajectory_generator {
namespace helpers {

	// On the decision to use a namespace here instead of a helpers class with static methods: https://stackoverflow.com/questions/1434937/namespace-functions-versus-static-methods-on-a-class

	struct LaneChange
	{
		int start_index, end_index;
		double y_at_start, y_at_end, duration;
	};

	void scale_param(FullTrajectory& editable_trajectory, PossibleParameter param, double scale_factor);
	void scale_duration(FullTrajectory& editable_trajectory, double duration_scale_factor);
	void add_to_param(FullTrajectory& editable_trajectory, PossibleParameter param, double value_to_add);
	void extrapolate_param_by_derivation(FullTrajectory& editable_trajectory, double start_value, PossibleParameter param, PossibleParameter derivation_param);
	void extrapolate_trajectory_to_t0(FullTrajectory& editable_trajectory, PossibleParameter pos_param, PossibleParameter velocity_param);
	std::vector<LaneChange> detect_lane_changes_from_states(FullTrajectory trajectory_positions);
	std::vector<LaneChange> detect_lane_changes_from_on_lane(FullTrajectory trajectory_positions);


} // helpers
} // trajectory_generator
} // mc
} // vfm
