//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2025 Robert Bosch GmbH. All rights reserved.
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


std::string VTDgenerator::generate()
{
	std::stringstream vtd_text{};

	const std::string layout{generateLayout()};
	const std::string players{generatePlayers()};
	const std::string player_actions{generatePlayerActions()};
	const std::string objects_control{generateMovingObjectsControl()};

	// main scenario block
	vtd_text << 
R"(<?xml version="1.0" encoding="UTF-8"?>
<!DOCTYPE Scenario>
<Scenario RevMajor="2" RevMinor="2">
	)"
	<< layout <<
 R"(<VehicleList ConfigFile="Distros/Current/Config/Players/Vehicles"/>
	<DriverList ConfigFile="Distros/Current/Config/Players/driverCfg.xml"/>
	<CharacterList ConfigFile="Distros/Current/Config/Players/Pedestrians"/>
	<ObjectList ConfigFile="Distros/Current/Config/Players/Objects"/>
	<DynObjects Path="Distros/Current/Config/DynObjects/Logic"/>
	<TrafficElements/>
	<PulkTraffic/>
	<TrafficControl>
		)"
		<< players
		<< player_actions <<
 R"(</TrafficControl>
	<MovingObjectsControl>
		)"
		<< objects_control <<
 R"(</MovingObjectsControl>
	<LightSigns/>
	<Selections/>
</Scenario>
)";

	return vtd_text.str();
}

std::string VTDgenerator::generateLayout()
{
	std::string layout{};

	// TODO: get num lanes from CEX
	const int num_lanes{2};

	if(num_lanes == 2)
	{
		layout = R"(<Layout Database="../Databases/dummy.opt.osgb" File="../Databases/dummy.xodr"/>
	)";
		m_lane_offset_x = 20.0;
		m_lane_offset_y = -1.75;
	}
	else if(num_lanes == 3)
	{
		layout = R"(<Layout Database="../Databases/SR_3L_BML.opt.osgb" File="../Databases/SR_3L_BML.xodr"/>
	)";
		m_lane_offset_x = -70.0;
		m_lane_offset_y = 21.0;
	}
	else
	{
		assert(false && "invalid number of lanes!\n");
	}

	return layout;
}

std::string VTDgenerator::generatePlayers() const
{
	std::string players{};

	int object_index{0};
	for (const std::string& vehicle_name : m_interpreted_trace.getVehicleNames())
	{
		const trajectory_generator::FullTrajectory& trajectory = m_interpreted_trace.getVehicleTrajectory(vehicle_name);

		players += generatePlayer(vehicle_name, trajectory.front().second.at(PossibleParameter::vel_x), object_index++);
	}
		
	return players;
}

std::string VTDgenerator::generatePlayer(const std::string& vehicle_name, const double speed, const int object_index) const
{
	const std::string control{(vehicle_name == "ego") ? "external" : "internal"};

	const trajectory_generator::FullTrajectory& trajectory = m_interpreted_trace.getVehicleTrajectory(vehicle_name);
	const double pathTargetS{trajectory.back().second.at(PossibleParameter::pos_x) - trajectory.front().second.at(PossibleParameter::pos_x)};

	const std::string player{
	 R"(<Player>
			<Description Driver="DefaultDriver" Control=")" + control + R"(" AdaptDriverToVehicleType="true" Type="AlfaRomeo_Brera_10_BiancoSpino" Name=")" + vehicle_name + R"("/>
			<Init>
				<Speed Value=")" + std::to_string(speed) + R"("/>
				<PosPathShape/>
                <PathShapeRef StartS="0.1" EndAction="continue" TargetS=")" + std::to_string(pathTargetS) + R"(" PathShapeId=")" + std::to_string(object_index) + R"("/>
			</Init>
		</Player>
		)"};

	return player;
}

std::string VTDgenerator::generatePlayerActions() const
{
	std::string player_actions{};

	for (const std::string& vehicle_name : m_interpreted_trace.getVehicleNames())
	{
		player_actions += generatePlayerAction(vehicle_name);
	}

	return player_actions;
}

std::string VTDgenerator::generatePlayerAction(const std::string& vehicle_name) const
{
	const trajectory_generator::FullTrajectory& trajectory = m_interpreted_trace.getVehicleTrajectory(vehicle_name);

	std::string action{};

	Waypoint waypoint{};
	// get x/y from start position for ALL actions, arbitrated with delay time
	waypoint.x = trajectory[0].second.at(PossibleParameter::pos_x) + m_lane_offset_x;
	waypoint.y = trajectory[0].second.at(PossibleParameter::pos_y) + m_lane_offset_y;

	for(int i=1; i<trajectory.size(); ++i)
	{
		const auto& trajectory_position{trajectory.at(i-1)};
		waypoint.time = trajectory_position.first;
		waypoint.v_target = trajectory.at(i).second.at(PossibleParameter::vel_x);
		const double v_old_target{trajectory_position.second.at(PossibleParameter::vel_x)};
		waypoint.v_rate = std::abs(waypoint.v_target - v_old_target); // use acc_x instead?

		action += generateSpeedChangeAction(vehicle_name, "speed_change" + std::to_string((int)waypoint.time), waypoint);
	}

	const std::string player_action{
	 R"(<PlayerActions Player=")" + vehicle_name + R"(">
		)" + action + R"(</PlayerActions>
		)"};

	return player_action;
}

std::string VTDgenerator::generateSpeedChangeAction(const std::string& vehicle_name, const std::string& name, const Waypoint& waypoint) const
{
	// note: the Force="true" is crucial here to overwrite the past action at activation after delay time!
	const std::string speed_change_action{
	 R"(<SpeedChange Rate=")" + std::to_string(waypoint.v_rate) + R"(" Target=")" + std::to_string(waypoint.v_target) + R"(" Force="true" ExecutionTimes="1" ActiveOnEnter="true" DelayTime=")" + std::to_string(waypoint.time) + R"("/>)"
	};

	const std::string action{
	 R"(	<Action Name=")" + name + R"(">
				<PosAbsolute CounterID="" CounterComp="COMP_EQ" Radius="5.0000000000000000e+00" X=")" + std::to_string(waypoint.x) + R"(" Y=")" + std::to_string(waypoint.y) + R"(" NetDist="false" CounterVal="0" Pivot=")" + vehicle_name + R"("/>
				)" + speed_change_action + R"(
			</Action>
		)"};	

	return action;
}

std::string VTDgenerator::generateMovingObjectsControl() const
{
	std::string path_shapes{};

	int object_index{0};
	for (const std::string& vehicle_name : m_interpreted_trace.getVehicleNames())
	{
		path_shapes += generatePolylinePathShape(vehicle_name, object_index++);
	}

	return path_shapes;
}

std::string VTDgenerator::generatePolylinePathShape(const std::string& vehicle_name, const int object_index) const
{
	const trajectory_generator::FullTrajectory& trajectory = m_interpreted_trace.getVehicleTrajectory(vehicle_name);

	std::string waypoints{};
	for(const auto& waypoint : trajectory)
	{
		const double x{waypoint.second.at(PossibleParameter::pos_x) + m_lane_offset_x};
		const double y{waypoint.second.at(PossibleParameter::pos_y) + m_lane_offset_y};
		waypoints += R"(<Waypoint X=")" + std::to_string(x) + R"(" Y=")" + std::to_string(y) + R"(" Options="0x00000000" Z="0.0" Weight="1.0" Yaw="0.0" Pitch="0.0" Roll="0.0"/>
			)";
	}

	const std::string path_shape{
	 R"(<PathShape ShapeId=")" + std::to_string(object_index) + R"(" ShapeType="polyline" Closed="false" Name="PathShape)" + std::to_string(object_index) + R"(">
			)" + waypoints + 
		R"(</PathShape>
		)"};

	return path_shape;
}
