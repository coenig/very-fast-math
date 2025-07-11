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

	std::string layout{R"(<Layout Database="../Databases/dummy.opt.osgb" File="../Databases/dummy.xodr"/>
	)"};
	std::string players{generatePlayers()};
	std::string player_actions{generatePlayerActions()};
	std::string objects_control{generateMovingObjectsControl()};

	// main scenario block
	// TODO: what about road topology? dummy.xodr ok? set 1/2/3 lane straight road?
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
 R"(<MovingObjectsControl/>
	<LightSigns/>
	<Selections/>
</Scenario>
)";

	return vtd_text.str();
}

std::string VTDgenerator::generatePlayers()
{
	std::string players{};

	for (auto& vehicle_name : m_interpreted_trace.getVehicleNames())
	{
		const trajectory_generator::FullTrajectory& trajectory = m_interpreted_trace.getVehicleTrajectory(vehicle_name);
		// TODO: get abs pos not rel pos!
		players += generatePlayer(vehicle_name, trajectory.front().second.at(PossibleParameter::vel_x), 
			Vector3{trajectory.front().second.at(PossibleParameter::pos_x), trajectory.front().second.at(PossibleParameter::pos_y), 0.0});
	}
		
		
	return players;
}

std::string VTDgenerator::generatePlayer(std::string name, double speed, Vector3 pos)
{
	std::string control = (name == "ego") ? "external" : "internal";

	std::string player{
	 R"(<Player>
			<Description Driver="DefaultDriver" Control=")" + control + R"(" AdaptDriverToVehicleType="true" Type="AlfaRomeo_Brera_10_BiancoSpino" Name=")" + name + R"("/>
			<Init>
				<Speed Value=")" + std::to_string(speed) + R"("/>
				<PosAbsolute X=")" + std::to_string(pos.x) + R"(" Y=")" + std::to_string(pos.y) + R"(" Z="0.0" Direction="0.0" AlignToRoad="true"/>
			</Init>
		</Player>
		)"};

	return player;
}

std::string VTDgenerator::generatePlayerActions()
{
	std::string player_actions{};

	for (auto& vehicle_name : m_interpreted_trace.getVehicleNames())
	{
		player_actions += generatePlayerAction(vehicle_name);
	}

	return player_actions;
}

std::string VTDgenerator::generatePlayerAction(std::string name)
{
	const trajectory_generator::FullTrajectory& trajectory = m_interpreted_trace.getVehicleTrajectory(name);

	std::string action{};

	for(const auto& trajectory_position : trajectory)
	{
		double time = trajectory_position.first;
		double rate = 0.0; // TODO: get rate from trajectory
		double target = trajectory_position.second.at(PossibleParameter::vel_x); // TODO: get target from trajectory

		action += generateSpeedChangeAction("speed_change" + std::to_string((int)time), rate, target, time);
	}

	std::string player_action{
	 R"(<PlayerActions Player=")" + name + R"(">
		)" + action + R"(</PlayerActions>
		)"};

	return player_action;
}

std::string VTDgenerator::generateSpeedChangeAction(std::string name, double rate, double target, double time)
{
	std::string speed_change_action{
	 R"(<SpeedChange Rate=")" + std::to_string(rate) + R"(" Target=")" + std::to_string(target) + R"(" ExecutionTimes="1" ActiveOnEnter="true" DelayTime=")" + std::to_string(time) + R"("/>)"
	};

	std::string action{
	 R"(	<Action Name=")" + name + R"(">
				<PosAbsolute CounterID="" CounterComp="COMP_EQ" Radius="5.0000000000000000e+00" X="1.1665091323852539e+02" Y="-1.9857120513916016e+00" NetDist="false" CounterVal="0" Pivot="Ego"/>
				)" + speed_change_action + R"(
			</Action>
		)"};	

	return action;
}

std::string VTDgenerator::generateMovingObjectsControl()
{
	return "\n\t";
}

std::string VTDgenerator::generatePolylinePathShape(std::string name, std::vector<Vector3> waypoints)
{
	return "";
}

