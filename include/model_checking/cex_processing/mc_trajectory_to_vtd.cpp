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

	std::string players{
		R"(<Player>
			<Description Driver="DefaultDriver" Control="external" AdaptDriverToVehicleType="true" Type="AlfaRomeo_Brera_10_BiancoSpino" Name="Ego"/>
			<Init>
				<Speed Value="2.7777777777777779e+01"/>
				<PosAbsolute X="1.1665091323852539e+02" Y="-1.9857120513916016e+00" Z="0.0000000000000000e+00" Direction="0.0000000000000000e+00" AlignToRoad="true"/>
			</Init>
		</Player>
		<Player>
			<Description Driver="DefaultDriver" Control="internal" AdaptDriverToVehicleType="true" Type="AlfaRomeo_Brera_10_BiancoSpino" Name="LeftFastOvertaker"/>
			<Init>
				<Speed Value="3.8888888888888886e+01"/>
				<PosAbsolute X="2.0459701538085938e+01" Y="1.1126174926757812e+00" Z="0.0000000000000000e+00" Direction="6.2707495235650450e+00" AlignToRoad="true"/>
			</Init>
		</Player>
		)"};

	std::string player_actions{
		R"(<PlayerActions Player="Ego">
			<Action Name="keep_velo">
				<PosAbsolute CounterID="" CounterComp="COMP_EQ" Radius="5.0000000000000000e+00" X="1.1665091323852539e+02" Y="-1.9857120513916016e+00" NetDist="false" CounterVal="0" Pivot="Ego"/>
				<SpeedChange Rate="4.0000000000000000e+00" Target="2.7777777777777779e+01" Force="true" ExecutionTimes="1" ActiveOnEnter="true" DelayTime="0.0000000000000000e+00"/>
			</Action>
		</PlayerActions>
		<PlayerActions Player="LeftFastOvertaker">
			<Action Name="keep_velo">
				<PosAbsolute CounterID="" CounterComp="COMP_EQ" Radius="5.0000000000000000e+00" X="2.0459701538085938e+01" Y="1.1126174926757812e+00" NetDist="false" CounterVal="0" Pivot="LeftFastOvertaker"/>
				<SpeedChange Rate="4.0000000000000000e+00" Target="3.8888888888888886e+01" Force="true" ExecutionTimes="1" ActiveOnEnter="true" DelayTime="0.0000000000000000e+00"/>
			</Action>
		</PlayerActions>
	)"};

	std::string objects_control{
		R"(
	)"};


	// main scenario block
	vtd_text << 
R"(<?xml version="1.0" encoding="UTF-8"?>
<!DOCTYPE Scenario>
<Scenario RevMajor="2" RevMinor="2">
	<Layout Database="../Databases/dummy.opt.osgb" File="../Databases/dummy.xodr"/>
	<VehicleList ConfigFile="Distros/Current/Config/Players/Vehicles"/>
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