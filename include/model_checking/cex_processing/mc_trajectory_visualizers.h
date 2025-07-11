//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file
#pragma once

#include "model_checking/cex_processing/mc_trajectory_configs.h"
#include "model_checking/cex_processing/mc_trajectory_generator.h"
#include "static_helper.h"
#include "geometry/images.h"
#include "geometry/gif_writer.h"
#include "geometry/polygon_2d.h"
#include "geometry/images.h"
#include "parser.h"
#include "fsm.h"
#include "failable.h"
#include "simulation/env2d_simple.h"
#include "model_checking/mc_types.h"

#include <vector>
#include <string>
#include <algorithm>
#include <cstdlib>
#include <cmath>
#include <memory>
#include <limits>


using namespace vfm::fsm;

namespace vfm {
namespace mc {
namespace trajectory_generator {


struct Vector3
{
	double x;
	double y;
	double z;
};

/// \brief This generates an OSC for simulation file based on the vehicle trajectories from an interpreted MC Trace.
class OSCgenerator : public Failable
{
public:
	OSCgenerator(
		const MCinterpretedTrace interpreted_trace) : Failable("Visu_OSC"),
		m_interpreted_trace(interpreted_trace)
	{};

	// Generate the final OSC file
	std::string generate(std::string scenario_name, bool control_ego, std::string prerequisites = "");
	std::string generate_as_csv();

private:
	std::string getVechicleDefinitions(std::string indent_prefix) const;
	std::string getPositionAndSpeedNodes(std::string indent_prefix) const;
	std::string getPositionAndSpeedNode(std::string indent_prefix, std::string vehicle_name, const FullTrajectory& trajectory) const;
	std::string getTrajectoryDefinitions(std::string indent_prefix, bool control_ego) const;
	std::string getTrajectoryDefinition(std::string indent_prefix, FullTrajectory trajectory, std::string vehicle_name, bool wrap_in_osc_node) const;
	std::string getTrajectoryReplayNodes(std::string indent_prefix, bool control_ego) const;
	std::string getEgoManeuvers(std::string indent_prefix, bool control_ego) const;
	
	void addVehicleTrajectoryFollower(std::stringstream& osc_ss, FullTrajectory trajectory, std::string vehicle_name, bool wrap_in_osc_node);
	void addIndented(std::stringstream& string_stream, std::string text, int additional_indentation = 0);
	std::string getIndent(int additional_indentation = 0);

	const MCinterpretedTrace m_interpreted_trace;
	int m_current_indentations{ 0 };
};

/// @brief Class to generate a VTD scenario from a MC CEX trace
class VTDgenerator : public Failable
{
public:
	VTDgenerator(const MCinterpretedTrace interpreted_trace) : Failable("Visu_VTD"),
		m_interpreted_trace(interpreted_trace)
	{};

	std::string generate();

private:
	std::string generatePlayers();
	std::string generatePlayer(std::string name, double speed, Vector3 pos);

	std::string generatePlayerActions();
	std::string generatePlayerAction(std::string name);
	std::string generateSpeedChangeAction(std::string name, double rate, double target, double time);

	std::string generateMovingObjectsControl();
	std::string generatePolylinePathShape(std::string name, std::vector<Vector3> waypoints);

	const MCinterpretedTrace m_interpreted_trace;

};


/// \brief This generates GIFs and images of the vehicle trajectories from an interpreted MC Trace.
class LiveSimGenerator : public Failable
{
public:
	LiveSimGenerator(
			const MCinterpretedTrace vehicles_trajectory) : Failable("Visu_GIF"),
		m_trajectory_provider(vehicles_trajectory)
	{};

	enum LiveSimType : int {
		constant_image_output = 1,
		incremental_image_output = 2,
		gif_animation = 4,
		cockpit = 8,
		birdseye = 16,
		always_paint_arrows = 32
	};


   /// <summary>
   /// Generates a road graph that contains only the topology of the road,
   /// for conveniece, a dummy EGO is also placed at Sec. 0.
   /// </summary>
   std::shared_ptr<RoadGraph> getRoadGraphTopologyFrom(const MCTrace& trace);
   
   void equipRoadGraphWithCars(const std::shared_ptr<RoadGraph> r, const size_t trajectory_index, const double x_scaling);

	/// \brief So far, "live" simulation means creating image files on the fly
	/// while a smart image viewer such as Sumatra PDF (can also handle PNGs) can be used
	/// to show the sim live. Possible image formates are currently: png, jpg, bmp, ppm.
	void generate(
		const std::string& base_output_name,
		const std::set<int>& agents_to_draw_arrows_for,
      const std::string& stage_name,
      const MCTrace& trace,
		const LiveSimType visu_type = static_cast<LiveSimType>(LiveSimType::constant_image_output | LiveSimType::birdseye),
      const std::vector<vfm::OutputType> single_images_output_types = { OutputType::png },
		const double x_scaling = 1.0,
		const double time_factor = 1.0,
		const long sleep_for_ms = 100);

private:
	const MCinterpretedTrace m_trajectory_provider;

	class GifRecorder
	{
	public:
		GifRecorder(std::string name, bool active) :
			m_name(name), m_active(active) {};

		void addImage(std::shared_ptr<Image> img, double frame_duration);
		void finish();

	private:
		const bool m_active;
		const std::string m_name;
		bool m_is_initialized{ false };
		GifWriter m_gif_writer{};
	};

	void paintVarBox(Image& img, DataPack& data, std::vector<PainterVariableDescription>& variables_to_be_painted);

	std::shared_ptr<Image> updateOutputImages(
		Env2D& env,
		const LiveSimType visu_type,
      const std::vector<vfm::OutputType> single_images_output_types,
      const std::string image_file_output,
		ExtraVehicleArgs var_vals,
		const DataPackPtr data,
		const DataPackPtr future_data,
		const std::shared_ptr<std::vector<PainterVariableDescription>> variables_to_be_painted,
		const std::set<int>& agents_to_draw_arrows_for,
      const std::shared_ptr<RoadGraph> road_graph);
};


} // trajectory_generator
} // mc
} // vfm
