//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file
#pragma once

#include "static_helper.h"

#include <filesystem>

#include "model_checking/cex_processing/mc_trajectory_generator.h"
#include "model_checking/cex_processing/mc_trajectory_visualizers.h"

namespace vfm {
namespace mc {
namespace trajectory_generator {

enum class CexTypeEnum { msatic, smv, kratos, invalid };

const std::map<CexTypeEnum, std::string> cex_type_map{
   { CexTypeEnum::msatic, "msatic" },
   { CexTypeEnum::smv, "smv" },
   { CexTypeEnum::kratos, "kratos" },
   { CexTypeEnum::invalid, "invalid" }
};

class CexType : public fsm::EnumWrapper<CexTypeEnum>
{
public:
   explicit CexType(const CexTypeEnum& enum_val) : EnumWrapper("CexType", cex_type_map, enum_val) {}
   explicit CexType(const int enum_val_as_num) : EnumWrapper("CexType", cex_type_map, enum_val_as_num) {}
   explicit CexType(const std::string& enum_val_as_string) : EnumWrapper("CexType", cex_type_map, enum_val_as_string) {}
   explicit CexType() : EnumWrapper("CexType", cex_type_map, CexTypeEnum::invalid) {}
};

struct VisualizationScales
{
public:
   double x_scaling = 1.0; // stretching along the X axis
   double duration_scale = 1.0; // Global duration factor for all output (incl. osc)
   double gif_duration_scale = 1.0; // Scaling the animation of GIF outputs
   int frames_per_second_gif = 15;
   int frames_per_second_osc = 5;
};

static const std::pair<std::string, std::string> TESTCASE_MODE_PREVIEW{ "preview", "preview" };
static const std::pair<std::string, std::string> TESTCASE_MODE_PREVIEW_2{ "preview2", "preview 2" };
static const std::pair<std::string, std::string> TESTCASE_MODE_CEX_FULL{ "cex-full", "full (1/7)" };
static const std::pair<std::string, std::string> TESTCASE_MODE_CEX_BIRDSEYE{ "cex-birdseye", "birdseye (2/7)" };
static const std::pair<std::string, std::string> TESTCASE_MODE_CEX_COCKPIT_ONLY{ "cex-cockpit-only", "cockpit (3/7)" };
static const std::pair<std::string, std::string> TESTCASE_MODE_CEX_SMOOTH_WITH_ARROWS_FULL{ "cex-smooth-with-arrows-full", "smooth-with-arrows (6/7)" };
static const std::pair<std::string, std::string> TESTCASE_MODE_CEX_SMOOTH_WITH_ARROWS_BIRDSEYE{ "cex-smooth-with-arrows-birdseye", "smooth-with-arrows-birdseye (7/7)" };
static const std::pair<std::string, std::string> TESTCASE_MODE_CEX_SMOOTH_FULL{ "cex-smooth-full", "full-smooth (4/7)" };
static const std::pair<std::string, std::string> TESTCASE_MODE_CEX_SMOOTH_BIRDSEYE{ "cex-smooth-birdseye", "birdseye-smooth (5/7)" };

static const std::map<std::string, std::string> ALL_TESTCASE_MODES{
   TESTCASE_MODE_PREVIEW,
   TESTCASE_MODE_PREVIEW_2,
   TESTCASE_MODE_CEX_FULL,
   TESTCASE_MODE_CEX_BIRDSEYE,
   TESTCASE_MODE_CEX_COCKPIT_ONLY,
   TESTCASE_MODE_CEX_SMOOTH_WITH_ARROWS_FULL,
   TESTCASE_MODE_CEX_SMOOTH_WITH_ARROWS_BIRDSEYE,
   TESTCASE_MODE_CEX_SMOOTH_FULL,
   TESTCASE_MODE_CEX_SMOOTH_BIRDSEYE
};

class VisualizationLaunchers
{
public:
   static constexpr auto SIM_TYPE_REGULAR = static_cast<LiveSimGenerator::LiveSimType>(LiveSimGenerator::LiveSimType::gif_animation
      | LiveSimGenerator::LiveSimType::birdseye
      | LiveSimGenerator::LiveSimType::cockpit // Remove cockpit for far faster rendering!
      | LiveSimGenerator::LiveSimType::incremental_image_output
      );

   static constexpr auto SIM_TYPE_SMOOTH = static_cast<LiveSimGenerator::LiveSimType>(LiveSimGenerator::LiveSimType::gif_animation
      | LiveSimGenerator::LiveSimType::birdseye
      | LiveSimGenerator::LiveSimType::cockpit // Remove cockpit for far faster rendering!
      );

   static constexpr auto SIM_TYPE_SMOOTH_WITH_ARROWS = static_cast<LiveSimGenerator::LiveSimType>(LiveSimGenerator::LiveSimType::gif_animation
      | LiveSimGenerator::LiveSimType::birdseye
      | LiveSimGenerator::LiveSimType::cockpit // Remove cockpit for far faster rendering!
      | LiveSimGenerator::LiveSimType::always_paint_arrows // Can make it quite crowded in smooth animation.
      );

   static constexpr auto SIM_TYPE_SMOOTH_COCKPIT = static_cast<LiveSimGenerator::LiveSimType>(LiveSimGenerator::LiveSimType::gif_animation
      | LiveSimGenerator::LiveSimType::cockpit
      | LiveSimGenerator::LiveSimType::always_paint_arrows
      );

   static constexpr auto SIM_TYPE_REGULAR_BIRDSEYE_ONLY = static_cast<LiveSimGenerator::LiveSimType>(LiveSimGenerator::LiveSimType::gif_animation
      | LiveSimGenerator::LiveSimType::birdseye
      | LiveSimGenerator::LiveSimType::incremental_image_output
      );

   static constexpr auto SIM_TYPE_REGULAR_BIRDSEYE_ONLY_NO_GIF = static_cast<LiveSimGenerator::LiveSimType>(
      LiveSimGenerator::LiveSimType::birdseye
      | LiveSimGenerator::LiveSimType::incremental_image_output
      );

   static constexpr auto SIM_TYPE_SMOOTH_BIRDSEYE_ONLY = static_cast<LiveSimGenerator::LiveSimType>(LiveSimGenerator::LiveSimType::gif_animation
      | LiveSimGenerator::LiveSimType::birdseye
      );

   static constexpr auto SIM_TYPE_SMOOTH_WITH_ARROWS_BIRDSEYE_ONLY = static_cast<LiveSimGenerator::LiveSimType>(LiveSimGenerator::LiveSimType::gif_animation
      | LiveSimGenerator::LiveSimType::birdseye
      | LiveSimGenerator::LiveSimType::always_paint_arrows // Can make it quite crowded in smooth animation.
      );

   static bool quickGenerateGIFs(
      const std::set<int>& cex_nums_to_generate, // In case cex file contains more than 1 CEX, numbers to pick.
      const std::string& path_cropped,
      const std::string& file_name_without_txt_extension,
      const CexType& cex_type,
      const std::map<std::string, std::string> modes,
      const std::set<int>& agents_to_draw_arrows_for = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10 }
   );

   static std::string toLimStr(double value, int precision = 2)
   {
      std::ostringstream out;
      out.precision(precision);
      out << std::fixed << value;
      return out.str();
   }

   // Provide frames_per_second <= 0 to disable interpolation.
   static bool interpretAndGenerate(
      const MCTrace& trace,
      const std::string& out_pathname_raw,
      const std::string& out_filename_raw,
      const LiveSimGenerator::LiveSimType sim_type,
      const std::set<int>& agents_to_draw_arrows_for,
      const VisualizationScales& settings,
      const std::string& stage_name,
      const bool generate_osc = true,
      const bool generate_gif = true);

private:
   static std::string writeProseTrafficScene(const MCTrace trace);

   static std::string writeProseSingleStep(
      const CarParsVec& others_past_vec,
      const CarParsVec& others_current_vec,
      const CarParsVec& others_future_vec,
      const CarPars& past_ego,
      const CarPars& ego,
      const int seconds);
};
}
}

extern "C" bool processCEX(
   const char* path_cropped,
   const char* cex_type,
   const bool include_smooth,
   const bool with_smooth_arrows);
}
