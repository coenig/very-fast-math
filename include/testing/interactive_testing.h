//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2024 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file
#pragma once

#include "parser.h"
#include "cpp_parsing/cpp_parser.h"
#include "cpp_parsing/input_parser.h"
#include "fsm.h"
#include "model_checking/cex_processing/mc_visualization_launchers.h"
#include "json_parsing/json.hpp"

#include <vector>
#include <string>
#include <thread>


namespace vfm {
namespace test {

constexpr int MAX_THREADS{ 8 };

enum class EnvModelCachedMode {
   use_cached_if_available,
   always_regenerate
};

enum class MCExecutionType {
   parser,
   mc,
   cex,
   parser_and_mc,
   mc_and_cex,
   all
};

static const std::string ENVMODEL_TEMPLATE_FILENAME{ "EnvModel.tpl" };

using EnvModelConfig = std::map<std::string, std::string>;

void termnate();
int gui(int argc, char* argv[]);
void runInterpreter();
void loopKratos();
void quickGenerateEnvModels(const std::vector<int>& car_nums = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10 }, const std::vector<bool>& including_planner = { true, false });

// Original call from main:
//performFSMCodeGeneration(
//   //"../src/examples/dummy_planner/vfm-includes-planner.txt",                                                // DUMMY
//   "../src/examples/bp/prototype/motion_planning/rule_based_planning/vfm-includes-viper.txt",   // REGULAR
//   //"../src/examples/old/viper/vfm-includes-viper.txt",                                            // FROZEN
//   //"../src/examples/old/viper_simple/vfm-includes-viper.txt",                                     // SIMPLE
//   //"../src/examples/old/viper_simpler/vfm-includes-viper.txt",                                    // SIMPLER
//   //"../src/examples/old/viper/vfm-includes-viper_empty.txt",                                      // EMPTY

//   //"../src/examples/bp/prototype/motion_planning/rule_based_planning/old/vfm-includes-viper-env-model.txt",            // REGULAR-ENV
//   //"../src/examples/bp/prototype/motion_planning/rule_based_planning/old/vfm-includes-viper-env-model_two_agents.txt", // With two cars + ego
//   //"../src/examples/bp/prototype/motion_planning/rule_based_planning/old/vfm-includes-viper-env-model-simple.txt",     // SIMPLE-ENV
//   //"",                                                                                                                           // EMPTY-ENV
//   (NATIVE_SMV_ENV_MODEL_DENOTER_OPEN
//      //+ NATIVE_SMV_ENV_MODEL_DENOTER_ALWAYS_REGENERATE + ","
//      + "NONEGOS=2, NUMLANES=3, SEGMENTS=1, VIPER=false, LONGCONTROL=false, LEFTLC=true, RIGHTLC=false, FARMERGINGCARS=false, DEBUG=false"
//      + NATIVE_SMV_ENV_MODEL_DENOTER_CLOSE
//      + "../src/templates/EnvModel.tpl").c_str(), // Native SMV Env Model
//   "../examples/gp/planner.cpp",
//   "// #vfm-option[[ general_mode << debug ]]\n// #vfm-option[[ target_mc << kratos ]]" // Command line input.
//);
int artifactRun(int argc, char* argv[]);

void cameraRotationTester();
void runMCExperiments(const MCExecutionType type = MCExecutionType::all);

void convenienceArtifactRunHardcoded(
   const MCExecutionType exec,
   const std::string& target_directory = "../examples/gp",
   const std::string& json_config_path = "../src/templates/envmodel_config.json",
   const std::string& env_model_template_path = "../src/templates/" + ENVMODEL_TEMPLATE_FILENAME,
   const std::string& vfm_includes_file_path = "../src/examples/bp/prototype/motion_planning/rule_based_planning/vfm-includes-viper.txt",
   const std::string& cache_dir = "./tmp",
   const std::string& path_to_external_folder = "../external/");

std::map<std::string, std::string> retrieveEnvModelDefinitionFromJSON(const std::string json_file, const EnvModelCachedMode cached_mode);

std::string doParsingRun(
   const std::pair<std::string, std::string>& envmodel_definition,
   const std::string& root_dir,
   const std::string& envmodel_tpl,
   const std::string& planner_path,
   const std::string& target_dir,
   const std::string& cached_dir, // Can be empty if no caching desired.
   const std::string& gui_name
);

void extractInitValuesAca4_1();
void aca4_1Run();

std::shared_ptr<RoadGraph> paintExampleRoadGraphStrangeJunction(const bool write_to_files = true, const std::shared_ptr<RoadGraph> ego_section = nullptr);
std::shared_ptr<RoadGraph> paintExampleRoadGraphRoundabout(const bool write_to_files = true, const std::shared_ptr<RoadGraph> ego_section = nullptr);

extern "C" char* morty(const char* input);
} // test
} // vfm