//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file

#include "testing/test_functions.h"
#include "testing/interactive_testing.h"
#include "simplification/simplification_function.h"
#include "model_checking/cex_processing/mc_visualization_launchers.h"
#include "cpp_parsing/cpp_parser.h"
#include "cpp_parsing/options.h"
#include "model_checking/counterexample_replay.h"
#include "vfmacro/script.h"
#include "gui/gui.h"
// #include "examples/fct_enumdefinitions.h" // TODO: This does not work on Linux (needed for replayCounterExample).

using namespace vfm;
using namespace test;
using namespace mc::trajectory_generator;

int main(int argc, char* argv[])
{
   test::convenienceArtifactRunHardcoded(test::MCExecutionType::cex, "../examples/gp_config_lanes=4_nonegos=1");
   termnate();

   //vfm::test::runTests();
   //termnate();

   //runInterpreter();
   //termnate();

   MCScene mc_scene{ argc, argv };
   return mc_scene.getFlRunInfo();
   termnate();

   //vfm::test::paintExampleRoadGraphs();
   //termnate();

   //aca4_1Run();
   //termnate();

   //return artifactRun(argc, argv);
   //termnate();

   //convenienceArtifactRunHardcoded(MCExecutionType::all);
   //termnate();

   //vfm::test::runMCExperiments(MCExecutionType::all);
   //termnate();

   //runInterpreter();
   //termnate();

   //loopKratos();
   //termnate();

   //quickGenerateEnvModels();
   //termnate();

   //processCEX("../examples/env_model_devel/generator/2024_01_09_Artificial2", CexType(CexTypeEnum::smv).getEnumAsString().c_str(), true, true);
   //termnate();

   //simplification::CodeGenerator::deleteAndWriteSimplificationRulesToFile(simplification::CodeGenerationMode::positive, "../include/simplification/simplification_pos.h");     // Normal mode.
   //simplification::CodeGenerator::deleteAndWriteSimplificationRulesToFile(simplification::CodeGenerationMode::negative, "../include/simplification/simplification.h"); // Normal mode (negative).
   //simplification::CodeGenerator::deleteAndWriteSimplificationRulesToFile(simplification::CodeGenerationMode::negative, "../include/model_checking/simplification.h", nullptr, true); // MC mode (negative).
   //termnate();

   //std::string dir{"../examples/mc/G1-demo_FSM/"};
   //vfm::replayCounterExample(
   //   dir + "FCT_ConditionsOK_counterexample_demo.txt",
   //   dir + "FCT_FSM_init.vfm",
   //   dir + "FCT_FSM.vfm",
   //   dir + "golf_steering_wheel.ppm",
   //   dir + "G1-demo-FSM-state-viz.png",
   //   dir + "G1-demo-FSM.dot",
   //   dir + "G1-demo-FSM_data.dot",
   //   Fct::associateVariables,
   //   vfm::defaultPaintFunction,
   //   Fct::my_vars_for_painter,
   //   Fct::my_buttons,
   //   "pdf",
   //   true);
   //termnate();

   //return 0;
}
