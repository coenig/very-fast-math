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

bool onSegment(Vec2D p, Vec2D q, Vec2D r) {
   return (q.x <= std::max(p.x, r.x) && q.x >= std::min(p.x, r.x) &&
           q.y <= std::max(p.y, r.y) && q.y >= std::min(p.y, r.y));
}
bool doesLineIntersectSegment(Vec2D L1, Vec2D L2, Vec2D M1, Vec2D M2, Image& img) {
   float A1 = L2.y - L1.y;
   float B1 = L1.x - L2.x;
   float C1 = (L2.x * L1.y) - (L1.x * L2.y);
   float A2 = M2.y - M1.y;
   float B2 = M1.x - M2.x;
   float C2 = (M2.x * M1.y) - (M1.x * M2.y);

   float denominator = A1 * B2 - A2 * B1;

   if (denominator == 0) throw std::runtime_error("The lines are parallel and do not intersect.");

   float x = -(B1 * (-C2) - B2 * (-C1)) / denominator;
   float y = -(A2 * (-C1) - A1 * (-C2)) / denominator;

   img.circle(x, y, 10, GREEN);

   // Check if the intersection Vec2D lies on the line segment
   return onSegment(M1, {x, y}, M2);
}

int main(int argc, char* argv[])
{
   Vec2D p1 = { 0, 300 }, q1 = { 300, 100 }; // Line segment

   // Define the line using two Vec2Ds
   Vec2D L1 = { 10, 0 }; // First Vec2D on the line
   Vec2D L2 = { 200, 600 }; // Second Vec2D on the line

   Image img{ 1000, 1000 };

   if (doesLineIntersectSegment(L1, L2, p1, q1, img)) {
      std::cout << "The line intersects the line segment." << std::endl;
   }
   else {
      std::cout << "The line does not intersect the line segment." << std::endl;
   }

   img.drawPolygon({ p1, q1 }, WHITE);
   img.drawPolygon({ L1, L2 }, RED);
   img.store("test");

   return 0;
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
