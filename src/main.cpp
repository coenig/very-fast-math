//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file

#include "testing/test_functions.h"
#include "testing/interactive_testing.h"
#include "simplification/code_block.h"
#include "model_checking/cex_processing/mc_visualization_launchers.h"
#include "cpp_parsing/cpp_parser.h"
#include "cpp_parsing/options.h"
#include "model_checking/counterexample_replay.h"
#include "vfmacro/script.h"
#include "gui/gui.h"
#include "simulation/road_graph.h"
#include "geometry/bezier_functions.h"
// #include "examples/fct_enumdefinitions.h" // TODO: This does not work on Linux (needed for replayCounterExample).

using namespace vfm;
using namespace test;
using namespace xml;
using namespace mc::trajectory_generator;


int main(int argc, char* argv[])
{
   //for (int i = 0; i < 100; i++) {
   //   Image img{ 1500, 1500 };
   //   img.fillImg(BROWN);
   //   Vec2D p0{ (float)i, (float)(i % 5) }, p1{ (float)(i % 7), (float)(i % 13) }, p2{ (float)(-i), (float)(i % 19) }, p3{ (float)(i % 23), (float)(i % 29) };
   //   float hull{ p0.distanceSquare(p1) + p1.distanceSquare(p2) + p2.distanceSquare(p3) };
   //   float line{ p0.distanceSquare(p3) };
   //   auto mid = bezier::cubicBezier(0.5, p0, p1, p2, p3);
   //   float mid_line{ p0.distance(mid) + mid.distance(p3) };
   //   std::cout << "ExactS: " << std::pow(bezier::arcLength(1, p0, p1, p2, p3), 2) << std::endl;
   //   std::cout << "Exact : " << bezier::arcLength(1, p0, p1, p2, p3) << std::endl;
   //   std::cout << "Hull  : " << hull << std::endl;
   //   std::cout << "Line  : " << line << std::endl;
   //   std::cout << "Avg   : " << (line + hull) / 2 << std::endl;
   //   std::cout << "Mid   : " << mid_line << std::endl;
   //   Pol2D p{};
   //   p.bezier(p0, p1, p2, p3);
   //   Pol2D arrow{};
   //   arrow.createArrow(p, 0.1);
   //   arrow.scale({ 50, 50 });
   //   arrow.translate({ 700, 700 });
   //   img.fillPolygon(arrow, BLUE);
   //   img.writeAsciiText(100, 100, std::to_string(bezier::arcLength(1, p0, p1, p2, p3)));
   //   img.writeAsciiText(100, 130, std::to_string(mid_line));
   //   img.store("bezier");
   //   std::cin.get();
   //   //std::cout << "DIFF: " << (line + hull) / 2 + std::pow(bezier::arcLength(1, p0, p1, p2, p3), 2) << std::endl;
   //}
   //termnate();

   auto traces = StaticHelper::extractMCTracesFromNusmvFile("../examples/gp_config_sections=5/debug_trace_array.txt");
   std::shared_ptr<RoadGraph> r{ mc::trajectory_generator::VisualizationLaunchers::getRoadGraphFrom(traces.at(0)) };
   std::cout << r->generateOSM()->serializeBlock() << std::endl;
   
   StaticHelper::writeTextToFile(r->generateOSM()->serializeBlock(), "test.osm");

   HighwayImage img{ 1600, 2600, std::make_shared<Plain2DTranslator>(), 4 };

   const Rec2D bounding_box{ r->getBoundingBox() };
   const float offset_x{ -bounding_box.upper_left_.x + 15 };
   const float offset_y{ -bounding_box.upper_left_.y + 15 };

   img.startOrKeepUpPDF();
   img.fillImg(BROWN);
   img.paintRoadGraph(r, { 500, 60 }, {}, true, offset_x, offset_y);
   img.store("test", OutputType::pdf);
   termnate();

   //mc::trajectory_generator::VisualizationLaunchers::quickGenerateGIFs(
   //   { 
   //      0, 1, 2, 3, 4, 5, 6, 7, 8, 9,
   //      10, 11, 12, 13, 14, 15, 16, 17, 18, 19,
   //      20, 21, 22, 23, 24, 25, 26, 27, 28, 29,
   //      30, 31, 32, 33, 34, 35, 36, 37, 38, 39,
   //      40, 41, 42, 43, 44, 45, 46, 47, 48, 49,
   //   }, 
   //   "../examples/gp_config_lanes=2_nonegos=1_segments=1", 
   //   "debug_trace_array",
   //   CexTypeEnum::smv,
   //   false, false, false);
   //termnate();

   //vfm::test::runTests();
   //termnate();

   //runInterpreter();
   //termnate();

   //aca4_1Run();
   //termnate();

   if (argc == 1) {
      std::cout << "Found no command line arguments. Creating artificial ones to trigger M²oRTy GUI.\nNote: Make sure to run from *vfm*/bin folder.\n";

      char* argvv[7];
      argc = 7;

      argvv[0] = argv[0];
      argvv[1] = "--mode";
      argvv[2] = "gui";
      argvv[3] = "--path-to-nuxmv";
#ifdef _WIN32
      argvv[4] = "../external/win32/nuXmv.exe";
#elif __linux__
      argvv[4] = "../external/linux64/nuXmv";
#else
      argvv[4] = "../external/linux64/nuXmv";
      std::cout << "Unknown OS, trying linux, but no guarantee.\n"
#endif
      argvv[5] = "--template-dir";
      argvv[6] = "../src/templates";

      return artifactRun(argc, argvv);
   }
   else {
      return artifactRun(argc, argv);
   }


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

   //code_block::CodeGenerator::deleteAndWriteSimplificationRulesToFile(code_block::CodeGenerationMode::positive, "../include/simplification/simplification_pos.h");     // Normal mode.
   //code_block::CodeGenerator::deleteAndWriteSimplificationRulesToFile(code_block::CodeGenerationMode::negative, "../include/simplification/simplification.h"); // Normal mode (negative).
   //code_block::CodeGenerator::deleteAndWriteSimplificationRulesToFile(code_block::CodeGenerationMode::negative, "../include/model_checking/simplification.h", nullptr, true); // MC mode (negative).
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
