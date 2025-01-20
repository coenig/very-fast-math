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
   const int num_lanes{ 2 };
   vfm::Vec2D A{ 100, 100 };
   vfm::Vec2D C{ 750, 400 };
   vfm::Vec2D B{ 200, 900 };

   vfm::Image img{ 1000, 1000 };
   img.fillImg(DARK_GREEN);

   Pol2D clothoid{};
   clothoid = clothoid.bezier(A, C, C, B, 0.001);
   clothoid = clothoid.normalize();
   vfm::Pol2D arrow{};
   vfm::Pol2D arrow2{};
   arrow.createArrow(clothoid, 100);
   img.fillPolygon(arrow, vfm::GREY);

   for (int i = 0; i < num_lanes - 1; i++) {
      auto segments = Pol2D::gestrichelterPfeil(clothoid, 3, 3);
      img.drawPolygons(segments, { { nullptr, std::make_shared<Color>(WHITE)}, {nullptr, nullptr} });
   }

   img.circle(A.x, A.y, 20, vfm::ORANGE);
   img.circle(C.x, C.y, 20, vfm::RED);
   img.circle(B.x, B.y, 20, vfm::ORANGE);
   img.store("test");
   termnate();

   //std::shared_ptr<HighwayTranslator> trans{ std::make_shared<Plain2DTranslator>() };
   //LaneSegment segment1{ 0, 0, 6 };
   //LaneSegment segment2{ 20, 2, 4 };
   //StraightRoadSection section1{ 4 };
   //section1.addLaneSegment(segment1);
   //section1.addLaneSegment(segment2);
   //HighwayImage image{ 1500, 200, trans, 4 };
   //std::shared_ptr<CarPars> ego = std::make_shared<CarPars>(2, 0, 13, HighwayImage::EGO_MOCK_ID);
   //CarParsVec others{ { 3, -10, 3, 0 }, { 1, 50, 11, 1 } };
   //std::map<int, std::pair<float, float>> future_positions_of_others{};
   //section1.setEgo(ego);
   //section1.setOthers(others);
   //section1.setFuturePositionsOfOthers(future_positions_of_others);

   //auto r = std::make_shared<RoadGraph>(0);
   //r->setMyRoad(section1);
   //image.paintRoadGraph(r);
   //image.store("test");
   //termnate();

   //vfm::test::runTests();
   //termnate();

   //runInterpreter();
   //termnate();

   MCScene mc_scene{ argc, argv };
   return mc_scene.getFlRunInfo();
   termnate();

   aca4_1Run();
   termnate();

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
