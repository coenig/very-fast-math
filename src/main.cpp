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
   std::shared_ptr<HighwayTranslator> trans2{ std::make_shared<Plain2DTranslator>() };
   std::shared_ptr<HighwayTranslator> trans3{ std::make_shared<Plain3DTranslator>(false) };
   LaneSegment segment11{ 0, 0, 6 };
   LaneSegment segment12{ 20, 0, 4 };
   LaneSegment segment21{ 0, 0, 4 };
   LaneSegment segment22{ 30, 0, 6 };
   LaneSegment segment31{ 0, 2, 6 };
   LaneSegment segment32{ 25, 0, 4 };
   StraightRoadSection section1{ 4, 50 };
   StraightRoadSection section2{ 4, 60 };
   StraightRoadSection section3{ 4, 100 };
   section1.addLaneSegment(segment11);
   section1.addLaneSegment(segment12);
   section2.addLaneSegment(segment21);
   section2.addLaneSegment(segment22);
   section3.addLaneSegment(segment31);
   section3.addLaneSegment(segment32);
   HighwayImage image2{ 3000, 2000, trans2, 4 };
   image2.restartPDF();
   image2.fillImg(BLACK);
   HighwayImage image3{ 3000, 2000, trans3, 4 };
   image3.restartPDF();
   image3.fillImg(BLACK);
   image3.paintEarthAndSky({ 1500, 200 });
   std::shared_ptr<CarPars> ego = std::make_shared<CarPars>(2, 45, 13, HighwayImage::EGO_MOCK_ID);
   std::map<int, std::pair<float, float>> future_positions_of_others1{};
   std::map<int, std::pair<float, float>> future_positions_of_others2{};
   std::map<int, std::pair<float, float>> future_positions_of_others3{};
   CarParsVec others1{ { 1, -20, 1, 0 }, { 2, -50, 11, 1 } };
   CarParsVec others2{ { 2, 4, 3, 2 }, { 1, 22, 11, 3 } };
   CarParsVec others3{ { 2, 40, 3, 4 }, { 1, 22, 11, 5 } };
   section1.setEgo(ego);
   section1.setOthers(others1);
   section1.setFuturePositionsOfOthers(future_positions_of_others1);

   section2.setEgo(nullptr);
   section2.setOthers(others2);
   section2.setFuturePositionsOfOthers(future_positions_of_others2);

   section3.setEgo(nullptr);
   section3.setOthers(others3);
   section3.setFuturePositionsOfOthers(future_positions_of_others3);

   auto r1 = std::make_shared<RoadGraph>(1);
   auto r2 = std::make_shared<RoadGraph>(2);
   auto r3 = std::make_shared<RoadGraph>(3);
   r1->setMyRoad(section1);
   r1->setOriginPoint({ 0, 0 });
   r1->setAngle(0);
   r2->setMyRoad(section2);
   r2->setOriginPoint({ 65, 4 });
   r2->setAngle(3.1415 / 3);
   r3->setMyRoad(section3);
   r3->setOriginPoint({ r3->getMyRoad().getLength() + 65 + vfm::LANE_WIDTH, 3 * vfm::LANE_WIDTH / vfm::LANE_WIDTH });
   r3->setAngle(-3.1415);
   r1->addSuccessor(r2);
   r3->addSuccessor(r2);
   image2.paintRoadGraph(r1, { 1500, 200 });
   image3.paintRoadGraph(r1, { 1500, 200 });
   image2.store("test2D", OutputType::pdf);
   image3.store("test3D", OutputType::pdf);
   termnate();

   vfm::test::runTests();
   termnate();

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
