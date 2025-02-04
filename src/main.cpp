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
   const Vec2D dim3D{ 500, 120 };

   // Strange junction
   {
      std::shared_ptr<HighwayTranslator> trans2{ std::make_shared<Plain2DTranslator>() };
      std::shared_ptr<HighwayTranslator> trans3{ std::make_shared<Plain3DTranslator>(false) };
      HighwayImage image2{ 3000, 2000, trans2, 4 };
      HighwayImage image3{ 1500, 500, trans3, 4 };
      image2.restartPDF();
      image2.fillImg(BLACK);
      image3.restartPDF();
      image3.fillImg(BLACK);
      //image3.paintEarthAndSky({ 1500, 500 });
      LaneSegment segment11{ 0, 2, 6 };
      LaneSegment segment12{ 20, 0, 4 };
      LaneSegment segment21{ 0, 0, 4 };
      LaneSegment segment22{ 30, 2, 6 };
      LaneSegment segment31{ 0, 2, 6 };
      LaneSegment segment32{ 25, 0, 4 };
      LaneSegment segment41{ 0, 2, 6 };
      LaneSegment segment42{ 35, 0, 4 };
      StraightRoadSection sectiona1{ 4, 50 };
      StraightRoadSection sectiona2{ 4, 60 };
      StraightRoadSection sectiona3{ 4, 100 };
      StraightRoadSection sectiona4{ 4, 55 };
      sectiona1.addLaneSegment(segment11);
      sectiona1.addLaneSegment(segment12);
      sectiona2.addLaneSegment(segment21);
      sectiona2.addLaneSegment(segment22);
      sectiona3.addLaneSegment(segment31);
      sectiona3.addLaneSegment(segment32);
      sectiona4.addLaneSegment(segment41);
      sectiona4.addLaneSegment(segment42);
      std::shared_ptr<CarPars> egoa = std::make_shared<CarPars>(2, 40, 13, HighwayImage::EGO_MOCK_ID);
      std::map<int, std::pair<float, float>> future_positions_of_others1{};
      std::map<int, std::pair<float, float>> future_positions_of_others2{};
      std::map<int, std::pair<float, float>> future_positions_of_others3{};
      std::map<int, std::pair<float, float>> future_positions_of_others4{};
      CarParsVec othersa1{ { 1, -20, 1, 0 }, { 2, -50, 11, 1 } };
      CarParsVec othersa2{ { 2, 4, 3, 2 }, { 1, 22, 11, 3 } };
      CarParsVec othersa3{ { 2, 40, 3, 4 }, { 1, 22, 11, 5 } };
      CarParsVec othersa4{ { 1, 50, 3, 6 }, { 1, 22, 11, 6 } };
      sectiona1.setEgo(egoa);
      sectiona1.setOthers(othersa1);
      sectiona1.setFuturePositionsOfOthers(future_positions_of_others1);
      sectiona2.setEgo(nullptr);
      sectiona2.setOthers(othersa2);
      sectiona2.setFuturePositionsOfOthers(future_positions_of_others2);
      sectiona3.setEgo(nullptr);
      sectiona3.setOthers(othersa3);
      sectiona3.setFuturePositionsOfOthers(future_positions_of_others3);
      sectiona4.setEgo(nullptr);
      sectiona4.setOthers(othersa4);
      sectiona4.setFuturePositionsOfOthers(future_positions_of_others4);

      auto ra1 = std::make_shared<RoadGraph>(1);
      auto ra2 = std::make_shared<RoadGraph>(2);
      auto ra3 = std::make_shared<RoadGraph>(3);
      auto ra4 = std::make_shared<RoadGraph>(4);
      ra1->setMyRoad(sectiona1);
      ra1->setOriginPoint({ 0, 6 });
      ra1->setAngle(0);
      ra2->setMyRoad(sectiona2);
      ra2->setOriginPoint({ 75, 5 * vfm::LANE_WIDTH });
      ra2->setAngle(3.1415 / 2.1);
      ra3->setMyRoad(sectiona3);
      ra3->setOriginPoint({ ra3->getMyRoad().getLength() + 85 + vfm::LANE_WIDTH, 6 });
      ra3->setAngle(-3.1415);
      ra4->setMyRoad(sectiona4);
      ra4->setOriginPoint({ 65, -2.5 * vfm::LANE_WIDTH });
      ra4->setAngle(-3.1415 / 1.9);
      ra1->addSuccessor(ra2);
      ra3->addSuccessor(ra2);
      ra1->addSuccessor(ra4);
      ra3->addSuccessor(ra4);
      image2.paintRoadGraph(ra1, { 500, 60 }, 0, {}, true, 60, 60);
      image2.store("../examples/junction", OutputType::pdf);
      image2.store("../examples/junction", OutputType::png);
      image3.paintRoadGraph(ra1, dim3D);
      image3.store("../examples/junction_3d", OutputType::pdf);
      image3.store("../examples/junction_3d", OutputType::png);
      //termnate();
   }

   // Roundabout
   {
      std::shared_ptr<HighwayTranslator> trans2{ std::make_shared<Plain2DTranslator>() };
      std::shared_ptr<HighwayTranslator> trans3{ std::make_shared<Plain3DTranslator>(false) };
      HighwayImage image2{ 3000, 2000, trans2, 4 };
      HighwayImage image3{ 1500, 500, trans3, 4 };
      image2.restartPDF();
      image2.fillImg(BLACK);
      image3.restartPDF();
      image3.fillImg(BLACK);
      //image3.paintEarthAndSky({ 1500, 500 });

      static constexpr int lanes0{ 3 };
      static constexpr int lanes1{ 3 };
      static constexpr int lanes2{ 3 };
      static constexpr int lanes3{ 3 };
      static constexpr int lanes4{ 3 };
      static constexpr int lanes5{ 3 };
      static constexpr int lanes6{ 3 };
      static constexpr int lanes7{ 3 };

      LaneSegment segment0{ 0, 0, (lanes0 - 1) * 2 };
      LaneSegment segment1{ 0, 0, (lanes1 - 1) * 2 };
      LaneSegment segment2{ 0, 0, (lanes2 - 1) * 2 };
      LaneSegment segment3{ 0, 0, (lanes3 - 1) * 2 };
      LaneSegment segment4{ 0, 0, (lanes4 - 1) * 2 };
      LaneSegment segment5{ 0, 0, (lanes5 - 1) * 2 };
      LaneSegment segment6{ 0, 0, (lanes6 - 1) * 2 };
      LaneSegment segment7{ 0, 0, (lanes7 - 1) * 2 };
      StraightRoadSection section0{ lanes0, 50 };
      StraightRoadSection section1{ lanes1, 50 };
      StraightRoadSection section2{ lanes2, 50 };
      StraightRoadSection section3{ lanes3, 50 };
      StraightRoadSection section4{ lanes4, 50 };
      StraightRoadSection section5{ lanes5, 50 };
      StraightRoadSection section6{ lanes6, 50 };
      StraightRoadSection section7{ lanes7, 50 };
      section0.addLaneSegment(segment0);
      section1.addLaneSegment(segment1);
      section2.addLaneSegment(segment2);
      section3.addLaneSegment(segment3);
      section4.addLaneSegment(segment4);
      section5.addLaneSegment(segment5);
      section6.addLaneSegment(segment6);
      section7.addLaneSegment(segment7);
      std::shared_ptr<CarPars> ego = std::make_shared<CarPars>(2, 40, 0, HighwayImage::EGO_MOCK_ID);
      std::map<int, std::pair<float, float>> future_positions_of_others{};
      CarParsVec others0{ { 0, 10, 0, 0 } };
      CarParsVec others1{ { 0, 40, 0, 1 } };
      CarParsVec others2{};
      CarParsVec others3{};
      CarParsVec others4{};
      CarParsVec others5{};
      CarParsVec others6{};
      CarParsVec others7{};
      section0.setEgo(nullptr);
      section0.setOthers(others0);
      section0.setFuturePositionsOfOthers(future_positions_of_others);
      section1.setEgo(nullptr);
      section1.setOthers(others1);
      section1.setFuturePositionsOfOthers(future_positions_of_others);
      section2.setEgo(nullptr);
      section2.setOthers(others2);
      section2.setFuturePositionsOfOthers(future_positions_of_others);
      section3.setEgo(nullptr);
      section3.setOthers(others3);
      section3.setFuturePositionsOfOthers(future_positions_of_others);
      section4.setEgo(nullptr);
      section4.setOthers(others4);
      section4.setFuturePositionsOfOthers(future_positions_of_others);
      section5.setEgo(nullptr);
      section5.setOthers(others5);
      section5.setFuturePositionsOfOthers(future_positions_of_others);
      section6.setEgo(nullptr);
      section6.setOthers(others6);
      section6.setFuturePositionsOfOthers(future_positions_of_others);
      section7.setEgo(ego);
      section7.setOthers(others7);
      section7.setFuturePositionsOfOthers(future_positions_of_others);

      auto r0 = std::make_shared<RoadGraph>(0);
      auto r1 = std::make_shared<RoadGraph>(1);
      auto r2 = std::make_shared<RoadGraph>(2);
      auto r3 = std::make_shared<RoadGraph>(3);
      auto r4 = std::make_shared<RoadGraph>(4);
      auto r5 = std::make_shared<RoadGraph>(5);
      auto r6 = std::make_shared<RoadGraph>(6);
      auto r7 = std::make_shared<RoadGraph>(7);
      r0->setMyRoad(section0);
      r1->setMyRoad(section1);
      r2->setMyRoad(section2);
      r3->setMyRoad(section3);
      r4->setMyRoad(section4);
      r5->setMyRoad(section5);
      r6->setMyRoad(section6);
      r7->setMyRoad(section7);
      constexpr float rad{ 60.0f };
      const Vec2D mid{ 25, rad };
      Vec2D p0{ 0, 0 };
      Vec2D p1{ 0, 0 };
      Vec2D p2{ 0, 0 };
      Vec2D p3{ 0, 0 };
      Vec2D p4{ 0, 0 };
      Vec2D p5{ 0, 0 };
      const float a0{ 0 };
      const float a1{ 3.1415 / 3 };
      const float a2{ 2 * 3.1415 / 3 };
      const float a3{ 3.1415 };
      const float a4{ 4 * 3.1415 / 3 };
      const float a5{ 5 * 3.1415 / 3 };
      p1.rotate(a1, mid);
      p2.rotate(a2, mid);
      p3.rotate(a3, mid);
      p4.rotate(a4, mid);
      p5.rotate(a5, mid);

      r0->setOriginPoint(p0);
      r0->setAngle(a0);
      r1->setOriginPoint(p1);
      r1->setAngle(a1);
      r2->setOriginPoint(p2);
      r2->setAngle(a2);
      r3->setOriginPoint(p3);
      r3->setAngle(a3);
      r4->setOriginPoint(p4);
      r4->setAngle(a4);
      r5->setOriginPoint(p5);
      r5->setAngle(a5);

      r6->setOriginPoint({ 120, 12.5 * 3.75 });
      r6->setAngle(0);
      r7->setOriginPoint({ 170, 19 * 3.75 });
      r7->setAngle(-3.1415);
      r0->addSuccessor(r1);
      r1->addSuccessor(r2);
      r2->addSuccessor(r3);
      r3->addSuccessor(r4);
      r4->addSuccessor(r5);
      r5->addSuccessor(r0);
      r7->addSuccessor(r2);
      r1->addSuccessor(r6);
      image2.paintRoadGraph(r1, { 500, 60 }, 0, {}, true, 10, 50);
      image2.store("../examples/roundabout", OutputType::pdf);
      image2.store("../examples/roundabout", OutputType::png);
      image3.paintRoadGraph(r1, dim3D, 0, {}, true, 0, 0);
      image3.store("../examples/roundabout_3d", OutputType::pdf);
      image3.store("../examples/roundabout_3d", OutputType::png);
      termnate();
   }

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
