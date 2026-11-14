#include "testing/interactive_testing.h"
#include "static_helper.h"
#include "vfmacro/script.h"

#include <gtest/gtest.h>
#include <filesystem>

namespace fs = std::filesystem;

class UcdJunctionInputTests : public ::testing::Test {
protected:
   fs::path original_directory{};
   fs::path workspace{};

   void SetUp() override
   {
      original_directory = fs::current_path();
      workspace = original_directory / "junction_input_test_workspace";
      fs::remove_all(workspace);
      fs::create_directories(workspace / "morty");
      fs::create_directories(workspace / "generated_junction");
      fs::current_path(workspace);
      vfm::test::setMortyLiveMode(true);

      const nlohmann::json config{
         { "#TEMPLATE", {
            { "UCD_CONFIG_PRIOS", "_junction;" },
            { "UCD_JUNCTION", true },
            { "UCD_USE_SHORTCUT", false },
            { "FIXED_SECTION_IDs", "@(0)@@(1)@@(2)@@(3)@" },
            { "FIXED_SECTION_SOURCE_Xs", "@(-100)@@(0)@@(0)@@(0)@" },
            { "FIXED_SECTION_SOURCE_Ys", "@(-2)@@(-2)@@(98)@@(-2)@" },
            { "FIXED_SECTION_ANGLEs", "@(0)@@(0)@@(270)@@(270)@" },
            { "FIXED_SECTION_LENGTHs", "@(100)@@(100)@@(100)@@(100)@" },
            { "_GENERATED_PATH", "./generated" },
            { "_EXTERNAL_PATH", "./external" },
            { "_ENVMODEL_PATH", "." },
         } },
         { "_junction", { { "NONEGOS", 2 } } },
      };
      vfm::StaticHelper::writeTextToFile(config.dump(), "morty/envmodel_config.tpl.json");
      vfm::StaticHelper::writeTextToFile(config.dump(), "morty/envmodel_config.json");
      vfm::StaticHelper::writeTextToFile(
         "MODULE main\n--EO-SPEC-STUFF\n",
         "generated_junction/main.smv");
      vfm::StaticHelper::writeTextToFile("", "generated_junction/debug_trace_array.txt");

      vfm::ScaleDescription scaling;
      scaling.setTimeScalingFactor(2);
      scaling.setDistanceScalingFactor(4);
      scaling.setGeometryScalingFactor(1);
      vfm::StaticHelper::writeTextToFile(
         scaling.serialize(),
         "generated_junction/scaling_info.txt");
   }

   std::string junctionTrace() const
   {
      std::string trace{ "Trace Description: junction conversion test\nTrace Type: Counterexample\n" };
      for (int step = 0; step < 2; ++step) {
         trace += " -> State: 1." + std::to_string(step + 1) + " <-\n";
         if (step == 0) trace += " planner.time_scaling = 2000\n planner.distance_scaling = 4000\n";
         trace += " env.lane_width = 400\n env.num_lanes = 2\n env.num_technical_lanes = 12\n";

         trace += " env.veh___609___.on_normalized_lane = 17\n";
         trace += " env.veh___619___.on_normalized_lane = 17\n";
         trace += " env.veh___609___.on_straight_section = " + std::string(step == 0 ? "0\n" : "-1\n");
         trace += " env.veh___619___.on_straight_section = " + std::string(step == 0 ? "2\n" : "-1\n");
         trace += " env.veh___609___.traversion_from = " + std::string(step == 0 ? "-1\n" : "0\n");
         trace += " env.veh___609___.traversion_to = " + std::string(step == 0 ? "-1\n" : "1\n");
         trace += " env.veh___619___.traversion_from = " + std::string(step == 0 ? "-1\n" : "2\n");
         trace += " env.veh___619___.traversion_to = " + std::string(step == 0 ? "-1\n" : "3\n");
         trace += " env.veh___609___.abs_pos = " + std::string(step == 0 ? "200\n" : "32\n");
         trace += " env.veh___619___.abs_pos = " + std::string(step == 0 ? "200\n" : "32\n");
      }
      return trace;
   }

   void TearDown() override
   {
      vfm::test::setMortyLiveMode(false);
      fs::current_path(original_directory);
      fs::remove_all(workspace);
   }
};

TEST_F(UcdJunctionInputTests, MapsGlobalPoseAndVelocityOnBothStraightRoutes)
{
   vfm::test::prepareInputForMortyUCD(
      "1,-50,-2,5,0,0;1,-2,50,0,-5,4.71238898;",
      -0.5f, 2, 12, "AUTO");

   const auto updated = vfm::StaticHelper::readFile("generated_junction/main.smv");
   EXPECT_NE(updated.find("INIT env.veh___609___.abs_pos = 200;"), std::string::npos);
   EXPECT_NE(updated.find("INIT env.veh___609___.v = 10;"), std::string::npos);
   EXPECT_NE(updated.find("INIT env.veh___609___.on_straight_section = 0;"), std::string::npos);
   EXPECT_NE(updated.find("INIT env.veh___619___.abs_pos = 200;"), std::string::npos);
   EXPECT_NE(updated.find("INIT env.veh___619___.v = 10;"), std::string::npos);
   EXPECT_NE(updated.find("INIT env.veh___619___.on_straight_section = 2;"), std::string::npos);
}

TEST_F(UcdJunctionInputTests, ComputesLateralOffsetForFlexibleLaneCounts)
{
   vfm::test::prepareInputForMortyUCD(
      "1,-50,-4,5,0,0;1,-4,50,0,-5,4.71238898;",
      -0.5f, 3, 13, "AUTO");

   const auto updated = vfm::StaticHelper::readFile("generated_junction/main.smv");
   EXPECT_NE(updated.find("INIT env.veh___609___.abs_pos = 200;"), std::string::npos);
   EXPECT_NE(updated.find("INIT env.veh___609___.on_straight_section = 0;"), std::string::npos);
   EXPECT_NE(updated.find("INIT env.veh___619___.abs_pos = 200;"), std::string::npos);
   EXPECT_NE(updated.find("INIT env.veh___619___.on_straight_section = 2;"), std::string::npos);
}

TEST_F(UcdJunctionInputTests, RejectsHeadingThatWouldSelectTheOtherApproach)
{
   const auto original = vfm::StaticHelper::readFile("generated_junction/main.smv");
   vfm::test::prepareInputForMortyUCD(
      "1,-50,-2,5,0,1.57079633;1,-2,50,0,-5,4.71238898;",
      -0.5f, 2, 12, "AUTO");

   EXPECT_EQ(vfm::StaticHelper::readFile("generated_junction/main.smv"), original);
}

TEST_F(UcdJunctionInputTests, JunctionLaneMappingIgnoresHeadingAdaptation)
{
   vfm::test::prepareInputForMortyUCD(
      "1,-50,3,5,2,0;1,3,50,2,-5,4.71238898;",
      -0.5f, 2, 12, "AUTO");

   const auto updated = vfm::StaticHelper::readFile("generated_junction/main.smv");
   EXPECT_NE(updated.find("INIT !env.veh___609___.lane_b0;"), std::string::npos);
   EXPECT_NE(updated.find("INIT env.veh___609___.lane_b1;"), std::string::npos);
   EXPECT_NE(updated.find("INIT !env.veh___619___.lane_b0;"), std::string::npos);
   EXPECT_NE(updated.find("INIT env.veh___619___.lane_b1;"), std::string::npos);
}

TEST_F(UcdJunctionInputTests, RejectsVehiclesOutsideTheConfiguredLateralRoadBand)
{
   const auto original = vfm::StaticHelper::readFile("generated_junction/main.smv");
   vfm::test::prepareInputForMortyUCD(
      "1,-50,4.1,5,0,0;1,-2,50,0,-5,4.71238898;",
      0.0f, 2, 12, "AUTO");

   EXPECT_EQ(vfm::StaticHelper::readFile("generated_junction/main.smv"), original);
}

TEST_F(UcdJunctionInputTests, RejectsVehiclesBeyondFiniteLongitudinalRouteEndpoints)
{
   const auto original = vfm::StaticHelper::readFile("generated_junction/main.smv");
   for (const auto& position : { "-102,-2", "101.5,-2" }) {
      vfm::test::prepareInputForMortyUCD(
         "1," + std::string(position) + ",5,0,0;1,-2,50,0,-5,4.71238898;",
         0.0f, 2, 12, "AUTO");
      EXPECT_EQ(vfm::StaticHelper::readFile("generated_junction/main.smv"), original);
   }
   vfm::test::prepareInputForMortyUCD(
      "1,-50,-2,5,0,0;1,-2,-101.5,0,-5,4.71238898;",
      0.0f, 2, 12, "AUTO");
   EXPECT_EQ(vfm::StaticHelper::readFile("generated_junction/main.smv"), original);
}

TEST_F(UcdJunctionInputTests, AllowsEndpointToleranceAndJoinedConnectorPositions)
{
   vfm::test::prepareInputForMortyUCD(
      "1,100.5,-2,5,0,0;1,-2,0,0,-5,4.71238898;",
      0.0f, 2, 12, "AUTO");

   const auto updated = vfm::StaticHelper::readFile("generated_junction/main.smv");
   EXPECT_NE(updated.find("INIT env.veh___609___.abs_pos = 400;"), std::string::npos);
   EXPECT_NE(updated.find("INIT env.veh___619___.abs_pos = 400;"), std::string::npos);
}

TEST_F(UcdJunctionInputTests, ConvertsSectionAndJoinedConnectorTrajectoriesToGlobalCoordinates)
{
   vfm::StaticHelper::writeTextToFile(
      junctionTrace(),
      "generated_junction/debug_trace_array.txt");
   const auto result = vfm::macro::Script::processScript(
      "@{" + workspace.generic_string() + "/}@.stringToHeap[MY_PATH].nil"
      "@{morty/envmodel_config.tpl.json}@.extractVehPosFromNusmvFile[_junction]");

   EXPECT_NE(result.find("-50.000000,-2.000000;-2.000000,50.000000;"), std::string::npos);
   EXPECT_NE(result.find("8.000000,-2.000000;-2.000000,-8.000000;"), std::string::npos);
}
