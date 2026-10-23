#include "static_helper.h"
#include "geometry/bezier_functions.h"
#include "model_checking/mc_types.h"
#include "testing/test_functions.h"
#include "vfmacro/script.h"
#include <gtest/gtest.h>
#include <cmath>
#include <cstdint>
#include <filesystem>

namespace fs = std::filesystem;

// 64-bit FNV-1a hash, used to detect unexpected changes in generated artifacts.
static uint64_t fnv1a64(const std::string& s) {
    uint64_t h{ 0xcbf29ce484222325ULL };
    for (const unsigned char c : s) {
        h ^= c;
        h *= 0x100000001b3ULL;
    }
    return h;
}

// This is the big block of vfm system tests, all executed as one big gtest blob.
// TODO: distribute into own individual tests.
TEST(IntegrationTests, GeneralTerms) { 
    auto results = vfm::test::runTests();

    for (const auto& result : results) {
        ASSERT_TRUE(result.second) << ("Failed: "+result.first);
    }
}

TEST(VfmacroTests, NilYieldsEmptyString) {
    const std::string result = vfm::macro::Script::processScript("@{}@.nil");
    ASSERT_EQ(result, "");
}

TEST(GeometryScalingTests, LegacyMetadataDefaultsGeometryToOne) {
    const vfm::ScaleDescription description(R"(
-- TIMESCALING((((2))))GNILACSEMIT
-- DISTANCESCALING((((8))))GNILACSECNATSID
-- SCALING DESCRIPTIONS
-- section_0.source.x,none
-- section_0_end,distance
-- EO SCALING DESCRIPTIONS
)");

    EXPECT_FLOAT_EQ(description.getTimeScalingFactor(), 2);
    EXPECT_FLOAT_EQ(description.getDistanceScalingFactor(), 8);
    EXPECT_FLOAT_EQ(description.getGeometryScalingFactor(), 1);
    ASSERT_EQ(description.getVariables().size(), 2);
    EXPECT_EQ(description.getVariables().front().type_, vfm::ScaleTypeEnum::none);
}

TEST(GeometryScalingTests, MetadataRoundTripKeepsScalesIndependent) {
    vfm::ScaleDescription description;
    description.setTimeScalingFactor(2);
    description.setDistanceScalingFactor(8);
    description.setGeometryScalingFactor(4);
    description.add(vfm::VariableScaleDescription("section_0.source.x,geometry"));

    const vfm::ScaleDescription restored(description.serialize());
    EXPECT_FLOAT_EQ(restored.getTimeScalingFactor(), 2);
    EXPECT_FLOAT_EQ(restored.getDistanceScalingFactor(), 8);
    EXPECT_FLOAT_EQ(restored.getGeometryScalingFactor(), 4);
    ASSERT_EQ(restored.getVariables().size(), 1);
    EXPECT_EQ(restored.getVariables().front().type_, vfm::ScaleTypeEnum::geometry);
}

TEST(GeometryScalingTests, TraceValuesUseIndependentScales) {
    vfm::ScaleDescription description;
    description.setTimeScalingFactor(2);
    description.setDistanceScalingFactor(8);
    description.setGeometryScalingFactor(4);
    description.add(vfm::VariableScaleDescription("section_0.source.x,geometry"));
    description.add(vfm::VariableScaleDescription("section_0_end,distance"));
    description.add(vfm::VariableScaleDescription("ego.v,velocity"));

    vfm::MCTrace trace;
    trace.getTrace().push_back({"1.1", {
        {"env.section_0.source.x", "-12"},
        {"section_0_end", "32"},
        {"env.ego.v", "24"},
        {"env.section_0.angle", "90"}}});
    vfm::StaticHelper::applyTimescaling(trace, description);

    const auto& values = trace.getConstTrace().front().second;
    EXPECT_FLOAT_EQ(std::stof(values.at("env.section_0.source.x")), -3);
    EXPECT_FLOAT_EQ(std::stof(values.at("section_0_end")), 4);
    EXPECT_FLOAT_EQ(std::stof(values.at("env.ego.v")), 6);
    EXPECT_EQ(values.at("env.section_0.angle"), "90");
}

TEST(TurningRadiusTests, HandlesDisabledStraightAndImpossibleTurns) {
    EXPECT_EQ(vfm::macro::Script::processScript(
        "@{90}@.minDistConnectionForTurningRadius[0,1,10,1,4,1]"), "1");
    EXPECT_EQ(vfm::macro::Script::processScript(
        "@{0}@.minDistConnectionForTurningRadius[5,1,10,1,4,1,90]"), "1");
    EXPECT_EQ(vfm::macro::Script::processScript(
        "@{180}@.minDistConnectionForTurningRadius[5,1,10,1,4,1]"), "11");
}

TEST(ConnectorGeometryTests, SharesEndpointTangentsAndLaneOffsets) {
    const auto curve = vfm::bezier::connectorBezier({0, 0}, {1, 0}, {4, 4}, {0, 1}, 1);
    EXPECT_FLOAT_EQ(curve[0].x, 0);
    EXPECT_FLOAT_EQ(curve[0].y, -1);
    EXPECT_FLOAT_EQ(curve[3].x, 5);
    EXPECT_FLOAT_EQ(curve[3].y, 4);
    EXPECT_FLOAT_EQ(curve[1].y, curve[0].y);
    EXPECT_FLOAT_EQ(curve[2].x, curve[3].x);
    EXPECT_GT(curve[1].x, curve[0].x);
    EXPECT_LT(curve[2].y, curve[3].y);
}

TEST(ConnectorGeometryTests, CurvatureBoundAcceptsWideTurnsAndRejectsCusps) {
    const auto wide_turn = vfm::bezier::connectorBezier({0, 0}, {1, 0}, {8, 8}, {0, 1});
    EXPECT_TRUE(vfm::bezier::hasMinimumRadius(wide_turn, 5));
    EXPECT_FALSE(vfm::bezier::hasMinimumRadius(wide_turn, 6));
    EXPECT_TRUE(vfm::bezier::hasMinimumRadius(
        vfm::bezier::connectorBezier({0, 0}, {1, 0}, {10, 0}, {1, 0}), 1000));
    EXPECT_FALSE(vfm::bezier::hasMinimumRadius({{{0, 0}, {1, 0}, {-1, 0}, {0, 0}}}, 1));
    EXPECT_FALSE(vfm::bezier::hasMinimumRadius({{{0, 0}, {0, 0}, {0, 0}, {0, 0}}}, 1));
}

TEST(ConnectorGeometryTests, CurvatureBoundCoversEndpointRounding) {
    const auto nominal = vfm::bezier::connectorBezier({0, 0}, {1, 0}, {8, 8}, {0, 1});
    ASSERT_TRUE(vfm::bezier::hasMinimumRadius(nominal, 2, 0.75));
    for (int direction = 0; direction < 360; direction += 15) {
        const float radians = direction * std::acos(-1.0f) / 180;
        const auto displaced = vfm::bezier::connectorBezier({0, 0}, {1, 0},
            {8 + 0.75f * std::cos(radians), 8 + 0.75f * std::sin(radians)}, {0, 1});
        EXPECT_LE(vfm::bezier::maxCurvature(displaced[0], displaced[1], displaced[2], displaced[3]), 0.5f);
    }
}

TEST(ConnectorGeometryTests, PreservesLegacyArclengthRounding) {
    const float pi = 3.1415926536f;
    for (int lanes = 1; lanes <= 3; ++lanes) {
        for (int lane = 0; lane < lanes; ++lane) {
            const float offset = (lane - (lanes - 1.0f) / 2) * 4;
            for (int degrees = 0; degrees < 360; degrees += 15) {
                const float angle = degrees * pi / 180;
                const vfm::Vec2D backward{std::cos(angle - pi), std::sin(angle - pi)};
                const vfm::Vec2D normal{std::cos(angle + pi / 2), std::sin(angle + pi / 2)};
                const vfm::Vec2D heading{std::cos(angle), std::sin(angle)};
                for (const float distance : {1.0f, 3.0f, 5.0f, 10.0f, 20.0f, 50.0f}) {
                    const vfm::Vec2D start{0, offset};
                    const vfm::Vec2D end = vfm::Vec2D{distance, 0} - backward * distance + normal * offset;
                    const float handle = start.distance(end) / 3;
                    const float legacy = vfm::bezier::arcLength(1, start, {handle, offset}, end + backward * handle, end);
                    const auto curve = vfm::bezier::connectorBezier({0, 0}, {1, 0},
                        vfm::Vec2D{distance, 0} + heading * distance, heading, -offset);
                    EXPECT_EQ(std::round(vfm::bezier::arcLength(1, curve[0], curve[1], curve[2], curve[3])),
                        std::round(legacy)) << "lanes=" << lanes << " lane=" << lane
                        << " angle=" << degrees << " distance=" << distance;
                }
            }
        }
    }
}

TEST(TurningRadiusTests, ConvertsGeometryUnitsAndMirrorsTurns) {
    EXPECT_EQ(vfm::macro::Script::processScript(
        "@{90}@.minDistConnectionForTurningRadius[5,1,10,1,4,1,90]"), "8");
    EXPECT_EQ(vfm::macro::Script::processScript(
        "@{270}@.minDistConnectionForTurningRadius[5,1,10,1,4,1,90]"), "8");
    EXPECT_EQ(vfm::macro::Script::processScript(
        "@{90}@.minDistConnectionForTurningRadius[5,4,40,1,4,4,90]"), "32");
}

TEST(TurningRadiusTests, QuantizedConnectorsRespectMinimumRadius) {
    const int distance = std::stoi(vfm::macro::Script::processScript(
        "@{80}@.minDistConnectionForTurningRadius[2,6,16,1,4,2]"));
    ASSERT_LE(distance, 16);

    const float heading = 80 * std::acos(-1.0f) / 180;
    const int cosine = static_cast<int>(std::round(100 * std::cos(heading)));
    const int sine = static_cast<int>(std::round(100 * std::sin(heading)));
    const vfm::Vec2D origin{0, 0};
    const vfm::Vec2D target{
        static_cast<float>(distance * (100 + cosine) / 100) / 2,
        static_cast<float>(distance * sine / 100) / 2};
    const auto handles = vfm::bezier::getNiceBetweenPoints(
        origin, vfm::Vec2D{-1, 0}, target,
        target + vfm::Vec2D{std::cos(heading), std::sin(heading)});

    const float curvature = vfm::bezier::maxCurvature(
        origin, handles[0], handles[2], target);
    EXPECT_LE(curvature, 0.5f) << "Actual turning radius: " << 1 / curvature;
}

TEST(TurningRadiusTests, BoundsCoverRotationsNegativeCoordinatesAndLargerDistances) {
    const float length = 2.5f;
    const float pi = std::acos(-1.0f);
    const auto model_direction = [](const int degrees) {
        const float radians = degrees / 360.0f * 2 * 3.1415f;
        return vfm::Vec2D{std::round(100 * std::cos(radians)), std::round(100 * std::sin(radians))};
    };
    for (const int units : {1, 2, 4}) {
        const float error = std::sqrt(2.0f) * (1.0f / units + 0.0052f * length) + 0.0001f;
        for (const int lanes : {1, 2}) {
            for (const int turn : {0, 45, 90, 270}) {
                const int maximum = 20 * units;
                const int minimum = std::stoi(vfm::macro::Script::processScript(
                    "@{" + std::to_string(turn) + "}@.minDistConnectionForTurningRadius[2,"
                    + std::to_string(3 * units) + "," + std::to_string(maximum) + ","
                    + std::to_string(lanes) + ",4," + std::to_string(units) + ",15," + std::to_string(error) + "]"));
                ASSERT_LE(minimum, maximum) << "units=" << units << " lanes=" << lanes << " turn=" << turn;
                for (int distance = minimum; distance <= maximum; ++distance) {
                    for (int heading = 0; heading < 360; heading += 15) {
                        const int target_heading = (heading + turn) % 360;
                        const vfm::Vec2D direction{std::cos(heading * pi / 180), std::sin(heading * pi / 180)};
                        const vfm::Vec2D next_direction{std::cos(target_heading * pi / 180), std::sin(target_heading * pi / 180)};
                        const auto model_sum = model_direction(heading) + model_direction(target_heading);
                        const auto model_source = model_direction(heading) * (length * units / 100);
                        const vfm::Vec2D drain{std::trunc(model_source.x) / units, std::trunc(model_source.y) / units};
                        const vfm::Vec2D target = drain + vfm::Vec2D{
                            std::trunc(distance * model_sum.x / 100) / units,
                            std::trunc(distance * model_sum.y / 100) / units};
                        for (int lane = 0; lane < lanes; ++lane) {
                            const auto curve = vfm::bezier::connectorBezier(direction * length, direction,
                                target, next_direction, (lane - (lanes - 1.0f) / 2) * 4);
                            EXPECT_LE(vfm::bezier::maxCurvature(curve[0], curve[1], curve[2], curve[3]), 0.50001f)
                                << "units=" << units << " turn=" << turn << " heading=" << heading
                                << " distance=" << distance << " lane=" << lane;
                        }
                    }
                }
            }
        }
    }
}

static std::string expandSectionTemplate(const std::string& settings,
    const fs::path& template_path = fs::absolute(fs::path(__FILE__).parent_path().parent_path() / "src/templates")) {

    return vfm::macro::Script::processScript(
        "@{" + template_path.string() + "}@***********.stringToHeap[MY_PATH].nil\n" + R"(
@{@SECTIONS=2; @SEGMENTS=1; @NUMLANES=1; @REACHABILITY_ONLY=true;
  @MODEL_INTERSECTION_GEOMETRY=true; @CONCRETE_MODEL=true;
  @ANGLEGRANULARITY=90; @MAXOUTGOINGCONNECTIONS=1;
  @MINDISTCONNECTIONS=1; @MAXDISTCONNECTIONS=2;
  @MAXDISTENDPOINTS=0; @MIN_TURNING_RADIUS=0;}@***********.eval.nil
)" + settings + R"(
@{EnvModel_Scaling.tpl}@********.include
@{EnvModel_Parameters.tpl}@********.Include
@{EnvModel_Geometry.tpl}@********.include
@{EnvModel_Sections.tpl}@*******.include
@{EnvModel_Reachability.tpl}@********.include.if[@{REACHABILITY_ONLY}@.eval]
)");
}

TEST(SectionTemplateTests, DisabledRadiusEmitsNoRadiusConstraints) {
    const std::string model = expandSectionTemplate("");
    ASSERT_NE(model.find("section_0_end : "), std::string::npos);
    ASSERT_EQ(model.find("@{"), std::string::npos);
    EXPECT_EQ(model.find("min_dist_for_turning_radius"), std::string::npos);
}

TEST(SectionTemplateTests, FractionalFixedLengthIsScaledBeforeRounding) {
    const std::string model = expandSectionTemplate(R"(
@{@DISTANCESCALING=2000; @GEOMETRYSCALING=2000;}@***********.eval.nil
@{@(1.5)@}@***********.stringToHeap[FIXED_SECTION_LENGTHs].nil
)");
    ASSERT_EQ(model.find("@{"), std::string::npos);
    const auto declaration = model.find("section_0_end : ");
    ASSERT_NE(declaration, std::string::npos);
    EXPECT_NE(model.find("section_0_end : 3 .. 3;"), std::string::npos)
        << model.substr(declaration, 80);
}

static const std::string fixed_quarter_turn_settings = R"(
@{@(0)@@(1)@}@***********.stringToHeap[FIXED_SECTION_IDs].nil
@{@(0)@@(2.5)@}@***********.stringToHeap[FIXED_SECTION_SOURCE_Xs].nil
@{@(0)@@(1.5)@}@***********.stringToHeap[FIXED_SECTION_SOURCE_Ys].nil
@{@(0)@@(90)@}@***********.stringToHeap[FIXED_SECTION_ANGLEs].nil
@{@(1)@@(1)@}@***********.stringToHeap[FIXED_SECTION_LENGTHs].nil
@{@(@(0)@@(@(1)@)@)@@(@(1)@@(@(-1)@)@)@}@***********.stringToHeap[FIXED_SECTION_CONNECTORS].nil
)";

TEST(SectionTemplateTests, GeometryAndDistanceUnitsRemainIndependent) {
    const std::string model = expandSectionTemplate(fixed_quarter_turn_settings + R"(
@{@DISTANCESCALING=4000; @GEOMETRYSCALING=2000;}@***********.eval.nil
)");
    ASSERT_EQ(model.find("@{"), std::string::npos);
    EXPECT_NE(model.find("section_1.source.x : 5 .. 5;"), std::string::npos);
    EXPECT_NE(model.find("section_1.source.y : 3 .. 3;"), std::string::npos);
    EXPECT_NE(model.find("section_1_end : 4 .. 4;"), std::string::npos);
}

TEST(SectionTemplateTests, FixedConnectorsRemainExemptFromMinimumRadius) {
    const std::string unconstrained = expandSectionTemplate(fixed_quarter_turn_settings);
    const std::string constrained = expandSectionTemplate(fixed_quarter_turn_settings + R"(
@{@MIN_TURNING_RADIUS=5;}@***********.eval.nil
)");
    ASSERT_EQ(unconstrained.find("@{"), std::string::npos);
    ASSERT_EQ(constrained.find("@{"), std::string::npos);
    ASSERT_NE(constrained.find("outgoing_connection_0_of_section_0 : 1 .. 1;"),
        std::string::npos);
    const auto unconstrained_start = unconstrained.find("FROZENVAR");
    const auto constrained_start = constrained.find("FROZENVAR");
    ASSERT_NE(unconstrained_start, std::string::npos);
    ASSERT_NE(constrained_start, std::string::npos);
    EXPECT_TRUE(unconstrained.substr(unconstrained_start)
        == constrained.substr(constrained_start))
        << "Fixed connectors should remain exempt from the minimum radius constraint.";
}

TEST(SectionTemplateTests, FeatureOffPreservesLegacySingleLaneArclength) {
    const std::string model = expandSectionTemplate(R"(
@{@DISTANCESCALING=2000; @MINDISTCONNECTIONS=5; @MAXDISTCONNECTIONS=5;}@***********.eval.nil
)");
    ASSERT_EQ(model.find("@{"), std::string::npos);
    ASSERT_EQ(model.find("min_dist_for_turning_radius"), std::string::npos);
    const std::string marker = "angle_from_sec_0_to_sec_1 = 90 & connection_distance_sec_0_to_sec_1 = 5 : ";
    const auto branch_start = model.find(marker);
    ASSERT_NE(branch_start, std::string::npos);
    const auto expression_start = branch_start + marker.size();
    const auto expression_end = model.find(';', expression_start);
    ASSERT_NE(expression_end, std::string::npos);
    const auto expression = vfm::StaticHelper::replaceAll(
        model.substr(expression_start, expression_end - expression_start),
        "connection_distance_sec_0_to_sec_1", "5");
    const int arclength = std::stoi(vfm::macro::Script::processScript(
        "@{trunc(" + expression + ") }@.eval[0]"));
    EXPECT_EQ(arclength, 16);
}

TEST(SectionTemplateTests, DisabledGeometryEmitsNoGeometryOrRadiusConstraints) {
    const std::string model = expandSectionTemplate(R"(
@{@MODEL_INTERSECTION_GEOMETRY=false; @MIN_TURNING_RADIUS=5;}@***********.eval.nil
)");
    ASSERT_NE(model.find("section_0_end : "), std::string::npos);
    ASSERT_EQ(model.find("@{"), std::string::npos);
    EXPECT_EQ(model.find("section_0.source.x"), std::string::npos);
    EXPECT_EQ(model.find("min_dist_for_turning_radius"), std::string::npos);
}

TEST(SectionTemplateTests, RadiusBoundsAreSharedAndUnknownAnglesAreForbidden) {
    const std::string model = expandSectionTemplate(R"(
@{@SECTIONS=3; @SECTIONSMAXLENGTH=2; @MIN_TURNING_RADIUS=2;
  @MAXDISTCONNECTIONS=8; @GEOMETRYSCALING=2000;}@***********.eval.nil
)");
    ASSERT_EQ(model.find("@{"), std::string::npos);
    ASSERT_EQ(model.find("#ERROR"), std::string::npos);
    const std::string declaration = "min_dist_for_turning_radius_at_angle_90 := ";
    const auto first = model.find(declaration);
    ASSERT_NE(first, std::string::npos);
    EXPECT_EQ(model.find(declaration, first + 1), std::string::npos);
    EXPECT_NE(model.find("angle_from_sec_0_to_sec_1 = 90 : min_dist_for_turning_radius_at_angle_90;"), std::string::npos);
    EXPECT_NE(model.find("angle_from_sec_1_to_sec_2 = 90 : min_dist_for_turning_radius_at_angle_90;"), std::string::npos);
    EXPECT_NE(model.find("TRUE : 17;"), std::string::npos);
    EXPECT_LE(std::stoi(model.substr(first + declaration.size())), 16);
}


const std::string tpljson{R"(
{
   "#TEMPLATE": {
      "ALLOW_ZEROLENGTH_SECTIONS": false,
      "ANGLEGRANULARITY": 72,
      "BMC_CNT": 100,
      "BMC_NUMBER_OF_CEXS": 1,
      "CALCULATE_CENTER_GAP": false,
      "CALCULATE_LEFT_GAP": false,
      "CALCULATE_RIGHT_GAP": false,
      "CALCULATE_RIGHT_GAP_REAR": false,
      "DEBUG": false,
      "DISTANCESCALING": 1000,
      "EGOLESS": true,
      "UCD": false,
      "FEASIBILITY": false,
      "KEEP_EGO_FROM_GREEN": false,
      "LANES_MAX_SPEEDS": "@(70)@@(70)@@(70)@@(70)@",
      "LANES_MIN_SPEEDS": "@(-70)@@(-70)@@(-70)@@(-70)@",
      "LTL_MODE": false,
      "MAXACCELEGO": 5,
      "MAXACCELNONEGO": 5,
      "MAXDISTCONNECTIONS": 30,
      "MAXOUTGOINGCONNECTIONS": 1,
      "MINACCELEGO": -1,
      "MINACCELNONEGO": -1,
      "MINDISTCONNECTIONS": 10,
      "MIN_TIME_BETWEEN_LANECHANGES": 0,
      "NONEGOS": 7,
      "NUMLANES": 2,
      "LATERAL_LC_GRANULARITY": 10,
      "MAX_JUMP_OVER_TECHNICAL_LANES": 7,
      "MAX_JUMP_UNTIL_VELOCITY": 3,
      "MIN_JUMP_FROM_VELOCITY": 12,
      "SIMPLE_LC": true,
      "LANE_WIDTH": 400,
      "FORWARD_DRIVING_CAR_IDS": "@(0)@",
      "BACKWARD_DRIVING_CAR_IDS": "@(1)@",
      "SCENGEN_EXISTENTIAL_PROPERTIES": "1-9",
      "SCENGEN_MODE": false,
      "SCENGEN_UNIVERSAL_PROPERTIES": "0-0",
      "SECTIONS": 1,
      "MAXSPEEDNONEGO": 15,
      "MINSTARTSPEEDNONEGO_UCD": 10,
      "MAXSTARTSPEEDNONEGO_UCD": 15,
      "UCD_CONFIG_PRIOS": "1",
      "UCD_MIN_VELOCITY_TO_ALLOW_LC": 1,
      "SECTIONSMAXLENGTH": 10000,
      "SECTIONSMINLENGTH": 10000,
      "SEGMENTS": 1,
      "SEGMENTSMINLENGTH": 10,
      "CONCRETE_MODEL": true,
      "MODEL_INTERSECTION_GEOMETRY": true,
      "SPEC": "#{ env.cnt == 0 }#",
      "ShowLOG": true,
      "TIMESCALING": 1000,
      "_BP_INCLUDES_FILE_PATH": "../src/examples/ego_less/vfm-includes.txt",
      "_CACHED_PATH": "../tmp/cached",
      "_EXTERNAL_PATH": "../external",
      "_GENERATED_PATH": "../tmp/generated",
      "_ENVMODEL_PATH": "../src/templates",
      "_CEX_FILE_NAME": "debug_trace_array.txt"
   }
}
    )"};

TEST(nuXmvTests, basicRun) {
    vfm::StaticHelper::removeAllFilesSafe(std::filesystem::path("../tmp"));
    vfm::StaticHelper::createDirectoriesSafe(std::string("../tmp"));
    vfm::StaticHelper::createDirectoriesSafe(std::string("../tmp/generated"));
    vfm::StaticHelper::createDirectoriesSafe(std::string("../tmp/cached"));
    vfm::StaticHelper::writeTextToFile(tpljson, "../tmp/envmodel_config.tpl.json");

    const std::string result = vfm::macro::Script::processScript(R"(
        @{../src/templates/}@.stringToHeap[MY_PATH]
        @{../../tmp/envmodel_config.tpl.json}@.generateEnvmodels
        @{../../tmp/envmodel_config.tpl.json}@.runMCJobs[16]
    )");

    ASSERT_EQ(result, R"(
        MY_PATH is set to the following path '/home/okl2abt/very-fast-math/src/templates' which is existing on the file system.
        Envmodel generation finished for '../src/templates//../../tmp/../../tmp/envmodel_config.tpl.json'.
        MC runs finished for '../src/templates//../../tmp/../../tmp/envmodel_config.tpl.json'.
    )");

    fs::path resultfile1 = "../tmp/generated_config_DUMMYVAR=0/debug_trace_array.txt";
    fs::path resultfile2 = "../tmp/generated_config_DUMMYVAR=0/0/preview2/preview2_2.png";
    
    // Verify the generated debug_trace_array.txt is unchanged via a content hash.
    // const std::string trace{ vfm::StaticHelper::readFile(resultfile1) };
    // const auto newval = fnv1a64(trace);
    // ASSERT_EQ(newval, 0x8479eea6bbcd6290ULL) << resultfile1 << " content changed unexpectedly.";

    // Use EXPECT_TRUE so the test continues even if this assertion fails
    EXPECT_TRUE(fs::exists(resultfile1)) << "File does not exist: " << resultfile1;
    EXPECT_TRUE(fs::exists(resultfile2)) << "File does not exist: " << resultfile2;
}

TEST(StaticTests, StaticHelper) { 

    std::string capitalized = vfm::StaticHelper::firstLetterCapital("abcd");
    ASSERT_EQ(capitalized, "Abcd");

}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
