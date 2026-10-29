#include "static_helper.h"
#include "geometry/bezier_functions.h"
#include "model_checking/mc_types.h"
#include "testing/test_functions.h"
#include "vfmacro/script.h"
#include <gtest/gtest.h>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <sstream>
#if defined(__linux__)
#include <unistd.h>
#endif

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

TEST(VfmacroTests, ScriptVariableAnnotationsDoNotAddApostrophes) {
    EXPECT_EQ(vfm::macro::Script::processScript(
        "@{42}@*.setScriptVar[test_number]\n@{test_number}@.scriptVar"),
        "test_number = 42\n42");
    EXPECT_EQ(vfm::macro::Script::processScript(
        "@{some text}@*.setScriptVar[test_text]\n@{test_text}@.scriptVar"),
        "test_text = some text\nsome text");
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

static void expectSmvCommentsWithoutApostrophes(const std::string& source) {
    std::istringstream lines(source);
    std::string line;
    size_t line_number = 0;
    while (std::getline(lines, line)) {
        ++line_number;
        const auto comment_start = line.find("--");
        if (comment_start != std::string::npos) {
            EXPECT_EQ(line.find('\'', comment_start), std::string::npos)
                << "Apostrophe in SMV comment at line " << line_number << ": " << line;
        }
    }
}

TEST(SmvTemplateTests, CommentsContainNoApostrophes) {
    const auto template_path = fs::absolute(fs::path(__FILE__).parent_path().parent_path() / "src/templates");
    size_t template_count = 0;
    for (const auto& entry : fs::directory_iterator(template_path)) {
        if (entry.is_regular_file() && entry.path().extension() == ".tpl") {
            ++template_count;
            SCOPED_TRACE(entry.path().string());
            expectSmvCommentsWithoutApostrophes(vfm::StaticHelper::readFile(entry.path()));
        }
    }
    EXPECT_GT(template_count, 0);
}

static std::string expandCompactUcdTemplate(const std::string& settings = "") {
    const auto template_path = fs::absolute(fs::path(__FILE__).parent_path().parent_path() / "src/templates");
    return vfm::macro::Script::processScript(
        "@{" + template_path.string() + "}@***********.stringToHeap[MY_PATH].nil\n" + R"(
@{@UCD=true; @EGOLESS=true; @UCD_TASK7_COMPACT=true; @SIMPLE_LC=true;
  @SECTIONS=1; @SEGMENTS=1; @NONEGOS=5; @NUMLANES=2; @LATERAL_LC_GRANULARITY=10;
  @MODEL_INTERSECTION_GEOMETRY=false; @TIMESCALING=2000; @DISTANCESCALING=4000;
  @MINACCELNONEGO=-3; @MAXACCELNONEGO=3; @MAX_JUMP_OVER_TECHNICAL_LANES=7;
  @MAX_JUMP_UNTIL_VELOCITY=3; @MIN_JUMP_FROM_VELOCITY=12;
  @MIN_TIME_BETWEEN_LANECHANGES=0; @UCD_MIN_VELOCITY_TO_ALLOW_LC=1;}@***********.eval.nil
)" + settings + "\n@{EnvModel.tpl}@********.include\n");
}

TEST(SmvTemplateTests, CompactUcdKeepsTheLegacyModeAvailable) {
    const auto compact = expandCompactUcdTemplate();
    const auto legacy = expandCompactUcdTemplate("@{@UCD_TASK7_COMPACT=false;}@***********.eval.nil");
    EXPECT_EQ(compact.find("@{"), std::string::npos);
    EXPECT_EQ(compact.find("#error"), std::string::npos);
    EXPECT_NE(compact.find("lane_coordinate : integer;"), std::string::npos);
    EXPECT_EQ(compact.find("INVAR ego.a = veh___609___.a;"), std::string::npos);
    EXPECT_EQ(legacy.find("lane_coordinate : integer;"), std::string::npos);
    EXPECT_EQ(legacy.find("VFM_TASK7_PARKED"), std::string::npos);
    EXPECT_NE(legacy.find("veh___609___.a : integer;"), std::string::npos);
    EXPECT_NE(legacy.find("INVAR ego.a = veh___609___.a;"), std::string::npos);
}

TEST(SmvTemplateTests, CompactUcdAcceptsDifferentCountsScalingAndLimits) {
    const auto model = expandCompactUcdTemplate(R"(
@{@NONEGOS=10; @NUMLANES=3; @LATERAL_LC_GRANULARITY=1;
  @TIMESCALING=1000; @DISTANCESCALING=2000; @MINACCELNONEGO=-5; @MAXACCELNONEGO=2;
  @MAX_JUMP_OVER_TECHNICAL_LANES=3; @MAXSPEEDNONEGO=20;
  @MIN_TIME_BETWEEN_LANECHANGES=2;}@***********.eval.nil
@{@(40)@}@***********.stringToHeap[FIXED_SECTION_LENGTHs].nil
)");
    EXPECT_EQ(model.find("@{"), std::string::npos);
    EXPECT_EQ(model.find("#error"), std::string::npos);
    EXPECT_NE(model.find("veh___699___.lane_coordinate : integer;"), std::string::npos);
    EXPECT_NE(model.find("veh___699___.lane_coordinate <= 6;"), std::string::npos);
    EXPECT_NE(model.find("VFM_TASK7_PARKED_POS_9 <= 80"), std::string::npos);
    EXPECT_NE(model.find("veh___699___.lane_b3 := veh___699___.lane_coordinate >= 5 & veh___699___.lane_coordinate <= 7;"), std::string::npos);
}

TEST(SmvTemplateTests, CompactOptionOffPreservesNonUcdEgoAndAcceleration) {
        const auto model = expandCompactUcdTemplate(R"(
@{@UCD=false; @UCD_TASK7_COMPACT=false; @EGOLESS=false; @SIMPLE_LC=false;
    @NUMLANES=3; @LATERAL_LC_GRANULARITY=0; @NONEGOS=2;}@***********.eval.nil
)");
        EXPECT_EQ(model.find("@{"), std::string::npos);
        EXPECT_EQ(model.find("#error UCD_TASK7_COMPACT"), std::string::npos);
        EXPECT_EQ(model.find("VFM_TASK7_PARKED"), std::string::npos);
        EXPECT_EQ(model.find("lane_coordinate : integer;"), std::string::npos);
        EXPECT_NE(model.find("veh___609___.a : integer;"), std::string::npos);
        EXPECT_NE(model.find("veh___619___.a : integer;"), std::string::npos);
        EXPECT_NE(model.find("ego.a := veh___609___.a;"), std::string::npos);
        EXPECT_NE(model.find("next(ego.v) := min(max(ego.v + ego.a, -ego.max_vel), ego.max_vel);"), std::string::npos);
        EXPECT_NE(model.find("veh___619___.do_lane_change : boolean;"), std::string::npos);
}

TEST(SmvTemplateTests, CompactUcdOnlyRejectsUnsupportedDynamics) {
    const auto unsupported = expandCompactUcdTemplate("@{@EGOLESS=false;}@***********.eval.nil");
    const auto zero_divisor = expandCompactUcdTemplate("@{@MIN_JUMP_FROM_VELOCITY=3;}@***********.eval.nil");
    EXPECT_NE(unsupported.find("#error UCD_TASK7_COMPACT"), std::string::npos);
    EXPECT_NE(zero_divisor.find("#error UCD_TASK7_COMPACT"), std::string::npos);
}

TEST(SmvTemplateTests, CompactUcdFallsBackWithoutStationaryLaneHold) {
    const auto model = expandCompactUcdTemplate("@{@UCD_MIN_VELOCITY_TO_ALLOW_LC=0;}@***********.eval.nil");
    EXPECT_EQ(model.find("#error"), std::string::npos);
    EXPECT_EQ(model.find("#if defined(VFM_TASK7_PARKED_POS_"), std::string::npos);
    EXPECT_NE(model.find("#define VFM_TASK7_PARKED_2 0"), std::string::npos);
}

TEST(SmvTemplateTests, AllVehiclesHavePositionUpdates) {
    for (const bool compact : {false, true}) {
        SCOPED_TRACE(compact ? "compact" : "legacy");
        const auto model = expandCompactUcdTemplate(compact ? "" : "@{@UCD_TASK7_COMPACT=false;}@***********.eval.nil");
        EXPECT_EQ(model.find("@{"), std::string::npos);
        EXPECT_EQ(model.find("#error"), std::string::npos);
        for (int vehicle_id = 0; vehicle_id < 5; ++vehicle_id) {
            const auto vehicle = "veh___6" + std::to_string(vehicle_id) + "9___";
            SCOPED_TRACE(vehicle);
            EXPECT_NE(model.find("next(" + vehicle + ".prev_abs_pos) := " + vehicle + ".abs_pos;"), std::string::npos);
            EXPECT_NE(model.find("next(" + vehicle + ".abs_pos) := case"), std::string::npos);
            if (compact) {
                EXPECT_NE(model.find("#if defined(VFM_TASK7_PARKED_POS_" + std::to_string(vehicle_id) + ")"), std::string::npos);
            }
        }
    }
}

static std::string expandSectionTemplate(const std::string& settings,
    const fs::path& template_path = fs::absolute(fs::path(__FILE__).parent_path().parent_path() / "src/templates")) {

    return vfm::macro::Script::processScript(
        "@{" + template_path.string() + "}@***********.stringToHeap[MY_PATH].nil\n" + R"(
@{@SECTIONS=2; @SEGMENTS=1; @NUMLANES=1; @REACHABILITY_ONLY=true;
  @MODEL_INTERSECTION_GEOMETRY=true;
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

TEST(SectionTemplateTests, ExpandedCommentsContainNoApostrophes) {
    const std::string model = expandSectionTemplate("");
    ASSERT_NE(model.find("section_0_end : "), std::string::npos);
    ASSERT_EQ(model.find("@{"), std::string::npos);
    expectSmvCommentsWithoutApostrophes(model);
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

TEST(SectionTemplateTests, UnusedCollisionGeometryEmitsNoStateOrDefinitions) {
    const std::string model = expandSectionTemplate("");
    ASSERT_EQ(model.find("@{"), std::string::npos);
    EXPECT_EQ(model.find("connector_curve_scale :="), std::string::npos);
    EXPECT_EQ(model.find("conn_0_to_1_chord"), std::string::npos);
    EXPECT_EQ(model.find("conn_0_to_1_arc_"), std::string::npos);
}

TEST(SectionTemplateTests, WorkspaceBoundsRespectTheGeometryFlag) {
    const std::string workspace = R"(
@{@(-200)@@(-200)@@(200)@@(200)@}@***********.stringToHeap[WORKSPACE_BOUNDs].nil
)";
    const std::string enabled = expandSectionTemplate(workspace);
    ASSERT_EQ(enabled.find("@{"), std::string::npos);
    EXPECT_NE(enabled.find("conn_0_to_1_chord : integer;"), std::string::npos);
    EXPECT_NE(enabled.find("section_0.source.x >= -200"), std::string::npos);
    const std::string disabled = expandSectionTemplate(workspace + R"(
@{@MODEL_INTERSECTION_GEOMETRY=false;}@***********.eval.nil
)");
    ASSERT_EQ(disabled.find("@{"), std::string::npos);
    EXPECT_EQ(disabled.find("section_0.source.x"), std::string::npos);
    EXPECT_EQ(disabled.find("connector_curve_scale :="), std::string::npos);
}

TEST(SectionTemplateTests, FixedObstacleTransformsPreserveTheirValuesAndBases) {
    const std::string rectangle = R"(
@{@(10)@}@***********.stringToHeap[RECT_OBSTACLES_TL_Xs].nil
@{@(10)@}@***********.stringToHeap[RECT_OBSTACLES_TL_Ys].nil
@{@(12)@}@***********.stringToHeap[RECT_OBSTACLES_BR_Xs].nil
@{@(12)@}@***********.stringToHeap[RECT_OBSTACLES_BR_Ys].nil
)";
    const std::string plain = expandSectionTemplate(rectangle);
    ASSERT_EQ(plain.find("@{"), std::string::npos);
    EXPECT_NE(plain.find("rect_obstacles_tl_x_0 := 10;"), std::string::npos);
    EXPECT_EQ(plain.find("rect_obstacles_shift_kx_0 :"), std::string::npos);
    for (const int geometry_factor : {1, 2}) {
        const std::string model = expandSectionTemplate(rectangle + R"(
@{@(3)@}@***********.stringToHeap[RECT_OBSTACLES_SHIFT_MIN_Xs].nil
@{@(3)@}@***********.stringToHeap[RECT_OBSTACLES_SHIFT_MAX_Xs].nil
@{@(-2)@}@***********.stringToHeap[RECT_OBSTACLES_SHIFT_MIN_Ys].nil
@{@(-2)@}@***********.stringToHeap[RECT_OBSTACLES_SHIFT_MAX_Ys].nil
@{@(1)@}@***********.stringToHeap[RECT_OBSTACLES_GROW_MIN_Ws].nil
@{@(1)@}@***********.stringToHeap[RECT_OBSTACLES_GROW_MAX_Ws].nil
@{@(2)@}@***********.stringToHeap[RECT_OBSTACLES_GROW_MIN_Hs].nil
@{@(2)@}@***********.stringToHeap[RECT_OBSTACLES_GROW_MAX_Hs].nil
)" + "@{@GEOMETRYSCALING=" + std::to_string(geometry_factor * 1000)
            + "; @RECT_OBSTACLES_SHIFT_STEP=1;}@***********.eval.nil\n");
        ASSERT_EQ(model.find("@{"), std::string::npos);
        EXPECT_NE(model.find("rect_obstacles_shift_kx_0 : 3 .. 3;"), std::string::npos);
        EXPECT_NE(model.find("rect_obstacles_shift_ky_0 : -2 .. -2;"), std::string::npos);
        EXPECT_NE(model.find("rect_obstacles_grow_kw_0 : 1 .. 1;"), std::string::npos);
        EXPECT_NE(model.find("rect_obstacles_grow_kh_0 : 2 .. 2;"), std::string::npos);
        EXPECT_NE(model.find("rect_obstacles_tl_x_0 := " + std::to_string(10 * geometry_factor)
            + " + " + std::to_string(geometry_factor) + " * rect_obstacles_shift_kx_0;"), std::string::npos);
        EXPECT_NE(model.find("rect_obstacles_br_y_0 := " + std::to_string(12 * geometry_factor)
            + " + " + std::to_string(geometry_factor)
            + " * (rect_obstacles_shift_ky_0 + rect_obstacles_grow_kh_0);"), std::string::npos);
    }
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

TEST(MCTraceTests, LooksUpLatestValuesWithoutChangingTheTrace) {
    vfm::MCTrace trace;
    trace.setOutputLevels(vfm::ErrorLevelEnum::invalid, vfm::ErrorLevelEnum::invalid);
    trace.addTraceStep({"1.1", {{"position", "10"}, {"constant", "7"}}});
    trace.addTraceStep({"1.2", {{"speed", "2"}}});
    trace.addTraceStep({"1.3", {{"position", "12"}}});
    const auto original = trace.getConstTrace();
    EXPECT_EQ(trace.getLastValueOfVariableAtStep("position", 0), "10");
    EXPECT_EQ(trace.getLastValueOfVariableAtStep("position", 1), "10");
    EXPECT_EQ(trace.getLastValueOfVariableAtStep("position", 2), "12");
    EXPECT_EQ(trace.getLastValueOfVariableAtStep("constant", 20), "7");
    EXPECT_EQ(trace.getLastValueOfVariableAtStep("speed", 0), "-1");
    EXPECT_EQ(trace.getLastValueOfVariableAtStep("position", -1), "-1");
    EXPECT_EQ(trace.getConstTrace(), original);
    vfm::MCTrace empty;
    empty.setOutputLevels(vfm::ErrorLevelEnum::invalid, vfm::ErrorLevelEnum::invalid);
    EXPECT_EQ(empty.getLastValueOfVariableAtStep("missing", 0), "-1");
}

class MortyLiveTests : public ::testing::Test {
protected:
    fs::path original_directory;
    fs::path directory;
    nlohmann::json config;

    void SetUp() override {
        original_directory = fs::current_path();
        directory = fs::temp_directory_path() / ("vfm-morty-live-" + std::to_string(
            std::chrono::steady_clock::now().time_since_epoch().count()));
        fs::create_directories(directory / "morty");
        fs::create_directories(directory / "generated_config_primary");
        fs::create_directories(directory / "generated_config_fallback");
        fs::create_directories(directory / "external/linux64");
        fs::current_path(directory);
        vfm::test::setMortyLiveMode(true);
        config = {
            {"#TEMPLATE", {
                {"UCD_CONFIG_PRIOS", "_config_primary;_config_fallback;"},
                {"UCD_USE_SHORTCUT", false},
                {"_GENERATED_PATH", "./generated"},
                {"_CACHED_PATH", "./cache"},
                {"_EXTERNAL_PATH", "./external"},
                {"_ENVMODEL_PATH", "."},
                {"_CEX_FILE_NAME", "debug_trace_array.txt"}}},
            {"_config_primary", {{"NONEGOS", 1}}},
            {"_config_fallback", {{"NONEGOS", 1}}}
        };
        writeConfig();
        for (const auto& config_name : {"generated_config_primary", "generated_config_fallback"}) {
            const fs::path folder{ config_name };
            vfm::StaticHelper::writeTextToFile("MODULE main\nINVARSPEC FALSE;\n", folder / "main.smv");
            vfm::StaticHelper::writeTextToFile("quit\n", folder / "script.cmd");
            vfm::StaticHelper::writeTextToFile("planner source\n", folder / "planner.cpp_combined.k2");
            vfm::StaticHelper::writeTextToFile("", folder / "debug_trace_array.txt");
            vfm::ScaleDescription scaling;
            scaling.setTimeScalingFactor(2);
            scaling.setDistanceScalingFactor(4);
            vfm::StaticHelper::writeTextToFile(scaling.serialize(), folder / "scaling_info.txt");
        }
    }

    void TearDown() override {
        vfm::test::setMortyLiveMode(false);
        fs::current_path(original_directory);
        fs::remove_all(directory);
    }

    void writeConfig() {
        vfm::StaticHelper::writeTextToFile(config.dump(), "morty/envmodel_config.tpl.json");
        vfm::StaticHelper::writeTextToFile(config.dump(), "morty/envmodel_config.json");
    }

    std::string trace(const int states) {
        std::string result = "Trace Description: test\nTrace Type: Counterexample\n";
        for (int step = 0; step < states; ++step) {
            result += " -> State: 1." + std::to_string(step + 1) + " <-\n";
            if (step == 0) result +=
                " planner.time_scaling = 2000\n planner.distance_scaling = 4000\n"
                " env.lane_width = 400\n env.num_lanes = 2\n env.num_technical_lanes = 4\n"
                " env.section_0.angle = 0\n env.section_0.source.x = 0\n env.section_0.source.y = 0\n"
                " env.section_0_end = 100\n env.veh___609___.v = 20\n"
                " env.veh___609___.on_normalized_lane = 0\n env.veh___609___.on_straight_section = 0\n"
                " env.veh___609___.traversion_from = -1\n env.veh___609___.traversion_to = -1\n";
            result += " env.veh___609___.abs_pos = " + std::to_string(step * 4) + "\n";
        }
        return result;
    }

    std::string positions() {
        return vfm::macro::Script::processScript("@{" + directory.generic_string()
            + "/}@.stringToHeap[MY_PATH].nil@{morty/envmodel_config.tpl.json}@.extractVehPosFromNusmvFile[_config_primary]");
    }

    void installFakeSolvers(const bool fail = false) {
        vfm::StaticHelper::writeTextToFile(
            "#!/bin/sh\nprintf 'kratos\\n' >> kratos_calls.txt\n"
            "for argument in \"$@\"; do\n case \"$argument\" in\n"
            " -output_file=*) output=${argument#-output_file=};;\n esac\ndone\n"
            "printf 'MODULE planner\\n' > \"$output\"\n", "external/linux64/kratos");
        vfm::StaticHelper::writeTextToFile("#!/bin/sh\nprintf 'nuXmv\\n' >> solver_calls.txt\n"
            "ps -o pgid= -p $$ >> solver_process_groups.txt\n"
            "cat trace_fixture.txt\nexit " + std::to_string(fail ? 7 : 0) + "\n", "external/linux64/nuXmv");
        vfm::StaticHelper::writeTextToFile(trace(4), "trace_fixture.txt");
    }

    void solve() {
        vfm::test::convenienceArtifactRunHardcoded(vfm::test::MCExecutionType::mc,
            "./generated_config_primary", "unused", ".", "unused", "./cache", "./external");
    }
};

TEST_F(MortyLiveTests, KeepsErrorsWithoutRecordingDiagnostics) {
    vfm::Failable logger("live-test");
    logger.setOutputLevels(vfm::ErrorLevelEnum::invalid, vfm::ErrorLevelEnum::invalid);
    logger.addDebug("debug");
    logger.addNote("note");
    logger.addWarning("warning");
    logger.addError("error");
    EXPECT_EQ(logger.hasErrorOccurred(vfm::ErrorLevelEnum::debug), 0);
    EXPECT_EQ(logger.hasErrorOccurred(vfm::ErrorLevelEnum::note), 0);
    EXPECT_EQ(logger.hasErrorOccurred(vfm::ErrorLevelEnum::warning), 1);
    EXPECT_EQ(logger.hasErrorOccurred(vfm::ErrorLevelEnum::error), 1);
}

TEST_F(MortyLiveTests, ReturnsStatusWithoutUnusedArtifacts) {
    vfm::StaticHelper::writeTextToFile(trace(4), "generated_config_primary/debug_trace_array.txt");
    EXPECT_EQ(vfm::test::prepareOutputForMortyUCD(0, 0, 0, false),
        "\n_config_primary:0|;\n_config_fallback:|;");
    EXPECT_FALSE(fs::exists("generated_config_primary/future_point_0.txt"));
    EXPECT_FALSE(fs::exists("generated_config_primary/morty_mc_results.txt"));

    vfm::StaticHelper::writeTextToFile(trace(1), "generated_config_primary/debug_trace_array.txt");
    EXPECT_EQ(vfm::test::prepareOutputForMortyUCD(0, 0, 0, false),
        "\n_config_primary:FINISHED\n_config_fallback:|;");
}

TEST_F(MortyLiveTests, PreservesShortcutFilesWhenEnabled) {
    config["#TEMPLATE"]["UCD_USE_SHORTCUT"] = true;
    writeConfig();
    vfm::StaticHelper::writeTextToFile(trace(4), "generated_config_primary/debug_trace_array.txt");
    vfm::test::prepareOutputForMortyUCD(0, 0, 0, false);
    EXPECT_TRUE(fs::exists("generated_config_primary/future_point_0.txt"));
    EXPECT_TRUE(fs::exists("generated_config_primary/future_point_3.txt"));
    EXPECT_FALSE(fs::exists("generated_config_primary/morty_mc_results.txt"));
}

TEST_F(MortyLiveTests, KeepsTheSameDrivingPoints) {
    vfm::StaticHelper::writeTextToFile(trace(5), "generated_config_primary/debug_trace_array.txt");
    const auto live_positions = positions();
    config["#TEMPLATE"]["UCD_USE_SHORTCUT"] = true;
    writeConfig();
    const auto complete_positions = positions();
    EXPECT_FALSE(live_positions.empty());
    EXPECT_EQ(std::count(live_positions.begin(), live_positions.end(), '\n'), 6);
    EXPECT_EQ(std::count(complete_positions.begin(), complete_positions.end(), '\n'), 10);
    EXPECT_EQ(complete_positions.substr(0, live_positions.size()), live_positions);
}

TEST_F(MortyLiveTests, CompactInputConstantsTrackObservationsAndStationaryConstraints) {
    const std::string main = "-- TASK7 INPUT CONSTANTS\n-- EO TASK7 INPUT CONSTANTS\n"
        "MODULE main\nINVAR env.veh___629___.v = 0;\n--EO-SPEC-STUFF\n";
    vfm::StaticHelper::writeTextToFile(main, "generated_config_primary/main.smv");
    vfm::StaticHelper::writeTextToFile("MODULE main\n--EO-SPEC-STUFF\n", "generated_config_fallback/main.smv");
    vfm::test::prepareInputForMortyUCD("0,10,0,5,0;1,20,4,-5,0;2,42,2,0,0;", -0.5f, 2, 12, "AUTO");
    auto updated = vfm::StaticHelper::readFile("generated_config_primary/main.smv");
    EXPECT_NE(updated.find("#define VFM_TASK7_PARKED_POS_2 168\n"), std::string::npos);
    EXPECT_NE(updated.find("#define VFM_TASK7_PARKED_LANE_2 11\n"), std::string::npos);
    EXPECT_EQ(updated.find("#define VFM_TASK7_PARKED_POS_0"), std::string::npos);
    EXPECT_NE(updated.find("INIT env.veh___629___.abs_pos = 168;"), std::string::npos);
    EXPECT_EQ(vfm::StaticHelper::readFile("generated_config_fallback/main.smv").find("VFM_TASK7"), std::string::npos);

    vfm::test::prepareInputForMortyUCD("0,10,0,5,0;1,20,4,-5,0;2,-1,4,0,0;", -0.5f, 2, 12, "AUTO");
    updated = vfm::StaticHelper::readFile("generated_config_primary/main.smv");
    EXPECT_NE(updated.find("#define VFM_TASK7_PARKED_POS_2 -4\n"), std::string::npos);
    EXPECT_NE(updated.find("INIT env.veh___629___.abs_pos = -4;"), std::string::npos);
    EXPECT_EQ(updated.find("VFM_TASK7_PARKED_POS_2"), updated.rfind("VFM_TASK7_PARKED_POS_2"));

    vfm::StaticHelper::writeTextToFile(vfm::StaticHelper::replaceAll(updated,
        "INVAR env.veh___629___.v = 0;", "INVAR env.veh___629___.v >= 0;"), "generated_config_primary/main.smv");
    vfm::test::prepareInputForMortyUCD("0,10,0,5,0;1,20,4,-5,0;2,43,4,0,0;", -0.5f, 2, 12, "AUTO");
    EXPECT_EQ(vfm::StaticHelper::readFile("generated_config_primary/main.smv").find("VFM_TASK7_PARKED_POS_2"), std::string::npos);
}

#if defined(__linux__)
TEST_F(MortyLiveTests, ReusesTranslationAndRunsNoFalseSpec) {
    installFakeSolvers();
    solve();
    solve();
    EXPECT_EQ(vfm::StaticHelper::readFile("kratos_calls.txt"), "kratos\n");
    EXPECT_EQ(vfm::StaticHelper::readFile("solver_calls.txt"), "nuXmv\nnuXmv\n");
    EXPECT_EQ(std::stoi(vfm::StaticHelper::readFile("solver_process_groups.txt")), ::getpgrp());
    EXPECT_FALSE(fs::exists("generated_config_primary/debug_trace_array_FALSE.txt"));
    EXPECT_FALSE(fs::exists("generated_config_primary/mc_runtimes.txt"));
    EXPECT_EQ(vfm::StaticHelper::readFile("generated_config_primary/main.smv"), "MODULE main\nINVARSPEC FALSE;\n");

    fs::last_write_time("generated_config_primary/planner.cpp_combined.k2.smv",
        fs::last_write_time("generated_config_primary/planner.cpp_combined.k2") - std::chrono::seconds(1));
    solve();
    EXPECT_EQ(vfm::StaticHelper::readFile("kratos_calls.txt"), "kratos\nkratos\n");
}

TEST_F(MortyLiveTests, FailedSolverCannotSupplyATrajectory) {
    installFakeSolvers(true);
    solve();
    EXPECT_TRUE(vfm::StaticHelper::readFile("generated_config_primary/debug_trace_array.txt").empty());
    EXPECT_EQ(vfm::test::prepareOutputForMortyUCD(0, 0, 0, false),
        "\n_config_primary:|;\n_config_fallback:|;");
}

TEST_F(MortyLiveTests, FailedTranslationIsNotReused) {
    installFakeSolvers();
    vfm::StaticHelper::writeTextToFile("exit 7\n", "external/linux64/kratos", true);
    solve();
    EXPECT_FALSE(fs::exists("generated_config_primary/planner.cpp_combined.k2.smv"));
    EXPECT_FALSE(fs::exists("solver_calls.txt"));
    EXPECT_TRUE(vfm::StaticHelper::readFile("generated_config_primary/debug_trace_array.txt").empty());
}

TEST_F(MortyLiveTests, DispatchesOnlyPrioritizedJobsWithoutPacing) {
    installFakeSolvers();
    fs::create_directories("generated_config_unused");
    vfm::mc::McWorkflow workflow;
    const auto started = std::chrono::steady_clock::now();
    const auto jobs = workflow.runMCJobs("./generated", [](const std::string&) { return true; },
        ".", "./morty", "envmodel_config.tpl.json", 16, false);
    const auto elapsed = std::chrono::steady_clock::now() - started;
    EXPECT_EQ(jobs.size(), 2);
    EXPECT_LT(elapsed, std::chrono::seconds(2));
    EXPECT_EQ(vfm::StaticHelper::readFile("solver_calls.txt"), "nuXmv\nnuXmv\n");
    EXPECT_FALSE(fs::exists("generated_config_unused/debug_trace_array.txt"));
}
#endif

TEST(StaticTests, StaticHelper) { 

    std::string capitalized = vfm::StaticHelper::firstLetterCapital("abcd");
    ASSERT_EQ(capitalized, "Abcd");

}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
