#include "static_helper.h"
#include "testing/test_functions.h"
#include "vfmacro/script.h"
#include <gtest/gtest.h>
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
