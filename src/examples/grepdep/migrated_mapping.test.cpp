//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2021 Robert Bosch GmbH.
//
// The reproduction, distribution and utilization of this file as
// well as the communication of its contents to others without express
// authorization is prohibited. Offenders will be held liable for the
// payment of damages. All rights reserved in the event of the grant
// of a patent, utility model or design.
//============================================================================================================

//#include <gtest>

#include "migrated_mapping.hpp"

#include <cassert>
// TEST(MigratedMappingTest, one)
//{

int main(int argc, char* argv[])
{
    g_verbose = true; // activate debug output
    MigratedMapping cached_mapping;

    const auto initially_added = addToCache(cached_mapping,
                                            "/workspace/",                              // path_to_main_repo
                                            {"tools/migration/grepdep/test/migrated/"}, // paths_in_main_repo,
                                            "tools/migration/grepdep/test/unmigrated/"  // path_to_dc_temp_folder
    );
    assert(initially_added == 1);

    writeToCache(cached_mapping, "test.cache");

    MigratedMapping new_mapping;
    readFromCache(new_mapping, "test.cache");

    const auto newly_added = addToCache(new_mapping,
                                        "/workspace/",                              // path_to_main_repo
                                        {"tools/migration/grepdep/test/migrated/"}, // paths_in_main_repo,
                                        "tools/migration/grepdep/test/unmigrated/"  // path_to_dc_temp_folder
    );
    assert(newly_added == 0);

    return 1;
}

// EXPECT_TRUE( it.first == "old_a.hpp" );
// EXPECT_TRUE( it.first == "tools/migration/grepdep/test/migrated/new_a.hpp" );
// for ( const auto& it : migrated_mapping){
//   EXPECT_TRUE( it.second == cached_mapping.at(it.first) );
// }
// }
