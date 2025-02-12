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


#ifndef MIGRATION_GREPDEP_GREPDEP_OPTIONS_HPP
#define MIGRATION_GREPDEP_GREPDEP_OPTIONS_HPP

#include <string>
#include <vector>

/// dirty global variable to facilitate cout debugging
static bool g_verbose{false};

/// options parsed from the command line and used not only in main() of grepdep
struct GrepDepOptions
{
    std::string comments_pattern;
    std::string include_command;
    std::string migrated_mapping_cache_filename;
    std::string path_to_root_folder;
    std::vector<std::string> exclude_directories;
    std::vector<std::string> exclude_patterns_reg;
    std::vector<std::string> exclude_patterns;
    std::vector<std::string> include_patterns;
    bool reverse;
    bool show_migrated;
};

#endif // MIGRATION_GREPDEP_GREPDEP_OPTIONS_HPP
