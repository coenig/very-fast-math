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

#include "create_dependency_tree.hpp"
#include "create_visualization.hpp"
#include "exec.hpp"
#include "grepdep_options.hpp"
#include "input_parser.hpp"
#include "migrated_mapping.hpp"
#include "string_utilities.hpp"

#include <algorithm>
#include <array>
#include <cstdio>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>


void termnate(const int exit_type = 0) // This is an intended typo to resolve name clashes.
{
    std::cout << std::endl << "<TERMINATED>" << std::endl;

#ifdef _WIN32
    std::cin.get(); // Prevent Windows from closing the terminal immediately after termination.
#endif

    std::exit(exit_type);
}

int main(int argc, char* argv[])
{
    std::cout << "### grepdep V_1.4 -- dep trace for PACE ###" << std::endl;
    std::cout << "### Disclaimer: grepdep does not compile the code, so you might get both a superset and a subset of "
                 "the actual dependencies. The latter can be caused if you use too strict ignore patterns. "
                 "Nonetheless, the result is usually quite accurate. ###"
              << std::endl;
    std::cout << "### Contact ###" << std::endl
              << "Koenig Lukas (CC-XD/EYC6) <lukas.koenig@de.bosch.com>" << std::endl
              << "Heinrich Benjamin (XC-AD/EFB3) <benjamin.heinrich@de.bosch.com>" << std::endl
              << std::endl;

    InputParser parser = createInputParser(argc, argv);
    if (parser.error_ocurred_)
    {
        std::cout << "Mandatory parameter missing. I will terminate." << std::endl;
        termnate(1);
    }

    g_verbose = parser.getCmdOption(CMD_VERBOSE) == "true";
    // TODO: fix filename parsing generically
    const std::string repo_target = sanitizePath(parser.getCmdOption(CMD_REPO_TARGET));
    const std::string migrated_mapping_cache_filename = parser.getCmdOption(CMD_CACHE_FILENAME);

    MigratedMapping migrated_mapping;
    readFromCache(migrated_mapping, migrated_mapping_cache_filename);
    if (parser.getCmdOption(CMD_CACHE_MIGRATED) == "true")
    {
        addToCache(migrated_mapping,
                   repo_target,                                           // path to main repo
                   split(parser.getCmdOption(CMD_FOLDERS_MIGRATED), ','), // paths in main repo
                   parser.getCmdOption(CMD_FOLDER_INTERMEDIATE));         // path to dc_temp folder
        writeToCache(migrated_mapping, migrated_mapping_cache_filename);
    }
    else
    {
        std::cout << "Omitting calculation of already migrated files, only using cached files." << std::endl;
    }

    const std::string root_filenames = parser.getCmdOption(CMD_FILES);
    if (split(root_filenames, ',').size() > 1U)
    {
        // TODO : re-add feature of building trees with multiple roots
        std::cerr << "checking for multiple root files at once is currently disabled" << std::endl;
        termnate(1);
    }

    GrepDepOptions o;
    o.comments_pattern = parser.getCmdOption(CMD_PATTERN_COMENTS);
    o.include_command = parser.getCmdOption(CMD_INCLUDE_COMMAND);
    o.path_to_root_folder = sanitizePath(parser.getCmdOption(CMD_REPO_ORIGIN));
    o.exclude_directories = split(parser.getCmdOption(CMD_EXCLUDE_DIRS), ',');
    o.exclude_patterns = split(parser.getCmdOption(CMD_EXCLUDE_PATTERNS), ',');
    o.exclude_patterns_reg = split(parser.getCmdOption(CMD_EXCLUDE_REG) + HARD_CODED_EXCLUDES_REG, ',');
    o.include_patterns = split(parser.getCmdOption(CMD_INCLUDE_PATTERNS), ',');
    o.reverse = parser.getCmdOption(CMD_REVERSE) == "true";
    o.show_migrated = parser.getCmdOption(CMD_SHOW_MIGRATED) == "true";

    const std::string root_filename = root_filenames;
    const auto files = findFiles(root_filename, o.path_to_root_folder);
    if (files.empty())
    {
        std::cerr << "could not find '" << root_filename << "' in '" << o.path_to_root_folder << "'" << std::endl;
        termnate(1);
    }
    const auto root_path_and_filename = files.back();
    const std::size_t num_files = files.size();
    std::string additional_info{""};
    if (num_files == 0U)
    {
        std::cerr << "WARNING: No files named '" << root_filename << "' found." << std::endl;
        termnate(1);
    }
    else if (num_files > 1U)
    {
        if (g_verbose)
        {
            std::cout << "WARNING: There are " << num_files << " files named '" << root_filename << "' in '"
                      << o.path_to_root_folder
                      << "'. I will search for includes of all these files and treat them as if they were the same."
                      << std::endl;
        }

        additional_info = NEWLINE_GRAPHVIZ + "(representing " + std::to_string(num_files) + " files with same name)";
    }

    DependencyNode root;
    root.identifier = root_filename;
    root.notes = "*ROOT*" + additional_info;
    root.migrated_from = root_path_and_filename.substr(0, root_path_and_filename.size() - root_filename.size());
    const auto tree = createDependencyTree(root, migrated_mapping, o);

    std::cout << std::endl
              << "### These are all the dependencies of '" << root_filenames << "' I found ###" << std::endl;

    const std::string target_repo_path = sanitizePath(repo_target + parser.getCmdOption(CMD_FOLDER_INTERMEDIATE));

    for (const auto& [key, node] : tree.nodes())
    {
        if (node->migrated_to.empty())
        {
            const std::string copy_from = sanitizePath(o.path_to_root_folder + node->migrated_from) + node->identifier;
            const std::string copy_to = target_repo_path + node->migrated_from;
            if (parser.getCmdOption(CMD_MOVE) == "true")
            {
                exec("mkdir -p " + copy_to);
                exec("mv " + copy_from + " " + copy_to);

                std::cout << "moving " << copy_from << " to " << copy_to << std::endl;
            }
            else if (parser.getCmdOption(CMD_COPY) == "true")
            {
                exec("mkdir -p " + copy_to);
                exec("cp -n " + copy_from + " " + copy_to);

                std::cout << "copying " << copy_from << " to " << copy_to << std::endl;
            }
            else if (g_verbose)
            {
                std::cout << "would copy " << copy_from << " to " << copy_to << std::endl;
            }
        }
    }
    std::cout << "### EO all the dependencies of '" << root_filenames << "' I found ###" << std::endl;

    if (parser.getCmdOption(CMD_SHOW_PDF) == "true" || parser.getCmdOption(CMD_CREATE_PDF) == "true")
    {
        auto filename = replaceAll(root_filenames, ",", "__") + ".dep_graph.dot";
        std::cout
            << std::endl
            << "Creating graph '" << filename
            << "'... Note that the graph is cut off on dependencies that are already displayed at some other node."
            << std::endl;

        if (parser.getCmdOption(CMD_REVERSE) != "true")
        {
            std::cout << "Note that cpp dependencies are added to the dependencies of the according hpp file "
                         "(including the hpp itself)."
                      << std::endl;
        }

        assert(tree.root() != nullptr);
        const auto root_node = *tree.root();
        createPDF(root_node,                                    // root_node
                  filename,                                     // output_filename
                  parser.getCmdOption(CMD_SHOW_PDF) == "true"); // open_generated_pdf
    }

    termnate();

    return 0;
}
