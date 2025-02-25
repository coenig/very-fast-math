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

#ifndef MIGRATION_CREATE_DEPENDENCY_TREE_HPP
#define MIGRATION_CREATE_DEPENDENCY_TREE_HPP

#include "dependency_node.hpp"
#include "dependency_tree.hpp"
#include "find_utilities.hpp"
#include "grepdep_options.hpp"
#include "migrated_mapping.hpp"
#include "string_utilities.hpp"

#include <algorithm>
#include <cassert>
#include <ios>
#include <set>
#include <string>
#include <vector>

namespace
{
/// given a filename, find its dependencies
inline DependencySet findDependencies(const std::string& filename, const GrepDepOptions& o)
{
    if (g_verbose)
    {
        std::cerr << "┉┉┉┉┉┉ findDependencies for " << filename << " (reverse: " << std::boolalpha << o.reverse << ")"
                  << std::endl;
    }

    DependencySet result;
    if (o.reverse)
    {
        result = dependencySetFromOriginalIncludes( //
            findFilesWithInclude(filename,
                                 o.path_to_root_folder,
                                 o.exclude_patterns,
                                 o.include_patterns,
                                 o.exclude_directories,
                                 o.include_command,
                                 o.comments_pattern));
    }
    else
    {
        result = dependencySetFromOriginalIncludes( //
            findIncludesWithinFile(filename,        //
                                   o.path_to_root_folder,
                                   o.include_command,
                                   o.exclude_patterns_reg));
        auto&& siblings = dependencySetFromOriginalIncludes( //
            findSiblings(filename,                           //
                         o.path_to_root_folder));
        result.merge(siblings);
    }

    if (g_verbose)
    {
        std::cerr << "┉┉┉┉┉┉ found: \n" << result << std::endl;
    }

    return result;
};
} // namespace

DependencyTree
createDependencyTree(const DependencyNode& root_node, const MigratedMapping& migrated_mapping, const GrepDepOptions& o)
{
    std::cout << "┏┳━━ Building dependency tree for '" << root_node.identifier << "'" << (o.reverse ? " (reverse)" : "")
              << std::endl;

    DependencyTree tree(root_node);
    tree.migrated() = migrated_mapping;
    tree.showMigrated() = o.show_migrated;
    while (tree.numberOpen() > 0)
    {
        std::cout << "┃┣━━(" << format(tree.numberOpen(), 3) << " / " << format(tree.numberTotal(), 3)
                  << ") currently open / total" << std::endl;
        const auto open = tree.pop();
        for (const auto& candidate : open)
        {
            std::cout << "┃┃ finding dependencies for '" << candidate.identifier << "'...\n";
            const auto dependencies = findDependencies(candidate.identifier, o);

            const auto number_added =
                std::count_if(dependencies.begin(),
                              dependencies.end(), //
                              [&](const DependencyNode& child) { return tree.add(child, candidate.identifier); });
            std::cout << "┃┃     ...found: " << format(number_added, 2, false) << " new (of "
                      << format(dependencies.size(), 2, false) << " total)" << std::endl;
        }
    }
    std::cout << "┗┻━━ Finished dependency tree for '" << root_node.identifier << "'" << std::endl;

    return tree;
}

#endif // MIGRATION_CREATE_DEPENDENCY_TREE_HPP
