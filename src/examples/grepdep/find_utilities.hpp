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

#ifndef MIGRATION_GREPDEP_FIND_UTILITIES_HPP
#define MIGRATION_GREPDEP_FIND_UTILITIES_HPP

#include "exec.hpp"
#include "string_utilities.hpp"

#include <algorithm>
#include <cassert>
#include <iostream>
#include <iterator>
#include <string>

namespace
{
/// add to any grep command to filter out results that don't use the include command or are commented out
void appendFilterForRealIncludes(std::string& s,
                                 const std::string& include_command,
                                 const std::string& comments_pattern) noexcept
{
    s += " | grep -F '" + include_command + "' | grep -vxE '[[:blank:]]*(" + comments_pattern + ".*)?'";
}
} // namespace

/// find a file in a given path, returns all occurrences
inline std::vector<std::string> findFiles(const std::string& filename, const std::string& path)
{
    const std::string command = "find " + path + " -type f -name \"" + filename + "\"" //
                                + " | grep -oP '(?<=" + path + ").*'"; // only return everything after the path;
    return split(exec(command), '\n');
}

/// find all files that include a certain file and match the specified options (path etc.)
inline std::vector<std::string> findFilesWithInclude(const std::string& filename,
                                                     const std::string& folder,
                                                     const std::vector<std::string>& exclude_patterns,
                                                     const std::vector<std::string>& include_patterns,
                                                     const std::vector<std::string>& exclude_directories,
                                                     const std::string& include_command,
                                                     const std::string& comments_pattern) noexcept
{
    using namespace std::string_literals;

    std::string ex_argument;
    for (const auto& pattern : exclude_patterns)
    {
        ex_argument += "--exclude '"s + pattern + "' ";
    }

    std::string in_argument;
    for (const auto& pattern : include_patterns)
    {
        in_argument += "--include '"s + pattern + "' ";
    }

    std::string ex_dir_argument;
    for (const auto& pattern : exclude_directories)
    {
        ex_dir_argument += "--exclude-dir '"s + pattern + "' ";
    }
    std::string command = "grep -rInH "s + in_argument + ex_argument + ex_dir_argument //
                          + "'" + filename + "'" + " " + folder;

    appendFilterForRealIncludes(command, include_command, comments_pattern);
    const std::string grep_output = exec(command);
    const auto grep_lines = split(grep_output, '\n');

    std::vector<std::string> result;
    result.reserve(grep_lines.size());
    std::transform(grep_lines.begin(), grep_lines.end(), std::back_inserter(result), [](const std::string& s) {
        // transform from format "path_and_filename:linenumber:text"
        const auto split_at_column = split(s, ':');
        return split_at_column[0];
    });
    return result;
}

/// find all files that include a certain file and match the specified options (path etc.)
inline std::vector<std::string> findIncludesWithinFile(const std::string& filename,
                                                       const std::string& path,
                                                       const std::string& include_command,
                                                       const std::vector<std::string>& exclude_patterns_reg) noexcept
{
    using namespace std::string_literals;
    std::string command = "find "s + path                                                  // find in origin folder
                          + " -name " + filename                                           // search for target filename
                          + " | xargs grep -xE '[[:blank:]]*" + include_command + "(.)*'"; // grep its includes


    for (const auto& pattern : exclude_patterns_reg)
    {
        command += " | grep -v '"s + pattern + "' "; // exclude to-be-excluded stuff
    }
    // isolate what is within the delimiters "" or <> and ends with *pp or .h
    command += " | grep -hoE '\"(.)*\\..pp\"|\"(.)*\\.h\"|<(.)*\\..pp>|<(.)*\\.h>'"s;

    const std::string grep_output = exec(command);
    const std::string cleaned_grep_output = //
        replaceAll(                         // exchange windows \ for linux / (yes that happened...)
            replaceAll(                     // remove delimiters ""
                replaceAll(                 // remove delimiter <
                    replaceAll(             // remove delimiter >
                        grep_output,
                        ">",
                        ""),
                    "<",
                    ""),
                "\"",
                ""),
            "\\",
            "/");
    const auto cleaned_includes = split(cleaned_grep_output, '\n');

    std::vector<std::string> path_and_filenames;
    path_and_filenames.reserve(cleaned_includes.size());

    if (g_verbose)
    {
        std::string cleaned_includes_str;

        for (const auto& s : cleaned_includes)
        {
            cleaned_includes_str += s + " ";
        }

        std::cerr << "\tTransforming... [(cleaned includes: " << cleaned_includes_str << "), path: " << path << "]"
                  << std::endl;
    }

    for (const auto& include_candidate : cleaned_includes)
    {
        const auto found_files = findFiles(split(include_candidate, '/').back(), path);
        // note: currently, the candidates contain false positives, fixing this is a minor priority // TODO
        if (not found_files.empty())
        {
            path_and_filenames.emplace_back(found_files.front()); // first come, first serve
        }
    }

    if (g_verbose)
    {
        std::string path_and_filenames_str;

        for (const auto& s : path_and_filenames)
        {
            path_and_filenames_str += s + " ";
        }

        std::cerr << "\t...Transforming ended [(paths and file names: " << path_and_filenames_str << ")]." << std::endl;
    }

    return path_and_filenames;
}

/// find all direct relatives to a file (hpp->inl->cpp->test)
inline std::vector<std::string> findSiblings(const std::string& filename, const std::string& path) noexcept
{
    const auto split_on_dot = split(filename, '.');
    const std::string base_filename = split_on_dot[0];

    assert(base_filename.length() > 2);

    const auto split_on_underscore = split(filename, '_');
    const std::string preamble = (split_on_underscore.size() > 1 ? split_on_underscore[0] : "");
    const std::string preamble_free_filename = base_filename.substr(preamble.size());
    const std::string command = std::string("find ") + path              // search in originating folder
                                + " -name *" + base_filename + ".inl"    // for same-named inls
                                + " -o -name *" + base_filename + ".cpp" // and cpps (with preamble)
                                + " -o -name *" + preamble + "*test" + preamble_free_filename
                                + ".cpp"                               // and tests (matches test and unittest preamble)
                                + " | grep -oP '(?<=" + path + ").*'"; // finally, take only everything after the path

    return split(exec(command), '\n');
}

#endif // MIGRATION_GREPDEP_FIND_UTILITIES_HPP
