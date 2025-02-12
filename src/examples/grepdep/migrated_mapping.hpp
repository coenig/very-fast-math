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

#ifndef MIGRATION_MIGRATED_MAPPING_HPP
#define MIGRATION_MIGRATED_MAPPING_HPP

#include "exec.hpp"
#include "grepdep_options.hpp"
#include "string_utilities.hpp"

#include <cstddef>
#include <fstream>
#include <iomanip>
#include <map>
#include <sstream>

/// map of already migrated files; key: filename, value: current location
using MigratedMapping = std::map<std::string, std::string>;

namespace
{
/// Finds the destinations in the new repo using the git history.
/// Example: git log --format="" --name-only --follow --decorate=no
/// staging/fusion_common/include/fusion_common/params/repSurfErrCorrParameters.hpp | grep dc_temp
inline std::string getOriginInDCTemp(const std::string& file,
                                     const std::string& path_to_main_repo,
                                     const std::string& path_to_dc_temp_folder)
{
    // remove path to main repo (if any) from filename
    const std::string filename = replaceAll(file, path_to_main_repo, "");
    // here we explitely tell git where the repo is, this makes the script independent form being run there
    // and this is the reason why we needed to remove the path before (git log takes relative paths)
    const std::string command = "git -C " + path_to_main_repo                                              //
                                + " log --format=\"\" --name-only --follow --decorate=no -- " + filename + //
                                " | grep " + path_to_dc_temp_folder;
    const auto result = split(exec(command), '\n');

    return result.empty() ? "" : result.back();
}

inline void sortAndTrimCache(const std::string& cache_filename)
{
    exec("sort -u " + cache_filename + " -o " + cache_filename);
}

} // namespace

/// Populates the mapping of files for which a destination in the target repo is known.
inline std::size_t addToCache(MigratedMapping& migrated_mapping,
                              const std::string& path_to_main_repo,
                              const std::vector<std::string>& paths_in_main_repo,
                              const std::string& path_to_dc_temp_folder)
{
    std::size_t number_of_added{0U};
    std::size_t number_of_known{0U};
    for (const auto& sub_path : paths_in_main_repo)
    {
        std::cout << "\nPopulating list of files already migrated to '" + sub_path + "'." << std::endl;

        const std::string path = sanitizePath(sub_path);
        const auto file_list = split(                         //
            exec("find " + path_to_main_repo +                //
                 " -path \"*/" + sanitizePath(path) + "*\"" + //
                 " -and -not -path \"*/build/*\"" +           //
                 " -and -not -path \"*/.git/*\"" +            //
                 " -type f"),
            '\n');

        const auto number_of_files = file_list.size();
        std::size_t nth_file{0U};

        auto progresDump = [&nth_file, &number_of_files](const std::string& text) -> std::string {
            return "\r[" + format(static_cast<float>(100.F * nth_file / number_of_files), 3, 0) + "%] " + text;
        };

        for (const auto& file_raw : file_list)
        {
            ++nth_file;
            const std::string origin = getOriginInDCTemp(file_raw, path_to_main_repo, path_to_dc_temp_folder);
            const std::string file = replaceAll(file_raw, path_to_main_repo, "");

            if (!origin.empty())
            {
                // we are only interested in the identifier = original file name
                const auto split_at_slash = split(origin, '/');
                const auto identifier = split_at_slash.back();
                if (migrated_mapping.count(identifier))
                {
                    ++number_of_known;
                    std::cout << progresDump("known: " + file) << std::endl;
                }
                else
                {
                    migrated_mapping[identifier] = file;
                    ++number_of_added;
                    std::cout << progresDump("added: " + file);
                    std::cout << "\n     <<< " << origin << std::endl;
                }
            }
            else
            {
                std::cout << "\r" << std::string(140, ' ');                   // clear line from last time
                std::cout << progresDump("processing:" + file) << std::flush; // only overwrite, no newline
            }
        }
        std::cout << progresDump("done with " + std::to_string(number_of_files) + " files; " //
                                 + std::to_string(number_of_added) + " newly added; "        //
                                 + std::to_string(number_of_known) + " were known")
                  << std::endl;
    }
    return number_of_added;
}

inline void writeToCache(const MigratedMapping& mm, const std::string& cache_filename)
{
    std::ofstream out_stream(cache_filename, std::ios::out);
    for (const auto& entry : mm)
    {
        out_stream << entry.first << "," << entry.second << "\n";
        if (g_verbose)
        {
            std::cerr << "writing  to  cache: " << entry.first << "," << entry.second << "\n";
        }
    }
    out_stream.close();
    sortAndTrimCache(cache_filename);
}

inline void readFromCache(MigratedMapping& mm, const std::string& cache_filename)
{
    // reading from checked-in file
    std::ifstream in_stream(cache_filename, std::ios::in);
    for (std::string line; std::getline(in_stream, line);)
    {
        const auto tokens = split(line, ',');
        if (tokens.size() == 2)
        {
            mm[tokens[0]] = tokens[1];
            if (g_verbose)
            {
                std::cerr << "reading from cache: " << tokens[0] << "," << tokens[1] << "\n";
            }
        }
        else
        {
            std::cerr << "malformed cache file line:";
            for (const auto& token : tokens)
            {
                std::cerr << "\n" << token;
            }
            std::cerr << std::endl;
        }
    }
}

#endif // MIGRATION_MIGRATED_MAPPING_HPP
