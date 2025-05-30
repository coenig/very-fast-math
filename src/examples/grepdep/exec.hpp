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

#ifndef MIGRATION_GREPDEP_EXEC_HPP
#define MIGRATION_GREPDEP_EXEC_HPP

#include "grepdep_options.hpp"

#include <array>
#include <iostream>
#include <memory>
#include <string>

/// Execute the given cmd on system level, and return the output generated by it.
inline std::string exec(const std::string cmd)
{
    if (g_verbose)
    {
        std::cout << "╔═══ Executing command\n" << cmd << std::endl;
    }

    std::array<char, 128> buffer;
    std::string result;
    std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd.c_str(), "r"), pclose);

    if (!pipe)
    {
        throw std::runtime_error("popen() failed!");
    }

    while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr)
    {
        result += buffer.data();
    }

    if (g_verbose)
    {
        std::cout << "╠═══ Returned output\n" << result;
        std::cout << "╚═══ done." << std::endl;
    }

    return result;
}

#endif // MIGRATION_GREPDEP_EXEC_HPP
