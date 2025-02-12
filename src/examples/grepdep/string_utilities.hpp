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

#ifndef MIGRATION_GREPDEP_STRING_UTILITIES_HPP
#define MIGRATION_GREPDEP_STRING_UTILITIES_HPP

#include <iomanip>
#include <sstream>
#include <string>
#include <type_traits>
#include <vector>

/// Split string at all occurrences of delim.
template <typename Out>
inline void split(const std::string& s, char delim, Out result)
{
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, delim))
    {
        *(result++) = item;
    }
}

/// Split string at all occurrences of delim (convenience function with return).
inline std::vector<std::string> split(const std::string& s, char delim)
{
    std::vector<std::string> elems;
    split(s, delim, std::back_inserter(elems));
    return elems;
}

/// Replace in str all occurrences of from with to.
inline std::string replaceAll(std::string str, const std::string& from, const std::string& to)
{
    std::size_t start_pos = 0;
    while ((start_pos = str.find(from, start_pos)) != std::string::npos)
    {
        str.replace(start_pos, from.length(), to);
        start_pos += to.length(); // Handles case where 'to' is a substring of 'from'
    }
    return str;
}

inline bool stringEndsWith(std::string const& full_string, std::string const& ending)
{
    if (full_string.length() >= ending.length())
    {
        return (0
                == full_string.compare(full_string.length() - ending.length(), //
                                       ending.length(),                        //
                                       ending));
    }
    else
    {
        return false;
    }
}

inline std::string sanitizeForGraphViz(const std::string& str)
{
    return replaceAll(  //
        replaceAll(     //
            replaceAll( //
                str,
                "/",
                "\\/"),
            ".",
            "\\."),
        "\"",
        "\\\"");
}

inline std::string undosanitizeForGraphViz(const std::string& str)
{
    return replaceAll(str, "\\", "");
}

/// add a final slash after path
inline std::string sanitizePath(const std::string& path)
{
    std::string result = path;
    if (result.back() != '/')
    {
        result.push_back('/');
    }
    return result; // NRVO
}

template <typename T, typename = std::enable_if_t<std::is_integral_v<T>>>
inline std::string format(const T integer, const std::int32_t width = 2, const bool leading_zeros = false)
{
    std::ostringstream result;
    result << std::fixed << std::setw(width);
    if (leading_zeros)
    {
        result << std::setfill('0');
    }
    result << integer;
    return result.str();
}

template <typename T, typename = std::enable_if_t<std::is_floating_point_v<T>>>
inline std::string format(const T scalar, const std::int32_t width = 6, const std::int32_t precision = 2)
{
    std::ostringstream result;
    result << std::fixed << std::setprecision(precision) << std::setw(width) << scalar;
    return result.str();
}

#endif // MIGRATION_GREPDEP_STRING_UTILITIES_HPP
