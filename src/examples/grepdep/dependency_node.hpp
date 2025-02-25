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

#ifndef MIGRATION_DEPENDENCY_NODE_HPP
#define MIGRATION_DEPENDENCY_NODE_HPP

#include "string_utilities.hpp"

#include <algorithm>
#include <memory>
#include <ostream>
#include <string>
#include <unordered_set>
#include <vector>

/// representation of a file migrated from one place to another
struct DependencyNode
{
    std::string identifier;    ///< unique identifier is files original name
    std::string migrated_from; ///< this is the path where the file originally was
    std::string migrated_to;   ///< this is the path AND FILENAME of the file now
    std::string notes;         ///< free-from field for additional notes

    /// all nodes this one depends on (includes + siblings (e.g., hpp<->cpp))
    std::vector<std::shared_ptr<DependencyNode>> children;
};

inline bool operator==(const DependencyNode& lhs, const DependencyNode& rhs)
{
    return lhs.identifier == rhs.identifier;
}

// note: this hash in std:: namespace is needed in order for the custom type DependencyNode to work in std::unsorted_*
namespace std
{
template <>
struct hash<DependencyNode>
{
    std::size_t operator()(const DependencyNode& n) const
    {
        using std::hash;
        using std::string;
        // here we could be fancy, wer are not, just taking the identifier since it should be unique
        return (hash<string>()(n.identifier));
    }
};
} // namespace std


using DependencyNodes = std::vector<DependencyNode>;
using DependencySet = std::unordered_set<DependencyNode>;


inline DependencyNode dependencyNodeFromOriginalInclude(const std::string& path_and_filename)
{
    DependencyNode result;
    if (!path_and_filename.empty())
    {
        result.identifier = split(path_and_filename, '/').back(); // last entry is file name without path
        result.migrated_from = path_and_filename.substr(0, path_and_filename.length() - result.identifier.length());
    }
    return result; // NRVO
}

inline DependencyNodes dependencyNodesFromOriginalIncludes(const std::vector<std::string>& path_and_filenames)
{
    DependencyNodes result;
    result.reserve(path_and_filenames.size());
    std::transform(path_and_filenames.begin(),
                   path_and_filenames.end(),   //
                   std::back_inserter(result), //
                   dependencyNodeFromOriginalInclude);

    return result; // NRVO
}

inline DependencySet dependencySetFromOriginalIncludes(const std::vector<std::string>& path_and_filenames)
{
    DependencySet result;
    for (const auto& path_and_filename : path_and_filenames)
    {
        result.insert(dependencyNodeFromOriginalInclude(path_and_filename));
    }

    return result; // NRVO
}

inline std::ostream& operator<<(std::ostream& os, const DependencyNode& n)
{
    os << n.identifier;
    if (!n.migrated_from.empty())
    {
        os << "\n└from:" << n.migrated_from;
    }
    if (!n.migrated_to.empty())
    {
        os << "\n└  to:" << n.migrated_to;
    }
    if (!n.notes.empty())
    {
        os << "\n└note:" << n.notes;
    }
    return os;
}

inline std::ostream& operator<<(std::ostream& os, const DependencyNodes& nn)
{
    for (const auto n : nn)
    {
        os << n << "\n";
    }
    return os;
}

inline std::ostream& operator<<(std::ostream& os, const DependencySet& nn)
{
    for (const auto n : nn)
    {
        os << n << "\n";
    }
    return os;
}

#endif // MIGRATION_DEPENDENCY_NODE_HPP
