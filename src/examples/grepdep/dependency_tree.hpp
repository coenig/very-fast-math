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

#ifndef MIGRATION_DEPENDENCY_TREE_HPP
#define MIGRATION_DEPENDENCY_TREE_HPP

#include "dependency_node.hpp"
#include "grepdep_options.hpp"
#include "migrated_mapping.hpp"

#include <ios>
#include <iostream>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <unordered_map>


/// The DependencyTree manages the DependencyNotes
/// * it holds all nodes in a map (in order facilitate duplicate findings)
/// * it keeps track of a candidate `open` list (also a set to remove duplicates)
/// * for convenience, it also --optionally-- holds the map of already migrated
class DependencyTree
{
  public:
    using NodePtr = std::shared_ptr<DependencyNode>;
    using Set = DependencySet;
    using Key = decltype(DependencyNode::identifier);

    DependencyTree(const DependencyNode& root)
        : root_(std::make_shared<DependencyNode>(root))
        , nodes_({{root_->identifier, root_}})
    {
        // make sure root is marked as not-yet expanded
        open_.insert(root);
    }

    bool add(const DependencyNode& node, std::optional<Key> parent = {})
    {
        if (g_verbose)
        {
            std::cout << "trying to add node '" << node.identifier << "'";
        }
        auto&& [it, added] = nodes_.insert({node.identifier, std::make_shared<DependencyNode>(node)});

        // node could be inserted, hence it is new
        if (added)
        {
            // if a parent key is given, check if it is part of the tree; if so: connect them
            if (parent)
            {
                auto parent_it = nodes_.find(parent.value());
                const bool found_parent = parent_it != nodes_.end();
                if (found_parent)
                {
                    // connect child and parent
                    parent_it->second->children.emplace_back(it->second);
                }
                if (g_verbose)
                {
                    std::cout << " (found parent '" << parent.value() << "': " << std::boolalpha << found_parent << ")";
                }
            }
            // if there is a map of already migrated nodes, annotate accordingly and (optionally) cut tree
            if (!migrated_.empty())
            {
                auto migrated_it = migrated_.find(node.identifier);
                const bool is_already_migrated = migrated_it != migrated_.end();
                if (is_already_migrated)
                {
                    it->second->migrated_to = migrated_it->second;
                }
                if (g_verbose)
                {
                    std::cout << " (is migrated: " << std::boolalpha << is_already_migrated << ")";
                }
                added = !is_already_migrated || show_migrated_;
            }
            if (added)
            {
                open_.insert(*it->second);
            }
        }
        if (g_verbose)
        {
            std::cout << std::endl;
        }
        return added;
    }

    /// return set of open nodes and removes them from the internal list
    Set pop()
    {
        auto result = open_;
        open_.clear();
        return result; // NRVO
    }

    const NodePtr root() const
    {
        return root_;
    }
    const std::unordered_map<Key, NodePtr> nodes() const
    {
        return nodes_;
    }
    MigratedMapping& migrated()
    {
        return migrated_;
    }
    const MigratedMapping& migrated() const
    {
        return migrated_;
    }
    bool& showMigrated()
    {
        return show_migrated_;
    }
    std::size_t numberOpen() const
    {
        return open_.size();
    }
    std::size_t numberTotal() const
    {
        return nodes_.size();
    }

  private:
    NodePtr root_;
    Set open_;
    MigratedMapping migrated_;
    std::unordered_map<Key, NodePtr> nodes_;
    bool show_migrated_{false};
};

#endif // MIGRATION_DEPENDENCY_TREE_HPP
