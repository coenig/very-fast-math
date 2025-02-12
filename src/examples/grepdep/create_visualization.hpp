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

#ifndef MIGRATION_GREPDEP_CREATE_VISUALIZATION_HPP
#define MIGRATION_GREPDEP_CREATE_VISUALIZATION_HPP

#include "dependency_node.hpp"
#include "exec.hpp"
#include "grepdep_options.hpp"

#include <fstream>

static const std::string NEWLINE_GRAPHVIZ = "&#92;n";

/// returns string meant for generating a graphviz pdf
static std::string getGraphviz(const DependencyNode& node, const std::string level = "N")
{
    if (g_verbose)
    {
        std::cerr << "getGraphviz for '" << node.identifier << "'" << std::endl;
    }
    std::string s;

    if (node.identifier != "ROOT")
    {
        std::string add_arg;
        std::string label = "label=\"" + node.identifier;

        if (!node.migrated_from.empty())
        {
            label += NEWLINE_GRAPHVIZ + "from: " + node.migrated_from;
        }
        if (!node.migrated_to.empty())
        {
            label += NEWLINE_GRAPHVIZ + "  to: " + node.migrated_to;
            add_arg += "style=filled fillcolor=gray";
        }
        if (!node.notes.empty())
        {
            label += NEWLINE_GRAPHVIZ + NEWLINE_GRAPHVIZ + node.notes;
        }

        // if (!valid_)
        // {
        //     add_arg = "shape=plaintext";
        // }

        s += level + " [" + label + "\" " + add_arg + "];\r\n";
    }

    for (auto i = 0U; i < node.children.size(); ++i)
    {
        const std::string lev_new = level + "_" + std::to_string(i);
        if (node.identifier != "ROOT")
        {
            s += level + "->" + lev_new + ";\r\n";
        }
        s += getGraphviz(*node.children.at(i), lev_new);
    }

    return s;
}


inline void
createPDF(const DependencyNode& root_node, const std::string& output_filename, const bool open_generated_pdf)
{
    std::ofstream file;
    const std::string pdf_filename = output_filename + ".pdf";

    file.open(output_filename);
    std::string s;
    s += "digraph G {\r\n";
    s += "rankdir=LR;\r\n";
    s += "node [shape=ellipse];\r\n";
    s += getGraphviz(root_node);
    s += "}\r\n";
    file << s;
    file.close();

    exec(("dot -Tpdf -o " + pdf_filename + " " + output_filename));

    if (open_generated_pdf)
    {
        exec(("xdg-open " + pdf_filename));
    }
}

#endif // MIGRATION_GREPDEP_CREATE_VISUALIZATION_HPP
