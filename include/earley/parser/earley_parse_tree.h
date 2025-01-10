//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file
#pragma once
#include "earley/parser/earley_edge.h"
#include <memory>
#include <map>
#include <set>

namespace vfm {
namespace earley {

class ParseTree {
public:
   ParseTree(const std::shared_ptr<Edge> root_edge);

   std::shared_ptr<Edge> getEdge();
   std::string toString(const bool useIndexForMultiSymbolWords = false) const;
   EdgeListList getDerivation() const;

private:
   mutable std::map<int, std::set<std::string>> sameLevelNodes_;
   std::shared_ptr<Edge> root_edge_;
};

} // earley
} // vfm
