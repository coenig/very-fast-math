//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file

#include "earley/parser/earley_parse_tree.h"

using namespace vfm::earley;

vfm::earley::ParseTree::ParseTree(const std::shared_ptr<Edge> root_edge) : root_edge_(root_edge)
{
}

std::shared_ptr<Edge> vfm::earley::ParseTree::getEdge()
{
   return root_edge_;
}

std::string vfm::earley::ParseTree::toString(const bool useIndexForMultiSymbolWords) const
{
   sameLevelNodes_.clear();
   std::string s = "digraph G {\n" + root_edge_->toString("", 0, useIndexForMultiSymbolWords, sameLevelNodes_);


   for (const auto& pair : sameLevelNodes_) {
      int level = pair.first;
      s += "{rank=same; ";
      for (const auto& node : sameLevelNodes_.at(level)) {
         s += node + " ";
      }
      s += "};\n";
   }

   return s + "\n}";
}

EdgeListList vfm::earley::ParseTree::getDerivation() const
{
   EdgeListList list;
   EdgeList initialEdgeList;
   root_edge_->setMarkedL(true);
   initialEdgeList.push_back(root_edge_);
   list.push_back(initialEdgeList);
   return root_edge_->getDerivation(list);
}