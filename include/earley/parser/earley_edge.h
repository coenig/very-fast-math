//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file
#pragma once
#include "failable.h"
#include <string>
#include <vector>
#include <memory>
#include <map>
#include <set>
#include <deque>

namespace vfm {
namespace earley {

const std::string EARLEY_EDGE_FAILABLE_NAME = "EarleyEdge";

class Edge;

using EdgeList = std::vector<std::shared_ptr<Edge>>;
using EdgeListList = std::vector<EdgeList>;

class Edge : Failable {
public:
   Edge(
      const int start,
      const int end,
      const int dot,
      const std::string& lhs,
      const std::deque<std::string> rhs, // Intentional no ref, use copy here.
      const EdgeList& children,
      const bool markedL,
      const bool markedR);

   Edge(const std::shared_ptr<Edge> active, const std::shared_ptr<Edge> inactive, const bool markedL, const bool markedR);
   Edge(const std::shared_ptr<Edge> edge);

   static std::string toString(const EdgeList& elist, const bool multiLetterSymbolsHaveIndex = false, const bool latex = false);
   static std::string toString(const EdgeListList& elist, const bool multiLetterSymbolsHaveIndex = false, const bool latex = false);

   /**
   * Checks whether this active edge matches the given inactive edge.
   */
   bool matches(const Edge& inactive) const;

   /**
   * Checks whether this edge spans over the entire input sequence.
   */
   bool isOverspanning(int token_size) const;

   /**
    * Checks whether the edge has real children or just text.
    */
   bool hasRealChildren() const;
   bool isMarkedL() const;
   void setMarkedL(bool marked);
   bool isMarkedR() const;
   void setMarkedR(bool markedR);
   int getStart() const;
   int getEnd() const;
   std::string getLhs() const;
   bool isActive() const;
   bool equal(const Edge& other) const;

   std::string toString() const;
   std::string toString(const std::string& id, const int level2, const bool useIndexForMultiSymbolWords, std::map<int, std::set<std::string>>& same_level_nodes) const;
   EdgeListList getDerivation(EdgeListList& soFar);

   const std::string lhs_;
   std::deque<std::string> rhs_;

   std::shared_ptr<Edge> copy() const;

private:
   static std::string replaceSpecialCharsHTML(const std::string& sString);
   static std::string replaceSpecialCharsLATEX(const std::string& sString);

   const int start_;
   const int end_;
   const int dot_;

   EdgeList children_;

   bool markedL_;
   bool markedR_;
};

} // earley
} // vfm
