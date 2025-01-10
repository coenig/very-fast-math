//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file

#include "earley/parser/earley_edge.h"
#include "earley/earley_grammar.h"
#include "static_helper.h"

using namespace vfm::earley;

vfm::earley::Edge::Edge(
   const int start,
   const int end,
   const int dot,
   const std::string& lhs,
   const std::deque<std::string> rhs,
   const EdgeList& children,
   const bool markedL,
   const bool markedR) : Failable(EARLEY_EDGE_FAILABLE_NAME),
   start_(start),
   end_(end),
   dot_(dot),
   children_(children),
   lhs_(lhs),
   rhs_(rhs)
{
   setMarkedL(markedL);
   setMarkedR(markedR);
}

vfm::earley::Edge::Edge(const std::shared_ptr<Edge> active, const std::shared_ptr<Edge> inactive, const bool markedL, const bool markedR)
   : Edge(active->getStart(), inactive->getEnd(), active->dot_ + 1, active->lhs_, active->rhs_,
      EdgeList(active->children_), markedL, markedR)
{
   children_.push_back(inactive);
}

vfm::earley::Edge::Edge(const std::shared_ptr<Edge> edge) : 
   Edge(edge->start_, edge->end_, edge->dot_, edge->lhs_, edge->rhs_, edge->children_, edge->isMarkedL(), false)
{
}

std::string vfm::earley::Edge::toString(const EdgeListList& elist, const bool multiLetterSymbolsHaveIndex, const bool latex)
{
   std::string s = "";
   s += toString(elist.at(0), multiLetterSymbolsHaveIndex, latex) + (latex ? "&" : "");
   int currentLength = 0;

   for (int i = 1; i < elist.size(); i++) {
      std::string tempStr = "";

      if (latex && currentLength > 200) {
         tempStr = "\\\\\n&";
         currentLength = 0;
      }

      std::string newString = (latex ? " \\Rightarrow " : RULE_TRANSITION_SYMBOL_DEFAULT) + toString(elist.at(i), multiLetterSymbolsHaveIndex, latex);
      currentLength += newString.length();
      s += tempStr + newString;
   }

   return s;
}

std::string vfm::earley::Edge::toString(const EdgeList& elist, const bool multiLetterSymbolsHaveIndex, const bool latex)
{
   std::string s;
   bool inUlineMode = false;

   for (const auto& edge : elist) {
      std::string lhs = latex ? replaceSpecialCharsLATEX(edge->lhs_) : edge->lhs_;

      if (!StaticHelper::stringStartsWith(lhs, "\\") && lhs.length() > 1 && lhs != PSEUDO_EPSILON_SYMBOL_DEFAULT) {
         if (latex) {
            if (multiLetterSymbolsHaveIndex) {
               lhs = lhs[0] + "_{" + lhs.substr(1) + "}";
            }
            else {
               lhs = "\\mbox{" + lhs + "}";
               //                    lhs = "\\ " + lhs + "\\ ";
            }
         }

         if (edge->isMarkedL()) {
            lhs = "\\overline{" + lhs + "}";
         }

         if (edge->isMarkedR() && !inUlineMode) {
            lhs = "\\underline{" + lhs;
            inUlineMode = true;
         } else if (!edge->isMarkedR() && inUlineMode) {
            lhs = "}" + lhs;
            inUlineMode = false;
         }
      }

      s += lhs + " ";
   }

   if (inUlineMode) {
      s += "}";
   }

   return s;
}

bool vfm::earley::Edge::isOverspanning(int token_size) const {
   return start_ == 0 && end_ == token_size; // getStart() == 0 && getEnd() == this.edgesFatherChartParser.getTokens().length;
}

std::string vfm::earley::Edge::getLhs() const {
   return lhs_;
}

bool vfm::earley::Edge::hasRealChildren() const {
   return !children_.empty();
}

bool vfm::earley::Edge::isMarkedL() const {
   return markedL_;
}

void vfm::earley::Edge::setMarkedL(bool marked) {
   markedL_ = marked;
}

bool vfm::earley::Edge::isMarkedR() const {
   return markedR_;
}

void vfm::earley::Edge::setMarkedR(bool markedR) {
   markedR_ = markedR;
}

int vfm::earley::Edge::getStart() const {
   return start_;
}

int vfm::earley::Edge::getEnd() const {
   return end_;
}

bool vfm::earley::Edge::matches(const Edge& inactive) const {
   return getEnd() == inactive.getStart() && rhs_[dot_] == inactive.lhs_;
}

bool vfm::earley::Edge::isActive() const {
   return dot_ < rhs_.size();
}

bool vfm::earley::Edge::equal(const Edge& other) const
{
   if (children_.size() != other.children_.size()) {
      return false;
   }

   for (int i = 0; i < children_.size(); i++) {
      if (!children_.at(i)->equal(*other.children_.at(i))) {
         return false;
      }
   }

   return getStart() == other.getStart() 
      && getEnd() == other.getEnd() 
      && dot_ == other.dot_
      && lhs_ == other.lhs_ 
      && rhs_ == other.rhs_;
}

std::string vfm::earley::Edge::toString() const 
{
   std::string rhs_str = rhs_.empty() ? "" : rhs_.at(0);

   for (int i = 1; i < rhs_.size(); i++) {
      rhs_str += ", " + rhs_.at(i);
   }

   return "(" + std::to_string(getStart()) + ", " + std::to_string(getEnd()) + ", " + std::to_string(dot_) + ", " + lhs_ + ", [" + rhs_str + "])";
}

std::string vfm::earley::Edge::toString(const std::string& id, const int level2, const bool useIndexForMultiSymbolWords, std::map<int, std::set<std::string>>& same_level_nodes) const 
{
   std::string s;

   int level = level2;

   if (!hasRealChildren()) {
      level = std::numeric_limits<int>::max();
   }

   same_level_nodes.insert({ level, std::set<std::string>{} }); // For safety insert empty set if not already existing.
   std::set<std::string>& sameLevel = same_level_nodes.at(level);

   sameLevel.insert("a" + id);

   std::string style = "";

   if (!hasRealChildren()) {
      style = "shape=rectangle,";

      if (PSEUDO_EPSILON_SYMBOL_DEFAULT == lhs_) {
         style = "shape=point,";
      }
   }


   std::string sString = lhs_;

   //if (sString == ChartParser.currentStartSymbol) { // TODO: Later
   //   style = "style=\"filled\"";
   //}

   // Special symbols.
   sString = replaceSpecialCharsHTML(sString);

   if (!StaticHelper::stringStartsWith(sString, "&") || !StaticHelper::stringStartsWith(sString, ";")) {
      if (useIndexForMultiSymbolWords && sString.length() > 1 && sString[1] != '\'') {
         sString = sString[0] + "<SUB>" + sString.substr(1) + "</SUB>";
      }
   }

   s += "a" + id + " [" + style + "label=<" + sString + ">];\n";

   int i = 0;

   for (const auto& child : children_) {
      s += "a" + id + " -> a" + (id + "b" + std::to_string(i)) + ";\n";
      s += child->toString(id + "b" + std::to_string(i), level + 1, useIndexForMultiSymbolWords, same_level_nodes);
      i++;
   }

   return s;
}

EdgeListList vfm::earley::Edge::getDerivation(EdgeListList& soFar) 
{
   EdgeList lastDeriv = soFar.back();
   EdgeList newDeriv;

   for (const auto& el : lastDeriv) {
      newDeriv.push_back(el);
   }

   for (int i = 0; i < newDeriv.size(); i++) {
      auto e = newDeriv.at(i);

      if (!e->children_.empty()) {
         newDeriv.erase(newDeriv.begin() + i);
         bool isRightSideMarked = lastDeriv.at(i)->isMarkedR();
         auto leftSideEdge = std::make_shared<Edge>(lastDeriv.at(i));
         leftSideEdge->setMarkedL(true);
         leftSideEdge->setMarkedR(isRightSideMarked);

         lastDeriv[i] = leftSideEdge;

         for (int j = e->children_.size() - 1; j >= 0; j--) {
            auto edge = std::make_shared<Edge>(e->children_.at(j));
            edge->setMarkedR(true);
            newDeriv.insert(newDeriv.begin() + i, edge);
         }

         soFar.push_back(newDeriv);

         return e->getDerivation(soFar);
      }
   }

   return soFar;
}

std::shared_ptr<Edge> vfm::earley::Edge::copy() const
{
   auto cp = std::make_shared<Edge>(*this);
   cp->children_.clear();

   for (const auto& child : children_) {
      cp->children_.push_back(child->copy());
   }

   return cp;
}

std::string vfm::earley::Edge::replaceSpecialCharsHTML(const std::string & sString)
{
   std::string ssString = StaticHelper::replaceAll(
      StaticHelper::replaceAll(
         StaticHelper::replaceAll(
            StaticHelper::replaceAll(sString,
               "XplusX", "&#043;"), 
            "XsemX", "&#059;"), 
         "XkaX", "&#040;"), 
      "XkzX", "&#041;");

   return StaticHelper::replaceSpecialCharsHTML_G(ssString);
}

std::string vfm::earley::Edge::replaceSpecialCharsLATEX(const std::string& sString) {
   std::string ssString = StaticHelper::replaceAll(
      StaticHelper::replaceAll(
         StaticHelper::replaceAll(
            StaticHelper::replaceAll(
               StaticHelper::replaceAll(
                  StaticHelper::replaceAll(
                     StaticHelper::replaceAll(sString,
                        "ä", "\\\"a"),
                     "ö", "\\\"o"),
                  "ü", "\\\"u"),
               "XplusX", "+"),
            "XsemX", ";"), 
         "XkaX", "("), 
      "XkzX", ")");

   return ssString;
}
