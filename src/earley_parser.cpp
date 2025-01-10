//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file

#include "earley/parser/earley_parser.h"
#include "static_helper.h"
#include <memory>

using namespace vfm::earley;

vfm::earley::Parser::Parser(const Grammar& grammar)
   : Parsable("EarleyParser"), grammar_(grammar)
{
}

bool vfm::earley::Parser::parseProgram(const std::vector<std::string>& tokens)
{
   int pos = 0;

   for (int i = 0; i < tokens.size(); i++) {
      const std::string tok = tokens[i];

      for (const auto& lhs : grammar_.getLHS(tok)) { // initialize with input token
         auto edge_ptr = std::make_shared<Edge>(i, i + 1, 1, lhs, std::deque<std::string>{ tok }, EdgeList{}, false, false);
         chart_.push_back(edge_ptr);
         addDebug("Adding edge: " + chart_.at(chart_.size() - 1)->toString());
      }

      bool change;
      do {
         const int len = chart_.size();
         change = ruleInvocation(pos, tokens);
         pos = len;
         change |= fundamentalRule();
      } while (change);
   }

   for (const auto& e : chart_) {
      if (e->isActive()) {
         addDebug("Active: " + e->toString());
      } else {
         addDebug("Inactive: " + e->toString());
         if (e->isOverspanning(tokens.size())) {
            parse_trees_from_last_calculation_.push_back(ParseTree(e));
         }
         else {
            broken_parse_trees_from_last_calculation_.push_back(ParseTree(e));
         }
      }
   }

   return !parse_trees_from_last_calculation_.empty();
}

bool vfm::earley::Parser::parseProgram(const std::string& program)
{
   auto tokens = grammar_.getTerminalDummyTokenString(*StaticHelper::tokenizeSimple(program));
   return parseProgram(tokens);
}

std::vector<ParseTree>& vfm::earley::Parser::getParseTreesFromLastCalculation()
{
   return parse_trees_from_last_calculation_;
}

std::vector<ParseTree>& vfm::earley::Parser::getBrokenParseTreesFromLastCalculation()
{
   return broken_parse_trees_from_last_calculation_;
}

bool vfm::earley::Parser::fundamentalRule()
{
   bool change = false;

   for (int i = 0; i < chart_.size(); i++) {
      auto e = std::make_shared<Edge>(chart_.at(i));
      if (e->isActive()) {
         for (int k = 0; k < chart_.size(); k++) {
            auto e2 = std::make_shared<Edge>(chart_.at(k));
            if (!e2->isActive() && e->matches(e2)) {
               auto nw = std::make_shared<Edge>(e, e2, false, false);

               if (!contains(chart_, nw)) {
                  chart_.push_back(nw);
                  change = true;
                  addDebug("Adding edge: " + nw->toString());
               }
            }
         }
      }
   }

   return change;
}

bool vfm::earley::Parser::ruleInvocation(const int pos, const std::vector<std::string>& tokens)
{
   bool change = false;

   for (int i = pos; i < chart_.size(); i++) {
      auto e = chart_.at(i);
      if (!e->isActive()) {
         for (const auto& lhs : grammar_.withLeftmost(e->lhs_)) {
            for (const auto& rhs : grammar_.rhs(lhs)) {
               if (rhs[0] != e->lhs_ || rhs.size() > tokens.size() - e->getStart()) {
                  continue;
               }

               auto nw = std::make_shared<Edge>(e->getStart(), e->getEnd(), 1, lhs, rhs, EdgeList{e}, false, false);

               if (!contains(chart_, nw)) {
                  chart_.push_back(nw);
                  change = true;
                  addDebug("Adding edge: " + nw->toString());
               }
            }
         }
      }
   }

   return change;
}

std::vector<ParseTree> vfm::earley::Parser::parse(Grammar& grammar, const std::vector<std::string>& program)
{
   grammar.declareTrailingSymbolsAsTerminal();
   return parseConst(grammar, program);
}

std::vector<ParseTree> vfm::earley::Parser::parse(Grammar& grammar, const std::string& program)
{
   grammar.declareTrailingSymbolsAsTerminal();
   return parseConst(grammar, program);
}

std::vector<ParseTree> vfm::earley::Parser::parseConst(const Grammar & grammar, const std::string & program)
{
   auto parser = Parser(grammar);
   parser.parseProgram(program);
   return parser.parse_trees_from_last_calculation_;
}

std::vector<ParseTree> vfm::earley::Parser::parseConst(const Grammar & grammar, const std::vector<std::string>& program)
{
   auto parser = Parser(grammar);
   parser.parseProgram(program);
   return parser.parse_trees_from_last_calculation_;
}

bool vfm::earley::Parser::contains(const EdgeList& list, const std::shared_ptr<Edge> edge)
{
   for (const auto& el : list) {
      if (el->equal(edge)) {
         return true;
      }
   }

   return false;
}
