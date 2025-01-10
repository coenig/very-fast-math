//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file
#pragma once
#include "parsable.h"
#include "earley/earley_grammar.h"
#include "earley/parser/earley_edge.h"
#include "earley/parser/earley_parse_tree.h"

namespace vfm {
namespace earley {

class Parser : public Parsable {
public:
   Parser(const Grammar& grammar);

   static std::vector<ParseTree> parse(Grammar& grammar, const std::string& program);
   static std::vector<ParseTree> parse(Grammar& grammar, const std::vector<std::string>& program);

   static std::vector<ParseTree> parseConst(const Grammar& grammar, const std::string& program);
   static std::vector<ParseTree> parseConst(const Grammar& grammar, const std::vector<std::string>& program);
   static bool contains(const EdgeList& list, const std::shared_ptr<Edge> edge);

   bool parseProgram(const std::string& program) override;
   bool parseProgram(const std::vector<std::string>& tokens) override;
   std::vector<ParseTree>& getParseTreesFromLastCalculation();
   std::vector<ParseTree>& getBrokenParseTreesFromLastCalculation();

private:
   Grammar grammar_;
   std::vector<std::shared_ptr<Edge>> chart_;
   std::vector<ParseTree> parse_trees_from_last_calculation_;
   std::vector<ParseTree> broken_parse_trees_from_last_calculation_;

   /**
   * The fundamental rule of chart parsing generates new edges by combining
   * fitting active and inactive edges.
   *
   * @return change flag
   */
   bool fundamentalRule();

   /**
   * Add all the rules of the grammar to the chart that are relevant: Find the
   * rule with the LHS of edge as the leftmost RHS symbol and maximally the
   * remaining length of the input.
   *
   * @param pos  current position in the chart
   * @return change flag
   */
   bool ruleInvocation(const int pos, const std::vector<std::string>& tokens);
};

} // earley
} // vfm
