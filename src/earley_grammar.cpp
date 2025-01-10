//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file

#include "earley/earley_grammar.h"
#include "static_helper.h"
#include "parser.h"

using namespace vfm::earley;

vfm::earley::Grammar::Grammar(
   const std::string& rules,
   const std::set<std::string> terminals,
   const std::string& startsymbol,
   const std::string& rule_separator_symbol,
   const std::string& rule_inner_separator_symbol,
   const std::string& rule_transition_symbol,
   const std::string& rule_nondeterminism_symbol) : Parsable("EarleyGrammar"),
   rule_separator_symbol_(rule_separator_symbol), 
   rule_inner_separator_symbol_(rule_inner_separator_symbol), 
   rule_transition_symbol_(rule_transition_symbol), 
   rule_nondeterminism_symbol_(rule_nondeterminism_symbol),
   startsymbol_(startsymbol)
{
   addProductions(rules);
   addTerminals(terminals_);
}

vfm::earley::Grammar::Grammar(
   const std::string& start_symbol,
   const std::string& rule_separator_symbol, 
   const std::string& rule_inner_separator_symbol, 
   const std::string& rule_transition_symbol, 
   const std::string& rule_nondeterminism_symbol)
   : Grammar("", {}, start_symbol, rule_separator_symbol, rule_inner_separator_symbol, rule_transition_symbol, rule_nondeterminism_symbol)
{
}

void vfm::earley::Grammar::addProduction(const std::string& lhs, const std::deque<std::string>& rhs_raw) 
{
   std::deque<std::string> rhs;

   for (const auto& el : rhs_raw) {
      if (el == space_symbol_) {
         rhs.push_back(" ");
      }
      else {
         rhs.push_back(el);
      }
   }

   if (rhs.size() == 1) {
      singletons_.insert(lhs);
   }

   productions_.insert({ lhs, {} }); // Dummy insert in case of missing mapping.
   std::set<std::deque<std::string>>& set = productions_.at(lhs);
   set.insert(rhs);

   if (!rhs.empty()) { // Empty right side prohibited for Earley parser, but we allow it here since the grammar can be made epsilon-free later.
      leftmost_.insert({ rhs[0], {} }); // Dummy insert in case of missing mapping.
      std::set<std::string>& lefts = leftmost_.at(rhs[0]);
      lefts.insert(lhs);
   }
}

void vfm::earley::Grammar::addProductions(const std::string& possibly_collapsed_rules)
{
   std::string decollapsed = decollapseRules(possibly_collapsed_rules);

   for (const auto& rule : StaticHelper::split(decollapsed, rule_separator_symbol_)) {
      if (!StaticHelper::isEmptyExceptWhiteSpaces(rule)) {
         auto pair = StaticHelper::split(rule, rule_transition_symbol_);
         std::string lhs = pair[0];
         std::string rhs_raw = pair[1];
         std::deque<std::string> rhs;

         StaticHelper::trim(lhs);
         StaticHelper::trim(rhs_raw);

         for (const auto& r_el : StaticHelper::split(rhs_raw, rule_inner_separator_symbol_)) {
            if (!r_el.empty()) {
               rhs.push_back(r_el);
            }
         }

         addProduction(lhs, rhs);
      }
   }
}

void vfm::earley::Grammar::removeProduction(const std::string& lhs, const std::deque<std::string>& rhs)
{
   if (!productions_.count(lhs)) {
      return;
   }

   auto& rhs_ptr = productions_.at(lhs);

   for (auto it = rhs_ptr.begin(); it != rhs_ptr.end(); ) {
      if (*it == rhs) {
         rhs_ptr.erase(it);
         break;
      }

      it++;
   }
}

std::map<std::string, std::set<std::deque<std::string>>> vfm::earley::Grammar::getProductions() const
{
   return productions_;
}

std::map<std::string, std::set<std::deque<std::string>>> vfm::earley::Grammar::getNonDummyProductions()
{
   std::map<std::string, std::set<std::deque<std::string>>> res;

   for (const auto& pair : productions_) {
      if (isNonTerminal(pair.first)) {
         res.insert(pair);
      }
   }

   return res;
}

void vfm::earley::Grammar::addTerminal(const std::string& terminal_symbol)
{
   auto dummy_name = StaticHelper::makeString(DUMMY_TOKEN_PREFIX) + std::to_string(terminals_.size());
   addProduction(terminal_symbol, { dummy_name });
   terminal_dummy_mapping_.insert({ terminal_symbol, dummy_name });
   terminals_.insert(terminal_symbol);
}

void vfm::earley::Grammar::addTerminals(const std::set<std::string>& terminal_symbols)
{
   for (const auto& terminal_symbol : terminal_symbols) {
      addTerminal(terminal_symbol);
   }
}

std::string vfm::earley::Grammar::toString(const bool collapse_rules, const bool hide_dummy_terminal_rules) const
{
   std::string s;
   std::string rules;
   std::string block_sep = BLOCK_SEPARATOR_SYMBOL;

   s += SERIALIZATION_SYMBOL_RULE_SEPARATOR+ " " + rule_separator_symbol_ + block_sep + "\n";
   s += SERIALIZATION_SYMBOL_RULE_INNER_SEPARATOR+ " " + rule_inner_separator_symbol_ + block_sep + "\n";
   s += SERIALIZATION_SYMBOL_RULE_TRANSITION+ " " + rule_transition_symbol_ + block_sep + "\n";
   s += SERIALIZATION_SYMBOL_RULE_NONDETERMINISM+ " " + rule_nondeterminism_symbol_ + block_sep + "\n";
   s += SERIALIZATION_SYMBOL_PSEUDO_EPSILON + " " + pseudo_epsilon_symbol_ + block_sep + "\n";
   s += SERIALIZATION_SYMBOL_SPACE + " " + space_symbol_ + block_sep + "\n";

   s += SERIALIZATION_SYMBOL_STARTSYMBOL + " " + startsymbol_ + block_sep + "\n";
   s += SERIALIZATION_SYMBOL_TERMINALS + " " + rule_inner_separator_symbol_;

   for (const auto& term_symb : getTerminals()) {
      s += term_symb + rule_inner_separator_symbol_;
   }

   s +=  block_sep + "\n" + SERIALIZATION_SYMBOL_NONTERMINALS + " " + rule_inner_separator_symbol_;

   for (const auto& nterm_symb : const_cast<Grammar*>(this)->getNonTerminals()) {
      s += nterm_symb + rule_inner_separator_symbol_;
   }

   s += block_sep + "\n" + SERIALIZATION_SYMBOL_RULES + "\n";

   for (const auto& ts : productions_) {
      if (!hide_dummy_terminal_rules || !terminals_.count(ts.first)) {
         bool first1 = true;
         rules += ts.first + " " + rule_transition_symbol_ + " ";

         for (const auto& sl : ts.second) {
            rules += (first1 ? "" : " " + rule_nondeterminism_symbol_ + " ");
            first1 = false;

            for (const auto& s2_raw : sl) {
               std::string s2 = s2_raw;

               if (s2_raw == " ") {
                  s2 = space_symbol_;
               }

               rules += rule_inner_separator_symbol_ + s2;
            }

            rules += rule_inner_separator_symbol_;
         }

         rules += " " + rule_separator_symbol_ + "\n";
      }
   }

   return s + (collapse_rules ? collapseRules(rules) : decollapseRules(rules)) + block_sep + "\n";
}

std::set<std::string> vfm::earley::Grammar::getLHS(const std::string& rhs) const
{
   std::set<std::string> set_intersection;
   
   if (leftmost_.count(rhs)) {
      for (const auto& el : leftmost_.at(rhs)) {
         if (singletons_.count(el)) {
            set_intersection.insert(el);
         }
      }
   }

   //if (set_intersection.empty()) {
   //   addError("Unknown terminal '" + rhs + "'.");
   //}

   return set_intersection;
}

std::set<std::string> vfm::earley::Grammar::withLeftmost(const std::string& rhs)
{
   return leftmost_.count(rhs) ? leftmost_.at(rhs) : std::set<std::string>{};
}

std::set<std::deque<std::string>> vfm::earley::Grammar::rhs(const std::string& lhs)
{
   return productions_.count(lhs) ? productions_.at(lhs) : std::set<std::deque<std::string>>{};
}

std::vector<std::string> vfm::earley::Grammar::getTerminalDummyTokenString(const std::vector<std::string>& realTerminalTokenString) const
{
   std::vector<std::string> dummyTokenString;

   for (int i = 0; i < realTerminalTokenString.size(); i++) {
      auto token = realTerminalTokenString[i];
      dummyTokenString.push_back(terminal_dummy_mapping_.count(token) ? terminal_dummy_mapping_.at(token) : token);
   }

   return dummyTokenString;
}

std::set<std::string> vfm::earley::Grammar::getTerminals() const
{
   return terminals_;
}

std::set<std::string> vfm::earley::Grammar::getNonTerminals()
{
   if (non_terminals_.empty()) {
      for (const auto& rule : productions_) {
         if (!terminals_.count(rule.first)) {
            non_terminals_.insert(rule.first);
         }
      }
   }

   return non_terminals_;
}

std::string vfm::earley::Grammar::getStartsymbol() const
{
   return startsymbol_;
}

int vfm::earley::Grammar::getSymbNum(const std::string& symbol)
{
   if (symb_nums_.empty()) {
      for (const auto& symb : getNonTerminals()) {
         symb_nums_[symb] = symb_nums_.size();
      }

      for (const auto& symb : getTerminals()) {
         symb_nums_[symb] = symb_nums_.size();
      }
   }

   return symb_nums_[symbol];
}

bool vfm::earley::Grammar::isNonTerminal(const std::string& token)
{
   return getNonTerminals().count(token);
}

void vfm::earley::Grammar::declareTrailingSymbolsAsTerminal()
{
   std::set<std::string> new_terminals;

   for (const auto& rule : productions_) {
      for (const auto& vec : rule.second) {
         for (const auto& symb : vec) {
            if (!productions_.count(symb) && !terminals_.count(symb) && (symb.empty() || symb[0] != DUMMY_TOKEN_PREFIX)) {
               new_terminals.insert(symb);
            }
         }
      }
   }

   addTerminals(new_terminals);
}

void vfm::earley::Grammar::replaceEmptyWordWithPseudoEpsilon()
{
   addTerminal(pseudo_epsilon_symbol_);

   for (auto& rule : productions_) {
      auto& rhs_set = rule.second;
      bool found = false;

      for (auto it = rhs_set.begin(); it != rhs_set.end(); ) {
         if (it->empty()) {
            it = rhs_set.erase(it);
            found = true;
            break;
         }
         else {
            ++it;
         }
      }

      if (found) {
         rhs_set.insert({pseudo_epsilon_symbol_});
      }
   }
}

bool vfm::earley::Grammar::parseProgram(const std::string& program)
{
   terminals_.clear();
   productions_.clear();
   leftmost_.clear();
   singletons_.clear();
   terminal_dummy_mapping_.clear();
   non_terminals_.clear();
   symb_nums_.clear();

   const auto blocks = StaticHelper::split(program, BLOCK_SEPARATOR_SYMBOL);

   for (const auto& block : blocks) {
      if (StaticHelper::stringContains(block, SERIALIZATION_SYMBOL_NONTERMINALS)) {
         std::string inner = StaticHelper::replaceAll(block, SERIALIZATION_SYMBOL_NONTERMINALS, "");
         StaticHelper::trim(inner);

         for (const auto& element : StaticHelper::split(inner, RULE_INNER_SEPARATOR_SYMBOL_DEFAULT)) {
            non_terminals_.insert(element);
         }
      }
      else if (StaticHelper::stringContains(block, SERIALIZATION_SYMBOL_TERMINALS)) {
         std::string inner = StaticHelper::replaceAll(block, SERIALIZATION_SYMBOL_TERMINALS, "");
         StaticHelper::trim(inner);

         for (const auto& element : StaticHelper::split(inner, RULE_INNER_SEPARATOR_SYMBOL_DEFAULT)) {
            addTerminal(element);
         }
      }
      else if (StaticHelper::stringContains(block, SERIALIZATION_SYMBOL_RULES)) {
         std::string inner = StaticHelper::replaceAll(block, SERIALIZATION_SYMBOL_RULES, "");
         StaticHelper::trim(inner);
         addProductions(inner);
      }
      else if (StaticHelper::stringContains(block, SERIALIZATION_SYMBOL_SPACE)) {
         space_symbol_ = StaticHelper::removeWhiteSpace(StaticHelper::replaceAll(block, SERIALIZATION_SYMBOL_SPACE, ""));
      }
      else if (StaticHelper::stringContains(block, SERIALIZATION_SYMBOL_PSEUDO_EPSILON)) {
         pseudo_epsilon_symbol_ = StaticHelper::removeWhiteSpace(StaticHelper::replaceAll(block, SERIALIZATION_SYMBOL_PSEUDO_EPSILON, ""));
      }
      else if (StaticHelper::stringContains(block, SERIALIZATION_SYMBOL_RULE_INNER_SEPARATOR)) {
         rule_inner_separator_symbol_ = StaticHelper::removeWhiteSpace(StaticHelper::replaceAll(block, SERIALIZATION_SYMBOL_RULE_INNER_SEPARATOR, ""));
      }
      else if (StaticHelper::stringContains(block, SERIALIZATION_SYMBOL_RULE_NONDETERMINISM)) {
         rule_nondeterminism_symbol_ = StaticHelper::removeWhiteSpace(StaticHelper::replaceAll(block, SERIALIZATION_SYMBOL_RULE_NONDETERMINISM, ""));
      }
      else if (StaticHelper::stringContains(block, SERIALIZATION_SYMBOL_RULE_SEPARATOR)) {
         rule_separator_symbol_ = StaticHelper::removeWhiteSpace(StaticHelper::replaceAll(block, SERIALIZATION_SYMBOL_RULE_SEPARATOR, ""));
      }
      else if (StaticHelper::stringContains(block, SERIALIZATION_SYMBOL_RULE_TRANSITION)) {
         rule_transition_symbol_ = StaticHelper::removeWhiteSpace(StaticHelper::replaceAll(block, SERIALIZATION_SYMBOL_RULE_TRANSITION, ""));
      }
      else if (StaticHelper::stringContains(block, SERIALIZATION_SYMBOL_STARTSYMBOL)) {
         startsymbol_ = StaticHelper::removeWhiteSpace(StaticHelper::replaceAll(block, SERIALIZATION_SYMBOL_STARTSYMBOL, ""));
      }
   }

   return false;
}

std::string vfm::earley::Grammar::decollapseRulesLeft(const std::string& script) const
{
   std::string newScript = "";
   std::vector<std::string> rules = StaticHelper::split(script, rule_separator_symbol_);

   for (const auto& r : rules) {
      if (!StaticHelper::isEmptyExceptWhiteSpaces(r)) {
         for (const auto& lhs : StaticHelper::split(StaticHelper::split(r, rule_transition_symbol_)[0], rule_nondeterminism_symbol_)) {
            std::string rhs = StaticHelper::split(r, rule_transition_symbol_)[1];
            std::string lhsProc = lhs;
            std::string rhsProc = rhs;
            StaticHelper::trim(lhsProc);
            StaticHelper::trim(rhsProc);

            newScript += lhsProc + " " + rule_transition_symbol_ + " " + rhsProc + rule_separator_symbol_ + "\n";
         }
      }
   }

   return cosmeticReplacements(newScript);
}

std::string vfm::earley::Grammar::decollapseRulesRight(const std::string& script) const
{
   std::string newScript = "";
   auto rules = StaticHelper::split(script, rule_separator_symbol_);

   for (std::string r : rules) {
      if (!StaticHelper::isEmptyExceptWhiteSpaces(r)) {
         for (std::string rhs : StaticHelper::split(StaticHelper::split(r, rule_transition_symbol_)[1], rule_nondeterminism_symbol_)) {
            std::string lhs = StaticHelper::split(r, rule_transition_symbol_)[0];
            std::string lhsProc = lhs;
            std::string rhsProc = rhs;
            StaticHelper::trim(lhsProc);
            StaticHelper::trim(rhsProc);

            newScript += lhsProc + " " + rule_transition_symbol_ + " " + rhsProc + rule_separator_symbol_ + "\n";
         }
      }
   }

   return cosmeticReplacements(newScript);
}

std::string vfm::earley::Grammar::collapseRulesRight(const std::string& script) const
{
   auto rules = StaticHelper::split(script, rule_separator_symbol_);
   std::map<std::string, std::vector<std::string>> allRules;

   for (std::string r : rules) {
      if (!StaticHelper::isEmptyExceptWhiteSpaces(r)) {
         auto split = StaticHelper::split(r, rule_transition_symbol_);
         auto split0 = split[0];
         auto split1 = split[1];
         StaticHelper::trim(split0);
         StaticHelper::trim(split1);

         if (!allRules.count(split0)) {
            allRules[split0] = {};
         }

         allRules.at(split0).push_back(split1);
      }
   }

   std::vector<std::string> ruleList;

   auto cmp = [](const std::string& c1, const std::string& c2) { 
      int c1Comma = StaticHelper::split(c1, ",").size() - 1;
      int c2Comma = StaticHelper::split(c2, ",").size() - 1;

      if (c1Comma != c2Comma) {
         return c1Comma < c2Comma;
      } else {
         return c1 < c2;
      }
   };

   std::set<std::string, decltype(cmp)> sortedKeyset(cmp);

   for (const auto& rule : allRules) {
      sortedKeyset.insert(rule.first);
   }

   for (std::string lhs : sortedKeyset) {
      std::string oneLine = "";

      oneLine += lhs + " " + rule_transition_symbol_ + " ";
      for (int i = 0; i < allRules.at(lhs).size() - 1; i++) {
         std::string rhs = allRules.at(lhs).at(i);
         oneLine += rhs + " " + rule_nondeterminism_symbol_ + " ";
      }
      for (int i = allRules.at(lhs).size() - 1; i < allRules.at(lhs).size(); i++) {
         std::string rhs = allRules.at(lhs).at(i);
         oneLine += rhs;
      }
      oneLine += rule_separator_symbol_;
      ruleList.push_back(oneLine);
   }

   std::string newScript = "";
   for (std::string s : ruleList) {
      newScript += s + "\n";
   }

   return cosmeticReplacements(newScript);
}

std::string vfm::earley::Grammar::collapseRulesLeft(const std::string& script_raw) const
{
   std::string script = StaticHelper::replaceAll(
      StaticHelper::replaceAll(
         StaticHelper::replaceAll(decollapseRulesLeft(script_raw), 
            "\n", ""), 
         "\r", ""), 
      "\t", "");

   auto rules = StaticHelper::split(script, rule_separator_symbol_);
   std::map<std::string, std::vector<std::string>> allRules;

   for (std::string r : rules) {
      if (!StaticHelper::isEmptyExceptWhiteSpaces(r)) {
         auto split = StaticHelper::split(r, rule_transition_symbol_);
         auto split0 = split[0];
         auto split1 = split[1];
         StaticHelper::trim(split0);
         StaticHelper::trim(split1);

         if (!allRules.count(split1)) {
            allRules[split1] = {};
         }

         allRules.at(split1).push_back(split0);
      }
   }

   std::vector<std::string> ruleList;
   std::set<std::string> sortedKeyset;

   for (const auto& rule : allRules) {
      sortedKeyset.insert(rule.first);
   }

   for (std::string rhs : sortedKeyset) {
      std::string oneLine = "";

      for (int i = 0; i < allRules.at(rhs).size() - 1; i++) {
         std::string lhs = allRules.at(rhs).at(i);
         oneLine += lhs + rule_nondeterminism_symbol_;
      }
      for (int i = allRules.at(rhs).size() - 1; i < allRules.at(rhs).size(); i++) {
         std::string lhs = allRules.at(rhs).at(i);
         oneLine += lhs;
      }

      oneLine += " " + rule_transition_symbol_ + " " + rhs + rule_separator_symbol_;
      ruleList.push_back(oneLine);
   }

   std::string newScript = "";
   for (std::string s : ruleList) {
      newScript += s + "\n";
   }

   return cosmeticReplacements(newScript);
}

std::string vfm::earley::Grammar::decollapseRules(const std::string& script) const
{
   return decollapseRulesLeft(decollapseRulesRight(script));
}

std::string vfm::earley::Grammar::collapseRules(const std::string& script) const
{
   return collapseRulesLeft(collapseRulesRight(script));
}

std::string vfm::earley::Grammar::cosmeticReplacements(const std::string& script_raw) const
{
   auto script = StaticHelper::removeWhiteSpace(script_raw);

   return StaticHelper::replaceAll(
      StaticHelper::replaceAll(
         StaticHelper::replaceAll(
            script,
            rule_transition_symbol_, " " + rule_transition_symbol_ + " "),
         rule_separator_symbol_, rule_separator_symbol_ + "\n"),
      rule_nondeterminism_symbol_, " " + rule_nondeterminism_symbol_ + " ");
}

void vfm::earley::Grammar::makeEpsilonFree()
{
   if (isEpsilonFree()) {
      removeAdditionalStartRulesForEpsilon();
      return;
   }

   for (const auto& rule : getNonDummyProductions()) {
      for (const auto& rhs : rule.second) {
         if (rhs.empty()) {
            removeEpsilonRule(rule.first);
         }
      }
   }

   makeEpsilonFree();
}

void vfm::earley::Grammar::removeAdditionalStartRulesForEpsilon()
{
   // Check if start symbol is an additional epsilon rule.
   int countSRuleEpsilon = 0;
   int countSRuleOther = 0;

   for (const auto& rule : getNonDummyProductions()) {
      for (const auto& rhs : rule.second) {
         for (const auto& symbol : rhs) {
            if (symbol == startsymbol_) {
               return;
            }
         }
      }

      if (rule.first == startsymbol_) {
         for (const auto& rhs : rule.second) {
            if (rhs.empty()) {
               countSRuleEpsilon++;
            }
            else if (rhs.size() == 1) {
               countSRuleOther++;
            }
            else {
               return;
            }
         }
      }
   }

   if (countSRuleEpsilon != 1 || countSRuleOther != 1) {
      return;
   }

   for (const auto& rhs : productions_.at(startsymbol_)) {
      if (rhs.size() == 1) {
         startsymbol_ = rhs[0];
      }

      removeProduction(startsymbol_, rhs);
      new_start_symbol_to_be_created_tmp_ = true;
   }
}

void vfm::earley::Grammar::removeEpsilonRule(const std::string& epsilon_rule_lhs)
{
   std::set<std::pair<std::string, std::deque<std::string>>> dummy;
   replaceSimpleRule(epsilon_rule_lhs, "", dummy);

   // Epsilon element of L(G).
   if (epsilon_rule_lhs == startsymbol_) {
      new_start_symbol_to_be_created_tmp_ = true;
   }
}

void vfm::earley::Grammar::replaceSimpleRule(
   const std::string& rule_lhs,
   const std::string& rule_rhs, 
   std::set<std::pair<std::string, std::deque<std::string>>>& alreadyFinished)
{
   bool changed = false;

   for (const auto& rule : getNonDummyProductions()) {
      for (const auto& wordRightSide : rule.second) {
         if (!alreadyFinished.count({ rule.first, wordRightSide })) {
            alreadyFinished.insert({ rule.first, wordRightSide });

            for (int i = 0; i < wordRightSide.size(); i++) {
               if (wordRightSide.at(i) == rule_lhs) {
                  std::deque<std::string> newRightSide;
                  std::string newLeftSide = rule.first;

                  for (int j = 0; j < wordRightSide.size(); j++) {
                     if (j == i) {
                        if (!rule_rhs.empty()) {
                           newRightSide.push_back(rule_rhs);
                        }
                     }
                     else {
                        newRightSide.push_back(wordRightSide.at(j));
                     }
                  }

                  if (newRightSide.size() != 1 || newLeftSide != newRightSide.at(0)) { // Not: X => X
                     addProduction(newLeftSide, newRightSide);
                     changed = true;
                  }
               }
            }
         }
      }
   }

   if (changed) {
      replaceSimpleRule(rule_lhs, rule_rhs, alreadyFinished); // Recursively replace rule.
   } else {
      auto corrected_rhs = rule_rhs.empty() ? std::deque<std::string>{} : std::deque<std::string>{ rule_rhs };
      removeProduction(rule_lhs, corrected_rhs);
   }
}

bool vfm::earley::Grammar::isEpsilonFree()
{
   bool isEpsilonRuleS = false;
   new_start_symbol_to_be_created_tmp_ = false;

   for (const auto& rule : getNonDummyProductions()) {
      for (const auto& rhs : rule.second) {
         if (rhs.empty()) {
            if (rule.first == startsymbol_) { // Is S epsilon rule?
               isEpsilonRuleS = true;
            }
            else { // Is any other epsilon rule?
               return false;
            }
         }
      }
   }
   
   if (isEpsilonRuleS) { // Look for S on right side if S is epsilon rule.
      new_start_symbol_to_be_created_tmp_ = true;

      for (const auto& rule : getNonDummyProductions()) {
         for (const auto& rhs : rule.second) {
            for (const auto& single_symb : rhs) {
               if (single_symb == startsymbol_) {
                  return false;
               }
            }
         }
      }
   }

   return true;
}

bool vfm::earley::Grammar::isCompletelyEpsilonFree()
{
   for (const auto& rule : getNonDummyProductions()) {
      for (const auto& rhs : rule.second) {
         if (rhs.empty()) {
            return false;
         }
      }
   }

   return true;
}