//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file
#pragma once
#include "parsable.h"
#include <map>
#include <string>
#include <vector>
#include <deque>
#include <set>

namespace vfm {
namespace earley {


const std::string BLOCK_SEPARATOR_SYMBOL =              "<block>";
const std::string RULE_SEPARATOR_SYMBOL_DEFAULT =       "$";
const std::string RULE_INNER_SEPARATOR_SYMBOL_DEFAULT = "'";
const std::string RULE_TRANSITION_SYMBOL_DEFAULT =      ":=>"; // Using : and = ensures that default tokenizer would pull these apart in a formula.
const std::string RULE_NONDETERMINISM_SYMBOL_DEFAULT =  ":|";  // Same here.
const std::string PSEUDO_EPSILON_SYMBOL_DEFAULT =       "<>";
const std::string START_SYMBOL_DEFAULT =                "~";
constexpr char DUMMY_TOKEN_PREFIX =                     '#';

const std::string SERIALIZATION_SYMBOL_NONTERMINALS =         "Non-terminals:";
const std::string SERIALIZATION_SYMBOL_TERMINALS =            "Terminals:";
const std::string SERIALIZATION_SYMBOL_STARTSYMBOL =          "Start-symbol:";
const std::string SERIALIZATION_SYMBOL_RULES =                "Production-rules:";
const std::string SERIALIZATION_SYMBOL_RULE_SEPARATOR =       "Rule-separator-symbol:";
const std::string SERIALIZATION_SYMBOL_RULE_INNER_SEPARATOR = "Rule-inner-separator:";
const std::string SERIALIZATION_SYMBOL_RULE_TRANSITION =      "Transition-symbol:";
const std::string SERIALIZATION_SYMBOL_RULE_NONDETERMINISM =  "Nondeterminism-symbol:";
const std::string SERIALIZATION_SYMBOL_PSEUDO_EPSILON =       "Pseudo-epsilon-symbol:";
const std::string SERIALIZATION_SYMBOL_SPACE =                "Space-symbol:";

class Grammar : public Parsable {
public:
   Grammar(const std::string& rules,
           const std::set<std::string> terminals,
           const std::string& startsymbol = START_SYMBOL_DEFAULT, 
           const std::string& rule_separator_symbol = RULE_SEPARATOR_SYMBOL_DEFAULT,
           const std::string& rule_inner_separator_symbol = RULE_INNER_SEPARATOR_SYMBOL_DEFAULT,
           const std::string& rule_transition_symbol = RULE_TRANSITION_SYMBOL_DEFAULT,
           const std::string& rule_nondeterminism_symbol = RULE_NONDETERMINISM_SYMBOL_DEFAULT);

   Grammar(const std::string& startsymbol = START_SYMBOL_DEFAULT,
           const std::string& rule_separator_symbol = RULE_SEPARATOR_SYMBOL_DEFAULT,
           const std::string& rule_inner_separator_symbol = RULE_INNER_SEPARATOR_SYMBOL_DEFAULT,
           const std::string& rule_transition_symbol = RULE_TRANSITION_SYMBOL_DEFAULT,
           const std::string& rule_nondeterminism_symbol = RULE_NONDETERMINISM_SYMBOL_DEFAULT);

   std::string toString(const bool collapse_rules = true, const bool hide_dummy_terminal_rules = true) const;

   /**
   * Looks up the left hand sides that can produce the given terminal.
   */
   std::set<std::string> getLHS(const std::string& rhs) const;

   /**
    * Set of all LHS that have a production that starts with the given symbol.
    */
   std::set<std::string> withLeftmost(const std::string& rhs);

   /**
    * Set of right-hand sides for the given left-hand side.
    */
   std::set<std::deque<std::string>> rhs(const std::string& lhs);

   std::vector<std::string> getTerminalDummyTokenString(const std::vector<std::string>& realTerminalTokenString) const;
   std::set<std::string> getTerminals() const;
   std::set<std::string> getNonTerminals();
   void addProduction(const std::string& lhs, const std::deque<std::string>& rhs);
   void addProductions(const std::string& possibly_collapsed_rules);
   void removeProduction(const std::string& lhs, const std::deque<std::string>& rhs);
   void addTerminal(const std::string& terminal_symbol);
   void addTerminals(const std::set<std::string>& terminal_symbols);
   std::string getStartsymbol() const;
   int getSymbNum(const std::string& symb);
   bool isNonTerminal(const std::string& token);

   std::map<std::string, std::set<std::deque<std::string>>> getProductions() const;
   std::map<std::string, std::set<std::deque<std::string>>> getNonDummyProductions();

   void declareTrailingSymbolsAsTerminal();
   void replaceEmptyWordWithPseudoEpsilon();

   bool parseProgram(const std::string& program) override;

   /**
   * Creates an epsilon-free grammar equivalent to the given grammar. 
   * The empty word might slip out of the language - or else,
   * the method possiblyCreateNewEpsilonStartsymbol has to be invoked
   * afterwards.
   */
   void makeEpsilonFree();

   /**
   * Checks if the grammar is epsilon-free. This check works for all
   * grammar types (in contrast to the conversion-to-epsilon-free
   * algorithm which works for type-2 grammars only).
   * 
   * @return  If the grammar is epsilon-free except for S => epsilon, if
   *          S is not part of any right side.
   */
   bool isEpsilonFree();

   /**
   * Checks if the grammar is "completely" epsilon-free, i.e., even without
   * S => epsilon. Other than that, this method is the same as
   * <code>isEpsilonFree</code>.
   * 
   * @return  If the grammar's rules are all epsilon-free without any
   *          exception.
   */
   bool isCompletelyEpsilonFree();

private:
   std::string rule_separator_symbol_;
   std::string rule_inner_separator_symbol_;
   std::string rule_transition_symbol_;
   std::string rule_nondeterminism_symbol_;
   std::string pseudo_epsilon_symbol_ = PSEUDO_EPSILON_SYMBOL_DEFAULT;
   std::string space_symbol_ = "*WSP*";

   std::string startsymbol_;
   std::set<std::string> terminals_;
   std::map<std::string, std::set<std::deque<std::string>>> productions_;
   std::map<std::string, std::set<std::string>> leftmost_;
   std::set<std::string> singletons_; /// Set of LHS with a single token as RHS.

   std::map<std::string, std::string> terminal_dummy_mapping_;
   std::set<std::string> non_terminals_;
   std::map<std::string, int> symb_nums_;
   
   bool new_start_symbol_to_be_created_tmp_ = false;

   /**
   * A | B => C;</BR>
   * </BR>
   * Becomes:</BR>
   * </BR>
   * A => C;</BR>
   * B => C;</BR>
   * </BR>
   * Note that newlines are ignored except for the beginning where the
   * first rule is assumed to begin on a new line. In the end, everything
   * after the last ";" is ignored.
   *
   * @param script  An arbitrary script.
   * @return  The decollapsed script.
   */
   std::string decollapseRulesLeft(const std::string& script) const;

   /**
   * A => B | C;</BR>
   * </BR>
   * Becomes:</BR>
   * </BR>
   * A => B;</BR>
   * A => C;</BR>
   * </BR>
   * Note that newlines are ignored except for the beginning where the
   * first rule is assumed to begin on a new line. In the end, everything
   * after the last ";" is ignored.
   *
   * @param script  An arbitrary script.
   * @return  The decollapsed script.
   */
   std::string decollapseRulesRight(const std::string& script) const;

   /**
   * A => B;</BR>
   * A => C;</BR>
   * </BR>
   * Becomes:</BR>
   * </BR>
   * A => B | C;</BR>
   * </BR>
   * Note that newlines are ignored except for the beginning where the
   * first rule is assumed to begin on a new line. In the end, everything
   * after the last ";" is ignored.
   *
   * @param script  An arbitrary script.
   * @return  The collapsed script.
   */
   std::string collapseRulesRight(const std::string& script) const;

   /**
   * A => C;</BR>
   * B => C;</BR>
   * </BR>
   * Becomes:</BR>
   * </BR>
   * A | B => C;</BR>
   * </BR>
   * Note that newlines are ignored except for the beginning where the
   * first rule is assumed to begin on a new line. In the end, everything
   * after the last ";" is ignored.
   *
   * @param script  An arbitrary script.
   * @return  The collapsed script.
   */
   std::string collapseRulesLeft(const std::string& script_raw) const;

   std::string decollapseRules(const std::string& script) const;
   std::string collapseRules(const std::string& script) const;
   std::string cosmeticReplacements(const std::string& script) const;

   /**
   * Removes a single specific simple rule, i.e., a rule with a single nonterminal
   * symbol on the left side and at most a single nonterminal symbol on the
   * right side. The method is called recursively
   * until all possible matches of the left side of the rule to remove in 
   * all rules are processed.
   * 
   * @param rule_lhs        Left side of the rule.
   * @param rule_rhs        Right side of the rule which has to be a single symbol. Encode the empty word as "".
   * @param alreadyFinished A set of rules that already have been processed.
   *                        Required for the recursive call only, so initially
   *                        invoke with <code>new HashSet<>()</code>.
   */
   void replaceSimpleRule(const std::string& rule_lhs, const std::string& rule_rhs, std::set<std::pair<std::string, std::deque<std::string>>>& alreadyFinished);

   /**
   * Removes a single specific epsilon rule. The method is called recursively
   * until all possible matches of the left side of the rule to remove in 
   * all rules are processed.
   * 
   * @param epsilonRule      The rule to remove.
   */
   void removeEpsilonRule(const std::string& epsilon_rule_lhs);
   void removeAdditionalStartRulesForEpsilon();
};

} // earley
} // vfm
