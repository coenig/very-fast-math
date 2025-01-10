//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file

#include "earley/recognizer/earley_recognizer.h"
#include "static_helper.h"

vfm::earley::Recognizer::Recognizer(const Grammar& grammar) : Parsable("EarleyRecognizer"), grammar_(grammar)
{}

bool vfm::earley::Recognizer::parseProgram(const std::string& program)
{
   auto tokens = *StaticHelper::tokenizeSimple(program);
   return parseProgram(tokens);
}

bool vfm::earley::Recognizer::parseProgram(const std::vector<std::string>& tokens)
{
   int symbCount = grammar_.getNonTerminals().size();
   int n = tokens.size();
   std::string j_input;

   predicted_.resize(symbCount);
   completed_.resize(symbCount);
   t_.resize(n + 1);

   for (int i = 0; i < symbCount; i++) {
      predicted_[i].resize(n + 1);
      completed_[i].resize(n + 1);

      for (int j = 0; j < n + 1; j++) {
         completed_[i][j].resize(n + 1);
      }
   }

   t_[0] = {};
   predict(grammar_.getStartsymbol(), 0);

   for (int j = 1; j <= n; j++) {
      t_[j] = {};
      j_input = tokens[j - 1];
      for (EarleyTabRule current_tab_rule : t_[j - 1]) {
         RuleWithDot current_rule = current_tab_rule.rule_;

         if (current_tab_rule.y_ == j - 1
             && !current_rule.after_dot_.empty()
             && current_rule.after_dot_.front() == j_input) {
            RuleWithDot copy = current_rule;
            copy.moveDotRight();
            add(copy, current_tab_rule.x_, j);
         }
      }
   }

   for (EarleyTabRule current_tab_rule : t_[n]) {
      if (current_tab_rule.x_ == 0 && current_tab_rule.y_ == n
          && current_tab_rule.rule_.head_ == grammar_.getStartsymbol()
          && current_tab_rule.rule_.after_dot_.empty()) {
         return true;
      }
   }

   return false;
}

bool vfm::earley::Recognizer::recognize(Grammar& grammar, const std::vector<std::string>& program)
{
   grammar.declareTrailingSymbolsAsTerminal();
   return recognizeConst(grammar, program);
}

bool vfm::earley::Recognizer::recognizeConst(const Grammar& grammar, const std::vector<std::string>& program)
{
   auto recognizer = Recognizer(grammar);
   return recognizer.parseProgram(program);
}

bool vfm::earley::Recognizer::recognize(Grammar& grammar, const std::string& program)
{
   grammar.declareTrailingSymbolsAsTerminal();
   return recognizeConst(grammar, program);
}

bool vfm::earley::Recognizer::recognizeConst(const Grammar & grammar, const std::string & program)
{
   auto recognizer = Recognizer(grammar);
   return recognizer.parseProgram(program);
}

void vfm::earley::Recognizer::predict(const std::string& symb, const int i) {
   int a = grammar_.getSymbNum(symb);

   addDebug("PRD (" + symb + ", " + std::to_string(i) + ")");

   if (!predicted_[a][i]) {
      predicted_[a][i] = true;

      for (const auto& pair : grammar_.getProductions()) {
         for (const auto& rhs : pair.second) {
            if (pair.first == symb) {
               add(RuleWithDot(pair.first, rhs), i, i);
            }
         }
      }
   }
}

void vfm::earley::Recognizer::add(const RuleWithDot& a, const int i, const int k)
{
   EarleyTabRule tab_rule{ i, k, a };
   RuleWithDot rule(tab_rule.rule_);
   std::string symbB;
   bool found;

   addDebug("ADD (" + a.toDebugString() + ", " + std::to_string(i) + ", " + std::to_string(k) + ")");

   if (!t_[k].count(tab_rule)) { // TODO: Set comparator ok??
      t_[k].insert(tab_rule);

      if (rule.after_dot_.empty()) {  // Beta ist leeres Wort.
         complete(rule.head_, i, k);
      }
      else if (grammar_.isNonTerminal(rule.after_dot_.front())) {
         symbB = rule.after_dot_.front();
         found = false;

         for (EarleyTabRule current_tab_rule : t_[k]) {
            if (current_tab_rule.x_ == k && current_tab_rule.y_ == k
                && current_tab_rule.rule_.head_ == symbB
                && current_tab_rule.rule_.after_dot_.empty()) {
               found = true;
               break;
            }
         }

         if (found) {
            RuleWithDot current_rule = rule;
            current_rule.moveDotRight();
            add(current_rule, i, k);
         }
         else {
            predict(symbB, k);
         }
      }
   }
}

void vfm::earley::Recognizer::complete(const std::string& symb, const int j, const int k)
{
   int a = grammar_.getSymbNum(symb);
   std::set<EarleyTabRule> finished;

   addDebug("CMP (" + symb + ", " + std::to_string(j) + ", " + std::to_string(k) + ")");

   if (!completed_[a][j][k]) {
      completed_[a][j][k] = true;

      for (auto it = t_[j].begin(); it != t_[j].end(); ++it) {
         EarleyTabRule current_tab_rule = *it;
         RuleWithDot current_rule = current_tab_rule.rule_;

         if (current_tab_rule.y_ == j
             && !current_rule.after_dot_.empty()
             && current_rule.after_dot_.front() == symb
             && !finished.count(current_tab_rule)) {
            RuleWithDot copy = current_rule;
            copy.moveDotRight();
            add(copy, current_tab_rule.x_, k);
            finished.insert(current_tab_rule);
            it = t_[j].begin();
         }
      }
   }
}
