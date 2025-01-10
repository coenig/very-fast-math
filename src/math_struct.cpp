//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file

#include "math_struct.h"
#include "parser.h"
#include "equation.h"
#include "term_logic_or.h"
#include "term_logic_and.h"
#include "term_logic_eq.h"
#include "term_logic_neq.h"
#include "term_logic_sm.h"
#include "term_logic_smeq.h"
#include "term_logic_gr.h"
#include "term_logic_greq.h"
#include "term_plus.h"
#include "term_minus.h"
#include "term_mult.h"
#include "term_div.h"
#include "term_abs.h"
#include "term_max.h"
#include "term_min.h"
#include "term_trunc.h"
#include "term_rand.h"
#include "term_array_length.h"
#include "term_mod.h"
#include "term_neg.h"
#include "term_val.h"
#include "term_var.h"
#include "meta_rule.h"
#include "term_compound.h"
#include "term_array.h"
#include "static_helper.h"
#include "data_pack.h"
#include "term.h"
#include "failable.h"
#include "earley/recognizer/earley_recognizer.h"
#include "earley/parser/earley_parser.h"
#include "simplification/simplification.h"
#include "simplification/simplification_pos.h"
#include <map>
#include <functional>
#include <string>
#include <iostream>
#include <sstream>
#include <exception>
#include <limits>
#include <ctime>
#include <cmath>
#include <random>
#include <cassert>
#include <cmath>
#include <chrono>
#include <stack>


using namespace vfm;

// Some of the remaining inconsistencies between jit and nojit, regarding only nan cases.
//    MathStruct::testJitVsNojitRandom({}, d, MathStruct::parseMathStruct("0&(0/0)"), 0, 0, 0, 0, 0, true);
//    MathStruct::testJitVsNojitRandom({}, d, MathStruct::parseMathStruct("1:(0/0)"), 0, 0, 0, 0, 0, true);
//    MathStruct::testJitVsNojitRandom({}, d, MathStruct::parseMathStruct("(0/0)&0"), 0, 0, 0, 0, 0, true);
//    MathStruct::testJitVsNojitRandom({}, d, MathStruct::parseMathStruct("(0/0):1"), 0, 0, 0, 0, 0, true);

typedef std::function<std::shared_ptr<Term>(const std::shared_ptr<Term>)> fct_type_one_par;
typedef std::function<std::shared_ptr<Term>(const std::shared_ptr<Term>, const std::shared_ptr<Term>)> fct_type_two_pars;

struct FunctionOnePar {
   std::string name_{};
   fct_type_one_par function_{};
};

struct FunctionTwoPars {
   std::string name_{};
   fct_type_two_pars function_{};
};

typedef std::pair<fct_type_two_pars, fct_type_two_pars> two_functions;
typedef std::pair<fct_type_two_pars, fct_type_one_par> two_functions_gop;
typedef std::pair<std::string, std::string> two_strings;
typedef std::pair<int, int> two_ints;
typedef std::pair<std::string, int> string_with_int;
typedef std::pair<std::string, two_ints> string_with_two_ints;
typedef std::pair<two_functions, two_strings> func_def;
typedef std::pair<std::pair<FunctionTwoPars, FunctionTwoPars>, FunctionTwoPars> three_functions;
typedef std::pair<func_def, int> distr_description;
typedef std::pair<two_functions_gop, string_with_int> neutral_description;
typedef std::pair<fct_type_two_pars, string_with_two_ints> neutralizing_description;

std::map<std::string, std::map<int, std::set<MetaRule>>> MathStruct::all_rules_{};

const std::vector<std::pair<ArithType, distr_description>> DISTRIBUTIVE_TERMS = {
   {ArithType::both, {{{_mult, _plus}, {SYMB_MULT, SYMB_PLUS}}, 1}},
   {ArithType::both, {{{_mult, _minus}, {SYMB_MULT, SYMB_MINUS}}, 1}},
   {ArithType::both, {{{_and, _or}, {SYMB_AND, SYMB_OR}}, 1}},
   {ArithType::both, {{{_or, _and}, {SYMB_OR, SYMB_AND}}, 0}},
   {ArithType::right, {{{_pow, _mult}, {SYMB_POW, SYMB_MULT}}, 1}},
   {ArithType::right, {{{_div, _plus}, {SYMB_DIV, SYMB_PLUS}}, 1}},
   {ArithType::right, {{{_div, _minus}, {SYMB_DIV, SYMB_MINUS}}, 1}}
};

const std::vector<std::pair<ArithType, neutral_description>> NEUTRAL_ELEMENTS = {
   {ArithType::right, {{_minus, _id}, {SYMB_MINUS, 0}}},
   {ArithType::both, {{_plus, _id}, {SYMB_PLUS, 0}}},
   {ArithType::both, {{_mult, _id}, {SYMB_MULT, 1}}},
   {ArithType::right, {{_div, _id}, {SYMB_DIV, 1}}},
   {ArithType::right, {{_pow, _id}, {SYMB_POW, 1}}},
   {ArithType::both, {{_and, _boolify}, {SYMB_AND, 1}}},
   {ArithType::both, {{_or, _boolify}, {SYMB_OR, 0}}}
};

const std::vector<std::pair<ArithType, neutralizing_description>> NEUTRALIZING_ELEMENTS = {
   {ArithType::left, {_div, {SYMB_DIV, {0, 0}}}},
   {ArithType::both, {_mult, {SYMB_MULT, {0, 0}}}},
   {ArithType::left, {_pow, {SYMB_POW, {1, 1}}}},
   {ArithType::both, {_and, {SYMB_AND, {0, 0}}}},
   {ArithType::both, {_or, {SYMB_OR, {1, 1}}}}
};

struct CountableOperator { // Operators that allow restructuring like x + x ==> 2 * x, x * x ==> x ** 2 etc.
   FunctionTwoPars weak_operator_plus_style_{};
   FunctionTwoPars weak_operator_{};
   FunctionTwoPars strong_operator_{};
   FunctionOnePar correction_operator_{};
   FunctionOnePar power_compensator_{};
   int neutral_operator_weak_op_{};
   int neutral_operator_strong_op_{};
};

const std::vector<CountableOperator> COUNTABLE_OPERATORS = {
   { // z * x + z * y ==> z * (x + y)
      { SYMB_PLUS, _plus },
      { SYMB_PLUS, _plus },
      { SYMB_MULT, _mult },
      { SYMB_ID, _id },
      { SYMB_ID, _id },
      0,
      1
   },
   { // z * x - z * y ==> z * (x + -y)
      { SYMB_PLUS, _plus },
      { SYMB_MINUS, _minus },
      { SYMB_MULT, _mult },
      { SYMB_NEG, _neg },
      { SYMB_ID, _id },
      0,
      1
   },
   { // z ** x * z ** y ==> z ** (x + y)
      { SYMB_MULT, _mult },
      { SYMB_MULT, _mult },
      { SYMB_POW, _pow },
      { SYMB_ID, _id },
      { SYMB_LN, _ln },
      1,
      1
   },
   { // z ** x / z ** y ==> z ** (x + -y) // TODO: This will simplify x/x to 1, even for x=0.
      { SYMB_MULT, _mult },
      { SYMB_DIV, _div },
      { SYMB_POW, _pow },
      { SYMB_NEG, _neg },
      { SYMB_LN, _ln },
      1,
      1
   }
};

int debug_output = 0;

std::vector<float> MathStruct::rand_vars(100);
int MathStruct::rand_var_count = 0;
int MathStruct::first_free_private_var_num_ = 0;
std::shared_ptr<std::vector<std::string>> MathStruct::additional_test_info_ = nullptr;

// This function is licensed under CC-BY-SA-4.0 which is a copy-left license
// (https://creativecommons.org/licenses/by-sa/4.0/deed.en)!
template <typename T> // From https://stackoverflow.com/questions/45507041/how-to-check-if-weak-ptr-is-empty-non-assigned
bool is_initialized(std::weak_ptr<T> const& weak) {
   using wt = std::weak_ptr<T>;
   return weak.owner_before(wt{}) || wt{}.owner_before(weak);
}

bool isAutoExtractedName(const std::string& name)
{
   return StaticHelper::stringStartsWith(name, "_") && StaticHelper::stringEndsWith(name, "_");
}

float MathStruct::evalJitOrNojit(
   const std::shared_ptr<DataPack> d, 
   const std::shared_ptr<FormulaParser> p, 
   const std::shared_ptr<MathStruct> m, 
   const bool jit,
   const int repetitions,
   long& time)
{
   float res = -std::numeric_limits<float>::infinity();
   auto mcopy = m->copy();
   bool intentionally_destroy_result_due_to_missing_assembly = false;
   if (jit) {
#if defined(ASMJIT_ENABLED)
      mcopy->createAssembly(d, p);
      MathStruct::randomize_rand_nums();
#else
      intentionally_destroy_result_due_to_missing_assembly = true;
      Failable::getSingleton("ASMJIT")->addError("Cannot evaluate '" + mcopy->serialize() + "' in ASMJIT mode. No ASMJIT available. Result will be intentionally destroyed.");
#endif
   }

   //auto d_copy = *d;              // Uncommenting these three comments makes equal_repetitions > 1 correct (but the time measurement inaccurate).
   //d_copy.initializeValuesBy(*d);

   std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
   for (int i = 0; i < repetitions; i++) {
      res = mcopy->eval(d, p);
      //d->initializeValuesBy(d_copy);
   }
   std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

   time = std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count();

   if (intentionally_destroy_result_due_to_missing_assembly) {
      if (std::isnan(res)) {
         res = 0; // In case of nan, we set res to a not-nan value.
      }
      else if (res == 0.0) {
         res = 1.0; // Zero becomes 1.
      }
      else {
         res = -res; // In all other cases, -res is different from res.
      }
   }

   return res;
}

void getRandomTermForSimplification(
   std::shared_ptr<MathStruct>& m, 
   const int seed, 
   const int depth, 
   const std::set<std::set<std::string>>& operators, 
   const int min_raw_formula_size,
   const std::shared_ptr<DataPack> d1 = nullptr,
   const std::shared_ptr<DataPack> d2 = nullptr)
{
   int seed_dummy = seed;
   m = MathStruct::randomTerm({ "x", "y", "z" }, depth, seed_dummy++, operators);

   while (m->getNodeCount() < min_raw_formula_size) {
      m = MathStruct::randomTerm({ "x", "y", "z" }, depth, seed_dummy++, operators);
   }

   if (d1 && d2) {
      d1->addOrSetSingleVal("x", 1);
      d1->addOrSetSingleVal("y", 2);
      d1->addOrSetSingleVal("z", 3);
      d2->addOrSetSingleVal("x", 1);
      d2->addOrSetSingleVal("y", 2);
      d2->addOrSetSingleVal("z", 3);
   }
}

void MathStruct::addAddMeasure(const std::shared_ptr<std::vector<std::vector<float>>> additional_measures, const int pos, const float val)
{
   if (additional_measures) {
      while (additional_measures->size() <= pos) {
         additional_measures->push_back(std::vector<float>());
      }

      (*additional_measures)[pos].push_back(val);
   }
}

std::map<std::string, std::map<int, std::set<MetaRule>>> vfm::MathStruct::getAllRulesSoFar()
{
   auto copy = all_rules_;
   return copy;
}

void vfm::MathStruct::resetAllRules()
{
   all_rules_.clear();
}

void addTestWarning(const std::shared_ptr<std::vector<std::string>> additional_test_info, const float val1, const float val2)
{
   if (additional_test_info) {
      additional_test_info->push_back(errorLevelMapping.at(ErrorLevelEnum::warning).second + ": " + std::to_string(val1) + " =? " + std::to_string(val2));
   }
}

bool isApproximatelyEqual(float f1, float f2, float epsilon)
{
   bool approx_equal = std::fabs(f1 - f2) < epsilon; // Vals differ by small constant.
   bool fac_approx = (f1 / f2) > 1 && (f1 / f2 - 1) < epsilon || (f2 / f1) > 1 && (f2 / f1 - 1) < epsilon; // Vals differ by small factor.
   bool inf_approx = std::numeric_limits<float>::infinity() == f1 && -std::numeric_limits<float>::infinity() == f2
                  || -std::numeric_limits<float>::infinity() == f1 && std::numeric_limits<float>::infinity() == f2;

   return approx_equal || fac_approx || inf_approx;
}

bool MathStruct::testEarleyParsing(
   const std::set<std::set<std::string>>& operators,
   const EarleyChoice choice,
   const std::string& m,
   const int seed,
   const int depth,
   const bool reverse_check_intentionally_malformed,
   const std::shared_ptr<earley::Grammar> grammar_raw,
   const std::shared_ptr<std::vector<std::vector<float>>> additional_measures,
   const std::shared_ptr<std::vector<std::string>> additional_test_info,
   const bool output_ok_too,
   const bool output_everything_always)
{
   static constexpr char UNCONDITIONALLY_MALFORMING_CHARS[] = { '(', ')', ',' };
   static constexpr size_t SIZE_OF_CHARS = 3;
   auto grammar = grammar_raw ? *grammar_raw : SingletonFormulaGrammar::getInstance();

   std::string formula = m;

   if (formula.empty()) {
      formula = MathStruct::randomTerm({ "x" }, depth, seed, operators)->serializePlainOldVFMStyle();

      if (reverse_check_intentionally_malformed) {
         int rand_num = std::rand() % formula.size();
         std::string my_char = StaticHelper::makeString(UNCONDITIONALLY_MALFORMING_CHARS[std::rand() % SIZE_OF_CHARS]);
         std::string first_half = formula.substr(0, rand_num);
         std::string second_half = formula.substr(rand_num % formula.size());
         assert(formula == first_half + second_half);

         formula = first_half + my_char + second_half;
      }
   }

   bool ok;
   std::vector<earley::ParseTree> trees_ok_raw;
   std::vector<earley::ParseTree> trees_broken_raw;
   std::vector<earley::ParseTree> trees_ok;
   std::vector<earley::ParseTree> trees_broken;

   if (choice == EarleyChoice::Recognizer) {
      ok = earley::Recognizer::recognize(grammar, formula);
   }
   else {
      grammar.declareTrailingSymbolsAsTerminal();
      earley::Parser parser(grammar);

      ok = parser.parseProgram(formula);

         trees_ok_raw = parser.getParseTreesFromLastCalculation();
         trees_broken_raw = parser.getBrokenParseTreesFromLastCalculation();

      for (auto& tree : trees_ok_raw) {
         if (tree.getEdge()->lhs_ == grammar.getStartsymbol()) {
            trees_ok.push_back(tree);
         }
      }

      for (auto& tree : trees_broken_raw) {
         if (tree.getEdge()->lhs_ == grammar.getStartsymbol()) {
            trees_broken.push_back(tree);
         }
      }
   }

   if (reverse_check_intentionally_malformed) ok = !ok;

   //const float msize = orig->getNodeCount(true);
   //const float ssize = m->getNodeCount(true);

   //addAddMeasure(additional_measures, 0, msize);
   //addAddMeasure(additional_measures, 1, ssize);
   //addAddMeasure(additional_measures, 2, count);
   //addAddMeasure(additional_measures, 3, m->isCompound() && isAutoExtractedName(m->getOptor()));

   if (trees_ok.size() > 1) {
      if (additional_test_info) {
         additional_test_info->push_back(errorLevelMapping.at(ErrorLevelEnum::warning).second + ": Ambiguous grammar for '" + formula + "'.");
      }
   }

   if (!ok || output_ok_too || output_everything_always) {
      std::string res_str;

      if (reverse_check_intentionally_malformed) {
         res_str = ok ? (OK_COLOR + "CORRECTLY FAILED the parsing test" + RESET_COLOR) : (FAILED_COLOR + "UNEXPECTEDLY been parsed SUCCESSFULLY" + RESET_COLOR);
      }
      else {
         res_str = ok ? (OK_COLOR + "CORRECTLY been parsed SUCCESSFULLY" + RESET_COLOR) : (FAILED_COLOR + "UNEXPECTEDLY FAILED the parsing test" + RESET_COLOR);
      }
      
      std::cout << "Formula '" + formula + "' has " << res_str << "." << std::endl;

      if (trees_ok.size() > 1) {
         std::cout << WARNING_COLOR << "There are >1 parse trees for the above formula. The grammar is ambiguous." << RESET_COLOR << std::endl;
      }

      if (!ok || output_everything_always) {
         std::cout << "This is the grammar I used:" << std::endl;
         std::cout << grammar.toString(false) << std::endl;

         if (!trees_ok.empty()) {
            std::cout << "I have created " << trees_ok.size() << " parse trees with start symbol as root:" << std::endl;

            for (const auto& tree : trees_ok) {
               std::cout << tree.toString() << std::endl;
            }
         }

         if (!trees_broken.empty()) {
            std::cout << std::endl << "I have created " << trees_broken.size() << " BROKEN parse trees with start symbol as root:" << std::endl;

            for (const auto& tree : trees_broken) {
               std::cout << tree.toString() << std::endl;
            }
         }
      }
   }

   return ok;
}

bool vfm::MathStruct::testExtracting(const std::string& fmla, const std::shared_ptr<FormulaParser> parser, const std::shared_ptr<DataPack> data_raw, const bool print_ok_too)
{
   auto data{ data_raw ? data_raw : std::make_shared<DataPack>() };
   return testExtracting({}, data, MathStruct::parseMathStruct(fmla, parser, data), 0, 3, 256, 256, 0.01, 10, nullptr, nullptr, print_ok_too);
}

bool MathStruct::testExtracting(
   const std::set<std::set<std::string>>& operators,
   const std::shared_ptr<DataPack>& d,
   const std::shared_ptr<MathStruct>& mm,
   const int seed,
   const int depth,
   const int max_const_val,
   const int max_array_val,
   const float epsilon,
   const int encapsulations,
   const std::shared_ptr<std::vector<std::vector<float>>> additional_measures,
   const std::shared_ptr<std::vector<std::string>> additional_test_info,
   const bool print_ok_too)
{
   std::shared_ptr<MathStruct> m{};
   std::shared_ptr<FormulaParser> p = std::make_shared<FormulaParser>();

   if (mm) {
      m = mm;
   }
   else {
      m = MathStruct::randomTerm(depth, operators, max_const_val, max_array_val, seed, true);
   }

   DataPack d_copy{};
   d_copy.initializeValuesBy(d);

   m = _id(m->toTermIfApplicable());
   m->setChildrensFathers(false, false);
   auto orig = m->copy();
   float f1 = m->eval(d);
   int count = 0;

   for (int i = 0; i < encapsulations; i++) {
      additional_test_info_ = additional_test_info;
      auto rand_subformula{ m->getUniformlyRandomSubFormula(seed + i) };

      if (!rand_subformula) break;

      if (rand_subformula->extractSubTermToFunction(p, 0.75, 0.25, seed + i, 1, 0, true)) {
         count++;
      }
   }

   d->initializeValuesBy(d_copy);
   float f2 = m->eval(d);

   m = m->getTermsJumpIntoCompounds()[0];
   orig = orig->getTermsJumpIntoCompounds()[0];

   bool equal = f1 == f2; // Vals are actually equal.
   bool approx_equal = isApproximatelyEqual(f1, f2, epsilon);
   bool ok = equal || approx_equal;

   if (std::isnan(f1)) {
      ok = std::isnan(f2); // Covers only the case that BOTH vals are nan.
      equal = true; // Vals are actually equal again.
   }

   const float msize = orig->getNodeCount(TraverseCompoundsType::go_into_compound_structures);
   const float ssize = m->getNodeCount(TraverseCompoundsType::go_into_compound_structures);

   addAddMeasure(additional_measures, 0, msize);
   addAddMeasure(additional_measures, 1, ssize);
   addAddMeasure(additional_measures, 2, count);
   addAddMeasure(additional_measures, 3, m->isTermCompound() && isAutoExtractedName(m->getOptor()));

   std::string old_str = StaticHelper::replaceAll(orig->printComplete(), "_", WARNING_COLOR + "_" + RESET_COLOR);
   std::string new_str = StaticHelper::replaceAll(m->printComplete(), "_", WARNING_COLOR + "_" + RESET_COLOR);

   if (!ok || print_ok_too) {
      std::cout << old_str << HIGHLIGHT_COLOR << "  -->  " << (int) msize << " nodes" << RESET_COLOR << std::endl << "= " << f1 << std::endl << new_str << HIGHLIGHT_COLOR << "  -->  " << (int) ssize << " nodes" << RESET_COLOR << std::endl << "= " << f2 << " (encapsulated)" << (ok ? OK_COLOR + "\tOK" : FAILED_COLOR + "\tFAILED") << RESET_COLOR << std::endl << std::endl;
   }
   else if (!equal /*&& print_ok_too*/) { // If not exactly equal, print notice.
      std::cout << old_str << HIGHLIGHT_COLOR << "  -->  " << (int) msize << " nodes" << RESET_COLOR << std::endl << "= " << f1 << std::endl << new_str << HIGHLIGHT_COLOR << "  -->  " << (int) ssize << " nodes" << RESET_COLOR << std::endl << "= " << f2 << " (encapsulated)" << (ok ? OK_COLOR + "\tOK" : FAILED_COLOR + "\tFAILED") << RESET_COLOR << std::endl << std::endl;
      std::cout << WARNING_COLOR << "OK, but results are only approximately equal, please check: " << f1 << " \t=? " << f2 << " (simplified)" << RESET_COLOR << std::endl;
      addTestWarning(additional_test_info, f1, f2);
   }

   return ok;
}

bool vfm::MathStruct::testSimplifying(const std::shared_ptr<MathStruct> m, const std::shared_ptr<DataPack> d, const bool output_ok_too)
{
   return testSimplifying({}, (d ? d : std::make_shared<DataPack>()), m, 0, 0, 0, 0, 0.01, nullptr, nullptr, output_ok_too);
}

bool vfm::MathStruct::testSimplifying(const std::string& m, const std::shared_ptr<DataPack> d, const bool output_ok_too)
{
   return testSimplifying(MathStruct::parseMathStruct(m, true, false), d, output_ok_too);
}

bool MathStruct::testSimplifying(
   const std::set<std::set<std::string>>& operators,
   const std::shared_ptr<DataPack>& d_raw,
   const std::shared_ptr<MathStruct>& mm,
   const int seed,
   const int depth,
   const int max_const_val,
   const int max_array_val,
   const float epsilon,
   const std::shared_ptr<std::vector<std::vector<float>>> additional_measures,
   const std::shared_ptr<std::vector<std::string>> additional_test_info,
   const bool print_ok_too)
{
   const auto d{ std::make_shared<DataPack>() };
   const auto d_copy{ std::make_shared<DataPack>() };
   d->initializeValuesBy(d_raw);
   d_copy->initializeValuesBy(d_raw);

   std::shared_ptr<MathStruct> m{};
   if (mm) {
      m = mm;
   }
   else {
      getRandomTermForSimplification(m, seed, depth, operators, 0, d, d_copy);
   }

   additional_test_info_ = additional_test_info;
   const auto copy{ m->copy()->toTermIfApplicable() };
   const auto simplified{ simplification::simplifyFast(copy->toTermIfApplicable()) };

   const float f1{ m->eval(d) };
   d->initializeValuesBy(d_copy);
   const float f2{ simplified->eval(d_copy) };

   bool equal{ f1 == f2 }; // Vals are actually equal.
   const bool approx_equal{ isApproximatelyEqual(f1, f2, epsilon) };
   bool ok{ equal || approx_equal };

   if (std::isnan(f1)) {
      ok = std::isnan(f2); // Covers only the case that BOTH vals are nan.
      equal = true; // Vals are actually equal again.
   }

   const float msize{ static_cast<float>(m->getNodeCount()) };
   const float ssize{ static_cast<float>(simplified->getNodeCount()) };
   const float compression{ ssize / msize };

   addAddMeasure(additional_measures, 0, compression);
   addAddMeasure(additional_measures, 1, msize);
   addAddMeasure(additional_measures, 2, ssize);

   if (!ok || print_ok_too) {
      std::cout << m->serializePlainOldVFMStyle() << HIGHLIGHT_COLOR << "  -->  " << (int) msize << " nodes" << RESET_COLOR << std::endl << "= " << f1 << std::endl << simplified->serializePlainOldVFMStyle() << HIGHLIGHT_COLOR << "  -->  " << (int) ssize << " nodes (" << "Compression: " << (int) (compression * 100.0) << "%)" << RESET_COLOR << std::endl << "= " << f2 << " (simplified)" << (ok ? OK_COLOR + "\tOK" : FAILED_COLOR + "\tFAILED") << RESET_COLOR << std::endl << std::endl;
   }
   else if (!equal /*&& print_ok_too*/) { // If not exactly equal, print notice.
      std::cout << m->serializePlainOldVFMStyle() << HIGHLIGHT_COLOR << "  -->  " << (int) msize << " nodes" << RESET_COLOR << std::endl << "= " << f1 << std::endl << simplified->serializePlainOldVFMStyle() << HIGHLIGHT_COLOR << "  -->  " << (int) ssize << " nodes (" << "Compression: " << (int) (compression * 100.0) << "%)" << RESET_COLOR << std::endl << "= " << f2 << " (simplified)" << (ok ? OK_COLOR + "\tOK" : FAILED_COLOR + "\tFAILED") << RESET_COLOR << std::endl << std::endl;
      std::cout << WARNING_COLOR << "OK, but results are only approximately equal, please check: " << f1 << " \t=? " << f2 << " (simplified)" << RESET_COLOR << std::endl;
      addTestWarning(additional_test_info, f1, f2);
   }

   if (msize < ssize) {
      const std::string warning{ "Formula grew in size via simplification." };

      if (additional_test_info) {
         additional_test_info->push_back(errorLevelMapping.at(ErrorLevelEnum::warning).second + ": " + warning);
      }

      Failable::getSingleton()->addWarning(warning);
   }

   return ok;
}

bool vfm::MathStruct::testFastSimplifying(const std::string& formula, const std::shared_ptr<DataPack> data, const bool output_ok_too)
{
   return testFastSimplifying(
      {},
      MathStruct::parseMathStruct(formula),
      0,
      0,
      0,
      0,
      0,
      nullptr,
      nullptr,
      output_ok_too
      );
}

bool MathStruct::testFastSimplifying(
   const std::set<std::set<std::string>>& operators,
   const std::shared_ptr<MathStruct>& mm,
   const int seed,
   const int depth,
   const int max_const_val,
   const int max_array_val,
   const int min_raw_formula_size,
   const std::shared_ptr<std::vector<std::vector<float>>> additional_measures,
   const std::shared_ptr<std::vector<std::string>> additional_test_info,
   const bool print_ok_too)
{
   std::shared_ptr<MathStruct> m;
   if (mm) {
      m = mm;
   }
   else {
      getRandomTermForSimplification(m, seed, depth, operators, min_raw_formula_size);
   }

   additional_test_info_ = additional_test_info;
   auto mla_hardcoded = m->copy()->toTermIfApplicable();
   auto mla_hardcoded_pos = m->copy()->toTermIfApplicable();
   auto mla_runtime = m->copy()->toTermIfApplicable();

   std::chrono::steady_clock::time_point begin_hardcoded = std::chrono::steady_clock::now();
   mla_hardcoded = simplification::simplifyFast(mla_hardcoded);
   std::chrono::steady_clock::time_point end_hardcoded = std::chrono::steady_clock::now();

   std::chrono::steady_clock::time_point begin_hardcoded_pos = std::chrono::steady_clock::now();
   mla_hardcoded_pos = simplification_pos::simplifyFast(mla_hardcoded_pos);
   std::chrono::steady_clock::time_point end_hardcoded_pos = std::chrono::steady_clock::now();

   std::chrono::steady_clock::time_point begin_runtime = std::chrono::steady_clock::now();
   mla_runtime = MathStruct::simplify(mla_runtime /*, print_ok_too*/);
   std::chrono::steady_clock::time_point end_runtime = std::chrono::steady_clock::now();

   const float msize = m->getNodeCount();
   const float ssize = mla_hardcoded->getNodeCount();
   const float compression = ssize / msize;
   const int time_hardcoded = std::chrono::duration_cast<std::chrono::microseconds>(end_hardcoded - begin_hardcoded).count();
   const int time_hardcoded_pos = std::chrono::duration_cast<std::chrono::microseconds>(end_hardcoded_pos - begin_hardcoded_pos).count();
   const int time_runtime = std::chrono::duration_cast<std::chrono::microseconds>(end_runtime - begin_runtime).count();

   addAddMeasure(additional_measures, 0, compression);
   addAddMeasure(additional_measures, 1, msize);
   addAddMeasure(additional_measures, 2, ssize);
   addAddMeasure(additional_measures, 3, time_hardcoded);
   addAddMeasure(additional_measures, 4, time_hardcoded_pos);
   addAddMeasure(additional_measures, 5, time_runtime);
   addAddMeasure(additional_measures, 6, (float)time_runtime / (float) time_hardcoded);
   addAddMeasure(additional_measures, 7, (float)time_runtime / (float) time_hardcoded_pos);

   bool ok = mla_hardcoded->isStructurallyEqual(mla_runtime) 
      && mla_hardcoded->isStructurallyEqual(mla_hardcoded_pos);

   if (!ok || print_ok_too) {
      m->setPrintFullCorrectBrackets(true);
      mla_hardcoded->setPrintFullCorrectBrackets(true);
      mla_runtime->setPrintFullCorrectBrackets(true);
      mla_hardcoded_pos->setPrintFullCorrectBrackets(true);

      Failable::getSingleton("Testing")->addNote("'" + m->serializePlainOldVFMStyleWithCompounds() + "' (original Formula).");
      Failable::getSingleton("Testing")->addNote("'" + mla_hardcoded->serializePlainOldVFMStyleWithCompounds() + "', simplified in '" + HIGHLIGHT_COLOR + std::to_string(time_hardcoded) + " micros" + RESET_COLOR + "' (hardcoded version).");
      Failable::getSingleton("Testing")->addNote("'" + mla_hardcoded_pos->serializePlainOldVFMStyle() + "', simplified in '" + HIGHLIGHT_COLOR + std::to_string(time_hardcoded_pos) + " micros" + RESET_COLOR + "' (hardcoded (pos) version).");
      Failable::getSingleton("Testing")->addNote("'" + mla_runtime->serializePlainOldVFMStyleWithCompounds() + "', simplified in '" + HIGHLIGHT_COLOR + std::to_string(time_runtime) + " micros" + RESET_COLOR + "' (runtime version).");
      Failable::getSingleton("Testing")->addNote((ok ? OK_COLOR + "SUCCESS" : FAILED_COLOR + "FAILED") + RESET_COLOR + ".");
   }

   return ok;
}

bool vfm::MathStruct::testVeryFastSimplifying(const std::set<std::set<std::string>>& operators, const std::shared_ptr<MathStruct>& mm, const int seed, const int depth, const int max_const_val, const int max_array_val, const int min_raw_formula_size, const std::shared_ptr<std::vector<std::vector<float>>> additional_measures, const std::shared_ptr<std::vector<std::string>> additional_test_info, const bool print_ok_too)
{
   std::shared_ptr<MathStruct> m;
   if (mm) {
      m = mm;
   }
   else {
      getRandomTermForSimplification(m, seed, depth, operators, min_raw_formula_size);
   }

   additional_test_info_ = additional_test_info;
   auto mla1 = m->copy()->toTermIfApplicable();
   auto mla2 = m->copy()->toTermIfApplicable();

   std::chrono::steady_clock::time_point begin2 = std::chrono::steady_clock::now();
   mla2 = simplification::simplifyFast(mla2/*, print_ok_too*/);
   std::chrono::steady_clock::time_point end2 = std::chrono::steady_clock::now();

   std::chrono::steady_clock::time_point begin1 = std::chrono::steady_clock::now();
   mla1 = simplification::simplifyVeryFast(mla1);
   std::chrono::steady_clock::time_point end1 = std::chrono::steady_clock::now();

   const float msize = m->getNodeCount();
   const float ssize = mla1->getNodeCount();
   const float compression = ssize / msize;
   const int time1 = std::chrono::duration_cast<std::chrono::microseconds>(end1 - begin1).count();
   const int time2 = std::chrono::duration_cast<std::chrono::microseconds>(end2 - begin2).count();

   addAddMeasure(additional_measures, 0, compression);
   addAddMeasure(additional_measures, 1, msize);
   addAddMeasure(additional_measures, 2, ssize);
   addAddMeasure(additional_measures, 3, time1);
   addAddMeasure(additional_measures, 4, time2);
   addAddMeasure(additional_measures, 5, (float) time2 / (float) time1);

   bool ok = mla1->isStructurallyEqual(mla2);

   if (!ok || print_ok_too) {
      Failable::getSingleton("Testing")->addNote("'" + m->serializePlainOldVFMStyle() + "' (original Formula).");
      Failable::getSingleton("Testing")->addNote("'" + mla1->serializePlainOldVFMStyle() + "', simplified in '" + HIGHLIGHT_COLOR + std::to_string(time1) + " micros" + RESET_COLOR + "' (very fast version).");
      Failable::getSingleton("Testing")->addNote("'" + mla2->serializePlainOldVFMStyle() + "', simplified in '" + HIGHLIGHT_COLOR + std::to_string(time2) + " micros" + RESET_COLOR + "' (fast version).");
      Failable::getSingleton("Testing")->addNote((ok ? OK_COLOR + "SUCCESS" : FAILED_COLOR + "FAILED") + RESET_COLOR + ".");
   }

   return ok;
}

bool MathStruct::testParsing(
   const std::set<std::set<std::string>>& operators,
   const std::shared_ptr<MathStruct>& mm,
   const int seed,
   const int depth,
   const int max_const_val,
   const int max_array_val,
   const float epsilon,
   const std::shared_ptr<std::vector<std::vector<float>>> additional_measures,
   const std::shared_ptr<std::vector<std::string>> additional_test_info,
   const bool print_ok_too,
   const std::shared_ptr<FormulaParser> parser_raw)
{
   bool parsing_error{false};

   ///// Workaround for ambiguous serialization of overloaded operators /////
   auto parser{ parser_raw ? parser_raw : SingletonFormulaParser::getInstance() };
   parser->resetAllErrors();

   std::shared_ptr<MathStruct> m1{};
   if (mm) {
      m1 = mm->copy();

      m1->applyToMeAndMyChildren([](const MathStructPtr m) { // Check for right-associative structures in associative operators.
         if (m->getOpStruct().op_pos_type == OutputFormatEnum::infix && m->getOpStruct().assoc && m->getOperands().size() >= 2) {
            auto rh_child = m->getOperands()[1];

            if (rh_child->getOpStruct().op_pos_type == OutputFormatEnum::infix && rh_child->getOpStruct().assoc
                && m->getOptor() == rh_child->getOptor()) {
               m->setPrintFullCorrectBrackets(true);
               Failable::getSingleton("VFM-Testing")->addWarning("Parsing test for formula\n\t" + m->serializeWithinSurroundingFormula() + "\nwill fail. "
                                                   + "Associative operators are output without brackets and then parsed as left-associate, but the formula enforces right-associativity which cannot be expressed in this context. "
                                                   + "The problematic part of the formula is highlit.");
               m->setPrintFullCorrectBrackets(false);
            }
         }
      });

   }
   else {
      m1 = MathStruct::randomTerm(depth, operators, max_const_val, max_array_val, seed, false, parser, AssociativityEnforce::left);

      std::string test_string{ m1->serialize() };
      MathStruct::parseMathStruct(test_string, *parser, true);
      int count{};

      while (parser->hasErrorOccurred()) {
         for (const auto& error : parser->getErrors().at(vfm::ErrorLevelEnum::error)) {
            if (!StaticHelper::stringContains(error.second, "remove lose sequence symbols")) {
               parsing_error = true;
               break; // The error is not about the lose sequence symbols, so let's just let it propagate.
            }
         }

         Failable::getSingleton()->addNote("I received 'remove lose sequence symbols' errors during tokenizing, so I'll re-iterate. " + OK_COLOR + "You can ignore all errors of this type for now." + RESET_COLOR + " In future, serialization needs to be adapted.");
         if (additional_test_info) additional_test_info->push_back(errorLevelMapping.at(ErrorLevelEnum::warning).second + ": Lose sequence symbol ambiguity");
         parser->resetAllErrors();

         m1 = MathStruct::randomTerm(depth, operators, max_const_val, max_array_val, seed + count, false, nullptr, AssociativityEnforce::left);
         test_string = m1->serialize();
         MathStruct::parseMathStruct(test_string, *parser, true);
         count++;
      }
      /////
   }

   addAddMeasure(additional_measures, 0, m1->getNodeCount());

   std::string sb = m1->serialize();
   std::shared_ptr<MathStruct> m2{ MathStruct::parseMathStruct(sb, true, false, parser) };

   m1->setPrintFullCorrectBrackets(true);
   m2->setPrintFullCorrectBrackets(true);

   std::string f1 = m1->serialize();
   std::string f2 = m2->serialize();

   bool ok = f1 == f2 && !parsing_error;

   if (!ok || print_ok_too) {
      std::cout << sb << " \t(simple)" << std::endl << f1 << " \t(full brackets)" << std::endl << f2 << " \t(parsed)" << (ok ? OK_COLOR + "\tOK" : FAILED_COLOR + "\tFAILED") << RESET_COLOR << std::endl << std::endl;
   }

   return ok;
}

bool MathStruct::testJitVsNojitRandom(
   const std::set<std::set<std::string>>& operators,
   const std::shared_ptr<DataPack> data,
   const std::shared_ptr<MathStruct> mm,
   const int seed,
   const int depth,
   const int max_const_val,
   const int max_array_val,
   const float epsilon,
   const std::shared_ptr<std::vector<std::vector<float>>> additional_measures,
   const bool print_ok_too,
   const int equal_repetitions)
{
   std::shared_ptr<MathStruct> m;
   std::shared_ptr<DataPack> d;
   
   if (mm) {
      m = mm;
   }
   else {
      m = MathStruct::randomTerm(depth, operators, max_const_val, max_array_val, seed);
   }

   if (data) {
      d = data;
   }
   else {
      d = std::make_shared<DataPack>();
   }

   addAddMeasure(additional_measures, 0, m->getNodeCount());

   long time_jit;
   long time_no_jit;

   auto d_copy = std::make_shared<DataPack>();
   d_copy->initializeValuesBy(d);
   float f1 = evalJitOrNojit(d, nullptr, m, false, equal_repetitions, time_no_jit);
   d->initializeValuesBy(d_copy);
   float f2 = evalJitOrNojit(d, nullptr, m, true, equal_repetitions, time_jit);

   addAddMeasure(additional_measures, 1, time_no_jit);
   addAddMeasure(additional_measures, 2, time_jit);

   bool equal = f1 == f2; // Vals are actually equal.
   bool approx_equal = isApproximatelyEqual(f1, f2, epsilon);
   bool ok = equal || approx_equal;

   if (std::isnan(f1)) {
      ok = std::isnan(f2); // Covers only the case that BOTH vals are nan.
      equal = true; // Vals are actually equal again.
   }

   if (!ok || print_ok_too) {
      std::cout << *m << std::endl << "= " << f1 << std::endl << "= " << f2 << " (jit)" << (ok ? OK_COLOR + "\tOK" : FAILED_COLOR + "\tFAILED") << RESET_COLOR << std::endl;
      std::cout << "Time REG: " << time_no_jit << std::endl;
      std::cout << "Time JIT: " << time_jit << std::endl << std::endl;
   }
   else if (!equal /*&& print_ok_too*/) { // If not exactly equal, print notice.
      std::cout << WARNING_COLOR << "OK, but results are only approximately equal, please check: " << f1 << " \t=? " << f2 << " (jit)" << RESET_COLOR << std::endl;
   }

   return ok;
}

bool vfm::MathStruct::testFormulaEvaluation(
   const std::shared_ptr<MathStruct> m,
   const std::function<float(const std::map<std::string, float>&)>& f, 
   const std::map<std::string, std::pair<RandomNumberType, std::pair<float, float>>>& replacements,
   const std::shared_ptr<FormulaParser> pp, 
   const std::shared_ptr<DataPack> dd, 
   const int seed,
   const float epsilon,
   const std::shared_ptr<std::vector<std::vector<float>>> additional_measures, 
   const std::shared_ptr<std::vector<std::string>> additional_test_info,
   const bool output_ok_too)
{
   std::shared_ptr<DataPack> d = dd ? dd : std::make_shared<DataPack>();
   std::shared_ptr<FormulaParser> p = pp ? pp : SingletonFormulaParser::getInstance();
   std::srand(seed);

   assert(m);

   std::map<std::string, float> nums;
   std::string nums_as_str;
   std::string gen_nums_as_str;

   for (const auto& el : replacements) {
      float min = el.second.second.first;
      float max = el.second.second.second;
      float num = min + static_cast <float> (rand()) / ( static_cast <float> (RAND_MAX / (max - min)));
      std::string min_max_str = "[" + std::to_string(el.second.second.first) + ", " + std::to_string(el.second.second.second) + "] ";

      if (el.second.first == RandomNumberType::Integer) {
         num = min + (rand() % static_cast<int>(max - min + 1));
         min_max_str = "{" + std::to_string((int) el.second.second.first) + "-" + std::to_string((int) el.second.second.second) + "} ";
      }
      else if (el.second.first == RandomNumberType::Bool) {
         num = static_cast<int>(num) % 2;
         min_max_str = "{0, 1} ";
      }

      d->addOrSetSingleVal(el.first, num);
      nums.insert({ el.first, num });
      nums_as_str += el.first + "=" + std::to_string(num) + " ";
      gen_nums_as_str += el.first + ":" + min_max_str;
   }

   float f1 = m->eval(d, p);
   float f2 = f(nums);

   bool equal = f1 == f2; // Vals are actually equal.
   bool approx_equal = isApproximatelyEqual(f1, f2, epsilon);
   bool ok = equal || approx_equal;

   if (std::isnan(f1)) {
      ok = std::isnan(f2); // Covers only the case that BOTH vals are nan.
      equal = true; // Vals are actually equal again.
   }

   if (!ok || output_ok_too) {
      std::cout << "Formula checked:             " << HIGHLIGHT_COLOR << m->serialize() << RESET_COLOR << std::endl;
      std::cout << "Result from formula:         " << HIGHLIGHT_COLOR << f1 << RESET_COLOR << std::endl;
      std::cout << "Result from native function: " << HIGHLIGHT_COLOR << f2 << RESET_COLOR << " ==> " << (ok ? OK_COLOR + "\tOK" : FAILED_COLOR + "\tFAILED") << RESET_COLOR << std::endl;
      std::cout << "Here's the data:             " << nums_as_str << std::endl;
   }
   else if (!equal /*&& print_ok_too*/) { // If not exactly equal, print notice.
      std::cout << WARNING_COLOR << "OK, but results are only approximately equal, please check: " << f1 << " \t=? " << f2 << " (jit)" << RESET_COLOR << std::endl;
      addTestWarning(additional_test_info, f1, f2);
   }

   if (additional_test_info) {
      std::string add_info;

      if (ok) {
         add_info = "SUCCESS: " + m->serialize() + " -- " + gen_nums_as_str;
      }
      else {
         auto m_copy = _id(m->copy()->toTermIfApplicable());

         m_copy->applyToMeAndMyChildren([nums](const MathStructPtr m) {
            if (m->isTermVar()) {
               auto var_name = m->toVariableIfApplicable()->getVariableName();

               if (nums.count(var_name)) {
                  float val = nums.at(var_name);
                  m->getFather()->replaceOperand(m->toTermIfApplicable(), _val(val));
               }
            }
         });

         add_info = "FAILED: " + m_copy->getOperands()[0]->serialize() + " (" + std::to_string(f1) + " != " + std::to_string(f2) + ")";
      }

      additional_test_info->push_back(add_info);
   }

   return ok;
}

bool vfm::MathStruct::testFormulaEvaluation(
   const std::string& formula, 
   const std::function<float(const std::map<std::string, float>&)>& f, 
   const std::map<std::string, std::pair<RandomNumberType, std::pair<float, float>>>& replacements,
   const std::shared_ptr<FormulaParser> pp, 
   const std::shared_ptr<DataPack> dd, 
   const int seed, 
   const float 
   epsilon, 
   const std::shared_ptr<std::vector<std::vector<float>>> additional_measures, 
   const std::shared_ptr<std::vector<std::string>> additional_test_info,
   const bool output_ok_too)
{
   return testFormulaEvaluation(
      MathStruct::parseMathStruct(formula, true, false, pp)->toTermIfApplicable(), f, replacements, pp, dd, seed, epsilon, additional_measures, additional_test_info_, output_ok_too
   );
}

bool vfm::MathStruct::testJitVsNojit(const std::shared_ptr<MathStruct> m, const std::shared_ptr<DataPack> d, const bool output_ok_too, const int equal_repetitions)
{
   return MathStruct::testJitVsNojitRandom({}, (d ? d : std::make_shared<DataPack>()), m, 0, 0, 0, 0, 0.01, nullptr, output_ok_too, equal_repetitions);
}

bool vfm::MathStruct::testJitVsNojit(const std::string& m, const std::shared_ptr<DataPack> d, const bool output_ok_too, const int equal_repetitions)
{
   return MathStruct::testJitVsNojit(MathStruct::parseMathStruct(m, true), d, output_ok_too, equal_repetitions);
}

bool vfm::MathStruct::testParsing(const std::string& formula, const bool output_ok_too, const std::shared_ptr<FormulaParser> parser)
{
   return testParsing({}, parseMathStruct(formula, parser), -1, -1, -1, -1, -1, nullptr, nullptr, output_ok_too, parser);
}

int MathStruct::prio() const
{
   return this->getOpStruct().precedence;
}

std::string MathStruct::getOptor() const
{
   return getOpStruct().op_name;
}

std::string vfm::MathStruct::getOptorOnCompoundLevel() const
{
   return getOpStruct().op_name;
}

std::string vfm::MathStruct::getOptorJumpIntoCompound() const
{
   return isTermCompound() ? opnds_[0]->getOpStruct().op_name : getOpStruct().op_name;
}

void vfm::MathStruct::setOptor(const std::string& new_optor_name)
{
   getOpStruct().op_name = new_optor_name;
}

void vfm::MathStruct::setOpnds(const std::vector<std::shared_ptr<Term>>& opds)
{
   opnds_ = opds;
}

bool MathStruct::isTermVar() const
{
   return false;
}

bool MathStruct::isTermArray() const
{
   return false;
}

void MathStruct::swap()
{
   auto temp = opnds_[0];
   opnds_[0] = opnds_[1];
   opnds_[1] = temp;
}

void vfm::MathStruct::swapJumpIntoCompounds()
{
   auto temp{ getTermsJumpIntoCompounds()[0] };
   getTermsJumpIntoCompounds()[0] = getTermsJumpIntoCompounds()[1];
   getTermsJumpIntoCompounds()[1] = temp;
}

bool vfm::MathStruct::isRootTerm() const
{
   return !father_.lock();
}

void MathStruct::applyToMeAndMyChildren(
   const std::function<void(std::shared_ptr<MathStruct>)>& f, 
   const TraverseCompoundsType go_into_compounds,
   const std::shared_ptr<bool> trigger_abandon_children_once,
   const std::shared_ptr<bool> trigger_break,
   const std::shared_ptr<std::vector<std::shared_ptr<MathStruct>>> blacklist,
   const std::shared_ptr<std::vector<std::shared_ptr<MathStruct>>> visited,
   const std::function<bool(const std::shared_ptr<MathStruct>)>& reverse_ordering_of_children)
{
   auto ptr_to_this{ shared_from_this() };

   if (trigger_break && *trigger_break || blacklist && std::find(blacklist->begin(), blacklist->end(), ptr_to_this) != blacklist->end()) {
      return;
   }

   f(shared_from_this());

   if (trigger_abandon_children_once && *trigger_abandon_children_once) {
      if (!isTermCompound()) { // If it's a compound, the trigger will be removed there, so the tree can be cut off there.
         *trigger_abandon_children_once = false;
      }

      return;
   }

   if (reverse_ordering_of_children(ptr_to_this)) {
      for (auto c = opnds_.rbegin(); c != opnds_.rend(); ++c) {
         (*c)->applyToMeAndMyChildren(f, go_into_compounds, trigger_abandon_children_once, trigger_break, blacklist, visited, reverse_ordering_of_children);
      }
   } else {
      for (const auto& c : opnds_) {
         c->applyToMeAndMyChildren(f, go_into_compounds, trigger_abandon_children_once, trigger_break, blacklist, visited, reverse_ordering_of_children);
      }
   }
}

void vfm::MathStruct::applyToMeAndMyChildrenIterative(
   const std::function<void(MathStructPtr)>& f, 
   const TraverseCompoundsType go_into_compounds,
   const FormulaTraversalType traversal_type)
{
   bool abort_dummy = false;
   applyToMeAndMyChildrenIterative(f, go_into_compounds, abort_dummy, traversal_type);
}

void MathStruct::applyToMeAndMyChildrenIterative(
   const std::function<void(MathStructPtr)>& f_raw, 
   const TraverseCompoundsType go_into_compounds,
   const bool& abort,
   const FormulaTraversalType traversal_type) 
{
   auto f{ [&f_raw](const MathStructPtr m) { // TODO: No idea if this is inefficient - but why should it be...
      if (m->isValid()) {
         f_raw(m);
      }
   } };

   if (traversal_type == FormulaTraversalType::PreOrder) {
      std::stack<MathStructPtr> s{};
      s.push(shared_from_this());

      while (!s.empty()) {
         auto temp{ s.top() };

         //if (go_into_compounds == TraverseCompoundsType::go_into_compound_structures || !temp->isCompoundOperator()) {
            f(temp);
         //}

         if (abort) return;

         s.pop();

         for (int i = temp->opnds_.size() - 1; i >= 0; i--) {
            if (go_into_compounds == TraverseCompoundsType::go_into_compound_structures || !temp->opnds_[i]->isCompoundStructure()) {
               s.push(temp->opnds_[i]);
            }
         }
      }
   }
   else if (traversal_type == FormulaTraversalType::PostOrder) {
      // From: https://www.geeksforgeeks.org/iterative-postorder-traversal-of-n-ary-tree/
      int currentRootIndex{ 0 };
      std::stack<std::pair<MathStructPtr, int>> st{};
      auto root{ shared_from_this() };

      while (root != nullptr || st.size() > 0) {
         if (root && go_into_compounds == TraverseCompoundsType::avoid_compound_structures && root->isTermCompound()) {
            root = root->opnds_[0];
            continue;
         }

         if (root) {
            st.push({ root, currentRootIndex });
            currentRootIndex = 0;

            if (root->opnds_.size() >= 1) {
               root = root->opnds_[0];
            }
            else {
               root = nullptr;
            }
            continue;
         }

         auto temp{ st.top() };
         st.pop();

         f(temp.first->isCompoundOperator() && go_into_compounds == TraverseCompoundsType::avoid_compound_structures ? temp.first->getFather() : temp.first);
         if (abort) return;

         while (st.size() > 0 && temp.second == st.top().first->opnds_.size() - 1) {
            temp = st.top();
            st.pop();
            f(temp.first->isCompoundOperator() && go_into_compounds == TraverseCompoundsType::avoid_compound_structures ? temp.first->getFather() : temp.first);
            if (abort) return;
         }

         if (st.size() > 0) {
            root = st.top().first->opnds_[temp.second + 1];
            currentRootIndex = temp.second + 1;
         }
      }
   }
   else {
      Failable::getSingleton("FormulaTraverser")->addFatalError("Unknown formula traversal type '" + std::to_string(static_cast<int>(traversal_type)) + "'.");
   }
}

void MathStruct::applyToMeAndMyChildrenIterativeReversePreorder(
   const std::function<void(MathStructPtr)>& f_raw, 
   const TraverseCompoundsType go_into_compounds,
   const bool& abort,
   const FormulaTraversalType traversal_type, 
   const std::set<std::string>& apply_at_and_abandon_children_of,
   const std::set<std::string>& break_it_off_at_except)
{
   auto f = [&f_raw](const MathStructPtr m) { // TODO: No idea if this is inefficient - but why should it be...
      if (m->isValid()) {
         f_raw(m);
      }
   };

   if (traversal_type == FormulaTraversalType::PreOrder) {
      std::stack<MathStructPtr> s{};
      s.push(shared_from_this());

      while (!s.empty()) {
         auto temp{ s.top() };
         bool is_at_stop_point{ (bool) apply_at_and_abandon_children_of.count(temp->getOptorOnCompoundLevel()) };
         bool is_at_break_point{ !break_it_off_at_except.count(temp->getOptorOnCompoundLevel()) && !is_at_stop_point };

         if (is_at_break_point) return;

         if (apply_at_and_abandon_children_of.empty() || is_at_stop_point) {
            f(temp);
         }

         if (abort) return;

         s.pop();

         if (!is_at_stop_point) {
            for (int i = 0; i < temp->getTermsJumpIntoCompounds().size(); i++) {
               //if (go_into_compounds == TraverseCompoundsType::go_into_compound_structures || !temp->opnds_[i]->isCompoundStructure()) {
                  s.push(temp->getTermsJumpIntoCompounds()[i]);
               //}
            }
         }
      }
   }
   //else if (traversal_type == FormulaTraversalType::PostOrder) {
   // TODO: Implement post order in reverse direction.
   //}
   else {
      Failable::getSingleton("FormulaTraverser")->addFatalError("Unknown formula traversal type '" + std::to_string(static_cast<int>(traversal_type)) + "'.");
   }
}

std::shared_ptr<MathStruct> vfm::MathStruct::retrieveNthChildWithProperty(const std::function<bool(const std::shared_ptr<MathStruct>)>& property, const int n)
{
   std::shared_ptr<MathStruct> res = nullptr;
   int count = n;
   auto breakup = std::make_shared<bool>(false);

   applyToMeAndMyChildren([&res, &count, property, breakup](const std::shared_ptr<MathStruct> m) {
      if (property(m)) {
         if (!count) {
            res = m;
            *breakup = true;
         }
         else {
            count--;
         }
      }
   }, TraverseCompoundsType::avoid_compound_structures, nullptr, breakup);

   return res;
}

std::vector<std::shared_ptr<MathStruct>> MathStruct::createAllCommVariationsInChildren(const std::shared_ptr<FormulaParser> parser) const
{
   auto p{ parser ? parser : SingletonFormulaParser::getInstance() };
   std::vector<std::shared_ptr<MathStruct>> vec{};
   std::vector<std::shared_ptr<MathStruct>> vec2{};

   vec2.push_back(const_cast<MathStruct*>(this)->copy());

   for (int i = 0; !vec2.empty(); i++) {
      vec.insert(vec.end(), vec2.begin(), vec2.end());
      vec2.clear();
      std::shared_ptr<MathStruct> temp_formula{ nullptr };

      for (const auto& vec_formula : vec) {
         auto vec_formula_copy{ vec_formula->copy() };

         temp_formula = vec_formula_copy->retrieveNthChildWithProperty([](const std::shared_ptr<MathStruct> m) {
            return m->getOpStruct().commut && m->getOpStruct().arg_num == 2 && !m->isRootTerm();
         }, i);

         if (temp_formula) {
            temp_formula->getFather()->replaceOperand(
               temp_formula->toTermIfApplicable(),
               p->termFactory(temp_formula->getOptor(), { temp_formula->getOperands()[1], temp_formula->getOperands()[0] })->toTermIfApplicable());

            vec2.push_back(vec_formula_copy);
         }
         else {
            break;
         }
      }
   }

   return vec;
}

std::vector<std::shared_ptr<MathStruct>> vfm::MathStruct::createAllCommVariationsInChildren(const std::vector<std::shared_ptr<MathStruct>>& formulae, const std::shared_ptr<FormulaParser> parser)
{
   std::vector<std::shared_ptr<MathStruct>> vec{};

   for (const auto& f : formulae) {
      auto vec2 = f->createAllCommVariationsInChildren(parser);
      vec.insert(vec.end(), vec2.begin(), vec2.end());
   }

   return vec;
}

std::set<MetaRule> vfm::MathStruct::createAllCommVariationsOfMetaRules(const std::set<MetaRule>& rules, const bool children_only, const std::shared_ptr<FormulaParser> parser)
{
   std::set<MetaRule> vec{};

   for (const auto& r : rules) {
      //std::cout << r.serialize() << std::endl;
      auto from_orig{ r.getFrom() };

      if (!children_only) {
         from_orig = _id(from_orig);
      }

      for (const auto& from : from_orig->createAllCommVariationsInChildren(parser)) {
         auto cp{ r.copy(true, true, true) };
         auto from_new{ from };

         if (!children_only) {
            from_new = from_new->opnds_[0];
         }

         auto new_rule = MetaRule(from_new->toTermIfApplicable(), cp.getTo(), cp.getStage(), cp.getConditions(), cp.getGlobalCondition(), parser);

         vec.insert(new_rule);
         //std::cout << "  " << new_rule.serialize() << std::endl;
      }
   }

   return vec;
}

std::vector<std::shared_ptr<MathStruct>> vfm::MathStruct::createAllCommVariations(const std::vector<std::shared_ptr<Term>>& formulae, const std::shared_ptr<FormulaParser> parser)
{
   std::vector<std::shared_ptr<MathStruct>> vec;

   for (const auto& f : formulae) {
      auto vec2 = _id(f)->createAllCommVariationsInChildren(parser);

      for (const auto& f2 : vec2) {
         vec.push_back(f2->opnds_[0]);
      }
   }

   return vec;
}

void MathStruct::applyToMeAndMyParents(const std::function<void(std::shared_ptr<MathStruct>)>& f)
{
   f(shared_from_this());

   if (!isRootTerm()) {
      father_.lock()->applyToMeAndMyParents(f);
   }
}

std::shared_ptr<MathStruct> vfm::MathStruct::getPtrToRootWithinNextEnclosingBlock()
{
   auto ptr = shared_from_this();
   auto father = ptr->getFather();

   while (father && father->isTermSequence()) {
      ptr = father;
      father = ptr->getFather();
   }

   return ptr;
}

std::shared_ptr<MathStruct> MathStruct::getPtrToRoot(
   const bool go_over_compound_bounderies,
   std::shared_ptr<std::vector<std::weak_ptr<TermCompound>>> visited)
{
   auto ptr = shared_from_this();
   
   while (ptr->getFather()) {
      ptr = ptr->father_.lock()->shared_from_this();
   }
   
   if (go_over_compound_bounderies && ptr->isCompoundStructure()) {
      if (!visited) {
         visited = std::make_shared<std::vector<std::weak_ptr<TermCompound>>>();
      }

      for (const std::weak_ptr<TermCompound> comp_ref : ptr->getCompoundStructureReferences()) {
         bool has_not_been_visited = std::find_if(visited->begin(), visited->end(), [&comp_ref](const std::weak_ptr<TermCompound>& ptr1) {
            return ptr1.lock() == comp_ref.lock();
         }) == visited->end();

         if (has_not_been_visited) {
            visited->push_back(comp_ref);
            return comp_ref.lock()->getPtrToRoot(true, visited);
         }
      }
   }

   return ptr;
}

std::shared_ptr<MathStruct> MathStruct::setPrintFullCorrectBrackets(const bool val)
{
    applyToMeAndMyChildren([val](std::shared_ptr<MathStruct> m){
        m->full_correct_brackets_ = val;
    });

    return shared_from_this();
}

bool MathStruct::printBrackets() const {
   if (full_correct_brackets_) return true;
   auto actual_father{ getFatherJumpOverCompound() };
   if (isRootTerm() || !actual_father) return false; // Assuming this to be top-level MathStruct.

   int father_prio = actual_father->prio();
   int my_prio = prio();
   bool father_assoc = actual_father->getOpStruct().assoc;
   bool father_infix = actual_father->getOpStruct().op_pos_type == OutputFormatEnum::infix;
   auto associativity_type = getOpStruct().associativity;
   auto option = getOpStruct().option;

   return option == AdditionalOptionEnum::always_print_brackets
       || associativity_type == AssociativityTypeEnum::right
       || (my_prio < father_prio || !father_assoc && my_prio == father_prio) && father_infix;
}

const std::string op_brack = StaticHelper::makeString(OPENING_BRACKET);
const std::string cl_brack = StaticHelper::makeString(CLOSING_BRACKET);
const std::string op_brace = StaticHelper::makeString(OPENING_BRACKET_BRACE_STYLE);
const std::string cl_brace = StaticHelper::makeString(CLOSING_BRACKET_BRACE_STYLE);

std::string vfm::MathStruct::serializePlainOldVFMStyle(const SerializationSpecial special) const
{
   return serialize(nullptr, SerializationStyle::plain_old_vfm, special);
}

std::string vfm::MathStruct::serializePlainOldVFMStyleWithCompounds() const
{
   return serialize(nullptr, SerializationStyle::plain_old_vfm, SerializationSpecial::print_compound_structures);
}

std::string vfm::MathStruct::serialize() const
{
   return serialize(SerializationSpecial::none);
}

std::string MathStruct::serialize(const SerializationSpecial special) const
{
   return serialize(nullptr, SerializationStyle::cpp, special);
}

std::string MathStruct::serialize(
   const std::shared_ptr<MathStruct> highlight,
   const SerializationStyle style,
   const SerializationSpecial special,
   const int indent,
   const int indent_step,
   const int line_len,
   const std::shared_ptr<DataPack> data) const
{
   std::stringstream s{};
   std::set<std::shared_ptr<const MathStruct>> visited{};
   serialize(s, highlight, style, special, indent, indent_step, line_len, data, visited);
   return s.str();
}

std::string MathStruct::serializePostfix() const
{
   return serialize(nullptr, SerializationStyle::nusmv, SerializationSpecial::enforce_postfix); // Assume nuSMV-style output; otherwise use complete signature.
}

std::string vfm::MathStruct::serializeWithinSurroundingFormula(const int levels_to_go_up) const
{
   MathStructPtr ptr_to_ancestor = const_cast<MathStruct*>(this)->shared_from_this();
   std::string target_level_isnt_root = " [...] ";

   for (int i = 0; i < levels_to_go_up; i++) {
      auto father = ptr_to_ancestor->getFather();
      if (!father) {
         target_level_isnt_root = "";
         break;
      }

      ptr_to_ancestor = father;
   }

   return target_level_isnt_root + ptr_to_ancestor->serialize(const_cast<MathStruct*>(this)->shared_from_this()) + target_level_isnt_root;
}

std::string vfm::MathStruct::serializeWithinSurroundingFormula(const int chars_left, const int chars_right) const
{
   const std::string serialized_full = serializeWithinSurroundingFormula();
   const int index_begin = serialized_full.find(HIGHLIGHT_TERM_BEGIN);
   const int index_end = serialized_full.find(HIGHLIGHT_TERM_END) + HIGHLIGHT_TERM_END.size();

   const size_t beg = std::max(0, index_begin - chars_left);
   const size_t end = index_end + chars_right;
   
   std::string before;
   std::string after;

   if (beg > 0) {
      before = "[...] ";
   }

   if (end < serialized_full.size()) {
      after = " [...]";
   }

   return before + serialized_full.substr(beg, end - beg) + after;
}

std::string vfm::MathStruct::serializeWithinSurroundingFormula() const
{
   return serializeWithinSurroundingFormula(std::numeric_limits<int>::max());
}

std::string vfm::MathStruct::serializeWithinSurroundingFormulaPlainOldVFMStyle() const
{
   return const_cast<MathStruct*>(this)->getPtrToRoot()->serialize(const_cast<MathStruct*>(this)->shared_from_this(), SerializationStyle::plain_old_vfm, SerializationSpecial::none);
}

std::string vfm::MathStruct::serializeWithinSurroundingFormulaPlainOldVFMStyleWithCompounds() const
{
   return const_cast<MathStruct*>(this)->getPtrToRoot()->serialize(const_cast<MathStruct*>(this)->shared_from_this(), SerializationStyle::plain_old_vfm, SerializationSpecial::print_compound_structures);
}

std::string vfm::MathStruct::getK2Operator() const
{
   return getOptor();
}

void vfm::VariablesWithKinds::renameVariablesForKratos()
{
   auto doit = [](std::set<std::string>& set) {
      std::set<std::string> to_remove_input{};
      std::set<std::string> to_add_input{};

      for (auto& var : set) {
         std::string new_var{ StaticHelper::replaceAll(var, "::", "____") };
         to_remove_input.insert(var);
         to_add_input.insert(new_var);
      }

      for (const auto& var : to_remove_input) set.erase(var);
      for (const auto& var : to_add_input) set.insert(var);
   };

   doit(input_variables_);
   doit(output_variables_);
   doit(pure_input_variables_);
   doit(pure_output_variables_);
   doit(pure_state_variables_);
}

std::string MathStruct::getNuSMVOperator() const
{
   return getOptor();
}

std::string MathStruct::serializeNuSMV() const
{
   return serializeNuSMV(DataPack::getSingleton(), SingletonFormulaParser::getInstance());
}

std::string MathStruct::serializeNuSMV(const std::shared_ptr<DataPack> data, const std::shared_ptr<FormulaParser> parser) const
{
   auto cp = _id(const_cast<MathStruct*>(this)->copy()->toTermIfApplicable()); // TODO: Clean solution is to make copy function const.

   cp->applyToMeAndMyChildrenIterative([data, parser](const MathStructPtr m) {
      if (m->getOptor() == SYMB_NONDET_RANGE_FUNCTION_FOR_MODEL_CHECKING) {
         m->replaceOperand(0, _val(m->opnds_[0]->eval(data, parser))); // Apparently, nusmv expects
         m->replaceOperand(1, _val(m->opnds_[1]->eval(data, parser))); // literal values here.
      }
   }, TraverseCompoundsType::avoid_compound_structures);

   return cp->getOperands()[0]->serialize(nullptr, SerializationStyle::nusmv, SerializationSpecial::none, 0, 3, std::numeric_limits<int>::max());
}

std::string vfm::MathStruct::getCPPOperator() const
{
   return getOptor();
}

std::string vfm::MathStruct::serializeK2(
   const std::map<std::string, std::string>& enum_values, 
   std::pair<std::map<std::string, std::string>, std::string>& additional_functions, 
   int& label_counter) const
{
   auto this_ptr{ const_cast<MathStruct*>(this) };
   std::string s{};
   std::string operands{};

   for (const auto& child : this_ptr->getTermsJumpIntoCompounds()) {
      operands += " " + child->serializeK2(enum_values, additional_functions, label_counter);
   }

   s = StaticHelper::makeString(OPENING_BRACKET) + this_ptr->getK2Operator() + operands + StaticHelper::makeString(CLOSING_BRACKET);

   return s;
}
std::string vfm::MathStruct::serializeK2() const
{
   std::pair<std::map<std::string, std::string>, std::string> dummy{};
   return serializeK2({}, dummy);
}

std::string vfm::MathStruct::serializeK2(const std::map<std::string, std::string>& enum_values, std::pair<std::map<std::string, std::string>, std::string>& additional_functions) const
{
   int dummy{ 0 };
   return serializeK2(enum_values, additional_functions, dummy);
}

std::string MathStruct::resolveOptor(const SerializationStyle style) const
{
   return style == SerializationStyle::nusmv 
      ? getNuSMVOperator()
      : (style == SerializationStyle::cpp ? getCPPOperator() : getOptor());
}

std::string spaces(const int indent) {
   std::string spaces;

   for (int i = 0; i < indent; i++) {
      spaces += " ";
   }

   return spaces;
}

std::string line_break(const int indent) {
   return "\n" + spaces(indent);
}

void insertNewlineIfNecessary(const MathStruct::SerializationStyle style, std::stringstream& os, const int indent, const int line_len)
{
   if (style == MathStruct::SerializationStyle::cpp && os.tellp() > line_len) {
      std::string myString = os.str();

      if (myString.size() > line_len) {
         auto pos = myString.find_last_of('\n');

         if (pos == std::string::npos) {
            pos = 0;
         }

         while (myString.size() - pos > line_len) {
            bool changed = false;

            for (int i = pos + line_len; i > pos + indent; i--) {
               if (myString[i] == ' ' || myString[i] == '\t') {
                  myString.insert(i + 1, line_break(indent));
                  pos = i + 1;
                  changed = true;
               }
            }

            if (!changed) {
               break;
            }
         }

         os.str(std::string());
         os << myString;
      }
   }
}

bool MathStruct::possiblyHighlightAndGoOnSerializing(
   std::stringstream& os,
   const std::shared_ptr<MathStruct> highlight,
   const SerializationStyle style,
   const SerializationSpecial special,
   const int indent,
   const int indent_step,
   const int line_len,
   const std::shared_ptr<DataPack> data,
   std::set<std::shared_ptr<const MathStruct>>& visited) const
{
   if (highlight
      && (highlight == shared_from_this()
         || special != SerializationSpecial::enforce_postfix_and_print_compound_structures
         && special != SerializationSpecial::print_compound_structures
         && highlight == const_cast<MathStruct*>(this)->thisPtrGoUpToCompound())
      ) {
      os << HIGHLIGHT_TERM_BEGIN;
      serialize(os, nullptr, style, special, indent, indent_step, line_len, data, visited);
      os << HIGHLIGHT_TERM_END;
      return true;
   }

   return false;
}

void MathStruct::serialize(
   std::stringstream& os, 
   const std::shared_ptr<MathStruct> highlight, 
   const SerializationStyle style,
   const SerializationSpecial special,
   const int indent,
   const int indent_step,
   const int line_len,
   const std::shared_ptr<DataPack> data,
   std::set<std::shared_ptr<const MathStruct>>& visited) const
{
   insertNewlineIfNecessary(style, os, indent, line_len);

   if (style == SerializationStyle::nusmv && !const_cast<MathStruct*>(this)->isCompoundOperator() && MetaRule::consists_purely_of_numbers(const_cast<MathStruct*>(this)->shared_from_this())) {
      os << const_cast<MathStruct*>(this)->constEval();
      return;
   }

   if ((special == SerializationSpecial::enforce_postfix_and_print_compound_structures 
       || special == SerializationSpecial::print_compound_structures) &&  visited.count(shared_from_this())) {
      os << SPECIAL_COLOR_2 << "  [[[" << getOptorJumpIntoCompound() << "]]]  " << RESET_COLOR;
      return;
   }

   if (possiblyHighlightAndGoOnSerializing(os, highlight, style, special, indent, indent_step, line_len, data, visited)) {
      visited.insert(shared_from_this());
      return;
   }

   visited.insert(shared_from_this());

   bool braces = getOpStruct().op_pos_type == OutputFormatEnum::braces_from_second_argument;

   if (special == SerializationSpecial::enforce_postfix || getOpStruct().op_pos_type == OutputFormatEnum::postfix) {
      for (std::size_t i = 0; i < opnds_.size(); ++i) {
         opnds_[i]->serialize(os, highlight, style, special, indent, indent_step, line_len, data, visited);
         os << POSTFIX_DELIMITER;
         insertNewlineIfNecessary(style, os, indent, line_len);
      }
      
      os << resolveOptor(style);
   }
   else if (getOpStruct().op_pos_type == OutputFormatEnum::infix) {
      bool pb = printBrackets() || style == SerializationStyle::nusmv && getOptor() != SYMB_DOT_ALT;
      std::string brack_o = pb ? op_brack : "";
      std::string brack_c = pb ? cl_brack : "";

      os << brack_o;

      if (!opnds_.empty()) { // Needed for formulas parsed with syntactical errors.
         opnds_[0]->serialize(os, highlight, style, special, indent, indent_step, line_len, data, visited);
         insertNewlineIfNecessary(style, os, indent, line_len);
      }

      for (std::size_t i = 1; i < opnds_.size(); ++i) {
         auto father{ getFather() };
         bool is_sequence{ isCompoundOperator() && father && father->isTermSequence() };
         bool is_dot{ getOptor() == SYMB_DOT_ALT };
         std::string spc1{ is_dot ? "" : (is_sequence && (style == SerializationStyle::cpp) ? "" : " ")};
         std::string spc2{ is_dot ? "" : (is_sequence && (style == SerializationStyle::cpp) ? line_break(indent) : spc1)};
         os << spc1 << resolveOptor(style) << spc2;
         opnds_[i]->serialize(os, highlight, style, special, indent, indent_step, line_len, data, visited);
         insertNewlineIfNecessary(style, os, indent, line_len);
      }

      os << brack_c;
   }
   else if (getOpStruct().op_pos_type == OutputFormatEnum::prefix || braces) {
      std::string spc = getOpStruct().op_name == SYMB_NEG ? " " : "";
      std::string brack_o = op_brack;
      std::string brack_c = cl_brack;
      std::string optr = resolveOptor(style);

      os << spc << optr << brack_o;

      if (!opnds_.empty()) {
         std::string delimiter = ", ";
         opnds_[0]->serialize(os, highlight, style, special, indent, indent_step, line_len, data, visited);
         insertNewlineIfNecessary(style, os, indent, line_len);

         if (braces && style == SerializationStyle::cpp) {
            os << brack_c << line_break(indent) << op_brace << line_break(indent + indent_step);
            brack_c = line_break(indent) + cl_brace;
            delimiter = "";
         }

         for (std::size_t i = 1; i < opnds_.size(); ++i) {
            os << delimiter;
            opnds_[i]->serialize(os, highlight, style, special, indent + indent_step, indent_step, line_len, data, visited);
            insertNewlineIfNecessary(style, os, indent + indent_step, line_len);

            if (StaticHelper::isEmptyExceptWhiteSpaces(delimiter)) {
               delimiter = line_break(indent) + cl_brace + line_break(indent) + "else " + op_brace  + line_break(indent + indent_step);
            }
         }
      }
      
      os << brack_c;
   }
   else if (getOpStruct().op_pos_type == OutputFormatEnum::plain) {
      os << resolveOptor(style);
   }
   else {
      std::stringstream ss;
      ss << "Output type '" << OutputFormat(getOpStruct().op_pos_type).getEnumAsString() << "' has not (yet?) been implemented.";
      throw std::runtime_error(ss.str());
   }
}

int vfm::MathStruct::getNodeCount()
{
   return getNodeCount(TraverseCompoundsType::avoid_compound_structures);
}

int vfm::MathStruct::getNodeCount(const TraverseCompoundsType include_compound_structures)
{
   int num = 0;

   applyToMeAndMyChildren([&num](const std::shared_ptr<MathStruct> m) {
      num++;
   }, include_compound_structures);

   return num;
}

int vfm::MathStruct::getLeafCount()
{
   return getLeafCount(TraverseCompoundsType::avoid_compound_structures);
}

int vfm::MathStruct::getLeafCount(const TraverseCompoundsType include_compound_structures)
{
   int num = 0;

   applyToMeAndMyChildren([&num](const std::shared_ptr<MathStruct> m) {
      if (m->getOperands().size() == 0) {
         num++;
      }
   }, include_compound_structures);

   return num;
}

int find_equation_sign(const std::string& formula) {
   return formula.find(Equation::my_struct.op_name);
}

std::shared_ptr<MathStruct> vfm::MathStruct::parseMathStruct(const std::string& formula, 
                                                             std::shared_ptr<FormulaParser> parser,
                                                             const std::shared_ptr<DataPack> data_to_retrieve_constants,
                                                             const DotTreatment dot_treatment)
{
   return parseMathStruct(formula, true, false, parser, data_to_retrieve_constants, dot_treatment);
}

std::shared_ptr<MathStruct> MathStruct::parseMathStruct(
   const std::string& formula,
   const bool preprocess_cpp_array_notation_and_brace_style_brackets,
   const bool preprocess_cpp_switchs_to_ifs_else_ifs_to_plain_ifs,
   std::shared_ptr<FormulaParser> parser,
   const std::shared_ptr<DataPack> data_to_retrieve_constants,
   const DotTreatment dot_treatment)
{
   return parseMathStruct(
      formula, 
      parser ? *parser : *SingletonFormulaParser::getInstance(),
      preprocess_cpp_array_notation_and_brace_style_brackets, 
      preprocess_cpp_switchs_to_ifs_else_ifs_to_plain_ifs, 
      DEFAULT_REGEX_FOR_TOKENIZER, 
      DEFAULT_IGNORE_CLASS_FOR_TOKENIZER, 
      {},
      data_to_retrieve_constants,
      AkaTalProcessing::none,
      dot_treatment);
}

std::shared_ptr<MathStruct> MathStruct::parseMathStruct(
   const std::string& formula, 
   FormulaParser& parser,
   const std::shared_ptr<DataPack> data_to_retrieve_constants,
   const DotTreatment dot_treatment)
{
   return parseMathStruct(
      formula, 
      parser, 
      true, 
      false, 
      DEFAULT_REGEX_FOR_TOKENIZER, 
      DEFAULT_IGNORE_CLASS_FOR_TOKENIZER, 
      {}, 
      data_to_retrieve_constants, 
      AkaTalProcessing::none, 
      dot_treatment);
}

std::shared_ptr<MathStruct> MathStruct::parseMathStruct(
   const std::string& formula_raw, 
   FormulaParser& parser, 
   const bool preprocess_cpp_array_notation_and_brace_style_brackets,
   const bool preprocess_cpp_switchs_to_ifs_else_ifs_to_plain_ifs,
   const std::vector<std::regex>& classes,
   const int ignore_class,
   const std::set<std::string>& ignore_tokens,
   const std::shared_ptr<DataPack> data_to_retrieve_constants,
   const AkaTalProcessing aka_processing,
   const DotTreatment dot_treatment)
{
   int found_eq = find_equation_sign(formula_raw);

   if (found_eq >= 0) { // TODO: So far no constants retrieval for equations.
      std::string before = formula_raw.substr(0, found_eq);
      std::string after_A = formula_raw.substr(found_eq + Equation::my_struct.op_name.size());
      
      size_t found_brack = after_A.find(CLOSING_MOD_BRACKET);

      if (found_brack != std::string::npos) {
         std::string after = after_A.substr(found_brack + 1, after_A.length() - 1);
         std::string modPart = after_A.substr(1, found_brack - 1);

         return std::make_shared<Equation>(
            parseMathStruct(before, parser, preprocess_cpp_array_notation_and_brace_style_brackets, preprocess_cpp_switchs_to_ifs_else_ifs_to_plain_ifs, classes, ignore_class, ignore_tokens, data_to_retrieve_constants, aka_processing, dot_treatment)->toTermIfApplicable(),
            parseMathStruct(after, parser, preprocess_cpp_array_notation_and_brace_style_brackets, preprocess_cpp_switchs_to_ifs_else_ifs_to_plain_ifs, classes, ignore_class, ignore_tokens, data_to_retrieve_constants, aka_processing, dot_treatment)->toTermIfApplicable(),
            parseMathStruct(modPart, parser, preprocess_cpp_array_notation_and_brace_style_brackets, preprocess_cpp_switchs_to_ifs_else_ifs_to_plain_ifs, classes, ignore_class, ignore_tokens, data_to_retrieve_constants, aka_processing, dot_treatment)->toTermIfApplicable());
      }
      else { // No explicit mod given.
         auto left = parseMathStruct(before, parser, preprocess_cpp_array_notation_and_brace_style_brackets, preprocess_cpp_switchs_to_ifs_else_ifs_to_plain_ifs, classes, ignore_class, ignore_tokens, data_to_retrieve_constants, aka_processing, dot_treatment);
         auto right = parseMathStruct(after_A, parser, preprocess_cpp_array_notation_and_brace_style_brackets, preprocess_cpp_switchs_to_ifs_else_ifs_to_plain_ifs, classes, ignore_class, ignore_tokens, data_to_retrieve_constants, aka_processing, dot_treatment);

         if (!left || !right) {
            return nullptr;
         }

         return std::make_shared<Equation>(left->toTermIfApplicable(), right->toTermIfApplicable());
      }
   }
   else {
      std::string preprocessed = formula_raw;
      if (preprocess_cpp_switchs_to_ifs_else_ifs_to_plain_ifs) {
         StaticHelper::preprocessCppConvertAllSwitchsToIfs(preprocessed);
         StaticHelper::preprocessCppConvertElseIfToPlainElse(preprocessed);
      }

      int start_pos = 0; // This is a dummy variable since we don't use the returned value from tokenize.
      preprocessed = parser.preprocessProgram(preprocess_cpp_array_notation_and_brace_style_brackets ? parser.preprocessCPPProgram(preprocessed) : preprocessed);

      std::shared_ptr<std::vector<std::string>> tokens = StaticHelper::tokenize(
         preprocessed, 
         parser, 
         start_pos, 
         std::numeric_limits<int>::max(), 
         false, 
         dot_treatment == DotTreatment::as_operator ? DEFAULT_REGEX_FOR_TOKENIZER_DOTLESS : classes, 
         ignore_class, 
         true, 
         true, 
         aka_processing, 
         ignore_tokens);

      StaticHelper::sanityCheckTokensFromCpp(*tokens);

      return parseMathStructFromTokens(tokens, parser, data_to_retrieve_constants);
   }
}

std::shared_ptr<MathStruct> MathStruct::parseMathStructFromTokens(
   const std::shared_ptr<std::vector<std::string>> formula_tokens,
   FormulaParser& parser,
   const std::shared_ptr<DataPack> data_to_retrieve_constants)
{
   std::shared_ptr<MathStruct> res = parser.parseShuntingYard(formula_tokens);

   if (res) {
      res->setChildrensFathers();
      res->applyImplicitLambdas(IMPLICIT_LAMBDAS);

      if (data_to_retrieve_constants) {
         res->applyToMeAndMyChildren([&data_to_retrieve_constants](const MathStructPtr m) {
            if (m->isTermVar()) {
               auto m_var = m->toVariableIfApplicable();
               auto var_name = m_var->getVariableName();

               if (data_to_retrieve_constants->isConst(var_name)) {
                  m_var->setConstVariable(data_to_retrieve_constants->getSingleVal(var_name));
               }
            }
            });
      }

      if (res->getOptor() != "@f") { // TODO: This needs to be integrated generally in the ternary rules for the case it occurs inside a formula.
         res = resolveTernaryOperators(res->toTermIfApplicable(), true, parser.shared_from_this()); // Can only be Term here, TODO: change return type of parseShuntingYard.
      }
   }

   return res;
}

void vfm::MathStruct::applyImplicitLambdas(const std::map<std::string, std::vector<int>>& implicit_lambdas)
{
   applyToMeAndMyChildren([&implicit_lambdas](const std::shared_ptr<MathStruct> m) {
      auto it{ implicit_lambdas.find(m->getOptor()) };

      if (it != implicit_lambdas.end()) {
         for (int i : it->second) {
            auto mi{ m->getOperands()[i] };

            if (!mi->isTermLambda()) {
               m->getOperands()[i] = _lambda(mi);
            }
         }

         m->setChildrensFathers(false, false);
      }
   });
}

int find(std::string& lines, const std::vector<std::string> & delimiters, const int& offset) {
   int i = std::numeric_limits<int>::max();

   for (auto s : delimiters) {
      i = std::min(static_cast<size_t>(i), lines.find(s, -offset));
   }

   return i;
}

std::vector<std::string> MathStruct::parseLines(std::string& lines, const std::vector<std::string>& delimiters, const int& offset = 0) {
   std::vector<std::string> vec{};
   size_t pos = find(lines, delimiters, 0);

   while (pos < lines.size()) {
      vec.push_back(lines.substr(0, pos));
      lines = lines.substr(pos + 1 + offset); // TODO: This is probably inefficient.
      pos = find(lines, delimiters, offset);
   }

   vec.push_back(lines);

   return vec;
}

/* 
 * Has to be re-run after every calculation step to allow for different
 * numbers when using ASMJIT.
 */
void MathStruct::randomize_rand_nums()
{
   if (rand_var_count > rand_vars.capacity()) throw std::exception(/*"Too many random numbers for assembly JIT calc."*/);

   while (rand_vars.size() < rand_var_count) {
      rand_vars.push_back(0);
   }

   for (int i = 0; i < rand_var_count; ++i) {
      rand_vars.at(i) = rand();
   }
}

std::vector<std::shared_ptr<Equation>> MathStruct::parseEquations(std::string& formulae)
{
   std::vector<std::shared_ptr<Equation>> vec{};
   std::vector<std::string> formula_vec = parseLines(formulae, { StaticHelper::makeString(PROGRAM_COMMAND_SEPARATOR) });

   for (std::string formula : formula_vec) {
      StaticHelper::trim(formula);
      if (!formula.empty()) vec.push_back(parseMathStruct(formula, true)->toEquationIfApplicable());
   }

   return vec;
}

std::shared_ptr<MathStruct> MathStruct::copy()
{
   if (this->isEquation()) {
      Equation *eq = dynamic_cast<Equation*>(this);
      return eq->copy();
   }
   else {
      Term *t = dynamic_cast<Term*>(this);
      return t->copy();
   }
}

std::string vfm::MathStruct::additionalOptionsForGraphvizNodeLabel()
{
   std::string additional_options = "";

   if (isTermVar() && StaticHelper::stringStartsWith(toVariableIfApplicable()->getOptor(), FORWARD_DECLARATION_PREFIX)) {
      additional_options += " color=red style=filled shape=diamond"; // Overriding the value from before.
   } 
   else if (assembly_created_ == AssemblyState::yes) {
      if (isTermVal() || isTermVar()) {
         additional_options += " fillcolor=lightyellow1 style=filled";
      }
      else {
         additional_options += " fillcolor=yellow style=filled";
      }
   }
   else if (assembly_created_ == AssemblyState::no) {
      additional_options += " fillcolor=lightgrey style=filled";
   }

   return additional_options;
}

std::string vfm::MathStruct::generateGraphvizWithFatherFollowing(const int spawn_children_threshold, const bool go_into_compounds)
{
   return generateGraphviz(
      spawn_children_threshold, 
      go_into_compounds, 
      std::make_shared<std::map<std::shared_ptr<MathStruct>, std::string>>(std::map<std::shared_ptr<MathStruct>, std::string>{}));
}

std::string MathStruct::generateGraphviz(
   const int spawn_children_threshold,
   const bool go_into_compounds,
   const std::shared_ptr<std::map<std::shared_ptr<MathStruct>, std::string>> follow_fathers,
   const std::string cmp_counter,
   const bool root,
   const std::string& node_name,
   const std::shared_ptr<std::map<std::shared_ptr<Term>, std::string>> visited,
   const std::shared_ptr<std::string> only_compound_edges_to)
{
   std::string s{};
   int count{ 0 };
   bool spawnChildNodes{ true };
   std::string additional_options{ additionalOptionsForGraphvizNodeLabel() };

   if (follow_fathers) {
      follow_fathers->insert({ shared_from_this(), node_name });
   }

   const std::string invalid = (isValid() ? "" : " <invalid>");
   if (!invalid.empty()) {
      additional_options += ", color=red";
   }

   if (root) {
      additional_options += ", penwidth=3";
   }

   if (!only_compound_edges_to) {
      s += node_name + " [label=\"" + getOptor() + invalid + "\" " + additional_options + "];\n";
   }

   std::string serialized_as_text = serialize() + invalid;
   if (!root && serialized_as_text.size() <= spawn_children_threshold && !only_compound_edges_to) {
      s = node_name + " [label=\"" + serialized_as_text + "\" " + additional_options + "];\n";
      spawnChildNodes = false;
   }

   auto actual_father{ getFatherJumpOverCompound() };

   if (follow_fathers) {
      if (actual_father) {
         if (follow_fathers->count(actual_father)) {
            s += node_name + "->" + follow_fathers->at(actual_father) + " [arrowhead=dot,color=grey];\n";
         }
         else {
            s += actual_father->generateGraphviz(
               spawn_children_threshold,
               go_into_compounds,
               follow_fathers,
               cmp_counter,
               false,
               "f" + cmp_counter + node_name,
               visited, 
               only_compound_edges_to);
            s += node_name + "->" + "f" + cmp_counter + node_name + " [arrowhead=dot,color=red];\n";
         }
      }
      else {
         s += node_name + "_deadend [label=\"\",shape=none,color=grey];\n";
         s += node_name + "->" + node_name + "_deadend [arrowhead=dot,color=grey];\n";
      }
   }

   std::vector<std::string> child_names{};

   for (const auto& term : opnds_) {
      std::string name = node_name + "_" + std::to_string(count);

      if (only_compound_edges_to) {
         s += term->generateGraphviz(spawn_children_threshold, go_into_compounds, follow_fathers, cmp_counter + std::to_string(count), false, node_name, visited, only_compound_edges_to);
      }
      else {
         if (spawnChildNodes) {
            std::string serialized;

            if (follow_fathers && follow_fathers->count(term)) {
               name = follow_fathers->at(term);
            }
            else {
               serialized = term->generateGraphviz(spawn_children_threshold, go_into_compounds, follow_fathers, cmp_counter + std::to_string(count), false, name, visited);
            }

            if (!serialized.empty() && serialized[0] == '$') {
               std::string actual_name{ serialized.substr(3) };
               s += node_name + "->" + actual_name + ";\n";

               if (actual_name == name || node_name == name) {
                  child_names.push_back(name);
               }
            }
            else {
               child_names.push_back(name);
               s += node_name + "->" + name + ";\n";
               s += serialized;
            }
         }
         else {
            auto node_name_ptr = std::make_shared<std::string>(node_name);
            s += term->generateGraphviz(spawn_children_threshold, go_into_compounds, follow_fathers, cmp_counter + std::to_string(count), false, node_name, visited, node_name_ptr);
         }
      }

      count++;
   }

   if (child_names.size() > 1) {
      s += "{rank=same; ";
      s += child_names.at(0);
      for (size_t i = 1; i < child_names.size(); i++) {
         s += "->" + child_names.at(i);
      }
      s += " [style = invis];}\n";
   }

   if (root) {
      s = "digraph G {\n"
         + std::string("root_title [label=\"") + StaticHelper::shortenToMaxSize(serialize(), 250) + "\" color=white];\n"
         + s + "}";
   }

   return s;
}

int vfm::MathStruct::depth_to_farthest_child()
{
   if (opnds_.empty()) {
      return 0;
   }
 
   int depth = 0;

   for (const auto& child : opnds_) {
      depth = std::max(depth, child->depth_to_farthest_child() + 1);
   }

   return depth;
}

int vfm::MathStruct::depth_from_root()
{
   std::shared_ptr<int> count = std::make_shared<int>(-1);

   applyToMeAndMyParents([count] (std::shared_ptr<MathStruct>) {
       (*count)++;
   });

   return *count;
}

const OutputFormatEnum CONST_OUT_TYPE = OutputFormatEnum::prefix;
const AssociativityTypeEnum CONST_ASSOCIATIVITY_TYPE = AssociativityTypeEnum::left;
const int CONST_PRIO = -1;
const bool CONST_ASSOCIATIVE = false;
const bool CONST_COMMUTATIVE = false;

bool vfm::MathStruct::extractSubTermToFunction(
    const std::shared_ptr<FormulaParser> parser, 
    const float var_parameter_probability, 
    const float val_parameter_probability, 
    const int seed,
    const int min_depth,
    const int min_dist_to_root,
    const bool simplify_first)
{
   if (isTermCompound() && isAutoExtractedName(child0()->getOptor()) || depth_from_root() < min_dist_to_root || depth_to_farthest_child() < min_depth) {
      return false;
   }

   if (isCompoundStructure() || isCompoundOperator()) {
      return false;
   }

   std::string name = "_" + parser->getCurrentAutoExtractedFunctionName() + "_";
   auto operands = std::make_shared<std::vector<std::shared_ptr<Term>>>();
   auto op_count = std::make_shared<int>(0);
   srand(seed);

   auto cp = copy()->toTermIfApplicable();

   if (simplify_first) {
       //cp = simplify(cp);
   }

   auto id_term = _id(cp);
   id_term->setChildrensFathers(false, false);
   cp->applyToMeAndMyChildren([operands, op_count, var_parameter_probability, val_parameter_probability](std::shared_ptr<MathStruct> m) {
      float p = (float) rand() / (float) RAND_MAX;
      if (m->isTermVal() && p <= val_parameter_probability || m->isTermVar() && p <= var_parameter_probability) {
         operands->push_back(m->toTermIfApplicable());
         auto temp_term = std::make_shared<TermVal>(*op_count);
         auto replace_term = std::make_shared<TermMetaCompound>(temp_term);
         auto ptr_to_father_m = m->father_.lock();
         if (ptr_to_father_m) {
            ptr_to_father_m->replaceOperand(m->toTermIfApplicable(), replace_term);
         }
         (*op_count)++;
      }
   });

   OperatorStructure opstruct(CONST_OUT_TYPE, CONST_ASSOCIATIVITY_TYPE, *op_count, CONST_PRIO, name, CONST_ASSOCIATIVE, CONST_COMMUTATIVE);

   parser->addDynamicTerm(opstruct, id_term->getTermsJumpIntoCompounds()[0]);
   std::shared_ptr<Term> t_new{ parser->termFactory(name, *operands) };

   auto ptr_to_father{ getFatherJumpOverCompound() };
   if (ptr_to_father) {
      ptr_to_father->replaceOperandLookIntoCompounds(toTermIfApplicable(), t_new, false);
      ptr_to_father->getPtrToRoot()->setChildrensFathers();
   }

   parser->setCurrentAutoExtractedFunctionName(StaticHelper::incrementAlphabeticNum(parser->getCurrentAutoExtractedFunctionName()));

   return true;
}

std::shared_ptr<MathStruct> vfm::MathStruct::getUniformlyRandomSubFormula(
   const int seed, 
   const bool avoid_root, 
   const bool enforce_leaf,
   const std::function<bool(std::shared_ptr<MathStruct>)>& f)
{
   std::vector<std::shared_ptr<MathStruct>> applicables{};

   applyToMeAndMyChildren([&f, &applicables, avoid_root, enforce_leaf](const std::shared_ptr<MathStruct> m) {
      if ((!avoid_root || !m->isRootTerm()) && (!enforce_leaf || m->getOperands().empty()) && f(m)) {
         if (!m->isCompoundStructure() && !m->isCompoundOperator()) {
            applicables.push_back(m);
         }
      }
   });

   if (applicables.empty()) {
      return nullptr;
   }
   else if (applicables.size() == 1) { // Speed up a bit since this is a very common case when evolving.
      return applicables[0];
   }
   else {
      std::mt19937 eng;
      eng.seed(seed);
      std::uniform_int_distribution<> distr(0, applicables.size() - 1);
      const int random_node_num = distr(eng);

      return applicables.at(random_node_num);
   }
}

bool MathStruct::isEquation()
{
   return typeid(*this) == typeid(Equation);
}

bool MathStruct::isTerm()
{
   return !isEquation();
}

bool vfm::MathStruct::checkIfAllChildrenHaveConsistentFatherQuiet(const bool recursive) const
{
   return checkIfAllChildrenHaveConsistentFatherRaw(recursive, true, true);
}

bool vfm::MathStruct::checkIfAllChildrenHaveConsistentFather(const bool recursive) const
{
   return checkIfAllChildrenHaveConsistentFatherRaw(recursive, true, false);
}

bool vfm::MathStruct::checkIfAllChildrenHaveConsistentFatherRaw(const bool recursive, const bool root, const bool quiet) const
{
   bool ok{ true };

   for (auto t : opnds_) {
      auto father = t->getFather();

      if (father) {
         if (father != shared_from_this()) {
            Failable::getSingleton("FatherConsistency")->addError("Father of '" + t->serializeWithinSurroundingFormulaPlainOldVFMStyle() + "' should be '" + serializePlainOldVFMStyle() + "', but is actually '" + father->serializePlainOldVFMStyle() + "'.");
            Failable::getSingleton("FatherConsistency")->addError("This is the tree: '" + const_cast<MathStruct*>(this)->getPtrToRoot()->generateGraphvizWithFatherFollowing(0, false) + "'.");
            ok = false;
         }
      }
      else {
         if (!t->isCompoundStructure()) {
            Failable::getSingleton("FatherConsistency")->addError("'" + t->serialize() + "' has no father in '" + serialize(t) + "'.");
            Failable::getSingleton("FatherConsistency")->addError("This is the tree: '" + const_cast<MathStruct*>(this)->getPtrToRoot()->generateGraphvizWithFatherFollowing(0, false) + "'.");
            ok = false;
         }
      }

      if (recursive && !t->checkIfAllChildrenHaveConsistentFatherRaw(true, false, quiet)) {
         ok = false;
      }
   }

   if (ok && root && !quiet) {
      Failable::getSingleton("FatherConsistency")->addNote("Fathers are all consistent in '" + serializePlainOldVFMStyle() + "'.");
   }

   return ok;
}

std::shared_ptr<MathStruct> MathStruct::setChildrensFathers(
   const bool recursive, 
   const bool including_compound_structure, 
   std::set<std::shared_ptr<MathStruct>>&& visited)
{
   return MathStruct::setChildrensFathers(
      including_compound_structure ? FatherSetterStyle::go_into_compound_structures : FatherSetterStyle::avoid_compound_structures,
      recursive,
      std::move(visited));
}

std::shared_ptr<MathStruct> MathStruct::setChildrensFathers(
   const FatherSetterStyle stype,
   const bool recursive,
   std::set<std::shared_ptr<MathStruct>>&& visited)
{
   const auto ptr_to_this{ shared_from_this() };

   if (!recursive && isTermCompound()) {
      // If non-recursive it's almost certain we want to set the father of the compound operator, not (only) the compound itself.
      opnds_[0]->setChildrensFathers(stype, recursive);
   }

   makeValid();

   if (!visited.count(ptr_to_this)) {
      visited.insert(ptr_to_this);

      for (auto t : ptr_to_this->opnds_) {
         if (t) {
            if (t->isCompoundStructure()) {
               // The father of a compound structure is always empty. For the case father_ should have been set somewhere else, we fix that here.
               t->father_.reset();
            }
            else {
               t->father_ = ptr_to_this;
               t->makeValid();
            }

            if (recursive && (stype == FatherSetterStyle::go_into_compound_structures || !t->isCompoundStructure())) {
               t->setChildrensFathers(stype, recursive, std::move(visited));
            }
         }
      }
   }

   return shared_from_this();
}

std::vector<std::shared_ptr<Term>>& MathStruct::getOperands()
{
   return opnds_;
}

std::vector<std::shared_ptr<Term>>& MathStruct::getOperandsConst() const
{
   return const_cast<MathStruct*>(this)->opnds_;
}

std::shared_ptr<Term> MathStruct::child0() const { return opnds_[0]; }
std::shared_ptr<Term> MathStruct::child1() const { return opnds_[1]; }
std::shared_ptr<Term> MathStruct::child2() const { return opnds_[2]; }
std::shared_ptr<Term> MathStruct::child3() const { return opnds_[3]; }
std::shared_ptr<Term> MathStruct::child4() const { return opnds_[4]; }

std::vector<std::shared_ptr<Term>>& vfm::MathStruct::getTermsJumpIntoCompounds()
{
   return isTermCompound() ? opnds_[0]->opnds_ : opnds_;
}

std::shared_ptr<Term> MathStruct::child0JumpIntoCompounds() const { return isTermCompound() ? opnds_[0]->opnds_[0] : opnds_[0]; }
std::shared_ptr<Term> MathStruct::child1JumpIntoCompounds() const { return isTermCompound() ? opnds_[0]->opnds_[1] : opnds_[1]; }
std::shared_ptr<Term> MathStruct::child2JumpIntoCompounds() const { return isTermCompound() ? opnds_[0]->opnds_[2] : opnds_[2]; }
std::shared_ptr<Term> MathStruct::child3JumpIntoCompounds() const { return isTermCompound() ? opnds_[0]->opnds_[3] : opnds_[3]; }
std::shared_ptr<Term> MathStruct::child4JumpIntoCompounds() const { return isTermCompound() ? opnds_[0]->opnds_[4] : opnds_[4]; }

std::vector<std::shared_ptr<Term>> vfm::MathStruct::getTermsJumpIntoCompoundsIncludingCompoundStructure()
{
   if (isTermCompound()) {
      auto res{ opnds_[0]->opnds_ };
      auto cs_terms{ opnds_[1]->opnds_ };

      res.insert(res.end(), cs_terms.begin(), cs_terms.end());

      return res;
   }

   return opnds_;
}

bool isCommandBlock(const std::string& optor, const int arg_pos = -1) {
   return optor == SYMB_SEQUENCE
      || (optor == SYMB_IF       && (arg_pos > 0  || arg_pos < 0))
      || (optor == SYMB_IF_ELSE  && (arg_pos > 0  || arg_pos < 0))
      || (optor == SYMB_WHILE    && (arg_pos > 0  || arg_pos < 0))
      || (optor == SYMB_WHILELIM && (arg_pos == 1 || arg_pos < 0))
      || (optor == SYMB_FOR_C    && (arg_pos == 3 || arg_pos < 0));
}

TermPtr vfm::MathStruct::randomProgram(
   const int line_count,
   const int line_depth,
   const long seed_raw,
   const int max_const_val,
   const int max_array_val,
   const bool workaround_for_mod_and_div,
   const std::shared_ptr<FormulaParser> parser_raw,
   const std::set<std::string>& inner_variables,
   const std::set<std::string>& input_variables,
   const std::set<std::string>& output_variables,
   const std::set<std::set<std::string>>& allowed_symbols_raw
)
{
   auto de_facto_output_vars = output_variables;

   de_facto_output_vars.insert(inner_variables.begin(), inner_variables.end());

   auto parser = parser_raw;
   if (!parser) {
      parser = SingletonFormulaParser::getInstance();
   }

   auto allowed_symbols = allowed_symbols_raw;
   std::srand(seed_raw);
   long seed = std::rand();

   allowed_symbols.insert(inner_variables);

   if (line_count <= 0) return _val0();

   TermPtr program = randomTerm(line_depth, allowed_symbols, max_const_val, max_array_val, seed++, workaround_for_mod_and_div, parser, AssociativityEnforce::none, {}, {}, true, true, input_variables, output_variables);

   if (!program->hasSideeffects() && !isCommandBlock(program->getOptor())) {
      auto it = std::begin(de_facto_output_vars);
      std::advance(it, std::rand() % de_facto_output_vars.size());
      program = parser->termFactory(SYMB_SET_VAR_A, { _var(SYMB_REF + *it), program });
   }

   for (int i = 1; i < line_count; i++) {
      auto new_term = randomTerm(line_depth, allowed_symbols, max_const_val, max_array_val, seed++, workaround_for_mod_and_div, parser, AssociativityEnforce::none, {}, {}, true, true, input_variables, output_variables);

      if (!new_term->hasSideeffects() && !isCommandBlock(new_term->getOptor())) {
         auto it = std::begin(de_facto_output_vars);
         std::advance(it, std::rand() % de_facto_output_vars.size());
         new_term = parser->termFactory(SYMB_SET_VAR_A, { _var(SYMB_REF + *it), new_term });
      }

     program = _seq(program, new_term);
   }

   return program;
}

std::shared_ptr<Term> MathStruct::randomTerm(
   const int depth, 
   const std::set<std::set<std::string>>& allowed_symbs, 
   const int max_const_val, 
   const int max_array_val,
   const long seed,
   const bool workaround_for_mod_and_div_etc,
   const std::shared_ptr<FormulaParser> parser_temp,
   const AssociativityEnforce enforce_associativity_for_associative_operators,
   const std::vector<std::string>& use_variables_instead_of_const_val,
   const std::set<std::string>& disallowed_symbols,
   const bool enforce_variable_on_left_side_of_assignment,
   const bool enforce_sound_if_and_assignment_structures,
   const std::set<std::string>& input_variables,
   const std::set<std::string>& output_variables
)
{
   std::shared_ptr<Term> res{ nullptr };
   auto emergency_set{ std::set<std::string>({ "::var_placeholder::" }) };

   std::set<std::string> var_names_input{ input_variables };
   std::set<std::string> var_names_output{ output_variables };
   auto parser{ parser_temp ? parser_temp : SingletonFormulaParser::getInstance() };

   std::set<std::string> allowed_symbols{};
   for (const auto& symb_set : allowed_symbs) {
      for (const auto& symb : symb_set) {
         if (!disallowed_symbols.count(symb)) {
            allowed_symbols.insert(symb);
         }
      }
   }

   for (const auto& symb : allowed_symbols) {
      if (!parser->isFunctionOrOperator(symb)) {
         var_names_input.insert(symb);
         var_names_output.insert(symb);
      }
   }

   std::srand(seed);

   if (depth <= 0 || allowed_symbols.empty()) {
      if (use_variables_instead_of_const_val.empty()) {
         TermPtr const_ptr;

         if (!input_variables.empty() && rand() % 2) {
            std::set<std::string>::const_iterator it3(input_variables.begin());
            int num3 = std::rand() % input_variables.size();
            advance(it3, num3);
            const_ptr = _var(*it3);
         }
         else {
            const_ptr = _val(std::rand() % (max_const_val - 1) + 1);
         }

         if (enforce_sound_if_and_assignment_structures // If setting a variable is allowed, do it.
             && (allowed_symbols.count(SYMB_SET_VAR_A) && !disallowed_symbols.count(SYMB_SET_VAR_A)
                 || allowed_symbols.count(SYMB_SET_VAR) && !disallowed_symbols.count(SYMB_SET_VAR))) {
            std::set<std::string>::const_iterator it(var_names_output.begin());
            
            if (var_names_output.empty()) {
               Failable::getSingleton()->addError("Cannot enforce_sound_if_and_assignment_structures due to empty set of variables.");
               it = emergency_set.begin();
            }
            else {
               advance(it, std::rand() % var_names_output.size());
            }
            
            return parser->termFactory(SYMB_SET_VAR_A, { _var(SYMB_REF + *it), const_ptr });
         }
         else {
            return const_ptr;
         }
      }
      else {
         return _var(use_variables_instead_of_const_val[std::rand() % use_variables_instead_of_const_val.size()]);
      }
   }

   std::set<std::string>::const_iterator it(allowed_symbols.begin());
   int num = std::rand() % allowed_symbols.size();
   std::advance(it, num);

   if (enforce_sound_if_and_assignment_structures) {
      if (!parser->termFactoryDummy(*it)) { // It's a variable or constant.
         if (allowed_symbols.count(SYMB_SET_VAR) && !disallowed_symbols.count(SYMB_SET_VAR)) {
            it = allowed_symbols.find(SYMB_SET_VAR);
         }
         else if (allowed_symbols.count(SYMB_SET_VAR_A) && !disallowed_symbols.count(SYMB_SET_VAR_A)) {
            it = allowed_symbols.find(SYMB_SET_VAR_A);
         }
      }
   }

   auto temp_vec = std::vector<std::shared_ptr<Term>>();

   auto num_paramss = parser->getNumParams(*it);
   int num_params = 0;

   if (!num_paramss.empty()) { // Use random number of parameters if there are overloaded functions.
      auto it2 = std::begin(num_paramss);
      int rand_num = std::rand() % num_paramss.size();
      std::advance(it2, rand_num);
      num_params = *it2;
   }

   int realDepth[3];
   realDepth[0] = depth;
   realDepth[1] = depth;
   realDepth[2] = depth;

   if (workaround_for_mod_and_div_etc) {
      // Special treatment for rare (and meaningless!) cases where assembly and interpreter results differ.
      if (*it == SYMB_MOD)  { realDepth[0] = 1; realDepth[1] = 1; } // This is to avoid mod(x/y) like terms which occasionally fail in tests and might obscure other problems...
      if (*it == SYMB_DIV)  { realDepth[1] = 1; } // This is to avoid x/0 like terms which fail when used in logic and/or terms and might obscure other problems during testing...
      if (*it == SYMB_SQRT || *it == SYMB_RSQRT) { realDepth[0] = 1; } // Like the directly above, only for sqrt.
      if (*it == SYMB_MALLOC) { return _malloc(5); } // Allow only malloc with constant 5.
   }

   std::vector<std::shared_ptr<Term>> ops2{};

   for (int i = 0; i < num_params; ++i)
   {
      if (i == 0 && enforce_variable_on_left_side_of_assignment 
          && (*it == SYMB_SET_VAR || *it == SYMB_SET_VAR_A)) {

         std::set<std::string>::const_iterator it2(var_names_output.begin());

         if (var_names_output.empty()) {
            Failable::getSingleton()->addError("Cannot enforce_variable_on_left_side_of_assignment due to empty set of variables.");
            it2 = emergency_set.begin();
         }
         else {
            int num2 = std::rand() % var_names_output.size();
            advance(it2, num2);
         }

         ops2.push_back(_var(SYMB_REF + *it2));
      }
      else {
         auto allowed_symbs_final = allowed_symbs;
         std::set<std::string> disallowed_symbs_new;

         if (enforce_associativity_for_associative_operators == AssociativityEnforce::right && i == 0
             || enforce_associativity_for_associative_operators == AssociativityEnforce::left && i == 1) {
            auto dummy_term = parser->termFactoryDummy(*it);

            if (dummy_term->isTermCompound()) {
               dummy_term = dummy_term->child0();
            }

            if (dummy_term->getOpStruct().op_pos_type == OutputFormatEnum::infix && dummy_term->getOpStruct().assoc) {
               allowed_symbs_final.clear();
               std::set<std::string> allowed_symbs_plain_set{};
               std::set<std::string> to_remove{}; // We do only stuff to operators, i.e. par_num = 2.

               for (const auto& optors : parser->getAllOps()) {
                  for (const auto& optor : optors.second) {
                     if (optor.second.op_pos_type == OutputFormatEnum::infix && optor.second.assoc && optor.second.precedence == dummy_term->getOpStruct().precedence) {
                        to_remove.insert(optors.first);
                     }
                  }
               }

               for (const auto& symb : allowed_symbols) {
                  if (to_remove.find(symb) == to_remove.end()) {
                     allowed_symbs_plain_set.insert(symb);
                  }
               }

               allowed_symbs_final.insert(allowed_symbs_plain_set);
            }
         }

         if (enforce_sound_if_and_assignment_structures) {
            if (isCommandBlock(*it, i)) {
               std::set<std::string> allowed_special = { SYMB_DELETE, SYMB_FOR, SYMB_FOR_C, SYMB_FUNC_EVAL, SYMB_FUNC_REF, SYMB_IF, 
                  SYMB_IF_ELSE, SYMB_MALLOC, SYMB_PRINT, SYMB_SEQUENCE, SYMB_SET_ARR, SYMB_SET_VAR, SYMB_SET_VAR_A, SYMB_WHILE, SYMB_WHILELIM };
               allowed_special.insert(var_names_output.begin(), var_names_output.end());

               for (const auto& symb_set : allowed_symbs_final) {
                  for (const auto& symb : symb_set) {
                     if (!allowed_special.count(symb)) {
                        disallowed_symbs_new.insert(symb);
                     }
                  }
               }
            } else {
               disallowed_symbs_new.insert(SYMB_IF_ELSE);
               disallowed_symbs_new.insert(SYMB_IF);
               disallowed_symbs_new.insert(SYMB_WHILE);
               disallowed_symbs_new.insert(SYMB_FOR_C);
               disallowed_symbs_new.insert(SYMB_SEQUENCE);
               disallowed_symbs_new.insert(SYMB_SET_VAR);
               disallowed_symbs_new.insert(SYMB_SET_VAR_A);
            }
         }

         ops2.push_back(randomTerm(
            realDepth[i] - 1,
            allowed_symbs_final,
            max_const_val, max_array_val,
            std::rand(),
            workaround_for_mod_and_div_etc,
            parser,
            enforce_associativity_for_associative_operators,
            use_variables_instead_of_const_val,
            disallowed_symbs_new,
            enforce_variable_on_left_side_of_assignment, 
            enforce_sound_if_and_assignment_structures,
            input_variables,
            output_variables));
      }
   }

   res = parser->termFactory(*it, ops2);
   if (parser->isArray(*it)) {
      res = std::make_shared<TermArray>(randomTerm(
         0,
         allowed_symbs, 
         max_array_val, 
         max_array_val, 
         std::rand(), 
         workaround_for_mod_and_div_etc, 
         parser, 
         enforce_associativity_for_associative_operators, 
         use_variables_instead_of_const_val,
         {},
         enforce_variable_on_left_side_of_assignment,
         enforce_sound_if_and_assignment_structures,
         input_variables,
         output_variables), *it);
   }
   else if (!res) { // Assume it's a variable if optor is unknown.
      if (std::rand() % 2) {
         res = _var(*it);
      } else { // Go for value sometimes.
         res = _val(std::rand() % (max_const_val - 1) + 1);
      }
   }

   res->setChildrensFathers();

   return res;
}

std::shared_ptr<Term> vfm::MathStruct::randomTerm(
   const std::vector<std::string>& use_variables_instead_of_const_val,
   const int depth,
   const long seed,
   const std::set<std::set<std::string>>& operators,
   const std::shared_ptr<FormulaParser> parser)
   {
      assert(!use_variables_instead_of_const_val.empty()); // Just a gatekeeper to fetch possibly accidental calling of this method rather than the regular one.

      return randomTerm(depth, operators, 256, 256, seed, true, parser, AssociativityEnforce::none, use_variables_instead_of_const_val);
   }

std::shared_ptr<Term> vfm::MathStruct::randomCBTerm(
   const int regular_term_depth,
   const int cb_term_depth,
   const std::set<std::set<std::string>>& allowed_symbols,
   const std::vector<std::string>& var_names,
   const int max_const_val, 
   const int max_array_val, 
   const long seed, 
   const bool workaround_for_mod_and_div, 
   const std::shared_ptr<FormulaParser> parser)
{
   auto symbs_with_vars = allowed_symbols;
   symbs_with_vars.insert(std::set<std::string>(var_names.begin(), var_names.end()));
   auto term1 = randomTerm(regular_term_depth, symbs_with_vars, max_const_val, max_array_val, seed, workaround_for_mod_and_div, parser);
   int var_num = std::rand() % var_names.size();

   if (!cb_term_depth) {
      return _set(var_names[var_num], term1);
   }

   int prob1 = 33;
   int prob2 = 66;

   std::srand(seed);

   int rand_num = std::rand() % 100;

   if (rand_num < prob1) { // set
      return _seq(_set(var_names[var_num], term1), randomCBTerm(regular_term_depth, cb_term_depth - 1, allowed_symbols, var_names, max_const_val, max_array_val, std::rand(), workaround_for_mod_and_div, parser));
   }
   else if (rand_num < prob2) { // if
      return _if(term1, randomCBTerm(regular_term_depth, cb_term_depth - 1, allowed_symbols, var_names, max_const_val, max_array_val, std::rand(), workaround_for_mod_and_div, parser));
   }
   else { // ifelse
      return _ifelse(
         term1, 
         randomCBTerm(regular_term_depth, cb_term_depth - 1, allowed_symbols, var_names, max_const_val, max_array_val, std::rand(), workaround_for_mod_and_div, parser),
         randomCBTerm(regular_term_depth, cb_term_depth - 1, allowed_symbols, var_names, max_const_val, max_array_val, std::rand(), workaround_for_mod_and_div, parser));
   }
}

void MathStruct::addCompoundStructureReference(TermCompound* ptr_to_owning_compound)
{
    compound_structure_references_.push_back(ptr_to_owning_compound);
}

bool MathStruct::isCompoundStructure() const
{
    return !compound_structure_references_.empty();
}

std::vector<std::weak_ptr<TermCompound>> vfm::MathStruct::getCompoundStructureReferences() const
{
   std::vector<std::weak_ptr<TermCompound>> vec;

   for (const auto& el : compound_structure_references_) {
      vec.push_back(el->toTermCompoundIfApplicable());
   }

   return vec;
}

std::shared_ptr<Equation> vfm::MathStruct::toEquationIfApplicable()
{
   return nullptr;
}

std::shared_ptr<TermVar> vfm::MathStruct::toVariableIfApplicable()
{
   return nullptr;
}

std::shared_ptr<TermVal> vfm::MathStruct::toValueIfApplicable()
{
   return nullptr;
}

std::shared_ptr<TermArray> vfm::MathStruct::toArrayIfApplicable()
{
   return nullptr;
}

std::shared_ptr<TermSetVar> vfm::MathStruct::toSetVarIfApplicable()
{
   return nullptr;
}

std::shared_ptr<TermSetArr> vfm::MathStruct::toSetArrIfApplicable()
{
   return nullptr;
}

std::shared_ptr<TermGetArr> vfm::MathStruct::toGetArrIfApplicable()
{
   return nullptr;
}

std::shared_ptr<TermMetaCompound> MathStruct::toMetaCompoundIfApplicable()
{
   return nullptr;
}

std::shared_ptr<TermMetaSimplification> MathStruct::toMetaSimplificationIfApplicable()
{
   return nullptr;
}

std::shared_ptr<TermOptional> vfm::MathStruct::toTermOptionalIfApplicable()
{
   return nullptr;
}

std::shared_ptr<TermAnyway> vfm::MathStruct::toTermAnywayIfApplicable()
{
   return nullptr;
}

std::shared_ptr<TermCompound> MathStruct::toTermCompoundIfApplicable()
{
   return nullptr;
}

std::shared_ptr<TermCompoundOperator> MathStruct::toCompoundOperatorIfApplicable()
{
   return nullptr;
}

bool MathStruct::isMetaCompound() const
{
   return false;
}

bool MathStruct::isMetaSimplification() const
{
   return false;
}

bool MathStruct::isTermOptional() const
{
   return false;
}

bool vfm::MathStruct::isTermPrint() const
{
   return false;
}

bool vfm::MathStruct::isTermPlus() const
{
   return false;
}

bool vfm::MathStruct::isTermLambda() const
{
   return false;
}

bool vfm::MathStruct::isTermGetVar() const
{
   return false;
}

bool vfm::MathStruct::isTermLogic() const
{
   return false;
}

bool MathStruct::isTermIf() const
{
   return false;
}

bool MathStruct::isTermIfelse() const
{
   return false;
}

bool MathStruct::isTermSequence() const
{
   return false;
}

bool vfm::MathStruct::isTermNot() const
{
   return false;
}

bool vfm::MathStruct::isTermIdentity() const
{
   return false;
}

bool vfm::MathStruct::isTermBool() const
{
   return false;
}

bool MathStruct::isTermCompound() const
{
   return false;
}

bool MathStruct::isCompoundOperator() const
{
   return false;
}

bool vfm::MathStruct::isTermSetVarOrAssignment() const
{
   return false;
}

bool vfm::MathStruct::isTermGetArr() const
{
   return false;
}

bool vfm::MathStruct::isTermSetArr() const
{
   return false;
}

bool vfm::MathStruct::isTermVal() const
{
   return false;
}

bool vfm::MathStruct::isTermVarOnLeftSideOfAssignment() const
{
   return false;
}

bool vfm::MathStruct::isTermVarNotOnLeftSideOfAssignment() const
{
   return false;
}

bool vfm::MathStruct::containsMetaCompound() const
{
   bool contains_meta{ false };

   const_cast<MathStruct*>(this)->applyToMeAndMyChildrenIterative([&contains_meta](const MathStructPtr& m) {
      contains_meta = m->isMetaCompound();
   }, TraverseCompoundsType::avoid_compound_structures, contains_meta);

   return contains_meta;
}

bool vfm::MathStruct::isSingleLineIfTypeLimited() const
{
   const auto this_ptr{ const_cast<MathStruct*>(this)->toTermIfApplicable() };

   if (!this_ptr || this_ptr->isCompoundOperator() || this_ptr->isTermSequence()) {
      return false; // It's either not a term (i.e. equation) or not a TermCompound or not a single line but a sequence of >1 lines.
   }

   auto father{ this_ptr->getFather() };
   if (father) father = father->thisPtrGoUpToCompound();

   return father && 
      (father->isTermSequence()
       || (father->isTermIf() || father->isTermIfelse()) && father->getTermsJumpIntoCompounds()[1] == this_ptr // It's the single statement of a "then" case.
       || (father->isTermIfelse() && father->getTermsJumpIntoCompounds()[2] == this_ptr));                     // It's the single statement of an "else" case.
}

bool vfm::MathStruct::hasLTLOperators() const
{
   std::shared_ptr<bool> has_ltl_operators{ std::make_shared<bool>(false) };

   const_cast<MathStruct*>(this)->applyToMeAndMyChildren([has_ltl_operators](const MathStructPtr m) {
      for (const auto& el : NUSMV_LTL_OERATORS) {
         if (el.op_name == m->getOptor()) {
            *has_ltl_operators = true;
            break;
         }
      }
   }, TraverseCompoundsType::avoid_compound_structures, nullptr, has_ltl_operators);

   return *has_ltl_operators;
}

bool MathStruct::isOverallConstant(const std::shared_ptr<std::vector<int>> non_const_metas, const std::shared_ptr<std::set<std::shared_ptr<MathStruct>>> visited)
{
   if (hasSideeffectsThis()) {
      return false;
   }

   bool all_constant = true;

   for (const auto& term : opnds_) {
      all_constant = all_constant && term->isOverallConstant(non_const_metas, visited);
   }

   return all_constant;
}

bool vfm::MathStruct::consistsPurelyOfNumbers()
{
   bool has_non_number_term{ false };

   applyToMeAndMyChildrenIterative([&has_non_number_term](const MathStructPtr m) {
      if (m->getOperands().empty() && !m->toValueIfApplicable() || m->isMetaCompound()) {
         has_non_number_term = true;
      }
   }, TraverseCompoundsType::avoid_compound_structures, has_non_number_term);

   return !has_non_number_term;
}

bool MathStruct::isAlwaysTrue()
{
   return isOverallConstant() && constEval(); // TODO: Use assembly if available.
}

bool MathStruct::isAlwaysFalse()
{
   return isOverallConstant() && !constEval(); // TODO: Use assembly if available.
}

bool MathStruct::hasSideeffectsThis() const
{
   return false;
}

bool vfm::MathStruct::hasSideeffects() const
{
   bool sideeffects{ false };

   const_cast<MathStruct*>(this)->applyToMeAndMyChildrenIterative([&sideeffects](std::shared_ptr<MathStruct> m) {
      sideeffects = sideeffects || m->hasSideeffectsThis();
   }, TraverseCompoundsType::avoid_compound_structures, sideeffects); // const_cast since applyToMeAndMyChildren is const in this case, but cannot know it.

   return sideeffects;
}

#if defined(ASMJIT_ENABLED)
void MathStruct::createAssembly(const std::shared_ptr<DataPack>& d, const std::shared_ptr<FormulaParser>& p, const bool reset)
{
   if (reset) {
      resetAssembly();
   }

   if (assembly_created_ == AssemblyState::unknown) {
      if (!jit_runtime_) {
         jit_runtime_ = std::make_shared<asmjit::JitRuntime>();
      }

      asmjit::CodeHolder code;                       // Holds code and relocation information.
      code.init(jit_runtime_->environment());        // Initialize to the same arch as JIT runtime.
      asmjit::x86::Compiler cc(&code);                 // Create and attach x86::Compiler to `code`.
      cc.addFunc(asmjit::FuncSignatureT<float>());
      auto func_ptr = createSubAssembly(cc, d, p);   // Create actual code.
     
      if (func_ptr) {
         cc.ret(*func_ptr);
         cc.endFunc();                               // End of the function body.
         cc.finalize();                              // Translate and assemble the whole `cc` content.
         jit_runtime_->add(&fast_eval_func_, &code); // Add the generated code to the runtime.
         assembly_created_ = AssemblyState::yes;
      }
      else {
         assembly_created_ = AssemblyState::no;

         for (const auto& op : opnds_) {
            op->createAssembly(d, p);
         }
      }
   }
}
void vfm::MathStruct::resetAssembly()
{
   applyToMeAndMyChildren([](const std::shared_ptr<MathStruct> m) {
      m->jit_runtime_ = nullptr;
      m->assembly_created_ = AssemblyState::unknown;
   });
}
#endif

std::ostream& operator<<(std::ostream& os, MathStruct const& m) {
   os << m.serialize();
   return os;
}

std::string MathStruct::printComplete(const std::string& buffer, const std::shared_ptr<std::vector<std::shared_ptr<const MathStruct>>> visited, const bool print_root) const
{
   std::string s = "";
   auto visited_real = visited;

   if (!visited_real) {
      visited_real = std::make_shared<std::vector<std::shared_ptr<const MathStruct>>>();
   }

   if (std::find(visited_real->begin(), visited_real->end(), shared_from_this()) != visited_real->end()) {
      s += "\n" + buffer + INDENTATION_DEBUG + "<REC> " + serialize();
      return s;
   }

   visited_real->push_back(shared_from_this());

   if (print_root) {
      s += serialize();
   }

   for (const auto& opd : opnds_) {
      s += opd->printComplete(buffer + INDENTATION_DEBUG, visited_real, false);
   }

   return s;
}

std::vector<std::string> vfm::MathStruct::generatePostfixVector(const bool nestDownToCompoundStructures)
{
   std::vector<std::string> tokens;

   for (int i = opnds_.size() - 1; i >= 0; --i) {
      auto sub_vector = opnds_[i]->generatePostfixVector(nestDownToCompoundStructures);
      tokens.insert(tokens.begin(), sub_vector.begin(), sub_vector.end());
   }

   tokens.push_back(getOptor());

   return tokens;
}

MathStruct::MathStruct(const std::vector<std::shared_ptr<Term>>& opds) :
   full_correct_brackets_(false)
{
   setOpnds(opds);
}

bool MathStruct::hasSideEffectsAnyChild()
{
   for (const auto& child : opnds_) {
      if (child->hasSideeffects()) {
         return true;
      }
   }

   return false;
}

void printTraceAndCheckForInfiniteLoop(
   const std::shared_ptr<Term> dummy_term,
   const int i,
   //std::shared_ptr<DataPack> d2,
   const bool print_trace,
   const std::shared_ptr<std::vector<TermPtr>> old_terms)
{
   //if (print_trace) {
   //   std::shared_ptr<DataPack> d3;

   //   if (!d2) {
   //      d2 = std::make_shared<DataPack>();
   //   }

   //   if (dummy_term->hasSideeffects()) {
   //      d3 = std::make_shared<DataPack>();
   //      d3->initializeValuesBy(*d2);
   //   }

   //   std::string eq = i ? "=" : " ";

   //   Failable::getSingleton("Simplification")->addNote("(" + std::to_string(i) + ") " + eq + " " + dummy_term->getTerms()[0]->serialize(nullptr, false, false, false, false) 
   //                       + "\t\t= " + std::to_string(dummy_term->eval(d2)) + " (nodes: " + std::to_string(dummy_term->getTerms()[0]->getNodeCount()) + ")");

   //   if (d3) {
   //      d2->initializeValuesBy(d3);
   //   }
   //}


   if (old_terms) {
      dummy_term->checkIfAllChildrenHaveConsistentFatherQuiet();

      bool print_it = false;
      int num = 0;
      for (const auto& old_term : *old_terms) {
         if (old_term->isStructurallyEqual(dummy_term)) {
            Failable::getSingleton("Simplification")->addError("Infinite loop detected during simplification. Here are the formulas of the loop:");
            print_it = true;
         }

         if (print_it) {
            Failable::getSingleton("Simplification")->addErrorPlain("\n----- (" + std::to_string(num) + ") -----\n");
            Failable::getSingleton("Simplification")->addErrorPlain(old_term->serialize());
            num++;
         }
      }

      if (print_it) {
         //Failable::getSingleton("Simplification")->addFatalError("Cannot recover from infinite loop.");
      }

      old_terms->push_back(dummy_term->copy());
   }
}



TermPtr MathStruct::simplify(
   const TermPtr term,
   const bool print_trace,
   const std::shared_ptr<DataPack> d,
   const bool check_for_infinite_loop,
   const std::shared_ptr<FormulaParser> parser,
   const std::shared_ptr<std::map<std::string, std::map<int, std::set<MetaRule>>>> rules_raw)
{
   auto old_terms{ check_for_infinite_loop ? std::make_shared<std::vector<TermPtr>>() : nullptr };
   int i{ 0 };

   //static std::string last = "";
   //std::cout << " ---\n";
   auto p = parser ? parser : SingletonFormulaParser::getInstance();
   const std::map<std::string, std::map<int, std::set<MetaRule>>>& all_rules{ rules_raw ? *rules_raw : p->getAllRelevantSimplificationRulesOrMore() };

   auto formula = _id(term);

   for (const auto& rrr : MetaRule::getMetaRulesByStage(all_rules)) {
      if (print_trace) {
         int stage = rrr.first;
         Failable::getSingleton()->addNotePlain("Entering simplification stage #" + std::to_string(stage) + ".");
      }

      std::map<std::string, std::map<int, std::set<MetaRule>>> rules{ rrr.second };

      bool changed = true;
      while (changed) {
         printTraceAndCheckForInfiniteLoop(formula, i, print_trace, old_terms);
         changed = false;
         formula->getOperands()[0]->applyToMeAndMyChildrenIterative([&changed, rules_raw, p, &rules, print_trace](const MathStructPtr m) {
            //std::string ser = m->getPtrToRoot()->getTermsJumpIntoCompounds()[0]->serializePlainOldVFMStyleWithCompounds();
            //if (last != ser) {
            //   std::cout << "     FAST:   " << ser << std::endl;
            //   last = ser;
            //}
            if (MetaRule::is_leaf(m)) return; // Caution, remove if simplifications on leaf level desired.
            bool changed_this_time = false;

            if (!m->isCompoundOperator() && m->isOverallConstant()) {
               m->replace(_val_plain(m->constEval()));
               changed_this_time = true; // Since it's not a leaf, there was definitely a change.
            }
            else if (!m->isTermCompound()) {
               auto& vec{ rules.at(m->getOptor()).at(m->getOpStruct().arg_num) };

               for (const auto& r : vec) {
                  if (m->applyMetaRule(r, print_trace)) {
                     changed_this_time = true;
                     break;
                  }
               }

               if (!changed_this_time) {
                  for (const auto& r_by_symb : rules.at(SYMB_ANYWAY)) {
                     for (const auto& r : r_by_symb.second) {
                        if (m->applyMetaRule(r, print_trace)) {
                           changed_this_time = true;
                           break;
                        }
                     }
                  }
               }
            }

            changed = changed || changed_this_time;
            }, TraverseCompoundsType::go_into_compound_structures, FormulaTraversalType::PostOrder);

         formula->setChildrensFathers(FatherSetterStyle::go_into_compound_structures, true);
      }
   }

   return formula->getOperands()[0];
}

TermPtr MathStruct::simplify(const std::string& formula, const bool print_trace, const std::shared_ptr<DataPack> d, const bool check_for_infinite_loop, const std::shared_ptr<FormulaParser> parser, const std::shared_ptr<std::map<std::string, std::map<int, std::set<MetaRule>>>> rules)
{
   return simplify(parseMathStruct(formula, true, false, parser)->toTermIfApplicable(), print_trace, d, check_for_infinite_loop, parser, rules);
}

std::vector<std::pair<std::shared_ptr<MathStruct>, std::shared_ptr<MathStruct>>> vfm::MathStruct::findDuplicateSubTerms(
   const std::shared_ptr<MathStruct> other,
   const int min_nodes_to_report,
   const TraverseCompoundsType traverse_into_compound_structures)
{
   auto res = std::make_shared<std::vector<std::pair<std::shared_ptr<MathStruct>, std::shared_ptr<MathStruct>>>>();
   std::shared_ptr<bool> trigger_abandon_children_once1 = std::make_shared<bool>(false);
   std::shared_ptr<bool> trigger_abandon_children_once2 = std::make_shared<bool>(false);

   applyToMeAndMyChildren([res, trigger_abandon_children_once1, trigger_abandon_children_once2, other, min_nodes_to_report, traverse_into_compound_structures](const std::shared_ptr<MathStruct> m)
   {
      other->applyToMeAndMyChildren([res, trigger_abandon_children_once1, trigger_abandon_children_once2, min_nodes_to_report, m](const std::shared_ptr<MathStruct> o)
      {
         if (o != m && (m->getNodeCount() >= min_nodes_to_report || o->getNodeCount() >= min_nodes_to_report) && m->isStructurallyEqual(o)) {
            res->push_back({ m, o });
            *trigger_abandon_children_once1 = true;
            *trigger_abandon_children_once2 = true;
         }
      }, traverse_into_compound_structures, trigger_abandon_children_once2);
   }, traverse_into_compound_structures, trigger_abandon_children_once1);

   return *res;
}

bool vfm::MathStruct::hasBooleanResult() const
{
   return false;
}

bool vfm::MathStruct::isStructurallyEqual(const std::shared_ptr<MathStruct> other)
{
   if (!other) {
      return false;
   }

   const auto me{ toTermIfApplicable() };
   const std::string my_optor{ isTermVar() ? StaticHelper::plainVarNameWithoutLocalOrRecursionInfo(me->getOptor()) : me->getOptor() };
   const std::string others_optor{ isTermVar() ? StaticHelper::plainVarNameWithoutLocalOrRecursionInfo(other->getOptor()) : other->getOptor() };
   const auto my_terms{ me->getOperands() };
   const auto others_terms{ other->getOperands() };

   if (my_optor == others_optor && my_terms.size() == others_terms.size()) {
      for (int i = 0; i < my_terms.size(); i++) {
         auto mine{ my_terms[i] };
         auto others{ others_terms[i] };

         if (!mine->isStructurallyEqual(others)) {
            return false;
         }
      }

      return true;
   }

   return false;
}

std::shared_ptr<MathStruct> vfm::MathStruct::getFatherRaw() const
{
   return father_.lock();
}

std::shared_ptr<MathStruct> vfm::MathStruct::getFather() const
{
   if (is_valid_) { 
      return getFatherRaw();
   }

   return nullptr;
}

std::shared_ptr<MathStruct> vfm::MathStruct::getFatherJumpOverCompound() const
{
   auto father{ getFather() };                                                     // When jumping over compounds, a compound structure is a root term (and has compound references).
   auto grandad{ father && isCompoundOperator() ? father->getFather() : nullptr }; // Compound operator always has compound as father - grandad is father of this compound.

   return isCompoundOperator() 
      ? grandad
      : father;
}

std::shared_ptr<MathStruct> vfm::MathStruct::thisPtrGoIntoCompound()
{
   return isTermCompound() ? opnds_[0] : shared_from_this();
}

std::shared_ptr<MathStruct> vfm::MathStruct::thisPtrGoUpToCompound()
{
   return isCompoundOperator() ? getFather() : shared_from_this();
}

void vfm::MathStruct::setFather(std::shared_ptr<MathStruct> new_father)
{
   father_ = new_father;
}

bool vfm::MathStruct::isValid() const
{
   return is_valid_;
}

void vfm::MathStruct::makeInvalid(const bool recursive)
{
   is_valid_ = false;

   if (recursive) {
      for (const auto& opnd : opnds_) {
         opnd->makeInvalid(true);
      }
   }
}

void vfm::MathStruct::makeValid(const bool recursive)
{
   is_valid_ = true;

   if (recursive) {
      for (const auto& opnd : opnds_) {
         opnd->makeValid(true);
      }
   }
}

bool vfm::MathStruct::isAssemblyCreated() const
{
   return assembly_created_ == AssemblyState::yes;
}

// Autogenerated function for meta rule:
// (a ? b) : c ==> ifelse(a, b, c)
inline bool ternaryRule1(const TermPtr formula, const std::shared_ptr<FormulaParser> parser)
{
   if (formula->getOptorOnCompoundLevel() != ":") {
      return false;
   }
   TermPtr formula_0 = formula->getTermsJumpIntoCompounds()[0];
   TermPtr formula_1 = formula->getTermsJumpIntoCompounds()[1];
   if (formula_0->getOptorOnCompoundLevel() != "?") {
      return false;
   }
   TermPtr formula_0_0 = formula_0->getTermsJumpIntoCompounds()[0];
   TermPtr formula_0_1 = formula_0->getTermsJumpIntoCompounds()[1];

   formula->replace(_ifelse(formula_0_0, formula_0_1, formula_1));
   return true;
}

// Autogenerated function for meta rule:
// a = ifelse(b, c, d) ==> ifelse(b, a = c, a = d) [[a: 'has_no_sideeffects']]
inline bool ternaryRule2(const TermPtr formula, const std::shared_ptr<FormulaParser> parser)
{
   if (formula->getOptorOnCompoundLevel() != "=") {
      return false;
   }
   TermPtr formula_0 = formula->getTermsJumpIntoCompounds()[0];
   TermPtr formula_1 = formula->getTermsJumpIntoCompounds()[1];
   if (formula_1->getOptorOnCompoundLevel() != "ifelse") {
      return false;
   }
   TermPtr formula_1_0 = formula_1->getTermsJumpIntoCompounds()[0];
   TermPtr formula_1_1 = formula_1->getTermsJumpIntoCompounds()[1];
   TermPtr formula_1_2 = formula_1->getTermsJumpIntoCompounds()[2];
   if (MetaRule::has_no_sideeffects(formula_0)) {
      formula->replace(_ifelse(formula_1_0, _set_alt(formula_0, formula_1_1), _set_alt(formula_0->copy(), formula_1_2)));
      return true;
   }
   return false;
}

std::shared_ptr<Term> vfm::MathStruct::resolveTernaryOperators(const std::shared_ptr<Term> formula, const bool check_for_consistency, const std::shared_ptr<FormulaParser> parser)
{
   auto formula_simp{ simplification::applyToFullFormula(formula, { ternaryRule1, ternaryRule2 }, parser) };
   
   if (check_for_consistency) {
      bool found{ false };

      formula_simp->applyToMeAndMyChildrenIterative([&found](const MathStructPtr m) {
         found = m->getOptor() == SYMB_COND_QUES || m->getOptor() == SYMB_COND_COL;

         if (found) {
            Failable::getSingleton()->addError(
               "Unconnected part '" + m->getOptor() + "' of ternary operator '?:' found in '" 
               + m->serializeWithinSurroundingFormula(100, 100) 
               + "'. Did you forget to use brackets? Nested ternary operators without brackets, as in a ? b : c ? d : e, are forbidden in vfm.");
         }
      }, TraverseCompoundsType::avoid_compound_structures, found);
   }

   return formula_simp;
}

void MathStruct::replaceOperand(const std::shared_ptr<Term> old, const std::shared_ptr<Term> by, const bool make_old_invalid)
{
   for (auto& term : opnds_) {
      if (term == old) {
         if (make_old_invalid) {
            old->makeInvalid();
         }

         if (old->isCompoundOperator()) {
            term = std::make_shared<TermCompoundOperator>(by->getOperands(), by->getOpStruct());
         }
         else {
            term = by;
         }
         
         term->father_ = shared_from_this();
         return;
      }
   }
}

void vfm::MathStruct::replaceOperand(const int op_num, const std::shared_ptr<Term> by, const bool make_old_invalid)
{
   if (make_old_invalid) {
      opnds_[op_num]->makeInvalid();
   }
   by->father_ = shared_from_this();
   opnds_[op_num] = by;
}

void vfm::MathStruct::replace(const std::shared_ptr<Term> by, const bool make_old_invalid)
{
   assert(!isRootTerm());
   getFather()->replaceOperand(toTermIfApplicable(), by, make_old_invalid);
}


void MathStruct::replaceOperandLookIntoCompounds(const std::shared_ptr<Term> old, const std::shared_ptr<Term> by, const bool make_old_invalid)
{
   for (auto& term_raw : opnds_) { // Situation: a + b     (we're the '+')
      auto& term{ term_raw->isTermCompound() ? term_raw->opnds_[0] : term_raw }; // Situation: a + compound(b, ...)  ==> look directly at 'b'

      if (term == old || term_raw == old) { // 'by' might be a compound already, then we need to replace it, not its CompoundOperator
         if (make_old_invalid) {
            old->makeInvalid();
         }
         term_raw = by; // We need to replace the actual term, not the possible CompoundOperator (which would invalidate the TermCompound)
         term_raw->father_ = shared_from_this();
         return;
      }
   }

   Failable::getSingleton()->addError("Term '" + old->serializePlainOldVFMStyle() + "' not replaced by '" + by->serializePlainOldVFMStyle() + "' in '" + serializePlainOldVFMStyle() + "'.");
}

void vfm::MathStruct::replaceJumpOverCompounds(const std::shared_ptr<Term> by, const bool make_old_invalid)
{
   auto father{ getFatherJumpOverCompound() }; 

   if (!father) {
      Failable::getSingleton()->addError("Term '" + serializePlainOldVFMStyle() + "' not replaced by '" + by->serializePlainOldVFMStyle() + "' in '" + getPtrToRoot()->serializePlainOldVFMStyle() + "'.");
   }

   father->replaceOperand(toTermIfApplicable(), by, make_old_invalid);
}

void vfm::MathStruct::swapSubtrees(const TermPtr sub_formula1, const TermPtr sub_formula2)
{
   auto father1 = sub_formula1->getFather();
   auto father2 = sub_formula2->getFather();
   bool success1 = false;
   bool success2 = false;

   assert(father1 && father2);

   for (auto& op : father1->opnds_) {
      if (op == sub_formula1) {
         op = sub_formula2;
         op->father_ = father1;
         success1 = true;
      }
   }

   for (auto& op : father2->opnds_) {
      if (op == sub_formula2) {
         op = sub_formula1;
         op->father_ = father2;
         success2 = true;
      }
   }

   assert(success1 && success2);
}

void vfm::MathStruct::removeLineFromSequence()
{
   auto father_raw{ getFatherJumpOverCompound() };
   auto father{ father_raw ? father_raw->getFather() : nullptr };

   if (!(father_raw || father && (father->isTermSequence() || father->isTermIf() || father->isTermIfelse() || father->isTermIdentity()))) {
      Failable::getSingleton()->addWarning("Couldn't delete line '" + serializeWithinSurroundingFormula(50, 50) + "'.");
      return;
   }

   auto grandad{ father ? father->getFatherJumpOverCompound() : nullptr };

   if (grandad && father->isTermIfelse()) {
      if (shared_from_this() == father->getTermsJumpIntoCompounds()[1]) { // We're the then case [1].
         if (!father->getTermsJumpIntoCompounds()[2]->hasSideeffects()) { // Else case is empty, as well.
            father->removeLineFromSequence();
         }
         else {
            father->replaceJumpOverCompounds(_if(_not(father->getTermsJumpIntoCompounds()[0]), father->getTermsJumpIntoCompounds()[2]));
         }
      }
      else { // We're the else case [2].
         if (!father->getTermsJumpIntoCompounds()[1]->hasSideeffects()) { // Then case is empty, as well.
            father->removeLineFromSequence();
         }
         else {
            father->replaceJumpOverCompounds(_if(father->getTermsJumpIntoCompounds()[0], father->getTermsJumpIntoCompounds()[1]));
         }
      }
   }
   else if (grandad && father->isTermIf()) {
      father->removeLineFromSequence();
   }
   else if (grandad && !father->isTermIdentity()) { // At this point father's a TermSequence (or something we're not covering, like a while/for/etc.).
      int op_num{ shared_from_this() == father->getTermsJumpIntoCompounds()[0] }; // op_num is the index of NOT us.
      father->replaceJumpOverCompounds(father->getTermsJumpIntoCompounds()[op_num]);
   }
   else {
      replaceJumpOverCompounds(_val(0)); // Cannot remove line, so replace it with 0.
   }
}

bool MathStruct::applicableMetaRule(
   std::shared_ptr<vfm::Term>& from_raw,
   std::shared_ptr<vfm::Term>& to_raw,
   std::vector<std::pair<std::shared_ptr<Term>, ConditionType>>& conditions,
   const std::shared_ptr<vfm::Term>& from_father,
   const std::shared_ptr<vfm::Term>& to_father,
   const std::shared_ptr<std::map<std::shared_ptr<Term>, std::shared_ptr<Term>>>& condition_fathers)
{
   auto ptr_to_this{ thisPtrGoIntoCompound() };
   auto ptr_to_father{ getFatherJumpOverCompound() };
   auto from_dummy{ from_father };
   auto to_dummy{ to_father };
   auto condition_dummy{ condition_fathers };
   bool root{ false };
   auto actual_from{ from_raw->thisPtrGoIntoCompound()->toTermIfApplicable() };
   auto actual_to{ to_raw->thisPtrGoIntoCompound()->toTermIfApplicable() };

   if (!from_dummy || !to_dummy || !condition_dummy) { // Root call ==> initialize dummies.
      from_dummy = _id(from_raw->toTermIfApplicable());
      to_dummy = _id(to_raw->toTermIfApplicable());
      condition_dummy = std::make_shared<std::map<std::shared_ptr<Term>, std::shared_ptr<Term>>>();

      for (const auto& pair : conditions) {
         std::shared_ptr<Term> dum{ _id_plain(pair.first) };
         dum->setChildrensFathers(false, false);
         condition_dummy->insert({ pair.first, dum });
      }

       root = true;
   }

   int size{ (int) ptr_to_this->getOperands().size() };
   int from_size{ (int) actual_from->getOperands().size() };
   bool matches{ actual_from->getOptor() == ptr_to_this->getOptor() && from_size == size || actual_from->toTermAnywayIfApplicable() && from_size - 1 == size }; // First operand of TermAnyway is identificator. TODO: check from_size == size only required for overloaded operators.

   setChildrensFathers(FatherSetterStyle::avoid_compound_structures, true); // TODO: REMOVE

   if (matches && !ptr_to_this->isMetaSimplification()) { // Current operator matches ==> go one level deeper.
      const bool b1{ ptr_to_this->isOverallConstant() };
      const bool b2{ from_raw->isOverallConstant() };
      if (b1 != b2 || b1 && constEval() != from_raw->constEval()) {
         return false;
      }

      if (actual_from->toTermAnywayIfApplicable() && from_size - 1 == getOperands().size()) { // Handle TermAnyways
         auto from_copy{ actual_from->copy() };
         auto old_from{ actual_from };
         auto old_to{ actual_to };
         bool changed{};

         auto func{ [this, &from_copy, &old_from, &old_to, &actual_from, &actual_to, &changed](const MathStructPtr m) {
            auto m_term_anyway{ m->toTermAnywayIfApplicable() };

            if (m_term_anyway && m_term_anyway->getTermsJumpIntoCompounds()[0]->constEval() == from_copy->getTermsJumpIntoCompounds()[0]->constEval()) {
               auto new_term{ this->thisPtrGoUpToCompound()->copy()->toTermIfApplicable() }; // TODO: Very slow operation!

               for (int i = 0; i < getTermsJumpIntoCompounds().size(); i++) {
                  new_term->getTermsJumpIntoCompounds()[i] = m->getOperands()[i + 1];
               }

               m_term_anyway->replaceJumpOverCompounds(new_term, false);

               if (m == actual_from) {
                  actual_from = new_term;
               }
               if (m == actual_to) {
                  actual_to = new_term;
               }

               changed = true;
            }
         } };

         changed = true;
         while (changed) {
            changed = false;
            actual_from->getPtrToRoot()->applyToMeAndMyChildrenIterative(func, TraverseCompoundsType::avoid_compound_structures);
            actual_from->setChildrensFathers(true, false);
         }

         changed = true;
         while (changed) {
            changed = false;
            actual_to->getPtrToRoot()->applyToMeAndMyChildrenIterative(func, TraverseCompoundsType::avoid_compound_structures);
            actual_to->setChildrensFathers(true, false);
         }

         for (const auto pair : conditions) {
            pair.first->getPtrToRoot()->applyToMeAndMyChildrenIterative(func, TraverseCompoundsType::avoid_compound_structures);
         }
      }

      for (int i = 0; i < size; i++) {
         if (!getTermsJumpIntoCompounds()[i]->applicableMetaRule(actual_from->getTermsJumpIntoCompounds()[i], actual_to, conditions, actual_from, to_dummy, condition_dummy)) {
            return false;
         }
      }
   }
   else if (matches && ptr_to_this->isMetaSimplification()) { // Don't match metas.
      return false;
   } 
   else if (actual_from->isTermOptional()) { // Go to actual meta if ended up at optional.
      return this->applicableMetaRule(actual_from->getTermsJumpIntoCompounds()[0], actual_to, conditions, actual_from, to_dummy, condition_dummy);
   }
   else {
      auto optional{ actual_from->getOpStruct().arg_num == 2 && actual_from->opnds_[0]->isTermOptional() ? actual_from->opnds_[0] : nullptr };
      auto other = optional ? actual_from->getOperands()[1] : nullptr;

      if (!optional && actual_from->getOpStruct().arg_num == 2 && actual_from->getOperands()[1]->isTermOptional()) {
         optional = actual_from->getOperands()[1];
         other = actual_from->getOperands()[0];
      }

      assert(!other || !other->isTermOptional());

      if (actual_from->isMetaSimplification() // Match with free meta ("(. * p_(0)) <===> (. * (3 + 4 * x))")...
          || optional) { // ... or handle the case that there's an optional child: o_(., .) + . <===> .
         assert(actual_from->getFatherRaw());
         float meta_num{};
         std::shared_ptr<MathStruct> actual_ptr{};

         if (actual_from->isMetaSimplification()) {
            meta_num = actual_from->opnds_[0]->constEval();
            actual_ptr = shared_from_this(); // Not ptr_to_this! We could be at a TermCompound and cannot return a dangling CompoundOperator.
         }
         else if (optional) {
            meta_num = optional->opnds_[0]->opnds_[0]->constEval();
            actual_ptr = optional->opnds_[1];

            if (!applicableMetaRule(other, actual_to, conditions, actual_from, to_dummy, condition_dummy)) {
               return false;
            }
         }

         auto blacklist{ std::make_shared<std::vector<std::shared_ptr<MathStruct>>>() };

         auto func = [meta_num, actual_ptr, blacklist] (std::shared_ptr<MathStruct> m) {
            for (auto& mc : m->opnds_) {
               if (mc->isMetaSimplification() && mc->getTermsJumpIntoCompounds()[0]->constEval() == meta_num) {
                  mc = actual_ptr->/*copy()->*/toTermIfApplicable();
                  //m->setChildrensFathers(false, false);
                  blacklist->push_back(mc);
               }
            }
         };

         actual_from->getPtrToRoot()->applyToMeAndMyChildren(func, TraverseCompoundsType::avoid_compound_structures, nullptr, nullptr, blacklist);
         blacklist->clear();
         to_dummy->applyToMeAndMyChildren(func, TraverseCompoundsType::avoid_compound_structures, nullptr, nullptr, blacklist);
         
         for (const auto pair : conditions) {
            blacklist->clear();
            condition_dummy->at(pair.first)->applyToMeAndMyChildren(func, TraverseCompoundsType::avoid_compound_structures, nullptr, nullptr, blacklist);
         }
      } else {
         return false;
      }
   }

   bool condition_ok{ false };
   if (root) {
      condition_ok = true;
      to_raw = to_dummy->getTermsJumpIntoCompounds()[0];
      from_raw = from_dummy->getTermsJumpIntoCompounds()[0];

      for (int i = 0; i < conditions.size() && condition_ok; i++) {
         auto pair = conditions.at(i);
         auto cond = condition_dummy->at(conditions.at(i).first)->getTermsJumpIntoCompounds()[0];
         auto mode = pair.second;
         bool ok{ MetaRule::checkCondition(mode, cond) };
         condition_ok = condition_ok && ok;
      }
   }

   if (actual_to->getOptor() != ptr_to_this->getOptor() && !ptr_to_father) { // We cannot replace operator if it's the root.
      return false;
   }

   if (root && condition_ok) { // Needed to get adapted rule in caller function.
      for (auto& cc : conditions) {
         cc.first = condition_dummy->at(cc.first)->getOperands()[0];
      }
   }

   return !root || condition_ok;
}

bool MathStruct::applyMetaRule(const MetaRule& rule, const bool print_application_note)
{
   //std::cout << "RULE: " << rule.serialize() << std::endl;
   //std::cout << "THIS: " << serializePlainOldVFMStyle() << std::endl;
   //if (rule.serialize() == "(o_(a, 0) + o_(b, 1) * c) - o_(d, 1) * c ==> a + c * (b +  --(d)) {{'has_no_sideeffects'}}"
   //   && this->serializePlainOldVFMStyle() == "ifelse(x, y, z) * abs(a) - abs(a)") {
   //   std::cout << std::endl;
   //   //return false;
   //}

   auto father_ptr{ getFatherJumpOverCompound() };
   auto this_ptr{ thisPtrGoIntoCompound()->toTermIfApplicable()};

   if (!father_ptr || !this_ptr) {
      return false; // No rule can be applied to top-level of tree or non-term math_structs.
   }

   if (!MetaRule::checkCondition(rule.getGlobalCondition(), this_ptr)) {
      return false;
   }

   MetaRule cp{ rule.copy(true, true, true) };

   auto from{ cp.getFrom() };
   auto to{ cp.getTo() };
   auto condition{ cp.getConditions() };
   bool res{ applicableMetaRule(from, to, condition, nullptr, nullptr, nullptr) };

   //from->checkCompoundConsistency();
   //from->checkIfAllChildrenHaveConsistentFather();

   if (res) {
      auto shorten{ [](const std::string& str) -> std::string { return StaticHelper::shortenInTheMiddle(str, 500, 0.5, SPECIAL_COLOR_2 + "[...]" + RESET_COLOR);  } };

      if (print_application_note) {
         //for (auto& cond : condition) { // Avoid printing the "id" around all terms in the conditions.
         //   cond.first = cond.first->getTermsJumpIntoCompounds()[0];
         //}

         auto adapted_rule{ MetaRule(from, to, cp.getStage(), condition, cp.getGlobalCondition())};
         
         Failable::getSingleton()->addNotePlain("Simplifiable   : " + shorten(serializeWithinSurroundingFormula()));
         Failable::getSingleton()->addNotePlain("Applicable rule: " + shorten(rule.serialize()));
         Failable::getSingleton()->addNotePlain("Adapted rule   : " + shorten(adapted_rule.serialize()));
      }

      if (!from->isStructurallyEqual(to)) { // Otherwise we might have run in a case where the actual "thing" is done in the condition.
         father_ptr->replaceOperandLookIntoCompounds(this_ptr, to); // TODO: Only in situations like x ==> x ~ x, the second x needs to be copied.
      }

      father_ptr->setChildrensFathers(true, true); // TODO: Can be false, false??

      if (print_application_note) {
         Failable::getSingleton()->addNotePlain("Simplified to  : " + shorten(to->serializeWithinSurroundingFormula()));
         Failable::getSingleton()->addNotePlain("");
      }

      if (additional_test_info_) {
         MathStruct::additional_test_info_->push_back(rule.serialize());
      }

      return true;
   } else {
      return false;
   }
}

bool vfm::MathStruct::checkCompoundConsistency()
{
   bool consistent{ true };

   applyToMeAndMyChildren([&consistent, this](const MathStructPtr m) {
      if (m->isTermCompound() &&
         (!m->opnds_[0] || !m->opnds_[1]
            || !m->opnds_[0]->isCompoundOperator() || m->opnds_[1]->isCompoundStructure()
            || m->opnds_[0]->getFather() != shared_from_this() || m->opnds_[1]->getFather() != shared_from_this())) {
         Failable::getSingleton("CompoundConsistency")->addError("Formula '" + serializePlainOldVFMStyle() + "' inconsistent at TermCompound '" + m->serializePlainOldVFMStyle() + "'. Compound structure and/or operator mismatch.");
         consistent = false;
      }
      else if (m->isCompoundStructure() && (!m->getFather() || !m->getFather()->isTermCompound())) {
         Failable::getSingleton("CompoundConsistency")->addError("Formula '" + serializePlainOldVFMStyle() + "' inconsistent at CompoundStructure '" + m->serializePlainOldVFMStyle() + "'. Father is not TermCompound.");
         consistent = false;
      }
      else if (m->isCompoundOperator() && (!m->getFather() || !m->getFather()->isTermCompound())) {
         Failable::getSingleton("CompoundConsistency")->addError("Formula '" + serializePlainOldVFMStyle() + "' inconsistent at CompoundOperator '" + m->serializePlainOldVFMStyle() + "'. Father is not TermCompound.");
         consistent = false;
      }
   });

   if (consistent) {
      Failable::getSingleton("CompoundConsistency")->addNote("Compounds are all consistent in '" + serializePlainOldVFMStyle() + "'.");
   }

   return consistent;
}

std::set<MetaRule> vfm::MathStruct::getSimplificationRules(const std::shared_ptr<FormulaParser> parser) const
{
   std::set<MetaRule> vec{};
   auto this_ptr{ const_cast<MathStruct*>(this)->thisPtrGoIntoCompound() };
   std::map<std::string, std::string> real_op_name = { { "~", this_ptr->getOptor() } };

   // Simplify neutral operations.
   for (const auto& ops : NEUTRAL_ELEMENTS) {
      auto gop = ops.second.first.second;       // Goal operator.
      auto wop = ops.second.first.first;
      auto wop_str = ops.second.second.first;
      auto zero_const = ops.second.second.second;
      auto n_type = ops.first;

      if (wop_str == this_ptr->getOptor() && this_ptr->getOpStruct().arg_num == 2) {
         if (n_type == ArithType::left || n_type == ArithType::both) {
            vec.insert(MetaRule(wop(_val(zero_const), _vara()), gop(_vara()), SIMPLIFICATION_STAGE_MAIN, {}, ConditionType::no_check, parser)); // "0 + a" => "a"
         }

         if (n_type == ArithType::right || n_type == ArithType::both) {
            vec.insert(MetaRule(wop(_vara(), _val(zero_const)), gop(_vara()), SIMPLIFICATION_STAGE_MAIN, {}, ConditionType::no_check, parser)); // "a + 0" => "a"
         }
      }
   }

   // Simplify neutralizing operations.
   for (const auto& ops : NEUTRALIZING_ELEMENTS) {
      auto wop = ops.second.first;
      auto wop_str = ops.second.second.first;
      auto zero_const = ops.second.second.second.first;
      auto goal_const = ops.second.second.second.second;
      auto n_type = ops.first;

      if (wop_str == this_ptr->getOptor() && this_ptr->getOpStruct().arg_num == 2) {
         if (n_type == ArithType::left || n_type == ArithType::both) {
            vec.insert(MetaRule(wop(_val(zero_const), _vara()), _val(goal_const), SIMPLIFICATION_STAGE_MAIN, {{_vara(), ConditionType::has_no_sideeffects}}, ConditionType::no_check, parser)); // "0 * a" => "0"
         }

         if (n_type == ArithType::right || n_type == ArithType::both) {
            vec.insert(MetaRule(wop(_vara(), _val(zero_const)), _val(goal_const), SIMPLIFICATION_STAGE_MAIN, {{_vara(), ConditionType::has_no_sideeffects}}, ConditionType::no_check, parser)); // "a * 0" => "0"
         }
      }
   }

   // Simplify distributive terms.
   for (const auto& ops : DISTRIBUTIVE_TERMS) {
      auto strop = ops.second.first.first.first; // Strong operator.
      auto wop = ops.second.first.first.second;  // Weak operator.
      auto strop_str = ops.second.first.second.first;
      auto wop_str = ops.second.first.second.second;
      auto one_const = ops.second.second;
      auto d_type = ops.first;

      if (strop_str == this_ptr->getOptor() && this_ptr->getOpStruct().arg_num == 2) {
         // TODO: These simplifications are basically ok, but they don't work together with the countable operators' simplifications. ==> Check and fix!
         //vec.push_back(MetaRule(strop(_vara(), wop(_varb(), _varc())), wop(strop(_vara(), _varb()), strop(_vara(), _varc())), { {_vara(), ConditionType::is_constant}, {_varb(), ConditionType::is_constant} }, ConditionType::has_no_sideeffects)); // "a * (b + c)" => "a * b + a * c"
         //vec.push_back(MetaRule(strop(_vara(), wop(_varb(), _varc())), wop(strop(_vara(), _varb()), strop(_vara(), _varc())), { {_vara(), ConditionType::is_constant}, {_varc(), ConditionType::is_constant} }, ConditionType::has_no_sideeffects)); // "a * (b + c)" => "a * b + a * c"
         //vec.push_back(MetaRule(strop(wop(_varb(), _varc()), _vara()), wop(strop(_varb(), _vara()), strop(_varc(), _vara())), { {_vara(), ConditionType::is_constant}, {_varb(), ConditionType::is_constant} }, ConditionType::has_no_sideeffects)); // "(b + c) * a" => "b * a + c * a"
         //vec.push_back(MetaRule(strop(wop(_varb(), _varc()), _vara()), wop(strop(_varb(), _vara()), strop(_varc(), _vara())), { {_vara(), ConditionType::is_constant}, {_varc(), ConditionType::is_constant} }, ConditionType::has_no_sideeffects)); // "(b + c) * a" => "b * a + c * a"
      }

      if (wop_str == this_ptr->getOptor() && this_ptr->getOpStruct().arg_num == 2) {
         if (d_type == ArithType::right || d_type == ArithType::both) {
            // TODO: Something's wrong with these rules, check and fix!
            //vec.push_back(MetaRule(wop(strop(_varc(), _vara()), strop(_varc(), _varb())), strop(_varc(), wop(_vara(), _varb())), {}, ConditionType::has_no_sideeffects)); // "c * a + c * b" => "c * (a + b)"
            //vec.push_back(MetaRule(wop(_varc(), strop(_varc(), _varb())), strop(wop(_val(one_const), _varb()), _varc()), { {_varc(), ConditionType::is_not_constant} }, ConditionType::has_no_sideeffects)); // "c + c * b" => "(1 + b) * c"
         } 
         
         if (d_type == ArithType::left || d_type == ArithType::both) {
            vec.insert(MetaRule(wop(strop(_vara(), _varc()), strop(_varb(), _varc())), strop(wop(_vara(), _varb()), _varc()), SIMPLIFICATION_STAGE_MAIN, {}, ConditionType::has_no_sideeffects, parser)); // "a * c + b * c" => "(a + b) * c"
            vec.insert(MetaRule(wop(_varc(), strop(_varb(), _varc())), strop(wop(_val(one_const), _varb()), _varc()), SIMPLIFICATION_STAGE_MAIN, { {_varc(), ConditionType::is_not_constant} }, ConditionType::has_no_sideeffects, parser)); // "c + b * c" => "(1 + b) * c"
         } 
         
         if (d_type == ArithType::both) {
            vec.insert(MetaRule(wop(strop(_varc(), _vara()), strop(_varb(), _varc())), strop(wop(_vara(), _varb()), _varc()), SIMPLIFICATION_STAGE_MAIN, {}, ConditionType::has_no_sideeffects, parser)); // "c * a + b * c" => "(a + b) * c"
            vec.insert(MetaRule(wop(strop(_vara(), _varc()), strop(_varc(), _varb())), strop(wop(_vara(), _varb()), _varc()), SIMPLIFICATION_STAGE_MAIN, {}, ConditionType::has_no_sideeffects, parser)); // "a * c + c * b" => "(a + b) * c"
            vec.insert(MetaRule(wop(strop(_vara(), _varc()), _varc()), strop(wop(_vara(), _val(one_const)), _varc()), SIMPLIFICATION_STAGE_MAIN, { {_varc(), ConditionType::is_not_constant} }, ConditionType::has_no_sideeffects, parser)); // "a * c + c" => "(a + 1) * c"
            vec.insert(MetaRule(wop(strop(_varc(), _vara()), _varc()), strop(wop(_vara(), _val(one_const)), _varc()), SIMPLIFICATION_STAGE_MAIN, { {_varc(), ConditionType::is_not_constant} }, ConditionType::has_no_sideeffects, parser)); // "c * a + c" => "(a + 1) * c"
         }
      }
   }

   auto insert = [&vec](const MetaRule& rule) {
      rule.setStage(SIMPLIFICATION_STAGE_PREPARATION); // Stage 0 since we generate a normal form here which we need for simplification.
      vec.insert(rule);

      MetaRule r1 = rule.copy(true, true, true);
      r1.setStage(SIMPLIFICATION_STAGE_MAIN); // Stage 10 since we need a normal form in the end, as well.
      vec.insert(r1);
   };

   // Order operands of commutative operators alphabetically.
   // TODO: Replace below with 
   //    o_(a, 0) ~ b ~ c ==> b ~ a ~ c [[a: 'is_variable', b: 'is_variable', a;b: 'of_first_two_operands_second_is_alphabetically_above', b~c: 'is_commutative_operation']]
   // once optional operator works within anyway operator for hard-coded simplification. Also, we need a mechanism to insert neutral operation as default of optional, as in:
   // 0 + x vs. 1 * y.
   //
   // TODO: Maybe we do not need the below rules in both stages. Possibly only in stage 10?
   if (this_ptr->getOpStruct().arg_num == 2 && this_ptr->getOpStruct().commut) {
      insert(MetaRule(
         this_ptr->getOptor() + "(a, b)",
         this_ptr->getOptor() + "(b, a)",
         -1,
         { {"a; b", ConditionType::of_first_two_operands_second_is_alphabetically_above },
           {"a", ConditionType::is_variable},
           {"b", ConditionType::is_variable},
         }));

      if (this_ptr->getOpStruct().assoc && this_ptr->getOpStruct().associativity == AssociativityTypeEnum::left) {
         insert(MetaRule(
            this_ptr->getOptor() + "(" + this_ptr->getOptor() + "(c, a), b)",
            this_ptr->getOptor() + "(" + this_ptr->getOptor() + "(c, b), a)",
            -1,
            { {"a; b", ConditionType::of_first_two_operands_second_is_alphabetically_above},
              {"a", ConditionType::is_variable},
              {"b", ConditionType::is_variable},
            }));
      }
      else if (this_ptr->getOpStruct().assoc && this_ptr->getOpStruct().associativity == AssociativityTypeEnum::right) {
         insert(MetaRule(
            this_ptr->getOptor() + "(a, " + this_ptr->getOptor() + "(b, c))",
            this_ptr->getOptor() + "(b, " + this_ptr->getOptor() + "(a, c))",
            -1,
            { {"a; b", ConditionType::of_first_two_operands_second_is_alphabetically_above},
            {"a", ConditionType::is_variable},
            {"b", ConditionType::is_variable},
            }));
      }
   }

   // Simplify countable operators.
   std::set<MetaRule> vec_countable;
   for (const auto& ops : COUNTABLE_OPERATORS) {
      auto wop = ops.weak_operator_.function_;                 // Weak operator1 "- +", "/ *".
      auto wop_plus = ops.weak_operator_plus_style_.function_; // Weak operator2, can be the same, but it has to be a "plus style" operator "+", "*".
      auto strop = ops.strong_operator_.function_;             // Strong operator "*".
      auto wop_str = ops.weak_operator_.name_;
      auto wop_plus_str = ops.weak_operator_plus_style_.name_;
      auto gop = ops.correction_operator_.function_;
      auto num1 = ops.neutral_operator_weak_op_;
      auto num2 = ops.neutral_operator_strong_op_;
      auto powcomp = ops.power_compensator_.function_;

      if (wop_str == this_ptr->getOptor() && this_ptr->getOpStruct().arg_num == 2) {
         vec_countable.insert(MetaRule(wop(wop(_vara(), strop(_vard(), _optional("b", num2))), strop(_vard(), _optional("c", num2))),
                                          wop_plus(_vara(), strop(_vard(), _plus(gop(_varb()), gop(_varc())))),
                                          SIMPLIFICATION_STAGE_MAIN,
                                          {},      // "(a - d * b) - d * c" ==> "a + d * (--b + --c)"
                                          ConditionType::has_no_sideeffects, parser));

         vec_countable.insert(MetaRule(wop(wop_plus(_optional("a", num1), strop(_vard(), _optional("b", num2))), strop(_vard(), _optional("c", num2))),
                                          wop_plus(_vara(), strop(_vard(), _plus(_varb(), gop(_varc())))),
                                          SIMPLIFICATION_STAGE_MAIN,
                                          {},      // "(a + d * b) - d * c" ==> "a + d * (b + --c)"
                                          ConditionType::has_no_sideeffects, parser));

         // "(a - d * b) - e * c" ==> "a + d * (--b + --(e / d * c))" [ e != d && e / d is integer. ]
         vec_countable.insert(MetaRule(wop(wop(_vara(), strop(_vard(), _optional("b", num2))), strop(_vare(), _optional("c", num2))),
            wop_plus(_vara(), strop(_vard(), _plus(gop(_varb()), gop(_mult(_div(powcomp(_vare()), powcomp(_vard())), _varc()))))),
            SIMPLIFICATION_STAGE_MAIN,
            {
               { _vard(), ConditionType::is_constant_and_evaluates_to_true },                                 // denominator cannot be zero
               { _eq(_vard(), _vare()), ConditionType::is_constant_and_evaluates_to_false },                  // d and e cannot be equal
               { _div(powcomp(_vare()), powcomp(_vard())), ConditionType::is_constant_integer },              // e is divisible by d
               { _eq(_div(_vare(), _vard()), _vare()), ConditionType::is_constant_and_evaluates_to_false },   // but d is not 1
            },
            ConditionType::has_no_sideeffects, parser));

         // "(a - d * b) - e * c" ==> "a + e * (--(d / e * b) + --c)" [ e != d && d / e is integer. ]
         vec_countable.insert(MetaRule(wop(wop(_vara(), strop(_vard(), _optional("b", num2))), strop(_vare(), _optional("c", num2))),
            wop_plus(_vara(), strop(_vare(), _plus(gop(_mult(_div(powcomp(_vard()), powcomp(_vare())), _varb())), gop(_varc())))),
            SIMPLIFICATION_STAGE_MAIN,
            {
               { _vare(), ConditionType::is_constant_and_evaluates_to_true },                                 // denominator cannot be zero
               { _eq(_vard(), _vare()), ConditionType::is_constant_and_evaluates_to_false },                  // d and e cannot be equal
               { _div(powcomp(_vard()), powcomp(_vare())), ConditionType::is_constant_integer },              // d is divisible by e
               { _eq(_div(_vard(), _vare()), _vard()), ConditionType::is_constant_and_evaluates_to_false },   // but e is not 1
            },
            ConditionType::has_no_sideeffects, parser));

         // "(a + d * b) - e * c" ==> "a + d * (b + --(e / d * c))" [ e != d && e / d is integer. ]
         vec_countable.insert(MetaRule(wop(wop_plus(_optional("a", num1), strop(_vard(), _optional("b", num2))), strop(_vare(), _optional("c", num2))),
            wop_plus(_vara(), strop(_vard(), _plus(_varb(), gop(_mult(_div(powcomp(_vare()), powcomp(_vard())), _varc()))))),
            SIMPLIFICATION_STAGE_MAIN,
            {
               { _vard(), ConditionType::is_constant_and_evaluates_to_true },                                 // denominator cannot be zero
               { _eq(_vard(), _vare()), ConditionType::is_constant_and_evaluates_to_false },                  // d and e cannot be equal
               { _div(powcomp(_vare()), powcomp(_vard())), ConditionType::is_constant_integer },              // e is divisible by d
               { _eq(_div(_vare(), _vard()), _vare()), ConditionType::is_constant_and_evaluates_to_false },   // but d is not 1
            },
            ConditionType::has_no_sideeffects, parser));

         // "(a + d * b) - e * c" ==> "a + e * (d / e * b + --c)" [ e != d && d / e is integer. ]
         vec_countable.insert(MetaRule(wop(wop_plus(_optional("a", num1), strop(_vard(), _optional("b", num2))), strop(_vare(), _optional("c", num2))),
            wop_plus(_vara(), strop(_vare(), _plus(_mult(_div(powcomp(_vard()), powcomp(_vare())), _varb()), gop(_varc())))),
            SIMPLIFICATION_STAGE_MAIN,
            {
               { _vare(), ConditionType::is_constant_and_evaluates_to_true },                                 // denominator cannot be zero
               { _eq(_vard(), _vare()), ConditionType::is_constant_and_evaluates_to_false },                  // d and e cannot be equal
               { _div(powcomp(_vard()), powcomp(_vare())), ConditionType::is_constant_integer },              // d is divisible by e
               { _eq(_div(_vare(), _vard()), _vare()), ConditionType::is_constant_and_evaluates_to_false },   // but d is not 1
            },
            ConditionType::has_no_sideeffects, parser));

         // Note: For now we only simplify      2 * x + 4 * y ==> 2 * (x + 2 * y)
         // But we don't go for                 6 * x + 8 * y ==> 2 * (3 * x + 4 * y)
         // because that could arguably be considered NOT a simplification since it has more nodes than before.
      }
      else if (wop_plus_str == this_ptr->getOptor() && this_ptr->getOpStruct().arg_num == 2) {
         // TODO: problematic rule set
         //    'o_(a, 1) / b ** o_(c, 1) * b ** o_(d, 1) ==> a * b ** (d +  --(c)) {{'has_no_sideeffects'}}'
         //    'o_(a, 0) - o_(b, 1) * c + o_(d, 1) * c ==> a + c * (d +  --(b)) {{'has_no_sideeffects'}}'
         // Will simplify: 3 + x + (3 + x) ==> 0
         // Will simplify: 3 * x * (3 * x) ==> (3 * x) ** 0
         //vec_countable.insert(MetaRule(wop_plus(wop(_optional("a", num1), strop(_vard(), _optional("b", num2))), strop(_vard(), _optional("c", num2))),
         //                                 wop_plus(_vara(), strop(_vard(), _plus(_varc(), gop(_varb())))),
         //                                 {},      // "(a - d * b) + d * c" ==> "a + d * (c + --b)"
         //                                 ConditionType::has_no_sideeffects, parser));
      }
   }

   vec_countable = createAllCommVariationsOfMetaRules(vec_countable);
   vec.insert(vec_countable.begin(), vec_countable.end());

   return vec;
}

std::set<MetaRule>& vfm::MathStruct::getStaticConvertedSimplificationRules(const std::shared_ptr<FormulaParser> parser)
{
   auto ptr_to_this_operator{ thisPtrGoIntoCompound() };
   auto ptr_to_this_possible_compound{ thisPtrGoUpToCompound() };
   auto all_ops{ all_rules_.find(ptr_to_this_operator->getOptor()) };
   
   if (all_ops == all_rules_.end()) {
      all_ops = all_rules_.insert({ ptr_to_this_operator->getOptor(), {} }).first;
   }

   auto res{ all_ops->second.find(ptr_to_this_operator->opnds_.size()) };

   if (res == all_ops->second.end()) {
      res = all_ops->second.insert({ static_cast<int>(ptr_to_this_operator->opnds_.size()), {} }).first;

      for (auto r : ptr_to_this_possible_compound->getSimplificationRules(parser)) {
         r.convertVeriablesToMetas();
         res->second.insert(r);
      }
   }

   return all_rules_.at(ptr_to_this_operator->getOptor()).at(ptr_to_this_operator->opnds_.size());
}

bool vfm::MathStruct::isRecursive()
{
   if (isTermCompound()) {
      std::set<MathStructPtr> visited;
      bool rec = false;
      auto break_ptr = std::make_shared<bool>(false);

      applyToMeAndMyChildren([break_ptr, &visited, &rec](const MathStructPtr m) {
         if (visited.count(m)) {
            *break_ptr = true;
            rec = true;
         }
         else {
            visited.insert(m);
         }
      }, TraverseCompoundsType::go_into_compound_structures, nullptr, break_ptr);

      return rec;
   }
   else {
      return false;
   }
}

float MathStruct::evalRange(
   const std::vector<std::pair<std::string, std::vector<std::shared_ptr<Term>>>>& assignmentRanges,
   const std::shared_ptr<Term> evalTerm,
   const bool useJIT)
{
   return evalRange(std::make_shared<DataPack>(), assignmentRanges, evalTerm, useJIT);
}

float MathStruct::evalRange(
   const std::shared_ptr<DataPack> data,
   const std::vector<std::pair<std::string, std::vector<std::shared_ptr<Term>>>>& assignmentRanges,
   const std::shared_ptr<Term> evalTerm,
   const bool useJIT)
{
   const int max_num_assignments = 10000;
   assert(assignmentRanges.size() <= max_num_assignments && "Too large number of assignments for 'evalRange'.");
   int counter[max_num_assignments];
   float result = 0.0f;
   DataPack d;

   d.initializeValuesBy(data);
   for (int i = 0; i <= assignmentRanges.size(); i++) {
      counter[i] = 0;
   }

   // Using original "data" here since there might be relevant external variables.
   while (!counter[assignmentRanges.size()]) {
      for (int i = 0; i < assignmentRanges.size(); i++) {
         if (useJIT) {
            randomize_rand_nums();
         }

         auto pair = assignmentRanges[i];
         auto var_name = pair.first;
         auto term_num = counter[i];
         auto var_term = pair.second[term_num];
         data->addOrSetSingleVal(var_name, var_term->eval(data));
      }

      float error = evalTerm->eval(data);

      if (error != error) { // This is true only for error = nan.
         return std::numeric_limits<float>::infinity();
      }
      else {
         result += error * error;
      }

      int i = 0;
      while (i < assignmentRanges.size() && counter[i] == assignmentRanges[i].second.size() - 1) {
         counter[i] = 0;
         i++;
      }

      counter[i]++;
   }

   data->initializeValuesBy(d);

   return result * (sqrtf(sqrtf((float)((assignmentRanges[assignmentRanges.size() - 1].second[0]->getNodeCount())))));
}

std::shared_ptr<Term> MathStruct::flattenFormula(const std::shared_ptr<Term> t, const std::set<std::string>& exception_operators)
{
   const auto mm{ _id(t->copy()) };
   mm->setChildrensFathers(); // TODO: Should be unnecessary.
   std::shared_ptr<Term> cp{ nullptr };
   
   while (!mm->isStructurallyEqual(cp)) {
      cp = mm->copy();

      mm->getOperands()[0]->applyToMeAndMyChildren([&exception_operators](const std::shared_ptr<MathStruct> m) {
         if (m->isTermCompound() && !exception_operators.count(m->getOptorOnCompoundLevel())) {
            m->toTermCompoundIfApplicable()->getFlatFormula();
         }
      });
   }

   return mm->getOperands()[0];
}

VariablesWithKinds MathStruct::findSetVariables(const std::shared_ptr<DataPack> data) const
{
   VariablesWithKinds fsm_controlled{};

   const_cast<MathStruct*>(this)->applyToMeAndMyChildren([&fsm_controlled, &data](const std::shared_ptr<MathStruct> m) {
      if (m->isTermSetVarOrAssignment()) {
         bool has_metas{ false };

         m->applyToMeAndMyChildren([&has_metas](const std::shared_ptr<MathStruct> m_inner) {
            if (m_inner->isMetaCompound()) {
               has_metas = true;
            }
            });

         if (!has_metas) {
            std::string name{ m->toSetVarIfApplicable()->getVarName(data) };

            if (!name.empty() && !StaticHelper::isPrivateVar(name)) {
               fsm_controlled.addOutputVariable(name);
            }
         }
      }
      else if (m->isTermVarNotOnLeftSideOfAssignment()) {
         std::string name{ m->toVariableIfApplicable()->getVariableName() };
         if (!StaticHelper::isPrivateVar(name)) {
            fsm_controlled.addInputVariable(name);
         }
      }
      }, TraverseCompoundsType::go_into_compound_structures);

   return fsm_controlled;
}
