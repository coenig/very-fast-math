//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file

#include "testing/test_functions.h"
#include "term.h"
#include "math_struct.h"
#include "fsm.h"
#include "data_pack.h"
#include "parser.h"
#include "failable.h"
#include "static_helper.h"
#include "examples/fct_enumdefinitions.h"
#include "vfmacro/script.h"
#include <iostream>
#include <algorithm>
#include <chrono>
#include <array>
#include <ctime>
#include <numeric>
#include <algorithm>
#include <thread>
#include <memory>
#include <string>

using namespace vfm;
using namespace fsm;
using namespace std::chrono;

typedef std::set<std::set<std::string>> setOfSets;
typedef std::vector<setOfSets> vecOfSetOfSets;
typedef std::vector<std::string> vecOfStr;
typedef std::vector<std::vector<std::string>> vecOfVecs;
typedef std::function<bool(const std::set<std::set<std::string>>& functions, std::shared_ptr<DataPack> d, const int seed, const int depth, const bool print)> TestFunction;
typedef std::vector<TestFunction> vecOfFunc;
typedef std::pair<std::string, std::function<float(const std::vector<float>&)>> mapOfFunc2;
typedef std::vector<std::vector<std::vector<mapOfFunc2>>> vecOfFunc2;
typedef std::pair<std::string, std::function<std::vector<std::string>(const std::vector<std::string>&)>> mapOfFunc3;
typedef std::vector<mapOfFunc3> vecOfFunc3;

const std::shared_ptr<std::vector<std::vector<float>>> additional_measures = std::make_shared<std::vector<std::vector<float>>>();
const std::shared_ptr<std::vector<std::string>> additional_info = std::make_shared<std::vector<std::string>>();

std::shared_ptr<FormulaParser> GENERAL_TEST_PARSER_FOR_EVAL;

float isPrimeReference(std::map<std::string, float> vals) {
   int number = vals.at("n");

   if (number < 2) return false;
   if (number == 2) return true;
   if (number % 2 == 0) return false;

   for (int i = 3; i * i <= number; i += 2){
      if (number % i == 0 ) return false;
   }

   return true;
}

float fibonacciReference(std::map<std::string, float> vals) {
   int number = vals.at("n");
   int a = 0, b = 1;
   if (number < 2) return number;

   for (int i = 1; i < number; ++i) {
      int c = a + b;
      a = b;
      b = c;
   }

   return b;
}

std::shared_ptr<FormulaParser> getGeneralParser()
{
   if (!GENERAL_TEST_PARSER_FOR_EVAL) {
      GENERAL_TEST_PARSER_FOR_EVAL = std::make_shared<FormulaParser>();
      GENERAL_TEST_PARSER_FOR_EVAL->initializeValuesBy(SingletonFormulaParser::getInstance());

      const std::string isprime_str = R"(
         @f(isprime__test) {p} {
            p == 2
            || p == 3
            || (
               @_c = (@_p = 1);
               while(square(@_c = _c + 1) <= p && _p) {
                  @_p = _p && p % _c;
               } 
            )
         };
      )";

      const std::string array_str = R"(
         @f(array__test) {a, b, c, x, y, z} {
            @arr = array3(a, b, c);
            fillArray3(arr, lambda(p_(0) * p_(1) * p_(2)), a, b, c);
            arr[x, y, z];
         };
      )";

      // This is an artificial function to test local variables in recursive calls. It's actually just reci__test(n) = n.
      const std::string reci_str = R"(
         @f(reci__test) {n} {
            @_n = n; 
            if (n > 0) {
               reci__test(reci__test(n - 1));
            };
            _n;
         };
      )";

      OperatorStructure reci_op_struct(OutputFormatEnum::prefix, AssociativityTypeEnum::left, 1, -1, "reci__test", false, false);
      GENERAL_TEST_PARSER_FOR_EVAL->forwardDeclareDynamicTerm(reci_op_struct);

      MathStruct::parseMathStruct(isprime_str, GENERAL_TEST_PARSER_FOR_EVAL);
      MathStruct::parseMathStruct(array_str, GENERAL_TEST_PARSER_FOR_EVAL);
      MathStruct::parseMathStruct(reci_str, GENERAL_TEST_PARSER_FOR_EVAL);
   }

   return GENERAL_TEST_PARSER_FOR_EVAL;
}

std::string m_line(const int length, const std::string symbol = "#")
{
   std::string result;
   for (int i = 0; i < length; i++)
      result += symbol;
   return result;
}

//////// TEST FUNCTIONS /////////
const auto TEST_JIT =                [](const std::set<std::set<std::string>>& operators, std::shared_ptr<DataPack> d, const int seed, const int depth, const bool print) { return MathStruct::testJitVsNojitRandom(operators, d, nullptr, seed, depth, 256, 256, 0.01, additional_measures, print); };
const auto TEST_PARSING =            [](const std::set<std::set<std::string>>& operators, std::shared_ptr<DataPack> d, const int seed, const int depth, const bool print) { return MathStruct::testParsing(operators, nullptr, seed, depth, 256, 256, 0.01, additional_measures, additional_info, print); };
const auto TEST_SIMPLIFY =           [](const std::set<std::set<std::string>>& operators, std::shared_ptr<DataPack> d, const int seed, const int depth, const bool print) { return MathStruct::testSimplifying(operators, d, nullptr, seed, depth, 256, 256, 0.01, additional_measures, additional_info, print); };
const auto TEST_SIMPLIFY_FAST =      [](const std::set<std::set<std::string>>& operators, std::shared_ptr<DataPack> d, const int seed, const int depth, const bool print) { return MathStruct::testFastSimplifying(operators, nullptr, seed, depth, 256, 256, depth, additional_measures, additional_info, print); };
const auto TEST_SIMPLIFY_VERY_FAST = [](const std::set<std::set<std::string>>& operators, std::shared_ptr<DataPack> d, const int seed, const int depth, const bool print) { return MathStruct::testVeryFastSimplifying(operators, nullptr, seed, depth, 256, 256, depth, additional_measures, additional_info, print); };
const auto TEST_EXTRACT =            [](const std::set<std::set<std::string>>& operators, std::shared_ptr<DataPack> d, const int seed, const int depth, const bool print) { return MathStruct::testExtracting(operators, d, nullptr, seed, depth, 256, 256, 0.01, 1000, additional_measures, additional_info, print); };
const auto TEST_FANNING =            [](const std::set<std::set<std::string>>& operators, std::shared_ptr<DataPack> d, const int seed, const int depth, const bool print) { return FSMs::testFanningOut(seed, 5, 3, depth, depth, 100, print); };
const auto TEST_EARLEY_P =           [](const std::set<std::set<std::string>>& operators, std::shared_ptr<DataPack> d, const int seed, const int depth, const bool print) { return MathStruct::testEarleyParsing(operators, MathStruct::EarleyChoice::Parser, "", seed, depth, seed % 2, nullptr, additional_measures, additional_info, print); };
const auto TEST_EARLEY_R =           [](const std::set<std::set<std::string>>& operators, std::shared_ptr<DataPack> d, const int seed, const int depth, const bool print) { return MathStruct::testEarleyParsing(operators, MathStruct::EarleyChoice::Recognizer, "", seed, depth, seed % 2, nullptr, additional_measures, additional_info, print); };
const auto TEST_EVAL =               [](const std::set<std::set<std::string>>& operators, std::shared_ptr<DataPack> d, const int seed, const int depth, const bool print) { 
   bool ok1 = MathStruct::testFormulaEvaluation(
      "isprime__test(n)", 
      isPrimeReference,
      { { "n", { RandomNumberType::Integer, { 0, 1000000 } } } },
      getGeneralParser(), 
      d, 
      seed, 
      0.01, 
      additional_measures, 
      additional_info, 
      print); 

   bool ok2 = MathStruct::testFormulaEvaluation(
      "isprime(n)", 
      isPrimeReference,
      { { "n", { RandomNumberType::Integer, { 0, 1000000 } } } },
      getGeneralParser(), 
      d, 
      seed, 
      0.01, 
      additional_measures, 
      additional_info, 
      print); 

   bool ok3 = MathStruct::testFormulaEvaluation(
      "fib(n)", 
      fibonacciReference,
      { { "n", { RandomNumberType::Integer, { 0, 10 } } } },
      getGeneralParser(), 
      d, 
      seed, 
      0.01, 
      additional_measures, 
      additional_info, 
      print); 

   bool ok4 = MathStruct::testFormulaEvaluation(
      "reci__test(n)", 
      [](std::map<std::string, float> vals) { return vals.at("n"); },
      { { "n", { RandomNumberType::Integer, { 0, 5 } } } },
      getGeneralParser(), 
      d, 
      seed, 
      0.01, 
      additional_measures, 
      additional_info, 
      print); 

   int dim = 4;

   bool ok5 = MathStruct::testFormulaEvaluation(
      "array__test(" + std::to_string(dim) + ", " + std::to_string(dim) + ", " + std::to_string(dim) + ", x, y, z)", 
      [](std::map<std::string, float> vals) { return vals.at("x") * vals.at("y") * vals.at("z");  },
      { 
         { "x", { RandomNumberType::Integer, { 0, dim - 1 } } },
         { "y", { RandomNumberType::Integer, { 0, dim - 1 } } },
         { "z", { RandomNumberType::Integer, { 0, dim - 1 } } },
      },
      getGeneralParser(), 
      d, 
      seed, 
      0.01, 
      additional_measures, 
      additional_info, 
      print); 

   return ok1 && ok2 && ok3 && ok4 && ok5;
};

const auto TEST_FIXED_SIMPL = [](const std::set<std::set<std::string>>& operators, std::shared_ptr<DataPack> d, const int seed, const int max, const bool print)
{
   auto fmla{ MathStruct::parseMathStruct(test::SIMPLIFICATION_TEST_BEFORE)->toTermIfApplicable() };
   fmla = mc::simplification::simplifyFast(fmla);
   bool ok{ StaticHelper::removeWhiteSpace(fmla->serialize()) == StaticHelper::removeWhiteSpace(test::SIMPLIFICATION_TEST_AFTER) };

   if (!ok || print) {
      Failable::getSingleton()->addNote("Formula '" + test::SIMPLIFICATION_TEST_BEFORE + "' simplified to '" + fmla->serialize() + "'.");

      if (!ok) {
         Failable::getSingleton()->addError("Should have been '" + test::SIMPLIFICATION_TEST_AFTER + "'.");
      }
   }

   return ok;
};

const auto TEST_SCRIPT_EXPANSION = [](const std::set<std::set<std::string>>& operators, std::shared_ptr<DataPack> d, const int seed, const int max, const bool print)
{
   static const std::array<int, 21> FIBS{ 0, 1, 1, 2, 3, 5, 8, 13, 21, 34, 55, 89, 144, 233, 377, 610, 987, 1597, 2584, 4181, 6765 };// , 10946, 17711, 28657, 46368, 75025, 121393, 196418, 317811, 514229, 832040 };
   std::srand(seed);
   int rand1 = std::rand() % (max - 0 + 1) + 0;
   int rand2 = rand1 * 13 % FIBS.size(); // Don't do too large numbers since it gets slow at some point. fibfast(20) is still very fast, though.

   std::string test_string1 = StaticHelper::replaceAll(R"(
@{@{
@(#0#)@
@(@{@{@{#0#}@*.sub[1].fib}@*}@.add[@{#0#}@*.sub[2].fib])@
}@**.if[this.smeq[#0#, 1]]}@***.newMethod[fib, 0]

@{var1=@{~{@{$}@.fib \0}~}@}@.nil
@{var2=@{var1}@}@.nil
@{var2}@
)", "$", std::to_string(rand1));

   std::string test_string2 = StaticHelper::replaceAll(R"(
@{@{
@(#0#)@
@(@{#0#.fibfast}@.sethard[
@{@{@{@{#0#}@*.sub[1]}@*.fibfast}@*}@.add[@{@{#0#}@*.sub[2]}@*.fibfast]])@
}@**.if[this.smeq[#0#, 1]]}@***.newMethod[fibfast, 0]

@{var1=@{~{@{$}@.fibfast \0}~}@}@.nil
@{var2=@{var1}@}@.nil
@{var2}@
)", "$", std::to_string(rand2));

   std::shared_ptr<macro::Script> s1 = std::make_shared<macro::Script>(nullptr, nullptr);
   if (!print) s1->setOutputLevels(ErrorLevelEnum::error, ErrorLevelEnum::error);
   s1->applyDeclarationsAndPreprocessors(test_string1);
   std::string result1 = StaticHelper::trimAndReturn(s1->getProcessedScript());

   std::shared_ptr<macro::Script> s2 = std::make_shared<macro::Script>(nullptr, nullptr);
   if (!print) s2->setOutputLevels(ErrorLevelEnum::error, ErrorLevelEnum::error);
   s2->applyDeclarationsAndPreprocessors(test_string2);
   std::string result2 = StaticHelper::trimAndReturn(s2->getProcessedScript());

   bool error1 = s1->hasErrorOccurred();
   bool error2 = s2->hasErrorOccurred();
   std::string expected1 = std::to_string(FIBS.at(rand1));
   std::string expected2 = std::to_string(FIBS.at(rand2));
   bool fine = !error1 && !error2 && result1 == expected1 && result2 == expected2;

   if (print && fine) {
      Failable::getSingleton()->addNote("Fibonacci scripts expanded correctly to fib(" + std::to_string(rand1) + ") = " + expected1 + " / fibfast(" + std::to_string(rand2) + ") = " + expected2 + ".");
   }

   if (!fine) {
      Failable::getSingleton()->addError("Fibonacci scripts did not expand correctly to fib(" + std::to_string(rand1) + ") = " + expected1 + " / fibfast(" + std::to_string(rand2) + ") = " + expected2 + " . Instead the result was: " + result1 + " / " + result2 + ".");
   }

   return fine;
};
//////// EO TEST FUNCTIONS /////////

//////// HELPER FUNCTIONS /////////
const std::set<std::string> VARS{ "a" };
const std::set<std::string> VARS_AND_ARRAYS{ "a", ".A", ".H_f"};
const setOfSets ARITH_TERMS =                         { TERMS_ARITH, VARS };
const setOfSets LOGIC_TERMS =                         { TERMS_LOGIC, VARS };
const setOfSets NO_VARS =                             { TERMS_ARITH, TERMS_LOGIC, TERMS_SPECIAL };
const setOfSets ALL_TERMS =                           { TERMS_LOGIC, TERMS_ARITH };
const setOfSets ALL_TERMS_PLUS_SPECIAL_WITHOUT_VARS = { TERMS_LOGIC, TERMS_ARITH, TERMS_SPECIAL, VARS_AND_ARRAYS };
const setOfSets ALL_TERMS_PLUS_SPECIAL =              { TERMS_LOGIC, TERMS_ARITH, TERMS_SPECIAL, VARS_AND_ARRAYS };
const setOfSets ALL_TERMS_PLUS_SPECIAL_AND_SET =      { TERMS_LOGIC, TERMS_ARITH, TERMS_SPECIAL, VARS_AND_ARRAYS, {/*"set", "Set", "Get"*/} };
//const setOfSets ALL_TERMS_PLUS_SPECIAL_AND_SET_AND_MEMORY = { TERMS_LOGIC, TERMS_ARITH, TERMS_SPECIAL, TERMS_MEMORY, VARS, {"set", "Set", "Get"}  };

const mapOfFunc2 sum = std::make_pair("SUM", [](const std::vector<float>& v) { return accumulate(v.begin(), v.end(), 0.0); });
const mapOfFunc2 avg = std::make_pair("AVG", [](const std::vector<float>& v) { return accumulate(v.begin(), v.end(), 0.0) / v.size(); });
const mapOfFunc2 avg_int = std::make_pair("AVG_I", [](const std::vector<float>& v) { return (int) avg.second(v); });
const mapOfFunc2 min = std::make_pair("MIN", [](const std::vector<float>& v) { return *std::min_element(v.begin(), v.end()); });
const mapOfFunc2 max = std::make_pair("MAX", [](const std::vector<float>& v) { return *std::max_element(v.begin(), v.end()); });

const mapOfFunc3 id = std::make_pair("IDENTITY", [](const std::vector<std::string>& v) -> std::vector<std::string> { return v; });
const mapOfFunc3 count = std::make_pair("COUNT-DISTINCT", [](const std::vector<std::string>& v) -> std::vector<std::string> {
   std::vector<std::string> res;
   std::vector<std::string> res2;

   long int cntmax = 0;
   int scntmax_size;
   int sum = 0;
   
   for (const auto& s : v) {
     if (std::find(res.begin(), res.end(), s) == res.end()) {
       res.push_back(s);
       auto cnt = std::count(v.begin(), v.end(), s);
       cntmax = std::max(static_cast<long int>(cnt), cntmax);
     }
   }

   scntmax_size = std::to_string(cntmax).size();

   for (const auto& s : res) {
     int cnt = std::count(v.begin(), v.end(), s);
     sum += cnt;
     std::string scnt = std::to_string(cnt);
     scnt = m_line(scntmax_size - scnt.size(), "0") + scnt;
      res2.push_back(scnt + "x   " + s);
   }
   
   std::sort(res2.begin(), res2.end());
   std::reverse(res2.begin(), res2.end());
   
   res2.push_back(std::to_string(sum) + "x");
   
   return res2; 
});

const std::vector<std::string> JIT_MEAS = { "nodes", "time reg", "time JIT" };
const std::vector<std::vector<mapOfFunc2>> JIT_MEAS_FUNCS = {{avg, max}, {avg}, {avg}};
//////// EO HELPER FUNCTIONS /////////

constexpr int NUM_JIT =
#ifdef ASMJIT_ENABLED
100
#else
0
#endif // ASMJIT_ENABLED
;

struct TestCase {
   std::string test_name_{};
   TestFunction test_function_{};
   int number_of_runs_{};
   int formula_depth_{};   // If applicable.
   setOfSets operators_{}; // If applicable.
   std::vector<std::string> additional_measures_{};
   std::vector<std::vector<mapOfFunc2>> additional_evaluation_functions_{};
   std::string additional_info_name_{};
   mapOfFunc3 additional_info_function{};
};

//////// TEST CASE DESCRIPTIONS /////////
static const std::vector<TestCase> TEST_CASES{
   // NUM            Name                     Test Function    Random [Iterations Depth Allowed-Terms]                       Measurements  Measurement-Functions Summary     Summary-Function
   /* 0  */ TestCase{"ASMJIT ARITH",          TEST_JIT,                NUM_JIT,   8,    ARITH_TERMS,                         JIT_MEAS,     JIT_MEAS_FUNCS,       "",         id},
   /* 1  */ TestCase{"ASMJIT LOGIC",          TEST_JIT,                NUM_JIT,   8,    LOGIC_TERMS,                         JIT_MEAS,     JIT_MEAS_FUNCS,       "",         id},
   /* 2  */ TestCase{"ASMJIT COMBINED",       TEST_JIT,                NUM_JIT,   8,    ALL_TERMS,                           JIT_MEAS,     JIT_MEAS_FUNCS,       "",         id},
   /* 3  */ TestCase{"PARSING",               TEST_PARSING,            100,       7,    ALL_TERMS_PLUS_SPECIAL,              {"nodes"},    {{avg, max}},         "Warnings", count},
   /* 4  */ TestCase{"EXTRACTING",            TEST_EXTRACT,            100,       7,    ALL_TERMS_PLUS_SPECIAL_AND_SET,      {"nodes raw", "nodes enc", "encaps", "fully"}, {{avg, max},  {avg, max},  {avg},    {sum}}, "Rules applied", count},
   /* 5  */ TestCase{"SIMPLIFYING",           TEST_SIMPLIFY,           10000,     5,    ALL_TERMS_PLUS_SPECIAL_AND_SET,      {"compression", "nodes raw", "nodes simp"}, {{min, avg, max}, {avg, max},  {avg, max}}, "Rules applied", count},
   /* 6  */ TestCase{"FAST SIMPLIFYING",      TEST_SIMPLIFY_FAST,      100,       5,    ALL_TERMS_PLUS_SPECIAL_WITHOUT_VARS, {"compression", "nodes raw", "nodes simp", "time hardcoded", "time hardcoded (pos)", "time interpreted", "speedup", "speedup (pos)"}, {{min, avg, max}, {min, avg, max},  {min, avg, max}, {avg}, {avg}, {avg}, {avg}, {avg}}, "Rules applied", count},
   /* 7  */ TestCase{"VERY FAST SIMPLIFYING", TEST_SIMPLIFY_VERY_FAST, 0,         2,    ALL_TERMS_PLUS_SPECIAL_WITHOUT_VARS, {"compression", "nodes raw", "nodes simp", "very fast", "fast", "speedup"}, {{min, avg, max}, {min, avg, max},  {min, avg, max}, {avg}, {avg}, {avg}}, "", id},
   /* 8  */ TestCase{"FSM FAN-OUT",           TEST_FANNING,            0,         4,    {},                                  {},           {},                   "", id}, // Deprecated
   /* 9  */ TestCase{"EARLEY RECOGNIZING",    TEST_EARLEY_R,           5,         5,    NO_VARS,                             {},           {},                   "Warnings", count},
   /* 10 */ TestCase{"EARLEY PARSING",        TEST_EARLEY_P,           5,         2,    NO_VARS,                             {},           {},                   "Warnings", count},
   /* 11 */ TestCase{"GENERAL EVALUATION",    TEST_EVAL,               50,       -1,   {},                                  {},           {},                   "Formulas tested", count},
   /* 12 */ TestCase{"SCRIPT_EXPANSION",      TEST_SCRIPT_EXPANSION,   100,       8,    {},                                  {},           {},                   "",         id},
   /* 13 */ TestCase{"FIXED_SIMPLIFICATION",  TEST_FIXED_SIMPL,        1,        -1,    {},                                  {},           {},                   "",         id},
};
//////// EO TEST CASE DESCRIPTIONS /////////

float genSeed() {

   return duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
}

std::string frame()
{
   return m_line(3);
}

std::string emptyLine(const int length) 
{
   return frame() + m_line(length - 2 * frame().size(), " ") + frame();
}

const std::string RIGHT_ALIGNMENT_SYMBOL = "$$&%&$$";

void print_result(
   const int count,
   const int num,
   const std::string& name,
   const std::vector<std::string>& additionals_names,
   const std::vector<std::vector<float>>& additionals,
   const std::string add_info_name,
   const std::vector<std::string> add_info,
   const std::vector<std::vector<mapOfFunc2>>& funcs,
   const long time)
{
   std::string title_str = " Results of " + HIGHLIGHT_COLOR + name + RESET_COLOR + " tests (" + std::to_string(time / 1000.0) + " s) ";
   std::string result_str = frame() + " " + (count == num ? OK_COLOR : FAILED_COLOR) + std::to_string(count) + " out of " + std::to_string(num) + " tests passed (" + std::to_string((100.0 * (float) (count) / (float) num)) + "%) ==> " + (count == num ? "OK" : "FAILED") + RESET_COLOR + " ";
   std::vector<std::string> additional_lines;

   int additionals_len = 0;
   int result_len = result_str.length() - OK_COLOR.length() - RESET_COLOR.length() + frame().size();
   int title_len = title_str.length() - HIGHLIGHT_COLOR.length() - RESET_COLOR.length();
   bool alter = false;
   int i = 0;

   const int COLORING_LENGTH_ADDS = HIGHLIGHT_COLOR.size() + RESET_COLOR.size();

   int buff_len = 0;
   for (const auto& name : additionals_names) {
      buff_len = std::max(buff_len, (int) name.size());
   }

   for (int i = 0; i < additionals.size(); i++) {
      std::string line = HIGHLIGHT_COLOR + additionals_names[i] + m_line(buff_len - additionals_names[i].size(), " ") + ":";

      for (int j = 0; j < funcs[i].size(); j++) {
         line += " #" + funcs[i][j].first + " ";
         line += std::to_string(funcs[i][j].second(additionals[i]));
      }

      line += RESET_COLOR + " ";
      additional_lines.push_back(line);
   }

   if (!add_info.empty()) {
      additional_lines.push_back(HIGHLIGHT_COLOR + RESET_COLOR);
      additional_lines.push_back(HIGHLIGHT_COLOR + add_info_name + ":" + RESET_COLOR);
      for (const auto& str : add_info) {
         auto color = StaticHelper::stringContains(str, errorLevelMapping.at(ErrorLevelEnum::warning).second) || StaticHelper::stringContains(str, "(EMPTY)")
            ? WARNING_COLOR 
            : (StaticHelper::stringContains(str, "SUCCESS")
               ? HIGHLIGHT_COLOR // could also be OK_COLOR
               : (StaticHelper::stringContains(str, "FAILURE")
                  ? FAILED_COLOR
                  : HIGHLIGHT_COLOR));
         additional_lines.push_back(color + str + " " + RESET_COLOR);
      }
   }

   for (const auto& line : additional_lines) {
      if (additionals_len < line.size()) {
         additionals_len = StaticHelper::replaceAll(line, RIGHT_ALIGNMENT_SYMBOL, "").size() + 1; // TODO: No idea why the +1 is needed.
      }
   }

   for (auto& line : additional_lines) {
      line = StaticHelper::replaceAll(line, RIGHT_ALIGNMENT_SYMBOL, m_line(additionals_len - (StaticHelper::replaceAll(line, RIGHT_ALIGNMENT_SYMBOL, "").size()), " "));
   }

   additionals_len -= COLORING_LENGTH_ADDS - 2 * frame().size() - 1;
   
   if (additionals_len > result_len) {
      result_len = additionals_len;
   }

   int num_spaces = result_len - result_str.size() + 3 * frame().size() + 1;
   if (OK_COLOR.empty()) {
      num_spaces -= 13; // No idea why 13. TODO: Remove magic number.
   }
   result_str += m_line(num_spaces, " ") + frame();

   while (title_len + i < result_len)
   {
      alter ? title_str = m_line(1, std::string(" ")) + title_str : title_str = title_str + m_line(1, std::string(" "));
      alter = !alter;
      i++;
   }

   std::cout << std::endl << title_str << std::endl;
   std::cout << m_line(result_len) << std::endl;

   if (!additional_lines.empty()) {
      std::cout << emptyLine(result_len) << std::endl;
   }

   std::cout << result_str << std::endl;

   if (!additional_lines.empty()) {
      std::cout << emptyLine(result_len) << std::endl;
   }

   for (const auto& line : additional_lines) {
      int empty_len = result_len - frame().size() * 2 - line.size() - 1 + COLORING_LENGTH_ADDS;
      std::cout << frame() << " " << line << m_line(empty_len, " ") << frame() << std::endl;
   }
   if (!additional_lines.empty()) {
      std::cout << emptyLine(result_len) << std::endl;
   }

   std::cout << m_line(result_len) << std::endl << std::endl;
}

int testTermsGeneral(
      const std::set<std::set<std::string>>& operators, 
      std::shared_ptr<DataPack> d, 
      const std::string test_name, 
      const int num,
      const int depth, 
      const int seed_init,
      const std::function<bool(const std::set<std::set<std::string>>& operators,
                               std::shared_ptr<DataPack> d,
                               const int seed,
                               const int depth,
                               const bool print)>& f,
      std::map<std::string, bool>& result)
{
   bool success{ true };
   std::string print_format{ OK_COLOR };
   int count{ 0 };
   std::cout << std::endl << HIGHLIGHT_COLOR << frame() << " " << "Testing " << num << " " << test_name << " instances with max. depth " << depth << " " << frame() << RESET_COLOR << std::endl;
   int time{ (int) std::time(nullptr) };

   for (int i = 0; i < num; i++) {
      bool print{ false };
      if (std::time(nullptr) - time > 3) {
         print = true;
         time = std::time(nullptr);
         std::cout << print_format << "[" << 100 * i / num << "%] " << RESET_COLOR;
      }
     
      bool success_temp{ f(operators, d, i + seed_init, depth, print) };
      
      result.insert({ test_name, success_temp });

      count += success_temp;
      success = success && success_temp;
      print_format = (success ? OK_COLOR : FAILED_COLOR);
   }

   std::cout << print_format << "[" << 100 << "%]" << RESET_COLOR << std::endl;
   return count;
}

std::map<std::string, bool> vfm::test::runTests(const int from_raw, const int to_raw)
{
   std::map<std::string, bool> result{};
   const int from{ std::max(from_raw, 0) };
   const int to{ from_raw < 0 ? (int) TEST_CASES.size() - 1 : std::max(from, to_raw)};

   std::cout << "\r\n===============================================" << std::endl;
   std::cout << "<vfm Tester> Hello, I'm starting the tests now." << std::endl;
   std::cout << "===============================================\r\n" << std::endl;

   std::vector<std::vector<float>> empty_adds{};
   std::vector<std::string> empty_add_names{};
   std::vector<std::vector<mapOfFunc2>> empty_funcs{};
   std::vector<std::string> empty_add_info{};
   std::vector<long> times{};

   std::vector<std::vector<std::vector<float>>> additionals{};
   std::vector<std::vector<std::string>> additional_infos{};
   std::vector<int> counts{};
   int sum{ 0 };
   int maxsum{ 0 };
   long glob_time{ 0 };

   std::shared_ptr<DataPack> d{ std::make_shared<DataPack>() };

   d->addOrSetSingleVal("a", genSeed() / 10000000);
   d->addOrSetSingleVal("x", genSeed() / 100000000);
   d->addOrSetSingleVal("y", genSeed() / 5000000);
   d->addOrSetSingleVal("z", genSeed() / 1000000);
   d->addOrSetSingleVal("const_approx", 5, true, true);

   for (float i = 0; i < 256; i++) {
      d->addArrayAndOrSetArrayVal(".H_f", (int)i, i / (i + 1));
      d->addArrayAndOrSetArrayVal(".A", (int)i, 1 + ((int)(i * i)) % 13);
   }

   std::string td_str{ HIGHLIGHT_COLOR + m_line(10) + " " + "Using this test data" + " " + m_line(10) + RESET_COLOR };
   int td_len{ (int) (td_str.length() - HIGHLIGHT_COLOR.length() - RESET_COLOR.length()) };

   std::cout << std::endl << td_str << std::endl << *d << HIGHLIGHT_COLOR << m_line(td_len) << RESET_COLOR << std::endl << std::endl;

   for (int i = std::max(0, from); i < std::min((int) TEST_CASES.size(), to + 1); i++) {
      //std::vector<float> new_additional_measures;
      //additionals.push_back(new_additional_measures);
      additional_measures->clear();
      additional_info->clear();
      int seed_init = genSeed() / (i + 1); // This will produce different seeds even if one loop lasts shorter than 1ms.

      while (counts.size() <= i) {
         counts.push_back(0);
      }

      std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
      counts[i] = testTermsGeneral(TEST_CASES[i].operators_, d, TEST_CASES[i].test_name_, TEST_CASES[i].number_of_runs_, TEST_CASES[i].formula_depth_, seed_init, TEST_CASES[i].test_function_, result);
      std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

      long time = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();

      while (times.size() <= i) {
         times.push_back(time);
      }

      glob_time += time;

      sum += counts[i];
      maxsum += TEST_CASES[i].number_of_runs_;

      while (additionals.size() <= i) {
         additionals.push_back(*additional_measures);
      }
     
      auto vec = TEST_CASES[i].additional_info_function.second(*additional_info);
      while (additional_infos.size() <= i) {
         additional_infos.push_back(vec);
      }
   }
 
   for (int i = std::max(0, from); i < std::min((int) TEST_CASES.size(), to + 1); i++) {
      empty_add_info.push_back(
         std::to_string(counts[i]) + "/" + std::to_string(TEST_CASES[i].number_of_runs_) + " " + TEST_CASES[i].test_name_ + " tests passed. " + RIGHT_ALIGNMENT_SYMBOL
         + (counts[i] == TEST_CASES[i].number_of_runs_
            ? (TEST_CASES[i].number_of_runs_ ? "SUCCESS" : "(EMPTY)")
            : "FAILURE"));
      print_result(counts[i], TEST_CASES[i].number_of_runs_, TEST_CASES[i].test_name_, TEST_CASES[i].additional_measures_, additionals[i], TEST_CASES[i].additional_info_name_ + " (#" + TEST_CASES[i].additional_info_function.first + ")", additional_infos[i], TEST_CASES[i].additional_evaluation_functions_, times.at(i));
   }

   print_result(sum, maxsum, "*ALL*", empty_add_names, empty_adds, "Summary", empty_add_info, empty_funcs, glob_time);

   return result;
}

// Live FSM test functions originating from G1 demo times.
#ifdef _WIN32
std::string dir = "../examples/G1-demo_FSM/";
#else
std::string dir = "/home/okl2abt/works/vfm/examples/G1-demo_FSM/";
#endif

std::string static_fsm_file_path = dir + "FCT_FSM.vfm";
std::string static_fsm_init_file_path = dir + "FCT_FSM_init.vfm";
std::string static_fsm_change_file_path = dir + "FCT_FSM_change.vfm";

//void testFSMsimple()
//{
//   auto resolver = std::make_shared<FSMResolverDefaultMaxTransWeight>(2.5);
//   std::shared_ptr<FSMs> m = std::make_shared<FSMs>(resolver, NonDeterminismHandling::note_error_on_non_determinism);
//   m->setOutputLevels(ErrorLevelEnum::note, ErrorLevelEnum::note);
//
//   associateVariables(m);
//
//   m->loadFromFile(fsm_init_file_path);
//   m->printAndThrowErrorsIfAny(true, false);
//   m->loadFromFile(fsm_file_path, false, true);
//   m->printAndThrowErrorsIfAny(true, false);
//
//   m->createGraficOfCurrentGraph(dir + "G1-demo-FSM.dot", true, "pdf", false, true);
//   m->createGraficOfCurrentGraph(dir + "G1-demo-FSM_data.dot", true, "pdf", false, false, true);
//
//   m->fanOutAllNonAtomicCallbacksToStates();
//   m->simplifyFSM();
//   m->createGraficOfCurrentGraph(dir + "G1-demo-FSM_FOUT.dot", true, "pdf", false, true);
//
//   //std::cout << std::endl << "---- FSM as CPP ----" << std::endl << m->toCPP() << std::endl << "---- EO FSM as CPP ----" << std::endl;
//   //term();
//
//   m->setStatePainter(paint_func);
//
//   for (;;)
//   {
//      m->loadFromFile(fsm_change_file_path, false);
//      m->printAndThrowErrorsIfAny(true, false);
//
//      m->createGraficOfCurrentGraph(dir + "G1-demo-FSM.dot", true, "pdf", false, true);
//      m->createGraficOfCurrentGraph(dir + "G1-demo-FSM_data.dot", true, "pdf", false, false, true);
//      m->createStateVisualizationOfCurrentState(dir + "G1-demo-FSM-state-viz.jpg", OutputType::jpg);
//
//      std::this_thread::sleep_for(std::chrono::milliseconds((int)m->evaluateFormula("expectedFctCycleTime")));
//      m->step(SteppingOrderEnum::execute_callbacks_then_transition);
//   }
//}

void testFSM()
{
   constexpr bool CREATE_NUSMV_CODE = false;
   constexpr bool SIMPLIFY_FSM = false;
   constexpr bool FAN_OUT_FSM = false;
   constexpr int NUM_OF_FANOUT_EQUIVALENCY_TESTS = 10;

   auto resolver = std::make_shared<FSMResolverDefaultMaxTransWeight>(2.5);
   std::shared_ptr<FSMs> m = std::make_shared<FSMs>(resolver, NonDeterminismHandling::note_error_on_non_determinism);

   m->setOutputLevels(ErrorLevelEnum::debug);

   Fct::associateVariables(m);

   m->loadFromFile(static_fsm_init_file_path);
   m->printAndThrowErrorsIfAny(true, false);
   m->loadFromFile(static_fsm_file_path, false, true);
   m->printAndThrowErrorsIfAny(true, false);

   if (SIMPLIFY_FSM) {
      std::cout << "SIMPLIFYING regular version..." << std::endl;
      m->simplifyConditions(false);
      m->removeUnsatisfiableTransitionsAndUnreachableStates();
   }

   m->createGraficOfCurrentGraph(dir + "G1-demo-FSM.dot", true, "pdf", false, vfm::fsm::GraphvizOutputSelector::graph_only);
   m->createGraficOfCurrentGraph(dir + "G1-demo-FSM_data.dot", true, "pdf", false, vfm::fsm::GraphvizOutputSelector::data_with_functions);

   if (FAN_OUT_FSM) {
      m->fanOutMultipleCallbacksToIndividualStates(); // Needed for equivqlency test, done at next step anyway.
      auto m_temp = m->copy();
      m->fanOutAllNonAtomicCallbacksToStates();

      if (SIMPLIFY_FSM) {
         std::cout << "SIMPLIFYING fanned-out version..." << std::endl;
         m->simplifyConditions(false);
         m->removeUnsatisfiableTransitionsAndUnreachableStates();
      }

      m->encapsulateLongConditionsIntoFunctions();

      std::string name_regular = dir + "FSM_regular_static.dot";
      std::string name_fanned = dir + "FSM_fanned_out_static.dot";
      m_temp.createGraficOfCurrentGraph(name_regular, true, "pdf", false, vfm::fsm::GraphvizOutputSelector::graph_only);
      m->createGraficOfCurrentGraph(name_fanned, true, "pdf", false, vfm::fsm::GraphvizOutputSelector::graph_only);
      std::cout << "Stored regular FSM in '" << name_regular << "'." << std::endl;
      std::cout << "Stored fanned-out FSM in '" << name_fanned << "'." << std::endl;
      std::cout << "Fanned-out version has " << m->getStateCount() << " states" << std::endl;

      for (int i = 1; i <= NUM_OF_FANOUT_EQUIVALENCY_TESTS; i++) {
         std::cout << i << "/" << NUM_OF_FANOUT_EQUIVALENCY_TESTS << ") Testing EQUIVALENCY of regular FSM and fanned-out FSM..." << std::endl;
         if (!m->testEquivalencyWith(m_temp, 20)) {
            return;
         }
      }

      std::cout << "Equivalency tests passed." << std::endl;
   }

   if (CREATE_NUSMV_CODE) {
      std::string nusmv_file = dir + (FAN_OUT_FSM ? "G1_FCT_fanned-out.smv" : "G1_FCT_pc.smv");
      std::cout << "Creating nuSMV code..." << std::endl;
      std::ofstream out(nusmv_file);

      out << "TODO: This does not currently work." << std::endl;
      //out << m->serializeNuSMV(FAN_OUT_FSM ? MCTranslationTypeEnum::fan_out_non_atomic_states : MCTranslationTypeEnum::emulate_program_counter, { { _true() }, { _true() } });
      out.close();
      std::cout << "Stored in " << nusmv_file << "." << std::endl;
   }

   //m->setStatePainter(paint_func);

   for (;;)
   {
      m->loadFromFile(static_fsm_change_file_path, false);
      m->printAndThrowErrorsIfAny(true, false);

      if (FAN_OUT_FSM) {
         m->createGraficOfCurrentGraph(dir + "G1-demo-FSM_fanned-out.dot", true, "pdf", false, vfm::fsm::GraphvizOutputSelector::graph_only);
         m->createGraficOfCurrentGraph(dir + "G1-demo-FSM_fanned-out_data.dot", true, "pdf", false, vfm::fsm::GraphvizOutputSelector::data_with_functions);
      }
      else {
         m->createGraficOfCurrentGraph(dir + "G1-demo-FSM.dot", true, "pdf", false, vfm::fsm::GraphvizOutputSelector::graph_only);
         m->createGraficOfCurrentGraph(dir + "G1-demo-FSM_data.dot", true, "pdf", false, vfm::fsm::GraphvizOutputSelector::data_with_functions);
      }

      //m->createStateVisualizationOfCurrentState(dir + "G1-demo-FSM-state-viz.jpg", OutputType::jpg);

      std::this_thread::sleep_for(std::chrono::milliseconds((int)m->evaluateFormula("expectedFctCycleTime")));
      m->step(SteppingOrderEnum::execute_callbacks_then_transition);
   }
}
