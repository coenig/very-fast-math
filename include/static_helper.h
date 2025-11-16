//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file
#pragma once

#include "term.h"
#include "model_checking/mc_types.h"
#include "fsm_transition.h"
#include <vector>
#include <sstream>
#include <string>
#include <iterator>
#include <regex>
#include <cmath>
#include <functional>
#include <deque>
#include <chrono>
#include <thread>
#include <queue>
#include <mutex>
#include <condition_variable>

#if __cplusplus >= 201703L // https://stackoverflow.com/a/51536462/7302562 and https://stackoverflow.com/a/60052191/7302562
#include <numeric>
#include <filesystem>
#endif

namespace vfm {

const std::string BEGIN_TAG_TIMESCALING_DESCRIPTIONS{ "-- SCALING DESCRIPTIONS" };
const std::string END_TAG_TIMESCALING_DESCRIPTIONS{ "-- EO SCALING DESCRIPTIONS" };
const std::string TIMESCALING_FILENAME{ "scaling_info.txt" };

const std::string RICK{ "wubba lubba dub dub" };
const std::string MORTY_ASCII_ART{R"(         ___                       
  __  __|_  )      ____ _____      
 |  \/  |/ /  ___ |  _ \_   _|   _ 
 | |\/| /___|/ _ \| |_) || || | | |
 | |  | |   | (_) |  _ < | || |_| |
 |_|  |_|    \___/|_| \_\|_| \__, |
                             |___/ 
)"};

class VariableScaleDescription;
class ScaleDescription;

enum class ScaleTypeEnum
{
   none,
   time,
   distance,
   velocity,
   acceleration
};

const std::map<ScaleTypeEnum, std::string> scale_type_map{
   { ScaleTypeEnum::none, "none" },
   { ScaleTypeEnum::time, "time" },
   { ScaleTypeEnum::distance, "distance" },
   { ScaleTypeEnum::velocity, "velocity" },
   { ScaleTypeEnum::acceleration, "acceleration" },
};

class ScaleType : public fsm::EnumWrapper<ScaleTypeEnum>
{
public:
   explicit ScaleType(const ScaleTypeEnum& enum_val) : EnumWrapper("TimescaleType", scale_type_map, enum_val) {}
   explicit ScaleType(const int enum_val_as_num) : EnumWrapper("TimescaleType", scale_type_map, enum_val_as_num) {}
   explicit ScaleType(const std::string& enum_val_as_string) : EnumWrapper("TimescaleType", scale_type_map, enum_val_as_string) {}
   explicit ScaleType() : EnumWrapper("TimescaleType", scale_type_map, ScaleTypeEnum::none) {}
};

using SplitCondition = const std::function<bool(const std::string& split_str, const int pos)>;

const SplitCondition FUNC_ALL_STRINGS_TO_TRUE = [](const std::string& split_str, const int pos) -> bool {
   return true; 
};

const auto TRUE_FUNCTION = [](const std::string& str, const int index) { return true; };
const std::set<std::string> DEFAULT_CPP_ARRAY_TYPE_NAMES = {
   "std::array",
   "std::vector",
   "vfc::TFixedVector",
   "vfc::TCArray",
   "TFixedVector",
   "TCArray",
   "FixedVector",
   "Array",
};

const std::string ARRAY_INDEX_DENOTER_OPEN{ "___6" };  // x[i][j] ==> x___6i9___6j9___
const std::string ARRAY_INDEX_DENOTER_CLOSE{ "9___" }; // x[i[j]] ==> x___6i___6j9___9___
const std::string FAILABLE_NAME_SANITY_CHECK{ "SanityCheckC++Tokens" };
const std::string FAILABLE_NAME_CPP_DATATYPES{ "C++Datatypes" };
const std::string STRING_PREFIX_TAL{ ".." };
const std::string STRING_PREFIX_AKA{ STRING_PREFIX_TAL + "." };
const std::string STRING_PREFIX_INIT{ STRING_PREFIX_AKA + "." };
const std::string REGEX_PREFIX{ "REGEX:" };
const std::string STRING_PREFIX_ENVMODEL_GEN{ "$" };

const std::string MC_INTERACTIVE_MODE_CONSTANT_NAME{ "mcint" };

const std::string JSON_VFM_FORMULA_PREFIX{ "#" };
const std::string JSON_NUXMV_FORMULA_BEGIN{ JSON_VFM_FORMULA_PREFIX + "{" };
const std::string JSON_NUXMV_FORMULA_END{ "}" + JSON_VFM_FORMULA_PREFIX };
const std::string JSON_TEMPLATE_DENOTER{ JSON_VFM_FORMULA_PREFIX + "TEMPLATE" };

class FormulaParser;

namespace simplification {
class SimplificationFunctionLine;
}

using Case = std::pair<std::string, std::string>;                                            // { Condition, Code }
using CaseWithBreakInfo = std::pair<Case, bool>;                                             // { Case, has_break }
using Cases = std::pair<std::vector<CaseWithBreakInfo>, std::shared_ptr<CaseWithBreakInfo>>; // { { CaseWithBreakInfo, CaseWithBreakInfo, ... }, *DefaultWithBreakInfo }
using SwitchBlock = std::pair<std::string, Cases>;                                           // { switch_var, Cases }

class BracketStructure : public std::enable_shared_from_this<BracketStructure> 
{
public:
   BracketStructure(const std::string& content, const std::vector<std::shared_ptr<BracketStructure>> children)
      : content_(content), children_(children)
   {}

   std::string getContentAt(const std::vector<int> path_through_tree);

   std::string serialize(const std::string& opening_bracket, const std::string& closing_bracket, const std::string& delimiter) const;

   std::string content_{};
   std::vector<std::shared_ptr<BracketStructure>> children_{};
};

class MetaRule;

// Helper implementation of a simple counter functionality on vector basis.
// Create vector of "arities" of the positions: ar = { an, ..., a1, a0 }.
// Then start with a counter instance: ci = { an - 1, ..., a1 - 1, a0 }. (Note the a0 (NOT a0 - 1) at the end.)
//
// To go through all possible combinations: 
// while (!VECTOR_COUNTER_IS_ALL_ZERO(ci)) {
//    VECTOR_COUNTER_DECREMENT(ar, ci);
//    *execute whatever code depends on the counter*
// }
const auto VECTOR_COUNTER_DECREMENT = [](const std::vector<int>& arity_vec, std::vector<int>& instance_vec) {
   for (int i = 0; i < arity_vec.size(); i++) {
      if (instance_vec[i] > 0) {
         instance_vec[i]--;
         break;
      }
      else {
         instance_vec[i] = arity_vec[i] - 1;
      }
   }
   };

const auto VECTOR_COUNTER_IS_ALL_ZERO = [](const std::vector<int>& instance_vec) {
   for (const auto& el : instance_vec) {
      if (el != 0) {
         return false;
      }
   }

   return true;
   };

template<class T>
class MapVector { // A special mock implementation of a map which preserves insertion order, using a vector.
public:

   T* at(const std::string& str)
   {
      for (auto& s : groups_) {
         if (s.first == str) return &s.second;
      }

      return nullptr;
   }

   void insert(const std::pair<std::string, T>& el)
   {
      if (!count(el.first)) {
         groups_.push_back(el);
      }
   }

   bool count(const std::string& el_name) const
   {
      return const_cast<MapVector*>(this)->at(el_name);
   }

   void erase(const std::string& el_name)
   {
      for (int i = 0; i < groups_.size(); i++) {
         if (groups_[i].first == el_name) {
            groups_.erase(groups_.begin() + i);
            return;
         }
      }
   }

   std::vector<std::pair<std::string, T>> groups_{};
};

class ThreadPool {
public:
   ThreadPool(int numThreads) : stop(false) 
   {
      for (int i = 0; i < numThreads; ++i) {
         threads.emplace_back([this] {
            while (true) {
               std::function<void()> job;
               {
                  std::unique_lock<std::mutex> lock(queueMutex);
                  condition.wait(lock, [this] { return stop || !jobs.empty(); });
                  if (stop && jobs.empty()) {
                     return;
                  }
                  job = std::move(jobs.front());
                  jobs.pop();
               }
               job();
            }
         });
      }
   }

   ~ThreadPool() 
   {
      {
         std::unique_lock<std::mutex> lock(queueMutex);
         stop = true;
      }
      condition.notify_all();
      for (std::thread& thread : threads) {
         thread.join();
      }
   }

   template <typename F>
   void enqueue(F&& f) 
   {
      {
         std::unique_lock<std::mutex> lock(queueMutex);
         jobs.emplace(std::forward<F>(f));
      }
      condition.notify_one();
   }

private:
   std::vector<std::thread> threads;
   std::queue<std::function<void()>> jobs;
   std::mutex queueMutex;
   std::condition_variable condition;
   bool stop;
};

class StaticHelper
{
public:
   // This function is licensed under CC-BY-SA-4.0 which is a copy-left license
   // (https://creativecommons.org/licenses/by-sa/4.0/deed.en)!
   inline static constexpr unsigned int stringToIntCompiletime(const char* str, int h = 0)
   {
      // From: https://stackoverflow.com/a/16388610
      return !str[h] ? 5381 : (stringToIntCompiletime(str, h + 1) * 33) ^ str[h];
   }

   static int deriveParNum(const std::shared_ptr<Term> formula);

   static std::string firstLetterCapital(const std::string& text);
   static std::string toUpperCamelCase(const std::string& snake_case_text);
   static std::string toUpperCase(const std::string& str);
   static std::string toLowerCase(const std::string& str);

   static std::string removeLastFileExtension(const std::string& file_path, const std::string& extension_denoter = ".");
   static std::string getLastFileExtension(const std::string& file_path, const std::string& extension_denoter = ".");

   static std::string substrComplement(const std::string& str, const int begin, const int end); /// Cuts only the substr part out.

   static std::string privateVarNameForRecursiveFunctions(const std::string& var_name, const int caller_id);

   static bool startsWithUppercase(const std::string& word);
   static bool stringStartsWith(const std::string& string, const std::string& prefix, const int offset = 0);
   static bool stringEndsWith(const std::string& string, const std::string& suffix);
   static bool stringContains(const std::string& string, const std::string& substring);
   static bool stringContains(const std::string& string, const char substring);

   /// <summary>
   /// Returns the number of occurrences of some substring in a string. E.g.:
   /// occurrencesInString("testest", "es") ==> 2.
   /// </summary>
   /// 
   /// <param name="count_overlapping">Iff occurrencesInString("ttt", "tt", ?) returns 2 (or else 1).</param>
   /// <returns></returns>
   static size_t occurrencesInString(const std::string& string, const std::string& sub_string, const bool count_overlapping = false);

   static std::string shortenToMaxSize(const std::string& s, const int max_size, const bool add_dots = true, const int shorten_to = -1, const float percentage_front = 1, const std::string& dots = "[...]");
   static std::string shortenInTheMiddle(const std::string& s, const int max_size, const float percentage_front = 0.5, const std::string& dots = "[...]");

   static std::string absPath(const std::filesystem::path& rel_path, const bool verbose = true);
   static std::string absPath(const std::string& rel_path, const bool verbose = true);

   static std::string cleanVarNameOfPossibleRefSymbol(const std::string& name);
   static std::string cleanVarNameOfPossibleRefSymbol(const MathStructPtr var_formula);

   static bool isFloatInteger(const float val);

   static long long convertTimeStampToMilliseconds(const std::string& timeStamp); // Format: hh:mm:ss.ms

   static std::map<std::string, std::vector<std::string>> readCSV(const std::string& filepath, const std::string& separator);

   /// Returns the first substring of <code>string</code> right from 
   /// <code>beginTagPos</code> starting after a <code>beginTag</code> and ending directly 
   /// before the first position of the matching <code>endTag</code>. 
   /// Tags within the ignore tags are ignored.</BR>
   /// </BR>
   /// Example: "<code>text@{more(Text)}@ (even (more) text)</code>" will yield
   /// "<code>even (more) text</code>" if <code>@{}@</code> are ignored parts and
   /// <code>()</code> denote the begin and end tags.
   /// </BR>
   /// </BR>
   /// Also, "<code>(@{)}@)</code>" will yield "<code>@{)}@</code>", ignoring the
   /// first closing tag.
   /// </BR>
   /// 
   /// @param string         The string to process.
   /// @param beginTag       A std::string denoting the begin of a bracket-like structure.
   /// @param endTag         A std::string denoting the end of a bracket-like structure.
   /// @param ignoreBegTags  List of tags that denote the beginning parts where
   ///                       occurrences of the tags are ignored.
   /// @param ignoreEndTags  List of matching end tags.
   /// @param beginTagPos    The position from which to start searching.
   /// 
   /// @return  The single substring between matching bracket pairs directly right from
   ///          <code>beginTagPos</code>. If there is no such
   ///          region, <code>null</code> is returned.
   static std::shared_ptr<std::string> extractFirstSubstringLevelwise(
      const std::string& string,
      const std::string& beginTag,
      const std::string& endTag,
      const int beginTagPos,
      const std::vector<std::string>& ignoreBegTags = std::vector<std::string>{},
      const std::vector<std::string>& ignoreEndTags = std::vector<std::string>{});

   /// Returns all the substrings of <code>string</code> right from 
   /// <code>beginTagPos</code> starting after a <code>beginTag</code> and ending directly 
   /// before the first position of the matching <code>endTag</code>. 
   /// Tags within the ignore tags are ignored.</BR>
   /// </BR>
   /// Example: "<code>text@{more(Text)}@ (even (more) text)</code>" will yield
   /// "<code>even (more) text</code>" if <code>@{}@</code> are ignored parts and
   /// <code>()</code> denote the begin and end tags.
   /// </BR>
   /// </BR>
   /// Also, "<code>(@{)}@)</code>" will yield "<code>@{)}@</code>", ignoring the
   /// first closing tag.
   /// </BR>
   /// 
   /// @param string         The string to process.
   /// @param beginTag       A std::string denoting the begin of a bracket-like structure.
   /// @param endTag         A std::string denoting the end of a bracket-like structure.
   /// @param ignoreBegTags  List of tags that denote the beginning parts where
   ///                       occurrences of the tags are ignored.
   /// @param ignoreEndTags  List of matching end tags.
   /// @param beginTagPos    The position from which to start searching.
   /// 
   /// @return  A list of all the substrings between matching bracket pairs. If there is no such
   ///          region, an empty list is returned.
   static std::vector<std::string> extractSubstringsLevelwise(
      const std::string& string,
      const std::string& beginTag,
      const std::string& endTag,
      const int beginTagPos = 0,
      const std::vector<std::string>& ignoreBegTags = std::vector<std::string>{},
      const std::vector<std::string>& ignoreEndTags = std::vector<std::string>{});

   static int findMatchingEndTagLevelwise(
      const std::string& string,
      const std::string& beginTag,
      const std::string& endTag,
      const int pos = 0,
      const std::vector<std::string>& ignoreBegTags = std::vector<std::string>{},
      const std::vector<std::string>& ignoreEndTags = std::vector<std::string>{});

   /// Returns the position of the end tag matching the begin tag at the
   /// position given as <code>pos</code>.
   /// 
   /// @param string         The string to process.
   /// @param beginTag       The begin tag to look for.
   /// @param endTag         The end tag to look for.
   /// @param ignoreBegTags  The tags marking beginnings of ignored parts.
   /// @param ignoreEndTags  The tags marking endings of ignored parts.
   /// @param pos            The position of the begin tag.
   /// 
   /// @return  The position of the matching end tag or <code>null</code>.
   static int findMatchingEndTagLevelwise(
      const std::string& string,
      int& pos,
      const std::string& beginTag,
      const std::string& endTag,
      const std::vector<std::string>& ignoreBegTags = std::vector<std::string>{},
      const std::vector<std::string>& ignoreEndTags = std::vector<std::string>{});

   /// Returns the position of the begin tag matching the end tag at the
   /// position given as <code>pos</code>.
   /// 
   /// @param string         The string to process.
   /// @param beginTag       The begin tag to look for.
   /// @param endTag         The end tag to look for.
   /// @param ignoreBegTags  The tags marking beginnings of ignored parts.
   /// @param ignoreEndTags  The tags marking endings of ignored parts.
   /// @param pos            The position of the begin tag.
   /// 
   /// @return  The position of the matching begin tag or <code>null</code>.
   static int findMatchingBegTagLevelwise(
      const std::string& string,
      const std::string& beginTag,
      const std::string& endTag,
      const int pos = 0,
      const std::vector<std::string>& ignoreBegTags = std::vector<std::string>{},
      const std::vector<std::string>& ignoreEndTags = std::vector<std::string>{});

   /// Retrieves the index of the begin of the first innermost bracket part in 
   /// the given string. More precisely, the algorithm looks for the first
   /// closing bracket in the string, and then returns to the matching opening
   /// bracket, returning its position.
   /// 
   /// @param string  The string to process.
   /// @param beg     The begin tag of the bracket-like structure.
   /// @param end     The end tag of the bracket-like structure.
   /// 
   /// @return  The index of the begin tag of the first innermost bracket part.
   ///          Returns {@code -1} if there is no such bracket part.
   static int indexOfFirstInnermostBeginBracket(
      const std::string& string, 
      const std::string& beg, 
      const std::string& end,
      const std::vector<std::string>& ignoreBegTags = std::vector<std::string>{},
      const std::vector<std::string>& ignoreEndTags = std::vector<std::string>{},
      const std::function<bool(const std::string& str, const int index)>& begin_function = TRUE_FUNCTION,
      const std::function<bool(const std::string& str, const int index)>& end_function = TRUE_FUNCTION
   );

   /// Checks for several begin and end tags if the given position 
   /// {@link #isWithinLevelwise(std::string, int, std::string, std::string)} of any of them.
   /// 
   /// @param string     The string to look at.
   /// @param pos        A position in the string.
   /// @param beginTags  A list of begin tags to look for.
   /// @param endTags    A list of end tags to look for.
   /// 
   /// @return  Iff the position is (levelwise) enclosed by at least one 
   ///          of the tag pairs.
   static bool isWithinAnyLevelwise(
      const std::string& string,
      const int pos,
      const std::vector<std::string>& beginTags,
      const std::vector<std::string>& endTags);

   /// This method returns true iff the given position is within enclosing begin
   /// and end tags, by ignoring sub-level tags. Note, however, that the method
   /// does not check the syntax and strictly requires well-formatted strings.
   /// It searches only from left to right for a closing tag, ignoring what
   /// comes to the left.</BR>
   /// </BR>
   /// (()()e()) => e is enclosed.</BR>
   /// (()())e(()) => e is not enclosed.</BR>
   /// (e(()) => e is NOT(!) enclosed (due to malformed string).
   /// 
   /// @param string    The string to look at.
   /// @param pos       A position in the string.
   /// @param beginTag  The begin tag to look for.
   /// @param endTag    The end tag to look for.
   /// 
   /// @return  Iff the position is (levelwise) enclosed by the tags.
   static bool isWithinLevelwise(
      const std::string& string,
      const int pos,
      const std::string& beginTag,
      const std::string& endTag);

   static int indexOfOnTopLevel(
      const std::string& string,
      const std::vector<std::string>& find,
      const int startPos,
      const std::string& beginTag,
      const std::string& endTag,
      const std::vector<std::string>& ignoreBegTags = std::vector<std::string>{},
      const std::vector<std::string>& ignoreEndTags = std::vector<std::string>{});

   /// \brief Takes three strings as input. Only the middle contains a string block that is supposed
   /// to be distributed into a before, an inner and an after part. The ints denote the portion of
   /// the string that divides the inner from the other two parts.
   /// Input:  "", "MyStringTest", "", 2, 8
   /// Output: "My", "String", "Test"
   ///
   /// To distribute some string named inner according to a bracket-like structure using "begin" and "end" as brackets, do:
   ///      std::string before, after;
   ///      int pos_begin = inner.find("begin");
   ///      int pos_end = findMatchingEndTagLevelwise(inner, pos_begin, "begin", "end");
   ///      distributeIntoBeforeInnerAfter(before, inner, after, pos_begin + "begin".size(), pos_end)
   static void distributeIntoBeforeInnerAfter(std::string& before_out, std::string& main_in_out, std::string& after_out, const int begin, const int end);

   static void distributeIntoBeforeInnerAfterFirstInnermostMatching(std::string& before_out, std::string& main_in_out, std::string& after_out, const std::string& begin_bracket, const std::string& end_bracket);
   
   template<class T = std::string>
   static bool count(const std::vector<T> vec, const T& item)
   {
      return std::find(vec.begin(), vec.end(), item) != vec.end();
   }

   static void distributeGetOnlyInner(std::string& main_in_out, const int begin, const int end);
   static void distributeGetOnlyInnerFirstInnermostMatching(std::string& main_in_out, const std::string& begin_bracket, const std::string& end_bracket);

   static bool isRenamedPrivateRecursiveVar(const std::string& var_name);
   static bool isRenamedPrivateVar(const std::string& var_name);
   static bool isPrivateVar(const std::string& var_name);
   static std::string plainVarNameWithoutLocalOrRecursionInfo(const std::string& input);
   static std::string makeString(char c);
   static void ltrim(std::string& s);
   static void rtrim(std::string& s);
   static void trim(std::string& s);
   static std::string ltrimAndReturn(const std::string& s);
   static std::string rtrimAndReturn(const std::string& s);
   static std::string trimAndReturn(const std::string& s);
   static void stripAtBeginning(std::string& str, const std::vector<char> of = {' '});
   static bool isOpeningBracket(const std::string & optor);
   static bool isClosingBracket(const std::string & optor);
   static bool isArgumentDelimiter(const std::string & optor);
   static bool isConstVal(const std::string & el);
   static bool isConstVar(const std::string & el);
   static std::map<int, std::vector<int>> getMappingFromCommandNumToLineNum(const std::string& raw_program);
   static std::string convertPtrAddressToString(const void* ptr);

   static std::vector<std::pair<std::string, std::string>> extractKeyValuePairs(
      const std::string& opt_string,
      const std::string& equal_sign = "=",
      const std::string& delimiter = ","
   );

   static std::string serializeBracketStructure(
      const std::shared_ptr<BracketStructure> bracket_structure, 
      const std::string& opening_bracket = "{",
      const std::string& closing_bracket = "}",
      const std::string& delimiter = ", ");

   static std::shared_ptr<BracketStructure> extractArbitraryBracketStructure(
      const std::string& bracket_structure_as_string, 
      const std::string& opening_bracket,
      const std::string& closing_bracket,
      const std::string& delimiter,
      const std::vector<std::string>& ignoreBegTags = {},
      const std::vector<std::string>& ignoreEndTags = {},
      const bool is_global_call = true);

   enum class SwitchCaseStyle {
      CPP,
      SMV
   };

   static void preprocessCppConvertCStyleArraysToVars(std::string& program);
   static void preprocessCppConvertArraysToCStyle(std::string& program, const std::set<std::string>& possible_array_type_names = DEFAULT_CPP_ARRAY_TYPE_NAMES);
   static void preprocessCppConvertElseIfToPlainElse(std::string& program);
   static void preprocessCppConvertAllSwitchsToIfs(std::string& program);
   static void preprocessSMVConvertAllSwitchsToIfs(std::string& program);
   static std::string convertSingleFlatSwitchToIf(const SwitchBlock& cases, const SwitchCaseStyle style = SwitchCaseStyle::CPP);

   static std::string floatToStringNoTrailingZeros(float value);

   ///
   /// \brief Just a non-member method for converting a number into a string and padding
   ///  it with zeroes at the beginning.
   /// \param num         The number to pad.
   /// \param str_length  The final desired length of the string including padding.
   /// \return  The padded number string.
   ///
   static std::string zeroPaddedNumStr(const long num, const int str_length);

   // Known bug: single quotes are not yet supported. Using '{' or '}' in a switch case will
   // srew up the brace counter.
   // tokens: { ..., "Switch", "(", "n", ")", "{", ..., "}", ... } (Has to start and end before other switch.)
   static SwitchBlock extractCasesFromFlatSwitchBlock(const std::vector<std::string>& switch_block_tokens);
   static std::shared_ptr<std::vector<std::string>> getChunkTokens(const std::string& chunk, const AkaTalProcessing aka_processing);
   static SwitchBlock extractCases(const std::string& switch_block_chunk);

   static SwitchBlock extractCasesFromFlatSwitchBlockSMV(const std::string& switch_var, const std::string& switch_block_chunk);

   static void addStringToken(std::vector<std::string>& tokens, const std::string& token);

   static bool sanityCheckTokensFromCpp(const std::vector<std::string>& tokens);

   /// \brief Returns a new string with all white spaces (space, tab, newline etc.)
   /// removed. The argument string is not changed.
   static std::string removeWhiteSpace(const std::string& string);
   static bool isEmptyExceptWhiteSpaces(const std::string& string);

   static std::string removeMultiLineComments(const std::string& string, const std::string& comment_open = PROGRAM_ML_COMMENT_DENOTER_BEGIN, const std::string& comment_close = PROGRAM_ML_COMMENT_DENOTER_END);
   static std::string removeSingleLineComments(const std::string& string, const std::string& comment_denoter = PROGRAM_COMMENT_DENOTER);
   static std::string removeComments(const std::string& string);

   static std::string removePartsOutsideOf(const std::string& code, const std::string& begin_tag, const std::string& end_tag);

   static std::string removeBlankLines(const std::string& string);

   static int getClassNum(std::string s, std::vector<std::regex> classes, int start_at_class, FormulaParser& parser);
   static void pushBack(std::vector<std::string>& tokens, const std::string& token, int class_num, int ignore_clas, const std::set<std::string>& ignore_tokenss, const std::map<std::string, std::string>& aka_map);

   static std::vector<std::pair<std::string, std::string>> deriveListOfPairsForAkaAnnotatedVarName(const std::string& var_name_with_akas);
   static void adjustTokenWithAkaOrTalInfo(std::string& token, const std::string& current_aka_or_tal);

   /// Contract for proper functioning in reverse mode:
   /// * regex classes are disjunct
   /// * OPENING_QUOTE == CLOSING_QUOTE
   /// * TODO: Opening and closing brackets might be inconsistent...
   static std::shared_ptr<std::vector<std::string>> tokenize(
      const std::string& formula,
      FormulaParser& parser,
      int& start_pos,
      const int max_tokens = std::numeric_limits<int>::max(),
      const bool reverse_direction = false,
      const std::vector<std::regex>& classes = DEFAULT_REGEX_FOR_TOKENIZER, 
      const int ignore_class = DEFAULT_IGNORE_CLASS_FOR_TOKENIZER,
      const bool replace_strings_with_print_commands = true,
      const bool remove_lose_sequence_symbols = true,
      const AkaTalProcessing aka_processing = AkaTalProcessing::none,
      const std::set<std::string>& ignore_tokens = {},
      const bool pass_on_quotes_without_processing = false,
      const std::function<bool(const std::vector<std::string>& tokens_so_far)> abort_function = [](const std::vector<std::string>&) { return false; });

   static std::shared_ptr<std::vector<std::string>> tokenizeSimple(
      const std::string& formula,
      const int max_tokens = std::numeric_limits<int>::max(),
      const bool reverse_direction = false,
      const std::vector<std::regex>& classes = DEFAULT_REGEX_FOR_TOKENIZER,
      const int ignore_class = DEFAULT_IGNORE_CLASS_FOR_TOKENIZER,
      const bool replace_strings_with_print_commands = true,
      const bool remove_lose_sequence_symbols = true,
      const AkaTalProcessing aka_processing = AkaTalProcessing::none); // Note the false here as opposed to the plain tokenize method.

   /// Tokenizes on character level. White spaces can be forced to be unified into ' '. If this option is chosen,
   /// space sequences can additionally be deflated to a single ' '.
   static std::vector<std::string> tokenizeCharacterwise(const std::string& string, const bool unify_white_space = false, const bool deflate_spaces_to_one = false);

   static std::vector<std::string> splitOnWhiteSpaces(const std::string& s);
   static std::vector<std::string> split(const std::string& s, char delim);
   template<typename Out>
   static void split(const std::string &s, char delim, Out result);
   static std::vector<std::string> split(const std::string& s, const std::string& delim, SplitCondition& f = FUNC_ALL_STRINGS_TO_TRUE, const bool keep_delimiter = false);
   static std::vector<std::string> split(const std::vector<std::string>& ss, const std::string& delim, SplitCondition& f = FUNC_ALL_STRINGS_TO_TRUE, const bool keep_delimiter = false);
   
   static std::string wrapOnTokens(const std::string& text_raw, const std::set<std::string>& wrap_before, const std::set<std::string>& wrap_after);
   static std::string wrapOnLineLength(const std::string& text_raw, const size_t max_line_length);
   static std::string wrap(const std::string& text_raw, const size_t max_line_length, const std::set<std::string>& additionally_wrap_before = {}, const std::set<std::string>& additionally_wrap_after = {});
   static std::string replaceAll(std::string str, const std::string& from, const std::string& to);
   static std::string replaceAllCounting(std::string str, const std::string& from, const int start_at = 0); /// A $$$ $$$ $$$ ==> A 001 002 003
   static std::string replaceAllRegex(const std::string& str, const std::string& from_regex, const std::string& to_regex);

   static bool isParsableAsFloat(const std::string& float_string);
   static bool isParsableAsInt(const std::string& int_string, const bool allow_empty = false);
   static bool isAlpha(const std::string& str);
   static bool isAlphaNumeric(const std::string& str);
   static bool isAlphaNumericOrUnderscore(const std::string& str);
   static bool isAlphaNumericOrUnderscoreOrColonOrComma(const std::string& str);
   static bool isAlphaNumericOrOneOfThese(const std::string& str, const std::string& additionally_allowed);
   static bool isValidVfmVarName(const std::string& str);
   static bool stringConformsTo(const std::string& str, const std::regex& regex);
   static std::string incrementAlphabeticNum(const std::string& num);

   static bool compareTransitionsHighestDestinationID(
      std::shared_ptr<fsm::FSMTransition> a, std::shared_ptr<fsm::FSMTransition> b)
   {
      return a->state_destination_ > b->state_destination_;
   }

   static bool isAssociative(const MathStructPtr formula, const std::string& infix_operator, const AssociativityTypeEnum type);
   
   // Super-fast, even faster than the same functionality in scope of simplifyFast.
   static TermPtr makeAssociative(const TermPtr formula, const std::string& infix_operator, const AssociativityTypeEnum type);

   static bool compareTransitionsHighestWeight(
      std::shared_ptr<fsm::FSMTransition> a, std::shared_ptr<fsm::FSMTransition> b)
   {
      float val1 = std::fabs(a->condition_->eval(a->data_));
      float val2 = std::fabs(b->condition_->eval(b->data_));
      return val1 > val2;
   }

   static bool isBooleanTrue(const std::string& bool_str);

   static std::string getFileNameFromPath(const std::string& path);
   static std::string getFileNameFromPathWithoutExt(const std::string& path);
   static std::string removeFileNameFromPath(const std::string& path);

   enum class TraceExtractionMode {
      regular,
      quick_only_detect_if_empty
   };


   // Function to extract an integer after a specific substring, like so:
   // f("this_is_my_string_12345_and_more", "my_string") ==> 12345
   static int extractIntegerAfterSubstring(const std::string& str, const std::string& substring);

   static std::vector<MCTrace> extractMCTracesFromMSATIC(const std::string& cexp_string); // For now only one CEX is extracted. Empty CEX returned as empty list.
   static std::vector<MCTrace> extractMCTracesFromMSATICFile(const std::string& cexp_string);

   static std::vector<MCTrace> extractMCTracesFromNusmv(const std::string& cexp_string, const TraceExtractionMode mode = TraceExtractionMode::regular);
   static std::vector<MCTrace> extractMCTracesFromNusmvFile(const std::string& path, const TraceExtractionMode mode = TraceExtractionMode::regular);
   static std::string serializeMCTraceNusmvStyle(const MCTrace& trace, const bool print_unchanged_values = false);

   static std::string readFile(const std::filesystem::path& path, const bool from_utf16 = false);

   static std::vector<MCTrace> extractMCTracesFromKratos(const std::string& cexp_string); // For now only one CEX is extracted. Empty CEX returned as empty list.
   static std::vector<MCTrace> extractMCTracesFromKratosFile(const std::string& path, const bool from_utf16 = false);
   static std::string serializeMCTraceKratosStyle(const MCTrace& trace);
   static std::string extractSeries(const MCTrace& trace, const std::vector<std::string>& variables);

   static std::string timeStamp();

   static std::string fromPascalToCamelCase(const std::string& line);
   static std::string replaceSpecialCharsHTML_G(const std::string& sString);
   static std::string replaceMathHTML(const std::string& sString);

   static int greatestCommonDivisor(const int a, const int b);
   static int leastCommonMultiple(const int a, const int b);
   static int greatestCommonDivisor(const std::vector<int>& vec);
   static int leastCommonMultiple(const std::vector<int>& vec);

   static bool isVariableNameDenotingArray(const std::string& var_name);
   static std::string makeVarNameNuSMVReady(const std::string& var_name);

   static bool isTermAssemblifiable(const std::string& optor, const std::shared_ptr<FormulaParser> parser_raw = nullptr);
   static void listAssemblyStateOfAllTerms(const std::shared_ptr<FormulaParser> parser_raw = nullptr);

   static std::string safeString(const std::string& arbitrary_string);
   static std::string fromSafeString(const std::string& safe_string);

   static std::string tokensAsString(const std::vector<std::string>& tokens, const bool remove_trailing_delimiter = false, const std::string& delimiter = " ");

   static size_t levensteinDistance(const std::string& a, const std::string& b);
   static std::tuple<size_t, size_t> findClosest(const std::vector<std::string>& strs, const std::string& query);

   static std::string replaceManyTimes(
      const std::string& str,
      const std::vector<std::string>& search_list,
      const std::vector<std::string>& replacements_list
   );

   static std::string replaceManyTimes(
      const std::string& str, 
      const std::vector<std::pair<std::string, std::string>>& replacements = { 
         { "++", "_increment_" },
         { "+", "_plus_" },
         { "-", "_minus_" },
         { "**", "_pow_" },
         { "*", "_mult_" },
         { "/", "_div_" },
         { "@", "_at_" },
         { ".", "_dot_" },
         { ",", "_comma_" },
         { "::", "_namespace_" },
         { ":", "_colon_" },
         { ";", "_semicolon_" },
         { "'", "_tick_" },
         { "==", "_equal_" },
         { "!=", "_notequal_" },
         { ">=", "_greaterorequal_" },
         { "<=", "_smallerorequal_" },
         { "&&", "_and_" },
         { "||", "_or_" },
         { ">", "_greater_" },
         { "<", "_smaller_" },
         { "!", "_not_" },
         { "%", "_modulo_" },
         { "&", "_ampersand_" },
         { "|", "_pipe_" },
         { "=", "_equal_sign_" },
         { "a_", "_anyway_operation_" },
      });

   // Print a self-overwriting progress bar like: PREFIX [XXXXX-----------------] progress_step/total_steps
   static std::string printProgress(const std::string& prefix, int progress_step, int total_steps = 100, int letters_length = 30);

   static std::string printTimeFormatted(const std::chrono::nanoseconds& parsing_time);

   static void checkForOutdatedSimplification(const std::shared_ptr<FormulaParser> parser);

   static bool isFileLockedWindows(const std::string& filename_raw); // Does not work on Unix.

   static int createImageFromGraphvizDotFile(const std::string& base_filename, const std::string& format = "pdf");
   static int createImageFromGraphvizDot(const std::string& graphviz, const std::string& target_filename, const std::string& format = "pdf");
   
   static int executeSystemCommand(const std::string& command);

   /// Execute the given cmd on system level, and return the output generated by it.
   static std::string exec(const std::string cmd, const bool verbose = false); // Deprecated, use the below.
   
   /// Execute the given cmd on system level, and return the output generated by it as well as exit code.
   static std::string execWithSuccessCode(const std::string& cmd, const bool verbose, int& exit_code, std::shared_ptr<std::string> path_to_store_meta_info);

   static std::string createKratosNdetFunction(const std::string& name, const std::vector<std::string> options);

   static void writeTextToFile(const std::string& text, const std::filesystem::path& path, const bool append = false);
   static void fakeCallWithCommandLineArguments(const std::string& argv_combined, std::function<void(int argc, char* argv[])> to_do);
   static void addNDetDummyFunctions(const std::shared_ptr<FormulaParser> parser, const std::shared_ptr<DataPack> data);

   static bool existsFileSafe(const std::string& filepath, const bool quiet = true);
   static bool existsFileSafe(const std::filesystem::path& filepath, const bool quiet = true);
   static bool removeFileSafe(const std::string& filepath, const bool quiet = true);
   static bool removeFileSafe(const std::filesystem::path& filepath, const bool quiet = true);
   static bool removeAllFilesSafe(const std::string& filepath, const bool quiet = true);
   static bool removeAllFilesSafe(const std::filesystem::path& filepath, const bool quiet = true);
   static bool createDirectoriesSafe(const std::string& filepath, const bool quiet = true);
   static bool createDirectoriesSafe(const std::filesystem::path& filepath, const bool quiet = true);
   static std::filesystem::file_time_type lastWritetimeOfFileSafe(const std::string& filepath, const bool quiet = true);
   static std::filesystem::file_time_type lastWritetimeOfFileSafe(const std::filesystem::path& filepath, const bool quiet = true);

   static void openWithOS(const std::string& path_raw, const Failable* logger = nullptr);

   static void applyTimescaling(MCTrace& trace, const ScaleDescription& ts_description);
   
   static std::pair<double, double> cartesianToWGS84(const double x, const double y);

   static std::vector<std::string> getSafeCharactersAsciiLike();
   static std::map<std::string, int> getSafeCharactersAsciiLikeReverse();

   StaticHelper() = delete;

private:
   /// Does the actual work for {@link #extractSubstringsLevelwise(std::string, std::string, std::string, LinkedList, LinkedList, int)}.
   /// 
   /// @param string               The string to process.
   /// @param beginTag             A std::string denoting the begin of a bracket-like structure.
   /// @param endTag               A std::string denoting the end of a bracket-like structure.
   /// @param ignoreBegTags        List of tags that denote the beginning parts where
   ///                             occurrences of the tags are ignored.
   /// @param ignoreEndTags        List of matching end tags.
   /// @param beginTagPos          The position from which to start searching.
   /// @param soFar                An initially empty list of substrings that is finally returned.
   /// @param stopAfterFirstMatch  Iff only the first matching substring is supposed to be returned.
   ///  
   /// @return  A list of all the substrings between matching bracket pairs. If there is no such
   ///          region, an empty list is returned.
   static std::vector<std::string> extractSubstringsLevelwise(
      const std::string& string,
      const std::string& beginTag,
      const std::string& endTag,
      const std::vector<std::string>& ignoreBegTags,
      const std::vector<std::string>& ignoreEndTags,
      const int beginTagPos,
      const bool stopAfterFirstMatch,
      std::vector<std::string>& soFar);
   
   static std::string checkForWrapInTokenMode(const std::set<std::string>& wrap_on, std::deque<char>& latest, const std::string& text, const char current_char, const int max_len);

   static int accumulate(const std::vector<int> vec, const std::function<int(int, int)>& function); // Used for gcd and lcm.

   static void fillSafeCharVecs();
};

class VariableScaleDescription : public Parsable
{
public:
   inline VariableScaleDescription() : Parsable("VariableTimescaleDescription") {}
   inline VariableScaleDescription(const std::string program) : VariableScaleDescription()
   {
      parseProgram(program);
   }

   inline std::string serialize() const
   {
      return "" + variable_name_ + "," + type_.getEnumAsString() + "";
   }

   inline bool parseProgram(const std::string& program) override
   {
      auto split{ StaticHelper::split(program, ",") };

      if (split.size() != 2) {
         addError("String '" + program + "' is not a valid VariableTimescaleDescription.");
         return false;
      }

      variable_name_ = split[0];
      type_ = ScaleType(split[1]);

      return true;
   }

   std::string variable_name_{};
   ScaleType type_{};
};

class ScaleDescription : public Parsable
{
public:
   inline ScaleDescription() : Parsable("ScaleDescription") {};

   inline ScaleDescription(const std::string& envmodel_str) : Parsable("ScaleDescription") {
      parseProgram(envmodel_str);
   };

   inline void setTimeScalingFactor(const float scaling_factor)
   {
      time_scaling_factor_ = scaling_factor;
   }

   inline void setDistanceScalingFactor(const float scaling_factor)
   {
      distance_scaling_factor_ = scaling_factor;
   }

   inline void add(const VariableScaleDescription& variable)
   {
      variables_.push_back(variable);
   }

   inline float getTimeScalingFactor() const
   {
      return time_scaling_factor_;
   }

   inline float getDistanceScalingFactor() const
   {
      return distance_scaling_factor_;
   }

   inline std::vector<VariableScaleDescription> getVariables() const
   {
      return variables_;
   }

   inline std::string serialize()
   {
      std::string res{};

      if (time_scaling_factor_ >= 0) {
         res += "-- TIMESCALING((((" + std::to_string(time_scaling_factor_) + "))))GNILACSEMIT\n";
         res += "-- DISTANCESCALING((((" + std::to_string(distance_scaling_factor_) + "))))GNILACSECNATSID\n";
      }

      res += BEGIN_TAG_TIMESCALING_DESCRIPTIONS + "\n";

      for (const auto& ts : variables_) {
         res += "-- " + ts.serialize() + "\n";
      }

      res += END_TAG_TIMESCALING_DESCRIPTIONS + "\n";

      return res;
   }

   inline bool parseProgram(const std::string& envmodel_str) override
   {
      auto relevant{ StaticHelper::removePartsOutsideOf(envmodel_str, BEGIN_TAG_TIMESCALING_DESCRIPTIONS, END_TAG_TIMESCALING_DESCRIPTIONS) };

      variables_.clear();
      for (const auto& line_raw : StaticHelper::split(relevant, "\n")) {
         auto line{ StaticHelper::replaceAll(line_raw, "--", "") };
         StaticHelper::trim(line);
         if (!StaticHelper::isEmptyExceptWhiteSpaces(line)) {
            variables_.push_back(VariableScaleDescription(line));
         }
      }

      time_scaling_factor_ = 1;
      distance_scaling_factor_ = 1;

      if (!envmodel_str.empty()) {
         time_scaling_factor_ = std::stof(StaticHelper::removePartsOutsideOf(envmodel_str, "TIMESCALING((((", "))))GNILACSEMIT"));
         distance_scaling_factor_ = std::stof(StaticHelper::removePartsOutsideOf(envmodel_str, "DISTANCESCALING((((", "))))GNILACSECNATSID"));
      }

      return true;
   }

   inline static bool createTimescalingFile(const std::string& generated_dir)
   {
      const std::string TIMESCALING_FILE{ generated_dir + TIMESCALING_FILENAME };

      if (std::filesystem::exists(TIMESCALING_FILE)) {
         Failable::getSingleton()->addNote("Found timescaling file '" + TIMESCALING_FILE + "'. (Assuming it has been generated earlier, typically during MC run which is the most accurate time to do so; omitting re-creation.)");
         return false;
      }
      else {
         Failable::getSingleton()->addNote("Creating timescaling file in '" + TIMESCALING_FILE + "'.");
      }

      const std::string ENVMODEL_PREFIX{ "EnvModel" };
      const std::string BEGIN_TAG_ENVMODEL{ "#include \"" + ENVMODEL_PREFIX };
      const std::string END_TAG_ENVMODEL{ "\"" };

      if (!std::filesystem::exists(generated_dir + "main.smv")) {
         Failable::getSingleton()->addWarning("No timescaling to create. File '" + generated_dir + "main.smv" + "' not found. Assuming no-EM mode and omitting timescaling.");
         return false;
      }

      const std::string env_model_filename{ ENVMODEL_PREFIX + StaticHelper::removePartsOutsideOf(StaticHelper::readFile(generated_dir + "main.smv"), BEGIN_TAG_ENVMODEL, END_TAG_ENVMODEL) };

      Failable::getSingleton()->addNote("Retrieved env model file path '" + generated_dir + env_model_filename + "' from " + generated_dir + "main.smv.");

      ScaleDescription desc{};
      desc.parseProgram(StaticHelper::readFile(generated_dir + env_model_filename));

      const std::string timescale_info{ desc.serialize() };

      Failable::getSingleton()->addNote("Timescaling information found in this file:\n" + timescale_info);
      Failable::getSingleton()->addNote("Storing timescaling information in '" + TIMESCALING_FILE + "'.");
      StaticHelper::writeTextToFile(timescale_info, TIMESCALING_FILE);

      return true;
   }

private:
   float time_scaling_factor_{};
   float distance_scaling_factor_{};
   std::vector<VariableScaleDescription> variables_{};
};

} // vfm

template <typename T>
std::ostream& operator<< (std::ostream& out, const std::vector<T>& v) {
   if (!v.empty()) {
      out << '[';
      std::copy(v.begin(), v.end(), std::ostream_iterator<T>(out, ", "));
      out << "\b\b]";
   }
   return out;
}

template <typename T>
std::ostream& operator<< (std::ostream& out, const std::set<T>& v) {
   if (!v.empty()) {
      out << '[';
      std::copy(v.begin(), v.end(), std::ostream_iterator<T>(out, ", "));
      out << "\b\b]";
   }
   return out;
}

template<typename S>
auto selectRandom(const S& s) {
   auto r = rand() % s.size(); // not _really_ random
   auto it = std::begin(s);
   std::advance(it, r);
   return it;
}
