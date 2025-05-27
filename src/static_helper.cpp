//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file

#include "static_helper.h"
#include "parser.h"
#include "failable.h"
#include "meta_rule.h"
#include "model_checking/msatic_parsing/msatic_trace.h"
#include "simplification/code_block.h"
#include "testing/interactive_testing.h"
#include <array>
#include <cassert>
#include <cctype>
#include <codecvt>
#include <exception>
#include <fstream>
#include <iomanip>
#include <map>
#include <regex>
#include <sstream>
#include <stack>

#if defined(ASMJIT_ENABLED)
#include "asmjit.h"
#endif

using namespace vfm;
using namespace simplification;


static std::vector<std::string>& getSAFE_CHARACTERS_ASCII_LIKE() {
   static auto SAFE_CHARACTERS_ASCII_LIKE = std::vector<std::string>();
   return SAFE_CHARACTERS_ASCII_LIKE;
}

static std::map<std::string, int>& getSAFE_CHARACTERS_ASCII_LIKE_REVERSE() {
   static auto SAFE_CHARACTERS_ASCII_LIKE_REVERSE = std::map<std::string, int>();
   return SAFE_CHARACTERS_ASCII_LIKE_REVERSE;
}

std::string StaticHelper::convertPtrAddressToString(const void* ptr) {
    std::stringstream ss;
    ss << ptr;  
    return ss.str(); 
}

std::vector<std::pair<std::string, std::string>> vfm::StaticHelper::extractKeyValuePairs(
   const std::string& opt_string, const std::string& equal_sign, const std::string& delimiter)
{
   std::vector<std::pair<std::string, std::string>> vec;

   for (const auto& chunk : StaticHelper::split(opt_string, delimiter)) {
      const auto pair = StaticHelper::split(chunk, equal_sign);
      std::string key = pair[0];
      std::string value = pair[1];

      StaticHelper::trim(key);
      StaticHelper::trim(value);

      vec.push_back({ key, value });
   }
   
   return vec;
}

std::map<int, std::vector<int>> StaticHelper::getMappingFromCommandNumToLineNum(const std::string& raw_program)
{
   char line_break_symb = '\n';
   int line_num = 1;
   int command_num = 1;
   std::vector<int> current_command_lines;
   std::map<int, std::vector<int>> mapping;
   bool non_white_space = false;

   for (int i = 0; i < raw_program.size(); i++) {
      char symb = raw_program[i];

      if (!non_white_space && !StaticHelper::removeWhiteSpace(std::string(1, symb)).empty()) {
          non_white_space = true;
      }

      if (symb == line_break_symb) {
          if (non_white_space) {
             current_command_lines.push_back(line_num);
          }
          line_num++;
      }

      if (symb == PROGRAM_COMMAND_SEPARATOR) {
          current_command_lines.push_back(line_num);
          mapping.insert(std::pair<int, std::vector<int>>(command_num, current_command_lines));
          current_command_lines.clear();
          command_num++;
          non_white_space = false;
      }
   }

   return mapping;
}

int StaticHelper::deriveParNum(const std::shared_ptr<Term> formula)
{
   std::shared_ptr<int> par_num = std::make_shared<int>(-1);

   // Number of expected parameters, is given by inspecting all the metas.
   formula->applyToMeAndMyChildren([par_num](std::shared_ptr<MathStruct> m) {
      if (m->isMetaCompound()) {
         *par_num = std::max(*par_num, (int)(m->getOperands()[0]->constEval()));
      }
   });

   (*par_num)++;

   return *par_num;
}

std::string StaticHelper::firstLetterCapital(const std::string& text)
{
   std::string newstr = text;
   newstr[0] = std::toupper(newstr[0]);
   return newstr;
}

std::string StaticHelper::toUpperCamelCase(const std::string& snake_case_text)
{
   const std::string snake_case_delimiter = "_";
   std::vector<std::string> parts = split(snake_case_text, snake_case_delimiter);
   std:: string result;

   for (std::string s : parts) {
      result += firstLetterCapital(s);
   }

   return result;
}

std::string StaticHelper::toUpperCase(const std::string& str)
{
   std::string s = str;

   for (auto& c : s) {
      c = toupper(c);
   }

   return s;
}

std::string StaticHelper::toLowerCase(const std::string& str)
{
   std::string s = str;

   for (auto& c : s) {
      c = tolower(c);
   }

   return s;
}

std::string vfm::StaticHelper::removeLastFileExtension(const std::string& file_path, const std::string& extension_denoter)
{
   size_t lastindex = file_path.find_last_of(extension_denoter);
   std::string rawname = lastindex == std::string::npos ? file_path : file_path.substr(0, lastindex); 
   return rawname;
}

std::string vfm::StaticHelper::getLastFileExtension(const std::string& file_path, const std::string& extension_denoter)
{
   size_t lastindex = file_path.find_last_of(extension_denoter);
   std::string rawname = lastindex == std::string::npos ? "" : file_path.substr(lastindex); 
   return rawname;
}

std::string vfm::StaticHelper::substrComplement(const std::string& str, const int begin, const int end)
{
   return str.substr(0, begin) + str.substr(end + begin);
}

std::string StaticHelper::removeWhiteSpace(const std::string& string)
{
   std::string str = string;
   str.erase(std::remove_if(str.begin(),
                            str.end(),
                            [](unsigned char x) {return std::isspace(x); }), 
             str.end());
   return str;
}

bool StaticHelper::isEmptyExceptWhiteSpaces(const std::string& string)
{
   return removeWhiteSpace(string).empty();
}

std::string vfm::StaticHelper::removeMultiLineComments(const std::string& string, const std::string& comment_open, const std::string& comment_close)
{
   int beg = indexOfFirstInnermostBeginBracket(string, comment_open, comment_close);

   if (beg >= 0) {
      int end = findMatchingEndTagLevelwise(string, comment_open, comment_close, beg);
      std::string comment_part = string.substr(beg, end - beg + comment_close.size());

      return removeMultiLineComments(StaticHelper::replaceAll(string, comment_part, ""), comment_open, comment_close);
   }

   return string;
}

std::string vfm::StaticHelper::removeSingleLineComments(const std::string& string, const std::string& comment_denoter)
{
   const auto lines = StaticHelper::split(string, "\n");
   std::string result;

   for (const auto& line : lines) {
      const auto comment_pos = line.find(comment_denoter);
      std::string line_cropped = comment_pos <= line.size() ? line.substr(0, comment_pos) : line;
      result += line_cropped + "\r\n";
   }

   return result;
}

std::string vfm::StaticHelper::removeComments(const std::string& string)
{
   return removeSingleLineComments(removeMultiLineComments(string));
}

std::string vfm::StaticHelper::removePartsOutsideOf(const std::string& code, const std::string& begin_tag, const std::string& end_tag)
{
   std::string code_old = code;
   std::shared_ptr<std::string> ptr = StaticHelper::extractFirstSubstringLevelwise(code_old, begin_tag, end_tag, 0);

   if (ptr) {
      return *ptr + removePartsOutsideOf(StaticHelper::replaceAll(code, begin_tag + *ptr + end_tag, ""), begin_tag, end_tag);
   }

   return "";
}

std::string vfm::StaticHelper::removeBlankLines(const std::string& string)
{
   std::string s;
   auto vec = split(string, '\n');

   for (const auto& l : vec) {
      if (!isEmptyExceptWhiteSpaces(l)) {
         s += l + "\n";
      }
   }

   return s;
}

std::string StaticHelper::privateVarNameForRecursiveFunctions(const std::string& var_name, const int caller_id)
{
   if (caller_id && StaticHelper::isRenamedPrivateVar(var_name) && !StaticHelper::isRenamedPrivateRecursiveVar(var_name))
   {
      return var_name + PRIVATE_VARIABLES_IN_RECURSIVE_CALLS_INTERNAL_SYMBOL + std::to_string(caller_id);
   }

   return var_name;
}

bool StaticHelper::startsWithUppercase(const std::string& word)
{
   return word.size() && std::isupper(static_cast<unsigned char>(word[0]));
}

bool StaticHelper::stringStartsWith(const std::string& string, const std::string& prefix, const int offset)
{
   if (offset != 0) {
      return !string.compare(offset, prefix.size(), prefix);
   }

   return string.rfind(prefix, 0) == 0;
}

bool StaticHelper::stringEndsWith(const std::string& string, const std::string& suffix)
{
   int index = string.rfind(suffix);
   return index >= 0 && index == (string.size() - suffix.size());
}

bool vfm::StaticHelper::stringContains(const std::string& string, const char substring)
{
   return stringContains(string, std::string(1, substring));
}

bool vfm::StaticHelper::stringContains(const std::string& string, const std::string& substring)
{
   return string.find(substring) != std::string::npos;
}

size_t vfm::StaticHelper::occurrencesInString(const std::string& string, const std::string& sub_string, const bool count_overlapping)
{
   size_t occurrences{ 0 };
   size_t pos{ 0 };
   while ((pos = string.find(sub_string, pos)) != std::string::npos) {
      ++occurrences;
      pos += count_overlapping ? 1 : sub_string.length();
   }

   return occurrences;
}

std::string vfm::StaticHelper::shortenToMaxSize(const std::string& s, const int max_size, const bool add_dots, const int shorten_to, const float percentage_front, const std::string& dots)
{
   int st = shorten_to < 0 ? max_size : shorten_to;
   
   if (s.size() > max_size) {
      const float size{ (float) std::min(st, max_size) };
      const float front{ size * percentage_front };
      const float back{ size * (1.0f - percentage_front) };

      return s.substr(0, front) + (add_dots ? dots : "") + s.substr(s.size() - back, back + 1);
   }

   return s;
}

std::string vfm::StaticHelper::shortenInTheMiddle(const std::string& s, const int max_size, const float percentage_front, const std::string& dots)
{
   return shortenToMaxSize(s, max_size, true, -1, percentage_front, dots);
}

std::string vfm::StaticHelper::absPath(const std::string& rel_path, const bool verbose)
{
   static const std::string FAILABLE_NAME = "Compiler-Version";

   if (verbose) {
      vfm::Failable::getSingleton(FAILABLE_NAME)->setOutputLevels(ErrorLevelEnum::debug);
   }

#if __cplusplus >= 201703L // https://stackoverflow.com/a/51536462/7302562 and https://stackoverflow.com/a/60052191/7302562
   std::filesystem::path p(rel_path);
   std::string abs_path = std::filesystem::weakly_canonical(p).string();

   if (verbose && !std::filesystem::exists(rel_path)) {
      vfm::Failable::getSingleton(FAILABLE_NAME)->addDebug("Retrieving absolute path for non-existing relative path:\n\t" + rel_path + " ===>\n\t" + abs_path + "\nPlease check if this is correct.");
   }

   return abs_path;
#else
   vfm::Failable::getSingleton(FAILABLE_NAME)->addWarning(
      "Could not retrieve absolute path of '" + rel_path + "' due to wrong C++ compiler. Returning relative path.");
   return rel_path;
#endif
}

std::string vfm::StaticHelper::cleanVarNameOfPossibleRefSymbol(const std::string& name)
{
   if (StaticHelper::stringStartsWith(name, SYMB_REF)) {
      return name.substr(SYMB_REF.size());
   }

   return name;
}

std::string vfm::StaticHelper::cleanVarNameOfPossibleRefSymbol(const MathStructPtr var_formula)
{
   auto m_var = var_formula->toVariableIfApplicable();
   return m_var ? cleanVarNameOfPossibleRefSymbol(m_var->getVariableName()) : "";
}

// This function is licensed under CC-BY-SA-4.0 which is a copy-left license
// (https://creativecommons.org/licenses/by-sa/4.0/deed.en)!
bool vfm::StaticHelper::isFloatInteger(const float val)
{
   return ::ceilf(val) == val; // From https://stackoverflow.com/a/5797186
}

long long vfm::StaticHelper::convertTimeStampToMilliseconds(const std::string& timeStamp) {
   int hours, minutes, seconds, milliseconds;
   char delimiter;

   std::istringstream iss(timeStamp);
   iss >> hours >> delimiter >> minutes >> delimiter >> seconds >> delimiter >> milliseconds;

   long long totalMilliseconds = (hours * 3600000) + (minutes * 60000) + (seconds * 1000) + milliseconds;

   return totalMilliseconds;
}

std::map<std::string, std::vector<std::string>> vfm::StaticHelper::readCSV(const std::string& filepath, const std::string& separator)
{
   auto file_str = StaticHelper::readFile(filepath);
   auto lines = vfm::StaticHelper::split(file_str, "\n");
   std::map<std::string, std::vector<std::string>> csv_data{};
   std::map<int, std::string> titles{};
   bool first{ true };
   int i;

   for (const auto& line : lines) {
      if (!line.empty()) {
         i = 0;
         for (const auto& item : StaticHelper::split(line, separator)) {
            if (first) {
               csv_data.insert({ item, {} });
               titles.insert({ i++, item });
            }
            else {
               csv_data.at(titles[i++]).push_back(item);
            }
         }

         first = false;
      }
   }

   return csv_data;
}

std::shared_ptr<std::string> StaticHelper::extractFirstSubstringLevelwise(
   const std::string& string, 
   const std::string& beginTag, 
   const std::string& endTag,
   const int beginTagPos, 
   const std::vector<std::string>& ignoreBegTags,
   const std::vector<std::string>& ignoreEndTags) {
   std::vector<std::string> soFar;

   std::vector<std::string> result = extractSubstringsLevelwise(
      string, 
      beginTag, 
      endTag, 
      ignoreBegTags, 
      ignoreEndTags, 
      beginTagPos, 
      true, 
      soFar);

   if (result.empty()) {
      return nullptr;
   }
   
   return std::make_shared<std::string>(result.front());
}

std::vector<std::string> StaticHelper::extractSubstringsLevelwise(
   const std::string& string, 
   const std::string& beginTag, 
   const std::string& endTag,
   const int beginTagPos, 
   const std::vector<std::string>& ignoreBegTags,
   const std::vector<std::string>& ignoreEndTags) {
   std::vector<std::string> soFar;

   return extractSubstringsLevelwise(
      string, 
      beginTag, 
      endTag, 
      ignoreBegTags, 
      ignoreEndTags, 
      beginTagPos, 
      false, 
      soFar);
}

std::vector<std::string> StaticHelper::extractSubstringsLevelwise(
   const std::string& string, 
   const std::string& beginTag, 
   const std::string& endTag, 
   const std::vector<std::string>& ignoreBegTags,
   const std::vector<std::string>& ignoreEndTags,
   const int beginTagPos,
   const bool stopAfterFirstMatch,
   std::vector<std::string>& soFar) {
   int curBegin = string.find(beginTag, beginTagPos);

   /* 
    * Find first non-ignored begin tag. This will be the start point of 
    * the string extracted by this method call. The end will be the first
    * non-ignored matching end tag.
    */
   while (curBegin >= 0
      && isWithinAnyLevelwise(string, curBegin, ignoreBegTags, ignoreEndTags)) {
      curBegin = string.find(beginTag, curBegin + 1);
   }

   if (curBegin < 0) {
      return soFar;
   }

   int endPos = findMatchingEndTagLevelwise(string, beginTag, endTag, curBegin,
      ignoreBegTags, ignoreEndTags);

   if (endPos >= 0) {
      if (!stopAfterFirstMatch) { // Go on recursively with rest of string.
         extractSubstringsLevelwise(string.substr(endPos + endTag.length()), beginTag, endTag, ignoreBegTags, ignoreEndTags, 0, stopAfterFirstMatch, soFar);
      }

      auto innerPartBegin = curBegin + beginTag.length();
      soFar.push_back(string.substr(innerPartBegin, endPos - innerPartBegin));
   }

   return soFar;
}

int vfm::StaticHelper::findMatchingEndTagLevelwise(const std::string& string, const std::string& beginTag, const std::string& endTag, const int pos, const std::vector<std::string>& ignoreBegTags, const std::vector<std::string>& ignoreEndTags)
{
   int dummy = pos;
   return findMatchingEndTagLevelwise(string, dummy, beginTag, endTag, ignoreBegTags, ignoreEndTags);
}

int StaticHelper::findMatchingEndTagLevelwise(
   const std::string& string,
   int& pos,
   const std::string& beginTag, 
   const std::string& endTag,
   const std::vector<std::string>& ignoreBegTags,
   const std::vector<std::string>& ignoreEndTags) {
   while (pos < string.size() && !stringStartsWith(string, beginTag, pos)) pos++;

   int count = 0; // Because we start on a begin tag.
   int next_inc = 1;

   for (int i = pos; i < string.length(); i += next_inc) {
      next_inc = 1;

      if (stringStartsWith(string, beginTag, i)
         && !isWithinAnyLevelwise(string, i, ignoreBegTags, ignoreEndTags)) {
         count++;
         next_inc = beginTag.size();
      }

      if (stringStartsWith(string, endTag, i)
         && !isWithinAnyLevelwise(string, i, ignoreBegTags, ignoreEndTags)) {
         count--;
         next_inc = endTag.size();
      }

      if (count == 0) {
         return i;
      }
   }

   return -1;
}

int StaticHelper::findMatchingBegTagLevelwise(
   const std::string& string,
   const std::string& beginTag, 
   const std::string& endTag,
   const int pos,
   const std::vector<std::string>& ignoreBegTags,
   const std::vector<std::string>& ignoreEndTags) {
   int count = 0;               // Because we start on an end tag.

   for (int i = pos; i >= 0; i--) {
      if (stringStartsWith(string, beginTag, i)
         && !isWithinAnyLevelwise(string, i, ignoreBegTags, ignoreEndTags)) {
         count++;
      }

      if (stringStartsWith(string, endTag, i)
         && !isWithinAnyLevelwise(string, i, ignoreBegTags, ignoreEndTags)) {
         count--;
      }

      if (count == 0) {
         return i;
      }
   }

   return -1;
}

int StaticHelper::indexOfFirstInnermostBeginBracket(
   const std::string& string, 
   const std::string& beg, 
   const std::string& end,
   const std::vector<std::string>& ignoreBegTags,
   const std::vector<std::string>& ignoreEndTags,
   const std::function<bool(const std::string& str, const int index)>& begin_function,
   const std::function<bool(const std::string& str, const int index)>& end_function) {
   int firstBeg = string.find(beg);
   int firstEnd = string.find(end, firstBeg);

   while (isWithinAnyLevelwise(string, firstBeg, ignoreBegTags, ignoreEndTags)) {
      firstBeg = string.find(beg, firstBeg + 1);
   }

   while (isWithinAnyLevelwise(string, firstEnd, ignoreBegTags, ignoreEndTags)) {
      firstEnd = string.find(end, firstEnd + 1);
   }

   if (firstBeg < 0 || firstEnd < 0 || firstBeg >= firstEnd) {
      return -1;
   }

   int result = findMatchingBegTagLevelwise(string, beg, end, firstEnd, ignoreBegTags, ignoreEndTags);

   if (begin_function(string, result) && end_function(string, firstEnd)) {
      return result;
   }
   else {
      auto string_copy = string;
      static constexpr char c = '#';
      bool collides = true;

      for (uint8_t i = 0; i < 256; i++) {
         collides = ((c + i) == beg[0]);

         for (const auto& tag : ignoreBegTags) {
            collides = ((c + i) == beg[0]);
         }

         for (const auto& tag : ignoreEndTags) {
            collides = ((c + i) == beg[0]);
         }

         if (!collides) break;
      }

      if (collides) {
         Failable::getSingleton()->addError("No dummy begin tag found that matches none of the given begin end or ignore tags.");
         return -1;
      }
      else {
         string_copy[result] = c;
         string_copy[firstEnd] = c;

         return indexOfFirstInnermostBeginBracket(string_copy, beg, end, ignoreBegTags, ignoreEndTags, begin_function, end_function);
      }
   }
}

bool StaticHelper::isWithinAnyLevelwise(
   const std::string& string, 
   const int pos, 
   const std::vector<std::string>& beginTags, 
   const std::vector<std::string>& endTags) {
   assert(beginTags.size() == endTags.size());

   if (beginTags.empty()) {
      return false;
   }

   bool isWithinAny{ false };

   for (int i = 0; i < beginTags.size() && !isWithinAny; i++) {
      isWithinAny = isWithinLevelwise(string, pos, beginTags[i], endTags[i]);
   }

   return isWithinAny;
}

bool StaticHelper::isWithinLevelwise(
   const std::string& string, 
   const int pos, 
   const std::string& beginTag, 
   const std::string& endTag) {
   int curPos = pos;
   int count = 0;

   if (beginTag == endTag) {
      bool within = false;

      for (int i = pos; i >= 0; i--) {
         if (string.substr(i, beginTag.length()) == beginTag) {
            within = !within;
         }
      }

      return within;
   }

   while (count >= 0 && curPos < string.length()) {
      int curBegin = string.find(beginTag, curPos + 1);
      int curEnd = string.find(endTag, curPos + 1);

      if (curBegin < 0) {
         curBegin = curEnd;
      }

      if (curEnd < 0) {
         break;
      }

      if (curEnd > curBegin) {
         count++;
         curPos = curBegin;
      } else {
         count--;
         curPos = curEnd;
      }
   }

   return count < 0;
}

int StaticHelper::indexOfOnTopLevel(
   const std::string& string, 
   const std::vector<std::string>& find, 
   const int startPos, 
   const std::string& beginTag, 
   const std::string& endTag,
   const std::vector<std::string>& ignoreBegTags,
   const std::vector<std::string>& ignoreEndTags) 
{
   for (int i = startPos; i < string.length(); i++) { // Find the next point matching any of the find strings and not within the ignore tags.
      if (!isWithinAnyLevelwise(string, i, ignoreBegTags, ignoreEndTags)) {
         for (const auto& findstr : find) {
            if (StaticHelper::stringStartsWith(string, findstr, i)) {
               return i;
            }
         }

         if (stringStartsWith(string, beginTag, i)) { // Jump over "(...)" construct at the current position.
            i = findMatchingEndTagLevelwise(string, beginTag, endTag, i) + endTag.size() - 1; // The "-1" is to compensate for the "i++".
         }
         else if (stringStartsWith(string, endTag, i)) {
            return i;
         }
      }
   }

   return -1;
}

void vfm::StaticHelper::distributeIntoBeforeInnerAfter(std::string& before_out, std::string& main_in_out, std::string& after_out, const int begin, const int end)
{
   assert(before_out.empty());
   assert(after_out.empty());

   before_out = main_in_out.substr(0, begin);
   after_out = main_in_out.substr(end);
   main_in_out = main_in_out.substr(begin, end - begin);
}

void vfm::StaticHelper::distributeGetOnlyInner(std::string& main_in_out, const int begin, const int end)
{
   std::string dummy1, dummy2;
   distributeIntoBeforeInnerAfter(dummy1, main_in_out, dummy2, begin, end);
}

bool StaticHelper::isRenamedPrivateRecursiveVar(const std::string& var_name)
{
   return var_name.find(PRIVATE_VARIABLES_IN_RECURSIVE_CALLS_INTERNAL_SYMBOL) != std::string::npos;
}

bool vfm::StaticHelper::isRenamedPrivateVar(const std::string& var_name)
{
   return var_name.find(PRIVATE_VARIABLES_INTERNAL_SYMBOL) != std::string::npos;
}

bool vfm::StaticHelper::isPrivateVar(const std::string& var_name)
{
   return isRenamedPrivateRecursiveVar(var_name)
          || isRenamedPrivateVar(var_name)
          || stringStartsWith(var_name, PRIVATE_VARIABLES_PREFIX)
          || stringStartsWith(var_name, SYMB_REF + PRIVATE_VARIABLES_PREFIX);
}

std::string vfm::StaticHelper::plainVarNameWithoutLocalOrRecursionInfo(const std::string& input) {
   size_t pos{ input.find(PRIVATE_VARIABLES_INTERNAL_SYMBOL) };

   if (pos != std::string::npos) {
      return input.substr(0, pos);
   }

   return input;
}

std::string StaticHelper::makeString(char c) {
   return std::string(1, c);
}

// This function is licensed under CC-BY-SA-4.0 which is a copy-left license
// (https://creativecommons.org/licenses/by-sa/4.0/deed.en)!
void StaticHelper::ltrim(std::string& s) {
   // From https://stackoverflow.com/questions/216823/whats-the-best-way-to-trim-stdstring
   s.erase(s.begin(), std::find_if(s.begin(), s.end(), [](int ch) {return !std::isspace(ch); }));
}

// This function is licensed under CC-BY-SA-4.0 which is a copy-left license
// (https://creativecommons.org/licenses/by-sa/4.0/deed.en)!
void StaticHelper::rtrim(std::string& s) {
   s.erase(std::find_if(s.rbegin(), s.rend(), [](int ch) {return !std::isspace(ch); }).base(), s.end());
}

void StaticHelper::trim(std::string& s) {
   ltrim(s);
   rtrim(s);
}

std::string StaticHelper::ltrimAndReturn(const std::string& s) {
   std::string s_copy = s;
   ltrim(s_copy);
   return s_copy;
}

std::string StaticHelper::rtrimAndReturn(const std::string& s) {
   std::string s_copy = s;
   rtrim(s_copy);
   return s_copy;
}

std::string vfm::StaticHelper::trimAndReturn(const std::string& s)
{
   std::string s_copy = s;
   trim(s_copy);
   return s_copy;
}

void vfm::StaticHelper::stripAtBeginning(std::string& str, const std::vector<char> of)
{
   int pos = 0;

   for (; pos < str.size(); pos++) {
      bool found = false;

      for (const auto c : of) {
         if (str[pos] == c) {
            found = true;
            break;
         }
      }

      if (!found) {
         break;
      }
   }

   str = str.substr(pos);
}

bool StaticHelper::isOpeningBracket(const std::string& optor) {
   return optor == makeString(OPENING_BRACKET);
}

bool StaticHelper::isArgumentDelimiter(const std::string& optor) {
   return optor == makeString(ARGUMENT_DELIMITER);
}

bool StaticHelper::isClosingBracket(const std::string& optor) {
   return optor == makeString(CLOSING_BRACKET);
}

bool StaticHelper::isConstVal(const std::string& s) {
   return !s.empty() && s.find_first_not_of(DIGITS_WITH_DECIMAL_POINT) == std::string::npos;
}

bool StaticHelper::isConstVar(const std::string& s) { // Must consist of allowed letters for variables, with the exception of ":" which is the second part of the ?: operator.
   return !s.empty() && s != SYMB_COND_COL && s.find_first_not_of(VAR_NAME_LETTERS) == std::string::npos;
   // TODO: This still can fail for infix operators like so: 
   // @f(myop, x, y, x + y); a myop -b    ==>    Tape ran out of items
}

bool StaticHelper::isParsableAsFloat(const std::string& float_string)
{
   std::istringstream iss(trimAndReturn(float_string));
   float f;
   iss >> std::noskipws >> f; // noskipws considers leading whitespace invalid
   // Check the entire string was consumed and if either failbit or badbit is set
   return iss.eof() && !iss.fail(); 
}

bool vfm::StaticHelper::isParsableAsInt(const std::string& int_string, const bool allow_empty)
{
   std::string s = int_string.empty() || int_string[0] != '+' && int_string[0] != '-' ? int_string : int_string.substr(1);

   if (!allow_empty && s.empty()) {
      return false;
   }

   return s.find_first_not_of("0123456789") == std::string::npos;
}

bool StaticHelper::isAlphaNumericOrOneOfThese(const std::string& str, const std::string& additionally_allowed)
{
   return find_if(str.begin(), str.end(), [&additionally_allowed](char c) { return !std::isalnum(c) && additionally_allowed.find(c) == std::string::npos; }) == str.end();
}

bool vfm::StaticHelper::isAlpha(const std::string& str)
{
   return find_if(str.begin(), str.end(), [](char c) { return !std::isalpha(c); }) == str.end();
}

bool vfm::StaticHelper::isAlphaNumeric(const std::string& str)
{
   return find_if(str.begin(), str.end(), [](char c) { return !std::isalnum(c); }) == str.end();
}

bool vfm::StaticHelper::isAlphaNumericOrUnderscore(const std::string& str)
{
   return find_if(str.begin(), str.end(), [](char c) { return !(std::isalnum(c)) && c != '_'; }) == str.end();
}

bool vfm::StaticHelper::isAlphaNumericOrUnderscoreOrColonOrComma(const std::string& str)
{
   return find_if(str.begin(), str.end(), [](char c) { return !(std::isalnum(c)) && c != '_' && c != ':' && c != ','; }) == str.end();
}

bool vfm::StaticHelper::isValidVfmVarName(const std::string& str)
{
   return !str.empty() && stringConformsTo(str, VAR_AND_FUNCTION_NAME_REGEX);
}

bool vfm::StaticHelper::stringConformsTo(const std::string& str, const std::regex& regex)
{
   for (const auto& c : str) {
      if (!std::regex_match(makeString(c), regex)) {
         return false;
      }
   }

   return true;
}

std::string vfm::StaticHelper::incrementAlphabeticNum(const std::string& num)
{
   std::string res = num;
   int pos = res.size() - 1;

   while (res[pos] == ALPHA_UPP[ALPHA_UPP.size() - 1]) {
      res[pos] = ALPHA_UPP[0];
      pos--;

      if (pos < 0) {
         res += ALPHA_UPP[0];
         return res;
      }
   }

   res[pos] = ALPHA_UPP[CHAR_NUM_MAPPING.at(res[pos]) + 1];

   return res;
}

bool vfm::StaticHelper::isAssociative(const MathStructPtr formula, const std::string& infix_operator, const AssociativityTypeEnum type)
{
   auto break_it_off{ std::make_shared<bool>(false) };
   const int forbidden_index = type == AssociativityTypeEnum::left ? 1 : 0;

   formula->applyToMeAndMyChildren([&infix_operator, type, forbidden_index, break_it_off](const MathStructPtr m_raw) {
      auto m{ m_raw->thisPtrGoIntoCompound() };

      if (m->getOptor() == infix_operator && m->getOperands()[forbidden_index]->getOptorJumpIntoCompound() == infix_operator) {
         *break_it_off = true;
      }
   }, TraverseCompoundsType::avoid_compound_structures, nullptr, break_it_off);

   return !(*break_it_off);
}

TermPtr vfm::StaticHelper::makeAssociative(const TermPtr formula, const std::string& infix_operator, const AssociativityTypeEnum type)
{
   TermPtr fmla{ _id(formula) };
   const int forbidden_index{ type == AssociativityTypeEnum::left ? 1 : 0 };
   const int good_index{ 1 - forbidden_index };
   bool changed{ true };

   while (changed) {
      changed = false;

      fmla->applyToMeAndMyChildrenIterative([&infix_operator, forbidden_index, good_index, &changed](const MathStructPtr m_raw) {
         auto m{ m_raw->thisPtrGoIntoCompound() };

         if (m->getOptor() == infix_operator && m->getOperands()[forbidden_index]->getOptorJumpIntoCompound() == infix_operator) {
            m->swap();
            auto temp{ m->getOperands()[forbidden_index] };
            m->getOperands()[forbidden_index] = m->getOperands()[good_index]->getTermsJumpIntoCompounds()[forbidden_index];
            m->getOperands()[good_index]->getTermsJumpIntoCompounds()[forbidden_index] = m->getOperands()[good_index]->getTermsJumpIntoCompounds()[good_index];
            m->getOperands()[good_index]->getTermsJumpIntoCompounds()[good_index] = temp;
            changed = true;
         }
      }, TraverseCompoundsType::avoid_compound_structures);
   }

   return fmla->getOperands()[0]->setChildrensFathers(true, false)->toTermIfApplicable();
}

void StaticHelper::pushBack(
   std::vector<std::string>& tokens, 
   const std::string& token, 
   int class_num, 
   int ignore_class, 
   const std::set<std::string>& ignore_tokens,
   const std::map<std::string, std::string>& aka_map) 
{
   if (class_num == ignore_class) return;

   if (token.length() > 1 
      && (token[0] == OPENING_BRACKET || token[0] == CLOSING_BRACKET 
         || token[0] == OPENING_BRACKET_BRACE_STYLE || token[0] == CLOSING_BRACKET_BRACE_STYLE
         || token[0] == OPENING_BRACKET_SQUARE_STYLE_FOR_ARRAYS || token[0] == CLOSING_BRACKET_SQUARE_STYLE_FOR_ARRAYS)) {
      pushBack(tokens, StaticHelper::makeString(token[0]), class_num, ignore_class, ignore_tokens, aka_map); // TODO: Remove
      pushBack(tokens, token.substr(1), class_num, ignore_class, ignore_tokens, aka_map);
   }
   else if (ignore_tokens.find(token) == ignore_tokens.end()) {
      auto it = aka_map.find(token);

      if (it == aka_map.end()) {
         for (const auto& aka : aka_map) {
            if (StaticHelper::stringStartsWith(token, aka.first + ".")) {
               tokens.push_back(token.substr(0, aka.first.size()) + aka.second + token.substr(aka.first.size()));
               return;
            }
         }

         tokens.push_back(token);
      }
      else {
         tokens.push_back(token + it->second);
      }
   }
}

int StaticHelper::getClassNum(std::string s, std::vector<std::regex> classes, int start_at_class, FormulaParser& parser) {
   for (size_t i = start_at_class; i < classes.size() + start_at_class; ++i) {
      int j = i % classes.size();

      if (std::regex_match(s, classes[j])) {
         return j;
      }
   }

   //std::stringstream ss; 
   //ss << "No regex matches '" << s << "'.";
   //parser.addError(ss.str());
   return -1;
}

void StaticHelper::addStringToken(std::vector<std::string>& tokens, const std::string& token)
{
   const std::string var_name_str_begin = "strin__";
   tokens.push_back(makeString(OPENING_BRACKET));
   tokens.push_back(SYMB_SET_VAR);
   tokens.push_back(makeString(OPENING_BRACKET));
   tokens.push_back(SYMB_REF + var_name_str_begin);
   tokens.push_back(makeString(ARGUMENT_DELIMITER));
   tokens.push_back(SYMB_MALLOC);
   tokens.push_back(makeString(OPENING_BRACKET));
   tokens.push_back(std::to_string(token.size()));
   tokens.push_back(makeString(CLOSING_BRACKET));
   tokens.push_back(makeString(CLOSING_BRACKET));
   tokens.push_back(SYMB_SEQUENCE);

   for (int i = 0; i < token.size(); i++) {
      tokens.push_back(SYMB_SET_VAR);
      tokens.push_back(makeString(OPENING_BRACKET));
      tokens.push_back(makeString(OPENING_BRACKET));
      tokens.push_back(std::to_string(i));
      tokens.push_back(SYMB_PLUS);
      tokens.push_back(var_name_str_begin);
      tokens.push_back(makeString(CLOSING_BRACKET));
      tokens.push_back(makeString(ARGUMENT_DELIMITER));
      tokens.push_back(std::to_string((int) token[i]));
      tokens.push_back(makeString(CLOSING_BRACKET));
      tokens.push_back(SYMB_SEQUENCE);
   }

   tokens.push_back(var_name_str_begin);
   tokens.push_back(makeString(CLOSING_BRACKET));
   tokens.push_back(makeString(ARGUMENT_DELIMITER));
   tokens.push_back(std::to_string(token.size()));
}

bool vfm::StaticHelper::sanityCheckTokensFromCpp(const std::vector<std::string>& tokens)
{
   const std::string DUMMY_STRING = "";

   for (int i = 0; i < tokens.size(); i++) {
      const std::string& last_token = ((i - 1) < tokens.size()) ? tokens.at(i - 1) : DUMMY_STRING;
      const std::string& current_token = tokens.at(i);
      const std::string& next_token = ((i + 1) < tokens.size()) ? tokens.at(i + 1) : DUMMY_STRING;
      const std::string& next_next_token = ((i + 2) < tokens.size()) ? tokens.at(i + 2) : DUMMY_STRING;

      if (current_token == "-" && next_token == ">") {
         Failable::getSingleton(FAILABLE_NAME_SANITY_CHECK)->addWarning("Found sequence of '-' and '>' in token stream '..." + last_token + current_token + next_token + next_next_token + "...'. Is this a left-over '->' operator from C++ or something fancy I don't get?");
         i += 3;
      }
   }

   return true;
}

int temp_var_num = 0;

std::string tempVarName()
{
   return ":" + std::to_string(temp_var_num++);
}

void insertAndYellOnMalformedAka(const std::string& token, const std::string& future_aka, std::map<std::string, std::string>& aka_map, const std::string& token_recovered)
{
   if (!aka_map.insert({ token, future_aka }).second) {
      Failable::getSingleton(FAILABLE_NAME_FOR_AKA_REPLACEMENT)->addError(
         "Variable '" + token + "' has more than one aka assigned to it: "
         + "'" + StaticHelper::fromSafeString(aka_map.at(token.substr(STRING_PREFIX_AKA.size()))) + "' and '" + token_recovered + "'.");
   }
}

std::vector<std::pair<std::string, std::string>> vfm::StaticHelper::deriveListOfPairsForAkaAnnotatedVarName(const std::string& var_name_with_akas)
{ // a...#sth1#.b...#sth2#.c...#sth3# ==> {{a, sth1}, {b, sth2}, {c, sth3}}
   std::vector<std::pair<std::string, std::string>> res;
   int pos = var_name_with_akas.find(STRING_PREFIX_AKA);
   int pos2 = -1;
   int last_pos = 0;

   while (pos != std::string::npos) {
      pos2 = var_name_with_akas.find(".", pos + STRING_PREFIX_AKA.size());
      std::string first = var_name_with_akas.substr(last_pos, pos - last_pos);
      std::string second = var_name_with_akas.substr(pos, pos2 - pos);
      last_pos = pos2 + 1;

      res.push_back({ first, second.substr(STRING_PREFIX_AKA.size()) });
      pos = var_name_with_akas.find(STRING_PREFIX_AKA, pos + 1);
   }

   if (pos2 != std::string::npos) {
      res.push_back({ var_name_with_akas.substr(pos2), "" });
   }

   return res;
}

void vfm::StaticHelper::adjustTokenWithAkaOrTalInfo(std::string& token, const std::string& current_aka_or_tal)
{
   auto list = deriveListOfPairsForAkaAnnotatedVarName(token);

   if (!list.empty()) {
      token = list.at(0).first;

      for (int i = 1; i < list.size(); i++) {
         token += list.at(i).first;
      }
   }

   token += current_aka_or_tal;
}

const std::regex VAR_AND_FUNCTION_NAME_REGEX_LOCAL("^[" + VAR_AND_FUNCTION_NAME_REGEX_STRING_BASE + "]*$");

bool isTokenVariable(const std::string& token)
{
   return std::regex_match(token, VAR_AND_FUNCTION_NAME_REGEX_LOCAL)
      && !isdigit(token.at(0))
      && token.at(0) != '.';
}

std::shared_ptr<std::vector<std::string>> StaticHelper::tokenize(
   const std::string& formula, 
   FormulaParser& parser,
   int& pos,
   const int max_tokens,
   const bool reverse_direction,
   const std::vector<std::regex>& classes, 
   const int ignore_class,
   const bool replace_strings_with_print_commands,
   const bool remove_lose_sequence_symbols,
   const AkaTalProcessing aka_processing,
   const std::set<std::string>& ignore_tokens,
   const bool pass_on_quotes_without_processing,
   const std::function<bool(const std::vector<std::string>& tokens_so_far)> abort_function)
{
   std::map<std::string, std::string> aka_map;

   if (formula.empty()) {
      return std::make_shared<std::vector<std::string>>();
   }

   const std::string arg_del = makeString(ARGUMENT_DELIMITER);
   auto tokens = std::make_shared<std::vector<std::string>>();
   std::string token;
   int class_num = getClassNum(makeString(formula[std::max(pos, 0)]), classes, 0, parser);

   for (pos; pos >= 0 && pos < formula.length() && tokens->size() < max_tokens && !abort_function(*tokens); reverse_direction ? --pos : ++pos) {
      const int cn = getClassNum(makeString(formula[pos]), classes, class_num, parser);
      bool is_in_quotes = false;
      const bool is_float_dot{ formula[pos] == '.' && StaticHelper::isParsableAsFloat(token) };

      if (!is_float_dot && (cn != class_num || pos == 0 && !pass_on_quotes_without_processing && formula[pos] == OPENING_QUOTE)) {
         //if (pos != 0) { // TODO: Not sure what this was about. Delete if everything keeps working.
            if (remove_lose_sequence_symbols && (token[0] == CLOSING_BRACKET || token == arg_del || token == SYMB_SEQUENCE)) { // TODO: Doesn't work correctly in reverse_direction.
               while (!tokens->empty() && tokens->back() == SYMB_SEQUENCE) {
                  tokens->pop_back();
               }
            }

            // With overloaded operators (e.g., -(x) vs. x - y),...
            // (a) if (c) {a} -(x) = 4;  // 1a <== from C++
            // ==> if (c) {a}; -(x) = 4; // 2a
            // ==> if (c, a); -(x) = 4;  // 3a ==> OK
            // ... vs. ...
            // (b) if (c) {a} - x = 4;  // 1b <== NOT from C++, but could happen in vfm
            // ==> if (c) {a}; - x = 4; // 2b
            // ==> if (c, a); - x = 4;  // 3b ==> not distinguishable from 3a
            // =!> if (c, a) - x = 4    // 4b 
            // ...is ambiguous. (Note that 1b could put bracktes around "x" and lose the space, and then be equal to 1a.)
            // Hence, we don't allow the C-style syntax of (b) anymore. Note that (a) is fine, and "if (c, a) - x = 4" is still possible.
            // 
            // Note that with the code below, the original syntax still works for operators that have no overload.
            // Nonetheless, this syntax should be avoided in practice since it could lead to legacy code silently ceasing to
            // work when an operator is overloaded in more recent code.
            // 
            if (remove_lose_sequence_symbols && parser.isOperatorPossibly(token)) { // TODO: Doesn't work correctly in reverse_direction.
               if (parser.isOperatorForSure(token)) {
                  while (!tokens->empty() && tokens->back() == SYMB_SEQUENCE) {
                     tokens->pop_back();
                  }
               }
               else if (!tokens->empty() && tokens->back() == SYMB_SEQUENCE) {
                  parser.addError("I was supposed to remove lose sequence symbols, but could not decide in this case since I could not determine if '" + token + "' is an operator or a function (there are " + std::to_string(parser.getNumParams(token).size()) + " > 1 overloads).");
               }
            }

            if (!reverse_direction && aka_processing == AkaTalProcessing::do_it && StaticHelper::stringStartsWith(token, STRING_PREFIX_TAL)) { // Captures TAL and AKA.
               const std::string token_recovered{ StaticHelper::fromSafeString(StaticHelper::replaceAll(token, ".", "")) };
               std::string current_aka_or_tal{};
               std::string future_aka_flexible{};
               std::string future_aka_static{};
               bool went_over_semicolon{ false };

               int token_idx{ (int) tokens->size() - 1 };

               while (!std::regex_match(tokens->at(token_idx), VAR_AND_FUNCTION_NAME_REGEX_LOCAL)) {
                  if (tokens->at(token_idx) == SYMB_SEQUENCE) {
                     went_over_semicolon = true;
                  }

                  token_idx--; // Go backwards to last variable to apply aka to it (and possibly its assignee).
               }

               if (StaticHelper::stringStartsWith(token, STRING_PREFIX_AKA)) { // Actually AKA.
                  const std::string temp_var_name = tempVarName();
                  current_aka_or_tal = STRING_PREFIX_AKA + StaticHelper::safeString("@" + temp_var_name + "=(" + token_recovered + ")");
                  future_aka_static = STRING_PREFIX_AKA + StaticHelper::safeString(temp_var_name);
                  future_aka_flexible = token;
               }
               else {                                                          // Actually TAL.
                  current_aka_or_tal = token;
                  future_aka_static = token;
                  future_aka_flexible = token;
               }

               // In "A a = b; // #vfm-aka [[ * ]]", attach * to both a and b, but assume a to be static and
               // b to be flexible. E.g. 
               //    A a = b[i] // #vfm-aka [[ if (i) { literal(x) } else { literal(y) } ]]
               // would attach "@temp_var = (if (i) { literal(x) } else { literal(y) })" to a and b. For future occurences
               // of a it would attach "temp_var", and for future occurences of b "if (i) { literal(x) } else { literal(y) }".
               // For TAL, only the original encoding is pasted onto every occurrence.
               if (isTokenVariable(tokens->at(token_idx))) {
                  insertAndYellOnMalformedAka(tokens->at(token_idx), current_aka_or_tal, aka_map, future_aka_flexible);
                  adjustTokenWithAkaOrTalInfo(tokens->at(token_idx), current_aka_or_tal);
               }

               if (went_over_semicolon 
                  && token_idx > 1 
                  && tokens->at(token_idx - 1) == SYMB_SET_VAR_A 
                  && isTokenVariable(tokens->at(token_idx - 2))) {
                  insertAndYellOnMalformedAka(tokens->at(token_idx - 2), future_aka_static, aka_map, token_recovered);
                  adjustTokenWithAkaOrTalInfo(tokens->at(token_idx - 2), current_aka_or_tal);
               }
            }
            else {
               pushBack(*tokens, token, class_num, ignore_class, ignore_tokens, aka_map);
            }

            token = "";
            class_num = cn;
         //}

         if (!pass_on_quotes_without_processing && formula[pos] == OPENING_QUOTE) {
            const int beg = pos + 1;
            int end = beg;
            is_in_quotes = true;

            for (; formula[end] != CLOSING_QUOTE; end++);
            const std::string tok = formula.substr(beg, end - beg);

            if (replace_strings_with_print_commands) {
               addStringToken(*tokens, tok);
            }
            else {
               tokens->push_back(tok);
            }

            pos = end;
         }
      }

      if (is_in_quotes) {
         class_num = ignore_class;
      } else {
         if (reverse_direction) {
            token = formula[pos] + token;
         }
         else {
            token = token + formula[pos];
         }
      }

      if (tokens->size() >= max_tokens || abort_function(*tokens)) { // Will abort at next check, but not due to pos out of range.
         reverse_direction ? ++pos : --pos; // Then we miss one further step in pos to be consistent.
      }
   }

   if (token[0] == CLOSING_BRACKET || token == arg_del) {
      if (remove_lose_sequence_symbols && !tokens->empty() && tokens->back() == SYMB_SEQUENCE) {
         tokens->pop_back();
      }
   }

   if (tokens->size() < max_tokens) {
      pushBack(*tokens, token, class_num, ignore_class, ignore_tokens, aka_map);
   }

   while (remove_lose_sequence_symbols && !tokens->empty() && tokens->back() == SYMB_SEQUENCE) {
      tokens->pop_back();
   }

   while (tokens->size() > max_tokens) { // Can happen, for example, when several brackets "(((" are treated as one token at first and separated later.
      tokens->pop_back();
   }

   return tokens;
}

std::shared_ptr<std::vector<std::string>> StaticHelper::tokenizeSimple(
   const std::string& formula,
   const int max_tokens,
   const bool reverse_direction,
   const std::vector<std::regex>& classes,
   const int ignore_class,
   const bool replace_strings_with_print_commands,
   const bool remove_lose_sequence_symbols,
   const AkaTalProcessing aka_processing)
{
   int dummy = reverse_direction ? formula.size() - 1 : 0;

   return tokenize(
      formula, 
      *SingletonFormulaParser::getLightInstance(), 
      dummy, 
      max_tokens, 
      reverse_direction, 
      classes, 
      ignore_class, 
      replace_strings_with_print_commands, 
      remove_lose_sequence_symbols,
      aka_processing);
}

std::vector<std::string> vfm::StaticHelper::tokenizeCharacterwise(const std::string& string, const bool unify_white_space, const bool deflate_spaces_to_one)
{
   std::vector<std::string> chars;
   chars.reserve(string.size());
   bool last_was_space = false;

   for (const auto c : string) {
      if (unify_white_space && std::isspace(c)) {
         if (!deflate_spaces_to_one || !last_was_space) {
            chars.push_back(" ");
         }

         last_was_space = true;
      }
      else {
         chars.push_back(makeString(c));
         last_was_space = false;
      }
   }

   return chars;
}

std::string BracketStructure::getContentAt(const std::vector<int> path_through_tree) 
{
   StaticHelper::serializeBracketStructure(shared_from_this());
   if (path_through_tree.empty()) {
      if (children_.empty()) {
         return StaticHelper::serializeBracketStructure(shared_from_this());
      }
      else {
         return children_.at(0)->getContentAt({});
      }
   }

   if (children_.size() > path_through_tree.at(0)) {
      std::vector<int> subvector = { path_through_tree.begin() + 1, path_through_tree.end() };
      return children_.at(path_through_tree.at(0))->getContentAt(subvector);
   }

   if (children_.size() == 1) { // Desperately try to go one deeper. Stupid C++ Syntax allows this in initializations...
      return children_.at(0)->getContentAt(path_through_tree);
   }

   return "";
}

// From https://stackoverflow.com/questions/236129/the-most-elegant-way-to-iterate-the-words-of-a-string
// This function is licensed under CC-BY-SA-4.0 which is a copy-left license
// (https://creativecommons.org/licenses/by-sa/4.0/deed.en)!
template<typename Out>
void StaticHelper::split(const std::string& s, char delim, Out result) 
{
   std::stringstream ss(s);
   std::string item;
   while (std::getline(ss, item, delim)) {
      *(result++) = item;
   }
}

std::vector<std::string> vfm::StaticHelper::splitOnWhiteSpaces(const std::string& input)
{
   std::istringstream buffer{ input };
   std::vector<std::string> ret{ std::istream_iterator<std::string>(buffer), {} };
   return ret;
}

std::vector<std::string> StaticHelper::split(const std::string& s, char delim)
{
   std::vector<std::string> elems;
   split(s, delim, std::back_inserter(elems));
   return elems;
}

std::vector<std::string> StaticHelper::split(const std::string& str, const std::string& delimiter, const SplitCondition& f, const bool keep_delimiter)
{
   std::vector<std::string> split;
   std::string s = str;

   size_t pos = 0;
   std::string token;
   std::string add_on;

   for (;;) {
      pos = s.find(delimiter, pos);
      bool found{ (pos != std::string::npos) };

      if (found) {
         bool valid{ f(s, pos) };

         if (valid) {
            token = s.substr(0, pos);
            split.push_back(add_on + token);
            s.erase(0, pos + delimiter.length());

            if (keep_delimiter) {
               add_on = delimiter;
            }

            pos = 0;
         }
         else {
            pos += delimiter.size();
         }
      }
      else {
         break;
      }
   }

   split.push_back(add_on + s);

   return split;
}

std::vector<std::string> StaticHelper::split(const std::vector<std::string>& ss, const std::string& delim, const SplitCondition& f, const bool keep_delimiter)
{
   std::vector<std::string> result;

   for (const auto& s : ss) {
      auto vec = split(s, delim, f, keep_delimiter);
      result.insert(result.end(), vec.begin(), vec.end());
   }

   return result;
}

size_t spaceLeft(const size_t line_length, const size_t word_length)
{
   return line_length >= word_length ? line_length - word_length : 0;
}

std::string vfm::StaticHelper::checkForWrapInTokenMode(const std::set<std::string>& wrap_on, std::deque<char>& latest, const std::string& text, const char current_char, const int max_len)
{
   bool matched = false;

   for (const auto& wo : wrap_on) {
      if (wo.size() <= latest.size()) {
         bool match_temp = true;

         for (int j = 0; j < wo.size(); j++) {
            if (latest[latest.size() - j - 1] != wo[wo.size() - j - 1]) {
               match_temp = false;
               break;
            }
         }

         if (match_temp) {
            matched = true;
            break;
         }
      }
   }

   latest.push_back(current_char);
   if (latest.size() > max_len) {
      latest.pop_front();
   }

   if (matched) {
      return "\n";
   }
   else {
      return "";
   }
}

std::string vfm::StaticHelper::wrapOnTokens(const std::string& text_raw, const std::set<std::string>& wrap_before, const std::set<std::string>& wrap_after)
{
   std::deque<char> latest;
   std::string text1;
   std::string text2;
   int max_len_before = 0;
   int max_len_after = 0;
   std::set<std::string> wrap_before_reversed;

   for (const auto& s : wrap_before) {
      std::string s2 = s;
      std::reverse(s2.begin(), s2.end());
      wrap_before_reversed.insert(s2);
   }

   for (const auto& s : wrap_before) {
      if (s.size() > max_len_before) {
         max_len_before = s.size();
      }
   }

   for (const auto& s : wrap_after) {
      if (s.size() > max_len_after) {
         max_len_after = s.size();
      }
   }

   for (int i = 0; i < text_raw.size(); i++) {
      text1 = text1 + checkForWrapInTokenMode(wrap_after, latest, text1, text_raw[i], max_len_after) + text_raw[i];
   }

   latest.clear();

   for (int i = text1.size() - 1; i >= 0; i--) {
      text2 = text1[i] + checkForWrapInTokenMode(wrap_before_reversed, latest, text2, text1[i], max_len_before) + text2;
   }

   return text2;
}

std::string vfm::StaticHelper::wrapOnLineLength(const std::string& text_raw, const size_t max_line_length)
{
   std::istringstream words(text_raw.c_str());
   std::ostringstream wrapped;
   std::string word;

   if (words >> word) {
      wrapped << word;
      size_t space_left = spaceLeft(max_line_length, word.length());
      while (words >> word) {
         if (space_left < word.length() + 1) {
            wrapped << '\n' << word;
            space_left = spaceLeft(max_line_length, word.length());
         }
         else {
            wrapped << ' ' << word;
            space_left = space_left >= word.length() + 1 ? space_left - (word.length() + 1) : 0;
         }
      }
   }

   return wrapped.str();
}

std::string StaticHelper::wrap(const std::string& text_raw, const size_t max_line_length, const std::set<std::string>& additionally_wrap_before, const std::set<std::string>& additionally_wrap_after)
{
   std::string s;

   for (const auto& line : StaticHelper::split(wrapOnTokens(text_raw, additionally_wrap_before, additionally_wrap_after), "\n")) {
      s += wrapOnLineLength(line, max_line_length) + "\n";
   }

   return s;
}

std::string StaticHelper::getFileNameFromPath(const std::string& path)
{
   return path.substr(path.find_last_of("/\\") + 1);
}

std::string vfm::StaticHelper::removeFileNameFromPath(const std::string& path)
{
   int length_of_file_name = StaticHelper::getFileNameFromPath(path).size();
   return path.substr(0, path.size() - length_of_file_name);
}

std::string StaticHelper::getFileNameFromPathWithoutExt(const std::string& path)
{
   return removeLastFileExtension(getFileNameFromPath(path));
}


std::string StaticHelper::replaceAll(std::string str, const std::string& from, const std::string& to) {
    size_t start_pos = 0;
    while((start_pos = str.find(from, start_pos)) != std::string::npos) {
        str.replace(start_pos, from.length(), to);
        start_pos += to.length(); // Handles case where 'to' is a substring of 'from'
    }
    return str;
}

std::string vfm::StaticHelper::replaceAllRegex(const std::string& str, const std::string& from_regex, const std::string& to_regex)
{
   std::regex regex_in(from_regex);
   return regex_replace(str, regex_in, to_regex);
}

bool StaticHelper::isBooleanTrue(const std::string& bool_str)
{
   auto str = StaticHelper::toLowerCase(StaticHelper::trimAndReturn(bool_str));
   bool parsable = StaticHelper::isParsableAsFloat(str);
   bool result = parsable ? std::stof(str) != 0.0f : str != "false";

   if (!parsable && str != "false" && str != "true") {
      Failable::getSingleton()->addWarning("Found supposedly boolean value '" + bool_str + "' which looks strange. Please make sure it is correct that I interpreted it as '" + std::to_string(result) + "'.");
   }

   return result;
}

// Function to extract an integer after a specific substring.
int extractIntegerAfterSubstring(const std::string& str, const std::string& substring) {
   size_t pos = str.find(substring);

   if (pos == std::string::npos) {
      std::cerr << "Error: Substring '" << substring << "' not found." << std::endl;
      return -1; // Or throw an exception
   }

   // Find the position of the first underscore after the substring.
   size_t startPos = pos + substring.length();
   size_t endPos = str.find_first_not_of("0123456789", startPos);

   // Extract the number string
   std::string numberStr;
   if (endPos == std::string::npos)
   {
      numberStr = str.substr(startPos);
   }
   else
   {
      numberStr = str.substr(startPos, endPos - startPos);
   }

   // Convert to integer and handle errors.
   try {
      return stoi(numberStr);
   }
   catch (const std::invalid_argument& e) {
      std::cerr << "Error: Invalid integer format after '" << substring << "'." << std::endl;
      return -1;
   }
   catch (const std::out_of_range& e) {
      std::cerr << "Error: Integer out of range after '" << substring << "'." << std::endl;
      return -1;
   }
}

void postprocessTrace(MCTrace& trace)
{ // TODO: Lots of hard-coded stuff.
   std::map<std::string, std::vector<bool>> lane_bools{};
   std::map<std::string, int> on_straight_section{}, traversion_from{}, traversion_to{};

   // Remove possible leading env. when using EnvModel as SMV module.
   for (auto& state : trace.getTrace()) {
      std::vector<std::pair<std::string, std::string>> to_add{};

      for (auto& varvals : state.second) {
         if (StaticHelper::stringStartsWith(varvals.first, "env.")) {
            to_add.push_back({ varvals.first.substr(4), varvals.second });
         }
      }

      for (const auto& el : to_add) {
         state.second.insert({ el.first, el.second });
      }
   }

   // Create special variables such as the "on_lane" or the "on_straight_section", "traversion_from", "traversion_to" variables.
   for (auto& state : trace.getTrace()) {
      for (auto& varvals : state.second) {
         if (StaticHelper::stringContains(varvals.first, "lane_b")) {
            const size_t lane_num = extractIntegerAfterSubstring(varvals.first, "lane_b");
            const bool bool_val = StaticHelper::isBooleanTrue(varvals.second);
            std::string which{};

            std::string helper{ StaticHelper::replaceAll(varvals.first, "env.", "") };
            if (StaticHelper::stringStartsWith(helper, "ego")) {
               which = "ego.";
            }
            else {
               which = StaticHelper::split(helper, ".")[0] + ".";
            }

            while (lane_bools[which].size() <= lane_num) {
               lane_bools[which].push_back(false);
            }

            lane_bools[which][lane_num] = bool_val; // Assuming all values are initialized in the beginning (otherwise we'd need to initialize to false or so).
         }
         else if (StaticHelper::stringContains(varvals.first, "is_on_sec_")) {
            const bool bool_val = StaticHelper::isBooleanTrue(varvals.second); // Works for 0..1 as well as true/false.

            if (bool_val) {
               const int sec_num = extractIntegerAfterSubstring(varvals.first, "is_on_sec_");
               std::string which{};

               std::string helper{ StaticHelper::replaceAll(varvals.first, "env.", "") };
               if (StaticHelper::stringStartsWith(helper, "ego")) {
                  which = "ego.";
               }
               else {
                  which = StaticHelper::split(helper, ".")[0] + ".";
               }

               on_straight_section[which] = sec_num;
               traversion_from[which] = -1;
               traversion_to[which] = -1;
            }
         }
         else if (StaticHelper::stringContains(varvals.first, "is_traversing_from_sec_")) {
            const bool bool_val = StaticHelper::isBooleanTrue(varvals.second); // Works for 0..1 as well as true/false.

            if (bool_val) {
               const int from_num = extractIntegerAfterSubstring(varvals.first, "from_sec_");
               const int to_num = extractIntegerAfterSubstring(varvals.first, "to_sec_");
               std::string which{};

               std::string helper{ StaticHelper::replaceAll(varvals.first, "env.", "") };
               if (StaticHelper::stringStartsWith(helper, "ego")) {
                  which = "ego.";
               }
               else {
                  which = StaticHelper::split(helper, ".")[0] + ".";
               }

               on_straight_section[which] = -1;
               traversion_from[which] = from_num;
               traversion_to[which] = to_num;
            }
         }

         varvals.second = StaticHelper::replaceAll(varvals.second, "0sd16_", ""); // Remove bitvector encoding (16 bits, signed, decimal)
      }

      for (const auto& pair : lane_bools) {
         int on_lane{0};

         for (int i = 0; i < pair.second.size(); i++) {
            if (pair.second[i]) {
               on_lane = i * 2;

               if (pair.second.size() > i + 1 && pair.second.at(i + 1)) {
                  on_lane++;
               }

               break;
            }
         }

         std::string on_lane_str = std::to_string(on_lane);

         state.second[pair.first + "on_lane"] = on_lane_str;
      }

      for (const auto& pair : on_straight_section) {
         int straight{ pair.second };
         int from{ traversion_from.at(pair.first) };
         int to{ traversion_to.at(pair.first) };

         state.second[pair.first + "on_straight_section"] = std::to_string(straight);
         state.second[pair.first + "traversion_from"] = std::to_string(from);
         state.second[pair.first + "traversion_to"] = std::to_string(to);
      }

      state.second["Array.CurrentState"] = "state_EnvironmentModel";
   }

   int size = trace.size();
   for (int i = 0; i < size; i++) {
      trace.getTrace().insert(trace.getTrace().begin() + 2 * i + 1, {"dummy", {{"Array.CurrentState", "state_TacticalPlanner"}}});
   }
}

std::vector<MCTrace> vfm::StaticHelper::extractMCTracesFromMSATIC(const std::string& cexp_string)
{
   mc::msatic::Parser parser;
   auto trace_input = parser.extractLinesFromRawTrace(cexp_string);
   auto parsed_trace = parser.parseStepsFromTrace(trace_input);
   postprocessTrace(parsed_trace);
   return parsed_trace.empty() ? std::vector<MCTrace>{} : std::vector<MCTrace>{ parsed_trace };
}

std::vector<MCTrace> vfm::StaticHelper::extractMCTracesFromMSATICFile(const std::string& cexp_string)
{
   return extractMCTracesFromMSATIC(readFile(cexp_string));
}

std::vector<MCTrace> StaticHelper::extractMCTracesFromNusmv(const std::string& cexp_string, const TraceExtractionMode mode)
{
   static const std::string DELIMITER{ "Trace Type: Counterexample" }; // TODO: Make sure this is actually always a fixed string in nuXmv.

   if (cexp_string.empty()) {
      if (mode != TraceExtractionMode::quick_only_detect_if_empty) {
         Failable::getSingleton()->addNote("Received empty CEX in function 'extractMCTraceFromNusmv'. Nothing to do.");
      }

      return {};
   }

   std::vector<MCTrace> traces{};

   if (mode == TraceExtractionMode::quick_only_detect_if_empty) {
      size_t occ{ occurrencesInString(cexp_string, DELIMITER) };
      std::vector<MCTrace> simp_res{};

      for (auto i = 0; i < occ; i++) {
         simp_res.push_back({}); // Fill with empty traces, since we only need to reflect HOW MANY traces.
      }

      return simp_res; // Return non-empty if and only if there is a CEX in the data.
   }

   for (const auto& single_cex : StaticHelper::split(cexp_string, DELIMITER)) {
      MCTrace ce{};

      auto lines = split(single_cex, '\n');

      if (lines.empty() || StaticHelper::stringContains(lines[lines.size() - 1], "-- no counterexample found")) {
         continue;
      }

      lines.push_back("->EOF");
      std::string state{};
      VarVals vars{};

      for (const auto& line : lines) {
         std::string cline{ StaticHelper::removeWhiteSpace(line) };

         if (!StaticHelper::stringStartsWith(cline, "#")) {
            if ((StaticHelper::stringStartsWith(cline, "->") || StaticHelper::stringStartsWith(cline, "--Loop"))
               && !StaticHelper::stringContains(cline, "->Input:")) { // Jump over "input" state to squash envModel and Planner cycle into one.
               if (!vars.empty()) {
                  ce.addTraceStep({ state, vars });
                  vars.clear();
               }

               if (StaticHelper::stringStartsWith(cline, "--Loop")) {
                  ce.addTraceStep({ "LOOP", {} });
               }
               else {
                  state = StaticHelper::replaceAll(StaticHelper::replaceAll(StaticHelper::replaceAll(cline, "<-", ""), "->Input:", "I"), "->State:", "");
               }
            }
            else if (!StaticHelper::stringContains(cline, "->Input:")) {
               if (!state.empty()) {
                  auto split = StaticHelper::split(cline, '=');

                  if (split.size() == 2) {
                     std::string var = split[0];
                     std::string val = split[1];
                     vars.insert({ var, val });
                  }
               }
            }
         }
      }

      postprocessTrace(ce);

      if (!ce.empty()) traces.push_back(ce); // Empty CEXs are not put in the list, such that empty list indicates no CEXs.
   }

   return traces;
}

std::vector<MCTrace> vfm::StaticHelper::extractMCTracesFromNusmvFile(const std::string& path, const TraceExtractionMode mode)
{
   return extractMCTracesFromNusmv(readFile(path), mode);
}

std::string vfm::StaticHelper::serializeMCTraceNusmvStyle(const MCTrace& trace, const bool print_unchanged_values)
{
   std::string s = "Trace Description: vfm emulated trace\nTrace Type: vfm emulation\n";
   VarVals currentvals{};

   for (const auto& state : trace.getConstTrace()) {
      s += "  -> State: " + state.first + " <-\n";

      for (const auto& varVal : state.second) {
         if (print_unchanged_values || !currentvals.count(varVal.first) || currentvals[varVal.first] != varVal.second)
            s += "    " + varVal.first + " = " + varVal.second + "\n";

         if (!print_unchanged_values) currentvals[varVal.first] = varVal.second;
      }
   }

   return s;
}

std::vector<MCTrace> vfm::StaticHelper::extractMCTracesFromKratos(const std::string& cexp_string)
{
   static const std::set<std::string> DATATYPES_TO_REMOVE{ {"int", "bool", "enum.0", "enum.1", "enum.2", "enum.3", "enum.4"} };
   static const std::string ASSIGN_PREFIX = "(assign (var ";
   static const std::string ASSIGN_PREFIX_A = "(assign ";
   static const std::string CALL_PREFIX = "(call ";
   static const std::string RETURN_VARIABLE_NAME = "ret"; // TODO: Should be something more distinct, so it doesn't interfere with some actual variable.
   static constexpr int INDENT_STEP = 2;

   MCTrace trace{};
   std::stack<std::string> return_variables{};
   const auto lines = StaticHelper::split(cexp_string, "\n");
   int pc = 1;
   int indent_last = 0;

   for (auto line : lines) {
      int indent = (line.size() - ltrimAndReturn(line).size()) / INDENT_STEP;

      for (int i = 0; i < indent_last - indent; i++) {
         if (!return_variables.empty()) {
            return_variables.pop();
         }
      }

      indent_last = indent;

      StaticHelper::trim(line);

      if (StaticHelper::stringStartsWith(line, ASSIGN_PREFIX) || StaticHelper::stringStartsWith(line, ASSIGN_PREFIX_A)) {
         std::string processed = StaticHelper::replaceAll(line, ASSIGN_PREFIX, ""); // Remove all...
         processed = StaticHelper::replaceAll(processed, ASSIGN_PREFIX_A, "");      // Remove all...
         processed.pop_back();                                                      // ...except the actual variable and the assigned constant.
         processed = StaticHelper::replaceAll(processed, "(const", "");             // These steps are highly dependent on the current Kratos output.
         processed = StaticHelper::replaceAll(processed, "|", "");

         for (const auto& el : DATATYPES_TO_REMOVE) {
            processed = StaticHelper::replaceAll(processed, el + ")", "");
         }

         StaticHelper::trim(processed);

         std::string var;
         std::string val;

         if (StaticHelper::stringEndsWith(processed, ")")) {
            int end_index = processed.size() - 1;
            int begin_index = StaticHelper::findMatchingBegTagLevelwise(processed, "(", ")", end_index);
            
            assert(begin_index >= 0);
            
            var = processed.substr(0, begin_index);
            val = processed.substr(begin_index, end_index);
            StaticHelper::trim(var);
            StaticHelper::trim(val);
         }
         else {
            const auto pair = StaticHelper::splitOnWhiteSpaces(processed); // TODO: Probably shiftable somehow to just one case.
            assert(pair.size() == 2);
            var = pair[0];
            val = pair[1];
         }

         VarVals full_state;

         if (var != RETURN_VARIABLE_NAME) {
            full_state.insert({ var, val });
            trace.addTraceStep({ "pc_" + std::to_string(pc), full_state }); // Introduce individual state for each var change.
         }
         else {
            std::cout << "Found return variable '" + RETURN_VARIABLE_NAME + "' in line '" + std::to_string(pc)
               + "'. Setting corresponding upper-level variable '" + return_variables.top() + "' to " + val + ".\n";

            full_state.insert({ return_variables.top(), val });
            trace.addTraceStep({ "pc_" + std::to_string(pc), full_state }); // Introduce individual state for each var change.
         }
      }
      else if (StaticHelper::stringStartsWith(line, CALL_PREFIX)) { // We only need to find the return variable name and put it on the stack.
         //(call (const | checkLCConditionsFastLane | (fun() (int))) (var | ret | int))                                     <<<=== ret
         //(call (const | rndet | (fun(int int) (int))) (const | -8 | int) (const | 6 | int) (var | veh___609___.a | int))  <<<=== veh___609___.a
         //(call | rndet | (const | 0 | int) (const | 70 | int) (var | veh___609___.v | int))                               <<<=== veh___609___.a

         auto tokens = *StaticHelper::tokenizeSimple(line, 1000000, true);

         for (const auto& token : tokens) {
            if (!DATATYPES_TO_REMOVE.count(token) && StaticHelper::isValidVfmVarName(token)) {
               return_variables.push(token);
               break;
            }
         }
      }

      pc++;
   }

   return trace.empty() ? std::vector<MCTrace>{} : std::vector<MCTrace>{ trace };
}

std::string StaticHelper::readFile(const std::string& path, const bool from_utf16)
{
   std::string ce_raw{};

    if (!StaticHelper::existsFileSafe(path)) {
       //Failable::getSingleton()->addNote("File '" + path + "' not found. Returning empty string.");
       return "";
    }

   if (from_utf16) {
      std::ifstream fin(path, std::ios::binary);
      fin.seekg(0, std::ios::end);
      size_t size = (size_t)fin.tellg();

      //skip BOM
      fin.seekg(2, std::ios::beg);
      size -= 2;

      std::u16string u16((size / 2) + 1, '\0');
      fin.read((char*)&u16[0], size);

      ce_raw = std::wstring_convert<
         std::codecvt_utf8_utf16<char16_t>, char16_t>{}.to_bytes(u16);
   }
   else {
      std::ifstream input(path);

      if (input.good()) {
         std::stringstream sstr;
         while (input >> sstr.rdbuf());
         ce_raw = sstr.str();
      } else {
         Failable::getSingleton()->addError("File '" + StaticHelper::absPath(path) + "' could not be read. The bits [good|eof|fail|bad] are ["
            + std::to_string(input.good())
            + std::to_string(input.eof())
            + std::to_string(input.fail())
            + std::to_string(input.bad())
            + "].");
      }
   }

   return ce_raw;
}

std::vector<MCTrace> vfm::StaticHelper::extractMCTracesFromKratosFile(const std::string& path, const bool from_utf16)
{
   return extractMCTracesFromKratos(readFile(path, from_utf16));
}

std::string vfm::StaticHelper::serializeMCTraceKratosStyle(const MCTrace& trace)
{
   return "TODO: Sorry, you haven't implemented this function, yet.";
}

std::string vfm::StaticHelper::timeStamp()
{
   const std::time_t timestamp{ std::time(nullptr) };
   const std::string timestamp_str{ std::string(std::asctime(std::localtime(&timestamp))) };
   return timestamp_str.substr(0, timestamp_str.size() - 1);
}

std::string vfm::StaticHelper::extractSeries(const MCTrace& trace, const std::vector<std::string>& variables)
{
   std::string s = "t";
   int cnt = 0;
   int time_stamp = 0;
   std::map<std::string, std::string> current_values;


   for (const auto& var : variables) {
      s += ";" + var;
   }

   s += "\n";

   for (const auto& trace_point : trace.getConstTrace()) {
      auto& state = trace_point.second;

      for (const auto& varval : state) {
         std::string current_var = varval.first;
         std::string current_val = varval.second;

         if (current_var == "Array.CurrentState" && current_val == "state_EnvironmentModel") {
            s += std::to_string(time_stamp++);

            for (const auto& var : variables) {
               if (current_values.count(var)) {
                  s += "; " + current_values.at(var);
               }
               else {
                  s += "; " + std::string("-");
               }
            }

            s += "\n";
            current_values.clear();
         }
         else {
            for (const auto& var : variables) {
               if (current_var == var) {
                  current_values[current_var] = current_val;
               }
            }
         }
      }
   }

   return s;
}

std::string vfm::StaticHelper::fromPascalToCamelCase(const std::string& line)
{
   bool active = true;
   std::string new_line;

   for(int i = 0; line[i] != '\0'; i++) {
      if(std::isalpha(line[i])) {
         if(active) {
            new_line += std::toupper(line[i]);
            active = false;
         } else {
            new_line += std::tolower(line[i]);
         }
      } else if(line[i] == '_') {
         active = true;
      }
   }

   return new_line;
}

int vfm::StaticHelper::greatestCommonDivisor(const int a, const int b)
{
#if __cplusplus >= 201703L
   return std::gcd(a, b);
#else
   return a == 0 ? b : greatestCommonDivisor(b % a, a);
#endif
}

int vfm::StaticHelper::leastCommonMultiple(const int a, const int b)
{
#if __cplusplus >= 201703L
   return std::lcm(a, b);
#else
   return std::abs(a * b) / greatestCommonDivisor(a, b);
#endif
}

int vfm::StaticHelper::accumulate(const std::vector<int> vec, const std::function<int(int, int)>& function)
{
   return vec.size() <= 2
      ? function(vec[0], vec[vec.size() - 1])
      : function(vec[0], accumulate(std::vector<int>(vec.begin() + 1, vec.end()), function));
}

int vfm::StaticHelper::greatestCommonDivisor(const std::vector<int>& vec)
{
   return accumulate(vec, [&](int a, int b) { return StaticHelper::greatestCommonDivisor(a, b); });
}

int vfm::StaticHelper::leastCommonMultiple(const std::vector<int>& vec)
{
   return accumulate(vec, [&](int a, int b) { return StaticHelper::leastCommonMultiple(a, b); });
}

bool vfm::StaticHelper::isVariableNameDenotingArray(const std::string& var_name)
{
   return var_name.size() > 1 && var_name[0] >= NATIVE_ARRAY_PREFIX_FIRST && var_name[0] <= NATIVE_ARRAY_PREFIX_LAST;
}

std::string vfm::StaticHelper::makeVarNameNuSMVReady(const std::string& var_name)
{
   auto new_name = isVariableNameDenotingArray(var_name) ? "Array" + var_name : var_name;

   if (StaticHelper::stringContains(new_name, "::")) {
      int pos = new_name.find("::");
      new_name = new_name.substr(pos + 2);
   }

   return new_name;
}

std::string vfm::StaticHelper::serializeBracketStructure(const std::shared_ptr<BracketStructure> bracket_structure, const std::string& opening_bracket, const std::string& closing_bracket, const std::string& delimiter)
{
   if (bracket_structure) {
      std::string s = bracket_structure->content_;
      bool first = true;

      if (bracket_structure->children_.size() > 1 || bracket_structure->content_.empty()) {
         s += opening_bracket;
      }

      for (const auto& child : bracket_structure->children_) {
         if (child != nullptr) {
            if (!first) {
               s += delimiter;
            }

            s += serializeBracketStructure(child, opening_bracket, closing_bracket, delimiter);
            first = false;
         }
      }

      if (bracket_structure->children_.size() > 1 || bracket_structure->content_.empty()) {
         s += closing_bracket;
      }

      return s;
   }

   return "";
}

std::shared_ptr<BracketStructure> vfm::StaticHelper::extractArbitraryBracketStructure(
   const std::string& bracket_structure_as_string, 
   const std::string& opening_bracket,
   const std::string& closing_bracket,
   const std::string& delimiter,
   const std::vector<std::string>& ignoreBegTags,
   const std::vector<std::string>& ignoreEndTags,
   const bool is_global_call)
{
   assert(ignoreBegTags.size() == ignoreEndTags.size());

   auto overall_structure = std::make_shared<BracketStructure>("", std::vector<std::shared_ptr<BracketStructure>>());
   std::vector<bool> within;

   const auto within_any = [&within]() {
      for (const auto& el : within) if (el) return true;
      return false;
   };

   const auto update_within = [&within, &ignoreBegTags, &ignoreEndTags, &bracket_structure_as_string](const int i) {
      while (within.size() < ignoreBegTags.size()) {
         within.push_back(false);
      }

      for (int j = 0; j < ignoreBegTags.size(); j++) {
         if (ignoreBegTags[j] == bracket_structure_as_string.substr(i, ignoreBegTags[j].size())) {
            if (ignoreBegTags[j] == ignoreEndTags[j]) {
               within[j] = !within[j];
            }
            else {
               within[j] = true;
            }
            return (int) ignoreBegTags[j].size(); // Assuming no two equal tags in the list.
         }
         else if (ignoreEndTags[j] == bracket_structure_as_string.substr(i, ignoreEndTags[j].size())) {
            within[j] = false;
            return (int) ignoreEndTags[j].size(); // Assuming no two equal tags in the list.
         }
      }

      return 0;
   };

   int next_inc = 1;
   std::string current_element;
   for (int i = 0; i < bracket_structure_as_string.size(); i += next_inc) {
      int next_inc = update_within(i);

      if (!next_inc) {
         next_inc = 1;

         if (within_any()) {
            current_element += bracket_structure_as_string[i];
         } else {
            if (bracket_structure_as_string.substr(i, opening_bracket.size()) == opening_bracket) {
               int pos_beg = i;
               int pos_end = findMatchingEndTagLevelwise(bracket_structure_as_string, pos_beg, opening_bracket, closing_bracket, ignoreBegTags, ignoreEndTags);
               std::string part = bracket_structure_as_string.substr(pos_beg + opening_bracket.size(), pos_end - pos_beg - opening_bracket.size());
               overall_structure->children_.push_back(extractArbitraryBracketStructure(part, opening_bracket, closing_bracket, delimiter, ignoreBegTags, ignoreEndTags, false));
               i = pos_end + closing_bracket.size() + 1;
               current_element.clear();
            }
            else if (bracket_structure_as_string.substr(i, delimiter.size()) == delimiter) {
               overall_structure->children_.push_back(std::make_shared<BracketStructure>(current_element, std::vector<std::shared_ptr<BracketStructure>>{}));
               i += delimiter.size();
               current_element.clear();
            }
            else {
               current_element += bracket_structure_as_string[i];
            }
         }
      }
   }

   if (!current_element.empty()) {
      overall_structure->children_.push_back(std::make_shared<BracketStructure>(current_element, std::vector<std::shared_ptr<BracketStructure>>{}));
   }

   return is_global_call ? overall_structure->children_.at(0) : overall_structure;
}

void vfm::StaticHelper::preprocessCppConvertCStyleArraysToVars(std::string& program)
{
   // TODO: What about stuff in "quotes"?
   std::regex reg1("\\s*\\]");
   std::regex reg2("\\]\\s*");
   std::regex reg3("\\s*\\[");
   std::regex reg4("\\[\\s*");
   program = std::regex_replace(program, reg1, "]");
   program = std::regex_replace(program, reg2, "]");
   program = std::regex_replace(program, reg3, "[");
   program = std::regex_replace(program, reg4, "[");

   program = StaticHelper::replaceAll(StaticHelper::replaceAll(program, "[", ARRAY_INDEX_DENOTER_OPEN), "]", ARRAY_INDEX_DENOTER_CLOSE);
}

void vfm::StaticHelper::preprocessCppConvertArraysToCStyle(std::string& program, const std::set<std::string>& possible_array_type_names)
{
   std::string array_type_name;
   int begin_array = indexOfFirstInnermostBeginBracket(
      program,
      "<",
      ">",
      { "\"" },
      { "\"" },
      [&possible_array_type_names, &array_type_name](const std::string& str, const int index) {
         int dummy_ind = index - 1;
         auto single_token = tokenize(
            str, 
            *SingletonFormulaParser::getLightInstance(), 
            dummy_ind, 
            1,
            true);
         assert(single_token && single_token->size() >= 1); // Assuming a program cannot start with "<".
         array_type_name = single_token->at(0);
         return possible_array_type_names.count(array_type_name);
      });

   if (begin_array >= 0) {
      int end_array = findMatchingEndTagLevelwise(program, begin_array, "<", ">", { "\"" }, { "\"" });
      int real_begin = begin_array;
      int prefix_length = 0;

      while (program.substr(real_begin, array_type_name.size()) != array_type_name) {
         real_begin--;
         prefix_length++;
      }

      begin_array++;
      std::string inner_part = program.substr(begin_array, end_array - begin_array);
      std::string full_decl = program.substr(real_begin, end_array - begin_array + prefix_length + 2);
      std::string before_part = program.substr(0, real_begin);
      std::string after_part = program.substr(end_array + 1);
      
      const std::set<char> E_O_ARRAY_DENOTER = { '=', '{', ';', '[', ',', ')' }; // Arr<T, S> x; Arr<T, S> x{..}; Arr<T, S> x={...};

      auto elements = StaticHelper::split(inner_part, ',');
      auto type = elements[0];
      std::string size = "-1"; // Denoter if no size is given (like in "std::vector<Type>").
      StaticHelper::trim(type);

      if (elements.size() > 1) { // Use actual size (as in "std::array<Type, Size>").
         size = elements[1];
         size = StaticHelper::removeWhiteSpace(size); // Needed for later when we tokenize on white spaces.
      }

      program = before_part + type + after_part;
      int diff = full_decl.size() - type.size();

      int i = real_begin - 1;

      while (i > 0 && std::isspace(program[i])) {
         i--; // Find last non-space character before current type name.
      }

      if (program[i] == '<') { // We're still within a nested array.
         // Look for the first '>' after which the first non-space character is NOT ',' or '>'. This must be the end of the array.
         // As in: std::array<std::array<int, 3>, 2> a2;

         char last = ' ';
         for (; i < program.size(); i++) {
            if (program[i] == '>') {
               i++;
               while (i < program.size() && std::isspace(program[i])) {
                  i++; // Find first non-space character after end of current array level.
               }

               if (program[i] != ',' && program[i] != '>') {
                  break;
               }
            }
         }
      }

      i++;

      // We're not within the array anymore. Then jump over the variable to place the array brackets:
      // ...a2[...];
      for (; i < program.size() && (!E_O_ARRAY_DENOTER.count(program[i])); i++);

      std::string from_beg = program.substr(0, i);
      std::string to_end = program.substr(i);

      program = from_beg + "[" + size + "]" + to_end;

      preprocessCppConvertArraysToCStyle(program, possible_array_type_names);
   }
}

void vfm::StaticHelper::preprocessCppConvertElseIfToPlainElse(std::string& program)
{
   std::string condition;

   int begin_if_else = indexOfFirstInnermostBeginBracket(
      program, 
      "{", 
      "}", 
      { "\"" }, 
      { "\"" }, 
      [&condition](const std::string& str, const int index) {
      condition = ")";
      int dummy_ind = index;
      auto tokens = tokenize(
         str,
         *SingletonFormulaParser::getLightInstance(),
         dummy_ind,
         std::numeric_limits<int>::max(),
         true, 
         DEFAULT_REGEX_FOR_TOKENIZER, DEFAULT_IGNORE_CLASS_FOR_TOKENIZER, 
         true, 
         true,
         AkaTalProcessing::none,
         {}, 
         false, 
         [](const std::vector<std::string> tokens_so_far) { 
            return tokens_so_far.size() > 1 && tokens_so_far.at(tokens_so_far.size() - 2) == "if"; // Note that qutation marks are not handled, assuming if (...) condition has no literal strings.
         });

      int bracket_count = 1;
      int i = 2;

      for (; i < tokens->size() && bracket_count > 0; i++) {
         condition = tokens->at(i) + " " + condition;
         if (tokens->at(i) == ")") bracket_count++;
         if (tokens->at(i) == "(") bracket_count--;
      }

      if (bracket_count == 0 && tokens->size() > i + 1) {
         return tokens->at(i) == "if" && tokens->at(i + 1) == "else";
      }
      else {
         return false;
      }
   });

   if (begin_if_else >= 0) {
      int end_if_else = findMatchingEndTagLevelwise(program, begin_if_else, "{", "}", { "\"" }, { "\"" });
      int overall_begin = begin_if_else;
      int dummy = 0;
      int bracket_count = 0;

      while (program[overall_begin] != ')') overall_begin--; // Find closing bracket of IF condition...
      overall_begin--;

      while (bracket_count >= 0) {                           // ...go backwards over the condition...
         if (program[overall_begin] == '(') {
            bracket_count--;                                 // ...keep track of nesting level...
         }
         else if (program[overall_begin] == ')') {
            bracket_count++;
         }

         overall_begin--;
      }

      while (program[overall_begin] != 'l') overall_begin--; // ...until finding the beginning of the next "else" on the correct level.
      while (program[overall_begin] != 'e') overall_begin--;

      std::string if_else_body = program.substr(begin_if_else + 1, end_if_else - begin_if_else + 1);
      std::string before_if_else = program.substr(0, overall_begin - 1);
      std::string after_if_else = program.substr(end_if_else + 1);
      std::string rest = "";
      
      int temp = 0;

      bool has_elseif_bracket = false;
      for (; dummy < after_if_else.size(); dummy++) {
         if (after_if_else[dummy] == '(') {
            has_elseif_bracket = true;
            break;
         }
         else if (after_if_else[dummy] == '{') {
            break;
         }
      }

      int end_else_if_blocks = has_elseif_bracket ? findMatchingEndTagLevelwise(after_if_else, dummy, "(", ")", { "\"" }, { "\"" }) : dummy;
      auto plain_between_part = removeWhiteSpace(after_if_else.substr(temp, dummy - temp));

      while (plain_between_part == "elseif" || plain_between_part == "else") {
         end_else_if_blocks = findMatchingEndTagLevelwise(after_if_else, dummy, "{", "}", { "\"" }, { "\"" });
         int opening_brace_pos = dummy;
         dummy = end_else_if_blocks;
         temp = end_else_if_blocks + 1;

         bool has_elseif_bracket = false;
         for (; dummy < after_if_else.size(); dummy++) {
            if (after_if_else[dummy] == '(') {
               has_elseif_bracket = true;
               break;
            }
            else if (after_if_else[dummy] == '{') {
               break;
            }
         }

         if (has_elseif_bracket) {
            end_else_if_blocks = findMatchingEndTagLevelwise(after_if_else, dummy, "(", ")", { "\"" }, { "\"" });
         }
         else {
            end_else_if_blocks = findMatchingEndTagLevelwise(after_if_else, opening_brace_pos, "{", "}", { "\"" }, { "\"" });
            temp = end_else_if_blocks + 1;
         }

         plain_between_part = removeWhiteSpace(after_if_else.substr(temp, dummy - temp));
      }

      if (end_else_if_blocks >= 0) {
         rest = after_if_else.substr(0, temp);
         after_if_else = after_if_else.substr(temp);
      }

      std::string nested_if_else = " else {\nif " + condition + " {\n" + if_else_body + rest + "\n}\n";

      program = before_if_else + nested_if_else + after_if_else;

      preprocessCppConvertElseIfToPlainElse(program);
   }
}

void vfm::StaticHelper::preprocessCppConvertAllSwitchsToIfs(std::string& program)
{
   int begin_switch = indexOfFirstInnermostBeginBracket(
      program, 
      "{", 
      "}", 
      { "\"" }, 
      { "\"" }, 
      [](const std::string& str, const int index) {
         int dummy_ind = index;
         auto tokens = tokenize(str, *SingletonFormulaParser::getLightInstance(), dummy_ind, 5, true);
         return tokens->size() >= 5 && tokens->at(4) == "switch" && tokens->at(3) == "(" && tokens->at(1) == ")";
      });

   if (begin_switch >= 0) {
      int end_switch = findMatchingEndTagLevelwise(program, begin_switch, "{", "}", { "\"" }, { "\"" });

      while (program[begin_switch] != '(') begin_switch--;
      while (program[begin_switch] != 's') begin_switch--;

      std::string switch_block = program.substr(begin_switch, end_switch - begin_switch + 1);
      std::string before_switch = program.substr(0, begin_switch - 1);
      std::string after_switch = program.substr(end_switch + 1);
      
      int pos = 0;
      auto tokens = tokenize(
         switch_block,
         *SingletonFormulaParser::getLightInstance(),
         pos,
         std::numeric_limits<int>::max(),
         false,
         DEFAULT_REGEX_FOR_TOKENIZER,
         DEFAULT_IGNORE_CLASS_FOR_TOKENIZER,
         false,
         false,
         AkaTalProcessing::none,
         {},
         true);

      program = before_switch + convertSingleFlatSwitchToIf(extractCasesFromFlatSwitchBlock(*tokens)) + after_switch;

      preprocessCppConvertAllSwitchsToIfs(program);
   }
}

const auto SPECIAL_REGEX_FOR_TOKENIZER = std::vector<std::regex>{
   std::regex("[a-zA-Z_" + SYMB_REF + "'" + "0-9.]"), // The missing colon sign here...
   std::regex("\\s"),
   std::regex("[:?\\&\\|<>=!~+\\*/#%^]"), // ...and the additional colon sign here are the only differences to DEFAULT_REGEX_FOR_TOKENIZER.
   std::regex("[-]"),
   std::regex("[(]"),
   std::regex("[)]"),
   std::regex("[{]"),
   std::regex("[}]"),
   std::regex("[,]"),
   std::regex("[;]"),
   std::regex("[\"]"),
};

void vfm::StaticHelper::preprocessSMVConvertAllSwitchsToIfs(std::string& program)
{
   std::string switch_var_name{};
   const std::string beg{"case"};
   const std::string end{"esac;"};

   int begin_switch = indexOfFirstInnermostBeginBracket(
      program, 
      beg, 
      end, 
      {},
      {}, 
      [&beg](const std::string& str, const int index) {
         return (index == 0 || std::isspace(str[index - 1]))
            && index < str.size() - 1 - beg.size() && std::isspace(str[index + beg.size() + 1]);
      });

   if (begin_switch >= 0) {
      int pos_dummy = begin_switch;
      auto backward_tokens = StaticHelper::tokenize(
         program,
         *SingletonFormulaParser::getLightInstance(),
         pos_dummy,
         10000, // TODO: Can be inefficient! Better use lambda to specify exactly when to stop.
         true,
         SPECIAL_REGEX_FOR_TOKENIZER
      );

      for (int i = 0; i < backward_tokens->size() - 1; i++) {
         if (backward_tokens->at(i) == ":=") {
            switch_var_name = backward_tokens->at(i + 1);
            break;
         }
      }

      if (switch_var_name.empty()) {
         Failable::getSingleton()->addError("Switch var name not found for case block beginning at position " + std::to_string(begin_switch) + " of SMV program:\n" + program);
         switch_var_name = "#DUMMY#";
      }
      else {
         Failable::getSingleton()->addNote("Found switch variable named '" + switch_var_name + "'.");
      }

      int end_switch = findMatchingEndTagLevelwise(program, begin_switch, "case", "esac;", {}, {});

      std::string switch_block = program.substr(begin_switch, end_switch - begin_switch + 5);
      std::string before_switch = program.substr(0, begin_switch - 1);
      std::string after_switch = program.substr(end_switch + 5);

      program = before_switch + convertSingleFlatSwitchToIf(extractCasesFromFlatSwitchBlockSMV(switch_var_name, switch_block), SwitchCaseStyle::SMV) + after_switch;

      preprocessSMVConvertAllSwitchsToIfs(program);
   }
}

std::string vfm::StaticHelper::convertSingleFlatSwitchToIf(const SwitchBlock& cases, const SwitchCaseStyle style)
{
   std::string if_block;
   std::string optional_else = "";

   for (int i = 0; i < cases.second.first.size(); i++) {
      auto& case_desc = cases.second.first.at(i);
      std::string block_contents = case_desc.first.second;

      if (!StaticHelper::isEmptyExceptWhiteSpaces(block_contents)) {
         int j = i;
         auto case_desc2 = cases.second.first.at(j);
         std::string additional;

         while (!case_desc2.second && j < cases.second.first.size() - 1) {
            Failable::getSingleton()->addWarning("Switch '" + cases.first + "' case '" + case_desc2.first.first + "' has no break at the end. I will assume this is intended.");
            j++;
            case_desc2 = cases.second.first.at(j);
            additional += case_desc2.first.second;
         }

         if (!case_desc2.second) { // The default case has to be added as well.
            Failable::getSingleton()->addWarning("Switch '" + cases.first + "' case '" + case_desc2.first.first + "' has no break at the end. I will assume this is intended.");
            if (cases.second.second) {
               additional += cases.second.second->first.second;
            }
         }

         block_contents += additional;

         if_block += optional_else + " if (" + (style == SwitchCaseStyle::SMV ? "" : cases.first + " == ") + case_desc.first.first + ") {\n" 
            + (style == SwitchCaseStyle::SMV ? cases.first + " = " : "") + block_contents
            + "\n}\n";
         optional_else = "else";
      }
   }

   if (cases.second.second) {
      if (!cases.second.second->second) {
         Failable::getSingleton()->addWarning("Switch '" + cases.first + "' case '" + cases.second.second->first.first + "' has no break at the end. I will assume this is intended.");
      }

      std::string block_contents = cases.second.second->first.second;
      if (!StaticHelper::isEmptyExceptWhiteSpaces(block_contents)) {
         if_block += optional_else + " {\n"
            + (style == SwitchCaseStyle::SMV ? cases.first + " = " : "") + block_contents
            + "\n}\n";
      }
   }

   return if_block;
}

std::string extractCaseCode(const std::vector<std::string>& switch_block_tokens, int& j, bool& has_break)
{
   std::string case_code;
   has_break = false;
   int count_braces = 0;
   bool has_outer_braces = false;
   bool is_in_quotes = false;

   while (switch_block_tokens.at(j) == ":") {
      j++;
   }

   while (switch_block_tokens.at(j) == "{") {
      has_outer_braces = true;
      count_braces++;
      j++; // Ignore leading "{"'s. If more "}" than "{", we ran over a default case without break.
   }

   for (; is_in_quotes || j < switch_block_tokens.size() && switch_block_tokens.at(j) != "case" && switch_block_tokens.at(j) != "break" && switch_block_tokens.at(j) != "default" && switch_block_tokens.at(j) != "default:" && count_braces >= 0; j++) {
      if (switch_block_tokens.at(j) == "\"") {
         is_in_quotes = !is_in_quotes;
      }

      if (!is_in_quotes) {
         if (has_outer_braces && switch_block_tokens.at(j) == "}" && count_braces == 1) {
            break;
         }

         if (switch_block_tokens.at(j) == "{") {
            count_braces++;
         }
         else if (switch_block_tokens.at(j) == "}") {
            count_braces--;
         }
      }

      if (count_braces >= 0) {
         case_code += switch_block_tokens.at(j) + " ";
      }
   }

   if (j < switch_block_tokens.size() && switch_block_tokens.at(j) == "break") {
      has_break = true;
   }
   else {
      j--; // If it's "case" then we need to go one back to not miss the next one in the outer method.
   }

   return case_code;
}

std::string vfm::StaticHelper::floatToStringNoTrailingZeros(float value) {
   std::string str = std::to_string(value);
   str.erase(str.find_last_not_of('0') + 1, std::string::npos);
   if (str.back() == '.') {
      str.pop_back();
   }
   return str;
}

std::string vfm::StaticHelper::zeroPaddedNumStr(const long num, const int str_length)
{
   std::stringstream ss;
   ss << std::setw(str_length) << std::setfill('0') << num;
   return ss.str();
}

vfm::SwitchBlock vfm::StaticHelper::extractCasesFromFlatSwitchBlock(const std::vector<std::string>& switch_block_tokens)
{
   assert(switch_block_tokens.size() >= 6 && "Too few tokens for vaild switch block definition.");

   size_t index_begin = 0;
   for (; index_begin < switch_block_tokens.size() && switch_block_tokens.at(index_begin) != "switch"; index_begin++);
   assert(switch_block_tokens.at(index_begin) == "switch" && "No switch keyword found in switch block.");

   size_t index_end = index_begin + 5;
   int count = 1;
   for (; index_end < switch_block_tokens.size() && count != 0; index_end++) {
      if (switch_block_tokens.at(index_end) == "{") count++;
      if (switch_block_tokens.at(index_end) == "}") count--;
   }
   index_end--;

         std::string ss;
         for (const auto& s : switch_block_tokens) ss += s;
         std::string tt;
         for (int i = 5 + index_begin; i < index_end; i++) tt += switch_block_tokens.at(i);

   SwitchBlock cases{ switch_block_tokens.at(2 + index_begin), {} };

   for (size_t i = 5 + index_begin; i < index_end; i++)  {
      auto t = switch_block_tokens.at(i);
      bool has_break;
      int j;

      if (t == "case") {
         std::string case_condition;
         j = i + 1;

         do {
            case_condition += switch_block_tokens.at(j);
         } while (!StaticHelper::stringContains(switch_block_tokens.at(j++), ":"));

         if (case_condition[case_condition.size() - 1] == ':') {
            case_condition = case_condition.substr(0, case_condition.size() - 1);
         }

         std::string case_code = extractCaseCode(switch_block_tokens, j, has_break);

         cases.second.first.push_back({ { case_condition, case_code }, has_break });

         i = j;
      }
      else if (t == "default" || t == "default:") {
         j = i + 1 + (t[t.size() - 1] != ':');
         std::string case_code = extractCaseCode(switch_block_tokens, j, has_break);
         cases.second.second = std::make_shared<CaseWithBreakInfo>(CaseWithBreakInfo{ { "#default#", case_code }, has_break });
      }
   }

   return cases;
}

SwitchBlock vfm::StaticHelper::extractCasesFromFlatSwitchBlockSMV(const std::string& switch_var, const std::string& switch_block_chunk)
{
   SwitchBlock cases{ switch_var, {} };
   std::string prog{ StaticHelper::replaceAll(StaticHelper::replaceAll(switch_block_chunk, "case", ""), "esac;", "") };

   auto cases_str{ StaticHelper::split(prog, ";") };

   for (const auto& case_str_raw : cases_str) {
      std::string case_str{ StaticHelper::trimAndReturn(case_str_raw) };

      if (!StaticHelper::isEmptyExceptWhiteSpaces(case_str)) {
         auto split = StaticHelper::split(case_str, ":", // Split on "case" colons, but not on conditional "?:" colons.
            [](const std::string& prog, const int pos) -> bool {
               int count = 0;
               for (int i = pos - 1; i >= 0; i--) {
                  // if (prog[i] == ':') return false; // TODO: This might be the better solution, since I cannot imagine another colon on the LEFT side of the "case" colon.
                  if (prog[i] == '(') count++; // Count brackets, nested colons can't be it.
                  if (prog[i] == ')') count--;
               }

               return !count;
            }
         );

         if (split.size() != 2) {
            Failable::getSingleton()->addError("Malformed switch block on variable '" + switch_var + "':\n" + switch_block_chunk);
         }

         if (split[0] == "TRUE") {
            cases.second.second = std::make_shared<CaseWithBreakInfo>(std::pair<std::pair<std::string, std::string>, bool>{ { split[0], split[1] }, true });
         }
         else {
            cases.second.first.push_back({ { split[0], split[1] }, true });
         }
      }
   }

   return cases;
}

std::shared_ptr<std::vector<std::string>> vfm::StaticHelper::getChunkTokens(const std::string& chunk, const AkaTalProcessing aka_processing)
{
   int num_temp = 0;

   auto tokens = StaticHelper::tokenize(chunk, *SingletonFormulaParser::getLightInstance(), num_temp,
      std::numeric_limits<int>::max(), false, DEFAULT_REGEX_FOR_TOKENIZER, DEFAULT_IGNORE_CLASS_FOR_TOKENIZER, false, true, aka_processing, {}, true
   );

   return tokens;
}

SwitchBlock vfm::StaticHelper::extractCases(const std::string& switch_block_chunk)
{
   std::string new_switch_block_chunk;
   int last_begin = 0;
   int index_beg = switch_block_chunk.find("{", switch_block_chunk.find("case", switch_block_chunk.find("{", switch_block_chunk.find("switch")))); // First case inner part of first switch.
   int index_end = findMatchingEndTagLevelwise(switch_block_chunk, index_beg, "{", "}", { "\"" }, { "\"" });

   while (index_beg >= 0 && index_end >= 0) {
      auto before_str = switch_block_chunk.substr(last_begin, index_beg);
      auto case_str = switch_block_chunk.substr(index_beg, index_end - index_beg + 1);
      auto after_str = switch_block_chunk.substr(index_end + 1);
      
      preprocessCppConvertAllSwitchsToIfs(case_str);
      preprocessCppConvertElseIfToPlainElse(case_str);

      new_switch_block_chunk += before_str + case_str + after_str;

      last_begin = index_beg;
      index_beg = switch_block_chunk.find("{", switch_block_chunk.find("case", index_end));
      index_end = findMatchingEndTagLevelwise(switch_block_chunk, index_beg, "{", "}", { "\"" }, { "\"" });
   }

   auto tokens = getChunkTokens(new_switch_block_chunk, AkaTalProcessing::none);

   return extractCasesFromFlatSwitchBlock(*tokens);
}

std::string vfm::StaticHelper::tokensAsString(const std::vector<std::string>& tokens, const bool remove_trailing_delimiter, const std::string& delimiter)
{
   std::string tokens_as_string;

   for (const auto& token : tokens) {
      tokens_as_string += token + delimiter;
   }

   if (remove_trailing_delimiter && StaticHelper::stringEndsWith(tokens_as_string, delimiter)) {
      tokens_as_string = tokens_as_string.substr(0, tokens_as_string.size() - delimiter.size());
   }

   return tokens_as_string;
}

// https://stackoverflow.com/a/70237726
// This function is licensed under CC-BY-SA-4.0 which is a copy-left license
// (https://creativecommons.org/licenses/by-sa/4.0/deed.en)!
size_t vfm::StaticHelper::levensteinDistance(const std::string& a, const std::string& b)
{
   std::vector<size_t> d_((a.size() + 1) * (b.size() + 1), size_t(-1));
   auto d = [&](size_t ia, size_t ib) -> size_t & {
      return d_[ia * (b.size() + 1) + ib];
   };

   std::function<size_t(size_t, size_t)> LevensteinInt =
      [&](size_t ia, size_t ib) -> size_t {
      if (d(ia, ib) != size_t(-1))
         return d(ia, ib);
      size_t dist = 0;
      if (ib >= b.size())
         dist = a.size() - ia;
      else if (ia >= a.size())
         dist = b.size() - ib;
      else if (a[ia] == b[ib])
         dist = LevensteinInt(ia + 1, ib + 1);
      else
         dist = 1 + std::min(
            std::min(
               LevensteinInt(ia, ib + 1),
               LevensteinInt(ia + 1, ib)
            ), LevensteinInt(ia + 1, ib + 1)
         );
      d(ia, ib) = dist;
      return dist;
   };

   return LevensteinInt(0, 0);
}

std::tuple<size_t, size_t> vfm::StaticHelper::findClosest(const std::vector<std::string>& strs, const std::string& query)
{
   size_t minv = size_t(-1), mini = size_t(-1);
   for (size_t i = 0; i < strs.size(); ++i) {
      size_t const dist = levensteinDistance(strs[i], query);
      if (dist < minv) {
         minv = dist;
         mini = i;
      }
   }

   return std::make_tuple(mini, minv);
}

void vfm::StaticHelper::addNDetDummyFunctions(const std::shared_ptr<FormulaParser> parser, const std::shared_ptr<DataPack> data)
{
   std::string ndet_dummy_code = "\
if (" + MC_INTERACTIVE_MODE_CONSTANT_NAME + " && p_(0) != p_(1)) { \
   print(\"Please resolve nondeterminism between \");\
   printf(p_(0));\
   print(\" and \");\
   printf(p_(1));\
   print(\" for \");\
   Print(literal(p_(0)));\
   print(\" : \");\
   @_temp=fctin();\
   if (_temp < p_(0) || _temp > p_(1)) {\
      printErr(\"Not in range\");\
   };\
   _temp\
} else { \
   ***\
}";

   for (int i = 0; i < 20; i++) { // TODO: We need a solution for functions that ACTUALLY have a flexible number of parameters. (@f is a special case.)
      OperatorStructure op_struct_ndet(
         OutputFormatEnum::prefix,
         AssociativityTypeEnum::left,
         i,
         -1,
         SYMB_NONDET_FUNCTION_FOR_MODEL_CHECKING,
         false,
         false,
         AdditionalOptionEnum::allow_arbitrary_number_of_arguments);
      parser->addDynamicTerm(
         op_struct_ndet,
         StaticHelper::replaceAll(ndet_dummy_code, "***", "if (rand(1), p_(0), p_(1))"),
         data);
   }

   OperatorStructure op_struct_rndet(
      OutputFormatEnum::prefix,
      AssociativityTypeEnum::left,
      2,
      -1,
      SYMB_NONDET_RANGE_FUNCTION_FOR_MODEL_CHECKING,
      false,
      false,
      AdditionalOptionEnum::none);

   data->declareVariable(MC_INTERACTIVE_MODE_CONSTANT_NAME, false);

   parser->addDynamicTerm(
      op_struct_rndet,
      //StaticHelper::replaceAll(ndet_dummy_code, "***", "rand(p_(1)-p_(0)) + p_(0)"),
      StaticHelper::replaceAll(ndet_dummy_code, "***", "p_(0)"),
      data);
}

std::string vfm::StaticHelper::replaceManyTimes(const std::string& str, const std::vector<std::string>& search_list, const std::vector<std::string>& replacements_list)
{
   assert(search_list.size() == replacements_list.size());

   const int highest_index = std::min(search_list.size(), replacements_list.size());

   std::string s = str;

   for (int i = 0; i < highest_index; i++) {
      s = replaceAll(s, search_list[i], replacements_list[i]);
   }

   return s;
}

std::string vfm::StaticHelper::replaceManyTimes(const std::string& str, const std::vector<std::pair<std::string, std::string>>& replacements)
{
   std::string s = str;

   for (const auto& rep : replacements) {
      s = replaceAll(s, rep.first, rep.second);
   }

   return s;
}

bool vfm::StaticHelper::isTermAssemblifiable(const std::string& optor, const std::shared_ptr<FormulaParser> parser_raw)
{
#if defined(ASMJIT_ENABLED)
   auto parser = parser_raw ? parser_raw : std::make_shared<FormulaParser>();
   auto output_level = parser->getStdOutLevel();
   auto error_level = parser->getStdErrLevel();
   parser->setOutputLevels(ErrorLevelEnum::invalid, ErrorLevelEnum::invalid);

   auto jit_runtime = std::make_shared<asmjit::JitRuntime>();

   asmjit::CodeHolder code;
   code.init(jit_runtime->environment());
   asmjit::x86::Compiler cc(&code);
   cc.addFunc(asmjit::FuncSignatureT<float>());
   auto d = std::make_shared<DataPack>();
   auto term = parser->termFactoryDummy(optor);
   parser->setOutputLevels(output_level, error_level);

   return (bool) term->createSubAssembly(cc, d, parser); // TODO: Check why this fails for Set3.
#else
   return false;
#endif
}

void vfm::StaticHelper::listAssemblyStateOfAllTerms(const std::shared_ptr<FormulaParser> parser_raw)
{
   std::set<std::string> entries;
   auto parser = parser_raw ? parser_raw : std::make_shared<FormulaParser>();

   Failable::getSingleton()->addNote("### Here come all the static terms I know of ###");

   for (const auto& optor : parser->getAllOps()) {
      std::string ass = isTermAssemblifiable(optor.first, parser) ? " ASS " : "noass";
      entries.insert("<" + ass + "> " + optor.first);
   }

   for (const auto& entry : entries) {
      Failable::getSingleton()->addNote(entry);
   }

   Failable::getSingleton()->addNote("### EO All the static terms I know of ###");
}

std::string vfm::StaticHelper::safeString(const std::string& arbitrary_string)
{
   if (getSAFE_CHARACTERS_ASCII_LIKE().empty()) fillSafeCharVecs();

   std::string res;

   for (const auto& c : arbitrary_string) {
      res += getSAFE_CHARACTERS_ASCII_LIKE()[(uint8_t) c];
   }

   return res;
}

std::string vfm::StaticHelper::fromSafeString(const std::string& safe_string)
{
   assert(safe_string.size() % 2 == 0);

   if (getSAFE_CHARACTERS_ASCII_LIKE().empty()) fillSafeCharVecs();

   std::string res;

   for (size_t i = 0; i + 1 < safe_string.size(); i += 2) {
      std::string pair = makeString(safe_string[i]) + makeString(safe_string[i + 1]);
      res += getSAFE_CHARACTERS_ASCII_LIKE_REVERSE()[pair];
   }

   return res;
}

void vfm::StaticHelper::fillSafeCharVecs()
{
   const static std::string SAFE_CHAR_SET{ "0123456789abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ" };

   if (!getSAFE_CHARACTERS_ASCII_LIKE().empty()) return;

   getSAFE_CHARACTERS_ASCII_LIKE().resize(256);

   for (size_t j = 0; j < SAFE_CHAR_SET.size(); j++) {
      for (size_t i = j * SAFE_CHAR_SET.size(); i < (j + 1) * SAFE_CHAR_SET.size(); i++) {
         if (i >= getSAFE_CHARACTERS_ASCII_LIKE().size()) {
            return;
         }

         getSAFE_CHARACTERS_ASCII_LIKE()[i] = makeString(SAFE_CHAR_SET.at(j)) + makeString(SAFE_CHAR_SET.at(i - j * SAFE_CHAR_SET.size()));
         getSAFE_CHARACTERS_ASCII_LIKE_REVERSE().insert({ getSAFE_CHARACTERS_ASCII_LIKE()[i], i });
      }
   }
}

std::string vfm::StaticHelper::replaceSpecialCharsHTML_G(const std::string& sString) 
{
   std::string ssString = replaceAll(
      replaceAll(
         replaceAll(
            replaceAll(
               replaceAll(
                  replaceAll(
                     replaceAll(sString, 
                        "ß", "&szlig;"), 
                     "ä", "&auml;"), 
                  "ö", "&ouml;"), 
               "ü", "&uuml;"), 
            "Ä", "&Auml;"), 
         "Ö", "&Ouml;"), 
      "Ü", "&Uuml;");

   return replaceMathHTML(ssString);
}

std::string vfm::StaticHelper::replaceMathHTML(const std::string& sString) {
   std::string ssString = replaceAll(
      replaceAll(
         replaceAll(
            replaceAll(
               replaceAll(
                  replaceAll(
                     replaceAll(
                        replaceAll(
                           replaceAll(
                              replaceAll(
                                 replaceAll(
                                    replaceAll(sString, 
                                       "\\circ", "<span style=\\\"font-style: normal\\\">&#9675;</span>"), 
                                    "\\in", "&isin;"), 
                                 "\\N", "<span style=\"font-style: normal\">&#x2115;</span>"), 
                              "\\cup", "&cup;"), 
                           "\\bigcup", "<span style=\"font-style: normal;font-size:1.3em;\">&cup;</span>"), 
                        "\\cdot", "&middot;"), 
                     "\\alpha", "&alpha;"),
                  "\\beta", "&beta;"), 
               "\\gamma", "&gamma;"), 
            "\\delta", "&delta;"),
         "\\lambda", "&lambda;"), 
      "\\emptyset", "<span style=\"font-style: normal\">&#8709;</span>");

   return ssString;
}

std::string vfm::StaticHelper::printProgress(const std::string& prefix, int progress_step, int total_steps, int letters_length)
{
   float ratio = static_cast<float>(progress_step) / static_cast<float>(total_steps - 1);

   if (std::isnan(ratio)) {
      ratio = 1;
   }

   int x_count = std::min(std::roundf(ratio * letters_length), static_cast<float>(letters_length));
   return "\r" + prefix + " [ " + std::string(x_count, 'X') + std::string(static_cast<size_t>(letters_length - x_count), '-') + " ] " + std::to_string(progress_step + 1) + "/" + std::to_string(total_steps) + "  ";
}

std::string vfm::StaticHelper::printTimeFormatted(const std::chrono::nanoseconds& parsing_time)
{
   auto hors = std::chrono::duration_cast<std::chrono::hours>(parsing_time);
   auto mins = std::chrono::duration_cast<std::chrono::minutes>(parsing_time - hors);
   auto secs = std::chrono::duration_cast<std::chrono::seconds>(parsing_time - hors - mins);
   auto mils = std::chrono::duration_cast<std::chrono::milliseconds>(parsing_time - hors - mins - secs);

   return StaticHelper::zeroPaddedNumStr(hors.count(), 2) + ":" 
      + StaticHelper::zeroPaddedNumStr(mins.count(), 2) + ":" 
      + StaticHelper::zeroPaddedNumStr(secs.count(), 2) + "." 
      + StaticHelper::zeroPaddedNumStr(mils.count(), 3);
}

int vfm::StaticHelper::createImageFromGraphvizDotFile(const std::string& base_filename, const std::string& format)
{
   std::string command = DOT_COMMAND + " -T" + format + " " + base_filename + " > " + base_filename + "." + format;
   return system(command.c_str());
}

void vfm::StaticHelper::checkForOutdatedSimplification(const std::shared_ptr<FormulaParser> parser)
{
   parser->resumeOutputOfMessages();
   auto all_errors{ parser->getErrors() };
   auto warnings{ all_errors.at(ErrorLevelEnum::warning) };

   for (const auto& warning : warnings) {
      if (StaticHelper::stringContains(warning.second, "W0001")) {
         const std::string file_path{ "../include" };
         const std::string file_name{ "model_checking/simplification.h" };
         const std::string full_path{ file_path + "/" + file_name };
         std::string new_path{};

         parser->addNotePlain(SPECIAL_COLOR_2 + "                                         " + RESET_COLOR);
         parser->addNote("Detected old version of '"
            + file_name + "'. I'll create a new one for you, please enter a location or hit " + SPECIAL_COLOR_2 + "ENTER" + RESET_COLOR + " if\n'"
            + SPECIAL_COLOR_2 + StaticHelper::absPath(full_path) + RESET_COLOR + "'\nis fine.");
         parser->addNotePlain(SPECIAL_COLOR_2 + "           Waiting for input...          " + RESET_COLOR);
         parser->addNotePlain("> ", "");

         std::getline(std::cin, new_path);

         if (new_path.empty()) {
            new_path = full_path;
         }

         parser->addNote("Creating new simplification procedure in '" + StaticHelper::absPath(new_path) + "'.");
         code_block::CodeGenerator::deleteAndWriteSimplificationRulesToFile(code_block::CodeGenerationMode::negative, new_path, parser, true); // MC mode.
         parser->addWarning("Note that you'll need to recompile and then re-run the cpp parser. The current run is possibly broken, we'll exit it to be safe.");
         std::exit(EXIT_FAILURE);
      }
   }
}

int vfm::StaticHelper::executeSystemCommand(const std::string& command)
{
   return system(command.c_str());
}

std::string vfm::StaticHelper::exec(const std::string cmd, const bool verbose)
{
   if (verbose) Failable::getSingleton("System")->addNote("Executing system command: '" + cmd + "'.");

   std::array<char, 128> buffer{};
   std::string result{};

#if defined(__linux__)
   std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd.c_str(), "r"), pclose);
#elif _WIN32
   std::unique_ptr<FILE, decltype(&_pclose)> pipe(_popen(cmd.c_str(), "r"), _pclose);
#endif // Other OS not supported.

   if (!pipe)
   {
      throw std::runtime_error("popen() failed!");
   }

   while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr)
   {
      result += buffer.data();
   }

   if (verbose) Failable::getSingleton("System")->addNote("System command execution ended with result '" + result + "'.");

   return result;
}

std::string vfm::StaticHelper::execWithSuccessCode(
   const std::string& cmd, 
   const bool verbose, 
   int& exit_code,
   std::shared_ptr<std::string> path_to_store_meta_info) {
   std::array<char, 16> buffer{};
   std::string result{};
   
   std::chrono::steady_clock::time_point time_elapsed;

   if (path_to_store_meta_info) {
      time_elapsed = std::chrono::steady_clock::now();
   }

#if defined(__linux__)
   auto pipe = popen(cmd.c_str(), "r"); // get rid of shared_ptr
#elif _WIN32
   auto pipe = _popen(cmd.c_str(), "r"); // get rid of shared_ptr
#endif

   if (!pipe) throw std::runtime_error("popen() failed!");

   while (fgets(buffer.data(), buffer.size(), pipe) != nullptr) {
      result += buffer.data();
      std::cout << buffer.data() << std::flush;
   }

#if defined(__linux__)
   exit_code = pclose(pipe);
#elif _WIN32
   exit_code = _pclose(pipe);
#endif

   if (path_to_store_meta_info) {
      auto time_all = (std::chrono::steady_clock::now() - time_elapsed);
      StaticHelper::writeTextToFile(timeStamp() + ": " + printTimeFormatted(time_all) + " time elapsed for execution of system command '" + cmd + "'.\n", *path_to_store_meta_info, true);
   }

   if (verbose) Failable::getSingleton("System")->addNote("System command execution ended with result '" + result + "'.");

   return result;
}

bool isFileLockedWindows(const std::string& filename_raw) {
   const std::string filename{ StaticHelper::replaceAll(filename_raw, "\\", "/") };
   std::fstream file(filename, std::ios::in | std::ios::binary);
   return !file.is_open();
}

int vfm::StaticHelper::createImageFromGraphvizDot(const std::string& graphviz, const std::string& target_filename, const std::string& format)
{
   std::ofstream out(target_filename);
   out << graphviz;
   out.close();

   return createImageFromGraphvizDotFile(target_filename, format);
}

std::string vfm::StaticHelper::createKratosNdetFunction(const std::string& name, const std::vector<std::string> options)
{
   static const std::vector<std::string> nth = { "first", "second", "third", "fourth", "fifth" };
   std::string pars;
   std::string cond = "(or ";
   std::string ret;
   int cnt = 0;

   for (const auto& option : options) {
      pars += "(var " + nth[cnt] + " " + option + ") ";
      ret = "(var ret " + option + ") ";
      cond += "(eq ret " + nth[cnt] + ") ";
      cnt++;
   }

   cond += ")";

   return std::string("(function " + name + " (" + pars + ")\n\
                       (return " + ret + ")\n\
                       (locals)\n\
                       (seq (havoc ret) (assume " + cond + ")))\n");
}

void vfm::StaticHelper::writeTextToFile(const std::string& text, const std::string& path, const bool append)
{
   std::ofstream out;

   if (append) {
      out.open(path, std::ios_base::app);
   }
   else {
      out.open(path);
   }

   if (out.good()) {
      out << text;
   } else {
      Failable::getSingleton()->addError("File '" + path + "' could not be written to.");

      if (out.bad()) {
         Failable::getSingleton()->addError("Bad file stream: Read/writing error on i/o operation.");
      }
      else if (out.fail()) {
         Failable::getSingleton()->addError("Failed file stream: Logical error on i/o operation.");
      }
   }
}

/*
   Eingabe für unsere Skalierung ist t_hut und x_hut, was die Skalierungsfaktoren für die Zeit und den Raum sind.
   Für die Umrechnung reale Welt => EnvModel gilt:
      Raum-Variablen werden mit x_hut multipliziert
      Zeit-Variablen werden mit t_hut multipliziert
      Geschw-Variablen werden mit x_hut/t_hut multipliziert
      Beschl.-Variablen werden mit x_hut/t_hut^2 multipliziert
   EnvModel => reale Welt entsprechend umgekehrt:
      Raum-Variablen werden durch x_hut dividiert
      Zeit-Variablen werden durch t_hut dividiert
      Geschw-Variablen werden mit t_hut/x_hut multipliziert
      Beschl.-Variablen werden mit t_hut^2/x_hut multipliziert
   D.h. für die Umrechnung in beide Richtungen reicht es, wenn wir...
      t_hut und x_hut kennen und
      für jede Variable wissen, ob sie 1, 2, 3 oder 4 ist.
*/

void vfm::StaticHelper::applyTimescaling(MCTrace& trace, const ScaleDescription& ts_description)
{
   for (auto& state : trace.getTrace()) {
      for (auto& var_assignment : state.second) {
         for (const auto& ts : ts_description.getVariables()) {
            if (ts.variable_name_ == var_assignment.first || "env." + ts.variable_name_ == var_assignment.first) {
               if (ts.type_ == ScaleTypeEnum::time) {
                  var_assignment.second = std::to_string(std::stof(var_assignment.second) / ts_description.getTimeScalingFactor());
               }
               else if (ts.type_ == ScaleTypeEnum::distance) {
                  var_assignment.second = std::to_string(std::stof(var_assignment.second) / ts_description.getDistanceScalingFactor());
               }
               else if (ts.type_ == ScaleTypeEnum::velocity) {
                  var_assignment.second = std::to_string(std::stof(var_assignment.second) * ts_description.getTimeScalingFactor() / ts_description.getDistanceScalingFactor());
               }
               else if (ts.type_ == ScaleTypeEnum::acceleration) {
                  var_assignment.second = std::to_string(std::stof(var_assignment.second) * ts_description.getTimeScalingFactor() * ts_description.getTimeScalingFactor() / ts_description.getDistanceScalingFactor());
               }
               else if (ts.type_ == ScaleTypeEnum::none) {
                  // Do nothing.
               }
               else {
                  Failable::getSingleton()->addError("Unknown time scaling type '" + ts.type_.getEnumAsString() + "'.");
               }
            }
         }
      }
   }
}

static constexpr double LAT_ZERO{ 48.999031665333156 }; // Coordinates of...
static constexpr double LON_ZERO{ 9.294116971817786 };  // ...Grossbottwar :-)

std::pair<double, double> vfm::StaticHelper::cartesianToWGS84(const double x, const double y)
{
   return { LAT_ZERO + x / 111000.0, LON_ZERO - y / 75000.0 };
}

// This function is licensed under CC-BY-SA-4.0 which is a copy-left license
// (https://creativecommons.org/licenses/by-sa/4.0/deed.en)!
void vfm::StaticHelper::fakeCallWithCommandLineArguments(const std::string& argv_combined, std::function<void(int argc, char* argv[])> to_do)
{ // From: https://stackoverflow.com/a/1511885
   std::vector<char*> args{};
   std::istringstream iss{ argv_combined };

   std::string token{};
   while (iss >> token) {
      char* arg = new char[token.size() + 1];
      copy(token.begin(), token.end(), arg);
      arg[token.size()] = '\0';
      args.push_back(arg);
   }
   args.push_back(0);

   to_do(args.size() - 1, &args[0]);

   for (size_t i = 0; i < args.size(); i++)
      delete[] args[i];
}

bool vfm::StaticHelper::existsFileSafe(const std::filesystem::path& filepath, const bool quiet)
{
   bool exists{ false };

   try {
      exists = std::filesystem::exists(filepath);
   }
   catch (const std::exception& e) {
      if (!quiet) {
         Failable::getSingleton()->addWarning("Existence of file '" + filepath.string() + "' could not be established. Error: " + e.what());
      }
   }

   return exists;
}

bool vfm::StaticHelper::existsFileSafe(const std::string& filepath, const bool quiet)
{
   return existsFileSafe(std::filesystem::path(filepath), quiet);
}

bool vfm::StaticHelper::removeFileSafe(const std::filesystem::path& filepath, const bool quiet)
{
   try {
      std::filesystem::remove(filepath);
      return true;
   }
   catch (const std::exception& e) {
      if (!quiet) {
         Failable::getSingleton()->addWarning("File '" + filepath.string() + "' not removed. Error: " + e.what());
      }

      return false;
   }
}

bool vfm::StaticHelper::removeFileSafe(const std::string& filepath, const bool quiet)
{
   return removeFileSafe(std::filesystem::path(filepath), quiet);
}

bool vfm::StaticHelper::removeAllFilesSafe(const std::filesystem::path& filepath, const bool quiet)
{
   try {
      std::filesystem::remove_all(filepath);
      return true;
   }
   catch (const std::exception& e) {
      if (!quiet) {
         Failable::getSingleton()->addWarning("Filetype item '" + filepath.string() + "' not removed. Error: " + e.what());
      }

      return false;
   }
}

bool vfm::StaticHelper::removeAllFilesSafe(const std::string& filepath, const bool quiet)
{
   return removeAllFilesSafe(std::filesystem::path(filepath), quiet);
}

bool vfm::StaticHelper::createDirectoriesSafe(const std::filesystem::path& filepath, const bool quiet)
{
   try {
      std::filesystem::create_directories(filepath);
      return true;
   }
   catch (const std::exception& e) {
      if (!quiet) {
         Failable::getSingleton()->addWarning("Directory '" + filepath.string()  + "' not created. Error: " + e.what());
      }

      return false;
   }
}

bool vfm::StaticHelper::createDirectoriesSafe(const std::string& filepath, const bool quiet)
{
   return createDirectoriesSafe(std::filesystem::path(filepath), quiet);
}

std::filesystem::file_time_type vfm::StaticHelper::lastWritetimeOfFileSafe(const std::filesystem::path& filepath, const bool quiet)
{
   try {
      return std::filesystem::last_write_time(filepath);
   }
   catch (const std::exception& e) {
      if (!quiet) {
         Failable::getSingleton()->addWarning("Last write time of file '" + filepath.string()  + "' not established. Error: " + e.what());
      }

      return std::filesystem::file_time_type{};
   }
}

std::filesystem::file_time_type vfm::StaticHelper::lastWritetimeOfFileSafe(const std::string& filepath, const bool quiet)
{
   return lastWritetimeOfFileSafe(std::filesystem::path(filepath), quiet);
}
