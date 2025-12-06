//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file

#include "vfmacro/script.h"
#include "geometry/bezier_functions.h"
#include "parser.h"
#include "simplification/simplification.h"
#include "geometry/images.h"
#include "simulation/highway_image.h"
#include "simulation/env2d_simple.h"
#include "simulation/highway_translators.h"
#include <cmath>
#include <sstream>


using namespace vfm;
using namespace macro;


std::map<int, std::shared_ptr<RoadGraph>> vfm::macro::Script::road_graphs_{};
std::map<std::string, std::shared_ptr<StraightRoadSection>> straight_road_sections_{};

vfm::macro::Script::Script(const std::shared_ptr<DataPack> data, const std::shared_ptr<FormulaParser> parser)
   : Failable("ScriptMacro"), vfm_data_(data ? data : std::make_shared<DataPack>()), vfm_parser_(parser ? parser : SingletonFormulaParser::getInstance())
{
   if (KEEP_COMMENTS_IN_PLAIN_TEXT) {
      putPlaceholderMapping(BEGIN_COMMENT);
      putPlaceholderMapping(END_COMMENT);
   }

   putPlaceholderMapping(PLAIN_TEXT_BEGIN_TAG); // Probably unnecessary, depending on how nesting is handled.
   putPlaceholderMapping(PLAIN_TEXT_END_TAG);   // Probably unnecessary, depending on how nesting is handled.
   putPlaceholderMapping(METHOD_PARS_BEGIN_TAG);
   putPlaceholderMapping(METHOD_PARS_END_TAG);
   putPlaceholderMapping(CONVERSION_PREFIX);
   putPlaceholderMapping(CONVERSION_POSTFIX);
   putPlaceholderMapping(START_TAG_FOR_NESTED_VARIABLES);
   putPlaceholderMapping(END_TAG_FOR_NESTED_VARIABLES);
   putPlaceholderMapping(INSCR_BEG_TAG);
   putPlaceholderMapping(INSCR_END_TAG);
   putPlaceholderMapping(INSCR_BEG_TAG_FOR_INTERNAL_USAGE);
   putPlaceholderMapping(INSCR_END_TAG_FOR_INTERNAL_USAGE);
   putPlaceholderMapping(BEGIN_LITERAL);
   putPlaceholderMapping(END_LITERAL);
   putPlaceholderMapping(EXPR_BEG_TAG_BEFORE);
   putPlaceholderMapping(EXPR_END_TAG_BEFORE);
   putPlaceholderMapping(EXPR_BEG_TAG_AFTER);
   putPlaceholderMapping(EXPR_END_TAG_AFTER);
}

int findLongestChainOfPrioritySymbols(const std::string& sub_script)
{
   std::string s{};

   while (StaticHelper::stringContains(sub_script, INSCR_END_TAG + s + INSCR_PRIORITY_SYMB)) {
      s += INSCR_PRIORITY_SYMB; // TODO: This is not the most efficient way to do it. At least remember where you were and don't start from top every time.
   }

   return s.size();
}

void Script::applyDeclarationsAndPreprocessors(const std::string& codeRaw2, const bool only_one_step)
{
   raw_script_ = codeRaw2;

   if (codeRaw2.empty()) {
      processed_script_ = "";
      return;
   }

   std::string codeRaw = inferPlaceholdersForPlainText(codeRaw2);                              // Secure plain-text parts.
   processed_script_ = evaluateAll(codeRaw, EXPR_BEG_TAG_BEFORE, EXPR_END_TAG_BEFORE);         // Evaluate expressions BEFORE run.

   extractInscriptProcessors(only_one_step);                                                   // DO THE ACTUAL THING.

   processed_script_ = undoPlaceholdersForPlainText(processed_script_);                        // Undo placeholder securing.
   processed_script_ = StaticHelper::replaceAll(processed_script_, NOP_SYMBOL, "");            // Clear all NOP symbols from script.
   processed_script_ = evaluateAll(processed_script_, EXPR_BEG_TAG_AFTER, EXPR_END_TAG_AFTER); // Evaluate expressions AFTER run.
}

std::string vfm::macro::Script::getProcessedScript() const
{
   return processed_script_;
}

void Script::extractInscriptProcessors(const bool only_one_step)
{
   // Find next preprocessor.
   int indexOfPrep = findNextInscriptPos();
   int i{};

   while (indexOfPrep >= 0) {
      std::string preprocessorScript = *StaticHelper::extractFirstSubstringLevelwise(processed_script_, INSCR_BEG_TAG, INSCR_END_TAG, indexOfPrep);
      int lengthOfPreprocessor = preprocessorScript.length() + INSCR_BEG_TAG.length() + INSCR_END_TAG.length();
      std::string partBefore = processed_script_.substr(0, indexOfPrep);
      std::string partAfter = processed_script_.substr(indexOfPrep + lengthOfPreprocessor);

      int indexCurr = getNextNonInscriptPosition(partAfter);
      indexCurr = (std::min)(indexCurr, (int)partAfter.length());

      std::string methods = partAfter.substr(0, indexCurr);
      partAfter = partAfter.substr(indexCurr);
      int methodPartBegin = preprocessorScript.length();
      preprocessorScript = preprocessorScript + methods;
      std::string placeholder_for_inscript{};

      int begin = indexOfPrep;
      auto trimmed = StaticHelper::trimAndReturn(preprocessorScript);

      if (getScriptData().method_part_begins_.count(trimmed) && getScriptData().method_part_begins_.at(trimmed).cachable_) { // TODO: Avoid multiple accesses to map.
         getScriptData().cache_hits_++;
         placeholder_for_inscript = getScriptData().method_part_begins_.at(trimmed).result_;
      }
      else {
         getScriptData().cache_misses_++;
         std::vector<std::string> methodSignaturesArray = getMethodSinaturesFromChain(methods); // TODO: Done twice for each subscript. Is this expensive?
         bool is_this_cachable{ isCachableChain(methodSignaturesArray) };

         if (methodPartBegin != preprocessorScript.length() && methodPartBegin >= 0) {
            int leftTrim = preprocessorScript.size() - StaticHelper::ltrimAndReturn(preprocessorScript).size();

            getScriptData().method_part_begins_.insert({
               trimmed,
               { methodPartBegin - leftTrim, placeholder_for_inscript, is_this_cachable }
               });
         }
         else {
            getScriptData().method_part_begins_.insert({
               trimmed,
               { -1, placeholder_for_inscript, is_this_cachable }
               });
         }

         placeholder_for_inscript = evaluateChain(preprocessorScript);

         if (StaticHelper::startsWithUppercase(methodSignaturesArray.at(0))) {
            // @{@{i}@.eval}@*.for[i, 1, 10]
            // Remove this whole IF clause to undo.

            auto s = std::make_shared<Script>(vfm_data_, vfm_parser_);
            addFailableChild(s);
            s->raw_script_ = placeholder_for_inscript;
            s->processed_script_ = placeholder_for_inscript;
            s->extractInscriptProcessors(false);
            placeholder_for_inscript = s->processed_script_;
         }

         getScriptData().method_part_begins_[trimmed].result_ = placeholder_for_inscript;
      }

      std::string placeholderFinal = checkForPlainTextTags(placeholder_for_inscript);
      processed_script_ = partBefore + placeholderFinal + partAfter;

      if (i++ % 100 == 0) {
         addNote(""
            + std::to_string(getScriptData().known_chains_.size()) + " known_chains_; "
            + std::to_string(getScriptData().method_part_begins_.size()) + " method_part_begins_; "
            + std::to_string(getScriptData().list_data_.size()) + " list_data_; "
            + std::to_string(processed_script_.size()) + " script size; "
            + std::to_string(getScriptData().cache_hits_) + "/" + std::to_string(getScriptData().cache_misses_) + " cache hits/misses; "
         );
      }

      if (only_one_step) return;

      indexOfPrep = findNextInscriptPos();
   }
}

std::string Script::checkForPlainTextTags(const std::string& script)
{
   std::string script2 = script;

   if (StaticHelper::stringStartsWith(script, PLAIN_TEXT_BEGIN_TAG) && StaticHelper::stringEndsWith(script, PLAIN_TEXT_END_TAG)) {
      script2 = replacePlaceholders(script, true);
   }

   if (StaticHelper::stringStartsWith(script, LAST_LETTER_BEG)) { // Starts with "{...":
      script2 = NOP_SYMBOL + script;        // Make "N{..." to avoid  "}@{...".
   }                                         // Note that this would PRINCIPLY work, but the other one wouldn't.

   if (StaticHelper::stringEndsWith(script, FIRST_LETTER_END)) { // Ends with "...}":
      script2 = script + NOP_SYMBOL;       // Make "...}N" to avoid "...}@{".
   }

   return script2;
}

std::string Script::evaluateChain(const std::string& chain)
{
   std::string processedChain{ StaticHelper::trimAndReturn(chain) };
   std::string processedRaw{ processedChain };

   if (getScriptData().known_chains_.count(processedRaw)) {
      getScriptData().cache_hits_++;
      return getScriptData().known_chains_.at(processedRaw);
   }

   getScriptData().cache_misses_++;

   RepresentableAsPDF repToProcess{};
   std::shared_ptr<int> methodBegin = getScriptData().method_part_begins_.count(processedRaw) && getScriptData().method_part_begins_.at(processedRaw).method_part_begin_ >= 0
      ? std::make_shared<int>(getScriptData().method_part_begins_.at(processedRaw).method_part_begin_)
      : nullptr;

   if (StaticHelper::stringStartsWith(processedChain, INSCR_BEG_TAG)
      && (!methodBegin
         || StaticHelper::findMatchingEndTagLevelwise(
            processedChain,
            INSCR_BEG_TAG,
            INSCR_END_TAG, 0) == (*methodBegin - INSCR_END_TAG.length()))) { // Whole script part surrounded by @{...}@.
      std::string repPart = *StaticHelper::extractFirstSubstringLevelwise(
         processedChain,
         INSCR_BEG_TAG,
         INSCR_END_TAG,
         0);

      if (!methodBegin) { // Possibly regular script.
         repToProcess = repfactory_instanceFromScript(repPart);
         processedChain = processedChain.substr(repPart.length() + INSCR_BEG_TAG.length() + INSCR_END_TAG.length());
      }
      else { // Treat as plain text.
         repToProcess = std::make_shared<Script>(vfm_data_, vfm_parser_);
         repToProcess->createInstanceFromScript(INSCR_BEG_TAG + repPart + INSCR_END_TAG);
         processedChain = processedRaw.substr(*methodBegin + 1);
      }
   }
   else {
      if (methodBegin) { // String is script without delimiters, but with method calls.
         std::string script = processedRaw.substr(0, *methodBegin);
         processedChain = processedRaw.substr(*methodBegin + 1);
         repToProcess = repfactory_instanceFromScript(script);

         if (!repToProcess) { // String is no script, but plain expression.
            repToProcess = std::make_shared<Script>(vfm_data_, vfm_parser_);
            repToProcess->createInstanceFromScript(script);
         }
      }
      else { // Assume whole string is expression without method calls.
         repToProcess = repfactory_instanceFromScript(processedChain);
         processedChain = "";
      }
   }

   addFailableChild(repToProcess, "");

   std::vector<std::string> methodSignaturesArray = getMethodSinaturesFromChain(processedChain);
   bool chachable{ isCachableChain(methodSignaturesArray) && processedRaw.size() <= MAXIMUM_STRING_SIZE_TO_CACHE };

   if (StaticHelper::isEmptyExceptWhiteSpaces(processedChain)) {
      if (chachable) getScriptData().known_chains_[processedRaw] = repToProcess->getRawScript(); // TODO: It might be inefficient to copy the string here.
      return repToProcess->getRawScript();
   }

   RepresentableAsPDF apply_method_chain = applyMethodChain(repToProcess, methodSignaturesArray);
   if (chachable) getScriptData().known_chains_[processedRaw] = apply_method_chain->getRawScript(); // TODO: It might be inefficient to copy the string here.

   return apply_method_chain->getRawScript();
}

/**
 * Applies a method chain to the given representable. The list of methods
 * has to comply with the object types generated in the process, i.e., in:</BR>
 *           [script].m1.m2.m3.m4.m5</BR>
 * [script] must provide a method named m1, [script].m1 must provide a method
 * named m2 and so on.
 *
 * @param original        The original script.
 * @param methodsToApply  A list of methods to apply as:
 *                        "*internalName*["par1", "par2", ...]"
 *
 * @return  A new representable with the methods applied.
 */
RepresentableAsPDF Script::applyMethodChain(const RepresentableAsPDF original, const std::vector<std::string>& methodsToApply)
{
   RepresentableAsPDF newRep = original;

   for (const auto& methodSignature : methodsToApply) {
      newRep = applyMethod(newRep, methodSignature);
   }

   return newRep;
}

RepresentableAsPDF Script::applyMethod(const RepresentableAsPDF rep, const std::string& methodSignature)
{
   std::string conversionTag = getConversionTag(
      CONVERSION_PREFIX
      + methodSignature
      + CONVERSION_POSTFIX);

   std::string methodName = StaticHelper::split(conversionTag, "[")[0];
   std::vector<std::string> pars = getMethodParameters(conversionTag);

   std::vector<std::string> actualParameters = getParametersFor(pars);
   std::string newScript = rep->applyMethodString(methodName, actualParameters);

   return repfactory_instanceFromScript(newScript);
}

std::string Script::fromBooltoString(const bool b)
{
   return b ? "1" : "0";
}

float deg2Rad(const float deg)
{
   static constexpr float PI{ 3.1415926536 };
   return deg * PI / 180;
}

std::string Script::arclengthCubicBezierFromStreetTopology(
   const std::string& lane_str, const std::string& angle_str, const std::string& distance_str, const std::string& num_lanes_str)
{
   if (!StaticHelper::isParsableAsInt(lane_str)) addError("Lane '" + lane_str + "' is not parsable as int in 'arclengthCubicBezierFromStreetTopology'.");
   if (!StaticHelper::isParsableAsFloat(angle_str)) addError("Angle '" + angle_str + "' is not parsable as float in 'arclengthCubicBezierFromStreetTopology'.");
   if (!StaticHelper::isParsableAsFloat(distance_str)) addError("Distance '" + distance_str + "' is not parsable as float in 'arclengthCubicBezierFromStreetTopology'.");
   if (!StaticHelper::isParsableAsInt(num_lanes_str)) addError("NumLanes '" + num_lanes_str + "' is not parsable as float in 'arclengthCubicBezierFromStreetTopology'.");
   if (hasErrorOccurred()) return "#ERROR-Check-Log";

   const int num_lanes{ std::stoi(num_lanes_str) };
   const int lane{ num_lanes - std::stoi(lane_str) - 1 };
   const float angle{ deg2Rad(std::stof(angle_str)) };
   const float distance{ std::stof(distance_str) };
   const float l{ (lane - (num_lanes - 1.0f) / 2.0f) * LANE_WIDTH };

   const Vec2D v{ std::cos(angle - deg2Rad(180)), sin(angle - deg2Rad(180)) };
   const Vec2D vr{ std::cos(angle + deg2Rad(90)), sin(angle + deg2Rad(90)) };
   const Vec2D P3{ Vec2D{distance, 0} - v * distance };

   const Vec2D p0{ 0,  l };
   const Vec2D p3{ P3 + vr * l };

   const float d{ p0.distance(p3) / 3.0f };

   Vec2D p1{ d, l };
   Vec2D p2{ p3 + v * d };
   
   return std::to_string((int) std::round(bezier::arcLength(1, p0, p1, p2, p3)))
      // Output for debugging, but does not work with distance scaling.
      //+ " -- l=" + std::to_string(l)
      //+ ", angle=" + std::to_string(angle) 
      //+ "(rad), v=" + v.serialize()
      //+ ", vr=" + vr.serialize()
      //+ ", P3=" + P3.serialize()
      //+ ", p0=" + p0.serialize()
      //+ ", p1=" + p1.serialize()
      //+ ", p2=" + p2.serialize()
      //+ ", p3=" + p3.serialize()
      //+ ", d=" + std::to_string(d)
      //+ ", N=" + std::to_string(num_lanes)
      ;
}

std::string Script::forloop(const std::string& varname, const std::string& loop_vec)
{
   return forloop(varname, loop_vec, "");
}

std::string Script::forloop(const std::string& varname, const std::string& from_raw, const std::string& to_raw)
{
   if (from_raw.empty() || StaticHelper::stringStartsWith(from_raw, BEGIN_TAG_IN_SEQUENCE)) { // Regular sequence @()@@()@...
      std::vector<std::string> loop_vec{};
      processSequence(from_raw, loop_vec);
      return forloop(varname, loop_vec, to_raw);
   }
   else {
      return forloop(varname, from_raw, to_raw, "1");
   }
}

std::string Script::forloop(const std::string& varname, const std::string& from_raw, const std::string& to_raw, const std::string& step_raw)
{
   return forloop(varname, from_raw, to_raw, step_raw, "");
}

std::string Script::forloop(const std::string& varname, const std::string& from_raw, const std::string& to_raw, const std::string& step_raw, const std::string& inner_separator)
{
   if (!StaticHelper::isParsableAsFloat(from_raw) || !StaticHelper::isParsableAsFloat(to_raw) || !StaticHelper::isParsableAsFloat(step_raw)) {
      addError("At least one of the loop limits '" + from_raw + "' and '" + to_raw + "' and/or the step '" + step_raw + "' is not a number.");
   }

   int from = std::stoi(from_raw);
   int to = std::stoi(to_raw);
   int step = std::stoi(step_raw);

   bool increasing = step >= 0;
   bool decreasing = !increasing;

   std::vector<std::string> loop_vec{};

   for (int i = from; (increasing && i <= to) || (decreasing && i >= to); i += step) {
      loop_vec.push_back(std::to_string(i));
   }

   return forloop(varname, loop_vec, inner_separator);
}

std::string Script::forloop(const std::string& varname, const std::vector<std::string>& loop_vec, const std::string& inner_separator) {
   std::string loopedVal = "";
   bool first{ true };

   for (const auto& i : loop_vec) {
      std::string currVal = StaticHelper::replaceAll(getRawScript(), varname, i);
      loopedVal += (!first ? StaticHelper::replaceAll(inner_separator, varname, i) : "") + currVal;
      first = false;
   }

   return loopedVal;
}

std::string Script::ifChoice(const std::string bool_str)
{
   bool boolVal{ StaticHelper::isBooleanTrue(bool_str) };

   int count = scriptSequence_.size();

   if (count > 2) { // Error case 1.
      addError(std::to_string(count) + " objects given to IF clause in '" + getRawScript() + "'.");
      return "#INVALID";
   }
   //else if (bool_str != "false" && bool_str != "true" && bool_str != "0" && bool_str != "1") { // Error case 2.
   //   addError("'" + bool_str + "' is not a Boolean value.");
   //}
   else if (boolVal) { // True case.
      return count > 0 ? scriptSequence_.at(0) : "";
   }
   else { // False case.
      if (scriptSequence_.size() < 2) { // No ELSE case defined.
         return "";
      }
      else {
         return scriptSequence_.at(1);
      }
   }
}

std::string Script::element(const std::string& num_str)
{
   if (!StaticHelper::isParsableAsFloat(num_str)) {
      addError("Sequence index '" + num_str + "' cannot be parsed to float/int.");
      return "#INVALID";
   }

   int num = std::stof(num_str);

   if (num < scriptSequence_.size()) {
      return scriptSequence_.at(num);
   }

   addError("Element with index " + std::to_string(num) + " not available in sequence of length " + std::to_string(scriptSequence_.size()));
   return "#INVALID";
}

float Script::stringToFloat(const std::string& str)
{
   if (StaticHelper::isParsableAsFloat(str)) {
      return std::stof(str);
   }
   else {
      addError("String '" + str + "' is not parsable as float.");
      return std::numeric_limits<float>::quiet_NaN();
   }
}

std::string Script::substring(const std::string& beg, const std::string& end)
{
   float begin{ stringToFloat(beg) };
   float ending{ stringToFloat(end) };

   if (std::isnan(begin) || std::isnan(ending)) {
      addError("Either begin '" + beg + "' or end '" + end + "' not parsable as int for substr function.");
      return "#INVALID";
   }
   else {
      return getRawScript().substr(begin, ending);
   }
}

std::string Script::newMethod(const std::string& methodName, const std::string& numPars)
{
   return newMethodD(methodName, numPars, INSCRIPT_STANDARD_PARAMETER_PATTERN);
}

std::string Script::newMethodD(const std::string& methodName, const std::string& numPars, const std::string& parameterPattern)
{
   if (getScriptData().inscriptMethodDefinitions.count(methodName)) {
      addWarning("Dynamic method '" + methodName + "' already exists. I will override it.");
   }

   if (!StaticHelper::isParsableAsFloat(numPars)) {
      addError("Parameter number '" + numPars + "' in definition of dynamic method '" + methodName + "' not parsable as float/int.");
      return "";
   }

   getScriptData().inscriptMethodDefinitions.insert({ methodName, getRawScript() });
   getScriptData().inscriptMethodParNums.insert({ methodName, (int)std::stof(numPars) });
   getScriptData().inscriptMethodParPatterns.insert({ methodName, parameterPattern });

   addNote("New method '" + methodName + "' with "
      + std::to_string(getScriptData().inscriptMethodParNums.at(methodName)) + " parameters defined as '"
      + getScriptData().inscriptMethodDefinitions.at(methodName) + "'. Parameter pattern: '" + parameterPattern + "'.");

   return "";
}

std::string Script::executeCommand(const std::string& methodName, const std::vector<std::string>& pars)
{
   std::string methodBody = getScriptData().inscriptMethodDefinitions.at(methodName);
   std::string pattern = getScriptData().inscriptMethodParPatterns.at(methodName);

   int numRepl = pars.size() + 1;
   std::vector<std::string> searchList{};
   std::vector<std::string> replacementList{};

   searchList.resize(numRepl);
   replacementList.resize(numRepl);

   searchList[0] = makroPattern(0, pattern);
   replacementList[0] = getRawScript();

   for (int i = 1; i < numRepl; i++) {
      searchList[i] = makroPattern(i, pattern);
      replacementList[i] = pars[i - 1];
   }

   return StaticHelper::replaceManyTimes(methodBody, searchList, replacementList);
}

std::string Script::makroPattern(const int i, const std::string& pattern)
{
   if (StaticHelper::replaceAll(pattern, "n", "").size() != pattern.length() - 1) {
      addError("Malformed makro parameter pattern pattern '" + pattern + "' does not contain exactly one 'n'.");
   }

   std::string makroPar = StaticHelper::replaceAll(pattern, "n", std::to_string(i));
   return makroPar;
}

std::string Script::applyMethodString(const std::string& method_name, const std::vector<std::string>& parameters)
{
   for (const auto& method_description : METHODS) {
      if (method_description.method_name_ == method_name && parameters.size() == method_description.par_num_) {
         return method_description.function_(parameters);
      }
   }

   if (method_name == "pushBack" && parameters.size() > 1) { // This is a hack to introduce comma which is usually separator of arguments.
      std::string appended{ parameters[0] };

      for (int i = 1; i < parameters.size(); i++) {
         appended += "," + parameters[i];
      }

      return applyMethodString(method_name, { appended });
   }

   if (getScriptData().inscriptMethodDefinitions.count(method_name) && getScriptData().inscriptMethodParNums.at(method_name) == parameters.size()) {
      return executeCommand(method_name, parameters);
   }

   int cnt{ 0 };
   std::string pars{ "'" + getRawScript() + "' (self (" + std::to_string(cnt++) + "))"};
   for (const auto& s : parameters) {
      pars += ", '" + s + "' (" + std::to_string(cnt++) + ")";
   }

   std::vector<std::string> method_names{};
   std::vector<std::string> method_names_lowercase{};

   for (const auto& meth : METHODS) {
      method_names.push_back(meth.method_name_);
      method_names_lowercase.push_back(StaticHelper::toLowerCase(meth.method_name_));
   }

   for (const auto& dynamic_method : getScriptData().inscriptMethodParNums) {
      method_names.push_back(dynamic_method.first);
      method_names_lowercase.push_back(StaticHelper::toLowerCase(dynamic_method.first));
   }

   auto closest = StaticHelper::getClosest(method_names_lowercase, StaticHelper::toLowerCase(method_name), method_names);
   std::string all_closest{};

   for (const auto& meth : METHODS) {
      if (meth.method_name_ == closest) {
         all_closest += " " + closest + "[" + std::to_string(meth.par_num_) + "]";
      }
   }

   for (const auto& meth : getScriptData().inscriptMethodParNums) {
      if (meth.first == closest) {
         all_closest += " " + closest + "[" + std::to_string(meth.second) + "]";
      }
   }

   std::string error_str{ "No method '" + method_name + "' found which takes " + std::to_string(parameters.size()) + " arguments. Arguments are: {" + pars + "}. " + 
      "Did you mean:" + all_closest + "?"};

   addError(error_str);
   return "#INVALID(" + error_str + ")";
}

bool vfm::macro::Script::isNativeMethod(const std::string& method_name) const
{
   for (const auto& method_description : METHODS) {
      if (method_description.method_name_ == method_name) {
         return true;
      }
   }

   return false;
}

bool vfm::macro::Script::isDynamicMethod(const std::string& method_name) const
{
   return getScriptData().inscriptMethodDefinitions.count(method_name);
}

bool vfm::macro::Script::isMethod(const std::string& method_name) const
{
   return isNativeMethod(method_name) || isDynamicMethod(method_name);
}

bool vfm::macro::Script::isCachableMethod(const std::string& method_name) const
{
   return getScriptData().uncachable_methods.count(method_name) == 0;
}


bool vfm::macro::Script::isCachableChain(const std::vector<std::string>& method_chain) const
{
   for (const auto& method : method_chain) {
      const size_t par_begin{ method.find("[") };
      const std::string method_w_o_pars{ par_begin == std::string::npos ? method : method.substr(0, par_begin) };

      if (!isCachableMethod(method_w_o_pars)) {
         return false;
      }
   }

   return true;
}

bool vfm::macro::Script::makeMethodCachable(const std::string& method_name)
{
   if (getScriptData().uncachable_methods.count(method_name)) {
      getScriptData().uncachable_methods.erase(method_name);
      return true;
   }

   return false; // Was already cachable (= not in "uncachable" list).
}

bool vfm::macro::Script::makeMethodUnCachable(const std::string& method_name)
{
   return getScriptData().uncachable_methods.insert(method_name).second;
}

void vfm::macro::Script::addDefaultDynamicMathods()
{
   getScriptData().inscriptMethodDefinitions.insert({ "fib", R"(@{
@(#0#)@
@(@{@{@{#0#}@*.sub[1].fib}@*}@.add[@{#0#}@*.sub[2].fib])@
}@**.if[@{#0#}@.smeq[1]])"});
   getScriptData().inscriptMethodParNums.insert({ "fib", 0 });
   getScriptData().inscriptMethodParPatterns.insert({ "fib", INSCRIPT_STANDARD_PARAMETER_PATTERN });

   getScriptData().inscriptMethodDefinitions.insert({ "Fib", R"(@{
@(#0#)@
@(@{@{@{#0#}@*.SubI[1].Fib}@*}@.AddI[@{#0#}@*.SubI[2].Fib])@
}@**.if[@{#0#}@.SmeqI[1]])"});
   getScriptData().inscriptMethodParNums.insert({ "Fib", 0 });
   getScriptData().inscriptMethodParPatterns.insert({ "Fib", INSCRIPT_STANDARD_PARAMETER_PATTERN });

   getScriptData().inscriptMethodDefinitions.insert({ "fibfast", R"(@{
@(#0#)@
@(@{#0#.fibfast}@.sethard[
@{@{@{@{#0#}@*.sub[1]}@*.fibfast}@*}@.add[@{@{#0#}@*.sub[2]}@*.fibfast]])@
}@**.if[@{#0#}@.smeq[1]])"});
   getScriptData().inscriptMethodParNums.insert({ "fibfast", 0 });
   getScriptData().inscriptMethodParPatterns.insert({ "fibfast", INSCRIPT_STANDARD_PARAMETER_PATTERN });

   std::stringstream s{};
   std::vector<std::string> method_names{};

   for (const auto& method : getScriptData().inscriptMethodDefinitions) {
      method_names.push_back(method.first);
   }

   s << method_names;

   addNote("The following default dynamic methods have been added to the script processor: " + s.str() + ".");
}

std::vector<std::string> Script::getMethodParameters(const std::string& conversionTag)
{
   if (!StaticHelper::stringContains(conversionTag, METHOD_PARS_BEGIN_TAG)
      || !StaticHelper::stringContains(conversionTag, METHOD_PARS_END_TAG)
      || conversionTag.find(METHOD_PARS_BEGIN_TAG) == conversionTag.find(METHOD_PARS_END_TAG) - 1) {
      return {};
   }

   std::string methPars = *StaticHelper::extractFirstSubstringLevelwise(
      conversionTag,
      METHOD_PARS_BEGIN_TAG,
      METHOD_PARS_END_TAG,
      0); // TODO: correct?

   std::string conversionTag2 = StaticHelper::stringContains(conversionTag, METHOD_PARS_BEGIN_TAG)
      ? methPars
      : conversionTag;

   // Parameters with nested parts.
   if (StaticHelper::stringContains(conversionTag, START_TAG_FOR_NESTED_VARIABLES)) {
      conversionTag2 = StaticHelper::trimAndReturn(conversionTag);
      int beg = conversionTag2.find(METHOD_PARS_BEGIN_TAG);
      int end = conversionTag2.rfind(METHOD_PARS_END_TAG);
      conversionTag2 = conversionTag2.substr(beg + 1, end - (beg + 1));

      std::vector<std::string> methodParametersWithTags = getMethodParametersWithTags(conversionTag2);
      std::vector<std::string> methodParametersArray{};

      methodParametersArray.resize(methodParametersWithTags.size());

      for (int i = 0; i < methodParametersWithTags.size(); i++) {
         methodParametersArray[i] = methodParametersWithTags.at(i);
      }

      return methodParametersArray;
   }

   bool inString = false;
   std::vector<std::string> pars;
   std::string currentPart = "";

   for (int i = 0; i < conversionTag2.length(); i++) {
      if (conversionTag2.at(i) == ',') {
         if (inString                       // Within a "string".
            || StaticHelper::isWithinLevelwise( // Within [ and ].
               conversionTag2,
               i,
               METHOD_PARS_BEGIN_TAG,
               METHOD_PARS_END_TAG)) {
            currentPart += ",";
         }
         else { // Not in special-char block.
            pars.push_back(currentPart);
            currentPart = "";
         }
      }

      if (conversionTag2.at(i) == '\"') {
         inString = !inString;
      }
      else if (!inString && conversionTag2.at(i) != ' ' && conversionTag2.at(i) != ',') {
         currentPart += conversionTag2.at(i);
      }

      if (inString && conversionTag2.at(i) != '\"') {
         currentPart += conversionTag2.at(i);
      }
   }

   pars.push_back(currentPart);

   std::vector<std::string> parsArray{};
   parsArray.resize(pars.size());

   for (int i = 0; i < pars.size(); i++) {
      parsArray[i] = pars.at(i);
   }

   return parsArray;
}

std::vector<std::string> Script::getMethodParametersWithTags(const std::string& conversionTag) 
{
   if (StaticHelper::stringStartsWith(conversionTag, ",")) {
      return getMethodParametersWithTags(conversionTag.substr(1));
   }

   std::vector<std::string> pars{};

   int nextComma = StaticHelper::indexOfOnTopLevel(
      conversionTag,
      { "," },
      0,
      START_TAG_FOR_NESTED_VARIABLES,
      END_TAG_FOR_NESTED_VARIABLES);

   std::string par = "";

   if (nextComma < 0) {
      par = StaticHelper::trimAndReturn(conversionTag);

      par = *StaticHelper::extractFirstSubstringLevelwise(
         par,
         START_TAG_FOR_NESTED_VARIABLES,
         END_TAG_FOR_NESTED_VARIABLES,
         0);

      pars.push_back(par);
      return pars;
   }

   par = StaticHelper::trimAndReturn(conversionTag.substr(0, nextComma));

   par = *StaticHelper::extractFirstSubstringLevelwise(
      par,
      START_TAG_FOR_NESTED_VARIABLES,
      END_TAG_FOR_NESTED_VARIABLES,
      0);

   pars.push_back(par);

   auto other = getMethodParametersWithTags(conversionTag.substr(nextComma));
   pars.insert(pars.end(), other.begin(), other.end());

   return pars;
}

std::vector<std::string> Script::getParametersFor(const std::vector<std::string>& rawPars)
{
   std::vector<std::string> parameters;
   parameters.resize(rawPars.size());

   for (int i = 0; i < rawPars.size(); i++) {
      parameters[i] = evaluateChain(rawPars[i]);
   }

   return parameters;
}

std::string Script::getConversionTag(const std::string& scriptWithoutComments)
{
   std::string conversionTag = "";

   for (int i = scriptWithoutComments.length() - 1 - CONVERSION_POSTFIX.length(); i >= 0; i--) {
      conversionTag = scriptWithoutComments.at(i) + conversionTag;
      if (i - CONVERSION_PREFIX.length() < 0
         || scriptWithoutComments.substr(i - CONVERSION_PREFIX.length(), i - (i - CONVERSION_PREFIX.length())) == CONVERSION_PREFIX) {
         break;
      }
   }
   return conversionTag;
}

int Script::findNextInscriptPos()
{
   int length_of_longest_priority_chain{ findLongestChainOfPrioritySymbols(processed_script_)};
   int pos = processed_script_.find(INSCR_END_TAG + std::string(length_of_longest_priority_chain, INSCR_PRIORITY_SYMB));
   int count = 0;               // Because we start on an end tag.

   for (int i = pos; i >= 0; i--) {
      if (StaticHelper::stringStartsWith(processed_script_, INSCR_BEG_TAG, i)) {
         count++;
      }

      if (StaticHelper::stringStartsWith(processed_script_, INSCR_END_TAG, i)) {
         count--;
      }

      if (count == 0) {
         int starPos = pos + INSCR_END_TAG.length();
         processed_script_ = processed_script_.substr(0, starPos)
            + processed_script_.substr(starPos + length_of_longest_priority_chain);

         return i;
      }
   }

   return -1;
}

ScriptData& Script::getScriptData() const
{
   return vfm_data_->getScriptData();
}

std::string Script::removeTaggedPartsOnTopLevel(
   const std::string& code,
   const std::string& beginTag,
   const std::string& endTag,
   std::vector<std::string> ignoreBegTags,
   std::vector<std::string> ignoreEndTags) 
{
   std::string s;
   int count = 0;

   for (int i = 0; i < code.length(); i++) {
      if (StaticHelper::stringStartsWith(code, beginTag, i)
         && !StaticHelper::isWithinAnyLevelwise(
            code,
            i,
            ignoreBegTags,
            ignoreEndTags)) {
         count++;
         i += beginTag.length() - 1;
      }
      else if (StaticHelper::stringStartsWith(code, endTag, i)
         && !StaticHelper::isWithinAnyLevelwise(
            code,
            i,
            ignoreBegTags,
            ignoreEndTags)) {
         count--;
         i += endTag.length() - 1;
      }
      else if (count == 0) {
         s += StaticHelper::makeString(code.at(i));
      }
   }

   return s;
}

std::string Script::undoPlaceholdersForPlainText(const std::string& script)
{
   return replacePlaceholders(script, false);
}

int Script::getNextNonInscriptPosition(const std::string& partAfter) 
{
   int count = 0;
   bool lastWasMethodEnd = true;

   for (int i = 0; i < partAfter.length(); i++) {
      char currChar = partAfter.at(i);
      std::string currStr = StaticHelper::makeString(currChar);

      if (count == 0) {
         if (lastWasMethodEnd) {
            if (!StaticHelper::stringStartsWith(partAfter, METHOD_CHAIN_SEPARATOR, i)) {
               return i; // No methods more to come (particularly at pos 0 if no methods at all).
            }
         }
         else if (!std::isalnum(currChar)
            && currStr != METHOD_PARS_BEGIN_TAG
            && currStr != METHOD_PARS_END_TAG
            && currStr != METHOD_CHAIN_SEPARATOR) {
            return i;
         }
      }

      lastWasMethodEnd = false;

      if (StaticHelper::stringStartsWith(partAfter, METHOD_PARS_BEGIN_TAG, i)) {
         lastWasMethodEnd = false;
         count++;
      }

      if (StaticHelper::stringStartsWith(partAfter, METHOD_PARS_END_TAG, i)) {
         lastWasMethodEnd = true;
         count--;
      }

      if (count < 0) {
         return i;
      }
   }

   return partAfter.length(); // Whole partAfter belongs to preprocessor.
}

std::string Script::inferPlaceholdersForPlainText(const std::string& script) 
{
   std::string script2 = script;
   int indexOf = script2.find(PLAIN_TEXT_BEGIN_TAG);
   std::vector<std::string> snippets;
   std::string before;
   std::string inner;
   std::string after = script2;

   std::string b = "";
   std::string e = "";
   if (!removePlaintextTagsAfterPreprocessorApplication()) {
      b = PLAIN_TEXT_BEGIN_TAG;
      e = PLAIN_TEXT_END_TAG;
   }

   while (indexOf >= 0) {
      int indexOf2 = script2.find(PLAIN_TEXT_END_TAG, indexOf);

      if (indexOf2 < 0) {
         addError("Plain-text begin tag has no matching end tag in script '" + script + "'.");
      }

      before = script2.substr(0, indexOf);
      inner = script2.substr(indexOf + PLAIN_TEXT_BEGIN_TAG.length(), indexOf2 - (indexOf + PLAIN_TEXT_BEGIN_TAG.length()));
      after = script2.substr(indexOf2 + PLAIN_TEXT_END_TAG.length());

      putPlaceholderMapping(inner);
      snippets.push_back(StaticHelper::replaceAll(StaticHelper::replaceAll(before, PLAIN_TEXT_BEGIN_TAG, ""), PLAIN_TEXT_END_TAG, ""));
      snippets.push_back(b + StaticHelper::replaceAll(StaticHelper::replaceAll(replacePlaceholders(inner, true), PLAIN_TEXT_BEGIN_TAG, ""), PLAIN_TEXT_END_TAG, "") + e);

      script2 = after;
      indexOf = script2.find(PLAIN_TEXT_BEGIN_TAG);
   }

   snippets.push_back(after);
   script2 = "";

   for (std::string s : snippets) {
      script2 += s;
   }

   return script2;
}

bool Script::removePlaintextTagsAfterPreprocessorApplication() const 
{
   return true;
}

std::string Script::replacePlaceholders(const std::string& toReplace, const bool to) 
{
   std::vector<std::string> searchList;
   std::vector<std::string> replacementList;

   searchList.resize(getScriptData().PLACEHOLDER_MAPPING.size());
   replacementList.resize(getScriptData().PLACEHOLDER_MAPPING.size());

   if (to) {
      fillLists(searchList, replacementList);
   }
   else {
      fillLists(replacementList, searchList);
   }

   return StaticHelper::replaceManyTimes(toReplace, searchList, replacementList);
}

void Script::fillLists(std::vector<std::string>& symbolList, std::vector<std::string>& placeholderList) 
{
   int i = 0;
   for (const auto& symbol : getScriptData().PLACEHOLDER_MAPPING) {
      std::string placeholder = symbolToPlaceholder(symbol.first);
      symbolList[i] = symbol.first;
      placeholderList[i] = placeholder;
      i++;
   }
}

void Script::putPlaceholderMapping(const char symbolName) 
{
   putPlaceholderMapping(StaticHelper::makeString(symbolName));
}

void Script::putPlaceholderMapping(const std::string& symbolName) 
{
   std::string before = "$<<$";
   std::string after = "$>>$";
   std::string placeholderPlain = before + std::to_string(std::hash<std::string>()(symbolName)) + after;

   getScriptData().PLACEHOLDER_MAPPING.insert({ symbolName, placeholderPlain });
   getScriptData().PLACEHOLDER_INVERSE_MAPPING.insert({ placeholderPlain, symbolName });
}

std::string Script::placeholderToSymbol(const std::string& placeholder) {
   std::string placeholderPlain = StaticHelper::replaceAll(placeholder, StaticHelper::convertPtrAddressToString(this), "");

   if (getScriptData().PLACEHOLDER_MAPPING.count(placeholderPlain)) {
      std::string symbol = getScriptData().PLACEHOLDER_INVERSE_MAPPING.at(placeholderPlain);
      return symbol;
   }

   return placeholder;
}

std::string Script::symbolToPlaceholder(const std::string& symbol) {
   if (getScriptData().PLACEHOLDER_MAPPING.count(symbol)) {
      std::string placeholderPlain = getScriptData().PLACEHOLDER_MAPPING.at(symbol);
      return placeholderPlain + StaticHelper::convertPtrAddressToString(this); // Adds the current object's ID to the placeholder to allow object-specific refactorings.
   }

   return symbol; // Nothing to replace.
}

int nextIndex(const std::string& script, const int current_index)
{
   int indexOfEqualSign = script.find(VARIABLE_DELIMITER, current_index);

   for (; indexOfEqualSign >= 0;) {
      int i = indexOfEqualSign + 1;
      for (; i < script.length() && std::isspace(script[i]); i++);
      if (i >= script.length()) return -1;
      if (script[i] == START_TAG_FOR_NESTED_VARIABLES[0] || script[i] == INSCR_BEG_TAG[0]) return indexOfEqualSign;
      indexOfEqualSign = script.find(VARIABLE_DELIMITER, i);
   }

   return -1;
}

std::string vfm::macro::Script::getRawScript() const
{
   return raw_script_;
}

void vfm::macro::Script::setRawScript(const std::string& script)
{
   raw_script_ = script;
}

RepresentableAsPDF vfm::macro::Script::repfactory_instanceFromScript(const std::string& script)
{
   std::shared_ptr<Script> dummy = std::make_shared<Script>(vfm_data_, vfm_parser_);
   addFailableChild(dummy, "");
   dummy->createInstanceFromScript(script);
   return dummy;
}

std::string Script::extractExpressionPart(const std::string& chain2)
{
   std::string chain = overwriteBrackets(chain2);

   int lastDotIndex = chain.length();
   int level = 0;

   for (int i = lastDotIndex - 1; i >= 0; i--) {
      if (level == 0 && chain.at(i) == METHOD_CHAIN_SEPARATOR.at(0)) {
         lastDotIndex = i;
      }
      else if (chain.at(i) == METHOD_PARS_BEGIN_TAG.at(0)) {
         level++;
      }
      else if (chain.at(i) == METHOD_PARS_END_TAG.at(0)) {
         level--;
      }
      else if (level == 0
         && !std::isalnum(chain.at(i))
         && chain.at(i) != ' '
         && chain.at(i) != '\n'
         && chain.at(i) != '\r'
         && chain.at(i) != '\t') {
         break;
      }
   }

   return chain2.substr(0, lastDotIndex);
}

std::string Script::overwriteBrackets(const std::string& s) 
{
   int index1 = s.find(START_TAG_FOR_NESTED_VARIABLES);
   int index2 = StaticHelper::findMatchingEndTagLevelwise(
      s,
      START_TAG_FOR_NESTED_VARIABLES,
      END_TAG_FOR_NESTED_VARIABLES,
      index1);

   if (index1 < 0 || index2 < 0) {
      return s;
   }

   std::string substring = s.substr(index1, index2 + END_TAG_FOR_NESTED_VARIABLES.length() - index1);

   return overwriteBrackets(StaticHelper::replaceAll(s, substring, singleCharSeq('-', substring.length())));
}

std::string Script::singleCharSeq(const char c, const int length) 
{
   std::string b;

   for (int i = 0; i < length; i++) {
      b += c;
   }

   return b;
}

std::vector<std::string> Script::getMethodSinaturesFromChain(const std::string& chainRest)
{
   if (StaticHelper::stringStartsWith(chainRest, METHOD_CHAIN_SEPARATOR)) {
      return getMethodSinaturesFromChain(chainRest.substr(1));
   }

   std::vector<std::string> signatures{};

   int nextPoint1 = StaticHelper::indexOfOnTopLevel(
      chainRest,
      { METHOD_CHAIN_SEPARATOR },
      0,
      START_TAG_FOR_NESTED_VARIABLES,
      END_TAG_FOR_NESTED_VARIABLES);

   int nextPoint2 = StaticHelper::indexOfOnTopLevel(
      chainRest,
      { METHOD_CHAIN_SEPARATOR },
      0,
      METHOD_PARS_BEGIN_TAG,
      METHOD_PARS_END_TAG);


   if (nextPoint1 < 0 || nextPoint2 < 0) {
      signatures.push_back(StaticHelper::trimAndReturn(chainRest));
      return signatures;
   }

   int nextPoint = (std::max)(nextPoint1, nextPoint2);

   std::string signature = chainRest.substr(0, nextPoint);
   std::string rest = chainRest.substr(nextPoint);

   signatures.push_back(StaticHelper::trimAndReturn(signature));

   auto remainder = getMethodSinaturesFromChain(rest);
   signatures.insert(signatures.end(), remainder.begin(), remainder.end());

   return signatures;
}

std::string vfm::macro::Script::evaluateAll(const std::string& string, const std::string& opening_tag, const std::string& closing_tag)
{
   int index1 = StaticHelper::indexOfFirstInnermostBeginBracket(string, opening_tag, closing_tag);
   int index2 = StaticHelper::findMatchingEndTagLevelwise(string, index1, opening_tag, closing_tag);

   if (index1 < 0 || index2 < 0) {
      return string;
   }

   index2 += opening_tag.size();
   std::string beforePart = string.substr(0, index1);
   std::string toEval = string.substr(index1 + opening_tag.length(), index2 - closing_tag.length() - (index1 + opening_tag.length()));
   std::string afterPart = string.substr(index2, string.length() - index2);

   auto split = StaticHelper::split(toEval, ROUNDING_DELIMITER);
   std::string round_to = "-1";

   if (split.size() == 2) {
      toEval = split.at(0);
      round_to = split.at(1);
   }
   else if (split.size() > 2) {
      addError("Malformed vfm expression '" + toEval + "' found.");
   }

   std::string newString = beforePart + evaluateExpression(toEval, round_to) + afterPart;

   return evaluateAll(newString, opening_tag, closing_tag);
}

std::string Script::formatExpression(const std::string& expression, const SyntaxFormat format)
{
   auto fmla = MathStruct::parseMathStruct(expression, vfm_parser_, vfm_data_);

   if (format == SyntaxFormat::vfm) {
      return fmla->serialize();
   }
   else if (format == SyntaxFormat::kratos2) {
      return fmla->serializeK2();
   }
   else if (format == SyntaxFormat::nuXmv) {
      return fmla->serializeNuSMV(getData(), getParser());
   }
   else {
      addError("Syntax format not supported.");
      return "#ERROR(Syntax format not supported.)";
   }
}

std::string Script::simplifyExpression(const std::string& expression)
{
   return simplification::simplifyFast(MathStruct::parseMathStruct(expression, vfm_parser_, vfm_data_)->toTermIfApplicable())->serialize();
}

std::string Script::toK2(const std::string& expression)
{
   std::pair<std::map<std::string, std::string>, std::string> dummy1{};
   int dummy2{};

   return MathStruct::parseMathStruct(expression, vfm_parser_, vfm_data_)->serializeK2({}, dummy1, dummy2);
}

std::string Script::evaluateExpression(const std::string& expression)
{
   return evaluateExpression(expression, "-1");
}

std::string Script::evaluateExpression(const std::string& expression, const std::string& decimals_str) 
{
   int decimals{-1};

   auto result = MathStruct::parseMathStruct(expression, vfm_parser_, vfm_data_)->eval(vfm_data_, vfm_parser_);


   if (!StaticHelper::isParsableAsFloat(decimals_str)) {
      addError("Rounding parameter '" + decimals_str + "' cannot be parsed to float/int.");
   }

   decimals = std::stof(decimals_str);

   //addDebug("Expression '" + expression + "' resulted in " + std::to_string(result) + (decimals >= 0 ? " (will be rounded to " + std::to_string(decimals) + " decimals)" : "") + ".");

   if (decimals < 0) {
      return std::to_string(result);
   }

   const double multiplier = std::pow(10.0, decimals);
   auto rounded = std::round(result * multiplier) / multiplier;
   auto rounded_str = std::to_string(rounded);

   if (rounded_str.find('.') != std::string::npos)
   {
      rounded_str = rounded_str.substr(0, rounded_str.find_last_not_of('0') + 1);
      if (rounded_str.find('.') == rounded_str.size() - 1)
      {
         rounded_str = rounded_str.substr(0, rounded_str.size() - 1);
      }
   }

   return rounded_str;
}

std::shared_ptr<DataPack> vfm::macro::Script::getData() const
{
   return vfm_data_;
}

inline std::shared_ptr<FormulaParser> Script::getParser() const
{
   return vfm_parser_;
}

inline std::string Script::getMyPath() const
{
   return StaticHelper::absPath(getData()->printHeap(MY_PATH_VARNAME, "."));
}

void Script::processSequence(const std::string& code, std::vector<std::string>& script_sequence) {
   std::string processed{ StaticHelper::trimAndReturn(code) };

   if (StaticHelper::isEmptyExceptWhiteSpaces(code)) {
      return;
   }

   std::string rest{};

   if (StaticHelper::stringStartsWith(processed, BEGIN_TAG_IN_SEQUENCE)
      && StaticHelper::stringEndsWith(processed, END_TAG_IN_SEQUENCE)) {
      // Sequence of scripts given.
      int indexTo{ StaticHelper::findMatchingEndTagLevelwise(
         processed, BEGIN_TAG_IN_SEQUENCE, END_TAG_IN_SEQUENCE, 0) };
      rest = processed.substr(indexTo + END_TAG_IN_SEQUENCE.length());
      processed = processed.substr(BEGIN_TAG_IN_SEQUENCE.length(), indexTo - BEGIN_TAG_IN_SEQUENCE.length());
      //processSequence(processed, script_sequence);
   }
   //else {

   std::string proc{ processed };

   if (StaticHelper::stringStartsWith(proc, INSCR_BEG_TAG) && StaticHelper::stringEndsWith(proc, INSCR_END_TAG)) {
      proc = proc.substr(INSCR_BEG_TAG.length(), proc.length() - INSCR_END_TAG.length() - INSCR_BEG_TAG.length());
   }

   script_sequence.push_back(proc);
   processSequence(rest, script_sequence);
}

void Script::createInstanceFromScript(const std::string& code)
{
   setRawScript(code);
   scriptSequence_.clear();
   processSequence(code, scriptSequence_);
}

std::string Script::evalItAllF(const std::string& n1Str, const std::string& n2Str, const std::function<float(float n1, float n2)> eval) {
   if (!StaticHelper::isParsableAsFloat(n1Str) || !StaticHelper::isParsableAsFloat(n2Str)) {
      addError("Operand '" + n1Str + "' and/or '" + n2Str + "' cannot be parsed to float.");
      return "0";
   }

   float n1 = std::stof(n1Str);
   float n2 = std::stof(n2Str);
   return std::to_string(eval(n1, n2));
}

std::string Script::evalItAllI(const std::string& n1Str, const std::string& n2Str, const std::function<long long(long long n1, long long n2)> eval) {
   if (!StaticHelper::isParsableAsInt(n1Str) || !StaticHelper::isParsableAsInt(n2Str)) {
      addError("Operand '" + n1Str + "' and/or '" + n2Str + "' cannot be parsed to int.");
      return "0";
   }

   long long n1 = std::stoll(n1Str);
   long long n2 = std::stoll(n2Str);
   return std::to_string(eval(n1, n2));
}

std::string Script::getTagFreeRawScript() {
   return StaticHelper::replaceManyTimes(getRawScript(), { INSCR_BEG_TAG, INSCR_END_TAG }, { "", "" });
}

std::string vfm::macro::Script::connectRoadGraphTo(const std::string& id2_str)
{
   if (!StaticHelper::isParsableAsFloat(getRawScript())) {
      addError("RoadGraph ID '" + getRawScript() + "' is not an integer.");
      return "#ERROR";
   }
   if (!StaticHelper::isParsableAsFloat(id2_str)) {
      addError("RoadGraph ID '" + id2_str + "' is not an integer.");
      return "#ERROR";
   }

   int id1{ (int)std::stof(getRawScript()) };
   int id2{ (int)std::stof(id2_str) };

   for (const auto& id : { id1, id2 }) {
      if (!road_graphs_.count(id)) {
         addError("RoadGraph with ID '" + std::to_string(id) + "' not found.");
         return "#ERROR";
      }
   }

   road_graphs_.at(id1)->addSuccessor(road_graphs_.at(id2));

   return "Road graph '" + std::to_string(id1) + "' connected to '" + std::to_string(id2) + "'.";
}

std::string vfm::macro::Script::storeRoadGraph(const std::string& filename)
{
   if (!StaticHelper::isParsableAsFloat(getRawScript())) {
      addError("RoadGraph ID '" + getRawScript() + "' is not an integer.");
      return "#ERROR";
   }

   int id{ (int) std::stof(getRawScript()) };

   if (!road_graphs_.count(id)) {
      addError("RoadGraph with ID '" + std::to_string(id) + "' not found.");
      return "#ERROR";
   }

   auto rg = road_graphs_.at(id);

   const bool infinite_highway{ rg->getNodeCount() == 1 };

   if (!vfm_data_->isVarDeclared("WIDTH_FACTOR_NON_INFINITE")) vfm_data_->addOrSetSingleVal("WIDTH_FACTOR_NON_INFINITE", 1);
   if (!vfm_data_->isVarDeclared("HEIGHT_FACTOR_NON_INFINITE")) vfm_data_->addOrSetSingleVal("HEIGHT_FACTOR_NON_INFINITE", 7);
   if (!vfm_data_->isVarDeclared("OFFSET_X_NON_INFINITE")) vfm_data_->addOrSetSingleVal("OFFSET_X_NON_INFINITE", 0);
   if (!vfm_data_->isVarDeclared("OFFSET_Y_NON_INFINITE")) vfm_data_->addOrSetSingleVal("OFFSET_Y_NON_INFINITE", 20);
   if (!vfm_data_->isVarDeclared("DIMENSION_X")) vfm_data_->addOrSetSingleVal("DIMENSION_X", 500);
   if (!vfm_data_->isVarDeclared("DIMENSION_Y")) vfm_data_->addOrSetSingleVal("DIMENSION_Y", 60);

   vfm::HighwayImage image{
      Env2D::getImageWidth(MAX_NUM_LANES_SIMPLE) * (infinite_highway ? 1 : (int) vfm_data_->getSingleVal("WIDTH_FACTOR_NON_INFINITE")),
      Env2D::getImageHeight() * (rg->getNodeCount() > 1 ? (int) vfm_data_->getSingleVal("HEIGHT_FACTOR_NON_INFINITE") : 1),
      std::make_shared<Plain2DTranslator>(),
      rg->getMyRoad().getNumLanes() };

   image.fillImg(BROWN);

   Rec2D bounding_box{ infinite_highway ? Rec2D{} : rg->getBoundingBox() };
   const float offset_x{ infinite_highway
      ? 60.0f
      : (int)vfm_data_->getSingleVal("OFFSET_X_NON_INFINITE")
   };

   const float offset_y{
      infinite_highway
      ?
      (float)rg->getMyRoad().getNumLanes() / 2.0f
      : (int)vfm_data_->getSingleVal("OFFSET_Y_NON_INFINITE")
   };

   image.paintRoadGraph(
      rg,
      { vfm_data_->getSingleVal("DIMENSION_X"), vfm_data_->getSingleVal("DIMENSION_Y") },
      {},
      true, offset_x, offset_y);

   image.store(filename);

   return "Road graph '" + std::to_string(id) + "' stored in '" + StaticHelper::absPath(filename) + "'.";
}

std::string vfm::macro::Script::createRoadGraph(const std::string& id)
{
   if (!StaticHelper::isParsableAsFloat(id)) {
      addError("RoadGraph with ID '" + id + "' CANNOT be created, because ID is not an integer.");
      return "#ERROR";
   }

   int rg_id{ std::stoi(id) };

   if (road_graphs_.count(rg_id)) {
      if (road_graphs_.at(rg_id)->parseProgram(getRawScript())) {
         return "Existing road graph '" + id + "' updated.";
      }
      else {
         return "#PARSING-ERROR";
      }
   }

   auto rg = std::make_shared<RoadGraph>(rg_id);

   if (rg_id == 0) {
      rg->my_road_.setEgo(std::make_shared<CarPars>(0, 0, 0, RoadGraph::EGO_MOCK_ID));
   }

   road_graphs_[rg_id] = rg;

   if (rg->parseProgram(getRawScript())) {
      return "Road graph '" + id + "' created.";
   } 
   else {
      return "#PARSING-ERROR";
   }
}

std::string vfm::macro::Script::processScript(
   const std::string& text,
   const DataPreparation data_prep,
   const bool only_one_step,
   const std::shared_ptr<DataPack> data_raw,
   const std::shared_ptr<FormulaParser> parser,
   const std::shared_ptr<Failable> father_failable,
   const SpecialOption option)
{

   auto data = data_raw;
   std::vector<std::string> messages{};
   std::stringstream strstr{};

   if (data) {
      messages.push_back("DataPack received");

      if (data_prep == DataPreparation::copy_data_pack_before_run || data_prep == DataPreparation::both) {
         messages.push_back("using new DataPack with data copied from the old one");
         data = std::make_shared<DataPack>();
         data->initializeValuesBy(data_raw);
      }
      else {
         messages.push_back("using existing DataPack (not a copy)");
      }

      if (data_prep == DataPreparation::reset_script_data_before_run || data_prep == DataPreparation::both) {
         messages.push_back("resetting script data");
         data->getScriptData().reset();
      }
      else {
         messages.push_back("keeping script data");
      }
   }
   else {
      messages.push_back("no DataPack received - creating a fresh one");
   }

   if (!messages.empty()) strstr << " <Notes: " << messages << ">";

   auto s = std::make_shared<Script>(data, parser);
   s->addNote("Processing script." + strstr.str());

   if (father_failable) {
      father_failable->addFailableChild(s);
   }

   s->addNote("Variable '" + MY_PATH_VARNAME + "' is set to '" + s->getMyPath() + "'.");

   if (option == SpecialOption::add_default_dynamic_methods) {
      s->addDefaultDynamicMathods();
   }

   s->applyDeclarationsAndPreprocessors(text, only_one_step);
   return s->getProcessedScript();
}
