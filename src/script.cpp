//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file

#include "vfmacro/script.h"
#include "gui/process_helper.h"
#include "geometry/bezier_functions.h"
#include "parser.h"
#include "simplification/simplification.h"
#include "geometry/images.h"
#include <cmath>


using namespace vfm;
using namespace macro;

std::map<std::string, std::shared_ptr<Script>> Script::known_chains_{};
std::map<std::string, std::vector<std::string>> Script::list_data_{};

std::map<std::string, std::string> Script::inscriptMethodDefinitions{};
std::map<std::string, int> Script::inscriptMethodParNums{};
std::map<std::string, std::string> Script::inscriptMethodParPatterns{};

std::set<std::string> Script::SPECIAL_SYMBOLS{};
std::map<std::string, std::string> Script::PLACEHOLDER_MAPPING{};
std::map<std::string, std::string> Script::PLACEHOLDER_INVERSE_MAPPING{};
int Script::cache_hits_{};
int Script::cache_misses_{};


vfm::macro::Script::Script(const std::shared_ptr<DataPack> data, const std::shared_ptr<FormulaParser> parser)
   : Failable("ScriptMacro"), vfm_data_(data), vfm_parser_(parser)
{
   if (KEEP_COMMENTS_IN_PLAIN_TEXT) {
      putPlaceholderMapping(BEGIN_COMMENT);
      putPlaceholderMapping(END_COMMENT);
   }
   else {
      SPECIAL_SYMBOLS.insert(BEGIN_COMMENT);
      SPECIAL_SYMBOLS.insert(END_COMMENT);
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

   SPECIAL_SYMBOLS.insert(PREPROCESSOR_FIELD_NAME);
   SPECIAL_SYMBOLS.insert(VARIABLE_DELIMITER);
   SPECIAL_SYMBOLS.insert(StaticHelper::makeString(END_VALUE));
   SPECIAL_SYMBOLS.insert(NOP_SYMBOL);
}

void Script::applyDeclarationsAndPreprocessors(const std::string& codeRaw2)
{
   raw_script_ = codeRaw2; // Nothing is done to rawScript code.

   if (codeRaw2.empty()) {
      processed_script_ = "";
      return;
   }

   std::string codeRaw = inferPlaceholdersForPlainText(codeRaw2);                              // Secure plain-text parts.
   processed_script_ = evaluateAll(codeRaw, EXPR_BEG_TAG_BEFORE, EXPR_END_TAG_BEFORE);         // Evaluate expressions BEFORE run.

   extractInscriptProcessors();                                                                // DO THE ACTUAL THING.

   processed_script_ = undoPlaceholdersForPlainText(processed_script_);                        // Undo placeholder securing.
   processed_script_ = StaticHelper::replaceAll(processed_script_, NOP_SYMBOL, "");            // Clear all NOP symbols from script.
   processed_script_ = evaluateAll(processed_script_, EXPR_BEG_TAG_AFTER, EXPR_END_TAG_AFTER); // Evaluate expressions AFTER run.
}

std::string vfm::macro::Script::getProcessedScript() const
{
   return processed_script_;
}

void Script::extractInscriptProcessors()
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

      if (method_part_begins_.count(trimmed)) {
         cache_hits_++;
         placeholder_for_inscript = method_part_begins_.at(trimmed).second;
      }
      else {
         cache_misses_++;
         if (methodPartBegin != preprocessorScript.length() && methodPartBegin >= 0) { // TODO: Do we need this??
            int leftTrim = preprocessorScript.size() - StaticHelper::ltrimAndReturn(preprocessorScript).size();

            method_part_begins_.insert({
               trimmed,
               { methodPartBegin - leftTrim, placeholder_for_inscript }
               });
         }
         else {
            method_part_begins_.insert({
               trimmed,
               { -1, placeholder_for_inscript }
               });
         }

         RepresentableAsPDF result = evaluateChain(removePreprocessors(raw_script_), preprocessorScript);
         placeholder_for_inscript = result->getRawScript();
         method_part_begins_[trimmed].second = placeholder_for_inscript;
      }

      std::string placeholderFinal = checkForPlainTextTags(placeholder_for_inscript);
      processed_script_ = partBefore + placeholderFinal + partAfter;
      indexOfPrep = findNextInscriptPos();

      if (i++ % 100 == 0) {
         addNote(""
            + std::to_string(known_chains_.size()) + " known_chains_; "
            + std::to_string(method_part_begins_.size()) + " method_part_begins_; "
            + std::to_string(list_data_.size()) + " list_data_; "
            + std::to_string(processed_script_.size()) + " script size; "
            + std::to_string(cache_hits_) + "/" + std::to_string(cache_misses_) + " cache hits/misses; "
         );
      }
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

RepresentableAsPDF Script::evaluateChain(const std::string& repScrThis, const std::string& chain)
{
   std::string processedChain{ StaticHelper::trimAndReturn(chain) };
   std::string processedRaw{ processedChain };

   if (known_chains_.count(processedRaw)) {
      cache_hits_++;
      return known_chains_.at(processedRaw);
   }

   cache_misses_++;

   RepresentableAsPDF repToProcess{};
   std::shared_ptr<int> methodBegin = method_part_begins_.count(processedRaw) && method_part_begins_.at(processedRaw).first >= 0
      ? std::make_shared<int>(method_part_begins_.at(processedRaw).first) 
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

         if (!repToProcess) { // String is no script, but plain expression.
            repToProcess = std::make_shared<Script>(vfm_data_, vfm_parser_);
            processedChain = createDummyrep(processedChain, repToProcess);
         }
         else {
            processedChain = processedChain.substr(repPart.length() + INSCR_BEG_TAG.length() + INSCR_END_TAG.length());
         }
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

         if (!repToProcess) { // String is no script, but plain expression.
            repToProcess = std::make_shared<Script>(vfm_data_, vfm_parser_);
            processedChain = createDummyrep(processedChain, repToProcess);
         }
         else {
            processedChain = "";
         }
      }
   }

   addFailableChild(repToProcess, "");

   if (StaticHelper::isEmptyExceptWhiteSpaces(processedChain)) {
      if (processedRaw.size() < 50) known_chains_[processedRaw] = repToProcess;
      return repToProcess;
   }

   std::vector<std::string> methodSignatures = getMethodSinaturesFromChain(processedChain);
   std::vector<std::string> methodSignaturesArray{};
   methodSignaturesArray.resize(methodSignatures.size());

   for (int i = 0; i < methodSignatures.size(); i++) {
      methodSignaturesArray[i] = methodSignatures.at(i);
   }

   RepresentableAsPDF apply_method_chain = applyMethodChain(repToProcess, methodSignaturesArray);
   if (processedRaw.size() < 50) known_chains_[processedRaw] = apply_method_chain;

   return apply_method_chain;
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

   std::vector<std::string> actualParameters = getParametersFor(pars, rep);
   std::string newScript = rep->applyMethodString(methodName, actualParameters);

   return repfactory_instanceFromScript(newScript);
}

std::string fromBooltoString(const bool b)
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
   if (inscriptMethodDefinitions.count(methodName)) {
      addError("Dynamic method '" + methodName + "' already exists.");
      return "";
   }

   if (!StaticHelper::isParsableAsFloat(numPars)) {
      addError("Parameter number '" + numPars + "' in definition of dynamic method '" + methodName + "' not parsable as float/int.");
      return "";
   }

   inscriptMethodDefinitions.insert({ methodName, getRawScript() });
   inscriptMethodParNums.insert({ methodName, (int)std::stof(numPars) });
   inscriptMethodParPatterns.insert({ methodName, parameterPattern });

   addNote("New method '" + methodName + "' with "
      + std::to_string(inscriptMethodParNums.at(methodName)) + " parameters defined as '"
      + inscriptMethodDefinitions.at(methodName) + "'. Parameter pattern: '" + parameterPattern + "'.");

   return "";
}

std::string Script::executeCommand(const std::string& methodName, const std::vector<std::string>& pars)
{
   std::string methodBody = inscriptMethodDefinitions.at(methodName);
   std::string pattern = inscriptMethodParPatterns.at(methodName);

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
   if (method_name == "for" && parameters.size() == 2) return forloop(parameters.at(0), parameters.at(1));
   else if (method_name == "for" && parameters.size() == 3) return forloop(parameters.at(0), parameters.at(1), parameters.at(2));
   else if (method_name == "for" && parameters.size() == 4) return forloop(parameters.at(0), parameters.at(1), parameters.at(2), parameters.at(3));
   else if (method_name == "for" && parameters.size() == 5) return forloop(parameters.at(0), parameters.at(1), parameters.at(2), parameters.at(3), parameters.at(4));
   else if (method_name == "serialize" && parameters.size() == 0) return formatExpression(getRawScript(), SyntaxFormat::vfm);
   else if (method_name == "serializeK2" && parameters.size() == 0) return toK2(getRawScript());
   else if (method_name == "serializeNuXmv" && parameters.size() == 0) {
      return formatExpression(getRawScript(), SyntaxFormat::nuXmv);
   }
   else if (method_name == "atVfmTupel" && parameters.size() == 1) {
      auto fmla = MathStruct::parseMathStruct(StaticHelper::replaceAll(getRawScript(), ";", ","), getParser(), getData());
      assert(StaticHelper::isParsableAsInt(parameters[0]));
      const int at{ std::stoi(parameters[0]) };
      return fmla->getTermsJumpIntoCompounds()[at]->serializePlainOldVFMStyle();
   }
   else if (method_name == "simplify" && parameters.size() == 0) return simplifyExpression(getRawScript());
   else if (method_name == "eval" && parameters.size() == 0) return evaluateExpression(getRawScript());
   else if (method_name == "eval" && parameters.size() == 1) return evaluateExpression(getRawScript(), parameters.at(0));
   else if (method_name == "nil" && parameters.size() == 0) return nil();
   else if (method_name == "id" && parameters.size() == 0) return id();
   else if (method_name == "idd" && parameters.size() == 0) return idd();
   else if (method_name == "if" && parameters.size() == 1) return ifChoice(parameters.at(0));
   else if (method_name == "at" && parameters.size() == 1) return element(parameters.at(0));
   else if (method_name == "substr" && parameters.size() == 2) return substring(parameters.at(0), parameters.at(1));
   else if (method_name == "split" && parameters.size() == 1) {
      auto spl = StaticHelper::split(getRawScript(), parameters[0]);
      std::string res{};
      for (const auto& s : spl) {
         res += BEGIN_TAG_IN_SEQUENCE + s + END_TAG_IN_SEQUENCE;
      }
      return res;
   }
   else if (method_name == "newMethod" && parameters.size() == 2) return newMethod(parameters.at(0), parameters.at(1));
   else if (method_name == "newMethod" && parameters.size() == 3) return newMethodD(parameters.at(0), parameters.at(1), parameters.at(2));

   else if (method_name == "strsize" && parameters.size() == 0) return std::to_string(getRawScript().length());
   else if (method_name == "seqlength" && parameters.size() == 0) return std::to_string(scriptSequence_.size());
   else if (method_name == "firstLetterCapital" && parameters.size() == 0) return StaticHelper::firstLetterCapital(getRawScript());
   else if (method_name == "toUpperCamelCase" && parameters.size() == 0) return StaticHelper::toUpperCamelCase(getRawScript());
   else if (method_name == "toUpperCase" && parameters.size() == 0) return StaticHelper::toUpperCase(getRawScript());
   else if (method_name == "toLowerCase" && parameters.size() == 0) return StaticHelper::toLowerCase(getRawScript());
   else if (method_name == "removeLastFileExtension" && parameters.size() == 0) return StaticHelper::removeLastFileExtension(getRawScript());
   else if (method_name == "removeLastFileExtension" && parameters.size() == 1) return StaticHelper::removeLastFileExtension(getRawScript(), parameters.at(0));
   else if (method_name == "getLastFileExtension" && parameters.size() == 0) return StaticHelper::getLastFileExtension(getRawScript());
   else if (method_name == "getLastFileExtension" && parameters.size() == 1) return StaticHelper::getLastFileExtension(getRawScript(), parameters.at(0));
   else if (method_name == "substrComplement" && parameters.size() == 2) return StaticHelper::substrComplement(getRawScript(), stringToFloat(parameters.at(0)), stringToFloat(parameters.at(1)));
   else if (method_name == "startsWithUppercase" && parameters.size() == 0) return fromBooltoString(StaticHelper::startsWithUppercase(getRawScript()));
   else if (method_name == "stringStartsWith" && parameters.size() == 1) return fromBooltoString(StaticHelper::stringStartsWith(getRawScript(), parameters.at(0)));
   else if (method_name == "stringStartsWith" && parameters.size() == 2) return fromBooltoString(StaticHelper::stringStartsWith(getRawScript(), parameters.at(0), stringToFloat(parameters.at(0))));
   else if (method_name == "stringEndsWith" && parameters.size() == 1) return fromBooltoString(StaticHelper::stringEndsWith(getRawScript(), parameters.at(0)));
   else if (method_name == "stringContains" && parameters.size() == 1) return fromBooltoString(StaticHelper::stringContains(getRawScript(), parameters.at(0)));
   else if (method_name == "shortenToMaxSize" && parameters.size() == 1) return StaticHelper::shortenToMaxSize(getRawScript(), stringToFloat(parameters.at(0)));
   else if (method_name == "shortenToMaxSize" && parameters.size() == 2) return StaticHelper::shortenToMaxSize(getRawScript(), stringToFloat(parameters.at(0)), StaticHelper::isBooleanTrue(parameters.at(1)));
   else if (method_name == "shortenToMaxSize" && parameters.size() == 3) return StaticHelper::shortenToMaxSize(getRawScript(), stringToFloat(parameters.at(0)), StaticHelper::isBooleanTrue(parameters.at(1)), stringToFloat(parameters.at(2)));
   else if (method_name == "shortenToMaxSize" && parameters.size() == 4) return StaticHelper::shortenToMaxSize(getRawScript(), stringToFloat(parameters.at(0)), StaticHelper::isBooleanTrue(parameters.at(1)), stringToFloat(parameters.at(2)), stringToFloat(parameters.at(3)));
   else if (method_name == "shortenInTheMiddle" && parameters.size() == 1) return StaticHelper::shortenInTheMiddle(getRawScript(), stringToFloat(parameters.at(0)));
   else if (method_name == "shortenInTheMiddle" && parameters.size() == 2) return StaticHelper::shortenInTheMiddle(getRawScript(), stringToFloat(parameters.at(0)), stringToFloat(parameters.at(1)));
   else if (method_name == "absPath" && parameters.size() == 0) return StaticHelper::absPath(getRawScript());
   else if (method_name == "zeroPaddedNumStr" && parameters.size() == 1) return StaticHelper::zeroPaddedNumStr(stringToFloat(getRawScript()), stringToFloat(parameters.at(0)));
   else if (method_name == "removeWhiteSpace" && parameters.size() == 0) return StaticHelper::removeWhiteSpace(getRawScript());
   else if (method_name == "isEmptyExceptWhiteSpaces" && parameters.size() == 0) return fromBooltoString(StaticHelper::isEmptyExceptWhiteSpaces(getRawScript()));
   else if (method_name == "removeMultiLineComments" && parameters.size() == 0) return StaticHelper::removeMultiLineComments(getRawScript());
   else if (method_name == "removeMultiLineComments" && parameters.size() == 2) return StaticHelper::removeMultiLineComments(getRawScript(), parameters.at(0), parameters.at(1));
   else if (method_name == "removeSingleLineComments" && parameters.size() == 0) return StaticHelper::removeSingleLineComments(getRawScript());
   else if (method_name == "removeSingleLineComments" && parameters.size() == 1) return StaticHelper::removeSingleLineComments(getRawScript(), parameters.at(0));
   else if (method_name == "removeComments" && parameters.size() == 0) return StaticHelper::removeComments(getRawScript());
   else if (method_name == "removePartsOutsideOf" && parameters.size() == 2) return StaticHelper::removePartsOutsideOf(getRawScript(), parameters.at(0), parameters.at(1));
   else if (method_name == "removeBlankLines" && parameters.size() == 0) return StaticHelper::removeBlankLines(getRawScript());
   else if (method_name == "wrap" && parameters.size() == 1) return StaticHelper::wrap(getRawScript(), stringToFloat(parameters.at(0)));
   else if (method_name == "replaceAll" && parameters.size() == 2) return StaticHelper::replaceAll(getRawScript(), parameters.at(0), parameters.at(1));
   else if (method_name == "replaceAllRegex" && parameters.size() == 2) return StaticHelper::replaceAllRegex(getRawScript(), parameters.at(0), parameters.at(1));
   else if (method_name == "isParsableAsFloat" && parameters.size() == 0) return fromBooltoString(StaticHelper::isParsableAsFloat(getRawScript()));
   else if (method_name == "isParsableAsInt" && parameters.size() == 0) return fromBooltoString(StaticHelper::isParsableAsInt(getRawScript()));
   else if (method_name == "isAlpha" && parameters.size() == 0) return fromBooltoString(StaticHelper::isAlpha(getRawScript()));
   else if (method_name == "isAlphaNumeric" && parameters.size() == 0) return fromBooltoString(StaticHelper::isAlphaNumeric(getRawScript()));
   else if (method_name == "isAlphaNumericOrUnderscore" && parameters.size() == 0) return fromBooltoString(StaticHelper::isAlphaNumericOrUnderscore(getRawScript()));
   else if (method_name == "isAlphaNumericOrUnderscoreOrColonOrComma" && parameters.size() == 0) return fromBooltoString(StaticHelper::isAlphaNumericOrUnderscoreOrColonOrComma(getRawScript()));
   else if (method_name == "isValidVfmVarName" && parameters.size() == 0) return fromBooltoString(StaticHelper::isValidVfmVarName(getRawScript()));
   else if (method_name == "stringConformsTo" && parameters.size() == 1) return fromBooltoString(StaticHelper::stringConformsTo(getRawScript(), std::regex(parameters.at(0))));
   else if (method_name == "getFileNameFromPath" && parameters.size() == 0) return StaticHelper::getFileNameFromPath(getRawScript());
   else if (method_name == "getFileNameFromPathWithoutExt" && parameters.size() == 0) return StaticHelper::getFileNameFromPathWithoutExt(getRawScript());
   else if (method_name == "removeFileNameFromPath" && parameters.size() == 0) return StaticHelper::removeFileNameFromPath(getRawScript());
   else if (method_name == "readFile" && parameters.size() == 0) return StaticHelper::readFile(getRawScript());
   else if (method_name == "fromPascalToCamelCase" && parameters.size() == 0) return StaticHelper::fromPascalToCamelCase(getRawScript());
   else if (method_name == "replaceSpecialCharsHTML_G" && parameters.size() == 0) return StaticHelper::replaceSpecialCharsHTML_G(getRawScript());
   else if (method_name == "replaceMathHTML" && parameters.size() == 0) return StaticHelper::replaceMathHTML(getRawScript());
   else if (method_name == "makeVarNameNuSMVReady" && parameters.size() == 0) return StaticHelper::makeVarNameNuSMVReady(getRawScript());
   else if (method_name == "safeString" && parameters.size() == 0) return StaticHelper::safeString(getRawScript());
   else if (method_name == "fromSafeString" && parameters.size() == 0) return StaticHelper::fromSafeString(getRawScript());
   else if (method_name == "levensteinDistance" && parameters.size() == 1) return std::to_string(StaticHelper::levensteinDistance(getRawScript(), parameters.at(0)));
   else if (method_name == "executeSystemCommand" && parameters.size() == 0) return std::to_string(StaticHelper::executeSystemCommand(getRawScript()));
   else if (method_name == "exec" && parameters.size() == 0) { return StaticHelper::exec(getRawScript()); }
   else if (method_name == "writeTextToFile" && parameters.size() == 1) { StaticHelper::writeTextToFile(getRawScript(), parameters.at(0)); return getRawScript(); }
   else if (method_name == "writeTextToFile" && parameters.size() == 2) { StaticHelper::writeTextToFile(getRawScript(), parameters.at(0), StaticHelper::isBooleanTrue(parameters.at(1))); return getRawScript(); }
   else if (method_name == "timestamp" && parameters.size() == 0) { return StaticHelper::timeStamp(); }
   else if (method_name == "morty" && parameters.size() == 0) { auto rep = getRawScript(); return StaticHelper::replaceAll(rep + MORTY_ASCII_ART, "\n", "\n" + rep); }
   else if (method_name == "rick" && parameters.size() == 0) { return getRawScript() + RICK; }

   else if (method_name == "mypath" && parameters.size() == 0) { return getMyPath(); }
   else if (method_name == "setScriptVar" && parameters.size() == 1) {
      std::string varname{ parameters.at(0) };

      if (list_data_.count(varname)) {
         std::string error{ "Variable '" + varname + "' has already been declared." };
         addError(error);
         return "#" + error + "#";
      }

      list_data_[varname] = { getRawScript() };

      return "";
   }
   else if (method_name == "scriptVar" && parameters.size() == 0) {
      std::string varname{ getRawScript() };

      if (!list_data_.count(varname)) {
         std::string error{ "Variable '" + varname + "' has not been declared." };
         addError(error);
         return "#" + error + "#";
      }

      return list_data_.at(varname).at(0);
   }

   else if (method_name == "include" && parameters.size() == 0) {
      std::string filepath{ StaticHelper::absPath(getMyPath()) + "/" + getRawScript() };

      if (StaticHelper::existsFileSafe(filepath)) {
         return StaticHelper::readFile(filepath);
      }
      else {
         addFatalError("File '" + filepath + "' not found.");
         return "#FILE_NOT_FOUND<" + filepath + ">";
      }
   }

   else if (method_name == "vfm_variable_declared" && parameters.size() == 0) return fromBooltoString(getData()->isVarDeclared(getRawScript()));
   else if (method_name == "vfm_variable_undeclared" && parameters.size() == 0) return fromBooltoString(!getData()->isVarDeclared(getRawScript()));
   else if (method_name == "sethard" && parameters.size() == 1) return sethard(parameters.at(0));
   else if (method_name == "smeq" && parameters.size() == 2) return exsmeq(parameters.at(0), parameters.at(1));
   else if (method_name == "sm" && parameters.size() == 2) return exsm(parameters.at(0), parameters.at(1));
   else if (method_name == "greq" && parameters.size() == 2) return exgreq(parameters.at(0), parameters.at(1));
   else if (method_name == "gr" && parameters.size() == 2) return exgr(parameters.at(0), parameters.at(1));
   else if (method_name == "eq" && parameters.size() == 2) return exeq(parameters.at(0), parameters.at(1));
   else if (method_name == "neq" && parameters.size() == 2) return exneq(parameters.at(0), parameters.at(1));
   else if (method_name == "add" && parameters.size() == 1) return exadd(parameters.at(0));
   else if (method_name == "sub" && parameters.size() == 1) return exsub(parameters.at(0));
   else if (method_name == "mult" && parameters.size() == 1) return exmult(parameters.at(0));
   else if (method_name == "div" && parameters.size() == 1) return exdiv(parameters.at(0));
   else if (method_name == "pow" && parameters.size() == 1) return expow(parameters.at(0));
   else if (method_name == "mod" && parameters.size() == 1) return exmod(parameters.at(0));
   else if (method_name == "not" && parameters.size() == 0) return exnot();
   else if (method_name == "space" && parameters.size() == 0) return space();
   else if (method_name == "arclengthCubicBezierFromStreetTopology" && parameters.size() == 3) return arclengthCubicBezierFromStreetTopology(getRawScript(), parameters.at(0), parameters.at(1), parameters.at(2));
   else if (method_name == "PIDs" && parameters.size() == 0) {
      auto pids = Process().getPIDs(getRawScript());
      std::string pids_str{};
      for (const auto& pid : pids) pids_str += std::to_string(pid) + ",";
      return pids_str;
   }
   else if (method_name == "KILLPIDs" && parameters.size() == 0) {
      std::string res{};
      for (const auto& pid : StaticHelper::split(getRawScript(), ",")) {
         if (!pid.empty()) res += std::to_string(Process().killByPID(std::stoi(pid)));
      }
      return res;
   }
   else if (method_name == "vfmheap" && parameters.size() == 0) return getData()->toStringHeap();
   else if (method_name == "vfmdata" && parameters.size() == 0) return getData()->toString();
   else if (method_name == "vfmfunc" && parameters.size() == 0) {
      auto parser = getParser();
      auto ops = parser->getAllOps();
      std::string res{};

      for (const auto& op : ops) {
         for (const auto& par_num : op.second) {
            auto fmla = parser->getDynamicTermMeta(op.first, par_num.first);
            res += par_num.second.serialize() + (fmla ? "\t ---> \t" + fmla->serializePlainOldVFMStyle(MathStruct::SerializationSpecial::enforce_square_array_brackets) : "\t(built-in)") + "\n";
         }
      }

      return res;
   }
   else if (method_name == "printHeap" && parameters.size() == 0) {
      return getData()->printHeap(getRawScript());
   }
   else if (method_name == "stringToHeap" && parameters.size() == 1) {
      getData()->addStringToDataPack(getRawScript(), parameters[0]);
      return parameters[0] + " set to point at '" + getRawScript() + "' in heap.";
      }

   else if (method_name == "listElement" && parameters.size() == 1) {
      if (!list_data_.count(getRawScript())) {
         std::string error_str{ "#ERROR<list '" + getRawScript() + "' not found by listElement method>" };
         addError(error_str);
         return error_str;
      }

      if (!StaticHelper::isParsableAsFloat(parameters[0])) {
         std::string error_str{ "#ERROR<index '" + parameters[0] + "' cannot be interpreted as number for listElement method>"};
         addError(error_str);
         return error_str;
      }

      int index{ std::stoi(parameters[0]) };

      if (index < 0 || index >= list_data_[getRawScript()].size()) {
         std::string error_str{ "#ERROR<index '" + parameters[0] + "' is out of bounds for list '" + getRawScript() + "' (size " + std::to_string(list_data_[getRawScript()].size()) + ") in method listElement>"};
         addError(error_str);
         return error_str;
      }

      return list_data_[getRawScript()][index];
   }
   else if (method_name == "clearList" && parameters.size() == 0) {
      if (!list_data_.count(getRawScript())) {
         list_data_[getRawScript()] = {};
      }

      list_data_[getRawScript()].clear();
   }
   else if (method_name == "asArray" && parameters.size() == 0) {
      std::string list_str{};

      for (const auto& el : list_data_[getRawScript()]) {
         list_str += BEGIN_TAG_IN_SEQUENCE + el + END_TAG_IN_SEQUENCE;
      }

      return StaticHelper::trimAndReturn(list_str);
   }
   else if (method_name == "printList" && parameters.size() == 0) {
      if (list_data_.count(getRawScript())) {
         std::string list_str{};

         for (const auto& el : list_data_[getRawScript()]) {
            list_str += el + "\n";
         }

         return StaticHelper::trimAndReturn(list_str);
      }
      else {
         std::string error_str{ "#ERROR<list '" + getRawScript() + "' not found by printList method>" };
         addError(error_str);
         return error_str;
      }
   }
   else if (method_name == "pushBack" && parameters.size() == 1) {
      if (!list_data_.count(getRawScript())) {
         list_data_[getRawScript()] = {};
      }
      list_data_[getRawScript()].push_back(parameters[0]);
      return "";
      }
   else if (method_name == "pushBack" && parameters.size() > 1) { // This is a hack to introduce comma which is usually separator of arguments.
      std::string appended{ parameters[0] };

      for (int i = 1; i < parameters.size(); i++) {
         appended += "," + parameters[i];
      }

      return applyMethodString(method_name, { appended });
   }

   else if (inscriptMethodDefinitions.count(method_name) && inscriptMethodParNums.at(method_name) == parameters.size()) {
      return executeCommand(method_name, parameters);
   }

   std::string pars{ "'" + getRawScript() + "' (self)"};
   for (const auto& s : parameters) {
      pars += ", '" + s + "'";
   }

   std::string error_str{ "No method '" + method_name + "' found which takes " + std::to_string(parameters.size()) + " arguments. Arguments are: {" + pars + "}." };

   addError(error_str);
   return "#INVALID(" + error_str + ")";
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

std::vector<std::string> Script::getParametersFor(const std::vector<std::string>& rawPars, const RepresentableAsPDF this_rep)
{
   std::vector<std::string> parameters;
   parameters.resize(rawPars.size());

   for (int i = 0; i < rawPars.size(); i++) {
      RepresentableAsPDF processedPar = evaluateChain(removePreprocessors(this_rep->getRawScript()), rawPars[i]);
      parameters[i] = processedPar->getRawScript();
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
   std::string s{};
   while (StaticHelper::stringContains(processed_script_, INSCR_END_TAG + s + INSCR_PRIORITY_SYMB)) {
      s += INSCR_PRIORITY_SYMB;
   }

   int pos = processed_script_.find(INSCR_END_TAG + s);
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
            + processed_script_.substr(starPos + s.length());

         return i;
      }
   }

   return -1;
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

std::string Script::removePreprocessors(const std::string& script)
{
   std::string newScript = script;

   while (StaticHelper::stringContains(newScript, INSCR_END_TAG + INSCR_PRIORITY_SYMB)) {
      newScript = StaticHelper::replaceAll(newScript, INSCR_END_TAG + INSCR_PRIORITY_SYMB, INSCR_END_TAG);
   }

   int indexOf = newScript.find(INSCR_BEG_TAG);

   while (indexOf >= 0) {
      auto processed = StaticHelper::extractFirstSubstringLevelwise(newScript, INSCR_BEG_TAG, INSCR_END_TAG, indexOf);

      if (!processed) return "#Error in script processing. No matching pair of '" + INSCR_BEG_TAG + "' and '" + INSCR_END_TAG + "'.";

      std::string preprocessorScript = *processed;

      int lengthOfPreprocessor = preprocessorScript.length() + INSCR_BEG_TAG.length() + INSCR_END_TAG.length();
      std::string partBefore = newScript.substr(0, indexOf);
      std::string partAfter = newScript.substr(indexOf + lengthOfPreprocessor);

      // Scale defined.
      int indexOf2 = preprocessorScript.find("|");

      if (indexOf2 >= 0) {
         std::string scaleStr = preprocessorScript.substr(0, indexOf2);
         if (StaticHelper::isParsableAsFloat(scaleStr)) {
            preprocessorScript = StaticHelper::trimAndReturn(preprocessorScript.substr(indexOf2 + 1));
         }
      }

      if (indexOf > 1 && newScript.at(indexOf - 1) == ASSIGNMENT_OPERATOR) {
         int indexOfFilenamebegin = partBefore.length() - 1;
         while (indexOfFilenamebegin > 0 && !std::isspace(partBefore.at(indexOfFilenamebegin))) indexOfFilenamebegin--;
         partBefore = partBefore.substr(0, indexOfFilenamebegin + 1);
      }

      int indexCurr = getNextNonInscriptPosition(partAfter);
      indexCurr = (std::min)(indexCurr, (int) partAfter.length());
      partAfter = partAfter.substr(indexCurr);
      newScript = partBefore + partAfter;
      indexOf = newScript.find(INSCR_BEG_TAG);
   }

   return newScript;
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

   searchList.resize(PLACEHOLDER_MAPPING.size());
   replacementList.resize(PLACEHOLDER_MAPPING.size());

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
   for (const auto& symbol : PLACEHOLDER_MAPPING) {
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

   PLACEHOLDER_MAPPING.insert({ symbolName, placeholderPlain });
   PLACEHOLDER_INVERSE_MAPPING.insert({ placeholderPlain, symbolName });
   SPECIAL_SYMBOLS.insert(symbolName);
}

std::string Script::placeholderToSymbol(const std::string& placeholder) {
   std::string placeholderPlain = StaticHelper::replaceAll(placeholder, StaticHelper::convertPtrAddressToString(this), "");

   if (PLACEHOLDER_MAPPING.count(placeholderPlain)) {
      std::string symbol = PLACEHOLDER_INVERSE_MAPPING.at(placeholderPlain);
      return symbol;
   }

   return placeholder;
}

std::string Script::symbolToPlaceholder(const std::string& symbol) {
   if (PLACEHOLDER_MAPPING.count(symbol)) {
      std::string placeholderPlain = PLACEHOLDER_MAPPING.at(symbol);
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

RepresentableAsPDF vfm::macro::Script::copy() const
{
   auto copy_script = std::make_shared<Script>(vfm_data_, vfm_parser_);

   addFailableChild(copy_script, "");

   copy_script->raw_script_ = raw_script_;
   copy_script->processed_script_ = processed_script_;
   copy_script->method_part_begins_ = method_part_begins_;

   return copy_script;
}

RepresentableAsPDF vfm::macro::Script::repfactory_instanceFromScript(const std::string& script)
{
   std::shared_ptr<Script> dummy = std::make_shared<Script>(vfm_data_, vfm_parser_);
   addFailableChild(dummy, "");
   dummy->createInstanceFromScript(script);
   return dummy;
}

std::string vfm::macro::Script::createDummyrep(const std::string& processed_raw, RepresentableAsPDF repToProcess)
{
   std::string processed;
   repToProcess->createInstanceFromScript(processed);
   std::string extract_expression_part = extractExpressionPart(processed);
   repToProcess->createInstanceFromScript(extract_expression_part);

   processed = processed.substr(extract_expression_part.length());

   if (StaticHelper::stringStartsWith(processed, METHOD_CHAIN_SEPARATOR)) {
      processed = processed.substr(METHOD_CHAIN_SEPARATOR.length());
   }

   return processed;
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

std::string Script::evalItAll(const std::string& n1Str, const std::string& n2Str, const std::function<float(float n1, float n2)> eval) {
   if (!StaticHelper::isParsableAsFloat(n1Str) || !StaticHelper::isParsableAsFloat(n2Str)) {
      addError("Operand '" + n1Str + "' and/or '" + n2Str + "' cannot be parsed to float/int.");
      return "0";
   }

   float n1 = std::stof(n1Str);
   float n2 = std::stof(n2Str);
   return std::to_string(eval(n1, n2));
}

std::string Script::getTagFreeRawScript() {
   return StaticHelper::replaceManyTimes(getRawScript(), { INSCR_BEG_TAG, INSCR_END_TAG }, { "", "" });
}

void vfm::macro::Script::clearStaticData()
{
   list_data_.clear();
   known_chains_.clear();
   inscriptMethodDefinitions.clear();
   inscriptMethodParNums.clear();
   inscriptMethodParPatterns.clear();
   PLACEHOLDER_MAPPING.clear();
   PLACEHOLDER_INVERSE_MAPPING.clear();
   cache_hits_ = 0;
   cache_misses_ = 0;
}

std::string vfm::macro::Script::processScript(
   const std::string& text,
   const std::shared_ptr<DataPack> data,
   const std::shared_ptr<FormulaParser> parser,
   const std::shared_ptr<Failable> father_failable)
{
   clearStaticData();

   auto s = std::make_shared<Script>(data ? data : std::make_shared<DataPack>(), parser ? parser : SingletonFormulaParser::getInstance());
   
   if (father_failable) {
      father_failable->addFailableChild(s);
   }

   s->addNote("Processing script. Variable '" + MY_PATH_VARNAME + "' is set to '" + s->getMyPath() + "'.");

   s->applyDeclarationsAndPreprocessors(text);
   return s->getProcessedScript();
}
