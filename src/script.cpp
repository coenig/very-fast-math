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

bool Script::ignorePreprocessorsAndAnimateOnce{};

ScriptTree::ScriptTree() : ScriptTree(nullptr) {}
ScriptTree::ScriptTree(std::shared_ptr<ScriptTree> father) : Failable("ScriptTree"), father_{ father } {}
std::map<std::string, RepresentableAsPDF> Script::knownChains{};
std::map<std::string, RepresentableAsPDF> Script::knownReps{};
std::map<std::string, std::string> Script::knownPreprocessors{};
std::map<std::string, std::vector<std::string>> Script::list_data_{};

std::map<std::string, std::string> DummyRepresentable::inscriptMethodDefinitions{};
std::map<std::string, int> DummyRepresentable::inscriptMethodParNums{};
std::map<std::string, std::string> DummyRepresentable::inscriptMethodParPatterns{};

std::set<std::string> Script::VARIABLES_MAYBE{};

//DummyRepresentable::DummyRepresentable(const std::shared_ptr<Script> repToEmbed) 
//   : DummyRepresentable(repToEmbed, std::make_shared<DataPack>(), SingletonFormulaParser::getInstance())
//{}

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
   putPlaceholderMapping(DECL_BEG_TAG);
   putPlaceholderMapping(DECL_END_TAG);
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

   SPECIAL_SYMBOLS.insert(THIS_NAME);
   SPECIAL_SYMBOLS.insert(PREPROCESSOR_FIELD_NAME);
   SPECIAL_SYMBOLS.insert(VARIABLE_DELIMITER);
   SPECIAL_SYMBOLS.insert(StaticHelper::makeString(END_VALUE));
   SPECIAL_SYMBOLS.insert(ANIMATE_FIELD_NAME);
   SPECIAL_SYMBOLS.insert(EXERCISE_FIELD_LONG_NAME);
   SPECIAL_SYMBOLS.insert(EXERCISE_FIELD_SHORT_NAME);
   SPECIAL_SYMBOLS.insert(NULL_VALUE);
   SPECIAL_SYMBOLS.insert(PREAMBLE_FOR_NON_SCRIPT_METHODS);
   SPECIAL_SYMBOLS.insert(NOP_SYMBOL);

   IGNORE_BEG_TAGS_IN_DECL.push_back(INSCR_BEG_TAG);
   IGNORE_BEG_TAGS_IN_DECL.push_back(INSCR_BEG_TAG_FOR_INTERNAL_USAGE);
   IGNORE_END_TAGS_IN_DECL.push_back(INSCR_END_TAG);
   IGNORE_END_TAGS_IN_DECL.push_back(INSCR_END_TAG_FOR_INTERNAL_USAGE);
}

std::string Script::applyDeclarationsAndPreprocessors(const std::string& codeRaw2, const int debugLevel) 
{
   rawScript = codeRaw2; // Nothing is done to rawScript code.
   resetDebugVars();

   if (codeRaw2.empty()) {
      processedScript = "";
      return "";
   }

   std::string codeRaw = inferPlaceholdersForPlainText(codeRaw2); // Secure plain-text parts.

   int colonPos = 0/*codeRaw.find(":") + 1*/;
   preamble = codeRaw.substr(0, colonPos);
   processedScript = codeRaw.substr(colonPos);

   processedScript = evaluateAll(processedScript, EXPR_BEG_TAG_BEFORE, EXPR_END_TAG_BEFORE);

   findAllVariables();
   setDeclaredFields();

   // SCRIPT TREE ** creation starts here.
   initializeScriptTree();
   std::map<std::string, std::shared_ptr<int>> themap;
   extractInscriptProcessors(themap, debugLevel);                    // Process all inscript preprocessors.
   // EO SCRIPT TREE ** The tree will not be adapted after this point.

   processedScript = remDecl(processedScript);                      // Cut out declarations.
   processedScript = undoPlaceholdersForPlainText(processedScript); // Undo placeholder securing.
   processedScript = StaticHelper::replaceAll(processedScript, NOP_SYMBOL, "");       // Clear all NOP symbols from script.
   ignorePreprocessorsAndAnimateOnce = false;  // Reset to not ignoring any fields.

   processedScript = evaluateAll(processedScript, EXPR_BEG_TAG_AFTER, EXPR_END_TAG_AFTER);

   return finalizeDebugScript(debugLevel > 0);
}

std::string vfm::macro::Script::getProcessedScript() const
{
   return processedScript;
}

std::string Script::findVarName(const std::string script, int& begin) const
{
   int original_begin = begin;
   auto tokens = StaticHelper::tokenize(script, *SingletonFormulaParser::getLightInstance(), begin, 1, true);

   if (!tokens || tokens->empty()) {
      addError("Variable name not found at position '" + std::to_string(original_begin) + "' in script '" + script + "'.");
      return "#INVALID";
   }

   std::string var_name = tokens->at(0);

   begin = original_begin - var_name.length();

   return var_name;
}

bool Script::extractInscriptProcessors(std::map<std::string, std::shared_ptr<int>>& processed, const int debugLevel)
{
   // Check for deep regression.
   auto times = processed.count(processedScript) ? processed.at(processedScript) : nullptr;

   if (times && *times > 30) {
      addError("Deep regression detected for script:\n" + processedScript);
   }

   if (!times) {
      processed.insert({ processedScript, std::make_shared<int>(1) });
   }

   processed[processedScript] = std::make_shared<int>(*processed.at(processedScript) + 1);
   std::string debugScript = processedScript;

   // Find next preprocessor.
   int indexOfPrep = findNextInscriptPos();

   bool debug = debugLevel > 0;
   bool debugWithPrep = debugLevel > 1;
   if (indexOfPrep < 0) {
      handleSingleDebugScript(debug, debugScript, -1, -1, debugWithPrep);
      return false; // No preprocessor tags.
   }

   bool changed = false;
   double scale = 1;
   std::string identifierName = "u-" + std::to_string(count);

   std::string preprocessorScript = *StaticHelper::extractFirstSubstringLevelwise(processedScript, INSCR_BEG_TAG, INSCR_END_TAG, indexOfPrep);
   int lengthOfPreprocessor = preprocessorScript.length() + INSCR_BEG_TAG.length() + INSCR_END_TAG.length();
   std::string partBefore = processedScript.substr(0, indexOfPrep);
   std::string partAfter = processedScript.substr(indexOfPrep + lengthOfPreprocessor);
   int indexOfSeparator = preprocessorScript.find("|");

   int scaleLength = 0;
   if (indexOfSeparator >= 0) {
      std::string scaleStr = preprocessorScript.substr(0, indexOfSeparator);
      if (StaticHelper::isParsableAsFloat(scaleStr)) { // Scale defined.
         scaleLength = scaleStr.length() + 1;
         scale = ::atof(scaleStr.c_str());
         preprocessorScript = StaticHelper::trimAndReturn(preprocessorScript.substr(indexOfSeparator + 1));
      }
   }

   bool hidden = true;
   int declLength = 0;
   int qualifiedDelta = 0;
   if (indexOfPrep > 1 && processedScript.at(indexOfPrep - 1) == ASSIGNMENT_OPERATOR) {
      //int indexOfFNBegin = RegExIndexer.lastIndexOf(partBefore, "\\W", partBefore.length() - 1);
      int indexOfFNBegin = partBefore.length() - 2;

      identifierName = findVarName(partBefore, indexOfFNBegin);
      identifierName = raiseAndGetQualifiedIdentifierName(identifierName);
      declLength = partBefore.length() - indexOfFNBegin - 1;
      partBefore = partBefore.substr(0, indexOfFNBegin + 1);
      hidden = false;

      std::string qualifiedIdentifierName = getQualifiedIdentifierName(preprocessorScript);
      if (qualifiedIdentifierName != preprocessorScript) {
         // The right side of the assignment has to be qualified.
         qualifiedDelta = qualifiedIdentifierName.length() - preprocessorScript.length();
         preprocessorScript = qualifiedIdentifierName;
         scriptTree->shiftEnd(partBefore.length() + declLength, qualifiedDelta);
      }

      if (getPreprocessors().count(identifierName)) {
         /*
          * If the identifier is re-assigned, known chains containing
          * it have to be removed.
          */

         removeChainsContainingIdentifier(identifierName);
      }
   }

   int indexCurr = getNextNonInscriptPosition(partAfter);
   indexCurr = (std::min)(indexCurr, (int) partAfter.length());

   std::string methods = partAfter.substr(0, indexCurr);
   partAfter = partAfter.substr(indexCurr);
   int methodPartBegin = preprocessorScript.length();
   preprocessorScript = preprocessorScript + methods;
   std::string placeholder_for_inscript{};

   // Refresh script tree.
   int wholePrepLength = preprocessorScript.length()
      + INSCR_BEG_TAG.length()
      + INSCR_END_TAG.length()
      + declLength
      + scaleLength;
   if (createScriptTree) {
      scriptTree->addScript(
         identifierName,
         preprocessorScript,
         partBefore.length(),
         partBefore.length() + wholePrepLength - 1);
   }

   // Store debug script.
   int begin = indexOfPrep - declLength;
   handleSingleDebugScript(debug, debugScript, begin, begin + wholePrepLength + starsIgnored - qualifiedDelta, debugWithPrep);

   // Check if the preprocessor is just an identifier name.
   std::string trimmed = StaticHelper::trimAndReturn(preprocessorScript);
   if (isVariable(trimmed)) {
      placeholder_for_inscript = VAR_BEG_TAG + trimmed + VAR_END_TAG; // TODO: Do we need this?
      addPreprocessor(preprocessorScript, identifierName, hidden, methodPartBegin);
   }
   else {
      addPreprocessor(preprocessorScript, identifierName, hidden, methodPartBegin);

      if (knownPreprocessors.count(preprocessorScript)) {
         placeholder_for_inscript = knownPreprocessors.at(preprocessorScript);
      }
      else {
         placeholder_for_inscript = placeholderForInscript(
            identifierName,
            preprocessorScript,
            scale,
            !StaticHelper::isWithinLevelwise(processedScript, indexOfPrep, DECL_BEG_TAG, DECL_END_TAG));

         //                if (dynamicExpansion) {
         knownPreprocessors.insert({ preprocessorScript, placeholder_for_inscript });
         //                }
      }
   }

   //if (!placeholder_for_inscript.empty()) { // TODO: Placeholder cannot be empty - but recheck please... (stupid Java...)
      std::string placeholderFinal = checkForPlainTextTags(placeholder_for_inscript);
      processedScript = partBefore + placeholderFinal + partAfter;
      scriptTree->shiftEnd(
         partBefore.length() + wholePrepLength - 1,
         placeholderFinal.length() - wholePrepLength);
      changed = true;
   //}

   processPendingVars();
   setDeclaredFields();
   count++;

   bool nested_call = extractInscriptProcessors(processed, debugLevel);

   return changed || nested_call;
}

std::string Script::getPreprocessorFromThis(const std::string& identifier) 
{
   return getPreprocessors().at(identifier);
}

void Script::removeChainsContainingIdentifier(const std::string& ident) 
{
   for (const auto& c : getAllQualifiedIdentifiers(ident)) {
      removeChainsContainingQualifiedIdentifier(c);
   }
}

void Script::removeChainsContainingQualifiedIdentifier(const std::string& qualIdent) 
{
   std::string idName = qualIdent;

   if (!(StaticHelper::stringStartsWith(qualIdent, INSCR_BEG_TAG)
      && StaticHelper::stringEndsWith(qualIdent, INSCR_END_TAG))) {
      idName = INSCR_BEG_TAG + idName + INSCR_END_TAG;
   }

   std::set<std::string> toRemove{};

   for (const auto& chain : knownChains) {
      if (StaticHelper::stringContains(chain.first, idName)) {
         toRemove.insert(chain.first);
      }
   }

   for (const auto& el : toRemove) {
      knownChains.erase(el);
   }
}

std::vector<std::string> Script::getAllQualifiedIdentifiers(const std::string& ident) 
{
   std::vector<std::string> idents{};

   if (!StaticHelper::stringStartsWith(ident, PREFIX_FOR_TIMED_IDENTIFIERS)) {
      idents.push_back(ident);
      return idents;
   }

   for (int i = 0; i <= identCounts.at(ident); i++) {
      idents.push_back(ident + QUALIFIED_IDENT_MARKER + std::to_string(i));
   }

   return idents;
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

void Script::processPendingVars() 
{
   int indexOfVar = processedScript.find(VAR_BEG_TAG);

   if (indexOfVar < 0) {
      return;
   }

   std::string varPart = *StaticHelper::extractFirstSubstringLevelwise(processedScript, VAR_BEG_TAG, VAR_END_TAG, indexOfVar);
   int lengthOfVar = varPart.length() + VAR_BEG_TAG.length() + VAR_END_TAG.length();
   std::string partBefore = processedScript.substr(0, indexOfVar);
   std::string partAfter = processedScript.substr(indexOfVar + lengthOfVar);
   std::string varVal = processChain(varPart);

   if (varVal != varPart) {
      std::string placeholder_for_inscript = placeholderForInscript(varPart, varVal, 1, true);
      processedScript =
         partBefore
         + placeholder_for_inscript // TODO: true?? 
         + partAfter;

      int wholePrepLength = varPart.length() + VAR_BEG_TAG.length() + VAR_END_TAG.length();

      scriptTree->shiftEnd(
         partBefore.length() + wholePrepLength - 1,
         placeholder_for_inscript.length() - wholePrepLength);
   }

   processPendingVars();
}

std::string Script::processChain(const std::string& chain) {
   processPreprocessors();

   for (const auto& k1_pair : alltimePreprocessors) {
      std::string k1 = k1_pair.first;
      std::string k = getUnqualifiedName(k1);
      std::string varVal = alltimePreprocessors.at(getQualifiedIdentifierName(k));

      //if (varVal.empty()) return ""; // TODO: Is that right? Capture the case that the result is actually empty.

      int mbeg{};
      if (methodPartBegins.count(varVal)) {
         mbeg = methodPartBegins.at(varVal);
      } else {
         mbeg = varVal.length();
      }

      std::string first = replaceIdentAtBeginningOfChain(chain, k, varVal, mbeg);
      if (!first.empty()) {
         return first;
      }

      if (k1 != k) {
         std::string second = replaceIdentAtBeginningOfChain(chain, k1, varVal, mbeg);
         if (!second.empty()) {
            return second;
         }
      }
   }

   return chain;
}

std::string Script::replaceIdentAtBeginningOfChain(const std::string& chain, const std::string& k, const std::string& varVal, int mbeg)
{
   if (StaticHelper::stringStartsWith(chain, k + ".") || chain == k) {
      std::string newChain = varVal + chain.substr(k.length());
      methodPartBegins.insert({ newChain, mbeg });
      return newChain;
   }
   else if (StaticHelper::stringStartsWith(chain, INSCR_BEG_TAG + k + INSCR_END_TAG)) {
      std::string newChain = varVal + chain.substr(INSCR_BEG_TAG.length() + k.length() + INSCR_END_TAG.length());
      methodPartBegins.insert({ newChain, mbeg });
      return newChain;
   }
   else {
      return "";
   }
}

std::string Script::getUnqualifiedName(const std::string& qualifiedName) 
{
   if (!StaticHelper::stringStartsWith(qualifiedName, PREFIX_FOR_TIMED_IDENTIFIERS)) {
      return qualifiedName;
   }

   return StaticHelper::split(qualifiedName, QUALIFIED_IDENT_MARKER)[0];
}

std::string Script::placeholderForInscript(
   const std::string& filename, // Don't remove this - LaTeX needs it.
   const std::string& preprocessorScript,
   const double scale,
   const bool allowRegularScripts)
{
   if (isVariable(preprocessorScript)) {
      return "";
   }

   RepresentableAsPDF result = evaluateChain(removePreprocessors(rawScript), preprocessorScript, nullptr); // TODO nullptr?

   if (true/*DummyRepresentable.class.isAssignableFrom(result.getClass())*/) { // TODO: Only plain-text scripts allowed for now.
      std::string replace;
      replace = StaticHelper::replaceAll(result->getRawScript(), PREAMBLE_FOR_NON_SCRIPT_METHODS, "");
      return replace;
   }
   else { // Regular script. Expansion allowed ==> null.
      return allowRegularScripts ? "" : preprocessorScript;
   }
}

RepresentableAsPDF Script::evaluateChain(const std::string& repScrThis, const std::string& chain, RepresentableAsPDF father)
{
   std::string processedChain{ processChain(StaticHelper::trimAndReturn(chain)) };
   std::string processedRaw{ processedChain };
   RepresentableAsPDF repThis{};

   if (knownChains.count(processedRaw)) {
      return knownChains.at(processedRaw);
   }

   if (knownReps.count(repScrThis)) {
      repThis = knownReps.at(repScrThis);
   }
   else if (repScrThis.empty()) {
      repThis = nullptr;
   }
   else {
      repThis = createThisObject(nullptr, repScrThis, father);
      knownReps[repScrThis] = repThis;
   }

   RepresentableAsPDF repToProcess{};
   std::shared_ptr<int> methodBegin = methodPartBegins.count(processedRaw) ? std::make_shared<int>(methodPartBegins.at(processedRaw)) : nullptr;

   if (StaticHelper::stringStartsWith(processedChain, THIS_NAME)) {
      if (!repThis) {
         addError("Cannot apply methods to \"this\", no reference representable given.");
      }

      repToProcess = repThis->copy();

      //ignorePreprocessorsAndAnimateOnce();
      repToProcess = createThisObject(repToProcess, repScrThis, father);
      processedChain = processedChain.substr(THIS_NAME.length());
   }
   else if (StaticHelper::stringStartsWith(processedChain, INSCR_BEG_TAG)
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
         repToProcess = repfactory_instanceFromScript(repPart, father);

         if (!repToProcess) { // String is no script, but plain expression.
            repToProcess = std::make_shared<DummyRepresentable>(repThis, vfm_data_, vfm_parser_);
            processedChain = createDummyrep(processedChain, repToProcess, father);
         }
         else {
            processedChain = processedChain.substr(repPart.length() + INSCR_BEG_TAG.length() + INSCR_END_TAG.length());
         }
      }
      else { // Treat as plain text.
         repToProcess = std::make_shared<DummyRepresentable>(repThis, vfm_data_, vfm_parser_);
         repToProcess->createInstanceFromScript(INSCR_BEG_TAG + repPart + INSCR_END_TAG, father);
         processedChain = processedRaw.substr(*methodBegin + 1);
      }
   }
   else {
      if (methodBegin) { // String is script without delimiters, but with method calls.
         std::string script = processedRaw.substr(0, *methodBegin);
         processedChain = processedRaw.substr(*methodBegin + 1);
         repToProcess = repfactory_instanceFromScript(script, father);

         if (!repToProcess) { // String is no script, but plain expression.
            repToProcess = std::make_shared<DummyRepresentable>(repThis, vfm_data_, vfm_parser_);
            repToProcess->createInstanceFromScript(script, father);
            //                    processed = createDummyrep(processed, repToProcess);
         }
         else {
            //                    processed = "";
         }
      }
      else { // Assume whole string is expression without method calls.
         repToProcess = repfactory_instanceFromScript(processedChain, father);

         if (!repToProcess) { // String is no script, but plain expression.
            repToProcess = std::make_shared<DummyRepresentable>(repThis, vfm_data_, vfm_parser_);
            processedChain = createDummyrep(processedChain, repToProcess, father);
         }
         else {
            processedChain = "";
         }
      }
   }

   addFailableChild(repToProcess, "");

   if (StaticHelper::isEmptyExceptWhiteSpaces(processedChain)) {
      knownChains[processedRaw] = repToProcess;
      return repToProcess;
   }

   std::vector<std::string> methodSignatures = getMethodSinaturesFromChain(processedChain);
   std::vector<std::string> methodSignaturesArray{};
   methodSignaturesArray.resize(methodSignatures.size());

   for (int i = 0; i < methodSignatures.size(); i++) {
      methodSignaturesArray[i] = methodSignatures.at(i);
   }

   RepresentableAsPDF apply_method_chain = applyMethodChain(repToProcess, methodSignaturesArray);
   knownChains[processedRaw] = apply_method_chain;
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

      if (newRep->toDummyIfApplicable()) {
         addNote(
            "Plain text method call: "
            + methodSignature
            + " (results in '"
            + StaticHelper::replaceAll(newRep->getRawScript(), PREAMBLE_FOR_NON_SCRIPT_METHODS, "")
            + "')");
      }
      else {
         addError(
            "Regular method call (NOT SUPPORTED!): "
            + methodSignature
            + " (results in "
            + newRep->getRawScript()
            + ")");
      }
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

   std::vector<std::string> actualParameters = getParametersFor(pars, rep, nullptr); // TODO: nullptr??
   std::string newScript = rep->applyMethodString(methodName, actualParameters);

   return repfactory_instanceFromScript(newScript, nullptr); // TODO: nullptr??
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

std::string DummyRepresentable::arclengthCubicBezierFromStreetTopology(
   const std::string& lane_str, const std::string& angle_str, const std::string& distance_str, const std::string& num_lanes_str)
{
   if (!StaticHelper::isParsableAsInt(lane_str)) addError("Lane '" + lane_str + "' is not parsable as int in 'arclengthCubicBezierFromStreetTopology'.");
   if (!StaticHelper::isParsableAsFloat(angle_str)) addError("Angle '" + angle_str + "' is not parsable as float in 'arclengthCubicBezierFromStreetTopology'.");
   if (!StaticHelper::isParsableAsFloat(distance_str)) addError("Distance '" + distance_str + "' is not parsable as float in 'arclengthCubicBezierFromStreetTopology'.");
   if (!StaticHelper::isParsableAsInt(num_lanes_str)) addError("NumLanes '" + num_lanes_str + "' is not parsable as float in 'arclengthCubicBezierFromStreetTopology'.");
   if (hasErrorOccurred()) return "#ERROR-Check-Log";

   const int lane{ std::stoi(lane_str) };
   const int num_lanes{ std::stoi(num_lanes_str) };
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
      + " -- l=" + std::to_string(l) 
      + ", angle=" + std::to_string(angle) 
      + "(rad), v=" + v.serialize()
      + ", vr=" + vr.serialize()
      + ", P3=" + P3.serialize()
      + ", p0=" + p0.serialize()
      + ", p1=" + p1.serialize()
      + ", p2=" + p2.serialize()
      + ", p3=" + p3.serialize()
      + ", d=" + std::to_string(d)
      + ", N=" + std::to_string(num_lanes)
      ;
}

std::string DummyRepresentable::applyMethodString(const std::string& method_name, const std::vector<std::string>& parameters)
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
   else if (method_name == "shortenToMaxSize" && parameters.size() == 2) return StaticHelper::shortenToMaxSize(getRawScript(), stringToFloat(parameters.at(0)), isBooleanTrue(parameters.at(1)));
   else if (method_name == "shortenToMaxSize" && parameters.size() == 3) return StaticHelper::shortenToMaxSize(getRawScript(), stringToFloat(parameters.at(0)), isBooleanTrue(parameters.at(1)), stringToFloat(parameters.at(2)));
   else if (method_name == "shortenToMaxSize" && parameters.size() == 4) return StaticHelper::shortenToMaxSize(getRawScript(), stringToFloat(parameters.at(0)), isBooleanTrue(parameters.at(1)), stringToFloat(parameters.at(2)), stringToFloat(parameters.at(3)));
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
   else if (method_name == "writeTextToFile" && parameters.size() == 2) { StaticHelper::writeTextToFile(getRawScript(), parameters.at(0), isBooleanTrue(parameters.at(1))); return getRawScript(); }
   else if (method_name == "timestamp" && parameters.size() == 0) { return StaticHelper::timeStamp(); }
   
   else if (method_name == "mypath" && parameters.size() == 0) { return getMyPath(); }

   else if (method_name == "include" && parameters.size() == 0) {
      std::string filepath{ StaticHelper::absPath(getMyPath()) + "/" + getRawScript() };

      if (StaticHelper::existsFileSafe(filepath)) {
         return StaticHelper::readFile(filepath);
      }
      else {
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

   else if (inscriptMethodDefinitions.count(method_name) && inscriptMethodParNums.at(method_name) == parameters.size()) return executeCommand(method_name, parameters);

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

std::vector<std::string> Script::getParametersFor(const std::vector<std::string>& rawPars, const RepresentableAsPDF this_rep, const RepresentableAsPDF father)
{
   std::vector<std::string> parameters;
   parameters.resize(rawPars.size());

   for (int i = 0; i < rawPars.size(); i++) {
      RepresentableAsPDF processedPar = evaluateChain(removePreprocessors(this_rep->getRawScript()), rawPars[i], father);

      if (processedPar->toDummyIfApplicable()) {
         parameters[i] = StaticHelper::replaceAll(processedPar->getRawScript(), PREAMBLE_FOR_NON_SCRIPT_METHODS, "");
      }
      else {
         addError("Parameter " + rawPars[i] + " could not be evaluated.");
      }
   }

   bool stdValBool = false;
   int stdValNum = 5;
   int i = 0;

   //try {
   //   for (Parameter par : m.getParameters()) {
   //      Object stdVal = par.getType().equals(Boolean.class)
   //         || par.getType().equals(Boolean.TYPE)
   //         ? stdValBool
   //         : stdValNum;
   //      stdVal = par.getType().equals(String.class) ? "string" : stdVal;
   //      String methodDescription =
   //         "Value for "
   //         + par.getType()
   //         + " parameter '"
   //         + par.getName()
   //         + "' (method "
   //         + m.getName()
   //         + ")";

   //      if (mw.getMethodDescription() != null) {
   //         methodDescription = mw.getMethodDescription();
   //      }

   //      String t;

   //      if (parameters == null) {
   //         t = GeneralDialog.getStringFromUser(
   //            methodDescription,
   //            stdVal + "",
   //            "$$" + m.getName() + "--" + par.getName() + "$$");
   //      }
   //      else {
   //         t = parameters[i];
   //      }

   //      if (t == null) {
   //         return null;
   //      }

   //      if (par.getType().equals(Boolean.class) || par.getType().equals(Boolean.TYPE)) {
   //         t = isStringTrue(t) ? "true" : "false";
   //         list[i] = Boolean.parseBoolean(t);
   //      }
   //      else
   //         if (par.getType().equals(Double.class) || par.getType().equals(Double.TYPE)) {
   //            list[i] = Double.parseDouble(t);
   //         }
   //         else
   //            if (par.getType().equals(Float.class) || par.getType().equals(Float.TYPE)) {
   //               list[i] = Float.parseFloat(t);
   //            }
   //            else
   //               if (par.getType().equals(Integer.class) || par.getType().equals(Integer.TYPE)) {
   //                  list[i] = Integer.parseInt(t);
   //               }
   //               else
   //                  if (par.getType().equals(Long.class) || par.getType().equals(Long.TYPE)) {
   //                     list[i] = Long.parseLong(t);
   //                  }
   //                  else
   //                     if (par.getType().equals(String.class)) {
   //                        list[i] = t;
   //                     }

   //      i++;
   //   }
   //}
   //catch (NumberFormatException e) {
   //   GlobalVariables.getParameters().logWarning(
   //      "Method invokation failed for method '"
   //      + m.getName() + "' due to the following exception: " + e.getMessage());
   //}

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

RepresentableAsPDF DummyRepresentable::createThisObject(const RepresentableAsPDF repThis, const std::string repScrThis, const RepresentableAsPDF father) {
   if (!repThis) {
      auto repThis_copy = std::make_shared<DummyRepresentable>(nullptr, getData(), getParser());
      addFailableChild(repThis_copy, "");
      repThis_copy->createInstanceFromScript(repScrThis, father);
      return repThis_copy;
   }

   return repThis;
}

bool Script::isVariable(const std::string& preprocessorScript)
{
   //if (!StaticHelper::isAlphaNumeric(preprocessorScript)) {
   //   return false;
   //}

   return VARIABLES_MAYBE.count(preprocessorScript);
}

std::map<std::string, std::string> Script::getPreprocessors() 
{
   processPreprocessors();
   return alltimePreprocessors;
}

void Script::processPreprocessors()
{
   allOfIt = 0;

   while (recalculatePreprocessors && !alltimePreprocessors.empty()) { // Do it as often as necessary.
      resetTime();
      recalculatePreprocessors = false;

      for (const auto& s : alltimePreprocessors) {
         processPreprocessor(s.first);
      }

      std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

      long thatsIt = measureElapsedTime("processing " + std::to_string(alltimePreprocessors.size()) + " preprocessors");
      allOfIt += thatsIt;
   }
}

void Script::processPreprocessor(const std::string& rawName)
{
   std::string name = rawName;
   auto& mapFromAlltimes = alltimePreprocessors;
   std::string code = mapFromAlltimes.at(name);
   int nextPoint1 = StaticHelper::indexOfOnTopLevel(code, { "." }, 0, START_TAG_FOR_NESTED_VARIABLES, END_TAG_FOR_NESTED_VARIABLES);
   int nextPoint2 = StaticHelper::indexOfOnTopLevel(code, { "." }, 0, INSCR_BEG_TAG, INSCR_END_TAG);

   if (nextPoint1 < 0 || nextPoint1 != nextPoint2) {
      nextPoint1 = code.length();
   }

   std::string objectName = StaticHelper::replaceAll(StaticHelper::replaceAll(code.substr(0, nextPoint1), INSCR_BEG_TAG, ""), INSCR_END_TAG, "");
   std::string rest = code.substr(nextPoint1);

   for (const auto& var_pair : alltimePreprocessors) {
      std::string var = var_pair.first;

      if (var == objectName) {
         std::string original = var_pair.second;
         std::string newChain = original + rest;
         addNote("Preprocessors -- changing from '" + mapFromAlltimes.at(name) + "' to '" + newChain + "'.");
         mapFromAlltimes[name] = newChain;

         if (methodPartBegins.count(original)) {
            int mBeg = methodPartBegins.at(original);
            methodPartBegins[newChain] = mBeg;
         }

         recalculatePreprocessors = true;
         return;
      }
   }
}

std::string Script::raiseAndGetQualifiedIdentifierName(const std::string& identifierName) 
{
   if (!StaticHelper::stringStartsWith(identifierName, PREFIX_FOR_TIMED_IDENTIFIERS)) {
      return identifierName;
   }

   if (!identCounts.count(identifierName)) {
      identCounts.insert({ identifierName, 0 });
   }
   else {
      identCounts.insert({ identifierName, identCounts.at(identifierName) + 1 });
   }

   return getQualifiedIdentifierName(identifierName);
}

std::string Script::getQualifiedIdentifierName(const std::string& identifierName) 
{
   if (!StaticHelper::stringStartsWith(identifierName, PREFIX_FOR_TIMED_IDENTIFIERS)) {
      return identifierName;
   }

   if (StaticHelper::stringStartsWith(identifierName, "u-")) {
      return identifierName;
   }

   if (!identCounts.count(identifierName)) {
      return identifierName;
   }

   return identifierName + QUALIFIED_IDENT_MARKER + std::to_string(identCounts.at(identifierName));
}

void Script::handleSingleDebugScript(const bool debug, const std::string& currScr, const int begSel, const int endSel, const bool includePreprocessors) 
{
   std::string currentScript = currScr;

   if (begSel >= 0) {
      std::string before = currScr.substr(0, begSel);
      std::string within = currScr.substr(begSel, endSel - begSel);
      std::string after = currScr.substr(endSel);
      currentScript = before + ">>>" + within + "<<<" + after;
   }

   if (debug) {
      std::string overallOutput = currentScript
         + (includePreprocessors ? "\n" + getPreprocessorStringForDeclarations(true, false) : "");
      debugScript += "\n\\noindent{\\huge\\HandCuffRight}\\begin{verbatim}(" + std::to_string(debugScriptCounter) + ") "
         + PLAIN_TEXT_BEGIN_TAG
         + overallOutput
         + PLAIN_TEXT_END_TAG
         + " \\end{verbatim}"
         + "\\thispagestyle{empty}\\pagebreak\n";
      debugAnimateInstruction += "page" + std::to_string(debugScriptCounter) + "->";
      debugScriptCounter++;
      adjustMaxLinesAndLineLengths("dum\nmy\n" + overallOutput);
   }
}

void Script::adjustMaxLinesAndLineLengths(const std::string& str)
{
   std::vector<std::string> lines = StaticHelper::split(str, "\n");
   int maxLocalWidth = 0;

   for (const auto& line : lines) {
      if (line.length() > maxLocalWidth) {
         maxLocalWidth = line.length();
      }
   }

   maxHeight = (std::max)(maxHeight, (int) lines.size());
   maxWidth = (std::max)(maxWidth, maxLocalWidth);
}

std::string Script::getPreprocessorStringForDeclarations(const bool includeHidden, const bool includeAll)
{
   std::map<std::string, std::string> thePreprocessors;
   if (includeAll) {
      //alltimePreprocessors.keySet().forEach(p->thePreprocessors.putAll(alltimePreprocessors.at(p)));
      addFatalError("Including 'all' preprocessors is not (yet?) supported.");
   }
   else {
      thePreprocessors = alltimePreprocessors;
   }

   if (thePreprocessors.empty()) {
      return "";
   }

   std::string s = "";
   int num = 1;

   for (const auto& datnam_pair : thePreprocessors) {
      std::string datnam = datnam_pair.first;

      if (includeHidden || includeAll || !isPreprocessorHidden(datnam)) {
         std::string thePreprocessor = thePreprocessors.at(datnam);
         std::string oneEntry = "\n" + PREPROCESSOR_FIELD_NAME + std::to_string(num)
            + VARIABLE_DELIMITER
            + START_TAG_FOR_NESTED_VARIABLES
            + datnam + VARIABLE_DELIMITER + StaticHelper::replaceAll(StaticHelper::replaceAll(thePreprocessor, "\n", "\\n"), "\n", "\\n")
            + END_TAG_FOR_NESTED_VARIABLES
            + "" + BEGIN_COMMENT
            + std::to_string(methodPartBegins.at(thePreprocessor))
            + END_COMMENT
            + ";";

         s += oneEntry;
         num++;
      }
   }

   return s;
}

bool Script::isPreprocessorHidden(const std::string& name)
{
   return HIDDEN_PREPROCESSORS.count(name);
}

int Script::findNextInscriptPos()
{
   std::string s{};
   while (StaticHelper::stringContains(processedScript, INSCR_END_TAG + s + INSCR_PRIORITY_SYMB)) {
      s += INSCR_PRIORITY_SYMB;
   }

   starsIgnored = s.length();

   int pos = processedScript.find(INSCR_END_TAG + s);
   int count = 0;               // Because we start on an end tag.

   for (int i = pos; i >= 0; i--) {
      if (StaticHelper::stringStartsWith(processedScript, INSCR_BEG_TAG, i)) {
         count++;
      }

      if (StaticHelper::stringStartsWith(processedScript, INSCR_END_TAG, i)) {
         count--;
      }

      if (count == 0) {
         int starPos = pos + INSCR_END_TAG.length();
         processedScript = processedScript.substr(0, starPos)
            + processedScript.substr(starPos + s.length());
         scriptTree->shiftEnd(pos - 1, -s.length());

         return i;
      }
   }

   return -1;
}

void Script::initializeScriptTree() 
{
   if (processedScript.empty()) {
      processedScript = rawScript.substr(rawScript.find(":") + 1);
   }

   scriptTree = std::make_shared<ScriptTree>();                                      // Create and...
   scriptTree->addScript("", processedScript, 0, processedScript.length() - 1); // ...initialize script tree.
}

std::string Script::remDecl(const std::string& codeRaw)
{
   std::string code = codeRaw;

   code = inferPlaceholdersForPlainText(code);

   std::string codeWithoutDec = removeTaggedPartsOnTopLevel(
      code,
      DECL_BEG_TAG,
      DECL_END_TAG,
      IGNORE_BEG_TAGS_IN_DECL,
      IGNORE_END_TAGS_IN_DECL);

   return undoPlaceholdersForPlainText(codeWithoutDec);
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

void Script::setDeclaredFields() 
{
   std::string withoutPreprocessors = removePreprocessors(processedScript);

   auto fieldSetter = extractNVPairs(
      withoutPreprocessors,
      BEGIN_LITERAL,
      END_LITERAL,
      END_VALUE,
      DECL_BEG_TAG,
      DECL_END_TAG);

   for (const auto& nameValue : fieldSetter) {
      std::string name = nameValue.first;
      std::string value = nameValue.second;
      setViaFakeReflections(name, value);
   }
}

void Script::setViaFakeReflections(const std::string& fieldName, const std::string& value) 
{
   std::string val = value;

   // "e" and "exerciseString" are reserved fields that cannot be used in child classes.
   if (fieldName == EXERCISE_FIELD_LONG_NAME
      || fieldName == EXERCISE_FIELD_SHORT_NAME) {
      //setExercise(new Exercise(value)); TODO?
      //exerciseString = this.getExercise().getRawExerciseString();
      return;
   }

   // Preprocessors.
   if (!ignorePreprocessorsAndAnimateOnce) {
      if (StaticHelper::stringStartsWith(fieldName, PREPROCESSOR_FIELD_NAME)) {
         if (!StaticHelper::stringStartsWith(val, START_TAG_FOR_NESTED_VARIABLES)
            || !StaticHelper::stringEndsWith(val, END_TAG_FOR_NESTED_VARIABLES)) {
            val = START_TAG_FOR_NESTED_VARIABLES + val + END_TAG_FOR_NESTED_VARIABLES;
         }

         std::string preprocessor = *StaticHelper::extractFirstSubstringLevelwise(
            val,
            START_TAG_FOR_NESTED_VARIABLES,
            END_TAG_FOR_NESTED_VARIABLES,
            0);

         addPreprocessor(preprocessor, false);
      }

      if (fieldName == ANIMATE_FIELD_NAME) {
         animate = value;
      }
   }

   // Fake set via reflections:
   if (fieldName == "field1") field1 = value;
   if (fieldName == "field2") field2 = value;
   if (fieldName == "field3") field3 = value;
}

void Script::addPreprocessor(const std::string& preprocessorScript, const bool hidden) 
{
   int indexOf = preprocessorScript.find(VARIABLE_DELIMITER);
   std::string datnam = StaticHelper::removeWhiteSpace(preprocessorScript.substr(0, indexOf));
   std::string plainPreprocessor = StaticHelper::trimAndReturn(preprocessorScript.substr(indexOf + 1));
   addPreprocessor(plainPreprocessor, datnam, hidden, StaticHelper::indexOfOnTopLevel(plainPreprocessor, { METHOD_CHAIN_SEPARATOR }, 0, INSCR_BEG_TAG, INSCR_END_TAG));
}

void Script::addPreprocessor(const std::string& preprocessor, const std::string& filename, const bool hidden, const int indexOfMethodsPartBegin)
{
   alltimePreprocessors.insert({ filename, StaticHelper::trimAndReturn(preprocessor) });

   if (indexOfMethodsPartBegin != preprocessor.length() && indexOfMethodsPartBegin >= 0) {
      int leftTrim = preprocessor.size() - StaticHelper::ltrimAndReturn(preprocessor).size();

      methodPartBegins.insert({ StaticHelper::trimAndReturn(preprocessor), indexOfMethodsPartBegin - leftTrim });
   }

   recalculatePreprocessors = true;

   if (hidden) {
      HIDDEN_PREPROCESSORS.insert(filename);
   }
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
         /*
          * Causes trouble if a preprocessors starts with "*doubleVal*|".
          * But how likely is that... Or... Let's just forbid it :-)
          */
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

std::vector<std::pair<std::string, std::string>> Script::extractNVPairs(
   const std::string& codeRaw,
   char beginLiteral,
   char endLiteral,
   char endValue,
   const std::string& beginTag,
   const std::string& endTag)
{
   std::string name;
   std::string value;
   const int nameMode = 0;
   const int valueRegularMode = 1;
   const int valueLiteralMode = 2;

   std::vector<std::pair<std::string, std::string>> variables;

   std::string declarations = getDeclarations(
      codeRaw,
      beginTag,
      endTag);

   int mode = nameMode;

   std::vector<std::string> nameValue;
   for (int i = 0; i < declarations.length(); i++) {

      if (mode == nameMode) {
         if (declarations.at(i) == ASSIGNMENT_OPERATOR) {
            nameValue.push_back(StaticHelper::removeWhiteSpace(name));

            if (declarations.find(START_TAG_FOR_NESTED_VARIABLES, i + 1) == i + 1) {
               std::string varValue = *StaticHelper::extractFirstSubstringLevelwise(
                  declarations,
                  START_TAG_FOR_NESTED_VARIABLES,
                  END_TAG_FOR_NESTED_VARIABLES,
                  i + 1);

               nameValue.push_back(varValue);
               std::pair<std::string, std::string> name_val_copy = { nameValue.at(0), nameValue.at(1) };
               name = "";
               value = "";
               nameValue.clear();
               variables.push_back(name_val_copy);
               mode = nameMode;

               i += START_TAG_FOR_NESTED_VARIABLES.length()
                  + END_TAG_FOR_NESTED_VARIABLES.length()
                  + varValue.length()
                  + 1; // This is for the semicolon (or whatever) at the end.
            }
            else {
               mode = valueRegularMode;
            }
         }
         else {
            name += declarations.at(i);
         }
      }
      else if (mode == valueRegularMode) {
         if (declarations.at(i) == beginLiteral) {
            mode = valueLiteralMode;
         }
         else if (declarations.at(i) == endValue) {
            nameValue.push_back(value);
            std::pair<std::string, std::string> name_val_copy = { nameValue.at(0), nameValue.at(1) };
            name = "";
            value = "";
            nameValue.clear();
            variables.push_back(name_val_copy);
            mode = nameMode;
         }
         else if (!StaticHelper::isEmptyExceptWhiteSpaces(StaticHelper::makeString(declarations.at(i)))) {
            value += StaticHelper::makeString(declarations.at(i));
         }
      }
      else if (mode == valueLiteralMode) {
         if (declarations.at(i) == endLiteral) {
            mode = valueRegularMode;
         }
         else {
            value += StaticHelper::makeString(declarations.at(i));
         }
      }
   }

   if (!StaticHelper::trimAndReturn(name).empty() || !StaticHelper::trimAndReturn(value).empty()) {
      if (nameValue.empty()) {
         nameValue.push_back(name);
      }

      nameValue.push_back(value);
      std::pair<std::string, std::string> name_val_copy = { nameValue.at(0), nameValue.at(1) };
      name = "";
      value = "";
      nameValue.clear();
      variables.push_back(name_val_copy);
   }

   return variables;
}

std::string Script::getDeclarations(
   std::string script,
   std::string beginTag,
   std::string endTag) 
{
   if (!StaticHelper::stringContains(script, beginTag) || !StaticHelper::stringContains(script, endTag)) {
      return "";
   }

   if (beginTag == endTag) {
      addError("It doesn't make sense to use level-wise matching when begin and end tag are equal.");
   }

   std::vector<std::string> declList = StaticHelper::extractSubstringsLevelwise(script, beginTag, endTag, 0, IGNORE_BEG_TAGS_IN_DECL, IGNORE_END_TAGS_IN_DECL);

   std::string declarations = "";

   for (std::string s : declList) {
      declarations += s;
   }

   return declarations;
}

std::string Script::finalizeDebugScript(const bool debug) const
{
   if (debug) {
      const int width = std::round((float) maxWidth * 5.55) + 10;
      const int height = std::round((float) maxHeight * 17.8) + 10;
      const std::string margin = "10";

      return std::string("latex:%artlet|gra|bbding|\\usepackage[paperwidth=") 
         + std::to_string(width) + "pt,paperheight=" + std::to_string(height) + "pt,hmargin={" + margin + "mm," + margin + "mm},vmargin={" + margin + "mm," + margin + "mm}]{geometry}%\n"
         + debugScript
         + DECL_BEG_TAG + "\n"
         + ANIMATE_FIELD_NAME + VARIABLE_DELIMITER + debugAnimateInstruction + ";"
         + "\n" + DECL_END_TAG;
   }
   else {
      return "";
   }
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

void Script::resetDebugVars() 
{
   debugScript = "";
   debugAnimateInstruction = "";
   debugScriptCounter = 1;
   maxHeight = 0;
   maxWidth = 0;
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

void Script::findAllVariables()
{
   int indexOf = nextIndex(processedScript, 0);

   while (indexOf > 0) {
      int index_of_dummy = indexOf - 1;
      
      std::string var_name = findVarName(processedScript, index_of_dummy);

      if (!var_name.empty() && StaticHelper::isAlphaNumericOrUnderscore(var_name) && StaticHelper::isAlpha(StaticHelper::makeString(var_name.at(0)))) {
         VARIABLES_MAYBE.insert(var_name);
         addNote("Found possible variable named '" + var_name + "' at position " + std::to_string(index_of_dummy) + " in script of length " + std::to_string(processedScript.size()) + ".");
      }

      indexOf = nextIndex(processedScript, indexOf + 1);
   }
}

std::string vfm::macro::Script::getRawScript() const
{
   return rawScript;
}

void vfm::macro::Script::setRawScript(const std::string& script)
{
   rawScript = script;
}

RepresentableAsPDF vfm::macro::Script::copy() const
{
   auto non_const_this = const_cast<Script*>(this);
   auto copy_script = std::make_shared<DummyRepresentable>(non_const_this->toDummyIfApplicable()->getEmbeddedRep(), vfm_data_, vfm_parser_);

   addFailableChild(copy_script, "");

   copy_script->SPECIAL_SYMBOLS = SPECIAL_SYMBOLS;
   copy_script->PLACEHOLDER_MAPPING = PLACEHOLDER_MAPPING;
   copy_script->PLACEHOLDER_INVERSE_MAPPING = PLACEHOLDER_INVERSE_MAPPING;
   copy_script->IGNORE_BEG_TAGS_IN_DECL = IGNORE_BEG_TAGS_IN_DECL;
   copy_script->IGNORE_END_TAGS_IN_DECL = IGNORE_END_TAGS_IN_DECL;
   copy_script->HIDDEN_PREPROCESSORS = HIDDEN_PREPROCESSORS;
   copy_script->alltimePreprocessors = alltimePreprocessors;
   copy_script->identCounts = identCounts;
   copy_script->rawScript = rawScript;
   copy_script->processedScript = processedScript;
   copy_script->preamble = preamble;
   copy_script->debugScript = debugScript;
   copy_script->debugAnimateInstruction = debugAnimateInstruction;
   copy_script->debugScriptCounter = debugScriptCounter;
   copy_script->starsIgnored = starsIgnored;
   copy_script->maxHeight = maxHeight;
   copy_script->maxWidth = maxWidth;
   copy_script->VARIABLES_MAYBE = VARIABLES_MAYBE;
   copy_script->field1 = field1;
   copy_script->field2 = field2;
   copy_script->field3 = field3;
   copy_script->animate = animate;
   copy_script->methodPartBegins = methodPartBegins;
   copy_script->scriptTree = scriptTree; // TODO: Currently only ptr copied.
   copy_script->recalculatePreprocessors = recalculatePreprocessors;
   copy_script->allOfIt = allOfIt;
   copy_script->count = count;
   copy_script->createScriptTree = createScriptTree;

   return copy_script;
}

RepresentableAsPDF vfm::macro::DummyRepresentable::copy() const
{
   auto copy_script = this->Script::copy()->toDummyIfApplicable();
   if (embeddedRep_) copy_script->embeddedRep_ = embeddedRep_->copy();
   return copy_script;
}

std::shared_ptr<DummyRepresentable> vfm::macro::Script::toDummyIfApplicable()
{
   return nullptr;
}

std::shared_ptr<DummyRepresentable> vfm::macro::DummyRepresentable::toDummyIfApplicable()
{
   return std::dynamic_pointer_cast<DummyRepresentable>(this->shared_from_this());
}

RepresentableAsPDF vfm::macro::Script::repfactory_instanceFromScript(const std::string& script, const RepresentableAsPDF father)
{
   // TODO? Currently only accepts DummyRep, i.e., plain-text scripts.
   //if (StaticHelper::stringStartsWith(script, PREAMBLE_FOR_NON_SCRIPT_METHODS)) {
      std::shared_ptr<DummyRepresentable> dummy = std::make_shared<DummyRepresentable>(nullptr, vfm_data_, vfm_parser_);
      addFailableChild(dummy, "");
      dummy->createInstanceFromScript(script, father);
      return dummy;
   //}

   ////RepresentableAsPDF applicablePDFType = ScriptConversionMethods.getApplicablePDFType(
   ////   ScriptConversionMethods.removeComments(script),
   ////   getAvailableTypes(),
   ////   father);

   //if (applicablePDFType == null) {
   //   return null;
   //   //            throw new RuntimeException("No applicable 'Representable' type found for script: " + script);
   //}
   //else {
   //   applicablePDFType.createInstanceFromScript(script, father);
   //}

   //return applicablePDFType;
}

std::string vfm::macro::Script::createDummyrep(const std::string& processed_raw, RepresentableAsPDF repToProcess, RepresentableAsPDF father)
{
   std::string processed;
   repToProcess->createInstanceFromScript(processed, father);
   std::string extract_expression_part = extractExpressionPart(processed);
   repToProcess->createInstanceFromScript(extract_expression_part, father);

   //        processed = processed.replace(extractExpressionPart, "");
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

   addDebug("Expression '" + expression + "' resulted in " + std::to_string(result) + (decimals >= 0 ? " (will be rounded to " + std::to_string(decimals) + " decimals)" : "") + ".");

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

std::string vfm::macro::Script::processScript(
   const std::string& text,
   const std::shared_ptr<DataPack> data,
   const std::shared_ptr<FormulaParser> parser,
   const std::shared_ptr<Failable> father_failable)
{
   knownPreprocessors.clear();
   list_data_.clear();
   knownChains.clear();
   knownReps.clear();
   ignorePreprocessorsAndAnimateOnce = false;
   DummyRepresentable::inscriptMethodDefinitions.clear();
   DummyRepresentable::inscriptMethodParNums.clear();
   DummyRepresentable::inscriptMethodParPatterns.clear();
   VARIABLES_MAYBE.clear();

   auto s = std::make_shared<DummyRepresentable>(nullptr, data ? data : std::make_shared<DataPack>(), parser ? parser : SingletonFormulaParser::getInstance());
   
   if (father_failable) {
      father_failable->addFailableChild(s);
   }

   s->addNote("Processing script. Variable '" + MY_PATH_VARNAME + "' is set to '" + s->getMyPath() + "'.");

   s->applyDeclarationsAndPreprocessors(text, 0);
   return s->getProcessedScript();
}
