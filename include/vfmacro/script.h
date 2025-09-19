//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file
#pragma once
#include "failable.h"
#include "static_helper.h"
#include "parser.h"
#include "data_pack.h"
#include <vector>
#include <map>
#include <set>
#include <string>
#include <memory>
#include <utility>
#include <limits>

namespace vfm {
namespace macro {

static constexpr int MAXIMUM_STRING_SIZE_TO_CACHE{ 50 }; // Too large strings hit rarely but take up lots of space. (TODO: is 50 a good guess?)

static const std::string MY_PATH_VARNAME{ "MY_PATH" };

static const std::string PREFIX_FOR_TIMED_IDENTIFIERS = "TT";
static const std::string QUALIFIED_IDENT_MARKER = "_";

// Protected symbols.
static const char ASSIGNMENT_OPERATOR = '=';
static const std::string PLAIN_TEXT_BEGIN_TAG = "@\"{";
static const std::string PLAIN_TEXT_END_TAG = "}\"@";

static const std::string METHOD_PARS_BEGIN_TAG = "[";
static const std::string METHOD_PARS_END_TAG = "]";
static const std::string METHOD_CHAIN_SEPARATOR = ".";

static const std::string CONVERSION_PREFIX = "**>";
static const std::string CONVERSION_POSTFIX = "<**";

static const std::string BEGIN_COMMENT = "/*";
static const std::string END_COMMENT = "*/";

static const std::string START_TAG_FOR_NESTED_VARIABLES = "[~(~{";
static const std::string END_TAG_FOR_NESTED_VARIABLES = "}~)~]";

static const std::string INSCR_BEG_TAG = "@{";
static const std::string INSCR_END_TAG = "}@";
static const char INSCR_PRIORITY_SYMB = '*';

static const std::string FIRST_LETTER_END = INSCR_END_TAG.substr(0, 1);
static const std::string LAST_LETTER_BEG = INSCR_BEG_TAG.substr(INSCR_BEG_TAG.length() - 1);

static const std::string VAR_BEG_TAG = "V$${";
static const std::string VAR_END_TAG = "}$$V";

static const std::string INSCR_BEG_TAG_FOR_INTERNAL_USAGE = INSCR_BEG_TAG + "$$$";
static const std::string INSCR_END_TAG_FOR_INTERNAL_USAGE = "$$$" + INSCR_END_TAG;

static const char BEGIN_LITERAL = '#';
static const char END_LITERAL = '#';

static const std::string ROUNDING_DELIMITER = "\\";

static const std::string EXPR_BEG_TAG_AFTER = "~{";
static const std::string EXPR_END_TAG_AFTER = "}~";
static const std::string EXPR_BEG_TAG_BEFORE = "~{{";
static const std::string EXPR_END_TAG_BEFORE = "}}~";

static const std::string PREPROCESSOR_FIELD_NAME = "prep";
static const std::string VARIABLE_DELIMITER = "=";
static const char END_VALUE = ';';
static const std::string INSCRIPT_STANDARD_PARAMETER_PATTERN = "#n#";

// Methods which
// * either can have different evaluations for the same body and parameters (e.g., eval, which depends on the data pack),
// * or which have side effects which need to be performed every time (e.g., KILLPIDs).
static const std::set<std::string> UNCACHABLE_METHODS{
   "include", "eval", "PIDs", "KILLPIDs", "scriptVar", "setScriptVar", "executeCommand",
   "vfmheap", "vfmdata", "vfmfunc", "sethard", "printHeap" };

 /// Only for internal usage, this symbol is removed from the script
 /// after the translation process is terminated.
const std::string NOP_SYMBOL = ":$N~O~P$:";
// EO Protected symbols.

const std::string STEPWISE_EXPANSION_ACTUAL_METHOD_NAME = "stepWiseScriptTranslation";
const std::string CREATE_EXERCISE_FROM_THIS_SCRIPT_METHOD_NAME = "Create exercise from this script";
const std::string URL_TO_THIS_SCRIPT_METHOD_NAME = "URL to this script";
const std::string PLAIN_GENERATOR_CODE_METHOD_NAME = "Plain Generator code";
const std::string DECOLLAPSE_RULES_RIGHT_METHOD_NAME = "Decollapse rules right";
const std::string DECOLLAPSE_RULES_LEFT_METHOD_NAME = "Decollapse rules left";
const std::string FORMAT_SCRIPT_METHOD_NAME = "Format script";
const std::string ADD_DECLARATIONS_TO_SCRIPT_METHOD_NAME = "Add declarations to script";
const std::string STEPWISE_EXPANSION_METHOD_NAME = "Stepwise script expansion";

 /// Not quite sure yet if comments should be kept in areas marked as plain
 /// text. First intuition: Comments are above all, so should be removed.
static constexpr bool KEEP_COMMENTS_IN_PLAIN_TEXT = false;

class Script;
class DummyRepresentable;

using RepresentableAsPDF = std::shared_ptr<Script>;

struct MethodPartBegin {
   int method_part_begin_{};
   std::string result_{};
   bool cachable_{};
};

struct ScriptMethodDescription {
   std::string method_name_{};
   int par_num_{};
   std::function<std::string(const std::vector<std::string>&)> function_{};

   bool operator< (const ScriptMethodDescription& o) const {
      return (method_name_ == o.method_name_) ? (par_num_ < o.par_num_) : (method_name_ < o.method_name_);
   }
};

class Script : public Failable, public std::enable_shared_from_this<Script> {
public:
   Script() = delete;
   Script(const std::shared_ptr<DataPack> data, const std::shared_ptr<FormulaParser> parser); // Path mypath relative entry point for file system operations.

   /// Central script processing method, performs the following actions:
   /// <UL>
   /// <LI>Cuts off the script preamble. Note that script preambles are defined as
   /// script.substring(0, script.indexOf(":")). Therefore, they may not "contain" a colon!</LI>
   /// <LI>Sets the declared fields to preliminary values, i.e., only those that are
   /// not subject to in-script preprocessors (including regularly defined preprocessors, though).</LI>
   /// <LI>In a loop, as long as the script changes:</LI>
   /// <UL>
   /// <LI>Processes in-script preprocessors for plain text methods only.</LI>
   /// <LI>Sets all now available declared fields.</LI>
   /// <LI>Processes ALL in-script preprocessors in the non-declarations part of the script.</LI>
   /// </UL>
   /// <LI>Sets the main script variable to the resulting script.</LI>
   /// </UL>
   /// After the call of this method the following is valid:
   /// <UL>
   /// <LI>The main script variable <code>this.scriptWithoutPreprocessors</code> contains
   /// <B>only plain script code from the middle part of the script</B>,
   /// without declarations or preprocessors. Particularly, the variable is non-null, no matter what.</LI>
   /// <LI>All variables are set to the defined values, including in-script and regular preprocessors.</LI>
   /// </UL>
   ///
   /// @param codeRaw2  The code to parse (without a real parser though). Comments must have been removed already.
   /// @return  Debug code if requested, <code>null</code> otherwise.
   void applyDeclarationsAndPreprocessors(const std::string& codeRaw2);

   std::shared_ptr<Script> copy() const;

   std::string getProcessedScript() const;
   std::string getRawScript() const;
   void setRawScript(const std::string& script);

   /// Creates a rep instance from the given script by using all theoretically
   /// available PDF types. I.e., even types not available in the actual current
   /// run can be used here. If the script starts with the non-script preamble,
   /// indicating the application of a conversion method with plain-text
   /// output, a dummy representa
   ///
   /// @param script  The script to create the rep instance from.
   /// @param father  The super representable of this script or {@code null}.
   ///
   /// @return  The rep instance.
   std::shared_ptr<Script> repfactory_instanceFromScript(const std::string& script);

   enum class DataPreparation {
      copy_data_pack_before_run, // Creates a copy of the data pack and works on that. I.e., you won't see the changes in the original data pack if something is altered by the script. Typically needed to make thread-safe.
      reset_script_data_before_run, // Typically desired, unless you want to re-use dynamic methods, cache etc. over multiple sessions.
      both,
      none
   };

   static std::string processScript(
      const std::string& text,
      const DataPreparation data_prep,
      const std::shared_ptr<DataPack> data_raw = nullptr, 
      const std::shared_ptr<FormulaParser> parser = nullptr,
      const std::shared_ptr<Failable> father_failable = nullptr);

   std::string simplifyExpression(const std::string& expression);

   enum class SyntaxFormat {
      vfm,
      nuXmv,
      kratos2
   };

   std::string formatExpression(const std::string& expression, const SyntaxFormat format);
   std::string evaluateExpression(const std::string& expression);
   std::string toK2(const std::string& expression);
   std::string evaluateExpression(const std::string& expression, const std::string& decimals);
   std::string nil() { return ""; }
   std::string id() { return getRawScript(); }
   std::string idd() { std::string s = getRawScript(); return StaticHelper::stringStartsWith(s, PLAIN_TEXT_BEGIN_TAG) ? s : PLAIN_TEXT_BEGIN_TAG + s + PLAIN_TEXT_END_TAG; }

   std::shared_ptr<DataPack> getData() const;
   std::shared_ptr<FormulaParser> getParser() const;
   std::string getMyPath() const;

   const std::string BEGIN_TAG_IN_SEQUENCE = "@(";
   const std::string END_TAG_IN_SEQUENCE = ")@";

   /// Looks for a sequence of several chunks of
   /// <code>@(...)@@(...)@@(...)@</code> in the code and puts them into
   /// <code>scriptSequence</code>.
   ///
   /// @param code  The raw code to process.
   void processSequence(const std::string& code, std::vector<std::string>& script_sequence);
   void createInstanceFromScript(const std::string& code);
   std::shared_ptr<Script> getRepresentableAsPDF();
   std::string evalItAll(const std::string& n1Str, const std::string& n2Str, const std::function<float(float n1, float n2)> eval);

   /// Retrieves the raw script, but without any <code>@{</code> or <code>@{</code>.
   /// Can be used to get the constant <code>2</code> from raw script
   /// <code>@{2}@</code>. But use carefully, since it actually removes ALL
   /// tags.
   ///
   /// @return  The raw script without any inscript tags.
   std::string getTagFreeRawScript();

   std::string sethard(const std::string& value) { method_part_begins_[getTagFreeRawScript()].result_ = value; return value; }
   std::string exsmeq(const std::string& n1Str, const std::string& n2Str) { return evalItAll(n1Str, n2Str, [](float a, float b) { return a <= b; }); }
   std::string exsm(const std::string& n1Str, const std::string& n2Str) { return evalItAll(n1Str, n2Str, [](float a, float b) { return a < b; }); }
   std::string exgreq(const std::string& n1Str, const std::string& n2Str) { return evalItAll(n1Str, n2Str, [](float a, float b) { return a >= b; }); }
   std::string exgr(const std::string& n1Str, const std::string& n2Str) { return evalItAll(n1Str, n2Str, [](float a, float b) { return a > b; }); }
   std::string exeq(const std::string& n1Str, const std::string& n2Str) { return evalItAll(n1Str, n2Str, [](float a, float b) { return a == b; }); }
   std::string exneq(const std::string& n1Str, const std::string& n2Str) { return evalItAll(n1Str, n2Str, [](float a, float b) { return a != b; }); }
   std::string exadd(const std::string& num) { return evalItAll(getTagFreeRawScript(), num, [](float a, float b) { return a + b; }); }
   std::string exsub(const std::string& num) { return evalItAll(getTagFreeRawScript(), num, [](float a, float b) { return a - b; }); }
   std::string exmult(const std::string& num) { return evalItAll(getTagFreeRawScript(), num, [](float a, float b) { return a * b; }); }
   std::string exdiv(const std::string& num) { return evalItAll(getTagFreeRawScript(), num, [](float a, float b) { return a / b; }); }
   std::string expow(const std::string& num) { return evalItAll(getTagFreeRawScript(), num, [](float a, float b) { return std::pow(a, b); }); }
   std::string exmod(const std::string& num) { return evalItAll(getTagFreeRawScript(), num, [](float a, float b) { return (int) a % (int) b; }); }
   std::string exnot() { return std::to_string(!StaticHelper::isBooleanTrue(getTagFreeRawScript()));}
   std::string space() { return " "; }

   std::string fromBooltoString(const bool b);

   std::string arclengthCubicBezierFromStreetTopology(const std::string& lane, const std::string& angle, const std::string& distance, const std::string& num_lanes);
   std::string forloop(const std::string& varname, const std::string& loop_vec);
   std::string forloop(const std::string& varname, const std::string& from_raw, const std::string& to_raw);
   std::string forloop(const std::string& varname, const std::string& from_raw, const std::string& to_raw, const std::string& step_raw);
   std::string forloop(const std::string& varname, const std::string& from_raw, const std::string& to_raw, const std::string& step_raw, const std::string& inner_separator);
   std::string forloop(const std::string& varname, const std::vector<std::string>& loop_vec, const std::string& inner_separator);
   std::string ifChoice(const std::string bool_str);
   std::string element(const std::string& num_str);
   float stringToFloat(const std::string& str);
   std::string substring(const std::string& beg, const std::string& end);
   std::string newMethod(const std::string& methodName, const std::string& numPars);
   std::string newMethodD(const std::string& methodName, const std::string& numPars, const std::string& parameterPattern);
   std::string executeCommand(const std::string& methodName, const std::vector<std::string>& pars);
   std::string makroPattern(const int i, const std::string& pattern);
   std::string applyMethodString(const std::string& method_name, const std::vector<std::string>& parameters);

private:
   /// If you try to avoid assigning the parameter,
   /// be careful, it's not as simple as it seems.
   static std::string createDummyrep(
      const std::string& processed,
      RepresentableAsPDF repToProcess);

   /// In a String *EXPR*.m1[p11, p12, ...].m2[p21, p22, ...].m3[...]...
   /// extract the *EXPR* part. *EXPR* is determined by cutting off from position x
   /// which is the position of the first dot "." after which only alpha-numeric
   /// characters, white space, dots or parts in brackets "[" and "]" occur.
   ///
   /// @param chain  The string to extract from.
   ///
   /// @return  The expression part (the whole string if there are no dots or
   ///          brackets).
   static std::string extractExpressionPart(const std::string& chain2);

   static std::string overwriteBrackets(const std::string& s);
   static std::string singleCharSeq(const char c, const int length);
   static std::vector<std::string> getMethodSinaturesFromChain(const std::string& chainRest);
   static std::string getConversionTag(const std::string& scriptWithoutComments);
   static std::vector<std::string> getMethodParametersWithTags(const std::string& conversionTag);

   std::vector<std::string> getParametersFor(const std::vector<std::string>& rawPars);
   RepresentableAsPDF applyMethodChain(const RepresentableAsPDF original, const std::vector<std::string>& methodsToApply);
   std::vector<std::string> getMethodParameters(const std::string& conversionTag);

   /// @param rep              The representable to apply the method to.
   /// @param methodSignature  The method internal name + parameters.
   ///
   /// @return  A new representable with the method applied.Note that rep
   /// can be changed in the process.
   RepresentableAsPDF applyMethod(const RepresentableAsPDF rep, const std::string& methodSignature);

   /// Evaluates a single chain of conversion method applications to a script.
   ///
   /// @param chain       The method chain to apply as: "*script*.m1.m2.m3.m4..."
   ///                    where *script* is a script enclosed in @{ ... }@
   ///                    or "this", and m1, m2, ... are method signatures.
   /// @param father      The rep {@code this} is a sub-script of.
   ///
   /// @return  A representable with the methods applied. Can return <code>null
   ///          </code> when the chain is not valid (e.g. when starting with a
   ///          variable).
   std::shared_ptr<Script> evaluateChain(const std::string& chain);

   /// Goes through the current version of
   /// {@link RepresentableDefault#processedScript} and replaces
   /// all inscript preprocessor parts with their resulting
   /// expanded value. In this process, chains are evaluated, possibly
   /// leading to the evaluation of new scripts and recursive calls of this
   /// method.</BR>
   /// </BR>
   /// Non-inscript preprocessors are evaluated during declarations evaluation.
   ///
   /// @return  Iff there has been a change on
   ///          {@link RepresentableDefault#processedScript}.
   void extractInscriptProcessors();

   /// Searches for plain-text parts <code>"@{ *PLAINTEXT/// }@"</code> in the
   /// script and replaces all symbols within them with placeholders, thereby
   /// also removing the surrounding tags.<BR/>
   /// <BR/>
   /// Note that there is no nesting of plain-text tags, meaning that
   /// <p><code>"@{ text "@{ more text}@"  }@"</code></p>
   /// will yield only one plain-text part
   /// <p><code>text "@{ more text</code></p>
   /// where the inside opening tag will get replaced by a placeholder and
   /// re-introduced later, while the extra outside closing tag will be
   /// removed for good leaving a trailing opening tag. In short:
   /// Don't do that!
   ///
   /// @param script  A normal script.
   /// @param deletePlainTextTags  Iff the plain text tags are to be removed from
   ///                             (or else left to remain in) the script.
   /// @return  The script without plain-text tags, and all symbols replaced
   ///          by placeholders.
   std::string inferPlaceholdersForPlainText(const std::string& script);

   /// <p>Plain-text tags mark regions that are not supposed to be subject to
   /// pre-processors or sub-scripts or declaration handling. Usually, they
   /// are read during first processing in
   /// {@link RepresentableDefault#applyDeclarationsAndPreprocessors(String)}
   /// and deleted afterward to avoid any side effects during the actual
   /// code translation by the specific lower-level Representable. Therefore,
   /// the default return value of this method is {@code true}. However,
   /// it may be desireable to keep the plain-text tags in the code and remove
   /// them on the lower level, to be able to identify them later if the
   /// script has to be translated another time.</p>
   /// <p>This is, at implementation
   /// time of this method, only the case for {@link LaTeX} scripts. (There,
   /// the tags are just not cared about during translation as they do not
   /// hurt, and finally removed by {@link LaTeXPDF} after a possible second
   /// translation run.)</p>
   ///
   /// @return  Iff the plain-text tags should be removed after the application
   ///          of pre-processors and declarations (or else kept in the code
   ///          for later use).
   bool removePlaintextTagsAfterPreprocessorApplication() const;

   /// Replaces all symbols in a string with the according placeholders or vice versa.
   ///
   /// @param toReplace  The string to process.
   /// @param to         Iff the replacement goes in the direction:
   ///                   symbols ==TO==> placeholders (or else:
   ///                   symbols <=FROM= placeholders).
   /// @return  The string with replaced symbols or placeholders.
   std::string replacePlaceholders(const std::string& toReplace, const bool to);

   /// @param symbolList       This list is filled with all symbols that have to be replaced.
   /// @param placeholderList  This list is filled with the according object-specific placeholders.
   void fillLists(std::vector<std::string>& symbolList, std::vector<std::string>& placeholderList);

   void putPlaceholderMapping(const char symbolName);
   void putPlaceholderMapping(const std::string& symbolName);

   /// @param placeholder  The object - specific placeholder in a plain - text
   /// section of the script.
   /// @return  The original symbol.
   std::string placeholderToSymbol(const std::string& placeholder);

   /// @param symbol  A symbol to be ignored in a plain-text section of the
   ///                script.
   /// @return  The object-specific placeholder to replace the symbol with.
   std::string symbolToPlaceholder(const std::string& symbol);

   /// Returns the next position which is after the complete method chain of
   /// the preprocessor.
   ///
   /// @param partAfter  The part AFTER the preprocessor base script.
   /// @return  The next position outside the preprocessor's method chain.
   int getNextNonInscriptPosition(const std::string& partAfter);

   /// Undoes the placeholder replacement for plain-text parts. As the placeholders
   /// were object-specific, we don't care about what has happened in the
   /// meantime with the plain-text parts, but just replace all the
   /// placeholders belonging to {@code this} object.
   ///
   /// @param script  The whole script.
   /// @return        All object-specific placeholders undone.
   std::string undoPlaceholdersForPlainText(const std::string& script);

   /// Removes all the tagged parts on the top level of the given string. More
   /// precisely, for tags "(", ")", the string:
   /// "((test) more test) still testing (not (quite) finished); (almost (there))."
   /// will yield:
   /// " still testing ; ." Note that all tags will be removed - this makes a
   /// difference for malformed strings, such as "(test))" which will NOT
   /// yield ")", but "".
   ///
   /// @param code           The string to remove tagged parts from.
   /// @param beginTag       The begin tag.
   /// @param endTag         The end tag.
   /// @param ignoreBegTags  List of tags to ignore in between.
   /// @param ignoreEndTags  List of tags to ignore in between.
   /// @return  The string without tagged parts.
   std::string removeTaggedPartsOnTopLevel(
      const std::string& code,
      const std::string& beginTag,
      const std::string& endTag,
      std::vector<std::string> ignoreBegTags,
      std::vector<std::string> ignoreEndTags);

   /// @param string  A string, possibly containing arithmetic expressions.
   ///
   /// @return  The string with each expression replaced by its evaluation.
   std::string evaluateAll(const std::string& string, const std::string& opening_tag, const std::string& closing_tag);

   /// Finds the inner-most, left-most inscript preprocessor position, by
   /// preferring more <code>*</code> symbols in the end over less.</BR>
   /// </BR>
   /// For example:
   /// <UL>
   /// <LI><code>.@{.@{.}@.}@.@{.}@.</code> will return 4.</LI>
   /// <LI><code>.@{.@{.}@.}@.@{.}@*.</code> will return 13.</LI>
   /// <LI><code>.@{.@{.}@.}@*.@{.}@*.</code> will return 1.</LI>
   /// <LI><code>.@{.@{.}@.}@*.@{.}@*.</code> will return 1.</LI>
   /// <LI><code>.@{.@{.}@.}@*.@{.}@*.@{.}@**</code> will return 21.</LI>
   /// <LI><code>.@{.@{.}@*.}@*.@{.}@*.</code> will return 4.</LI>
   /// </UL>
   /// As a side effect, the extra <code>@</code> symbols of the end tag
   /// matching the returned begin tag position are deleted from
   /// processedScript.
   ///
   /// @return  The next inscript begin tag position. If no such position
   ///          exists, -1 is returned and no side effects occur.
   int findNextInscriptPos();

   ScriptData& getScriptData();

   /// If the script is embedded in plain text tags, replace all symbols with
   /// placeholders, but leave plain-text tags for later.
   ///
   /// @param  script  The script to check.
   /// @return  The possibly processed script.
   std::string checkForPlainTextTags(const std::string& script);

   std::string raw_script_{};
   std::string processed_script_{};

   std::map<std::string, MethodPartBegin> method_part_begins_{};
   std::shared_ptr<DataPack> vfm_data_{};
   std::shared_ptr<FormulaParser> vfm_parser_{};
   std::vector<std::string> scriptSequence_{};

   std::set<ScriptMethodDescription> METHODS{
      { "for", 2, [this](const std::vector<std::string>& parameters) { return forloop(parameters.at(0), parameters.at(1)); } },
      { "for", 3, [this](const std::vector<std::string>& parameters) { return forloop(parameters.at(0), parameters.at(1), parameters.at(2)); } },
      { "for", 4, [this](const std::vector<std::string>& parameters) { return forloop(parameters.at(0), parameters.at(1), parameters.at(2), parameters.at(3)); } },
      { "for", 5, [this](const std::vector<std::string>& parameters) { return forloop(parameters.at(0), parameters.at(1), parameters.at(2), parameters.at(3), parameters.at(4)); } },
      { "serialize", 0, [this](const std::vector<std::string>& parameters) { return formatExpression(getRawScript(), SyntaxFormat::vfm); } },
      { "serializeK2", 0, [this](const std::vector<std::string>& parameters) { return toK2(getRawScript()); } },
      { "serializeNuXmv", 0, [this](const std::vector<std::string>& parameters) { return formatExpression(getRawScript(), SyntaxFormat::nuXmv); } },
      { "atVfmTupel", 1, [this](const std::vector<std::string>& parameters) {
         auto fmla = MathStruct::parseMathStruct(StaticHelper::replaceAll(getRawScript(), ";", ","), getParser(), getData());
         assert(StaticHelper::isParsableAsInt(parameters[0]));
         const int at{ std::stoi(parameters[0]) };
         return fmla->getTermsJumpIntoCompounds()[at]->serializePlainOldVFMStyle();
      } },
      { "simplify", 0, [this](const std::vector<std::string>& parameters) { return simplifyExpression(getRawScript()); } },
      { "eval", 0, [this](const std::vector<std::string>& parameters) { return evaluateExpression(getRawScript()); } },
      { "eval", 1, [this](const std::vector<std::string>& parameters) { return evaluateExpression(getRawScript(), parameters.at(0)); } },
      { "nil", 0, [this](const std::vector<std::string>& parameters) { return nil(); } },
      { "id", 0, [this](const std::vector<std::string>& parameters) { return id(); } },
      { "idd", 0, [this](const std::vector<std::string>& parameters) { return idd(); } },
      { "if", 1, [this](const std::vector<std::string>& parameters) { return ifChoice(parameters.at(0)); } },
      { "at", 1, [this](const std::vector<std::string>& parameters) { return element(parameters.at(0)); } },
      { "substr", 2, [this](const std::vector<std::string>& parameters) { return substring(parameters.at(0), parameters.at(1)); } },
      { "split", 1, [this](const std::vector<std::string>& parameters) { 
         auto spl = StaticHelper::split(getRawScript(), parameters[0]);
         std::string res{};
         for (const auto& s : spl) {
            res += BEGIN_TAG_IN_SEQUENCE + s + END_TAG_IN_SEQUENCE;
         }
         return res;
      } },
      { "newMethod", 2, [this](const std::vector<std::string>& parameters) { return newMethod(parameters.at(0), parameters.at(1)); } },
      { "newMethod", 3, [this](const std::vector<std::string>& parameters) { return newMethodD(parameters.at(0), parameters.at(1), parameters.at(2)); } },
      { "strsize", 0, [this](const std::vector<std::string>& parameters) { return std::to_string(getRawScript().length()); } },
      { "seqlength", 0, [this](const std::vector<std::string>& parameters) { return std::to_string(scriptSequence_.size()); } },
      { "firstLetterCapital", 0, [this](const std::vector<std::string>& parameters) { return StaticHelper::firstLetterCapital(getRawScript()); } },
      { "toUpperCamelCase", 0, [this](const std::vector<std::string>& parameters) {  return StaticHelper::toUpperCamelCase(getRawScript()); } },
      { "toUpperCase", 0, [this](const std::vector<std::string>& parameters) { return StaticHelper::toUpperCase(getRawScript()); } },
      { "toLowerCase", 0, [this](const std::vector<std::string>& parameters) { return StaticHelper::toLowerCase(getRawScript()); } },
      { "removeLastFileExtension", 0, [this](const std::vector<std::string>& parameters) { return StaticHelper::removeLastFileExtension(getRawScript()); } },
      { "removeLastFileExtension", 1, [this](const std::vector<std::string>& parameters) { return StaticHelper::removeLastFileExtension(getRawScript(), parameters.at(0)); } },
      { "getLastFileExtension", 0, [this](const std::vector<std::string>& parameters) { return StaticHelper::getLastFileExtension(getRawScript()); } },
      { "getLastFileExtension", 1, [this](const std::vector<std::string>& parameters) { return StaticHelper::getLastFileExtension(getRawScript(), parameters.at(0)); } },
      { "substrComplement", 2, [this](const std::vector<std::string>& parameters) { return StaticHelper::substrComplement(getRawScript(), stringToFloat(parameters.at(0)), stringToFloat(parameters.at(1))); } },
      { "startsWithUppercase", 0, [this](const std::vector<std::string>& parameters) { return fromBooltoString(StaticHelper::startsWithUppercase(getRawScript())); } },
      { "stringStartsWith", 1, [this](const std::vector<std::string>& parameters) { return fromBooltoString(StaticHelper::stringStartsWith(getRawScript(), parameters.at(0))); } },
      { "stringStartsWith", 2, [this](const std::vector<std::string>& parameters) { return fromBooltoString(StaticHelper::stringStartsWith(getRawScript(), parameters.at(0), stringToFloat(parameters.at(0)))); } },
      { "stringEndsWith", 1, [this](const std::vector<std::string>& parameters) { return fromBooltoString(StaticHelper::stringEndsWith(getRawScript(), parameters.at(0))); } },
      { "stringContains", 1, [this](const std::vector<std::string>& parameters) { return fromBooltoString(StaticHelper::stringContains(getRawScript(), parameters.at(0))); } },
      { "shortenToMaxSize", 1, [this](const std::vector<std::string>& parameters) { return StaticHelper::shortenToMaxSize(getRawScript(), stringToFloat(parameters.at(0))); } },
      { "shortenToMaxSize", 2, [this](const std::vector<std::string>& parameters) { return StaticHelper::shortenToMaxSize(getRawScript(), stringToFloat(parameters.at(0)), StaticHelper::isBooleanTrue(parameters.at(1))); } },
      { "shortenToMaxSize", 3, [this](const std::vector<std::string>& parameters) { return StaticHelper::shortenToMaxSize(getRawScript(), stringToFloat(parameters.at(0)), StaticHelper::isBooleanTrue(parameters.at(1)), stringToFloat(parameters.at(2))); } },
      { "shortenToMaxSize", 4, [this](const std::vector<std::string>& parameters) { return StaticHelper::shortenToMaxSize(getRawScript(), stringToFloat(parameters.at(0)), StaticHelper::isBooleanTrue(parameters.at(1)), stringToFloat(parameters.at(2)), stringToFloat(parameters.at(3))); } },
      { "shortenInTheMiddle", 1, [this](const std::vector<std::string>& parameters) { return StaticHelper::shortenInTheMiddle(getRawScript(), stringToFloat(parameters.at(0))); } },
      { "shortenInTheMiddle", 2, [this](const std::vector<std::string>& parameters) { return StaticHelper::shortenInTheMiddle(getRawScript(), stringToFloat(parameters.at(0)), stringToFloat(parameters.at(1))); } },
      { "absPath", 0, [this](const std::vector<std::string>& parameters) { return StaticHelper::absPath(getRawScript()); } },
      { "zeroPaddedNumStr", 1, [this](const std::vector<std::string>& parameters) { return StaticHelper::zeroPaddedNumStr(stringToFloat(getRawScript()), stringToFloat(parameters.at(0))); } },
      { "removeWhiteSpace", 0, [this](const std::vector<std::string>& parameters) { return StaticHelper::removeWhiteSpace(getRawScript()); } },
      { "isEmptyExceptWhiteSpaces", 0, [this](const std::vector<std::string>& parameters) { return fromBooltoString(StaticHelper::isEmptyExceptWhiteSpaces(getRawScript())); } },
      { "removeMultiLineComments", 0, [this](const std::vector<std::string>& parameters) { return StaticHelper::removeMultiLineComments(getRawScript()); } },
      { "removeMultiLineComments", 2, [this](const std::vector<std::string>& parameters) { return StaticHelper::removeMultiLineComments(getRawScript(), parameters.at(0), parameters.at(1)); } },
      { "removeSingleLineComments", 0, [this](const std::vector<std::string>& parameters) { return StaticHelper::removeSingleLineComments(getRawScript()); } },
      { "removeSingleLineComments", 1, [this](const std::vector<std::string>& parameters) { return StaticHelper::removeSingleLineComments(getRawScript(), parameters.at(0)); } },
      { "removeComments", 0, [this](const std::vector<std::string>& parameters) { return StaticHelper::removeComments(getRawScript()); } },
      { "removePartsOutsideOf", 2, [this](const std::vector<std::string>& parameters) { return StaticHelper::removePartsOutsideOf(getRawScript(), parameters.at(0), parameters.at(1)); } },
      { "removeBlankLines", 0, [this](const std::vector<std::string>& parameters) { return StaticHelper::removeBlankLines(getRawScript()); } },
      { "wrap", 1, [this](const std::vector<std::string>& parameters) { return StaticHelper::wrap(getRawScript(), stringToFloat(parameters.at(0))); } },
      { "openWithOS", 0, [this](const std::vector<std::string>& parameters) { StaticHelper::openWithOS(getRawScript(), this); return ""; } },
      { "replaceAll", 2, [this](const std::vector<std::string>& parameters) { return StaticHelper::replaceAll(getRawScript(), parameters.at(0), parameters.at(1)); } },
      { "replaceAllCounting", 1, [this](const std::vector<std::string>& parameters) { return StaticHelper::replaceAllCounting(getRawScript(), parameters.at(0)); } },
      { "replaceAllCounting", 2, [this](const std::vector<std::string>& parameters) { return StaticHelper::replaceAllCounting(getRawScript(), parameters.at(0), std::stof(parameters.at(1))); } },
      { "replaceAllRegex", 2, [this](const std::vector<std::string>& parameters) { return StaticHelper::replaceAllRegex(getRawScript(), parameters.at(0), parameters.at(1)); } },
   };
};

} // macro
} // vfm
