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
#include "gui/process_helper.h"
#include "simulation/road_graph.h"
#include "model_checking/mc_workflow.h"
#include "model_checking/mc_types.h"
#include "testing/interactive_testing.h"
#include <vector>
#include <thread>
#include <chrono>
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
static constexpr char INSCR_PRIORITY_SYMB = '*';

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

static const std::string BEGIN_TAG_IN_SEQUENCE = "@(";
static const std::string END_TAG_IN_SEQUENCE = ")@";

static const std::string PREPROCESSOR_FIELD_NAME = "prep";
static const std::string VARIABLE_DELIMITER = "=";
static const char END_VALUE = ';';
static const std::string INSCRIPT_STANDARD_PARAMETER_PATTERN = "#n#";

static const std::string MC_PACKAGE_PREFIX{ "gp" };

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

static std::map<int, std::shared_ptr<Script>> running_scripts{};
static int first_free_script_id{ 0 };

struct ScriptMethodDescription {
   std::string method_name_{};
   int par_num_{};
   std::function<std::string(const std::string&, const std::vector<std::string>&)> function_{};

   bool operator< (const ScriptMethodDescription& o) const {
      return (method_name_ == o.method_name_) ? (par_num_ < o.par_num_) : (method_name_ < o.method_name_);
   }
};

class Script : public Failable, public std::enable_shared_from_this<Script> {
public:
   static std::map<int, std::shared_ptr<RoadGraph>> road_graphs_; // TODO: Non-static!
   static std::map<std::string, std::shared_ptr<StraightRoadSection>> straight_road_sections_; // TODO: Non-static!

   Script() = delete;
   Script(const std::shared_ptr<DataPack> data, const std::shared_ptr<FormulaParser> parser);

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
   /// @param codeRaw2  The code to parse. Comments must have been removed already.
   /// @param only_one_step  Iff only one step of preprocessor application is to be performed.
   /// @return  Debug code if requested, <code>null</code> otherwise.
   std::string applyDeclarationsAndPreprocessors(const std::string& codeRaw2, const bool only_one_step);

   enum class DataPreparation {
      copy_data_pack_before_run, // Creates a copy of the data pack and works on that. I.e., you won't see the changes in the original data pack if something is altered by the script. Typically needed to make thread-safe.
      reset_script_data_before_run, // Typically desired, unless you want to re-use dynamic methods, cache etc. over multiple sessions.
      both,
      none
   };

   enum class SpecialOption {
      add_default_dynamic_methods,
      none
   };

   // TODO: Get rid of this static method and put most of the logic into the constructor of Script.
   static std::string processScript(
      const std::string& text,
      const DataPreparation data_prep = DataPreparation::none,
      const bool only_one_step = false,
      const std::shared_ptr<DataPack> data_raw = nullptr,        // Data and parser preferred over those given by script_this.
      const std::shared_ptr<FormulaParser> parser = nullptr,
      const std::shared_ptr<Failable> father_failable = nullptr,
      const SpecialOption option = SpecialOption::none,
      const std::shared_ptr<Script> script_this = nullptr);

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
   std::string idd(const std::string& body) { return StaticHelper::stringStartsWith(body, PLAIN_TEXT_BEGIN_TAG) ? body : PLAIN_TEXT_BEGIN_TAG + body + PLAIN_TEXT_END_TAG; }

   std::shared_ptr<DataPack> getData() const;
   std::shared_ptr<FormulaParser> getParser() const;
   std::string getMyPath() const;

   /// Looks for a sequence of several chunks of
   /// <code>@(...)@@(...)@@(...)@</code> in the code and puts them into
   /// <code>scriptSequence</code>.
   ///
   /// @param code  The raw code to process.
   std::vector<std::string> processSequence(const std::string& code);
   void processSequence(const std::string& code, std::vector<std::string>& script_sequence);
   std::string evalItAllF(const std::string& n1Str, const std::string& n2Str, const std::function<float(float n1, float n2)> eval);
   std::string evalItAllI(const std::string& n1Str, const std::string& n2Str, const std::function<long long(long long n1, long long n2)> eval);

   /// Retrieves the raw script, but without any <code>@{</code> or <code>}@</code>.
   /// Can be used to get the constant <code>2</code> from raw script
   /// <code>@{2}@</code>. But use carefully, since it actually removes ALL
   /// tags.
   ///
   /// @return  The raw script without any inscript tags.
   std::string getTagFreeRawScript(const std::string& body);

   std::string sethard(const std::string& body, const std::string& value) {
      getScriptData().method_part_begins_[getTagFreeRawScript(body)].result_ = value; return value; 
   }

   std::string exsmeq(const std::string& body, const std::string& num) { return evalItAllF(getTagFreeRawScript(body), num, [](float a, float b) { return a <= b; }); }
   std::string exsm(const std::string& body, const std::string& num) { return evalItAllF(getTagFreeRawScript(body), num, [](float a, float b) { return a < b; }); }
   std::string exgreq(const std::string& body, const std::string& num) { return evalItAllF(getTagFreeRawScript(body), num, [](float a, float b) { return a >= b; }); }
   std::string exgr(const std::string& body, const std::string& num) { return evalItAllF(getTagFreeRawScript(body), num, [](float a, float b) { return a > b; }); }
   std::string exeq(const std::string& body, const std::string& num) { return evalItAllF(getTagFreeRawScript(body), num, [](float a, float b) { return a == b; }); }
   std::string exneq(const std::string& body, const std::string& num) { return evalItAllF(getTagFreeRawScript(body), num, [](float a, float b) { return a != b; }); }
   std::string exadd(const std::string& body, const std::string& num) { return evalItAllF(getTagFreeRawScript(body), num, [](float a, float b) { return a + b; }); }
   std::string exsub(const std::string& body, const std::string& num) { return evalItAllF(getTagFreeRawScript(body), num, [](float a, float b) { return a - b; }); }
   std::string exmult(const std::string& body, const std::string& num) { return evalItAllF(getTagFreeRawScript(body), num, [](float a, float b) { return a * b; }); }
   std::string exdiv(const std::string& body, const std::string& num) { return evalItAllF(getTagFreeRawScript(body), num, [](float a, float b) { return a / b; }); }
   std::string expow(const std::string& body, const std::string& num) { return evalItAllF(getTagFreeRawScript(body), num, [](float a, float b) { return std::pow(a, b); }); }
   std::string exmod(const std::string& body, const std::string& num) { return evalItAllF(getTagFreeRawScript(body), num, [](float a, float b) { return (int) a % (int) b; }); }

   std::string exsmeqI(const std::string& body, const std::string& num) { return evalItAllI(getTagFreeRawScript(body), num, [](long long a, long long b) { return a <= b; }); }
   std::string exsmI(const std::string& body, const std::string& num) { return evalItAllI(getTagFreeRawScript(body), num, [](long long a, long long b) { return a < b; }); }
   std::string exgreqI(const std::string& body, const std::string& num) { return evalItAllI(getTagFreeRawScript(body), num, [](long long a, long long b) { return a >= b; }); }
   std::string exgrI(const std::string& body, const std::string& num) { return evalItAllI(getTagFreeRawScript(body), num, [](long long a, long long b) { return a > b; }); }
   std::string exeqI(const std::string& body, const std::string& num) { return evalItAllI(getTagFreeRawScript(body), num, [](long long a, long long b) { return a == b; }); }
   std::string exneqI(const std::string& body, const std::string& num) { return evalItAllI(getTagFreeRawScript(body), num, [](long long a, long long b) { return a != b; }); }
   std::string exaddI(const std::string& body, const std::string& num) { return evalItAllI(getTagFreeRawScript(body), num, [](long long a, long long b) { return a + b; }); }
   std::string exsubI(const std::string& body, const std::string& num) { return evalItAllI(getTagFreeRawScript(body), num, [](long long a, long long b) { return a - b; }); }
   std::string exmultI(const std::string& body, const std::string& num) { return evalItAllI(getTagFreeRawScript(body), num, [](long long a, long long b) { return a * b; }); }
   std::string exdivI(const std::string& body, const std::string& num) { return evalItAllI(getTagFreeRawScript(body), num, [](long long a, long long b) { return a / b; }); }
   std::string expowI(const std::string& body, const std::string& num) { return evalItAllI(getTagFreeRawScript(body), num, [](long long a, long long b) { return std::pow(a, b); }); }
   std::string exmodI(const std::string& body, const std::string& num) { return evalItAllI(getTagFreeRawScript(body), num, [](long long a, long long b) { return (int) a % (int) b; }); }

   std::string exnot(const std::string& body) { return std::to_string(!StaticHelper::isBooleanTrue(getTagFreeRawScript(body)));}
   std::string space() { return " "; }

   std::string fromBooltoString(const bool b);

   std::string arclengthCubicBezierFromStreetTopology(const std::string& lane, const std::string& angle, const std::string& distance, const std::string& num_lanes, const std::string& lane_width);
   std::string forloop(const std::string& body, const std::string& varname, const std::string& loop_vec);
   std::string forloop(const std::string& body, const std::string& varname, const std::string& from_raw, const std::string& to_raw);
   std::string forloop(const std::string& body, const std::string& varname, const std::string& from_raw, const std::string& to_raw, const std::string& step_raw);
   std::string forloop(const std::string& body, const std::string& varname, const std::string& from_raw, const std::string& to_raw, const std::string& step_raw, const std::string& inner_separator, const std::set<std::string>& except = {});
   std::string forloop(const std::string& body, const std::string& varname, const std::vector<std::string>& loop_vec, const std::string& inner_separator);
   std::string ifChoice(const std::string& sequence_str, const std::string bool_str);
   std::string element(const std::string& sequence_str, const std::string& num_str);
   float stringToFloat(const std::string& str);
   std::string substring(const std::string& body, const std::string& beg, const std::string& end);
   std::string newMethod(const std::string& body, const std::string& methodName, const std::string& numPars);
   std::string newMethodD(const std::string& body, const std::string& methodName, const std::string& numPars, const std::string& parameterPattern);
   std::string executeCommand(const std::string& body, const std::string& methodName, const std::vector<std::string>& pars);
   std::string makroPattern(const int i, const std::string& pattern);
   std::string applyMethodString(const std::string& body, const std::string& method_name, const std::vector<std::string>& parameters);

   bool isNativeMethod(const std::string& method_name) const;
   bool isDynamicMethod(const std::string& method_name) const;
   bool isMethod(const std::string& method_name) const;
   bool isCachableMethod(const std::string& method_name) const;
   bool makeMethodCachable(const std::string& method_name);   // Returns false if the method was already cachable.
   bool makeMethodUnCachable(const std::string& method_name); // Returns false if the method was already uncachable.

   void addDefaultDynamicMathods();

private:
   bool isCachableChain(const std::vector<std::string>& method_chain) const;

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
   void applyMethodChain(std::string& original, const std::vector<std::string>& methodsToApply);
   std::vector<std::string> getMethodParameters(const std::string& conversionTag);

   /// @param rep              The representable to apply the method to.
   /// @param methodSignature  The method internal name + parameters.
   ///
   /// @return  A new representable with the method applied.Note that rep
   /// can be changed in the process.
   void applyMethod(std::string& rep, const std::string& methodSignature);

   /// Evaluates a single chain of conversion method applications to a script.
   ///
   /// @param chain       The method chain to apply as: "*script*.m1.m2.m3.m4..."
   ///                    where *script* is a script enclosed in @{ ... }@
   ///                    or "this", and m1, m2, ... are method signatures.
   /// @param father      The rep {@code this} is a sub-script of.
   ///
   /// @return  The evaluated script as string.
   std::string evaluateChain(const std::string& chain);

   /// Goes through the current version of
   /// {@link RepresentableDefault#processedScript} and replaces
   /// all inscript preprocessor parts with their resulting
   /// expanded value. In this process, chains are evaluated, possibly
   /// leading to the evaluation of new scripts and recursive calls of this
   /// method.</BR>
   /// </BR>
   /// Non-inscript preprocessors are evaluated during declarations evaluation.
   void extractInscriptProcessors(std::string& processed_script, const bool only_one_step);

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
   int findNextInscriptPos(std::string& script);

   ScriptData& getScriptData() const;

   /// If the script is embedded in plain text tags, replace all symbols with
   /// placeholders, but leave plain-text tags for later.
   ///
   /// @param  script  The script to check.
   /// @return  The possibly processed script.
   std::string checkForPlainTextTags(const std::string& script);

   std::shared_ptr<DataPack> vfm_data_{};
   std::shared_ptr<FormulaParser> vfm_parser_{};

   int id_{ -1 };                                             // Invalid default ID, overwritten by constructor.
   bool stop_me_{ false };                                    // Set to true to abort further script evaluation and return current state.
   std::vector<std::shared_ptr<Script>> detached_children_{}; // Subsequent script processing threads detached from this.

   inline std::string identifyWithChildren() const
   {
      std::string res{ std::to_string(id_) };

      if (!detached_children_.empty()) res += "{ ";

      for (const auto& child : detached_children_) {
         res += child->identifyWithChildren() + " ";
      }

      if (!detached_children_.empty()) res += "}";

      return res;
   }

   inline std::string setScriptVar(const std::string& body, const std::string& parameter1, const std::string& parameter2)
   { // Any parameter2 will be counted as "FORCE".
      std::string varname{ parameter1 };

      if (parameter2 == "" && getScriptData().list_data_.count(varname)) {
         std::string error{ "Variable '" + varname + "' has already been defined, use 'force' as second argument to force re-definition." };
         addError(error);
         return "#" + error + "#";
      }

      getScriptData().list_data_[varname] = { body };

      return body;
   }

   inline std::string listElement(const std::string& body, const std::vector<std::string>& parameters)
   {
      if (!getScriptData().list_data_.count(body)) {
         std::string error_str{ "#ERROR<list '" + body + "' not found by listElement method>" };
         addError(error_str);
         return error_str;
      }

      if (!StaticHelper::isParsableAsFloat(parameters[0])) {
         std::string error_str{ "#ERROR<index '" + parameters[0] + "' cannot be interpreted as number for listElement method>" };
         addError(error_str);
         return error_str;
      }

      int index{ std::stoi(parameters[0]) };

      if (index < 0 || index >= getScriptData().list_data_[body].size()) {
         std::string error_str{ "#ERROR<index '" + parameters[0] + "' is out of bounds for list '" + body + "' (size " + std::to_string(getScriptData().list_data_[body].size()) + ") in method listElement>" };
         addError(error_str);
         return error_str;
      }

      return getScriptData().list_data_[body][index];
   }

   std::string connectRoadGraphTo(const std::string& body, const std::string& id2_str);
   std::string storeRoadGraph(const std::string& body, const std::string& filename);
   std::string createRoadGraph(const std::string& body, const std::string& id);

   ScriptMethodDescription m0{ "forExcept", 6, [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string { return forloop(body, parameters.at(0), parameters.at(1), parameters.at(2), parameters.at(3), parameters.at(4), { parameters.at(5) }); } };
   ScriptMethodDescription m1{ "for", 2, [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string { return forloop(body, parameters.at(0), parameters.at(1)); } };
   ScriptMethodDescription m2{ "for", 3, [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string { return forloop(body, parameters.at(0), parameters.at(1), parameters.at(2)); } };
   ScriptMethodDescription m3{ "for", 4, [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string { return forloop(body, parameters.at(0), parameters.at(1), parameters.at(2), parameters.at(3)); } };
   ScriptMethodDescription m4{ "for", 5, [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string { return forloop(body, parameters.at(0), parameters.at(1), parameters.at(2), parameters.at(3), parameters.at(4)); } };

   std::map<std::string, std::string> retrievePaths(
      const mc::McWorkflow& workflow, 
      const std::string& relative_path_to_json_tpl_file,
      const std::string& config_name)
   {
      std::string json_tpl_filename{ DEFAULT_FILE_NAME_JSON_TEMPLATE };
      std::string path_json{ getMyPath() };

      if (!relative_path_to_json_tpl_file.empty()) {
         json_tpl_filename = StaticHelper::getLastFileExtension(relative_path_to_json_tpl_file, "/");
         path_json = path_json + "/" + StaticHelper::removeLastFileExtension(relative_path_to_json_tpl_file, "/");
      }

      std::map<std::string, std::string> map{};
      const std::string path_generated_parent{ workflow.getGeneratedParentDir(path_json, json_tpl_filename).string() };
      const std::string path_generated{ workflow.getGeneratedDir(path_json, json_tpl_filename).string() + config_name };
      const std::string path_cached{ workflow.getCachedDir(path_json, json_tpl_filename).string() };
      const std::string path_external{ workflow.getExternalDir(path_json, json_tpl_filename).string() };
      const std::string path_template{ workflow.getEnvmodelDir(path_json, json_tpl_filename).string() };

      map.insert({ "path_json", path_json });
      map.insert({ "path_template", path_template });
      map.insert({ "path_generated", path_generated });
      map.insert({ "path_generated_parent", path_generated_parent });
      map.insert({ "path_cached", path_cached });
      map.insert({ "path_external", path_external });

      for (const auto& p : map) {
         addNote("Setting directory '" + p.first + "' to '" + StaticHelper::absPath(p.second, false) + "'.");
      }

      return map;
   }

   mc::McWorkflow prepareMCWorkflow(const std::shared_ptr<DataPack> data, const std::shared_ptr<FormulaParser> parser, const bool set_default_templates_path)
   {
      if (set_default_templates_path) {
         data->addStringToDataPack("../src/templates", MY_PATH_VARNAME);
      }

      return mc::McWorkflow(data, parser);
   }

   ScriptMethodDescription m5{
      "runMCJob", 
      1, 
      [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string 
      { 
         auto mc_workflow = prepareMCWorkflow(vfm_data_, vfm_parser_, body.empty());
         std::string config_name{ parameters.at(0) };
         std::map<std::string, std::string> paths{ retrievePaths(mc_workflow, body, config_name)};

         mc_workflow.runMCJob(
            paths.at("path_generated"),
            config_name,
            paths.at("path_template"),
            paths.at("path_json"),
            DEFAULT_FILE_NAME_JSON_TEMPLATE);

         return "MC run via script finished for '" + config_name + "'.";
      } 
   };

   ScriptMethodDescription m6{
      "runMCJobs",
      1,
      [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string
      {
         auto mc_workflow = prepareMCWorkflow(vfm_data_, vfm_parser_, body.empty());
         std::string num_threads_str{ parameters.at(0) };

         if (!StaticHelper::isParsableAsFloat(num_threads_str)) {
            return "#ERROR<Parameter for number of threads '" + num_threads_str + "' is not a valid number in runMCJobs method>#";
         }

         std::map<std::string, std::string> paths{ retrievePaths(mc_workflow, body, "")};

         mc_workflow.runMCJobs(
            std::filesystem::path(paths.at("path_generated")), // TODO: Don't need this parameter.
            [](const std::string& folder) -> bool { return true; },
            paths.at("path_template"),
            paths.at("path_json"),
            DEFAULT_FILE_NAME_JSON_TEMPLATE, 
            std::stoi(num_threads_str));

         return "MC runs via script finished.";
      }
   };

   ScriptMethodDescription m7{
      "generateEnvmodels",
      0,
      [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string
      {
         auto mc_workflow = prepareMCWorkflow(vfm_data_, vfm_parser_, body.empty());
         std::map<std::string, std::string> paths{ retrievePaths(mc_workflow, body, "") }; // Note that only path_template is used, the others might be broken.

         mc_workflow.generateEnvmodels(paths.at("path_json"), DEFAULT_FILE_NAME_JSON_TEMPLATE, FILE_NAME_ENVMODEL_ENTRANCE, nullptr);

         return "Envmodel generation via script finished.";
      }
   };

   std::string allModesStr() {
      const auto all_modes = mc::ALL_TEST_CASE_MODES_PLAIN_NAMES();
      std::string all_modes_str{};
      all_modes_str += all_modes.at(0);
      for (int i = 1; i < all_modes.size(); i++) {
         const std::string mode{ all_modes.at(i) };
         all_modes_str += "/" + mode;
      }
      return all_modes_str;
   }

   ScriptMethodDescription m8{
      "generateTestCases",
      0,
      [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string
      {
         std::string result{ std::string("No modes given for method 'generateTestCases'. Please add 'all' or '/'-separated selection of these as parameter: ") + "[" + allModesStr() + "]" };
         addNote(result);
         return result;
      }
   };

   ScriptMethodDescription m9{
      "generateTestCases",
      1,
      [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string
      {
         auto mc_workflow = prepareMCWorkflow(vfm_data_, vfm_parser_, body.empty());
         const std::map<std::string, std::string> paths{ retrievePaths(mc_workflow, body, "") };
         const std::string path_generated_parent{ paths.at("path_generated_parent") };
         const std::string path_json{ paths.at("path_json") };
         const std::string path_template{ paths.at("path_template") };

         std::vector<std::string> sec_ids{};

         try {
            for (const auto& entry : std::filesystem::directory_iterator(path_generated_parent)) { // Find all packages.
               std::string possible{ entry.path().filename().string() };
               if (std::filesystem::is_directory(entry) && possible != MC_PACKAGE_PREFIX && StaticHelper::stringStartsWith(possible, MC_PACKAGE_PREFIX)) {
                  sec_ids.push_back(possible);
               }
            }
         }
         catch (const std::exception& ex) {
            addError("Error occurred during collection of packages for test case generation: '" + std::string(ex.what()) + "'");
         }

         const std::string raw_modes{ parameters[0] == "all" ? allModesStr() : parameters[0]};
         std::map<std::string, std::string> modes{};
         std::string modes_str{};

         for (const auto& mode : StaticHelper::split(raw_modes, "/")) {
            if (mc::ALL_TESTCASE_MODES.count(mode)) {
               modes.insert({ mode, mc::ALL_TESTCASE_MODES.at(mode) });
               modes_str += " " + mode;
            }
            else {
               addWarning("Mode candidate '" + mode + "' is not an available mode. Will be ignored.");
            }
         }

         std::string json_tpl_filename{ body.empty() ? DEFAULT_FILE_NAME_JSON_TEMPLATE : body };

         mc_workflow.createTestCases(modes, path_template, path_json, json_tpl_filename, sec_ids);

         return "Test case generation via script finished for these modes: '" + modes_str + " '.";
      }
   };

   ScriptMethodDescription m10{
      "makeCachable",
      0,
      [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string
      {
         if (!isMethod(body)) {
            std::string error{ "Neither dynamic nor native method found with name '" + body + "'." };
            addError(error);
            return "#INVALID(" + error + ")";
         }

         return makeMethodCachable(body) 
            ? "Method '" + body + "' has been made cachable." 
            : "Method '" + body + "' was already cachable.";
      }
   };

   ScriptMethodDescription m11{
      "makeUnCachable",
      0,
      [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string
      {
         if (!isMethod(body)) {
            std::string error{ "Neither dynamic nor native method found with name '" + body + "'." };
            addError(error);
            return "#INVALID(" + error + ")";
         }

         return makeMethodUnCachable(body)
            ? "Method '" + body + "' has been made uncachable. Note that existing cache entries will still remain, unless you call 'resetScriptData'."
            : "Method '" + body + "' was already uncachable.";
      }
   };

   ScriptMethodDescription m12{
      "resetScriptData",
      0,
      [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string
      {
         getScriptData().reset();
         return "Script data has been reset. (vfm data remains untouched.)";
      }
   };

   ScriptMethodDescription m13{
      "resetAllData",
      0,
      [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string
      {
         vfm_data_->reset();
         return "All data has been reset.";
      }
   };

   ScriptMethodDescription arith01f{ "smeq", 1, [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string { return exsmeq(body, parameters.at(0)); } };
   ScriptMethodDescription arith02f{ "sm", 1, [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string { return exsm(body, parameters.at(0)); } };
   ScriptMethodDescription arith03f{ "greq", 1, [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string { return exgreq(body, parameters.at(0)); } };
   ScriptMethodDescription arith04f{ "gr", 1, [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string { return exgr(body, parameters.at(0)); } };
   ScriptMethodDescription arith05f{ "eq", 1, [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string { return exeq(body, parameters.at(0)); } };
   ScriptMethodDescription arith06f{ "neq", 1, [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string { return exneq(body, parameters.at(0)); } };
   ScriptMethodDescription arith07f{ "add", 1, [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string { return exadd(body, parameters.at(0)); } };
   ScriptMethodDescription arith08f{ "sub", 1, [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string { return exsub(body, parameters.at(0)); } };
   ScriptMethodDescription arith09f{ "mult", 1, [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string { return exmult(body, parameters.at(0)); } };
   ScriptMethodDescription arith10f{ "div", 1, [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string { return exdiv(body, parameters.at(0)); } };
   ScriptMethodDescription arith11f{ "pow", 1, [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string { return expow(body, parameters.at(0)); } };
   ScriptMethodDescription arith12f{ "mod", 1, [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string { return exmod(body, parameters.at(0)); } };
   ScriptMethodDescription arith01i{ "SmeqI", 1, [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string { return exsmeqI(body, parameters.at(0)); } };
   ScriptMethodDescription arith02i{ "SmI", 1, [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string { return exsmI(body, parameters.at(0)); } };
   ScriptMethodDescription arith03i{ "GreqI", 1, [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string { return exgreqI(body, parameters.at(0)); } };
   ScriptMethodDescription arith04i{ "GrI", 1, [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string { return exgrI(body, parameters.at(0)); } };
   ScriptMethodDescription arith05i{ "EqI", 1, [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string { return exeqI(body, parameters.at(0)); } };
   ScriptMethodDescription arith06i{ "NeqI", 1, [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string { return exneqI(body, parameters.at(0)); } };
   ScriptMethodDescription arith07i{ "AddI", 1, [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string { return exaddI(body, parameters.at(0)); } };
   ScriptMethodDescription arith08i{ "SubI", 1, [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string { return exsubI(body, parameters.at(0)); } };
   ScriptMethodDescription arith09i{ "MultI", 1, [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string { return exmultI(body, parameters.at(0)); } };
   ScriptMethodDescription arith10i{ "DivI", 1, [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string { return exdivI(body, parameters.at(0)); } };
   ScriptMethodDescription arith11i{ "PowI", 1, [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string { return expowI(body, parameters.at(0)); } };
   ScriptMethodDescription arith12i{ "ModI", 1, [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string { return exmodI(body, parameters.at(0)); } };

   inline std::shared_ptr<Script> getRunningScriptOrSelf(const std::string& body)
   {
      int id{ id_ };

      if (!body.empty()) {
         if (StaticHelper::isParsableAsFloat(body)) { // Use id from script if available.
            id = (int)std::stof(body);
         }
         else {
            addError("Body '" + body + "' is not an id. Returning self.");
            return shared_from_this();
         }
      }

      if (running_scripts.count(id)) {
         return running_scripts[id];
      }
      else {
         addError("No running script with id '" + body + "' found. Returning self.");
         return shared_from_this();
      }
   }

   inline void stop()
   {
      addNote("Script '" + std::to_string(id_) + "' has been requested to stop.");
      stop_me_ = true;
   }

   inline void stopWithAllChildren()
   {
      for (const auto& child : detached_children_) {
         child->stopWithAllChildren();
      }

      stop();
   }

   ScriptMethodDescription m14{ "sleep", 1, [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string
      {
         std::this_thread::sleep_for(std::chrono::milliseconds((int)std::stof(parameters[0])));
         return body;
      } };

   ScriptMethodDescription m15{ "Detach", 0, [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string
      {
         std::shared_ptr<Script> detached_script{ std::make_shared<Script>(vfm_data_, vfm_parser_) };
         int detached_id{ detached_script->id_ };
         detached_children_.push_back(detached_script);

         std::thread t{
            macro::Script::processScript,
            body,
            macro::Script::DataPreparation::copy_data_pack_before_run,
            false,
            vfm_data_,
            vfm_parser_,
            nullptr, // Detach also from father Failable. TODO: Is that desired?
            SpecialOption::none,
            detached_script
         };

         t.detach();

         return std::to_string(detached_id);
      } };

   ScriptMethodDescription m16{ "Identify", 0, [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string 
      { 
         return "Me with my detached children: " + getRunningScriptOrSelf(body)->identifyWithChildren();
      }};

   ScriptMethodDescription m17{ "StopScript", 0, [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string 
   {
      auto script = getRunningScriptOrSelf(body);
      script->stopWithAllChildren();
      return "Requested to stop script '" + std::to_string(script->id_) + "' and all its children.";
   }};

   ScriptMethodDescription m18{ "ListRunningScripts", 0, [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string 
   {
      std::string res{ "Running scripts:\n" };

      for (const auto& pair : running_scripts) {
         const std::string me{ pair.first == id_ ? " (me)" : "" };
         res += pair.second->identifyWithChildren() + me + "\n";
      }

      return res;
   }};

   std::string includeMe(const std::string& body)
   {
      std::string filepath{ StaticHelper::absPath(getMyPath()) + "/" + body };

      if (StaticHelper::existsFileSafe(filepath)) {
         return StaticHelper::readFile(filepath);
      }
      else {
         addFatalError("File '" + filepath + "' not found.");
         return "#FILE_NOT_FOUND<" + filepath + ">";
      }
   }

   std::set<ScriptMethodDescription> METHODS{
      m0,
      m1,
      m2,
      m3,
      m4,
          // In the following examples, the json template filename is the default, but can also explicitly be given as @{SOME_NAME.tpl.json}@.
      m5, // Example: @{}@.runMCJob[_config_d=1000_lanes=1_maxaccel=3_maxaccelego=3_minaccel=-8_minaccelego=-8_nonegos=3_sections=5_segments=1_t=1100_vehlen=5]
      m6, // Example: @{}@.runMCJobs[10]
      m7, // Example: @{}@.generateEnvmodels
      m8, // Example: @{}@.generateTestCases       ==> Will fail, but present list of available modes.
      m9, // Example: @{}@.generateTestCases[all]
          // Full example (enclosing tags @<...>@ only for M²oRTy UI, leave out for plain script processing):
          // @<
          // @{}@.generateEnvmodels 
          // @{}@.runMCJobs[10] 
          // @{}@.generateTestCases[all]
          // >@
          // Don't forget to add '@{../src/templates/}@.stringToHeap[MY_PATH]' if desired.
      m10,
      m11,
      m12,
      m13,
      m14,
      m15,
      m16,
      m17,
      m18,
      { "serialize", 0, [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string { return formatExpression(body, SyntaxFormat::vfm); } },
      { "serializeK2", 0, [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string { return toK2(body); } },
      { "serializeNuXmv", 0, [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string { return formatExpression(body, SyntaxFormat::nuXmv); } },
      { "atVfmTupel", 1, [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string {
         auto fmla = MathStruct::parseMathStruct(StaticHelper::replaceAll(body, ";", ","), getParser(), getData());
         assert(StaticHelper::isParsableAsInt(parameters[0]));
         const int at{ std::stoi(parameters[0]) };
         return fmla->getTermsJumpIntoCompounds()[at]->serializePlainOldVFMStyle();
      } },
      { "simplify", 0, [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string { return simplifyExpression(body); } },
      { "eval", 0, [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string { return evaluateExpression(body); } },
      { "eval", 1, [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string { return evaluateExpression(body, parameters.at(0)); } },
      { "nil", 0, [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string { return nil(); } },
      { "id", 0, [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string { return body; } },
      { "idd", 0, [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string { return idd(body); } },
      { "if", 1, [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string { return ifChoice(body, parameters.at(0)); } },
      { "at", 1, [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string { return element(body, parameters.at(0)); } },
      { "substr", 2, [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string { return substring(body, parameters.at(0), parameters.at(1)); } },
      { "split", 1, [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string { 
         auto spl = StaticHelper::split(body, parameters[0]);
         std::string res{};
         for (const auto& s : spl) {
            res += BEGIN_TAG_IN_SEQUENCE + s + END_TAG_IN_SEQUENCE;
         }
         return res;
      } },
      { "newMethod", 2, [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string { return newMethod(body, parameters.at(0), parameters.at(1)); } },
      { "newMethod", 3, [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string { return newMethodD(body, parameters.at(0), parameters.at(1), parameters.at(2)); } },
      { "strsize", 0, [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string { return std::to_string(body.length()); } },
      { "seqlength", 0, [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string { return std::to_string(processSequence(body).size()); }},
      { "firstLetterCapital", 0, [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string { return StaticHelper::firstLetterCapital(body); } },
      { "toUpperCamelCase", 0, [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string {  return StaticHelper::toUpperCamelCase(body); } },
      { "toUpperCase", 0, [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string { return StaticHelper::toUpperCase(body); } },
      { "toLowerCase", 0, [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string { return StaticHelper::toLowerCase(body); } },
      { "removeLastFileExtension", 0, [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string { return StaticHelper::removeLastFileExtension(body); } },
      { "removeLastFileExtension", 1, [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string { return StaticHelper::removeLastFileExtension(body, parameters.at(0)); } },
      { "getLastFileExtension", 0, [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string { return StaticHelper::getLastFileExtension(body); } },
      { "getLastFileExtension", 1, [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string { return StaticHelper::getLastFileExtension(body, parameters.at(0)); } },
      { "substrComplement", 2, [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string { return StaticHelper::substrComplement(body, stringToFloat(parameters.at(0)), stringToFloat(parameters.at(1))); } },
      { "startsWithUppercase", 0, [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string { return fromBooltoString(StaticHelper::startsWithUppercase(body)); } },
      { "stringStartsWith", 1, [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string { return fromBooltoString(StaticHelper::stringStartsWith(body, parameters.at(0))); } },
      { "stringStartsWith", 2, [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string { return fromBooltoString(StaticHelper::stringStartsWith(body, parameters.at(0), stringToFloat(parameters.at(0)))); } },
      { "stringEndsWith", 1, [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string { return fromBooltoString(StaticHelper::stringEndsWith(body, parameters.at(0))); } },
      { "stringContains", 1, [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string { return fromBooltoString(StaticHelper::stringContains(body, parameters.at(0))); } },
      { "shortenToMaxSize", 1, [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string { return StaticHelper::shortenToMaxSize(body, stringToFloat(parameters.at(0))); } },
      { "shortenToMaxSize", 2, [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string { return StaticHelper::shortenToMaxSize(body, stringToFloat(parameters.at(0)), StaticHelper::isBooleanTrue(parameters.at(1))); } },
      { "shortenToMaxSize", 3, [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string { return StaticHelper::shortenToMaxSize(body, stringToFloat(parameters.at(0)), StaticHelper::isBooleanTrue(parameters.at(1)), stringToFloat(parameters.at(2))); } },
      { "shortenToMaxSize", 4, [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string { return StaticHelper::shortenToMaxSize(body, stringToFloat(parameters.at(0)), StaticHelper::isBooleanTrue(parameters.at(1)), stringToFloat(parameters.at(2)), stringToFloat(parameters.at(3))); } },
      { "shortenInTheMiddle", 1, [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string { return StaticHelper::shortenInTheMiddle(body, stringToFloat(parameters.at(0))); } },
      { "shortenInTheMiddle", 2, [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string { return StaticHelper::shortenInTheMiddle(body, stringToFloat(parameters.at(0)), stringToFloat(parameters.at(1))); } },
      { "absPath", 0, [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string { return StaticHelper::absPath(body); } },
      { "zeroPaddedNumStr", 1, [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string { return StaticHelper::zeroPaddedNumStr(stringToFloat(body), stringToFloat(parameters.at(0))); } },
      { "removeWhiteSpace", 0, [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string { return StaticHelper::removeWhiteSpace(body); } },
      { "isEmptyExceptWhiteSpaces", 0, [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string { return fromBooltoString(StaticHelper::isEmptyExceptWhiteSpaces(body)); } },
      { "removeMultiLineComments", 0, [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string { return StaticHelper::removeMultiLineComments(body); } },
      { "removeMultiLineComments", 2, [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string { return StaticHelper::removeMultiLineComments(body, parameters.at(0), parameters.at(1)); } },
      { "removeSingleLineComments", 0, [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string { return StaticHelper::removeSingleLineComments(body); } },
      { "removeSingleLineComments", 1, [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string { return StaticHelper::removeSingleLineComments(body, parameters.at(0)); } },
      { "removeComments", 0, [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string { return StaticHelper::removeComments(body); } },
      { "removePartsOutsideOf", 2, [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string { return StaticHelper::removePartsOutsideOf(body, parameters.at(0), parameters.at(1)); } },
      { "removeBlankLines", 0, [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string { return StaticHelper::removeBlankLines(body); } },
      { "wrap", 1, [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string { return StaticHelper::wrap(body, stringToFloat(parameters.at(0))); } },
      { "openWithOS", 0, [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string { StaticHelper::openWithOS(body, this); return ""; } },
      { "replaceAll", 2, [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string { return StaticHelper::replaceAll(body, parameters.at(0), parameters.at(1)); } },
      { "replaceAllCounting", 1, [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string { return StaticHelper::replaceAllCounting(body, parameters.at(0)); } },
      { "replaceAllCounting", 2, [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string { return StaticHelper::replaceAllCounting(body, parameters.at(0), std::stof(parameters.at(1))); } },
      { "replaceAllRegex", 2, [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string { return StaticHelper::replaceAllRegex(body, parameters.at(0), parameters.at(1)); } },
      { "isParsableAsFloat", 0, [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string { return fromBooltoString(StaticHelper::isParsableAsFloat(body)); } },
      { "isParsableAsInt", 0, [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string { return fromBooltoString(StaticHelper::isParsableAsInt(body)); } },
      { "isAlpha", 0, [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string { return fromBooltoString(StaticHelper::isAlpha(body)); } },
      { "isAlphaNumeric", 0, [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string { return fromBooltoString(StaticHelper::isAlphaNumeric(body)); } },
      { "isAlphaNumericOrUnderscore", 0, [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string { return fromBooltoString(StaticHelper::isAlphaNumericOrUnderscore(body)); } },
      { "isAlphaNumericOrUnderscoreOrColonOrComma", 0, [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string { return fromBooltoString(StaticHelper::isAlphaNumericOrUnderscoreOrColonOrComma(body)); } },
      { "isValidVfmVarName", 0, [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string { return fromBooltoString(StaticHelper::isValidVfmVarName(body)); } },
      { "stringConformsTo", 1, [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string { return fromBooltoString(StaticHelper::stringConformsTo(body, std::regex(parameters.at(0)))); } },
      { "getFileNameFromPath", 0, [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string { return StaticHelper::getFileNameFromPath(body); } },
      { "getFileNameFromPathWithoutExt", 0, [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string { return StaticHelper::getFileNameFromPathWithoutExt(body); } },
      { "removeFileNameFromPath", 0, [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string { return StaticHelper::removeFileNameFromPath(body); } },
      { "readFile", 0, [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string { return StaticHelper::readFile(body); } },
      { "fromPascalToCamelCase", 0, [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string { return StaticHelper::fromPascalToCamelCase(body); } },
      { "replaceSpecialCharsHTML_G", 0, [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string { return StaticHelper::replaceSpecialCharsHTML_G(body); } },
      { "replaceMathHTML", 0, [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string { return StaticHelper::replaceMathHTML(body); } },
      { "makeVarNameNuSMVReady", 0, [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string { return StaticHelper::makeVarNameNuSMVReady(body); } },
      { "safeString", 0, [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string { return StaticHelper::safeString(body); } },
      { "fromSafeString", 0, [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string { return StaticHelper::fromSafeString(body); } },
      { "levensteinDistance", 1, [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string { return std::to_string(StaticHelper::levensteinDistance(body, parameters.at(0))); } },
      { "executeSystemCommand", 0, [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string { return std::to_string(StaticHelper::executeSystemCommand(body)); } },
      { "exec", 0, [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string { return StaticHelper::exec(body); } },
      { "writeTextToFile", 1, [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string { StaticHelper::writeTextToFile(body, parameters.at(0)); return body; } },
      { "writeTextToFile", 2, [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string { StaticHelper::writeTextToFile(body, parameters.at(0), StaticHelper::isBooleanTrue(parameters.at(1))); return body; } },
      { "timestamp", 0, [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string { return StaticHelper::timeStamp(); } },
      { "morty", 0, [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string { auto rep = body; return StaticHelper::replaceAll(rep + MORTY_ASCII_ART, "\n", "\n" + rep); } },
      { "rick", 0, [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string { return body + RICK; } },
      { "mypath", 0, [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string { return getMyPath(); } },
      { "setScriptVar", 1, [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string { 
         return setScriptVar(body, parameters[0], "");
      } },
      { "setScriptVar", 2, [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string { 
         return setScriptVar(body, parameters[0], parameters[1]);
      } },
      { "scriptVar", 0, [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string { 
         std::string varname{ body };

         if (!getScriptData().list_data_.count(varname)) {
            std::string error{ "Variable '" + varname + "' has not been declared." };
            addError(error);
            return "#" + error + "#";
         }

         return getScriptData().list_data_.at(varname).at(0);
      } },
      { "include", 0, [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string {
         return includeMe(body);
      } },
      { "Include", 0, [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string {
         return includeMe(body);
      } },
      { "vfm_variable_declared", 0, [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string { return fromBooltoString(getData()->isVarDeclared(body)); } },
      { "vfm_variable_undeclared", 0, [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string { return fromBooltoString(!getData()->isVarDeclared(body)); } },
      { "sethard", 1, [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string { return sethard(body, parameters.at(0)); } },
      arith01f,
      arith02f,
      arith03f,
      arith04f,
      arith05f,
      arith06f,
      arith07f,
      arith08f,
      arith09f,
      arith10f,
      arith11f,
      arith12f,
      arith01i,
      arith02i,
      arith03i,
      arith04i,
      arith05i,
      arith06i,
      arith07i,
      arith08i,
      arith09i,
      arith10i,
      arith11i,
      arith12i,
      { "not", 0, [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string { return exnot(body); } },
      { "space", 0, [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string { return space(); } },
      { "arclengthCubicBezierFromStreetTopology", 4, [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string { return arclengthCubicBezierFromStreetTopology(body, parameters.at(0), parameters.at(1), parameters.at(2), parameters.at(3)); } },
      { "PIDs", 0, [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string { 
         const auto pids = Process().getPIDs(body);
         std::string pids_str{};
         for (const auto& pid : pids) pids_str += std::to_string(pid) + ",";
         return pids_str;
      } },
      { "KILLPIDs", 0, [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string { 
         std::string res{};
         for (const auto& pid : StaticHelper::split(body, ",")) {
            if (!pid.empty()) res += std::to_string(Process().killByPID(std::stoi(pid)));
         }
         return res;
      } },
      { "killAfter", 1, [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string {
         // Example: @{nuXmv}@.killAfter[30].Detach
         const auto pids = Process().getPIDs(body);
         const int wait_seconds{ (int)std::stof(parameters.at(0)) };
         int num{};

         for (const auto& pid : pids) {
            long long running_time{ Process().getProcessRunningDurationSeconds(pid) };

            if (running_time >= wait_seconds) {
               if (Process().killByPID(pid)) {
                  num++;
               }
            }
         }
         
         std::string message{ "\n" + StaticHelper::timeStamp() + ": " };
         if (num > 0) {
            message += std::to_string(num) + " PIDs based on '" + body + "' have been killed after '" + parameters[0] + "' seconds of runtime.";
         } else {
            message += "No '" + body + "'-based PID running long enough to be killed.";
         }
         
         addNote(message);
         std::this_thread::sleep_for(std::chrono::milliseconds(3000));
         return "@{" + body + "}@.killAfter[" + parameters[0] + "]" + message;
      } },
      { "METHODs", 0, [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string { 
         std::string s{};

         for (const auto& method_description : METHODS) {
            std::string uncachable{ isCachableMethod(method_description.method_name_) ? "" : " <uncachable>" };
            s += method_description.method_name_ + "[" + std::to_string(method_description.par_num_) + "] 'native'" + uncachable + "\n";
         }

         for (const auto& dynamic_method : getScriptData().inscriptMethodDefinitions) {
            std::string method_name{ dynamic_method.first };
            std::string definition{ dynamic_method.second };
            int par_num{ getScriptData().inscriptMethodParNums.at(method_name) };
            std::string pattern{ getScriptData().inscriptMethodParPatterns.at(method_name) };
            std::string uncachable{ isCachableMethod(method_name) ? "" : " <uncachable>" };

            // Avoid self-evaluation.
            definition = StaticHelper::replaceAll(StaticHelper::replaceAll(definition, INSCR_BEG_TAG, "{{"), INSCR_END_TAG, "}}");

            s += method_name + " (" + std::to_string(par_num) + " " + pattern + ") '" + definition + "'" + uncachable + "\n";
         }

         return body + s;
      } },
      { "vfmheap", 0, [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string { return getData()->toStringHeap(); } },
      { "vfmdata", 0, [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string { return getData()->toString(); } },
      { "vfmfunc", 0, [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string { 
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
      } },
      { "printHeap", 0, [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string { return getData()->printHeap(body); } },
      { "stringToHeap", 1, [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string {
         std::string res{ parameters[0] + " set to refer to '" + body + "' in heap." };

         if (parameters[0] == MY_PATH_VARNAME) { // Setting my path, special case.
            if (StaticHelper::existsFileSafe(body)) {
               res = parameters[0] + " is set to the following path '" + StaticHelper::absPath(body) + "' which is existing on the file system.";
            }
            else {
               res = parameters[0] + " has NOT been set to '" + body + "' because the path does not exist on the file system. It still points to '" + getMyPath() + "'.";
               addError(res);
               return res;
            }
         }

         getData()->addStringToDataPack(body, parameters[0]);
         return res;
      } },
      { "listElement", 1, [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string { return listElement(body, parameters); } },
      { "clearList", 0, [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string { 
         if (!getScriptData().list_data_.count(body)) {
            getScriptData().list_data_[body] = {};
         }

         getScriptData().list_data_[body].clear();

         return "";
      } },
      { "asArray", 0, [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string { 
         std::string list_str{};

         for (const auto& el : getScriptData().list_data_[body]) {
            list_str += BEGIN_TAG_IN_SEQUENCE + el + END_TAG_IN_SEQUENCE;
         }

         return StaticHelper::trimAndReturn(list_str);
      } },
      { "printList", 0, [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string { 
         if (getScriptData().list_data_.count(body)) {
            std::string list_str{};

            for (const auto& el : getScriptData().list_data_[body]) {
               list_str += el + "\n";
            }

            return StaticHelper::trimAndReturn(list_str);
         }
         else {
            std::string error_str{ "#ERROR<list '" + body + "' not found by printList method>" };
            addError(error_str);
            return error_str;
         }
      } },
      { "printLists", 0, [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string { 
            std::string list_str{};

            for (const auto& el : getScriptData().list_data_) {
               list_str += el.first + ",";
            }

            return StaticHelper::trimAndReturn(list_str);
      } },
      { "pushBack", 1, [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string { 
         if (!getScriptData().list_data_.count(body)) {
            getScriptData().list_data_[body] = {};
         }
         getScriptData().list_data_[body].push_back(parameters[0]);
         return "";
      } },
      { "pushBack", 1, [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string { 
         if (!getScriptData().list_data_.count(body)) {
            getScriptData().list_data_[body] = {};
         }
         getScriptData().list_data_[body].push_back(parameters[0]);
         return "";
      } },
      { "createRoadGraph", 1, [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string { return createRoadGraph(body, parameters[0]); } },
      { "storeRoadGraph", 1, [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string { return storeRoadGraph(body, parameters[0]); } },
      { "connectRoadGraphTo", 1, [this](const std::string& body, const std::vector<std::string>& parameters) -> std::string { return connectRoadGraphTo(body, parameters[0]); } },
   };
};

} // macro
} // vfm
