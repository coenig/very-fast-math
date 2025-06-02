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

static const std::string THIS_NAME = "this";
static const std::string PREPROCESSOR_FIELD_NAME = "prep";
static const std::string VARIABLE_DELIMITER = "=";
static const char END_VALUE = ';';
static const std::string PREAMBLE_FOR_NON_SCRIPT_METHODS = "$$INFO-METHOD-RETURN-VALUE$$\n";
static const std::string INSCRIPT_STANDARD_PARAMETER_PATTERN = "#n#";

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

class ScriptTree : public Failable, public std::enable_shared_from_this<ScriptTree> {
public:
   ScriptTree();
   ScriptTree(std::shared_ptr<ScriptTree> father);

   std::string getIdName() const {
      return idName_;
   }

   void activateDebug() {
      debug_ = true;
   }

   std::vector<std::shared_ptr<ScriptTree>> getChildren() const {
      return children_;
   }

   std::shared_ptr<ScriptTree> getFather() const {
      return father_.lock();
   }

   std::shared_ptr<int> getBeg() const {
      return beg_;
   }

   std::shared_ptr<int> getEnd() const {
      return end_;
   }

   void addScript(const std::string& scriptIdentifier, const std::string& script, const int beg, const int end) {
      addScriptPrivate(scriptIdentifier, script, beg, end);
   }

   /// Adjusts all begins and ends starting at <code>oldEnd</code> or behind by
   /// <code>delta</code>. Note that sub-script nodes may be shifted to the end
   /// of the area of the higher-level script if its area shrinks from right to
   /// left, as no information is stored which parts of the area (which the
   /// sub-script area is part of) have been changed.</BR>
   /// </BR>
   /// For example: In <code>@{d@{x}@d}@</code>,  <code>x</code> will eventually
   /// end up being being shifted to the end of the <code>dxd</code> area,
   /// resulting in <code>x</code> being assigned the position <code>2-2</code>
   /// (instead of <code>2-2</code>).</BR>
   /// </BR>
   /// Therefore, the script tree cannot replace the script, but it should only
   /// be used for looking up the inscript preprocessor hierarchy defined by the
   /// script.
   ///
   ///
   /// @param oldEnd  The index, left of which no adjustments are necessary
   ///                (except if an area shrinks over some of its sub-areas.
   /// @param delta   The delta value to add to all parts right of
   ///                <code>oldEnd</code>.
   void shiftEnd(const int oldEnd, const int delta) {
      shiftEnd(oldEnd, delta, (std::numeric_limits<int>::min)(), (std::numeric_limits<int>::max)());

      if (debug_) {
         //storeAsPDF();
      }
   }

private:
   int count_{0};
   std::string EXCEPTION_EXPLANATION{};
   std::string END_DIGRAPH = "}";
   std::string BEGIN_DIGRAPH = "digraph G {";
   std::string GRAPHVIZ_DOT_PREAMBLE = "dot:";

    /// @param oldEnd  The index, left of which no adjustments are necessary 
    ///                (except if an area shrinks over some of its sub-areas.
    /// @param delta   The delta value to add to all parts right of 
    ///                <code>oldEnd</code>.
    /// @param min     The minimum given from the higher-level father.
    /// @param max     The maximum given from the higher-level father.
    void shiftEnd(const int oldEnd, const int delta, const int min, const int max) {
        if (delta == 0) {
            return;
        }
        
        int myLength = *end_ - *beg_;
        
        if (*beg_ >= oldEnd) {
            *beg_ += delta;
        }
        
        if (*end_ >= oldEnd) {
            *end_ += delta;
        }
        
        if (*end_ > max) {
            *end_ = max;
            *beg_ = *end_ - myLength; 
        }
        
        if (*beg_ < min) {
            *beg_ = min;
        }
        
        if (*beg_ > *end_) {
            *beg_ = *end_ + 1;
        }
        
        for (const auto s : children_) {
            s->shiftEnd(oldEnd, delta, *beg_, *end_);
        }
    }

   std::string createDOTExceptionNode(const std::string& exceptionDescription) 
   {
      return "exceptionNode" + std::to_string(count_++) + " [label=\"" + exceptionDescription + "\" shape=\"rectangle\" color=\"red\"];";
   }

   const std::string PREP_TREE_INFO = createDOTExceptionNode("Error in prep tree");

   std::string errorNodes(const int beg, const int end) 
   {
      return PREP_TREE_INFO + "dgsezusi [label=\"" + std::to_string(beg) + "/" + std::to_string(end) + " ??\" color=\"red\"];";
   }

   std::string gvInner() {
      std::string s = "";
      std::string processedIDName = idName_.empty() ? "this" : idName_;
      std::string safeNameFrom = StaticHelper::safeString(processedIDName);
      std::string add = father_.lock() 
         ? "='" + StaticHelper::replaceSpecialCharsHTML_G(script_) + "'" 
         : " [" + std::to_string(*beg_) + "/" + std::to_string(*end_) + "]";

      s += safeNameFrom + " [label=<" + "" + (processedIDName + add) + ">];\n";

      for (int i = 0; i < children_.size(); i++) {
         auto child = children_.at(i);
         std::string safeNameTo = StaticHelper::safeString(child->idName_);
         s += safeNameFrom + "->" + safeNameTo + " [label=\"" + std::to_string(*child->beg_) + "/" + std::to_string(*child->end_) + "\"];\n";
         s += child->gvInner();
      }

      return s;
   }

   std::string getGraphvizTree(const std::shared_ptr<std::string> error) {
      std::string s = GRAPHVIZ_DOT_PREAMBLE + "\n"
         + BEGIN_DIGRAPH
         + (error ? *error : std::string(""))
         + gvInner()
         + END_DIGRAPH;

      return s;
   }

   std::shared_ptr<ScriptTree> findRoot() {
      if (!father_.lock()) {
         return shared_from_this();
      }

      return father_.lock()->findRoot();
   }

   std::vector<std::shared_ptr<ScriptTree>> traverseDepthFirst() {
      std::vector<std::shared_ptr<ScriptTree>> list;
      return traverseDepthFirst(list);
   }

   std::vector<std::shared_ptr<ScriptTree>> traverseDepthFirst(std::vector<std::shared_ptr<ScriptTree>> soFar) {
      for (const auto& child : children_) {
         child->traverseDepthFirst(soFar);
      }

      soFar.push_back(shared_from_this());
      return soFar;
   }

   void addScriptPrivate(const std::string& scriptIdentifier, const std::string& script, const int beg, const int end) 
   {
      if (!beg_ || !end_) {
         beg_ = std::make_shared<int>(beg);
         end_ = std::make_shared<int>(end);
      }

      if (*beg_ == beg && *end_ == end) {
         idName_ = scriptIdentifier;
         script_ = StaticHelper::replaceAll(script, "\n", " ");
      }
      else if (*beg_ > beg || *end_ < end) {
         std::string error_nodes = errorNodes(beg, end);
         EXCEPTION_EXPLANATION = getGraphvizTree(std::make_shared<std::string>(error_nodes));
         addError("Node range [" + std::to_string(*beg_) + "-" + std::to_string(*end_) + "] too narrow for requested range [" + std::to_string(beg) + "-" + std::to_string(end) + "].");
      }
      else {
         int i;
         for (i = 0; i < children_.size(); i++) {
            std::shared_ptr<ScriptTree> n = children_.at(i);
            if (*n->beg_ <= beg && *n->end_ >= end) { // Go down in child.
               n->addScript(scriptIdentifier, script, beg, end);
               return;
            }
            else if (*n->beg_ > end) { // Fit in gap.
               if (i == 0 || *children_.at(i - 1)->end_ < beg) {
                  break;
               }
            }
            else if (beg <= *n->beg_ && beg <= *n->end_ && end >= *n->beg_ && end >= *n->end_) { // New node is father of (several) child(ren).
               int storeBeg = i;
               int storeEnd;
               while (i < children_.size() && end >= *children_.at(i)->end_) {
                  i++;
               }
               storeEnd = i - 1;

               std::shared_ptr<ScriptTree> nNew = std::make_shared<ScriptTree>(shared_from_this());
               nNew->addScript(scriptIdentifier, script, beg, end);

               // Remove from direct children.
               for (int j = storeBeg; j <= storeEnd; j++) {
                  nNew->children_.push_back(children_.at(storeBeg));
                  children_.erase(children_.begin() + storeBeg);
               }

               children_.insert(children_.begin() + storeBeg, nNew);

               return;
            }
         }

         if (i > 0 && *children_.at(i - 1)->end_ >= beg) {
            std::string error_nodes = errorNodes(beg, end);
            EXCEPTION_EXPLANATION = getGraphvizTree(std::make_shared<std::string>(error_nodes));

            //StaticHelper::createImageFromGraphvizDot(EXCEPTION_EXPLANATION.substr(4), "error.dot");

            addError("Cannot insert '" + script + "' interval [" + std::to_string(beg) + "-" + std::to_string(end) + "] into node tree " + getGraphvizTree(nullptr) + ".");
         }

         std::shared_ptr<ScriptTree> n = std::make_shared<ScriptTree>(shared_from_this());
         n->addScript(scriptIdentifier, script, beg, end);
         children_.insert(children_.begin() + i, n);
      }

      if (debug_) {
         //storeAsPDF();
      }
   }

   std::vector<std::shared_ptr<ScriptTree>> children_{};
   std::weak_ptr<ScriptTree> father_{};
   std::shared_ptr<int> beg_{}, end_{};
   std::string idName_{};
   std::string script_{};
   bool debug_{ false };
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

   virtual std::shared_ptr<Script> createThisObject(const std::shared_ptr<Script> repThis, const std::string repScrThis, const std::shared_ptr<Script> father) = 0;
   virtual void createInstanceFromScript(const std::string& code, std::shared_ptr<Script> father) = 0;
   std::shared_ptr<Script> copy() const;
   virtual std::shared_ptr<DummyRepresentable> toDummyIfApplicable();

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
   std::shared_ptr<Script> repfactory_instanceFromScript(const std::string& script, const std::shared_ptr<Script> father);

   bool isBooleanTrue(const std::string& bool_str)
   {
      return StaticHelper::isBooleanTrue(bool_str);
   }

   static std::string processScript(
      const std::string& text,
      const std::shared_ptr<DataPack> data = nullptr, 
      const std::shared_ptr<FormulaParser> parser = nullptr,
      const std::shared_ptr<Failable> father_failable = nullptr);

   static std::set<std::string> VARIABLES_MAYBE; // TODO: Made static for simplicity, but should be part of ind. script to allow scoped variables.

protected:
   static std::map<std::string, std::string> knownPreprocessors; // TODO: no static! Should belong to some base class belonging to a single expansion "session".
   static std::map<std::string, std::vector<std::string>> list_data_; // TODO: no static! Should belong to some base class belonging to a single expansion "session".

   /// Creates a placeholder for an inscript method call. If the result is a
   /// plain text (as opposed to an image), the result itself is the placeholder.
   /// If it's an image, the empty string "" is returned. Representables
   /// that allow images as inscript preprocessors have to override this method
   /// and "do something" when the empty string occurs. (If <code>
   /// allowRegularMethodCalls</code> is <code>false</code>, an exception is
   /// thrown. If the method call contains a variable name in the beginning,
   /// <code>null</code> is returned.
   ///
   /// @param filename             The name of the preprocessor.
   /// @param preprocessorScript   The script of the preprocessor.
   /// @param scale                The scale of the preprocessor, in case it
   ///                             creates an image.
   /// @param allowRegularScripts  In the declarations part regular scripts should
   ///                             not be expanded, so the script itself is
   ///                             returned in this case.
   ///
   /// @return  A placeholder, the plain-text result or <code>null</code> if
   ///          a regular method call in the end requires the handling of
   ///          a specific rep such as LaTeX.
   std::string placeholderForInscript(
      const std::string& filename, // Don't remove this - LaTeX needs it.
      const std::string& preprocessorScript,
      const double scale,
      const bool allowRegularScripts);

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

private:

   /// If you try to avoid assigning the parameter,
   /// be careful, it's not as simple as it seems.
   static std::string createDummyrep(
      const std::string& processed,
      RepresentableAsPDF repToProcess,
      RepresentableAsPDF father);

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

   std::vector<std::string> getParametersFor(const std::vector<std::string>& rawPars, const RepresentableAsPDF this_rep, const RepresentableAsPDF father);
   RepresentableAsPDF applyMethodChain(const RepresentableAsPDF original, const std::vector<std::string>& methodsToApply);
   std::vector<std::string> getMethodParameters(const std::string& conversionTag);

   /// @param rep              The representable to apply the method to.
   /// @param methodSignature  The method internal name + parameters.
   ///
   /// @return  A new representable with the method applied.Note that rep
   /// can be changed in the process.
   RepresentableAsPDF applyMethod(const RepresentableAsPDF rep, const std::string & methodSignature);

   virtual std::string applyMethodString(const std::string& method_name, const std::vector<std::string>& parameters) = 0;

   /// Evaluates a single chain of conversion method applications to a script.
   ///
   /// @param repScrThis  The representable script to be considered "this". Can be
   ///                    {@code null} if chain does not begin with "this".
   /// @param chain       The method chain to apply as: "*script*.m1.m2.m3.m4..."
   ///                    where *script* is a script enclosed in @{ ... }@
   ///                    or "this", and m1, m2, ... are method signatures.
   /// @param father      The rep {@code this} is a sub-script of.
   ///
   /// @return  A representable with the methods applied. Can return <code>null
   ///          </code> when the chain is not valid (e.g. when starting with a
   ///          variable).
   std::shared_ptr<Script> evaluateChain(const std::string& repScrThis, const std::string& chain, std::shared_ptr<Script> father);

   std::string findVarName(const std::string script, int& begin) const;

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
   bool extractInscriptProcessors();

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

    /// Finds all potential variables, i.e., all substrings that stand left
    /// of a "=", start with an alphabetic character and contain only
    /// alphanumeric characters.
   void findAllVariables();

   std::string removePreprocessors(const std::string& script);

   /// Returns the next position which is after the complete method chain of
   /// the preprocessor.
   ///
   /// @param partAfter  The part AFTER the preprocessor base script.
   /// @return  The next position outside the preprocessor's method chain.
   int getNextNonInscriptPosition(const std::string& partAfter);

   /// Adds a preprocessor to this representable.
   ///
   /// @param preprocessorScript  The preprocessor code containing a filename
   ///                            to store the PDF in and the actual
   ///                            preprocessor code.
   /// @param hidden        Iff the preprocessor is hidden. For Latex, e.g.,
   ///                      preprocessors are used "inscript", hidden from
   ///                      the user.
   void addPreprocessor(const std::string& preprocessorScript);

   /// Adds a preprocessor to this representable. Note that the preprocessor
   /// code has to be well-formatted - as opposed to the other method with
   /// the same name which first cleans up the preprocessor code.
   ///
   /// @param preprocessor  Plain and cleaned preprocessor code.
   /// @param filename      The filename to store the preprocessor in.
   /// @param hidden        Iff the preprocessor is hidden. For Latex, e.g.,
   ///                      preprocessors are used "inscript", hidden from
   ///                      the user.
   void addPreprocessor(const std::string& preprocessor, const std::string& filename, const int indexOfMethodsPartBegin);

   bool isVariable(const std::string& preprocessorScript);

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

   /// Replaces a variable name, if any, with the correct script at the
   /// beginning of <code>chain</code>.
   ///
   /// @param chain  The chain to process.
   ///
   /// @return  The processed chain - can be equal to received chain.
   std::string processChain(const std::string& chain);

   std::string raiseAndGetQualifiedIdentifierName(const std::string& identifierName);
   std::string getQualifiedIdentifierName(const std::string& identifierName);
   std::string getUnqualifiedName(const std::string& qualifiedName);
   std::map<std::string, std::string> getPreprocessors();

   /// Replaces a possible variable name at the beginning with the actual
   /// preprocessor code.
   void processPreprocessor(const std::string& rawName);

   /// Replaces all variable names with the actual code to process.
   /// For example, if x1=this.min, y1=x1.sim becomes y1=this.min.sim.</BR>
   /// </BR>
   /// Note: Pretty sure, this method can be done more efficiently. However,
   /// it's really micro-management - better focus on other efficiency issues.
   void processPreprocessors();

   void processPendingVars();
   std::string replaceIdentAtBeginningOfChain(const std::string& chain, const std::string& k, const std::string& varVal, int mbeg);

   /// If the script is embedded in plain text tags, replace all symbols with
   /// placeholders, but leave plain-text tags for later.
   ///
   /// @param  script  The script to check.
   /// @return  The possibly processed script.
   std::string checkForPlainTextTags(const std::string& script);
   std::vector<std::string> getAllQualifiedIdentifiers(const std::string& ident);
   void removeChainsContainingIdentifier(const std::string& ident);
   void removeChainsContainingQualifiedIdentifier(const std::string& qualIdent);
   std::string getPreprocessorFromThis(const std::string& identifier);

   /// Stores all symbols that are somehow restricted as they serve a special
   /// purpose. Caution: Don't count on this list to be really completely
   /// exhaustive. I.e., look at the code now and then for pending changes.
   std::set<std::string> SPECIAL_SYMBOLS{};
   std::map<std::string, std::string> PLACEHOLDER_MAPPING{};
   std::map<std::string, std::string> PLACEHOLDER_INVERSE_MAPPING{};
   std::map<std::string, std::string> alltimePreprocessors{};
   std::map<std::string, int> identCounts{};

   std::string rawScript{};
   std::string processedScript{};
   int starsIgnored{};

   std::map<std::string, int> methodPartBegins{};
  
   bool recalculatePreprocessors{ true };
   //long allOfIt{};
   int count{ 0 };
   static std::map<std::string, std::shared_ptr<Script>> knownChains; // TODO: no static! Should belong to some base class belonging to a single expansion "session".
   static std::map<std::string, std::shared_ptr<Script>> knownReps; // TODO: no static! Should belong to some base class belonging to a single expansion "session".

   std::shared_ptr<DataPack> vfm_data_{};
   std::shared_ptr<FormulaParser> vfm_parser_{};
};

class DummyRepresentable : public Script
{
public:
   const std::string BEGIN_TAG_IN_SEQUENCE = "@(";
   const std::string END_TAG_IN_SEQUENCE = ")@";

   //DummyRepresentable(const std::shared_ptr<Script> repToEmbed);
   inline DummyRepresentable(
      const std::shared_ptr<DataPack> data, 
      const std::shared_ptr<FormulaParser> parser)
      : Script(data, parser) {}

   std::shared_ptr<DummyRepresentable> toDummyIfApplicable() override;

   std::shared_ptr<Script> createThisObject(const std::shared_ptr<Script> repThis, const std::string repScrThis, const std::shared_ptr<Script> father) override;

   /// Looks for a sequence of several chunks of
   /// <code>@(...)@@(...)@@(...)@</code> in the code and puts them into
   /// <code>scriptSequence</code>.
   ///
   /// @param code  The raw code to process.
   void processSequence(const std::string& code, std::vector<std::string>& script_sequence) {
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

   void createInstanceFromScript(const std::string& code, std::shared_ptr<Script> father) override
   {
      setRawScript(StaticHelper::replaceAll(code, PREAMBLE_FOR_NON_SCRIPT_METHODS, "")); // Because applyScriptsAndPreprocessors is not called by Dummy.
      scriptSequence_.clear();
      processSequence(code, scriptSequence_);
   }

   std::string createScriptFromInstance() {
      std::string result = "";

      for (const auto& s : scriptSequence_) {
         result += BEGIN_TAG_IN_SEQUENCE + s + END_TAG_IN_SEQUENCE;
      }

      return result;
   }

   std::shared_ptr<Script> getRepresentableAsPDF() {
      return shared_from_this();
   }

   std::string evalItAll(const std::string& n1Str, const std::string& n2Str, const std::function<float(float n1, float n2)> eval) {
      if (!StaticHelper::isParsableAsFloat(n1Str) || !StaticHelper::isParsableAsFloat(n2Str)) {
         addError("Operand '" + n1Str + "' and/or '" + n2Str + "' cannot be parsed to float/int.");
         return "0";
      }

      float n1 = std::stof(n1Str);
      float n2 = std::stof(n2Str);
      return std::to_string(eval(n1, n2));
   }

   /// Retrieves the raw script, but without any <code>@{</code> or <code>@{</code>.
   /// Can be used to get the constant <code>2</code> from raw script
   /// <code>@{2}@</code>. But use carefully, since it actually removes ALL
   /// tags.
   ///
   /// @return  The raw script without any inscript tags.
   std::string getTagFreeRawScript() {
      return StaticHelper::replaceManyTimes(getRawScript(), { INSCR_BEG_TAG, INSCR_END_TAG }, { "", "" });
   }

   std::string sethard(const std::string& value) { knownPreprocessors[getTagFreeRawScript()] = value; return value; }
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
   std::string exnot() { return std::to_string(!isBooleanTrue(getTagFreeRawScript()));}

   std::string space() { return " "; }
   
   std::string arclengthCubicBezierFromStreetTopology(const std::string& lane, const std::string& angle, const std::string& distance, const std::string& num_lanes);

   std::string forloop(const std::string& varname, const std::string& loop_vec)
   {
      return forloop(varname, loop_vec, "");
   }

   std::string forloop(const std::string& varname, const std::string& from_raw, const std::string& to_raw) 
   {
      if (from_raw.empty() || StaticHelper::stringStartsWith(from_raw, BEGIN_TAG_IN_SEQUENCE)) { // Regular sequence @()@@()@...
         std::vector<std::string> loop_vec{};
         processSequence(from_raw, loop_vec);
         return forloop(varname, loop_vec, to_raw);
      }
      else if (VARIABLES_MAYBE.count(from_raw)) { // Assume empty sequence due to unresolved variable. TODO: Is this always correct?
         return forloop(varname, std::vector<std::string>{}, to_raw);
      }

      return forloop(varname, from_raw, to_raw, "1");
   }

   std::string forloop(const std::string& varname, const std::string& from_raw, const std::string& to_raw, const std::string& step_raw) 
   {
      return forloop(varname, from_raw, to_raw, step_raw, "");
   }

   std::string forloop(const std::string& varname, const std::string& from_raw, const std::string& to_raw, const std::string& step_raw, const std::string& inner_separator) 
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

   std::string forloop(const std::string& varname, const std::vector<std::string>& loop_vec, const std::string& inner_separator) {
      std::string loopedVal = "";
      bool first{ true };

      for (const auto& i : loop_vec) {
         std::string currVal = StaticHelper::replaceAll(getRawScript(), varname, i);
         loopedVal += (!first ? StaticHelper::replaceAll(inner_separator, varname, i) : "") + currVal;
         first = false;
      }

      return loopedVal;
   }

   std::string ifChoice(const std::string bool_str) 
   {
      bool boolVal{isBooleanTrue(bool_str)};

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

   std::string element(const std::string& num_str) 
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

   float stringToFloat(const std::string& str)
   {
      if (StaticHelper::isParsableAsFloat(str)) {
         return std::stof(str);
      }
      else {
         addError("String '" + str + "' is not parsable as float.");
         return std::numeric_limits<float>::quiet_NaN();
      }
   }

   std::string substring(const std::string& beg, const std::string& end) 
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

   std::string newMethod(const std::string& methodName, const std::string& numPars) 
   {
      return newMethodD(methodName, numPars, INSCRIPT_STANDARD_PARAMETER_PATTERN);
   }

   std::string newMethodD(const std::string& methodName, const std::string& numPars, const std::string& parameterPattern) 
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
      inscriptMethodParNums.insert({ methodName, (int) std::stof(numPars) });
      inscriptMethodParPatterns.insert({ methodName, parameterPattern });

      addNote("New method '" + methodName + "' with " 
         + std::to_string(inscriptMethodParNums.at(methodName)) + " parameters defined as '" 
         + inscriptMethodDefinitions.at(methodName) + "'. Parameter pattern: '" + parameterPattern + "'.");

      return "";
   }

   std::string executeCommand(const std::string& methodName, const std::vector<std::string>& pars) 
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

   std::string makroPattern(const int i, const std::string& pattern) 
   {
      if (StaticHelper::replaceAll(pattern, "n", "").size() != pattern.length() - 1) {
         addError("Malformed makro parameter pattern pattern '" + pattern + "' does not contain exactly one 'n'.");
      }

      std::string makroPar = StaticHelper::replaceAll(pattern, "n", std::to_string(i));
      return makroPar;
   }

   std::string applyMethodString(const std::string& method_name, const std::vector<std::string>& parameters) override;

   static std::map<std::string, std::string> inscriptMethodDefinitions; // TODO: no static! Should belong to some base class belonging to a single expansion "session".
   static std::map<std::string, int> inscriptMethodParNums;             // TODO: no static! Should belong to some base class belonging to a single expansion "session".
   static std::map<std::string, std::string> inscriptMethodParPatterns; // TODO: no static! Should belong to some base class belonging to a single expansion "session".

private:
   void invokeMethod();

   std::vector<std::string> scriptSequence_{};
};

} // macro
} // vfm