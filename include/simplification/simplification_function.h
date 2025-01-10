//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file
#pragma once
#include "failable.h"
#include "parser.h"

#include <memory>

namespace vfm {
namespace simplification {

static const std::string PLACEHOLDER_FORMULA_NAME{ "FORMULANAME" };

class SimplificationFunctionLineCustom;
class SimplificationFunctionLineDecl;
class SimplificationFunctionLineDef;
class SimplificationFunctionLineIf;
class SimplificationFunctionLineNull;
class SimplificationFunctionLinePlaceholder;
class SimplificationFunctionLineReturn;

class SimplificationFunctionLine : public std::enable_shared_from_this<SimplificationFunctionLine>, public Failable
{
public:
   SimplificationFunctionLine(const std::string& content, const std::string& comment);

   virtual std::string serializeSingleLine() const;
   virtual std::string serializeSingleLine(const int indent, const std::string& formula_name) const;
   std::string missingPredecessorNote() const;
   void applyToMeAndMyChildren(const std::function<void(const std::shared_ptr<SimplificationFunctionLine>)>& f);
   virtual void applyToMeAndMyChildren(const std::function<void(const std::shared_ptr<SimplificationFunctionLine>)>& f, bool& breakitoff);
   virtual void applyToMeAndMyParents(const std::function<void(const std::shared_ptr<SimplificationFunctionLine>)>& f, const bool exclude_me = false);
   void replace(const std::shared_ptr<SimplificationFunctionLine> line);
   
   bool isEmptyLine() const;
   bool isEmptyFromThisLineOn() const;

   enum class MarkerMode {
      never,
      always,
      automatic
   };

   std::string serializeBlock() const;
   std::string serializeBlock(const int indent, const std::string& formula_name, const MarkerMode marker_mode = MarkerMode::automatic) const;
   bool isStructurallyEqual(const std::shared_ptr<SimplificationFunctionLine> other);

   virtual std::shared_ptr<SimplificationFunctionLineIf> toIfIfApplicable() { return nullptr; }
   virtual std::shared_ptr<SimplificationFunctionLineReturn> toReturnIfApplicable() { return nullptr; }
   virtual std::shared_ptr<SimplificationFunctionLineDecl> toDeclIfApplicable() { return nullptr; }
   virtual std::shared_ptr<SimplificationFunctionLineDef> toDefIfApplicable() { return nullptr; }
   virtual std::shared_ptr<SimplificationFunctionLineCustom> toCustomIfApplicable() { return nullptr; }
   virtual std::shared_ptr<SimplificationFunctionLineNull> toNullIfApplicable() { return nullptr; }
   virtual std::shared_ptr<SimplificationFunctionLinePlaceholder> toPlaceholderIfApplicable() { return nullptr; }

   virtual std::string getContent() const;
   bool containsLineBelowIncludingThis(const std::string& line) const;
   bool containsLineAboveIncludingThis(const std::string& line) const;
   void setContent(const std::string& content);
   std::shared_ptr<SimplificationFunctionLine> getSuccessor() const;
   std::shared_ptr<SimplificationFunctionLine> getPredecessor() const;
   std::shared_ptr<SimplificationFunctionLine> getLastLine();
   std::shared_ptr<SimplificationFunctionLine> getFirstLine();
   bool checkForEqualDefinitions();
   void setSuccessor(const std::shared_ptr<SimplificationFunctionLine> successor);
   void setPredecessor(const std::shared_ptr<SimplificationFunctionLine> predecessor);
   void appendAtTheEnd(const std::shared_ptr<SimplificationFunctionLine> line, const bool replace_last_line = false, const bool insert_one_before_end = false);
   std::string getComment() const;
   void setComment(const std::string& comment);
   void setFather(const std::shared_ptr< SimplificationFunctionLine> father);
   std::shared_ptr<SimplificationFunctionLine> getFather() const;
   void makeComment(const bool do_it);
   bool isComment() const;
   std::shared_ptr<SimplificationFunctionLine> goToNextNonEmptyLine();
   std::shared_ptr<SimplificationFunctionLineIf> goToLastIfInBlock();

   void removeAllRedefinitionsOrReinitializations();
   void mergeWithOtherSimplificationFunction(const std::shared_ptr<SimplificationFunctionLine> other, const FormulaParser& p);

   std::shared_ptr<SimplificationFunctionLine> copy();

   bool checkIntegrity();
   int nodeCount();

private:
   bool make_comment_{ false };
   std::string content_{};
   std::string comment_{};
   std::shared_ptr<SimplificationFunctionLine> successor_{};
   std::shared_ptr<SimplificationFunctionLine> predecessor_{}; // Only one of predecessor_ or father_ can be non-null.
   std::weak_ptr<SimplificationFunctionLine> father_{}; // Non-null only for the FIRST line in an IF sub-statement.

   virtual std::shared_ptr<SimplificationFunctionLine> copy_core() const = 0;
};

enum class IfConditionMode
{
   equal_comparison,
   value_comparison,      // formula_0->isTermVal() && formula_0->toValueIfApplicable()->getValue() == 1.000000
   overloaded_comparison, // formula_1->getOptor() == "-" && formula_1->getTerms().size() == 2
   anyway_comparison,     // formula->getTerms().size() == 2.000000 && formula->getOptor() == formula_1->getOptor()
   free,                  // MetaRule::is_constant_and_evaluates_to_false(m_1) && MetaRule::no_check(m)
   irregular,
};

enum class IfConditionComparisonSideMode
{
   operator_name_specifier, // formula->getOptor()
   terms_name_specifier,    // formula_1->getTerms().size()
   operator_name,           // "+", "-", etc.
   number,                  // You know what a number is...
   value_check,             // formula_0->isTermVal()
   value_getter,            // formula_0->toValueIfApplicable()->getValue()
   irregular
};

class IfConditionComparisonSide
{
public:
   IfConditionComparisonSide();
   IfConditionComparisonSide(const IfConditionComparisonSideMode mode, const std::string& formula_name, const std::vector<int>& trail);
   IfConditionComparisonSide(const IfConditionComparisonSideMode mode, const float number);
   IfConditionComparisonSide(const IfConditionComparisonSideMode mode, const std::string& operator_name);

   std::string serialize() const;
   void setFormulaName(const std::string& name);
   IfConditionComparisonSideMode getMode() const;
   std::string getFormulaName() const;
   std::vector<int> getTrail() const;
   std::string getOperatorName() const;
   float getNumber() const;
   void makeLastTrailItemNative();

private:
   IfConditionComparisonSideMode mode_{ IfConditionComparisonSideMode::operator_name_specifier };
   std::string formula_name_{};
   std::vector<int> trail_{};
   float number_{};
   std::string operator_name_{};
   bool make_last_trail_item_native_{ false }; // Iff: formula_1_2_3 ==> formula_1_2->getTerms()[3]
};

class IfCondition
{
public:
   IfCondition();

   // formula_1->getOptor() == "-" && formula_1->getTerms().size() == 2
   IfCondition(const IfConditionMode mode, const std::string& formula_name, const std::vector<int>& trail, const std::string& operator_name, const float value);

   // formula->getTerms().size() == 2.000000 && formula->getOptor() == formula_1->getOptor()
   // trail1                                    trail2                 trail3
   IfCondition(const IfConditionMode mode, const std::string& formula_name, const std::vector<int>& trail1, const std::vector<int>& trail2, const std::vector<int>& trail3, const float value);

   IfCondition(const IfConditionMode mode, const std::string& formula_name, const std::vector<int>& trail, const float value);
   IfCondition(const IfConditionMode mode, const IfConditionComparisonSide& side1, const IfConditionComparisonSide& side2);
   IfCondition(const IfConditionMode mode, const std::string& free_string);

   std::string serialize() const;
   void setFormulaName(const std::string& name);
   IfConditionMode getMode() const;
   std::vector<IfConditionComparisonSide> getSides() const;
   bool isAnywayConditionFirstPart() const;             // Of type: formula->getTerms().size() == 2.000000
   bool isAnywayConditionSecondPart() const;            // Of type: formula->getOptor() == formula_1->getOptor()
   int isSizeComparison() const;                        // Of type: m->getTerms().size() == 2.000000;
   int isOptorComparison(const FormulaParser& p) const; // Of type: m->getOptor() == "max"

   void reInitialize(const IfCondition other);

   bool isOtherEqual(const IfCondition& other) const;
   bool isOtherMoreSpecific(const IfCondition& other, const FormulaParser& p) const;
   bool isOtherUnrelated(const IfCondition& other, const FormulaParser& p) const;

private:
   IfConditionMode mode_{};
   IfConditionComparisonSide side1_{};
   IfConditionComparisonSide side2_{};
   IfConditionComparisonSide side3_{};
   IfConditionComparisonSide side4_{};
   std::string free_string_{};
};

class SimplificationFunctionLineIf : public SimplificationFunctionLine
{
public:
   void applyToMeAndMyChildren(const std::function<void(const std::shared_ptr<SimplificationFunctionLine>)>& f, bool& breakitoff) override;
   std::shared_ptr<SimplificationFunctionLineIf> toIfIfApplicable() override;
   std::string getContent() const override;
   std::string serializeSingleLine() const override;
   std::string serializeSingleLine(const int indent, const std::string& formula_name) const override;
   IfCondition getCondition() const;
   std::string getConditionString() const;
   std::shared_ptr<SimplificationFunctionLine> getBodyIf() const;
   void setBodyIf(const std::shared_ptr<SimplificationFunctionLine> body);
   std::shared_ptr<SimplificationFunctionLine> getBodyElse() const;
   void setBodyElse(const std::shared_ptr<SimplificationFunctionLine> body);
   std::vector<std::shared_ptr<SimplificationFunctionLineIf>> getElseIfs() const;
   void addElseIf(const IfCondition& condition, const std::shared_ptr<SimplificationFunctionLine> body);
   void setCondition(const IfCondition& condition);

   SimplificationFunctionLineIf( // Don't call this directly, always use getInstance to get fathers initialized correctly.
      const IfCondition& condition,
      const std::shared_ptr<SimplificationFunctionLine> body_if,
      const std::shared_ptr<SimplificationFunctionLine> body_else = nullptr,
      const std::string& comment = "");

   static std::shared_ptr<SimplificationFunctionLineIf> getInstance(
      const IfCondition& condition,
      const std::shared_ptr<SimplificationFunctionLine> body_if,
      const std::shared_ptr<SimplificationFunctionLine> body_else = nullptr,
      const std::string& comment = "");

private:
   std::shared_ptr<SimplificationFunctionLine> body_if_;
   std::vector<std::shared_ptr<SimplificationFunctionLineIf>> else_ifs_;
   std::shared_ptr<SimplificationFunctionLine> body_else_;
   mutable IfCondition condition_{};

   std::shared_ptr<SimplificationFunctionLine> copy_core() const override;
};

class SimplificationFunctionLineDef : public SimplificationFunctionLine
{
public:
   SimplificationFunctionLineDef(
      const std::vector<int>& trace_up_to,
      const int next,
      const std::string& custom_right_side = "",
      const std::string& comment = "",
      const bool declare = true);

   virtual std::shared_ptr<SimplificationFunctionLineDef> toDefIfApplicable() override;
   std::string getLeftSide() const;
   std::string getContent() const override;
   std::vector<int> getTraceUpTo() const;
   int getNext() const;
   std::string serializeSingleLine() const override;
   std::string serializeSingleLine(const int indent, const std::string& formula_name) const override;
   void setDeclare(const bool do_it);
   bool isDeclare() const;

private:
   std::vector<int> trace_up_to_{};
   int next_{};
   std::string custom_rhs_{};
   bool declare_{};

   std::shared_ptr<SimplificationFunctionLine> copy_core() const override;
};

class SimplificationFunctionLineDecl : public SimplificationFunctionLine
{
public:
   SimplificationFunctionLineDecl(const std::vector<int>& trace_up_to, const int next, const std::string& comment = "");

   virtual std::shared_ptr<SimplificationFunctionLineDecl> toDeclIfApplicable() override;
   std::string getContent() const override;
   std::vector<int> getTraceUpTo() const;
   int getNext() const;
   std::string serializeSingleLine() const override;
   std::string serializeSingleLine(const int indent, const std::string& formula_name) const override;

private:
   std::vector<int> trace_up_to_;
   int next_;

   std::shared_ptr<SimplificationFunctionLine> copy_core() const override;
};

class SimplificationFunctionLineReturn : public SimplificationFunctionLine
{
public:
   SimplificationFunctionLineReturn(const bool value, const std::string& comment = "");

   virtual std::shared_ptr<SimplificationFunctionLineReturn> toReturnIfApplicable() override;
   std::string serializeSingleLine() const override;
   std::string serializeSingleLine(const int indent, const std::string& formula_name) const override;
   //std::string getContent() const override;
   bool getRetValue() const;
   void setRetValue(const bool value);

private:
   bool ret_value_;

   std::shared_ptr<SimplificationFunctionLine> copy_core() const override;
};

class SimplificationFunctionLineCustom : public SimplificationFunctionLine
{
public:
   SimplificationFunctionLineCustom(const std::string& arbitrary_content, const std::string& comment = "");

   virtual std::shared_ptr<SimplificationFunctionLineCustom> toCustomIfApplicable() override;
   std::string getRetValue() const;
   void setRetValue(const bool arbitrary_content);

private:
   std::string arbitrary_content_;

   std::shared_ptr<SimplificationFunctionLine> copy_core() const override;
};

class SimplificationFunctionLineNull : public SimplificationFunctionLine
{
public:
   SimplificationFunctionLineNull();

   virtual std::shared_ptr<SimplificationFunctionLineNull> toNullIfApplicable() override;
   std::string serializeSingleLine(const int indent, const std::string& formula_name) const override;

private:
   std::shared_ptr<SimplificationFunctionLine> copy_core() const override;
};

class SimplificationFunctionLinePlaceholder : public SimplificationFunctionLineNull
{
public:
   SimplificationFunctionLinePlaceholder();

   virtual std::shared_ptr<SimplificationFunctionLinePlaceholder> toPlaceholderIfApplicable() override;
   std::string serializeSingleLine(const int indent, const std::string& formula_name) const override;

private:
   std::shared_ptr<SimplificationFunctionLine> copy_core() const override;
};

enum class CodeGenerationMode
{
   positive, // Default
   negative
};

class CodeGenerator
{
public:
   static std::shared_ptr<simplification::SimplificationFunctionLine> createCodeFromMetaRule(
      const MetaRule& rule, 
      const CodeGenerationMode code_generation_mode,
      const bool optimize_away_first_condition,
      const std::string& formula_name,
      const std::string& parser_name, 
      const std::shared_ptr<FormulaParser> parser);

   static std::string createCodeFromMetaRuleString(const MetaRule& rule, const CodeGenerationMode code_generation_mode, const std::string& name, const bool optimize_away_first_condition, const std::string& parser_name, const std::shared_ptr<FormulaParser> parser);
   static std::string createCodeForAllSimplificationRules(const CodeGenerationMode code_generation_mode, const std::shared_ptr<FormulaParser> parser = nullptr, const bool mc_mode = false);
   static std::string getTrailForMetaRuleCodeGeneration(const std::vector<int>& trail_orig, const bool make_last_item_native = false);
   static void deleteAndWriteSimplificationRulesToFile(const CodeGenerationMode code_generation_mode, const std::string& path, const std::shared_ptr<FormulaParser> parser_raw = nullptr, const bool mc_mode = false);

private:
   static std::string createHeadPartOfSimplificationCode(
      const bool mc_mode,
      const CodeGenerationMode code_generation_mode,
      const std::set<vfm::MetaRule>& abandoned_rules, 
      const std::set<vfm::MetaRule>& additional_rules, 
      const std::map<std::string, std::map<int, std::set<MetaRule>>>& all_used_rules,
      const std::shared_ptr<FormulaParser> parser);

   static std::string createFootPartOfSimplificationCode(
      const CodeGenerationMode code_generation_mode,
      const bool mc_mode);

   static std::string addTopLevelSimplificationFunction(
      const CodeGenerationMode code_generation_mode,
      const std::string& inner_part_before,
      const std::vector<std::pair<int, std::string>>& inner_part,
      const std::string& inner_part_after,
      const bool very_fast);

   static std::string createInnerPartOfSimplificationCode(
      const CodeGenerationMode code_generation_mode,
      const std::map<std::string, std::map<int, std::set<vfm::MetaRule>>>& all_rules,
      const std::shared_ptr<vfm::FormulaParser> parser,
      std::string& inner_part_before,
      std::vector<std::pair<int, std::string>>& inner_part_fast,
      std::vector<std::pair<int, std::string>>& inner_part_very_fast,
      std::string& inner_part_after,
      const bool mc_mode);
};

}; // simplification
}; // vfm