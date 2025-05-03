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
namespace code_block {

static const std::string PLACEHOLDER_FORMULA_NAME{ "FORMULANAME" };

class CodeBlockCustom;
class CodeBlockDecl;
class CodeBlockDef;
class CodeBlockIf;
class CodeBlockNull;
class CodeBlockPlaceholder;
class CodeBlockReturn;

class CodeBlock : public std::enable_shared_from_this<CodeBlock>, public Failable
{
public:
   CodeBlock(const std::string& content, const std::string& comment);

   virtual std::string serializeSingleLine() const;
   virtual std::string serializeSingleLine(const int indent, const std::string& formula_name) const;
   std::string missingPredecessorNote() const;
   void applyToMeAndMyChildren(const std::function<void(const std::shared_ptr<CodeBlock>)>& f);
   virtual void applyToMeAndMyChildren(const std::function<void(const std::shared_ptr<CodeBlock>)>& f, bool& breakitoff);
   virtual void applyToMeAndMyParents(const std::function<void(const std::shared_ptr<CodeBlock>)>& f, const bool exclude_me = false);
   void replace(const std::shared_ptr<CodeBlock> line);
   
   bool isEmptyLine() const;
   bool isEmptyFromThisLineOn() const;

   enum class MarkerMode {
      never,
      always,
      automatic
   };

   std::string serializeBlock() const;
   std::string serializeBlock(const int indent, const std::string& formula_name, const MarkerMode marker_mode = MarkerMode::automatic) const;
   bool isStructurallyEqual(const std::shared_ptr<CodeBlock> other);

   virtual std::shared_ptr<CodeBlockIf> toIfIfApplicable() { return nullptr; }
   virtual std::shared_ptr<CodeBlockReturn> toReturnIfApplicable() { return nullptr; }
   virtual std::shared_ptr<CodeBlockDecl> toDeclIfApplicable() { return nullptr; }
   virtual std::shared_ptr<CodeBlockDef> toDefIfApplicable() { return nullptr; }
   virtual std::shared_ptr<CodeBlockCustom> toCustomIfApplicable() { return nullptr; }
   virtual std::shared_ptr<CodeBlockNull> toNullIfApplicable() { return nullptr; }
   virtual std::shared_ptr<CodeBlockPlaceholder> toPlaceholderIfApplicable() { return nullptr; }

   virtual std::string getContent() const;
   bool containsLineBelowIncludingThis(const std::string& line) const;
   bool containsLineAboveIncludingThis(const std::string& line) const;
   void setContent(const std::string& content);
   std::shared_ptr<CodeBlock> getSuccessor() const;
   std::shared_ptr<CodeBlock> getPredecessor() const;
   std::shared_ptr<CodeBlock> getLastLine();
   std::shared_ptr<CodeBlock> getFirstLine();
   bool checkForEqualDefinitions();
   void setSuccessor(const std::shared_ptr<CodeBlock> successor);
   void setPredecessor(const std::shared_ptr<CodeBlock> predecessor);
   void appendAtTheEnd(const std::shared_ptr<CodeBlock> line, const bool replace_last_line = false, const bool insert_one_before_end = false);
   std::string getComment() const;
   void setComment(const std::string& comment);
   void setFather(const std::shared_ptr< CodeBlock> father);
   std::shared_ptr<CodeBlock> getFather() const;
   void makeComment(const bool do_it);
   bool isComment() const;
   std::shared_ptr<CodeBlock> goToNextNonEmptyLine();
   std::shared_ptr<CodeBlockIf> goToLastIfInBlock();

   void removeAllRedefinitionsOrReinitializations();
   void mergeWithOtherSimplificationFunction(const std::shared_ptr<CodeBlock> other, const FormulaParser& p);

   std::shared_ptr<CodeBlock> copy();

   bool checkIntegrity();
   int nodeCount();

private:
   bool make_comment_{ false };
   std::string content_{};
   std::string comment_{};
   std::shared_ptr<CodeBlock> successor_{};
   std::shared_ptr<CodeBlock> predecessor_{}; // Only one of predecessor_ or father_ can be non-null.
   std::weak_ptr<CodeBlock> father_{}; // Non-null only for the FIRST line in an IF sub-statement.

   virtual std::shared_ptr<CodeBlock> copy_core() const = 0;
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

class CodeBlockIf : public CodeBlock
{
public:
   void applyToMeAndMyChildren(const std::function<void(const std::shared_ptr<CodeBlock>)>& f, bool& breakitoff) override;
   std::shared_ptr<CodeBlockIf> toIfIfApplicable() override;
   std::string getContent() const override;
   std::string serializeSingleLine() const override;
   std::string serializeSingleLine(const int indent, const std::string& formula_name) const override;
   IfCondition getCondition() const;
   std::string getConditionString() const;
   std::shared_ptr<CodeBlock> getBodyIf() const;
   void setBodyIf(const std::shared_ptr<CodeBlock> body);
   std::shared_ptr<CodeBlock> getBodyElse() const;
   void setBodyElse(const std::shared_ptr<CodeBlock> body);
   std::vector<std::shared_ptr<CodeBlockIf>> getElseIfs() const;
   void addElseIf(const IfCondition& condition, const std::shared_ptr<CodeBlock> body);
   void setCondition(const IfCondition& condition);

   CodeBlockIf( // Don't call this directly, always use getInstance to get fathers initialized correctly.
      const IfCondition& condition,
      const std::shared_ptr<CodeBlock> body_if,
      const std::shared_ptr<CodeBlock> body_else = nullptr,
      const std::string& comment = "");

   static std::shared_ptr<CodeBlockIf> getInstance(
      const IfCondition& condition,
      const std::shared_ptr<CodeBlock> body_if,
      const std::shared_ptr<CodeBlock> body_else = nullptr,
      const std::string& comment = "");

private:
   std::shared_ptr<CodeBlock> body_if_;
   std::vector<std::shared_ptr<CodeBlockIf>> else_ifs_;
   std::shared_ptr<CodeBlock> body_else_;
   mutable IfCondition condition_{};

   std::shared_ptr<CodeBlock> copy_core() const override;
};

class CodeBlockDef : public CodeBlock
{
public:
   CodeBlockDef(
      const std::vector<int>& trace_up_to,
      const int next,
      const std::string& custom_right_side = "",
      const std::string& comment = "",
      const bool declare = true);

   virtual std::shared_ptr<CodeBlockDef> toDefIfApplicable() override;
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

   std::shared_ptr<CodeBlock> copy_core() const override;
};

class CodeBlockDecl : public CodeBlock
{
public:
   CodeBlockDecl(const std::vector<int>& trace_up_to, const int next, const std::string& comment = "");

   virtual std::shared_ptr<CodeBlockDecl> toDeclIfApplicable() override;
   std::string getContent() const override;
   std::vector<int> getTraceUpTo() const;
   int getNext() const;
   std::string serializeSingleLine() const override;
   std::string serializeSingleLine(const int indent, const std::string& formula_name) const override;

private:
   std::vector<int> trace_up_to_;
   int next_;

   std::shared_ptr<CodeBlock> copy_core() const override;
};

class CodeBlockReturn : public CodeBlock
{
public:
   CodeBlockReturn(const bool value, const std::string& comment = "");

   virtual std::shared_ptr<CodeBlockReturn> toReturnIfApplicable() override;
   std::string serializeSingleLine() const override;
   std::string serializeSingleLine(const int indent, const std::string& formula_name) const override;
   //std::string getContent() const override;
   bool getRetValue() const;
   void setRetValue(const bool value);

private:
   bool ret_value_;

   std::shared_ptr<CodeBlock> copy_core() const override;
};

class CodeBlockCustom : public CodeBlock
{
public:
   CodeBlockCustom(const std::string& arbitrary_content, const std::string& comment = "");

   virtual std::shared_ptr<CodeBlockCustom> toCustomIfApplicable() override;
   std::string getRetValue() const;
   void setRetValue(const bool arbitrary_content);

private:
   std::string arbitrary_content_;

   std::shared_ptr<CodeBlock> copy_core() const override;
};

class CodeBlockNull : public CodeBlock
{
public:
   CodeBlockNull();

   virtual std::shared_ptr<CodeBlockNull> toNullIfApplicable() override;
   std::string serializeSingleLine(const int indent, const std::string& formula_name) const override;

private:
   std::shared_ptr<CodeBlock> copy_core() const override;
};

class CodeBlockPlaceholder : public CodeBlockNull
{
public:
   CodeBlockPlaceholder();

   virtual std::shared_ptr<CodeBlockPlaceholder> toPlaceholderIfApplicable() override;
   std::string serializeSingleLine(const int indent, const std::string& formula_name) const override;

private:
   std::shared_ptr<CodeBlock> copy_core() const override;
};

enum class CodeGenerationMode
{
   positive, // Default
   negative
};

class CodeGenerator
{
public:
   static std::shared_ptr<code_block::CodeBlock> createCodeFromMetaRule(
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

}; // code_block
}; // vfm