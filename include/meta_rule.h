//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file
#pragma once
#include "term_val.h"
#include "term_var.h"
#include "term_meta_simplification.h"
#include "static_helper.h"
#include "term.h"
#include "parser.h"
#include "parsable.h"

namespace vfm {

const std::string OPENING_BRACKET_GLOBAL_CONDITION = "{{";
const std::string CLOSING_BRACKET_GLOBAL_CONDITION = "}}"; // TODO: "if (.) { if (.) {.}}" could yield a wrong cut-off when using just two closing braces here.
const std::string OPENING_BRACKET_LOCAL_CONDITIONS = "[[";
const std::string CLOSING_BRACKET_LOCAL_CONDITIONS = "]]"; // TODO: "x[y[i]]" could yield a wrong cut-off when using just two closing brackets here.
const std::string FROM_TO_SYMBOL = "==>";
const std::string QUOTE_SYMBOL_FOR_CONTITION_NAME = "'";
const std::string DELIMITER_LOCAL_CONDITIONS = ",";        // Disambiguation with f(., .) works only when combined with the QUOTE symbol.
const std::string DELIMITER_LOCAL_CONDITIONS_AFTER_TERM = ":"; // TODO: Ternary operator, once implemented, could create a problem here.
const std::string STAGE_SERIALIZATION_DELIMITER = "$";     // Last part of serialized rule is $x with x being the integer denoting the stage.

constexpr int SIMPLIFICATION_STAGE_PREPARATION{ 0 };
constexpr int SIMPLIFICATION_STAGE_MAIN{ 10 };

using RulesByStage = std::map<int, std::map<std::string, std::map<int, std::set<MetaRule>>>>;
using Rules = std::map<std::string, std::map<int, std::set<MetaRule>>>;

/// <summary>
/// A MetaRule represents a rule of type:
///    x ==> y [[ *under condition set C* ]] {{ *and under global condition G* }}
/// 
/// For example, this is a typical MetaRule:
///    if (p_(0)) { p_(1) } { p_(2) } ==> p_(1) [[p_(0): 'is_constant_and_evaluates_to_true']] {{ 'no_check' }}
/// 
/// It states that an IF statement (x) can be reduced to the THEN case p_(1) (y) if the condition p_(0) is constantly true (C).
/// The global condition G is no_check here, which simply means "always true" and can also be expressed by leaving out the {{...}} part.
/// MetaRules are used in MathStruct::simplify, simplification::simplifyFast etc. to simplify a formula or program in a greedy manner, 
/// meaning that the formula tree is traversed, and at each node all rules are tried out one after the other; if a rule matches it is applied 
/// immediately, before the traversal continues. The traversal is repeated until no changes occur. 
/// 
/// Additional notes:
/// - The metas "p_(i)" can be replaced by variables "x", "y", ... at construction; use convertVeriablesToMetas() afterwards
/// - The TermOptional "o_(p_(i), c)" can be used to denote an operand that can or cannot exist, and is treated as c, if missing. For example,
///      o_(a, 1) * x + o_(b, 1) * x ==> (a + b) * x
///   means: "a * x + b * x" ==> (a + b) * x, where a and/or b can be missing and are then treated as 1. A special case covered by this rule is:
///      x + x ==> (1 + 1) * x
/// - Conditions are implemented in plain C++ via static functions, see below. Therefore, they have full Turing power to calculate their result.
/// - The enum class defining the condition names is placed in math_struct.h. (Cf. "Caution" note below.)
/// - ALTERNATIVE USECASE (non-local application): The above description (USECASE 1), implicitly assumed that the conditions in C are read-only operations
///   yielding a true/false answer. However, this implies that only local changes to the formula tree are possible - although conditions can
///   traverse the full formula to decide if the result is true or false. To exploit the full power of conditions, it is allowed to let 
///   a condition modify the formula during traversal ("MODIFYING CONDITION"). The following constraints need to be obeyed:
///   * The condition must have a capital letter in its name, typically a capitalized part indicating what it does, like
///        is_any_var_of_second_on_any_left_side_of_assignment_in_first_AND_INSERT
///   * All "regular" read-only conditions have no capital letters.
///   * A modifying condition is allowed only as LAST condition in each MetaRule. (Otherwise, a modification might occur although
///     the rule is deemed non-matching afterwards.) Particularly, only ONE modifying condition per rule is allowed.
///   * (For now) A rule containing a modifying condition cannot apply additional changes via the regular mechanism, i.e., we need to obey:
///        x = y
///     for all modifying conditions.
/// 
/// Caution: MetaRules can be evaluated at runtime (MathStruct::simplify) or via code generation (e.g., simplification::simplifyFast).
///          To fit both usecases, it is important to obey the following naming conventions:
///          - Conditions need to be named equally in:
///            * The enum class ConditionType in math_struct.h.
///            * The string name in the condition_type_map in math_struct.h.
///            * The function name of the static function below.
///          - Regular read-only conditions need to be lower-case.
///          - Modifying conditions need to contain at least one upper-case letter.
///          - Pascal case is optional, but recommended.
/// </summary>
class MetaRule : public Parsable {
public:
   
   /// This constructor is called eventually and expects only real operator names.
   MetaRule(
      const TermPtr from,
      const TermPtr to,
      const int stage,
      const std::vector<std::pair<std::shared_ptr<Term>, ConditionType>>& condition_types = std::vector<std::pair<std::shared_ptr<Term>, ConditionType>>(),
      const ConditionType global_condition = ConditionType::no_check,
      const std::shared_ptr<FormulaParser> parser = nullptr);

   MetaRule(
      const std::string& serialized_meta_rule, 
      const int stage,
      const std::shared_ptr<FormulaParser> parser = nullptr);

   /// This constructor is called by all string-based constructors.
   MetaRule(
      const std::string& from,
      const std::string& to,
      const int stage,
      const std::vector<std::pair<std::string, ConditionType>>& condition_types = std::vector<std::pair<std::string, ConditionType>>(),
      const ConditionType global_condition = ConditionType::no_check,
      const std::map<std::string, std::string>& real_op_names = {},
      const std::shared_ptr<FormulaParser> parser = nullptr);

   MetaRule(
      const std::string& from,
      const std::string& to,
      const int stage,
      const std::string& condition,
      const ConditionType global_condition = ConditionType::no_check,
      const std::map<std::string, std::string>& real_op_names = {},
      const std::shared_ptr<FormulaParser> parser = nullptr);

   MetaRule(
      const std::string& from,
      const std::string& to,
      const int stage,
      const ConditionType global_condition,
      const std::map<std::string, std::string>& real_op_names = {},
      const std::shared_ptr<FormulaParser> parser = nullptr);

   std::shared_ptr<Term> getTo() const;
   std::shared_ptr<Term> getFrom() const;
   void setTo(const std::shared_ptr<Term> to);
   void setFrom(const std::shared_ptr<Term> from);
   std::vector<std::pair<std::shared_ptr<Term>, ConditionType>> getConditions() const;
   ConditionType getGlobalCondition() const;
   MetaRule copy(const bool deep_copy_from, const bool deep_copy_to, const bool deep_copy_conditions) const;
   std::string serialize(const bool include_stage = true) const;
   std::string serialize(const bool convert_back_to_variables, const bool include_stage) const;
   void convertVeriablesToMetas();
   bool containsModifyingCondition() const;
   bool parseProgram(const std::string& program) override;

   /// <summary>
   /// Assuming "sensible" formulas. Not allowed: !(!(...)), bool(bool(...)), bool(not(...)) etc.
   /// which are simplified away, anyway.
   /// </summary>
   static bool isNegationOf(const TermPtr t0, const TermPtr t1);

   static bool is_constant(const std::shared_ptr<MathStruct> term);
   static bool consists_purely_of_numbers(const std::shared_ptr<MathStruct> term);
   static bool is_not_constant(const std::shared_ptr<MathStruct> term);
   static bool is_constant_and_evaluates_to_true(const std::shared_ptr<MathStruct> term);
   static bool is_constant_and_evaluates_to_false(const std::shared_ptr<MathStruct> term);
   static bool is_constant_and_evaluates_to_positive(const std::shared_ptr<MathStruct> term);
   static bool is_constant_and_evaluates_to_negative(const std::shared_ptr<MathStruct> term);
   static bool is_constant_0(const std::shared_ptr<MathStruct> term);
   static bool is_constant_1(const std::shared_ptr<MathStruct> term);
   static bool is_non_boolean_constant(const std::shared_ptr<MathStruct> term);
   static bool is_not_non_boolean_constant(const std::shared_ptr<MathStruct> term);
   static bool is_constant_integer(const std::shared_ptr<MathStruct> term);
   static bool is_constant_non_integer(const std::shared_ptr<MathStruct> term);
   static bool has_no_sideeffects(const std::shared_ptr<MathStruct> term);
   static bool has_sideeffects(const std::shared_ptr<MathStruct> term);
   static bool is_leaf(const std::shared_ptr<MathStruct> term);
   static bool is_not_leaf(const std::shared_ptr<MathStruct> term);
   static bool is_variable(const std::shared_ptr<MathStruct> term);
   static bool is_not_variable(const std::shared_ptr<MathStruct> term);
   static bool has_boolean_result(const std::shared_ptr<MathStruct> term);
   static bool has_non_boolean_result(const std::shared_ptr<MathStruct> term);
   static bool of_first_two_operands_second_is_alphabetically_above(const std::shared_ptr<MathStruct> term);
   static bool first_two_operands_are_negations_of_each_other(const std::shared_ptr<MathStruct> term);
   static bool first_two_variables_are_equal_except_reference(const std::shared_ptr<MathStruct> term);
   static bool first_two_variables_are_not_equal_except_reference(const std::shared_ptr<MathStruct> term);
   static bool not_is_root(const std::shared_ptr<MathStruct> term);
   static bool second_operand_contained_in_first_operand(const std::shared_ptr<MathStruct> term);
   static bool second_operand_not_contained_in_first_operand(const std::shared_ptr<MathStruct> term);
   static bool second_appears_as_read_variable_in_first(const std::shared_ptr<MathStruct> term);
   static bool second_appears_not_as_read_variable_in_first(const std::shared_ptr<MathStruct> term);
   static bool is_any_var_of_second_on_any_left_side_of_assignment_in_first(const std::shared_ptr<MathStruct> term);
   static bool is_any_var_of_second_on_any_left_side_of_assignment_in_first_AND_INSERT(const std::shared_ptr<MathStruct> term);
   static bool is_any_var_of_second_on_any_left_side_earlier_AND_INSERT(const std::shared_ptr<MathStruct> term);
   static bool is_no_var_of_second_on_any_left_side_of_assignment_in_first(const std::shared_ptr<MathStruct> term);
   static bool is_commutative_operation(const std::shared_ptr<MathStruct> term);
   static bool is_not_commutative_operation(const std::shared_ptr<MathStruct> term);
   static bool is_associative_operation(const std::shared_ptr<MathStruct> term);
   static bool is_not_associative_operation(const std::shared_ptr<MathStruct> term);
   static bool is_comparator_or_equality_operation(const std::shared_ptr<MathStruct> term);
   static bool is_not_comparator_or_equality_operation(const std::shared_ptr<MathStruct> term);
   static bool contains_meta(const std::shared_ptr<MathStruct> term);
   static bool contains_no_meta(const std::shared_ptr<MathStruct> term);
   static bool parent_is_get(const std::shared_ptr<MathStruct> term);
   static bool parent_is_not_get(const std::shared_ptr<MathStruct> term);
   static bool no_check(const std::shared_ptr<MathStruct> term);

   static bool checkCondition(const ConditionType mode, const std::shared_ptr<MathStruct> term);

   static bool isConditionModifying(const std::string& name);
   static bool isConditionModifying(const ConditionType& cond);
   static bool isConditionModifying(const ConditionTypeWrapper& cond);

   bool operator< (const MetaRule& right) const;

   void setRuleAbandoned() const;
   bool isRuleAbandoned() const;
   int getStage() const;
   void setStage(const int stage) const;

   static int countMetaRules(const std::map<std::string, std::map<int, std::set<MetaRule>>>& rules);
   static RulesByStage getMetaRulesByStage(const Rules& rules);

private:
   static std::string convertString(const std::string str, const std::map<std::string, std::string> real_op_names);
   
   static std::vector<std::pair<std::shared_ptr<Term>, ConditionType>> convertStringVec(
      const std::vector<std::pair<std::string, ConditionType>> vec,
      const std::map<std::string, std::string> real_op_names,
      const std::shared_ptr<FormulaParser> parser);

   static int insertOrNotAndReturn(std::map<std::string, int>& map, int& count, const std::string& name);

   static void convertSingleTerm(
      std::shared_ptr<Term>& t,
      std::map<std::string, int>& map,
      int& count,
      const std::function<void(std::shared_ptr<MathStruct>)>& func);

   TermPtr convertAnywayOperations(const TermPtr m_raw);
   void convertAllAnywayOperations();

   bool checkRuleForConsistency() const;

   std::shared_ptr<Term> from_;
   std::shared_ptr<Term> to_;
   std::vector<std::pair<std::shared_ptr<Term>, ConditionType>> under_conditions_;
   ConditionType global_condition_;
   std::shared_ptr<FormulaParser> parser_;
   mutable bool rule_abandoned_ = false; // Used for code generation, if true, this rule will not be generated, and a note will be written in the auto-generated comments.
   mutable int stage_{}; // Used by simplification to apply some ordering to the rules. "Stage 0" rules will be applied first, then "stage 1" etc.
};

} // vfm
