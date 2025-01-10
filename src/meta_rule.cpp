//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file

#include "meta_rule.h"
#include "term_compound.h"


using namespace vfm;

TermPtr MetaRule::convertAnywayOperations(const TermPtr m_raw)
{
   if (!m_raw) return m_raw;

   auto m = _id(m_raw);
   bool changed = true;

   while (changed) {
      changed = false;

      m->applyToMeAndMyChildrenIterative([this, &changed, m_raw](const MathStructPtr m2) {
         auto m2_compound{ m2->toTermCompoundIfApplicable() };
         if ( m2_compound && StaticHelper::stringStartsWith(m2_compound->getCompoundOperator()->getOptor(), SYMB_ANYWAY_S)) { // Assume that each i+1 adds an additional SYMB_ANYWAY0 to i.
            auto m2_compound_operator{ m2_compound->getCompoundOperator() };
            m2_compound_operator->getOperands().insert(m2_compound_operator->getOperands().begin(), _val(m2_compound_operator->getOptor().size()));
            auto replacement = parser_->termFactory(SYMB_ANYWAY, m2_compound_operator->getOperands());
            replacement->setChildrensFathers(false, false);
            m2->replace(replacement); // We need to go over compound border: compound(~, ...) ==> a_(...).
            changed = true;
         }
      }, TraverseCompoundsType::avoid_compound_structures, FormulaTraversalType::PostOrder);
   }

   return m->child0JumpIntoCompounds();
}

void MetaRule::convertAllAnywayOperations()
{
   from_ = convertAnywayOperations(from_);
   to_ = convertAnywayOperations(to_);

   for (auto& cond : under_conditions_) {
      cond.first = convertAnywayOperations(cond.first);
   }
}

MetaRule::MetaRule(
   const TermPtr from,
   const TermPtr to,
   const int stage,
   const std::vector<std::pair<std::shared_ptr<Term>, ConditionType>>& condition_types,
   const ConditionType global_condition,
   const std::shared_ptr<FormulaParser> parser) :
   Parsable("MetaRule"),
   from_(from),
   to_(to),
   under_conditions_(condition_types),
   global_condition_(global_condition),
   parser_(parser ? parser : SingletonFormulaParser::getInstance()),
   stage_{stage}
{
   if (from_) from_->setChildrensFathers();
   if (to_) to_->setChildrensFathers();

   for (const auto& pairs : under_conditions_) {
      pairs.first->setChildrensFathers();
   }

   convertAllAnywayOperations();
   checkRuleForConsistency();
}

MetaRule::MetaRule(
   const std::string& serialized_meta_rule, 
   const int stage,
   const std::shared_ptr<FormulaParser> parser)
   : MetaRule(nullptr, nullptr, stage, std::vector<std::pair<std::shared_ptr<Term>, ConditionType>>{}, ConditionType::no_check, parser)
{
   parseProgram(serialized_meta_rule);
   convertAllAnywayOperations();
   checkRuleForConsistency();
}

MetaRule::MetaRule(
   const std::string& from,
   const std::string& to,
   const int stage,
   const std::vector<std::pair<std::string, ConditionType>>& condition_types,
   const ConditionType global_condition,
   const std::map<std::string, std::string>& real_op_names,
   const std::shared_ptr<FormulaParser> parser) : MetaRule(
      MathStruct::parseMathStruct(convertString(from, real_op_names), false, false, parser)->toTermIfApplicable(), // Cannot use Cpp syntax for performance reasons.
      MathStruct::parseMathStruct(convertString(to, real_op_names), false, false, parser)->toTermIfApplicable(),   // Cannot use Cpp syntax for performance reasons.
      stage,
      convertStringVec(condition_types, real_op_names, parser),
      global_condition)
{}

MetaRule::MetaRule(
   const std::string& from,
   const std::string& to,
   const int stage,
   const std::string& condition,
   const ConditionType global_condition,
   const std::map<std::string, std::string>& real_op_names,
   const std::shared_ptr<FormulaParser> parser) :
   MetaRule(from,
      to,
      stage,
      { { condition, ConditionType::is_constant_and_evaluates_to_true } },
      global_condition,
      real_op_names,
      parser)
{}

MetaRule::MetaRule(
   const std::string& from,
   const std::string& to,
   const int stage,
   const ConditionType global_condition,
   const std::map<std::string, std::string>& real_op_names,
   const std::shared_ptr<FormulaParser> parser) :
   MetaRule(from,
      to,
      stage,
      std::vector<std::pair<std::string, ConditionType>>{},
      global_condition,
      real_op_names,
      parser)
{}

bool vfm::MetaRule::checkRuleForConsistency() const
{
   const auto& conds{ getConditions() };
   const size_t size{ conds.size() };
   bool is_modifying{ false };
   bool is_consistent{ true };

   for (int i = 0; i < size; i++) {
      const auto& cond = conds.at(i);

      if (isConditionModifying(cond.second)) { // Find modifying conditions.
         is_modifying = true;

         if (i < size - 1) { // Modifying condition needs to be unique at the end.
            addError("Modifying condition '" + ConditionTypeWrapper(cond.second).getEnumAsString()
               + "' found at non-last position (" + std::to_string(i + 1) + "<" + std::to_string(size)
               + ") in rule '" + serialize()
               + "'. This is not allowed since modifications could be applied even if a rule is recognized as non-matching afterward.");

            is_consistent = false;
         }
      }
   }

   if (is_modifying) { // Modifying condition 
      if (!from_->isStructurallyEqual(to_)) {
         addError("MetaRule '" + serialize() + "' has a modifying condition, but 'from' is unequal to 'to'.");
         is_consistent = false;
      }
   }

   // TODO: Prohibit x ==> y [[ z: '...' ]], i.e., all metas must occur in from_.

   return is_consistent;
}

std::shared_ptr<Term> MetaRule::getTo() const
{
   return to_;
}

std::shared_ptr<Term> MetaRule::getFrom() const
{
   return from_;
}

void MetaRule::setTo(const std::shared_ptr<Term> to)
{
   to_ = to;
}

void MetaRule::setFrom(const std::shared_ptr<Term> from)
{
   from_ = from;
}

std::vector<std::pair<std::shared_ptr<Term>, ConditionType>> MetaRule::getConditions() const
{
   return under_conditions_;
}

ConditionType MetaRule::getGlobalCondition() const
{
   return global_condition_;
}

MetaRule MetaRule::copy(const bool deep_copy_from, const bool deep_copy_to, const bool deep_copy_conditions) const
{
   std::vector<std::pair<std::shared_ptr<Term>, ConditionType>> vec{};

   for (const auto& pair : under_conditions_) {
      vec.push_back({ 
         deep_copy_conditions ? pair.first->copy() : pair.first,
         pair.second
      });
   }

   return MetaRule(
      deep_copy_from ? from_->copy() : from_,
      deep_copy_to ? to_->copy() : to_,
      stage_,
      vec, 
      global_condition_);
}

std::string convertMetasToVariables(const std::string& with_metas)
{
   auto res = with_metas;

   for (int i = 0; i < ALPHA_ALL.size(); i++) {
      std::string meta_name = SYMB_META_SIMP + OPENING_BRACKET + std::to_string(i) + CLOSING_BRACKET;
      std::string var_name = StaticHelper::makeString(ALPHA_ALL[i]);
      res = StaticHelper::replaceAll(res, meta_name, var_name);
   }

   return res;
}

std::string vfm::MetaRule::serialize(const bool include_stage) const
{
   return serialize(true, include_stage);
}

std::string MetaRule::serialize(const bool convert_back_to_variables, const bool include_stage) const
{
   std::string cond{};
   int count{ 0 };

   for (const auto& pair : under_conditions_) {
      count++;
      std::string comma = count != under_conditions_.size() ? DELIMITER_LOCAL_CONDITIONS + " " : "";
      auto pair_ser = pair.first->serialize(nullptr, MathStruct::SerializationStyle::plain_old_vfm, MathStruct::SerializationSpecial::none, 0, 0);

      if (convert_back_to_variables) {
         pair_ser = convertMetasToVariables(pair_ser);
      }

      cond += pair_ser + DELIMITER_LOCAL_CONDITIONS_AFTER_TERM + " " + QUOTE_SYMBOL_FOR_CONTITION_NAME + ConditionTypeWrapper(pair.second).getEnumAsString() + QUOTE_SYMBOL_FOR_CONTITION_NAME + "" + comma;
   }

   if (!cond.empty()) {
      cond = " " + OPENING_BRACKET_LOCAL_CONDITIONS + cond + CLOSING_BRACKET_LOCAL_CONDITIONS;
   }

   if (global_condition_ != ConditionType::no_check) {
      cond += " " + OPENING_BRACKET_GLOBAL_CONDITION + QUOTE_SYMBOL_FOR_CONTITION_NAME + ConditionTypeWrapper(global_condition_).getEnumAsString() + QUOTE_SYMBOL_FOR_CONTITION_NAME + CLOSING_BRACKET_GLOBAL_CONDITION;
   }

   auto from_ser = from_ ? from_->serialize(nullptr, MathStruct::SerializationStyle::plain_old_vfm, MathStruct::SerializationSpecial::none, 0, 0) : "<NULLPTR>";
   auto to_ser = to_ ? to_->serialize(nullptr, MathStruct::SerializationStyle::plain_old_vfm, MathStruct::SerializationSpecial::none, 0, 0) : "<NULLPTR>";

   if (from_ && to_ && from_ser == to_ser) {
      auto from_copy = from_->copy();
      auto to_copy = to_->copy();

      from_copy->setPrintFullCorrectBrackets(true);
      to_copy->setPrintFullCorrectBrackets(true);

      from_ser = from_copy->serialize(nullptr, MathStruct::SerializationStyle::plain_old_vfm, MathStruct::SerializationSpecial::none, 0, 0);
      to_ser = to_copy->serialize(nullptr, MathStruct::SerializationStyle::plain_old_vfm, MathStruct::SerializationSpecial::none, 0, 0);
   }

   if (convert_back_to_variables) {
      from_ser = convertMetasToVariables(from_ser);
      to_ser = convertMetasToVariables(to_ser);
   }

   return
      (from_ ? from_ser : "NULLPTR")
      + " " + FROM_TO_SYMBOL + " " +
      (to_ ? to_ser : "NULLPTR")
      + cond
      + (include_stage 
         ? " " + STAGE_SERIALIZATION_DELIMITER + std::to_string(stage_)
         : "");
}

void MetaRule::convertVeriablesToMetas()
{
   std::map<std::string, int> map{};
   int count{ 0 };

   auto func = [&map, &count](const std::shared_ptr<MathStruct> m) {
      if (m->isTermVar()) {
         auto t = m->toVariableIfApplicable();
         auto name = t->getVariableName();
         int num = insertOrNotAndReturn(map, count, name);
         auto replace = _meta_simplification(_val(num));
         t->getFather()->replaceOperand(t, replace);
      }
   };

   convertSingleTerm(from_, map, count, func);
   convertSingleTerm(to_, map, count, func);

   for (auto& pair : under_conditions_) {
      convertSingleTerm(pair.first, map, count, func);
   }
}

bool vfm::MetaRule::containsModifyingCondition() const
{
   for (const auto& cond : getConditions()) {
      if (isConditionModifying(cond.second)) {
         return true;
      }
   }

   return false;
}

bool MetaRule::parseProgram(const std::string& program_raw)
{
   std::string program{ program_raw };
   if (StaticHelper::stringContains(program_raw, STAGE_SERIALIZATION_DELIMITER)) { // If the program contains a stage denoter, set the stage and remove from program.
      const std::vector<std::string> pair{ StaticHelper::split(program_raw, STAGE_SERIALIZATION_DELIMITER) };
      assert(pair.size() == 2); // STAGE_SERIALIZATION_DELIMITER needs to be singular in the program, and followed only by a single number.
      program = pair.at(0);
      StaticHelper::trim(program);
      stage_ = std::stoi(pair.at(1));
   }

   const int pos_begin_local_conds = StaticHelper::indexOfFirstInnermostBeginBracket(program, OPENING_BRACKET_LOCAL_CONDITIONS, CLOSING_BRACKET_LOCAL_CONDITIONS);
   const int pos_begin_global_cond = StaticHelper::indexOfFirstInnermostBeginBracket(program, OPENING_BRACKET_GLOBAL_CONDITION, CLOSING_BRACKET_GLOBAL_CONDITION);
   const int pos_end_local_conds = StaticHelper::findMatchingEndTagLevelwise(program, OPENING_BRACKET_LOCAL_CONDITIONS, CLOSING_BRACKET_LOCAL_CONDITIONS);  // Start at 0, since we only expect one pair of brackets.
   const int pos_end_global_cond = StaticHelper::findMatchingEndTagLevelwise(program, OPENING_BRACKET_GLOBAL_CONDITION, CLOSING_BRACKET_GLOBAL_CONDITION); // Start at 0, since we only expect one pair of braces.

   const std::string local_conditions = pos_begin_local_conds >= 0
      ? program.substr(pos_begin_local_conds + OPENING_BRACKET_LOCAL_CONDITIONS.size(), pos_end_local_conds - pos_begin_local_conds - OPENING_BRACKET_LOCAL_CONDITIONS.size())
      : "";

   const std::string global_condition = pos_begin_global_cond >= 0
      ? program.substr(pos_begin_global_cond + OPENING_BRACKET_LOCAL_CONDITIONS.size(), pos_end_global_cond - pos_begin_global_cond - OPENING_BRACKET_GLOBAL_CONDITION.size())
      : ConditionTypeWrapper(ConditionType::no_check).getEnumAsString();

   std::string from_to;

   if (pos_begin_local_conds >= 0) {
      if (pos_begin_global_cond >= 0) { // Both are >= 0.
         from_to = program.substr(0, std::min(pos_begin_local_conds, pos_begin_global_cond));
      }
      else { // Only local is >= 0.
         from_to = program.substr(0, pos_begin_local_conds);
      }
   }
   else if (pos_begin_global_cond >= 0) { // Only global is >= 0.
      from_to = program.substr(0, pos_begin_global_cond);
   }
   else { // Both are < 0.
      from_to = program;
   }

   auto split = StaticHelper::split(from_to, FROM_TO_SYMBOL);

   if (split.size() != 2) {
      addFatalError("Expected main rule part '" + from_to + "' of meta rule description '" + program + "' to contain rule transition symbol '" + FROM_TO_SYMBOL + "', but none found.");
   }

   from_ = MathStruct::parseMathStruct(split[0], parser_)->toTermIfApplicable();
   to_ = MathStruct::parseMathStruct(split[1], parser_)->toTermIfApplicable();
   global_condition_ = ConditionTypeWrapper(StaticHelper::replaceAll(global_condition, QUOTE_SYMBOL_FOR_CONTITION_NAME, "")).getEnum();
   under_conditions_.clear();

   if (!local_conditions.empty()) {
      for (const auto& local_cond : StaticHelper::split(local_conditions, QUOTE_SYMBOL_FOR_CONTITION_NAME + DELIMITER_LOCAL_CONDITIONS)) {
         auto split = StaticHelper::split(local_cond, DELIMITER_LOCAL_CONDITIONS_AFTER_TERM);
         auto term = split[0];
         auto condition = split[1];

         under_conditions_.push_back({ MathStruct::parseMathStruct(term, parser_)->toTermIfApplicable(), ConditionTypeWrapper(
            StaticHelper::replaceAll(StaticHelper::removeWhiteSpace(condition), QUOTE_SYMBOL_FOR_CONTITION_NAME, "")).getEnum() });
      }
   }

   return from_ && to_;
}

bool MetaRule::isNegationOf(const TermPtr t0_raw, const TermPtr t1_raw)
{
   auto t0{ t0_raw->thisPtrGoIntoCompound() };
   auto t1{ t1_raw->thisPtrGoIntoCompound() };

   bool neg0{ t0->getOptor() == SYMB_NOT };
   bool neg1{ t1->getOptor() == SYMB_NOT };

   if (neg0 == neg1) {
      return false;
   }

   auto tt0{ t0 };
   auto tt1{ t1 };

   if (tt0->getOptor() == SYMB_BOOLIFY) {
      tt0 = tt0->getTermsJumpIntoCompounds()[0];
   }
   if (tt1->getOptor() == SYMB_BOOLIFY) {
      tt1 = tt1->getTermsJumpIntoCompounds()[0];
   }

   return neg0 && tt0->getOperands()[0]->isStructurallyEqual(tt1)
      || neg1 && tt1->getOperands()[0]->isStructurallyEqual(tt0);
}

bool MetaRule::first_two_variables_are_equal_except_reference(const std::shared_ptr<MathStruct> term_raw) {
   auto term{ term_raw->thisPtrGoIntoCompound() };

   if (term->getOperands().size() < 2) {
      return false;
   }

   auto t0{ term->getOperands()[0]->toVariableIfApplicable() };
   auto t1{ term->getOperands()[1]->toVariableIfApplicable() };

   return t0 && t1 && StaticHelper::cleanVarNameOfPossibleRefSymbol(t0->getVariableName()) == StaticHelper::cleanVarNameOfPossibleRefSymbol(t1->getVariableName());
}

bool MetaRule::second_operand_contained_in_first_operand(const std::shared_ptr<MathStruct> term_raw)
{
   auto term{ term_raw->thisPtrGoIntoCompound() };

   if (term->getOperands().size() >= 2) {
      const auto a0{ term->getOperands()[0] };
      const auto a1_var{ term->getOperands()[1]->toVariableIfApplicable() };
      const auto a1{ a1_var ? _var(StaticHelper::cleanVarNameOfPossibleRefSymbol(a1_var->getVariableName())) : term->getOperands()[1] };
      bool contained{ false };

      a0->applyToMeAndMyChildrenIterative([&contained, &a1](const MathStructPtr m) {
         contained = m->isStructurallyEqual(a1);
      }, TraverseCompoundsType::avoid_compound_structures, contained);

      return contained;
   }

   return false;
}

bool MetaRule::second_appears_as_read_variable_in_first(const std::shared_ptr<MathStruct> term_raw)
{
   auto term{ term_raw->thisPtrGoIntoCompound() };

   if (term->getOperands().size() < 2) {
      return false;
   }

   auto t1{ term->getOperands()[1]->toVariableIfApplicable() };

   if (!t1) {
      return false;
   }

   std::string var_name{ StaticHelper::cleanVarNameOfPossibleRefSymbol(t1->getVariableName()) };
   bool contains{ false };

   term->getOperands()[0]->applyToMeAndMyChildrenIterative([&contains, &var_name](const MathStructPtr m) {
      auto m_var{ m->toVariableIfApplicable() };
      contains = m_var && m_var->isTermVarNotOnLeftSideOfAssignment() && m_var->getVariableName() == var_name;
   }, TraverseCompoundsType::avoid_compound_structures, contains);

   return contains;
}

bool MetaRule::is_any_var_of_second_on_any_left_side_of_assignment_in_first(const std::shared_ptr<MathStruct> term_raw)
{
   auto term{ term_raw->thisPtrGoIntoCompound() };

   if (term->getOperands().size() < 2) {
      return false;
   }

   TermPtr t0{ term->getOperands()[0] };
   TermPtr t1{ term->getOperands()[1] };
   bool contains{ false };

   std::set<std::string> vars{};

   t1->applyToMeAndMyChildrenIterative([&vars](const MathStructPtr m) {
      auto m_var{ m->toVariableIfApplicable() };

      if (m_var) {
         vars.insert(m_var->getVariableName()); // No cleaning of ref symbol here, assuming we're not on the left side of an assignment.
      }
   }, TraverseCompoundsType::avoid_compound_structures);

   t0->applyToMeAndMyChildrenIterative([&contains, &vars](const MathStructPtr m) {
      contains = m->isTermVarOnLeftSideOfAssignment() && vars.count(StaticHelper::cleanVarNameOfPossibleRefSymbol(m->getOptor()));
   }, TraverseCompoundsType::avoid_compound_structures, contains);

   return contains;
}

TermPtr goUpThroghIFsAndSequences(const MathStructPtr father, const MathStructPtr grandad, bool skip_first_check)
{
   auto flex_grandad{ grandad }; // Sequence operation, 'a = term' is part of.
   auto flex_father{ father };

   while (skip_first_check || flex_grandad->child0JumpIntoCompounds() == flex_father) { // We're still the left child, i.e., beginning of a sequence. // TODO: Check in first iteration unnecessary.
      skip_first_check = false;

      while (flex_grandad && flex_grandad->isTermSequence() && flex_grandad->getFather()) {
         flex_father = flex_grandad;
         flex_grandad = flex_grandad->getFather()->getFather();
      }

      if (flex_grandad && (flex_grandad->isTermIf() || flex_grandad->isTermIfelse())) {
         while (flex_grandad && (flex_grandad->isTermIf() || flex_grandad->isTermIfelse())) {
            flex_father = flex_grandad;
            flex_grandad = flex_grandad->getFather()->getFather();
         }

         if (flex_grandad && flex_grandad->isTermSequence()) {
            if (flex_grandad->child1JumpIntoCompounds() == flex_father) { // We're the right child, so we just need to find the predecessor.
               if (!(flex_grandad->child0JumpIntoCompounds()->isTermSetVarOrAssignment() || flex_grandad->child0JumpIntoCompounds()->isTermSequence())) {
                  return nullptr; // Don't go over IF in sequence.
               }

               return flex_grandad->child0JumpIntoCompounds();
            }
            else { // We're the left child, so go on...
               // No action needed.
            }
         }
         else {
            return nullptr; // We're at the beginning of a sequence that we cannot get "out of".
         }
      }
      else {
         return nullptr; // We're at the beginning of a sequence that we cannot get "out of".
      }
   }

   return nullptr;
}

/// <summary>
/// Assume a term b as below (no check will be performed, so we'll get a crash for other types of terms):
///    - ...; A; if (>>> b <<<) {...}; ...                  OR
///    - ...; A; if (>>> b <<<) {...} {...}; ...            OR
///    - ...; A; a = >>> b <<<; ...                         OR
///    - ...; A; if (...) { a = >>> b <<<; ...}; ...        OR
///    - ...; A; if (...) { a = >>> b <<<; ...} {...}; ...  OR
///    - ...; A; if (...) {...} { a = >>> b <<<; ...}; ...
/// Then A is taken as the first argument of a subequent call to 'is_any_var_of_second_on_any_left_side_of_assignment_in_first_AND_INSERT' 
/// (i.e., as the starting point for propagating backwards to find variables to be inlined). If A is missing, false will be returned.
/// </summary>
/// <param name="term">A term b as in the summary.</param>
/// <returns>Iff any variable in b has been replaced by an inlined substitute.</returns>
bool MetaRule::is_any_var_of_second_on_any_left_side_earlier_AND_INSERT(const std::shared_ptr<MathStruct> term)
{
   auto father = term->getFather()->thisPtrGoUpToCompound();    // Assume exists.
   auto grandad = father->getFather()->thisPtrGoUpToCompound(); // Can be null.

   if (grandad) {
      TermPtr result = nullptr;

      if (father->isTermIf() || father->isTermIfelse()) { // We're the condition of an IF clause (otherwise it would have been a "=" as required by the function interface).
         if ((grandad->isTermSequence() && grandad->child0JumpIntoCompounds() == father) // We're an IF statement at the beginning of a sequence.
            || grandad->isTermIf() || grandad->isTermIfelse()) { // We're a purely nested IF (note that this situation is usually simplified away).
            result = goUpThroghIFsAndSequences(father, grandad, true);
         }
         else if (grandad->isTermSequence() && grandad->child0JumpIntoCompounds() != father) {
            if (!(grandad->child0JumpIntoCompounds()->isTermSetVarOrAssignment() || grandad->child0JumpIntoCompounds()->isTermSequence())) {
               return false; // Don't go over IF in sequence.
            }

            result = grandad->child0JumpIntoCompounds();
         }
      }
      else if (father->isTermSetVarOrAssignment()) { // We're on the right side of an assignment (because this is required by the function interface).
         if (grandad->isTermSequence()) {
            if (grandad->child1JumpIntoCompounds() == father) { // We're the right child, i.e., NOT beginning of a sequence, so we just need to find the predecessor.
               if (!(grandad->child0JumpIntoCompounds()->isTermSetVarOrAssignment() || grandad->child0JumpIntoCompounds()->isTermSequence())) {
                  return false; // Don't go over IF in sequence.
               }

               result = grandad->child0JumpIntoCompounds();
            }
            else { // We're the left child, i.e., beginning of a sequence.
               result = goUpThroghIFsAndSequences(father, grandad, false);
            }
         }
         else if (grandad->isTermIf() || grandad->isTermIfelse()) { // We're the ONLY assignment in an IF or ELSE body.
            result = goUpThroghIFsAndSequences(father, grandad, true);
         }
      }

      if (result) {
         return is_any_var_of_second_on_any_left_side_of_assignment_in_first_AND_INSERT(_seq_plain(result, term->toTermIfApplicable()));
      }
   }

   return false;
}

bool MetaRule::is_any_var_of_second_on_any_left_side_of_assignment_in_first_AND_INSERT(const std::shared_ptr<MathStruct> term)
{
   if (term->getTermsJumpIntoCompounds().size() < 2) {
      return false;
   }

   TermPtr t0{ term->getTermsJumpIntoCompounds()[0] };
   TermPtr t1{ term->getTermsJumpIntoCompounds()[1] };
   bool contains{ false };
   bool abort_dummy{ false };

   std::map<std::string, std::vector<TermPtr>> vars{};

   t1->applyToMeAndMyChildrenIterative([&vars](const MathStructPtr m) {
      auto m_var{ m->toVariableIfApplicable() };

      if (m_var) {
         auto it{ vars.insert({ m_var->getVariableName(), {} }) };
         it.first->second.push_back(m_var); // No cleaning of ref symbol here, assuming we're not on the left side of an assignment.
      }
   }, TraverseCompoundsType::avoid_compound_structures);

   auto func = [&contains, &vars, &abort_dummy](const MathStructPtr m) {
      // We're at "@a = x" by definition of applyToMeAndMyChildrenIterativeReverse
      auto m_var_name{ m->getTermsJumpIntoCompounds()[0]->toVariableIfApplicable()->getVariableName() };

      if (m_var_name[0] == SYMB_REF[0]) { // Only if it's "@a = x", not "a = x".
         m_var_name = m_var_name.substr(SYMB_REF.size());
         auto it{ vars.find(m_var_name) };

         if (it != vars.end()) {
            auto right_side{ m->getTermsJumpIntoCompounds()[1] };
            bool assigned_var_on_right_side{ false };

            right_side->applyToMeAndMyChildrenIterative([&assigned_var_on_right_side, &m_var_name](const MathStructPtr m2) {
               auto m2_var{ m2->toVariableIfApplicable() };
               assigned_var_on_right_side = m2_var ? m2_var->getVariableName() == m_var_name : assigned_var_on_right_side;
            }, TraverseCompoundsType::avoid_compound_structures, assigned_var_on_right_side);

            if (!assigned_var_on_right_side) { // If it's "@a = a + 1", don't do it.
               contains = true;

               for (const auto& el : it->second) {
                  el->replace(right_side->copy());
               }
            }

            vars.erase(it); // We need to erase "a" in any case, since there was definitely a setter for "a".
         }
      }
      else {
         abort_dummy = true; // Abort on "a = x", since we don't know which variable might be affected by assigning the memory at "a".
      }
   };

   t0->applyToMeAndMyChildrenIterativeReversePreorder(func, TraverseCompoundsType::avoid_compound_structures, abort_dummy);

   TermPtr tmp_ptr{ t0 };
   while (!tmp_ptr->isRootTerm()) {
      tmp_ptr = tmp_ptr->getFather()->toTermIfApplicable();

      if (tmp_ptr->isTermSequence() && (tmp_ptr->child0()->child1()->isTermIf() || tmp_ptr->child0()->child1()->isTermIfelse())) {
         abort_dummy = false;
         auto child{ tmp_ptr->child0()->child0() };
         child->applyToMeAndMyChildrenIterativeReversePreorder(func, TraverseCompoundsType::avoid_compound_structures, abort_dummy);
      }
   }

   return contains;
}

constexpr float EPSILON{ 0.0001 };

bool MetaRule::is_constant(const std::shared_ptr<MathStruct> term) { return term->isOverallConstant(); }
bool MetaRule::consists_purely_of_numbers(const std::shared_ptr<MathStruct> term) { return term->consistsPurelyOfNumbers(); }
bool MetaRule::is_not_constant(const std::shared_ptr<MathStruct> term) { return !term->isOverallConstant(); }
bool MetaRule::is_constant_and_evaluates_to_true(const std::shared_ptr<MathStruct> term) { return term->isOverallConstant() && term->constEval(); }
bool MetaRule::is_constant_and_evaluates_to_false(const std::shared_ptr<MathStruct> term) { return term->isOverallConstant() && !term->constEval(); }
bool MetaRule::is_constant_and_evaluates_to_positive(const std::shared_ptr<MathStruct> term) { return term->isOverallConstant() && term->constEval() > 0; }
bool MetaRule::is_constant_and_evaluates_to_negative(const std::shared_ptr<MathStruct> term) { return term->isOverallConstant() && term->constEval() < 0; }
bool MetaRule::is_constant_0(const std::shared_ptr<MathStruct> term) { return term->isOverallConstant() && term->constEval() == 0; }
bool MetaRule::is_constant_1(const std::shared_ptr<MathStruct> term) { return term->isOverallConstant() && term->constEval() == 1; }
bool MetaRule::is_non_boolean_constant(const std::shared_ptr<MathStruct> term) { return term->isOverallConstant() && term->constEval() != 1 && term->constEval() != 0; }
bool MetaRule::is_not_non_boolean_constant(const std::shared_ptr<MathStruct> term) { return !term->isOverallConstant() || term->constEval() == 1 || term->constEval() == 0; }
bool MetaRule::is_constant_integer(const std::shared_ptr<MathStruct> term) { return is_constant(term) && StaticHelper::isFloatInteger(term->constEval()); }
bool MetaRule::is_constant_non_integer(const std::shared_ptr<MathStruct> term) { return is_constant(term) && !StaticHelper::isFloatInteger(term->constEval()); }
bool MetaRule::has_no_sideeffects(const std::shared_ptr<MathStruct> term) { return !term->hasSideeffects(); }
bool MetaRule::has_sideeffects(const std::shared_ptr<MathStruct> term) { return term->hasSideeffects(); }
bool MetaRule::is_leaf(const std::shared_ptr<MathStruct> term) { return term->getTermsJumpIntoCompounds().empty(); }
bool MetaRule::is_not_leaf(const std::shared_ptr<MathStruct> term) { return !term->getTermsJumpIntoCompounds().empty(); }
bool MetaRule::is_variable(const std::shared_ptr<MathStruct> term) { return term->isTermVar(); }
bool MetaRule::is_not_variable(const std::shared_ptr<MathStruct> term) { return !is_variable(term); }
bool MetaRule::has_boolean_result(const std::shared_ptr<MathStruct> term) { return term->hasBooleanResult(); }
bool MetaRule::has_non_boolean_result(const std::shared_ptr<MathStruct> term) { return !term->hasBooleanResult(); }
bool MetaRule::of_first_two_operands_second_is_alphabetically_above(const std::shared_ptr<MathStruct> term) { return term->getTermsJumpIntoCompounds().size() >= 2 && term->getTermsJumpIntoCompounds()[0]->getOptor().compare(term->getTermsJumpIntoCompounds()[1]->getOptor()) > 0; }
bool MetaRule::first_two_operands_are_negations_of_each_other(const std::shared_ptr<MathStruct> term) { return term->getTermsJumpIntoCompounds().size() >= 2 && isNegationOf(term->getTermsJumpIntoCompounds()[0], term->getTermsJumpIntoCompounds()[1]); }
bool MetaRule::first_two_variables_are_not_equal_except_reference(const std::shared_ptr<MathStruct> term) { return !first_two_variables_are_equal_except_reference(term); }
bool MetaRule::not_is_root(const std::shared_ptr<MathStruct> term) { return !term->isRootTerm(); }
bool MetaRule::second_operand_not_contained_in_first_operand(const std::shared_ptr<MathStruct> term) { return !second_operand_contained_in_first_operand(term); }
bool MetaRule::second_appears_not_as_read_variable_in_first(const std::shared_ptr<MathStruct> term) { return !second_appears_as_read_variable_in_first(term); }
bool MetaRule::is_no_var_of_second_on_any_left_side_of_assignment_in_first(const std::shared_ptr<MathStruct> term) { return !is_any_var_of_second_on_any_left_side_of_assignment_in_first(term); }
bool MetaRule::is_commutative_operation(const std::shared_ptr<MathStruct> term) { return term->thisPtrGoIntoCompound()->getOpStruct().commut; }
bool MetaRule::is_not_commutative_operation(const std::shared_ptr<MathStruct> term) { return !is_commutative_operation(term); }
bool MetaRule::is_associative_operation(const std::shared_ptr<MathStruct> term) { return term->thisPtrGoIntoCompound()->getOpStruct().assoc; }
bool MetaRule::is_not_associative_operation(const std::shared_ptr<MathStruct> term) { return !is_associative_operation(term); }
bool MetaRule::is_comparator_or_equality_operation(const std::shared_ptr<MathStruct> term) { const std::string optor = term->getOptorOnCompoundLevel(); return optor == SYMB_EQ || optor == SYMB_NEQ || optor == SYMB_SM || optor == SYMB_SMEQ || optor == SYMB_GR || optor == SYMB_GREQ; }
bool MetaRule::is_not_comparator_or_equality_operation(const std::shared_ptr<MathStruct> term) { return !is_comparator_or_equality_operation(term); }
bool MetaRule::contains_meta(const std::shared_ptr<MathStruct> term) { return term->containsMetaCompound(); }
bool MetaRule::contains_no_meta(const std::shared_ptr<MathStruct> term) { return !contains_meta(term); }
bool MetaRule::parent_is_get(const std::shared_ptr<MathStruct> term) { return term->getFather()->getOptor() == SYMB_GET_VAR; }
bool MetaRule::parent_is_not_get(const std::shared_ptr<MathStruct> term) { return !parent_is_get(term); }
bool MetaRule::no_check(const std::shared_ptr<MathStruct> term) { return true; }

bool MetaRule::checkCondition(const ConditionType mode, const std::shared_ptr<MathStruct> term)
{
   return mode == ConditionType::is_constant && is_constant(term)
      || mode == ConditionType::consists_purely_of_numbers && consists_purely_of_numbers(term)
      || mode == ConditionType::is_not_constant && is_not_constant(term)
      || mode == ConditionType::is_constant_and_evaluates_to_true && is_constant_and_evaluates_to_true(term)
      || mode == ConditionType::is_constant_and_evaluates_to_false && is_constant_and_evaluates_to_false(term)
      || mode == ConditionType::is_constant_and_evaluates_to_positive && is_constant_and_evaluates_to_positive(term)
      || mode == ConditionType::is_constant_and_evaluates_to_negative && is_constant_and_evaluates_to_negative(term)
      || mode == ConditionType::is_constant_0 && is_constant_0(term)
      || mode == ConditionType::is_constant_1 && is_constant_1(term)
      || mode == ConditionType::is_non_boolean_constant && is_non_boolean_constant(term)
      || mode == ConditionType::is_not_non_boolean_constant && is_not_non_boolean_constant(term)
      || mode == ConditionType::is_constant_integer && is_constant_integer(term)
      || mode == ConditionType::is_constant_non_integer && is_constant_non_integer(term)
      || mode == ConditionType::has_no_sideeffects && has_no_sideeffects(term)
      || mode == ConditionType::has_sideeffects && has_sideeffects(term)
      || mode == ConditionType::is_leaf && is_leaf(term)
      || mode == ConditionType::is_not_leaf && is_not_leaf(term)
      || mode == ConditionType::is_variable && is_variable(term)
      || mode == ConditionType::is_not_variable && is_not_variable(term)
      || mode == ConditionType::has_boolean_result && has_boolean_result(term)
      || mode == ConditionType::has_non_boolean_result && has_non_boolean_result(term)
      || mode == ConditionType::of_first_two_operands_second_is_alphabetically_above && of_first_two_operands_second_is_alphabetically_above(term)
      || mode == ConditionType::first_two_operands_are_negations_of_each_other && first_two_operands_are_negations_of_each_other(term)
      || mode == ConditionType::not_is_root && not_is_root(term)
      || mode == ConditionType::second_operand_contained_in_first_operand && second_operand_contained_in_first_operand(term)
      || mode == ConditionType::second_operand_not_contained_in_first_operand && second_operand_not_contained_in_first_operand(term)
      || mode == ConditionType::first_two_variables_are_equal_except_reference && first_two_variables_are_equal_except_reference(term)
      || mode == ConditionType::first_two_variables_are_not_equal_except_reference && first_two_variables_are_not_equal_except_reference(term)
      || mode == ConditionType::second_appears_as_read_variable_in_first && second_appears_as_read_variable_in_first(term)
      || mode == ConditionType::second_appears_not_as_read_variable_in_first && second_appears_not_as_read_variable_in_first(term)
      || mode == ConditionType::is_any_var_of_second_on_any_left_side_of_assignment_in_first && is_any_var_of_second_on_any_left_side_of_assignment_in_first(term)
      || mode == ConditionType::is_any_var_of_second_on_any_left_side_of_assignment_in_first_AND_INSERT && is_any_var_of_second_on_any_left_side_of_assignment_in_first_AND_INSERT(term)
      || mode == ConditionType::is_any_var_of_second_on_any_left_side_earlier_AND_INSERT && is_any_var_of_second_on_any_left_side_earlier_AND_INSERT(term)
      || mode == ConditionType::is_no_var_of_second_on_any_left_side_of_assignment_in_first && is_no_var_of_second_on_any_left_side_of_assignment_in_first(term)
      || mode == ConditionType::is_commutative_operation && is_commutative_operation(term)
      || mode == ConditionType::is_not_commutative_operation && is_not_commutative_operation(term)
      || mode == ConditionType::is_associative_operation && is_associative_operation(term)
      || mode == ConditionType::is_not_associative_operation && is_not_associative_operation(term)
      || mode == ConditionType::is_comparator_or_equality_operation && is_comparator_or_equality_operation(term)
      || mode == ConditionType::is_not_comparator_or_equality_operation && is_not_comparator_or_equality_operation(term)
      || mode == ConditionType::contains_meta && contains_meta(term)
      || mode == ConditionType::contains_no_meta && contains_no_meta(term)
      || mode == ConditionType::parent_is_get && parent_is_get(term)
      || mode == ConditionType::parent_is_not_get && parent_is_not_get(term)
      || mode == ConditionType::no_check;
}

std::string MetaRule::convertString(const std::string str, const std::map<std::string, std::string> real_op_names)
{
   std::string res = str;

   for (const auto& real_op_name : real_op_names) {
      res = StaticHelper::replaceAll(res, real_op_name.first, real_op_name.second);
   }

   return res;
}

std::vector<std::pair<std::shared_ptr<Term>, ConditionType>> MetaRule::convertStringVec(
   const std::vector<std::pair<std::string, ConditionType>> vec,
   const std::map<std::string, std::string> real_op_names,
   const std::shared_ptr<FormulaParser> parser)
{
   std::vector<std::pair<std::shared_ptr<Term>, ConditionType>> vec2;

   for (const auto& pair : vec) {
      std::string new_cond = pair.first;

      for (const auto& real_op_name : real_op_names) {
         new_cond = StaticHelper::replaceAll(pair.first, real_op_name.first, real_op_name.second);
      }

      vec2.push_back(std::pair<std::shared_ptr<Term>, ConditionType>(
         MathStruct::parseMathStruct(new_cond, false, false, parser)->toTermIfApplicable(), pair.second)); // Cannot use Cpp syntax for performance reasons.
   }

   return vec2;
}

int MetaRule::insertOrNotAndReturn(std::map<std::string, int>& map, int& count, const std::string& name)
{
   auto pair{ map.find(name) };

   if (pair == map.end()) {
      map.insert(std::pair<std::string, int>(name, count));
      pair = map.find(name);
      count++;
   }

   return pair->second;
}

void MetaRule::convertSingleTerm(
   std::shared_ptr<Term>& t,
   std::map<std::string, int>& map,
   int& count,
   const std::function<void(std::shared_ptr<MathStruct>)>& func)
{
   if (t->isTermVar()) {
      auto name = t->toVariableIfApplicable()->getVariableName();
      t = _meta_simplification(_val(insertOrNotAndReturn(map, count, name)));
   }
   else {
      t->applyToMeAndMyChildren(func);
   }
}

bool vfm::MetaRule::isConditionModifying(const std::string& name)
{
   return StaticHelper::toLowerCase(name) != name; // Def: A condition is modifying iff its name contains capital letters.
}

bool vfm::MetaRule::isConditionModifying(const ConditionTypeWrapper& cond)
{
   return isConditionModifying(cond.getEnumAsString());
}

bool vfm::MetaRule::isConditionModifying(const ConditionType& cond)
{
   return isConditionModifying(ConditionTypeWrapper(cond));
}

bool vfm::MetaRule::operator<(const MetaRule& right) const
{
   return serialize(false) < right.serialize(false);
}

void vfm::MetaRule::setRuleAbandoned() const
{
   rule_abandoned_ = true;
}

bool vfm::MetaRule::isRuleAbandoned() const
{
   return rule_abandoned_;
}

int vfm::MetaRule::getStage() const
{
   return stage_;
}

void vfm::MetaRule::setStage(const int stage) const
{
   stage_ = stage;
}

int vfm::MetaRule::countMetaRules(const std::map<std::string, std::map<int, std::set<MetaRule>>>& rules)
{
   int cnt{};

   for (const auto& rules_by_op : rules) {
      for (const auto& rules_by_op_arg_num : rules_by_op.second) {
         for (const auto& rule : rules_by_op_arg_num.second) {
            cnt++;
         }
      }
   }

   return cnt;
}

RulesByStage vfm::MetaRule::getMetaRulesByStage(const Rules& rules)
{
   std::map<int, std::map<std::string, std::map<int, std::set<MetaRule>>>> res{};

   // Insert all rules.
   for (const auto& rules_by_op : rules) {
      for (const auto& rules_by_op_arg_num : rules_by_op.second) {
         for (const auto& rule : rules_by_op_arg_num.second) {
            res.insert({ rule.stage_, {} });
            res.at(rule.stage_).insert({ rules_by_op.first, {} });
            res.at(rule.stage_).at(rules_by_op.first).insert({ rules_by_op_arg_num.first, {} });
            res.at(rule.stage_).at(rules_by_op.first).at(rules_by_op_arg_num.first).insert(rule);
         }
      }
   }

   // Insert empty slots to capture overloaded operators, even if not every overload has a rule.
   for (const auto& stage : res) {
      for (const auto& rules_by_op : rules) {
         res.at(stage.first).insert({ rules_by_op.first, {} });

         for (const auto& rules_by_op_arg_num : rules_by_op.second) {
            res.at(stage.first).at(rules_by_op.first).insert({ rules_by_op_arg_num.first, {} });
         }
      }
   }

   return res;
}
