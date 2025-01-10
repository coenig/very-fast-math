//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file

#include "term_logic_and.h"
#include "meta_rule.h"
#include "parser.h"
#include <functional>
#include <cmath>

using namespace vfm;

const OperatorStructure TermLogicAnd::my_struct(OutputFormatEnum::infix, AssociativityTypeEnum::left, OPNUM_AND, PRIO_AND, SYMB_AND, true, true);

OperatorStructure TermLogicAnd::getOpStruct() const
{
   return my_struct;
}

float TermLogicAnd::eval(const std::shared_ptr<DataPack>& varVals, const std::shared_ptr<FormulaParser> parser, const bool suppress_sideeffects)
{
#if defined(ASMJIT_ENABLED)
   if (assembly_created_ == AssemblyState::yes) return (*fast_eval_func_)();
#endif

   // Last three arguments: Invert only if one operand nan and another one finite and true.
   return evalSimple(varVals, std::logical_and<float>(), 1, 0, parser, suppress_sideeffects, true, true, false);
}

std::set<MetaRule> vfm::TermLogicAnd::getSimplificationRules(const std::shared_ptr<FormulaParser> parser) const
{
   std::set<MetaRule> vec = MathStruct::getSimplificationRules(parser);
   auto cond1 = std::vector<std::pair<std::shared_ptr<Term>, ConditionType>>{{_varb(), ConditionType::is_non_boolean_constant}};
   auto cond2 = std::vector<std::pair<std::shared_ptr<Term>, ConditionType>>{{_vara(), ConditionType::is_non_boolean_constant}};
   auto cond3 = std::vector<std::pair<std::shared_ptr<Term>, ConditionType>>{{_varb(), ConditionType::is_not_non_boolean_constant}};
   auto cond4 = std::vector<std::pair<std::shared_ptr<Term>, ConditionType>>{{_vara(), ConditionType::is_not_non_boolean_constant}};

   vec.insert(MetaRule(_and(_vara(), _varb()), _and(_vara(), _boolify(_varb())), SIMPLIFICATION_STAGE_MAIN, cond1)); // "a && b", "a && bool(b)" [b is non-bool const]
   vec.insert(MetaRule(_and(_vara(), _varb()), _and(_boolify(_vara()), _varb()), SIMPLIFICATION_STAGE_MAIN, cond2)); // "a && b", "bool(a) && b" [a is non-bool const]

   vec.insert(MetaRule(_and(_vara(), _boolify(_varb())), _and(_vara(), _varb()), SIMPLIFICATION_STAGE_MAIN, cond3)); // "a && bool(b)", "a && b" [b is NOT non-bool const]
   vec.insert(MetaRule(_and(_boolify(_vara()), _varb()), _and(_vara(), _varb()), SIMPLIFICATION_STAGE_MAIN, cond4)); // "bool(a) && b", "a && b" [a is NOT non-bool const]

   vec.insert(MetaRule(_and(_vara(), _varb()), _false(), SIMPLIFICATION_STAGE_MAIN, { { _seq(_vara(), _varb()), ConditionType::first_two_operands_are_negations_of_each_other } }, ConditionType::has_no_sideeffects)); // "a && !a", "false"

   return vec;
}

#if defined(ASMJIT_ENABLED)
std::shared_ptr<x86::Xmm> TermLogicAnd::createSubAssembly(asmjit::x86::Compiler & cc, const std::shared_ptr<DataPack>& d, const std::shared_ptr<FormulaParser>& p)
{
   lambda_vno lambda = [](asmjit::x86::Compiler& cc, x86::Xmm& x, x86::Xmm& y) {
      cc.mulss(x, y);
      xmmCompareTo(cc, x, 0, cmp_not_equal);
   };

   return subAssemblyVarnumOps(cc, d, lambda);
}
#endif

TermLogicAnd::TermLogicAnd(const std::vector<std::shared_ptr<Term>>& opds): TermLogic(opds)
{
}

TermLogicAnd::TermLogicAnd(const std::shared_ptr<Term> opd1, const std::shared_ptr<Term> opd2) : TermLogicAnd(std::vector<std::shared_ptr<Term>>{opd1, opd2})
{
}

std::shared_ptr<Term> TermLogicAnd::copy(const std::shared_ptr<std::map<std::shared_ptr<Term>, std::shared_ptr<Term>>> already_copied) {
   return copySimple<TermLogicAnd>(already_copied);
}

std::string vfm::TermLogicAnd::getNuSMVOperator() const
{
   return "&";
}

std::string vfm::TermLogicAnd::getK2Operator() const
{
   return "and";
}
