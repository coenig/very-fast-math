//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file

#include "term_logic_or.h"
#include "meta_rule.h"
#include "parser.h"
#include <functional>

using namespace vfm;

const OperatorStructure TermLogicOr::my_struct(OutputFormatEnum::infix, AssociativityTypeEnum::left, OPNUM_OR, PRIO_OR, SYMB_OR, true, true);

OperatorStructure TermLogicOr::getOpStruct() const
{
   return my_struct;
}

float TermLogicOr::eval(const std::shared_ptr<DataPack>& varVals, const std::shared_ptr<FormulaParser> parser, const bool suppress_sideeffects)
{
#if defined(ASMJIT_ENABLED)
   if (assembly_created_ == AssemblyState::yes) return (*fast_eval_func_)();
#endif

   // Last three arguments: Invert only if one operand nan and another one finite and false.
   return evalSimple(varVals, std::logical_or<float>(), 0, 0, parser, suppress_sideeffects, true, false, true);
}

std::set<MetaRule> vfm::TermLogicOr::getSimplificationRules(const std::shared_ptr<FormulaParser> parser) const
{
   std::set<MetaRule> vec = MathStruct::getSimplificationRules(parser);
   auto cond1 = std::vector<std::pair<std::shared_ptr<Term>, ConditionType>>{{_varb(), ConditionType::is_non_boolean_constant}};
   auto cond2 = std::vector<std::pair<std::shared_ptr<Term>, ConditionType>>{{_vara(), ConditionType::is_non_boolean_constant}};

   vec.insert(MetaRule(_or(_vara(), _varb()), _or(_vara(), _boolify(_varb())), SIMPLIFICATION_STAGE_MAIN, cond1)); // "a || b", "a || bool(b)" [b is non-bool const]
   vec.insert(MetaRule(_or(_vara(), _varb()), _or(_boolify(_vara()), _varb()), SIMPLIFICATION_STAGE_MAIN, cond2)); // "a || b", "bool(a) || b" [a is non-bool const]
   vec.insert(MetaRule(_or(_vara(), _varb()), _true(), SIMPLIFICATION_STAGE_MAIN, { { _seq(_vara(), _varb()), ConditionType::first_two_operands_are_negations_of_each_other } }, ConditionType::has_no_sideeffects)); // "a || !a", "true"
   vec.insert(MetaRule(_or(_not(_vara()), _varb()), _ltl_implication(_vara(), _varb()), SIMPLIFICATION_STAGE_MAIN)); // "!a || b", "a => b"

   return vec;
}

#if defined(ASMJIT_ENABLED)
std::shared_ptr<x86::Xmm> TermLogicOr::createSubAssembly(asmjit::x86::Compiler& cc, const std::shared_ptr<DataPack>& d, const std::shared_ptr<FormulaParser>& p)
{
   lambda_vno lambda = [](asmjit::x86::Compiler& cc, x86::Xmm& x, x86::Xmm& y) {
      // TODO: This HAS to go easier!
      x86::Xmm z = cc.newXmm();
      setXmmVar(cc, z, y);

      xmmCompareTo(cc, x, 0, cmp_not_equal);
      xmmCompareTo(cc, z, 0, cmp_not_equal);
      cc.addss(x, z);
      xmmCompareTo(cc, x, 0, cmp_not_equal);
   };

   return subAssemblyVarnumOps(cc, d, lambda);
}
#endif

TermLogicOr::TermLogicOr(const std::vector<std::shared_ptr<Term>>& opds) : TermLogic(opds)
{
}

TermLogicOr::TermLogicOr(const std::shared_ptr<Term> opd1, const std::shared_ptr<Term> opd2) : TermLogicOr(std::vector<std::shared_ptr<Term>>{opd1, opd2})
{
}

std::shared_ptr<Term> TermLogicOr::copy(const std::shared_ptr<std::map<std::shared_ptr<Term>, std::shared_ptr<Term>>> already_copied) {
   return copySimple<TermLogicOr>(already_copied);
}

std::string TermLogicOr::getNuSMVOperator() const
{
   return "|";
}

std::string vfm::TermLogicOr::getK2Operator() const
{
   return "or";
}
