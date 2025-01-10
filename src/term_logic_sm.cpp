//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file

#include "term_logic_sm.h"
#include "meta_rule.h"
#include "parser.h"
#include <functional>

using namespace vfm;

const OperatorStructure TermLogicSm::my_struct(OutputFormatEnum::infix, AssociativityTypeEnum::left, OPNUM_SM, PRIO_SM, SYMB_SM, false, false, AdditionalOptionEnum::always_print_brackets);

OperatorStructure TermLogicSm::getOpStruct() const
{
   return my_struct;
}

float TermLogicSm::eval(const std::shared_ptr<DataPack>& varVals, const std::shared_ptr<FormulaParser> parser, const bool suppress_sideeffects)
{
#if defined(ASMJIT_ENABLED)
   if (assembly_created_ == AssemblyState::yes) return (*fast_eval_func_)();
#endif

   return evalSimple(varVals, std::less<float>(), getOperands()[0]->eval(varVals, parser, suppress_sideeffects), 1, parser, suppress_sideeffects, true); // Invert if nan.
}

#if defined(ASMJIT_ENABLED)
std::shared_ptr<x86::Xmm> TermLogicSm::createSubAssembly(asmjit::x86::Compiler& cc, const std::shared_ptr<DataPack>& d, const std::shared_ptr<FormulaParser>& p)
{
   lambda_vno lambda = [](asmjit::x86::Compiler& cc, x86::Xmm& x, x86::Xmm& y) {
      xmmCompareTo(cc, x, y, cmp_less); // FAILS for x < y!
   };

   return subAssemblyVarnumOps(cc, d, lambda);
}
#endif

TermLogicSm::TermLogicSm(const std::vector<std::shared_ptr<Term>>& opds) : TermLogic(opds)
{
}

vfm::TermLogicSm::TermLogicSm(const std::shared_ptr<Term> opd1, const std::shared_ptr<Term> opd2) : TermLogicSm(std::vector<std::shared_ptr<Term>>{opd1, opd2})
{
}

std::shared_ptr<Term> TermLogicSm::copy(const std::shared_ptr<std::map<std::shared_ptr<Term>, std::shared_ptr<Term>>> already_copied) {
   return copySimple<TermLogicSm>(already_copied);
}

std::set<MetaRule> vfm::TermLogicSm::getSimplificationRules(const std::shared_ptr<FormulaParser> parser) const
{
   std::set<MetaRule> vec = MathStruct::getSimplificationRules(parser);

   vec.insert(MetaRule(_sm(_vara(), _vara()), _val0(), SIMPLIFICATION_STAGE_MAIN, {}, ConditionType::has_no_sideeffects)); // "a < a", "0"
   vec.insert(MetaRule(_sm(_neg(_vara()), _neg(_varb())), _gr(_vara(), _varb()), SIMPLIFICATION_STAGE_MAIN));              // "--(a) < --(b)", "a > b"

   return vec;
}

std::string vfm::TermLogicSm::getK2Operator() const
{
   return "lt";
}
