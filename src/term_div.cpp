//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file

#include "term_div.h"
#include "term_mult.h"
#include "meta_rule.h"

using namespace vfm;

const OperatorStructure TermDiv::my_struct(OutputFormatEnum::infix, AssociativityTypeEnum::left, OPNUM_DIV, PRIO_DIV, SYMB_DIV, false, false);

OperatorStructure TermDiv::getOpStruct() const
{
   return my_struct;
}

std::shared_ptr<Term> TermDiv::getInvPattern(int argNum)
{
   return getInvPatternSimpleMinusStyle<TermMult, TermDiv>(argNum);
}

float TermDiv::eval(const std::shared_ptr<DataPack>& varVals, const std::shared_ptr<FormulaParser> parser, const bool suppress_sideeffects)
{
#if defined(ASMJIT_ENABLED)
   if (assembly_created_ == AssemblyState::yes) return (*fast_eval_func_)();
#endif

   return evalSimple(varVals, std::divides<float>(), getOperands()[0]->eval(varVals, parser, suppress_sideeffects), 1, parser, suppress_sideeffects);
}

#if defined(ASMJIT_ENABLED)
std::shared_ptr<x86::Xmm> TermDiv::createSubAssembly(asmjit::x86::Compiler& cc, const std::shared_ptr<DataPack>& d, const std::shared_ptr<FormulaParser>& p)
{
   lambda_vno lambda = [](asmjit::x86::Compiler& cc, x86::Xmm& x, x86::Xmm& y) {cc.divss(x, y); };
   return subAssemblyVarnumOps(cc, d, lambda);
}
#endif

TermDiv::TermDiv(const std::vector<std::shared_ptr<Term>>& opds) : TermArith(opds)
{
}

vfm::TermDiv::TermDiv(const std::shared_ptr<Term> opd1, const std::shared_ptr<Term> opd2) : TermDiv(std::vector<std::shared_ptr<Term>>{opd1, opd2})
{
}

std::shared_ptr<Term> TermDiv::copy(const std::shared_ptr<std::map<std::shared_ptr<Term>, std::shared_ptr<Term>>> already_copied) {
   return copySimple<TermDiv>(already_copied);
}

std::set<MetaRule> vfm::TermDiv::getSimplificationRules(const std::shared_ptr<FormulaParser> parser) const
{
   std::set<MetaRule> vec = MathStruct::getSimplificationRules(parser);

   // TODO: Do we need this?
   //vec.push_back(MetaRule("a / a", "1", "a", ConditionType::has_no_sideeffects));

   return vec;
}

std::string vfm::TermDiv::getK2Operator() const
{
   return "div";
}
