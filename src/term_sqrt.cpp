//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file

#include "term_sqrt.h"
#include "term_mult.h"
#include "meta_rule.h"
#include "parser.h"

using namespace vfm;

const OperatorStructure TermSQRT::my_struct(OutputFormatEnum::prefix, AssociativityTypeEnum::left, OPNUM_SQRT, PRIO_SQRT, SYMB_SQRT, true, true);

TermSQRT::TermSQRT(const std::vector<std::shared_ptr<Term>>& t) : TermArith(t) {}
TermSQRT::TermSQRT(const std::shared_ptr<Term>& t) : TermSQRT(std::vector<std::shared_ptr<Term>>{t}) {}

std::shared_ptr<Term> TermSQRT::getInvPattern(int argNum)
{
   return MathStruct::parseMathStruct("p_(0) * p_(0)", false)->toTermIfApplicable();
}

OperatorStructure TermSQRT::getOpStruct() const
{
   return my_struct;
}

float TermSQRT::eval(const std::shared_ptr<DataPack>& varVals, const std::shared_ptr<FormulaParser> parser, const bool suppress_sideeffects)
{
#if defined(ASMJIT_ENABLED)
   if (assembly_created_ == AssemblyState::yes) return (*fast_eval_func_)();
#endif

   return sqrtf(getOperands()[0]->eval(varVals, parser, suppress_sideeffects));
}

#if defined(ASMJIT_ENABLED)
std::shared_ptr<x86::Xmm> TermSQRT::createSubAssembly(asmjit::x86::Compiler & cc, const std::shared_ptr<DataPack>& d, const std::shared_ptr<FormulaParser>& p)
{
   lambda_so lambda = [](asmjit::x86::Compiler& cc, x86::Xmm& x) {
      cc.sqrtss(x, x);
   };

   return subAssemblySingleOp(cc, d, lambda);
}
#endif

std::shared_ptr<Term> TermSQRT::copy(const std::shared_ptr<std::map<std::shared_ptr<Term>, std::shared_ptr<Term>>> already_copied) {
   return copySimple<TermSQRT>(already_copied);
}

std::set<MetaRule> TermSQRT::getSimplificationRules(const std::shared_ptr<FormulaParser> parser) const
{
   std::set<MetaRule> vec = MathStruct::getSimplificationRules(parser);

   vec.insert(MetaRule(_sqrt(_mult(_vara(), _vara())), _abs(_vara()), SIMPLIFICATION_STAGE_MAIN, { {_vara(), ConditionType::has_no_sideeffects} })); // "sqrt(a * a)", "abs(a)"

   return vec;
}
