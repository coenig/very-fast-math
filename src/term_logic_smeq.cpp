//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file

#include "term_logic_smeq.h"
#include "meta_rule.h"
#include "parser.h"
#include <functional>

using namespace vfm;

const OperatorStructure TermLogicSmEq::my_struct(OutputFormatEnum::infix, AssociativityTypeEnum::left, OPNUM_SMEQ, PRIO_SMEQ, SYMB_SMEQ, false, false, AdditionalOptionEnum::always_print_brackets);

OperatorStructure TermLogicSmEq::getOpStruct() const
{
   return my_struct;
}

float TermLogicSmEq::eval(const std::shared_ptr<DataPack>& varVals, const std::shared_ptr<FormulaParser> parser, const bool suppress_sideeffects)
{
#if defined(ASMJIT_ENABLED)
   if (assembly_created_ == AssemblyState::yes) return (*fast_eval_func_)();
#endif

   return evalSimple(varVals, std::less_equal<float>(), getOperands()[0]->eval(varVals, parser, suppress_sideeffects), 1, parser, suppress_sideeffects, true); // Invert if nan.
}

#if defined(ASMJIT_ENABLED)
std::shared_ptr<x86::Xmm> TermLogicSmEq::createSubAssembly(asmjit::x86::Compiler& cc, const std::shared_ptr<DataPack>& d, const std::shared_ptr<FormulaParser>& p)
{
   lambda_vno lambda = [](asmjit::x86::Compiler& cc, x86::Xmm& x, x86::Xmm& y) {
      xmmCompareTo(cc, x, y, cmp_less_equal); // FAILS for x < y!
   };

   return subAssemblyVarnumOps(cc, d, lambda);
}
#endif

TermLogicSmEq::TermLogicSmEq(const std::vector<std::shared_ptr<Term>>& opds) : TermLogic(opds)
{
}

vfm::TermLogicSmEq::TermLogicSmEq(const std::shared_ptr<Term> opd1, const std::shared_ptr<Term> opd2) : TermLogicSmEq(std::vector<std::shared_ptr<Term>>{opd1, opd2})
{
}

std::shared_ptr<Term> TermLogicSmEq::copy(const std::shared_ptr<std::map<std::shared_ptr<Term>, std::shared_ptr<Term>>> already_copied) {
   return copySimple<TermLogicSmEq>(already_copied);
}

std::set<MetaRule> vfm::TermLogicSmEq::getSimplificationRules(const std::shared_ptr<FormulaParser> parser) const
{
   std::set<MetaRule> vec = MathStruct::getSimplificationRules(parser);

   vec.insert(MetaRule(_smeq(_vara(), _vara()), _val1(), SIMPLIFICATION_STAGE_MAIN, {}, ConditionType::has_no_sideeffects)); // "a <= a", "1"
   vec.insert(MetaRule(_smeq(_neg(_vara()), _neg(_varb())), _greq(_vara(), _varb()), SIMPLIFICATION_STAGE_MAIN));            // "--(a) <= --(b)", "a >= b"

   return vec;
}

std::string vfm::TermLogicSmEq::getK2Operator() const
{
   return "le";
}
