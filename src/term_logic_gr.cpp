//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file

#include "term_logic_gr.h"
#include "meta_rule.h"
#include "parser.h"
#include <functional>

using namespace vfm;

const OperatorStructure TermLogicGr::my_struct(OutputFormatEnum::infix, AssociativityTypeEnum::left, OPNUM_GR, PRIO_GR, SYMB_GR, false, false, AdditionalOptionEnum::always_print_brackets);

OperatorStructure TermLogicGr::getOpStruct() const
{
   return my_struct;
}

float TermLogicGr::eval(const std::shared_ptr<DataPack>& varVals, const std::shared_ptr<FormulaParser> parser, const bool suppress_sideeffects)
{
#if defined(ASMJIT_ENABLED)
   if (assembly_created_ == AssemblyState::yes) return (*fast_eval_func_)();
#endif

   return evalSimple(varVals, std::greater<float>(), getOperands()[0]->eval(varVals, parser, suppress_sideeffects), 1, parser, suppress_sideeffects);
}

#if defined(ASMJIT_ENABLED)
std::shared_ptr<x86::Xmm> TermLogicGr::createSubAssembly(asmjit::x86::Compiler& cc, const std::shared_ptr<DataPack>& d, const std::shared_ptr<FormulaParser>& p)
{
   lambda_vno lambda = [](asmjit::x86::Compiler& cc, x86::Xmm& x, x86::Xmm& y) {
      xmmCompareTo(cc, x, y, cmp_greater); // FAILS for x < y!
   };

   return subAssemblyVarnumOps(cc, d, lambda);
}
#endif

TermLogicGr::TermLogicGr(const std::vector<std::shared_ptr<Term>>& opds) : TermLogic(opds)
{
}

vfm::TermLogicGr::TermLogicGr(const std::shared_ptr<Term> opd1, const std::shared_ptr<Term> opd2) : TermLogicGr(std::vector<std::shared_ptr<Term>>{opd1, opd2})
{
}

std::shared_ptr<Term> TermLogicGr::copy(const std::shared_ptr<std::map<std::shared_ptr<Term>, std::shared_ptr<Term>>> already_copied) {
   return copySimple<TermLogicGr>(already_copied);
}

std::set<MetaRule> vfm::TermLogicGr::getSimplificationRules(const std::shared_ptr<FormulaParser> parser) const
{
   std::set<MetaRule> vec = MathStruct::getSimplificationRules(parser);

   vec.insert(MetaRule(_gr(_vara(), _vara()), _false(), SIMPLIFICATION_STAGE_MAIN, {}, ConditionType::has_no_sideeffects)); //  "a > a", "0"
   vec.insert(MetaRule(_gr(_neg(_vara()), _neg(_varb())), _sm(_vara(), _varb()), SIMPLIFICATION_STAGE_MAIN));               // "--(a) > --(b)", "a < b"

   return vec;
}

std::string vfm::TermLogicGr::getK2Operator() const
{
   return "gt";
}
