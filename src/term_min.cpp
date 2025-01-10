//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file

#include "term_min.h"
#include "meta_rule.h"
#include "parser.h"
#include <cmath>

using namespace vfm;

const OperatorStructure TermMin::my_struct(OutputFormatEnum::prefix, AssociativityTypeEnum::left, OPNUM_MIN, PRIO_MIN, SYMB_MIN, false, true);

TermMin::TermMin(const std::vector<std::shared_ptr<Term>>& t) : TermArith(t)
{
}

vfm::TermMin::TermMin(const std::shared_ptr<Term> opd1, const std::shared_ptr<Term> opd2) : TermMin(std::vector<std::shared_ptr<Term>>{opd1, opd2})
{
}

OperatorStructure TermMin::getOpStruct() const
{
   return my_struct;
}

float TermMin::eval(const std::shared_ptr<DataPack>& varVals, const std::shared_ptr<FormulaParser> parser, const bool suppress_sideeffects)
{
#if defined(ASMJIT_ENABLED)
   if (assembly_created_ == AssemblyState::yes) return (*fast_eval_func_)();
#endif

   //   return eval_simp(varVals, std::fmin(), this->opnds[0]->eval(varVals, parser, suppress_sideeffects), 1); // TODO (doesn't work yet)
   float first{ getOperands()[0]->eval(varVals, parser, suppress_sideeffects) };
   float second{ getOperands()[1]->eval(varVals, parser, suppress_sideeffects) };
   if (std::isnan(first) || std::isnan(second)) return second; // Emulate behavior of x86 assembly.
   return std::min(first, second);
}

#if defined(ASMJIT_ENABLED)
std::shared_ptr<x86::Xmm> TermMin::createSubAssembly(asmjit::x86::Compiler & cc, const std::shared_ptr<DataPack>& d, const std::shared_ptr<FormulaParser>& p)
{
   lambda_vno lambda = [](asmjit::x86::Compiler& cc, x86::Xmm& x, x86::Xmm& y) {
      cc.minss(x, y);
   };

   return subAssemblyVarnumOps(cc, d, lambda);
}
#endif

std::shared_ptr<Term> TermMin::copy(const std::shared_ptr<std::map<std::shared_ptr<Term>, std::shared_ptr<Term>>> already_copied) {
   return copySimple<TermMin>(already_copied);
}

std::set<MetaRule> vfm::TermMin::getSimplificationRules(const std::shared_ptr<FormulaParser> parser) const
{
   std::set<MetaRule> vec = MathStruct::getSimplificationRules(parser);

   vec.insert(MetaRule(_min(_vara(), _vara()), _vara(), SIMPLIFICATION_STAGE_MAIN, {}, ConditionType::has_no_sideeffects));
   vec.insert(MetaRule(_min(_neg(_vara()), _neg(_varb())), _neg(_max(_vara(), _varb())), SIMPLIFICATION_STAGE_MAIN, {}, ConditionType::has_no_sideeffects)); // "--(a) min --(b)", "--(a max b)" - TODO: Maybe works with sideeffects, too?
   
   return vec;
}

std::string vfm::TermMin::serializeK2(const std::map<std::string, std::string>& enum_values, std::pair<std::map<std::string, std::string>, std::string>& additional_functions, int& label_counter) const
{
   // (min a b) becomes (ite (le a b) a b)
   auto smeq_part = _smeq_plain(child0(), child1())->serializeK2(enum_values, additional_functions, label_counter);
   auto alt1 = child0()->serializeK2(enum_values, additional_functions, label_counter);
   auto alt2 = child1()->serializeK2(enum_values, additional_functions, label_counter);

   return "(ite " + smeq_part + " " + alt1 + " " + alt2 + ")";
}
