//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file

#include "term_max.h"
#include "meta_rule.h"
#include "parser.h"
#include <cmath>

using namespace vfm;

const OperatorStructure TermMax::my_struct(OutputFormatEnum::prefix, AssociativityTypeEnum::left, OPNUM_MAX, PRIO_MAX, SYMB_MAX, false, true);

TermMax::TermMax(const std::vector<std::shared_ptr<Term>>& t) : TermArith(t)
{
}

vfm::TermMax::TermMax(const std::shared_ptr<Term> opd1, const std::shared_ptr<Term> opd2) : TermMax(std::vector<std::shared_ptr<Term>>{opd1, opd2})
{
}

OperatorStructure TermMax::getOpStruct() const
{
   return my_struct;
}

float TermMax::eval(const std::shared_ptr<DataPack>& varVals, const std::shared_ptr<FormulaParser> parser, const bool suppress_sideeffects)
{
#if defined(ASMJIT_ENABLED)
   if (assembly_created_ == AssemblyState::yes) return (*fast_eval_func_)();
#endif

   float first{ getOperands()[0]->eval(varVals, parser, suppress_sideeffects) };
   float second{ getOperands()[1]->eval(varVals, parser, suppress_sideeffects) };
   if (std::isnan(first) || std::isnan(second)) return second; // Emulate behavior of x86 assembly.
   return std::max(first, second);
}

#if defined(ASMJIT_ENABLED)
std::shared_ptr<x86::Xmm> TermMax::createSubAssembly(asmjit::x86::Compiler & cc, const std::shared_ptr<DataPack>& d, const std::shared_ptr<FormulaParser>& p)
{
   lambda_vno lambda = [](asmjit::x86::Compiler& cc, x86::Xmm& x, x86::Xmm& y) {
      cc.maxss(x, y);
   };

   return subAssemblyVarnumOps(cc, d, lambda);
}
#endif

std::string vfm::TermMax::serializeK2(const std::map<std::string, std::string>& enum_values, std::pair<std::map<std::string, std::string>, std::string>& additional_functions, int& label_counter) const
{
   // (max a b) becomes (ite (ge a b) a b)
   auto greq_part = _greq_plain(child0(), child1())->serializeK2(enum_values, additional_functions, label_counter);
   auto alt1 = child0()->serializeK2(enum_values, additional_functions, label_counter);
   auto alt2 = child1()->serializeK2(enum_values, additional_functions, label_counter);

   return "(ite " + greq_part + " " + alt1 + " " + alt2 + ")";
}

std::shared_ptr<Term> TermMax::copy(const std::shared_ptr<std::map<std::shared_ptr<Term>, std::shared_ptr<Term>>> already_copied) {
   return copySimple<TermMax>(already_copied);
}

std::set<MetaRule> vfm::TermMax::getSimplificationRules(const std::shared_ptr<FormulaParser> parser) const
{
   std::set<MetaRule> vec = MathStruct::getSimplificationRules(parser);

   vec.insert(MetaRule(_max(_vara(), _vara()), _vara(), SIMPLIFICATION_STAGE_MAIN, {}, ConditionType::has_no_sideeffects));                                  // "a max a", "a"
   vec.insert(MetaRule(_max(_neg(_vara()), _neg(_varb())), _neg(_min(_vara(), _varb())), SIMPLIFICATION_STAGE_MAIN, {}, ConditionType::has_no_sideeffects)); // "--(a) max --(b)", "--(a min b)" TODO: Maybe works with sideeffects, too?

   return vec;
}