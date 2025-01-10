//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file

#include "term_while_limited.h"
#include "meta_rule.h"
#include "parser.h"
#include <functional>
#include <vector>

using namespace vfm;

const OperatorStructure TermWhileLimited::my_struct(OutputFormatEnum::prefix, AssociativityTypeEnum::left, OPNUM_WHILELIM, PRIO_WHILELIM, SYMB_WHILELIM, false, false);

OperatorStructure TermWhileLimited::getOpStruct() const
{
   return my_struct;
}

float TermWhileLimited::eval(const std::shared_ptr<DataPack>& varVals, const std::shared_ptr<FormulaParser> parser, const bool suppress_sideeffects)
{
#if defined(ASMJIT_ENABLED)
   if (assembly_created_ == AssemblyState::yes) return (*fast_eval_func_)();
#endif

   std::shared_ptr<Term> condition_term = getOperands()[0];
   std::shared_ptr<Term> body_term = getOperands()[1];
   std::shared_ptr<Term> limit_term = getOperands()[2];

   // Note: Does this have to be undone after loop? Pretty sure not, but keep in mind if problems occur.
   // As a recursive loop may only point to the root of a compound structure, the metas of a while_limited
   // will always occur in the contex of a while_limited - meaning they always have to be protected on stack.
   auto prohibit_func = [](const std::shared_ptr<MathStruct> m)
   {
      if (m->isMetaCompound()) {
         m->toMetaCompoundIfApplicable()->prohibitUpdateStack();
      }
   };
   condition_term->applyToMeAndMyChildren(prohibit_func);
   body_term->applyToMeAndMyChildren(prohibit_func);

   float cond = condition_term->eval(varVals, parser, suppress_sideeffects);
   float result = 0;
   float max_iterations = limit_term->eval(varVals, parser, suppress_sideeffects); // Only once - that's the whole point of the limitation.
   int count = 0;

   while (cond && count < max_iterations)
   {
      result = body_term->eval(varVals, parser, suppress_sideeffects);
      cond = condition_term->eval(varVals, parser, suppress_sideeffects);
      count++;
   }

   return result;
}

#if defined(ASMJIT_ENABLED)
std::shared_ptr<x86::Xmm> TermWhileLimited::createSubAssembly(asmjit::x86::Compiler& cc, const std::shared_ptr<DataPack>& d, const std::shared_ptr<FormulaParser>& p)
{
   return nullptr;
}
#endif

TermWhileLimited::TermWhileLimited(const std::vector<std::shared_ptr<Term>>& opds) : TermArith(opds)
{
}

vfm::TermWhileLimited::TermWhileLimited(const std::shared_ptr<Term> opd1, const std::shared_ptr<Term> opd2, const std::shared_ptr<Term> opd3) : TermWhileLimited(std::vector<std::shared_ptr<Term>>{opd1, opd2, opd3})
{
}

std::shared_ptr<Term> TermWhileLimited::copy(const std::shared_ptr<std::map<std::shared_ptr<Term>, std::shared_ptr<Term>>> already_copied) {
   return copySimple<TermWhileLimited>(already_copied);
}

std::set<MetaRule> vfm::TermWhileLimited::getSimplificationRules(const std::shared_ptr<FormulaParser> parser) const
{
   std::set<MetaRule> vec = MathStruct::getSimplificationRules(parser);

   vec.insert(MetaRule(_whilelim(_vara(), _varb(), _varc()), _val0(), SIMPLIFICATION_STAGE_MAIN, { {_smeq(_varc(), _val(0)) , ConditionType::is_constant_and_evaluates_to_true } }, ConditionType::has_no_sideeffects)); // Limiting to zero or less loops yields 0.
   vec.insert(MetaRule(_whilelim(_vara(), _varb(), _varc()), _val0(), SIMPLIFICATION_STAGE_MAIN, { {_vara(), ConditionType::is_constant_and_evaluates_to_false}})); // Condition constantly false yields 0.
   
   return vec;
}
