//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file

#include "term_plus.h"
#include "term_minus.h"
#include "term_neg.h"
#include "term_val.h"
#include "meta_rule.h"
#include "parser.h"
#include <functional>
#include <vector>

using namespace vfm;

const OperatorStructure TermPlus::my_struct(OutputFormatEnum::infix, AssociativityTypeEnum::left, OPNUM_PLUS, PRIO_PLUS, SYMB_PLUS, true, true);

OperatorStructure TermPlus::getOpStruct() const
{
   return my_struct;
}

std::shared_ptr<Term> TermPlus::getInvPattern(int argNum)
{
   return getInvPatternSimplePlusStyle<TermMinus>(argNum);
}

float TermPlus::eval(const std::shared_ptr<DataPack>& varVals, const std::shared_ptr<FormulaParser> parser, const bool suppress_sideeffects)
{
#if defined(ASMJIT_ENABLED)
   if (assembly_created_ == AssemblyState::yes) return (*fast_eval_func_)();
#endif

   return evalSimple(varVals, std::plus<float>(), 0, 0, parser, suppress_sideeffects);
}

std::set<MetaRule> vfm::TermPlus::getSimplificationRules(const std::shared_ptr<FormulaParser> parser) const
{
   std::set<MetaRule> vec = MathStruct::getSimplificationRules(parser);

   auto insert = [&vec](const MetaRule& rule) {
      rule.setStage(SIMPLIFICATION_STAGE_PREPARATION); // Stage 0 since we generate a normal form here which we need for simplification.
      vec.insert(rule);

      MetaRule r1 = rule.copy(true, true, true);
      r1.setStage(SIMPLIFICATION_STAGE_MAIN); // Stage 10 since we need a normal form in the end, as well.
      vec.insert(r1);
   };

   vec.insert(MetaRule(_plus(_vara(), _neg(_varb())), _minus(_vara(), _varb()), SIMPLIFICATION_STAGE_MAIN)); // "a + --(b)", "a - b"
   
   // TODO: Do we need these?
   vec.insert(MetaRule(_plus(_neg(_vara()), _varb()), _minus(_varb(), _vara()), SIMPLIFICATION_STAGE_MAIN, { { _vara(), ConditionType::is_not_constant }}, ConditionType::has_no_sideeffects)); // "--(a) + b", "b - a"
   //vec.insert(MetaRule("a + b", "b - --(a)", { {"a", ConditionType::is_constant_and_evaluates_to_negative} }, ConditionType::has_no_sideeffects));
   //vec.insert(MetaRule("a + a", "2 * a", ConditionType::has_no_sideeffects));

   // Collect constants of plus and minus.
   insert(MetaRule(_plus(_vara(), _minus(_varb(), _varc())), _plus(_minus(_vara(), _varc()), _varb()), -1, { {_varc() , ConditionType::is_constant} }, ConditionType::has_no_sideeffects)); // "a + (b - c)", "(a - c) + b"

   // Plus/Minus normal form.
   insert(MetaRule(_plus(_vara(), _minus(_varb(), _varc())), _minus(_plus(_vara(), _varb()), _varc()), -1, { {_plus(_vara(), _varc()) , ConditionType::is_not_constant} }, ConditionType::has_no_sideeffects)); // "a + (b - c)", "(a + b) - c"
   insert(MetaRule(_plus(_minus(_vara(), _varb()), _varc()), _minus(_plus(_vara(), _varc()), _varb()), -1, { {_plus(_vara(), _varc()) , ConditionType::is_not_constant} }, ConditionType::has_no_sideeffects)); // "(a - b) + c", "(a + c) - b"

   insert(MetaRule("a + b * c ==> a -  --(b) * c [[b: 'is_constant_and_evaluates_to_negative']]", -1, parser)); 

   return vec;
}

#if defined(ASMJIT_ENABLED)
std::shared_ptr<x86::Xmm> TermPlus::createSubAssembly(asmjit::x86::Compiler& cc, const std::shared_ptr<DataPack>& d, const std::shared_ptr<FormulaParser>& p)
{
   lambda_vno lambda = [](asmjit::x86::Compiler& cc, x86::Xmm& x, x86::Xmm& y) {cc.addss(x, y);};
   return subAssemblyVarnumOps(cc, d, lambda);
}
#endif

TermPlus::TermPlus(const std::vector<std::shared_ptr<Term>>& opds) : TermArith(opds)
{
}

vfm::TermPlus::TermPlus(const std::shared_ptr<Term> opd1, const std::shared_ptr<Term> opd2) : TermPlus(std::vector<std::shared_ptr<Term>>{opd1, opd2})
{
}

std::shared_ptr<Term> TermPlus::copy(const std::shared_ptr<std::map<std::shared_ptr<Term>, std::shared_ptr<Term>>> already_copied) 
{
   return copySimple<TermPlus>(already_copied);
}

bool vfm::TermPlus::isTermPlus() const
{
   return true;
}

std::string vfm::TermPlus::getK2Operator() const
{
   return "add";
}
