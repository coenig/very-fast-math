//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file

#include "term_plus.h"
#include "term_minus.h"
#include "term_neg.h"
#include "meta_rule.h"
#include "parser.h"
#include "vector"

using namespace vfm;

const OperatorStructure TermMinus::my_struct(OutputFormatEnum::infix, AssociativityTypeEnum::left, OPNUM_MINUS, PRIO_MINUS, SYMB_MINUS, false, false);

OperatorStructure TermMinus::getOpStruct() const
{
   return my_struct;
}

std::shared_ptr<Term> TermMinus::getInvPattern(int argNum)
{
   return getInvPatternSimpleMinusStyle<TermPlus, TermMinus>(argNum);
}

float TermMinus::eval(const std::shared_ptr<DataPack>& varVals, const std::shared_ptr<FormulaParser> parser, const bool suppress_sideeffects)
{
#if defined(ASMJIT_ENABLED)
   if (assembly_created_ == AssemblyState::yes) return (*fast_eval_func_)();
#endif

   return evalSimple(varVals, std::minus<float>(), getOperands()[0]->eval(varVals, parser, suppress_sideeffects), 1, parser, suppress_sideeffects);
}

std::set<MetaRule> vfm::TermMinus::getSimplificationRules(const std::shared_ptr<FormulaParser> parser) const
{
   std::set<MetaRule> vec = MathStruct::getSimplificationRules(parser);

   auto insert = [&vec](const MetaRule& rule) {
      rule.setStage(SIMPLIFICATION_STAGE_PREPARATION); // Stage 0 since we generate a normal form here which we need for simplification.
      vec.insert(rule);

      MetaRule r1 = rule.copy(true, true, true);
      r1.setStage(SIMPLIFICATION_STAGE_MAIN); // Stage 10 since we need a normal form in the end, as well.
      vec.insert(r1);
   };

   vec.insert(MetaRule(_minus(_vara(), _varb()), _plus(_vara(), _neg(_varb())), SIMPLIFICATION_STAGE_MAIN, { {_varb(), ConditionType::is_constant_and_evaluates_to_negative} })); // "a - b", "a + --(b)"
   vec.insert(MetaRule(_minus(_vara(), _neg(_varb())), _plus(_vara(), _varb()), SIMPLIFICATION_STAGE_MAIN));                                                                      // "a - --(b)", "a + b"
   vec.insert(MetaRule(_minus(_val0(), _vara()), _neg(_vara()), 10));                                                                                      // "0 - a", "--(a)"
   // vec.insert(MetaRule("a - a", "0", ConditionType::has_no_sideeffects)); // TODO: Delete, should come with general rule in MathStruct.                 // "a - a", "0"
   vec.insert(MetaRule(_minus(_neg(_vara()), _varb()), _neg(_plus(_vara(), _varb())), SIMPLIFICATION_STAGE_MAIN));                                                                // "--(a) - b", "--(a + b)"
   vec.insert(MetaRule(_minus(_vara(), _varb()), _plus(_neg(_varb()), _vara()), SIMPLIFICATION_STAGE_MAIN, { {_varb(), ConditionType::is_constant}, {_vara(), ConditionType::is_not_constant} })); // "a - b", "--(b) + a"
   
   // Normal form for Minus: Left-associative: ( ... (((. - .) - .) - .) ... )
   insert(MetaRule(_minus(_vara(), _minus(_varb(), _varc())), _plus(_minus(_vara(), _varb()), _varc()), -1, {}, ConditionType::has_no_sideeffects)); // "a - (b - c)", "(a - b) + c"
   insert(MetaRule(_minus(_vara(), _plus(_varb(), _varc())), _minus(_minus(_vara(), _varb()), _varc()), -1, {}, ConditionType::has_no_sideeffects)); // "a - (b + c)", "(a - b) - c"

   // ...and send constants downstream: ((((a - c) - ... for constant c
   insert(MetaRule(_minus(_minus(_vara(), _varc()), _varb()), _minus(_minus(_vara(), _varb()), _varc()), -1, { {_varb(), ConditionType::is_constant}, {_varc(), ConditionType::is_not_constant} }, ConditionType::has_no_sideeffects)); // "(a - c) - b", "(a - b) - c"
   insert(MetaRule(_minus(_minus(_vara(), _varc()), _varb()), _minus(_vara(), _plus(_varb(), _varc())), -1, { {_varc(), ConditionType::is_constant}, {_varb(), ConditionType::is_constant} }, ConditionType::has_no_sideeffects));      // "(a - c) - b", "a - (b + c)"
   insert(MetaRule(_minus(_plus(_vara(), _varb()), _varc()), _plus(_minus(_vara(), _varc()), _varb()), -1, { {_vara(), ConditionType::is_constant}, {_varc(), ConditionType::is_constant} }, ConditionType::has_no_sideeffects));       // "(a + b) - c", "(a - c) + b"
   insert(MetaRule(_minus(_minus(_vara(), _varb()), _varc()), _minus(_minus(_vara(), _varc()), _varb()), -1, { {_vara(), ConditionType::is_constant}, {_varc(), ConditionType::is_constant} }, ConditionType::has_no_sideeffects));     // "(a - b) - c", "(a - c) - b"

   // Collect constants of combined plus and minus.
   insert(MetaRule(_minus(_plus(_vara(), _varb()), _varc()), _plus(_minus(_vara(), _varc()), _varb()), -1, { {_minus(_vara(), _varc()) , ConditionType::is_constant} }, ConditionType::has_no_sideeffects)); // "(a + b) - c", "(a - c) + b"

   return vec;
}

#if defined(ASMJIT_ENABLED)
std::shared_ptr<x86::Xmm> TermMinus::createSubAssembly(asmjit::x86::Compiler& cc, const std::shared_ptr<DataPack>& d, const std::shared_ptr<FormulaParser>& p)
{
   lambda_vno lambda = [](asmjit::x86::Compiler& cc, x86::Xmm& x, x86::Xmm& y) {cc.subss(x, y);};
   return subAssemblyVarnumOps(cc, d, lambda);
}
#endif

TermMinus::TermMinus(const std::vector<std::shared_ptr<Term>>& opds) : TermArith(opds)
{
}

TermMinus::TermMinus(const std::shared_ptr<Term> opd1, const std::shared_ptr<Term> opd2) : TermMinus(std::vector<std::shared_ptr<Term>>{opd1, opd2})
{
}

std::shared_ptr<Term> TermMinus::copy(const std::shared_ptr<std::map<std::shared_ptr<Term>, std::shared_ptr<Term>>> already_copied) {
   return copySimple<TermMinus>(already_copied);
}

std::string vfm::TermMinus::getK2Operator() const
{
   return "sub";
}
