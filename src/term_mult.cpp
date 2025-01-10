//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file

#include "term_mult.h"
#include "term_div.h"
#include "term_meta_compound.h"
#include "meta_rule.h"
#include "parser.h"
#include <vector>

using namespace vfm;

const OperatorStructure TermMult::my_struct(OutputFormatEnum::infix, AssociativityTypeEnum::left, OPNUM_MULT, PRIO_MULT, SYMB_MULT, true, true);

OperatorStructure TermMult::getOpStruct() const
{
   return my_struct;
}

std::shared_ptr<Term> TermMult::getInvPattern(int argNum)
{
   return getInvPatternSimplePlusStyle<TermDiv>(argNum);
}

float TermMult::eval(const std::shared_ptr<DataPack>& varVals, const std::shared_ptr<FormulaParser> parser, const bool suppress_sideeffects)
{
#if defined(ASMJIT_ENABLED)
   if (assembly_created_ == AssemblyState::yes) return (*fast_eval_func_)();
#endif

   return evalSimple(varVals, std::multiplies<float>(), 1, 0, parser, suppress_sideeffects);
}

#if defined(ASMJIT_ENABLED)
std::shared_ptr<x86::Xmm> TermMult::createSubAssembly(asmjit::x86::Compiler& cc, const std::shared_ptr<DataPack>& d, const std::shared_ptr<FormulaParser>& p)
{
   lambda_vno lambda = [](asmjit::x86::Compiler& cc, x86::Xmm& x, x86::Xmm& y) {cc.mulss(x, y); };
   return subAssemblyVarnumOps(cc, d, lambda);
}
#endif

TermMult::TermMult(const std::vector<std::shared_ptr<Term>>& opds) : TermArith(opds)
{
}

vfm::TermMult::TermMult(const std::shared_ptr<Term> opd1, const std::shared_ptr<Term> opd2) : TermMult(std::vector<std::shared_ptr<Term>>{opd1, opd2})
{
}

std::shared_ptr<Term> TermMult::copy(const std::shared_ptr<std::map<std::shared_ptr<Term>, std::shared_ptr<Term>>> already_copied) {
   return copySimple<TermMult>(already_copied);
}

std::set<MetaRule> vfm::TermMult::getSimplificationRules(const std::shared_ptr<FormulaParser> parser) const
{
   std::set<MetaRule> vec = MathStruct::getSimplificationRules(parser);

   // TODO: The below two rules are not bad, per se, when combined with going right-associative in the preparation stage in TermAnyway.
   // However, they srew up other intended simplifications, particularly the one shown as example in the readme. We need to balance these in future.
   //// "a * (b + c)", "a * b + a * c"
   //vec.insert(MetaRule(_mult(_vara(), _plus(_varb(), _varc())), _plus(_mult(_vara(), _varb()), _mult(_vara(), _varc())), SIMPLIFICATION_STAGE_PREPARATION, {}, ConditionType::has_no_sideeffects));

   //// "a * (b - c)", "a * b - a * c"
   //vec.insert(MetaRule(_mult(_vara(), _minus(_varb(), _varc())), _minus(_mult(_vara(), _varb()), _mult(_vara(), _varc())), SIMPLIFICATION_STAGE_PREPARATION, {}, ConditionType::has_no_sideeffects));

   // TODO: Below rule leads to infinite loop with "3*(3+x)".
   //vec.insert(MetaRule(_mult(_vara(), _plus(_varb(), _varc())), _plus(_mult(_vara(), _varb()), _mult(_vara(), _varc())), SIMPLIFICATION_STAGE_MAIN, { { _vara(), ConditionType::is_constant }, { _varb(), ConditionType::is_constant } }, ConditionType::has_no_sideeffects)); // a * (b + c) ==> a * b + a * c, if a and b are constant

   vec.insert(MetaRule(_mult(_vara(), _div(_val1(), _vara())), _val1(), SIMPLIFICATION_STAGE_MAIN, { {_vara(), ConditionType::is_constant_and_evaluates_to_true} }, ConditionType::has_no_sideeffects)); // "a * (1/a)", "1"
   vec.insert(MetaRule(_mult(_div(_val1(), _vara()), _vara()), _val1(), SIMPLIFICATION_STAGE_MAIN, { {_vara(), ConditionType::is_constant_and_evaluates_to_true} }, ConditionType::has_no_sideeffects)); // "(1/a) * a", "1"
   vec.insert(MetaRule(_mult(_vara(), _neg(_varb())), _neg(_mult(_vara(), _varb())), SIMPLIFICATION_STAGE_MAIN));                                                                                        // "a * --(b)", "--(a * b)"

   // Yields infinite loop together with "(a - d * b) - e * c" ==> "a + e * (--(d / e * b) + --c)" [ e != d && d / e is integer. ]
   //vec.insert(MetaRule(_mult(_vara(), _plus(_varb(), _varc())), _plus(_mult(_vara(), _varb()), _mult(_vara(), _varc())), { { _vara(), ConditionType::is_constant }, { _varb(), ConditionType::is_constant } }, ConditionType::has_no_sideeffects)); // a * (b + c) ==> a * b + a * c, if a and b are constant

   //vec.insert(MetaRule(_mult(_vara(), _varb()), _neg(_mult(_neg(_vara()), _varb())), { { _vara(), ConditionType::is_constant_and_evaluates_to_negative } }));                 // "a * --(b)", "--(a * b)"

   // TODO: Do we need these? They might lead to infinite loops(?) Check and fix!
   //vec.insert(MetaRule("--(a) * b", "--(a * b)"));
   //vec.insert(MetaRule("a * b", "--(--(a) * b)", { {_vara(), ConditionType::is_constant_and_evaluates_to_negative} }));
   vec.insert(MetaRule(_mult(_vara(), _varb()), _neg(_mult(_vara(), _neg(_varb()))), SIMPLIFICATION_STAGE_MAIN, { {_varb(), ConditionType::is_constant_and_evaluates_to_negative} })); // "a * b", "--(a * --(b))"

   return vec;
}

std::string vfm::TermMult::getK2Operator() const
{
   return "mul";
}
