//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file

#include "term_mod.h"
#include "meta_rule.h"
#include "parser.h"
#include <cmath>

using namespace vfm;

const OperatorStructure TermMod::my_struct(OutputFormatEnum::infix, AssociativityTypeEnum::left, OPNUM_MOD, PRIO_MOD, SYMB_MOD, false, false); // Really left?

TermMod::TermMod(const std::vector<std::shared_ptr<Term>>& t) : TermArith(t)
{
}

vfm::TermMod::TermMod(const std::shared_ptr<Term> opd1, const std::shared_ptr<Term> opd2) : TermMod(std::vector<std::shared_ptr<Term>>{opd1, opd2})
{
}

OperatorStructure TermMod::getOpStruct() const
{
   return my_struct;
}

float TermMod::eval(const std::shared_ptr<DataPack>& varVals, const std::shared_ptr<FormulaParser> parser, const bool suppress_sideeffects)
{
#if defined(ASMJIT_ENABLED)
   if (assembly_created_ == AssemblyState::yes) return (*fast_eval_func_)();
#endif

   float second = getOperands()[1]->eval(varVals, parser, suppress_sideeffects);
   float first = getOperands()[0]->eval(varVals, parser, suppress_sideeffects);
//   if (second != (int) second) return 0.0; // Adjust to x86 assembly where mod towards a fractional float yields zero.
   return std::fmod(first, second);
}

#if defined(ASMJIT_ENABLED)
// From: https://stackoverflow.com/questions/9505513/floating-point-modulo-operation
// mod = value - Trunc(value / modulus) * modulus;
std::shared_ptr<x86::Xmm> TermMod::createSubAssembly(asmjit::x86::Compiler& cc, const std::shared_ptr<DataPack>& d, const std::shared_ptr<FormulaParser>& p)
{
   lambda_vno lambda = [](asmjit::x86::Compiler& cc, x86::Xmm& x, x86::Xmm& y) {
      x86::Xmm xx = cc.newXmm();
      cc.movss(xx, x);
      cc.divss(xx, y);
      cc.roundss(xx, xx, 3);
      cc.mulss(xx, y);
      cc.subss(x, xx);
   };

   return subAssemblyVarnumOps(cc, d, lambda);
}
#endif

std::shared_ptr<Term> TermMod::copy(const std::shared_ptr<std::map<std::shared_ptr<Term>, std::shared_ptr<Term>>> already_copied) {
   return copySimple<TermMod>(already_copied);
}


std::set<MetaRule> vfm::TermMod::getSimplificationRules(const std::shared_ptr<FormulaParser> parser) const
{
   std::set<MetaRule> vec = MathStruct::getSimplificationRules(parser);

   //vec.push_back(MetaRule("a mod 1", "0")); // This rule doesn't work for non-integer a.
   vec.insert(MetaRule(_mod(_mult(_mod(_vara(), _varc()), _mod(_varb(), _varc())), _varc()), _mod(_mult(_vara(), _varb()), _varc()), SIMPLIFICATION_STAGE_MAIN, {}, ConditionType::has_no_sideeffects)); // "((a mod c) * (b mod c)) mod c", "(a*b) mod c"

   return vec;
}

std::string TermMod::getNuSMVOperator() const
{
   return "mod";
}

std::string vfm::TermMod::getK2Operator() const
{
   return "rem";
}
