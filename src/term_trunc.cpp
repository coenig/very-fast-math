//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file

#include "term_trunc.h"
#include "meta_rule.h"
#include "parser.h"
#include <cmath>

using namespace vfm;

const OperatorStructure TermTrunc::my_struct(OutputFormatEnum::prefix, AssociativityTypeEnum::left, OPNUM_TRUNC, PRIO_TRUNC, SYMB_TRUNC, true, true);

TermTrunc::TermTrunc(const std::vector<std::shared_ptr<Term>>& t) : TermArith(t) {}
TermTrunc::TermTrunc(const std::shared_ptr<Term>& t) : TermTrunc(std::vector<std::shared_ptr<Term>>{t}) {}

OperatorStructure TermTrunc::getOpStruct() const
{
   return my_struct;
}

float TermTrunc::eval(const std::shared_ptr<DataPack>& varVals, const std::shared_ptr<FormulaParser> parser, const bool suppress_sideeffects)
{
#if defined(ASMJIT_ENABLED)
   if (assembly_created_ == AssemblyState::yes) return (*fast_eval_func_)();
#endif

   return trunc(getOperands()[0]->eval(varVals, parser, suppress_sideeffects));
}

#if defined(ASMJIT_ENABLED)
std::shared_ptr<x86::Xmm> TermTrunc::createSubAssembly(asmjit::x86::Compiler& cc, const std::shared_ptr<DataPack>& d, const std::shared_ptr<FormulaParser>& p)
{
   lambda_so lambda = [](asmjit::x86::Compiler& cc, x86::Xmm& x) {
      cc.roundss(x, x, 3); // 3 => round toward 0 (truncate). http://rayseyfarth.com/asm/pdf/ch11-floating-point.pdf
   };

   return subAssemblySingleOp(cc, d, lambda);
}
#endif

std::shared_ptr<Term> TermTrunc::copy(const std::shared_ptr<std::map<std::shared_ptr<Term>, std::shared_ptr<Term>>> already_copied) {
   std::shared_ptr<Term> ptr(new TermTrunc(getOperands()[0]->copy(already_copied)));
   ptr->setChildrensFathers(false, false);
   return ptr;
}

std::set<MetaRule> vfm::TermTrunc::getSimplificationRules(const std::shared_ptr<FormulaParser> parser) const
{
   std::set<MetaRule> vec = MathStruct::getSimplificationRules(parser);

   vec.insert(MetaRule(_trunc(_trunc(_vara())), _trunc(_vara()), SIMPLIFICATION_STAGE_MAIN));                 // "trunc(trunc(a))", "trunc(a)"
   vec.insert(MetaRule(_trunc(_or(_vara(), _varb())), _or(_vara(), _varb()), SIMPLIFICATION_STAGE_MAIN));     // "trunc(a : b)", "a : b"
   vec.insert(MetaRule(_trunc(_and(_vara(), _varb())), _and(_vara(), _varb()), SIMPLIFICATION_STAGE_MAIN));   // "trunc(a & b)", "a & b"
   vec.insert(MetaRule(_trunc(_gr(_vara(), _varb())), _gr(_vara(), _varb()), SIMPLIFICATION_STAGE_MAIN));     // "trunc(a > b)", "a > b"
   vec.insert(MetaRule(_trunc(_sm(_vara(), _varb())), _sm(_vara(), _varb()), SIMPLIFICATION_STAGE_MAIN));     // "trunc(a < b)", "a < b"
   vec.insert(MetaRule(_trunc(_greq(_vara(), _varb())), _greq(_vara(), _varb()), SIMPLIFICATION_STAGE_MAIN)); // "trunc(a >= b)", "a >= b"
   vec.insert(MetaRule(_trunc(_smeq(_vara(), _varb())), _smeq(_vara(), _varb()), SIMPLIFICATION_STAGE_MAIN)); // "trunc(a <= b)", "a <= b"
   vec.insert(MetaRule(_trunc(_eq(_vara(), _varb())), _eq(_vara(), _varb()), SIMPLIFICATION_STAGE_MAIN));     // "trunc(a == b)", "a == b"
   vec.insert(MetaRule(_trunc(_neq(_vara(), _varb())), _neq(_vara(), _varb()), SIMPLIFICATION_STAGE_MAIN));   // "trunc(a != b)", "a != b"
   vec.insert(MetaRule(_trunc(_not(_vara())), _not(_vara()), SIMPLIFICATION_STAGE_MAIN));                     // "trunc(not(a))", "not(a)"
   vec.insert(MetaRule(_trunc(_boolify(_vara())), _boolify(_vara()), SIMPLIFICATION_STAGE_MAIN));             // "trunc(bool(a))", "bool(a)"

   return vec;
}

std::string TermTrunc::getNuSMVOperator() const
{
   return "";
}
