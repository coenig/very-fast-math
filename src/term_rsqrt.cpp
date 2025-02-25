//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file

#include "term_rsqrt.h"
#include "term_mult.h"
#include "meta_rule.h"

using namespace vfm;

const OperatorStructure TermRSQRT::my_struct(OutputFormatEnum::prefix, AssociativityTypeEnum::left, OPNUM_RSQRT, PRIO_RSQRT, SYMB_RSQRT, true, true);

TermRSQRT::TermRSQRT(const std::vector<std::shared_ptr<Term>>& t) : TermArith(t) {}
TermRSQRT::TermRSQRT(const std::shared_ptr<Term>& t) : TermRSQRT(std::vector<std::shared_ptr<Term>>{t}) {}

std::shared_ptr<Term> TermRSQRT::getInvPattern(int argNum)
{
   return MathStruct::parseMathStruct("1 / (p_(0) * p_(0))", false)->toTermIfApplicable();
}

OperatorStructure TermRSQRT::getOpStruct() const
{
   return my_struct;
}

float TermRSQRT::eval(const std::shared_ptr<DataPack>& varVals, const std::shared_ptr<FormulaParser> parser, const bool suppress_sideeffects)
{
#if defined(ASMJIT_ENABLED)
   if (assembly_created_ == AssemblyState::yes) return (*fast_eval_func_)();
#endif

#if defined(ASMJIT_ENABLED)
   // This is done to be sure to get the same result as in JIT version (since it IS the JIT version).
   // TODO: Do the compilation only once, as long as varVals doesn't change.
   asmjit::CodeHolder code;
   code.init(jit_runtime_.environment());

   asmjit::x86::Compiler cc(&code);
   cc.addFunc(asmjit::FuncSignatureT<float>());

   float subres = getOperands()[0]->eval(varVals, parser, suppress_sideeffects);
   x86::Xmm x = cc.newXmm();
   setXmmVar(cc, x, subres);
   cc.rsqrtss(x, x);

   cc.ret(x);

   cc.endFunc();
   cc.finalize();

   jit_runtime_.add(&func_, &code);

   return func_();
#else
   return 1 / sqrtf(getTermsJumpIntoCompounds()[0]->eval(varVals, parser, suppress_sideeffects)); // More accurate but may be different from JIT version.
#endif
}

#if defined(ASMJIT_ENABLED)
std::shared_ptr<x86::Xmm> TermRSQRT::createSubAssembly(asmjit::x86::Compiler & cc, const std::shared_ptr<DataPack>& d, const std::shared_ptr<FormulaParser>& p)
{
   lambda_so lambda = [](asmjit::x86::Compiler& cc, x86::Xmm& x) {
      cc.rsqrtss(x, x);
   };

   return subAssemblySingleOp(cc, d, lambda);
}
#endif

std::shared_ptr<Term> TermRSQRT::copy(const std::shared_ptr<std::map<std::shared_ptr<Term>, std::shared_ptr<Term>>> already_copied) {
   return copySimple<TermRSQRT>(already_copied);
}

std::set<MetaRule> TermRSQRT::getSimplificationRules(const std::shared_ptr<FormulaParser> parser) const
{
   std::set<MetaRule> vec = MathStruct::getSimplificationRules(parser);

   // Better don't do anything here, since it's a fast, rough approximation.

   return vec;
}
