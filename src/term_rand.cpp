//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file

#include "term_rand.h"

using namespace vfm;

const OperatorStructure TermRand::my_struct(OutputFormatEnum::prefix, AssociativityTypeEnum::left, OPNUM_RAND, PRIO_RAND, SYMB_RAND, false, false);

TermRand::TermRand(const std::vector<std::shared_ptr<Term>>& t) : Term(t) {}
TermRand::TermRand(const std::shared_ptr<Term>& t) : TermRand(std::vector<std::shared_ptr<Term>>{t}) {}

OperatorStructure TermRand::getOpStruct() const
{
   return my_struct;
}

// https://stackoverflow.com/questions/5008804/generating-random-integer-from-a-range
static std::random_device rd; // Only used once to initialise (seed) engine.
static std::mt19937 rng(0); // Random-number engine used (Mersenne-Twister in this case, SET TO 0 SEED, not rd).

float TermRand::eval(const std::shared_ptr<DataPack>& varVals, const std::shared_ptr<FormulaParser> parser, const bool suppress_sideeffects)
{
#if defined(ASMJIT_ENABLED)
   if (assembly_created_ == AssemblyState::yes) return (*fast_eval_func_)();
#endif

   std::uniform_int_distribution<int> uni(0, static_cast<int>(getOperands()[0]->eval(varVals, parser, suppress_sideeffects)));
   return static_cast<int>(uni(rng));
}

#if defined(ASMJIT_ENABLED)
std::shared_ptr<x86::Xmm> TermRand::createSubAssembly(asmjit::x86::Compiler& cc, const std::shared_ptr<DataPack>& d, const std::shared_ptr<FormulaParser>& p)
{
   float* rv = rand_vars.data();
   lambda_so lambda = [rv](asmjit::x86::Compiler& cc, x86::Xmm& x) {
      x86::Xmm val = cc.newXmm();
      setXmmVarToAddressLocation(cc, val, rv, rand_var_count * sizeof(float));

      // From here on "val mod x".
      x86::Xmm y = cc.newXmm();
      x86::Xmm xx = cc.newXmm();
      cc.movss(y, x);
      cc.movss(xx, val);
      cc.divss(xx, y);
      cc.roundss(xx, xx, 3);
      cc.mulss(xx, y);
      cc.subss(val, xx);
      cc.movss(x, val);
      ++rand_var_count;
   };

   return subAssemblySingleOp(cc, d, lambda);
}
#endif

std::shared_ptr<Term> TermRand::copy(const std::shared_ptr<std::map<std::shared_ptr<Term>, std::shared_ptr<Term>>> already_copied)
{
   std::shared_ptr<Term> ptr(new TermRand(getOperands()[0]->copy(already_copied)));
   ptr->setChildrensFathers(false, false);
   return ptr;
}
