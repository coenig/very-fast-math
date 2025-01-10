//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file
#pragma once
#include "term_arith.h"


namespace vfm {

/// \brief The reverse square root function rsqrt(x) = 1 / sqrt(x). It is frequently used
/// in graphics programming, therefore, it has its own Term which uses the extremely fast
/// assembly operation RSQRTSS for calculation. However, note that the operation adopts a
/// rough approximation of the function. Using MathStruct::parseMathStruct("1 / sqrt(x)")
/// is slower, but more accurate.
class TermRSQRT :
   public TermArith
{
public:
   const static OperatorStructure my_struct;

   OperatorStructure getOpStruct() const override;
   float eval(const std::shared_ptr<DataPack>& varVals, const std::shared_ptr<FormulaParser> parser = nullptr, const bool suppress_sideeffects = false) override;
#if defined(ASMJIT_ENABLED)
   std::shared_ptr<x86::Xmm> createSubAssembly(asmjit::x86::Compiler & cc, const std::shared_ptr<DataPack>& d, const std::shared_ptr<FormulaParser>& p = nullptr) override;
#endif
   std::shared_ptr<Term> copy(const std::shared_ptr<std::map<std::shared_ptr<Term>, std::shared_ptr<Term>>> already_copied = nullptr) override;
   std::shared_ptr<Term> getInvPattern(int argNum) override;
   std::set<MetaRule> getSimplificationRules(const std::shared_ptr<FormulaParser> parser) const override;

   TermRSQRT(const std::vector<std::shared_ptr<Term>>& t);
   TermRSQRT(const std::shared_ptr<Term>& t);

private:
#if defined(ASMJIT_ENABLED)
   MathStruct::JITFunc func_;
   asmjit::JitRuntime jit_runtime_;
#endif
};

} // vfm