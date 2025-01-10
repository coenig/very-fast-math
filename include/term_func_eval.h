//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file
#pragma once
#include "term.h"


namespace vfm {

/// \brief The "func" operator interprets the first argument a0 as a reference to some function f, and
/// the remaining (variable number of) arguments a1..an as arguments for f. The evaluation of this operator is:
///      func(a0, ..., an) := f(a1, ..., an).
///
/// Using TermFuncRef or TermFuncLambda, a reference to a desired function f can be created.
class TermFuncEval :
   public Term
{
public:
   const static OperatorStructure my_struct;

   OperatorStructure getOpStruct() const override;
   float eval(const std::shared_ptr<DataPack>& varVals, const std::shared_ptr<FormulaParser> parser = nullptr, const bool suppress_sideeffects = false) override;
#if defined(ASMJIT_ENABLED)
   std::shared_ptr<x86::Xmm> createSubAssembly(asmjit::x86::Compiler & cc, const std::shared_ptr<DataPack>& d, const std::shared_ptr<FormulaParser>& p = nullptr) override;
#endif
   std::shared_ptr<Term> copy(const std::shared_ptr<std::map<std::shared_ptr<Term>, std::shared_ptr<Term>>> already_copied = nullptr) override;
   TermFuncEval(const std::vector<std::shared_ptr<Term>>& t);
};

} // vfm