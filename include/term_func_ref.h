//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file
#pragma once
#include "term.h"


namespace vfm {

/// \brief The "@f(fct)" operator creates a reference to function fct. For example:
///      @f(+) ==> Reference to plus operator.
///      @f(-) ==> Reference to minus operator.
///      @f(mod) ==> Refernece to modulo operator.
///
/// Combined with TermFuncEval ("func"), function references can be used to pass a function as argument
/// to another function. For example:
///      doMath := func(p_(2), p_(0), p_(1)); // Define some not yet specified operation for two arguments p_(0), p_(1).
///
/// Now it can be called with differend base operations p_(2):
///      doMath(3, 4, @f(+)) ==> 7
///      doMath(3, 4, @f(-)) ==> -1
///      doMath(3, 4, @f(*)) ==> 12
///      doMath(3, 4, @f(/)) ==> 0.75
///
/// See also TermFuncLambda for ad-hoc definitions of functions and their references.
class TermFuncRef :
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

   TermFuncRef(const std::vector<std::shared_ptr<Term>>& t);
   TermFuncRef(const std::shared_ptr<Term> t);
   TermFuncRef(const std::shared_ptr<Term> t1, const std::shared_ptr<Term> t2);
};

} // vfm