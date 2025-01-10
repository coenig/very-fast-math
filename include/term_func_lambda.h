//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2022 Robert Bosch GmbH. All rights reserved.
//============================================================================================================
/// @file
#pragma once
#include "term.h"


namespace vfm {

/// \brief The "lambda" operator creates a reference to a function which is ad-hoc 
/// defined via the operation itself. For example:
///      lambda(p_(0) * p_(1)) 
///         ==> Reference to a NEW function which multiplies its two arguments: x * y.
///
///      lambda(set(@_j, 1) >> while((set(@_i, _i + 1) <= p_(1)), set(@_j, _j * p_(0)))) 
///         ==> Reference to a NEW function which implements a power operator: x ** y.
///
/// Combined with TermFuncEval ("func"), function references can be used to pass a function as argument
/// to another function. For example:
///      doMath := func(p_(2), p_(0), p_(1)); // Define some not yet specified operation for two arguments p_(0), p_(1).
///
/// Now it can be called with differend base operations p_(2):
///      doMath(3, 4, lambda(p_(0) * p_(1))) ==> 12
///      doMath(3, 4, lambda(set(@_j, 1) >> while((set(@_i, _i + 1) <= p_(1)), set(@_j, _j * p_(0))))) ==> 81
///
/// See also TermFuncRef for references to explicitly defined functions.
class TermFuncLambda :
   public Term
{
public:
   const static OperatorStructure my_struct;

   OperatorStructure getOpStruct() const override;
   float eval(const std::shared_ptr<DataPack>& varVals, const std::shared_ptr<FormulaParser> parser = nullptr, const bool suppress_sideeffects = false) override;
#if defined(ASMJIT_ENABLED)
   std::shared_ptr<x86::Xmm> createSubAssembly(asmjit::x86::Compiler & cc, const std::shared_ptr<DataPack>& d, const std::shared_ptr<FormulaParser>& p = nullptr) override;
#endif
   bool isTermLambda() const override;

   std::shared_ptr<Term> copy(const std::shared_ptr<std::map<std::shared_ptr<Term>, std::shared_ptr<Term>>> already_copied = nullptr) override;
   TermFuncLambda(const std::vector<std::shared_ptr<Term>>& t);
   TermFuncLambda(const std::shared_ptr<Term> t);
};

} // vfm